// LibrePods - AirPods liberated from Apple's ecosystem
// Copyright (C) 2025 LibrePods contributors
// SPDX-License-Identifier: AGPL-3.0-only

//! L2CAP transport implementation using raw AF_BLUETOOTH sockets via `nix`.
//!
//! References:
//! - `main.cpp` `connectToDevice()` — Qt L2CAP socket

use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::sync::atomic::{AtomicBool, Ordering};

use librepods_core::protocol::packets;
use librepods_core::transport::{L2capTransport, TransportError};
use nix::libc;

// ---------------------------------------------------------------------------
// Bluetooth socket address (Linux-specific)
// ---------------------------------------------------------------------------

/// `sockaddr_l2` for Bluetooth L2CAP connections.
#[repr(C)]
struct SockAddrL2 {
    l2_family: u16,
    l2_psm: u16,
    l2_bdaddr: [u8; 6],
    l2_cid: u16,
    l2_bdaddr_type: u8,
}

const AF_BLUETOOTH: i32 = 31;
const BTPROTO_L2CAP: i32 = 0;
const SOL_L2CAP: i32 = 6;
const L2CAP_OPTIONS: i32 = 1;
const SOL_BLUETOOTH: i32 = 274;
const BT_SECURITY: i32 = 4;
const BT_SECURITY_MEDIUM: u8 = 2;

/// Linux `struct l2cap_options` from <bluetooth/l2cap.h>.
#[repr(C)]
struct L2capOptions {
    omtu: u16,
    imtu: u16,
    flush_to: u16,
    mode: u8,
    fcs: u8,
    max_tx: u8,
    txwin_size: u16,
}

fn l2cap_options(fd: &OwnedFd) -> Result<L2capOptions, TransportError> {
    // Safety: all fields are integers, and the buffer matches the Linux ABI.
    let mut options: L2capOptions = unsafe { std::mem::zeroed() };
    let mut len = std::mem::size_of::<L2capOptions>() as libc::socklen_t;
    let ret = unsafe {
        libc::getsockopt(
            fd.as_raw_fd(),
            SOL_L2CAP,
            L2CAP_OPTIONS,
            &mut options as *mut L2capOptions as *mut libc::c_void,
            &mut len,
        )
    };
    if ret < 0 {
        return Err(TransportError::with_source(
            "failed to read L2CAP options",
            std::io::Error::last_os_error(),
        ));
    }
    Ok(options)
}

/// Parse a Bluetooth MAC address string (e.g. "AA:BB:CC:DD:EE:FF") into 6 bytes.
/// BlueZ stores addresses in little-endian order.
fn parse_bdaddr(addr: &str) -> Result<[u8; 6], TransportError> {
    let parts: Vec<&str> = addr.split(':').collect();
    if parts.len() != 6 {
        return Err(TransportError::new(format!(
            "invalid Bluetooth address: {addr}"
        )));
    }
    let mut bytes = [0u8; 6];
    for (i, part) in parts.iter().enumerate() {
        bytes[5 - i] = u8::from_str_radix(part, 16)
            .map_err(|e| TransportError::with_source(format!("bad hex in address: {part}"), e))?;
    }
    Ok(bytes)
}

// ---------------------------------------------------------------------------
// L2CAP socket implementation
// ---------------------------------------------------------------------------

/// Linux L2CAP socket transport using raw `AF_BLUETOOTH` sockets.
pub struct L2capLinux {
    fd: OwnedFd,
    connected: AtomicBool,
}

impl L2capLinux {
    /// Connect to an AirPods device at `address` (e.g. "AA:BB:CC:DD:EE:FF")
    /// on PSM [`packets::AAP_PSM`] (0x1001).
    pub fn connect(address: &str) -> Result<Self, TransportError> {
        Self::connect_psm(address, packets::AAP_PSM)
    }

    /// Connect to a device on a specific PSM.
    pub fn connect_psm(address: &str, psm: u16) -> Result<Self, TransportError> {
        let bdaddr = parse_bdaddr(address)?;

        // Create Bluetooth L2CAP socket using raw libc.
        // We bypass nix::sys::socket::socket() because nix's
        // AddressFamily::from_i32() does not include AF_BLUETOOTH,
        // causing it to silently fall back to AF_UNSPEC and fail.
        let raw_fd = unsafe {
            libc::socket(
                AF_BLUETOOTH,
                libc::SOCK_SEQPACKET | libc::SOCK_CLOEXEC,
                BTPROTO_L2CAP,
            )
        };
        if raw_fd < 0 {
            let err = std::io::Error::last_os_error();
            let hint = match err.raw_os_error() {
                Some(libc::EPERM) | Some(libc::EACCES) => {
                    "\nHint: Bluetooth sockets require elevated privileges. Try one of:\n  \
                     • sudo setcap cap_net_raw,cap_net_admin+eip ./target/release/librepods-cli\n  \
                     • Run with sudo"
                }
                Some(libc::EAFNOSUPPORT) => {
                    "\nHint: AF_BLUETOOTH not supported. Is the bluetooth kernel module loaded?\n  \
                     • sudo modprobe bluetooth"
                }
                _ => "",
            };
            return Err(TransportError::with_source(
                format!("failed to create L2CAP socket (AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP){hint}"),
                err,
            ));
        }

        // Safety: we just created this fd and checked it's valid (>= 0)
        let fd = unsafe { OwnedFd::from_raw_fd(raw_fd) };

        // Motion streaming requires an encrypted link, as in MousePods'
        // l2cap.py. Linux struct bt_security is { u8 level; u8 key_size; }.
        let security = [BT_SECURITY_MEDIUM, 0];
        let ret = unsafe {
            libc::setsockopt(
                fd.as_raw_fd(),
                SOL_BLUETOOTH,
                BT_SECURITY,
                security.as_ptr() as *const libc::c_void,
                security.len() as libc::socklen_t,
            )
        };
        if ret < 0 {
            return Err(TransportError::with_source(
                "failed to require encrypted L2CAP link for motion streaming",
                std::io::Error::last_os_error(),
            ));
        }

        // Configure before connect so the receive MTU is advertised during
        // L2CAP negotiation. Preserve the kernel's other socket options.
        let mut options = l2cap_options(&fd)?;
        options.imtu = packets::AAP_MTU;
        options.omtu = packets::AAP_MTU;
        let ret = unsafe {
            libc::setsockopt(
                fd.as_raw_fd(),
                SOL_L2CAP,
                L2CAP_OPTIONS,
                &options as *const L2capOptions as *const libc::c_void,
                std::mem::size_of::<L2capOptions>() as libc::socklen_t,
            )
        };
        if ret < 0 {
            return Err(TransportError::with_source(
                format!("failed to set L2CAP MTU to {} bytes", packets::AAP_MTU),
                std::io::Error::last_os_error(),
            ));
        }

        // Build sockaddr_l2
        let addr = SockAddrL2 {
            l2_family: AF_BLUETOOTH as u16,
            l2_psm: psm.to_le(),
            l2_bdaddr: bdaddr,
            l2_cid: 0,
            l2_bdaddr_type: 0, // BDADDR_BREDR
        };

        // Connect
        let addr_ptr = &addr as *const SockAddrL2 as *const libc::sockaddr;
        let addr_len = std::mem::size_of::<SockAddrL2>() as libc::socklen_t;
        let ret = unsafe { libc::connect(fd.as_raw_fd(), addr_ptr, addr_len) };
        if ret < 0 {
            let err = std::io::Error::last_os_error();
            return Err(TransportError::with_source(
                format!("L2CAP connect to {address} PSM 0x{psm:04X} failed"),
                err,
            ));
        }

        let options = l2cap_options(&fd)?;
        log::info!(
            "L2CAP connected to {address} PSM 0x{psm:04X}, receive MTU: {}, send MTU: {}",
            options.imtu,
            options.omtu,
        );

        Ok(Self {
            fd,
            connected: AtomicBool::new(true),
        })
    }
}

impl L2capTransport for L2capLinux {
    fn send(&self, data: &[u8]) -> Result<(), TransportError> {
        if !self.connected.load(Ordering::Relaxed) {
            return Err(TransportError::NotConnected);
        }
        let n = nix::unistd::write(&self.fd, data)
            .map_err(|e| TransportError::with_source("L2CAP write failed", e))?;
        if n != data.len() {
            return Err(TransportError::new(format!(
                "short write: {n}/{} bytes",
                data.len()
            )));
        }
        Ok(())
    }

    fn recv(&self, buf: &mut [u8]) -> Result<usize, TransportError> {
        if !self.connected.load(Ordering::Relaxed) {
            return Err(TransportError::NotConnected);
        }
        let n = nix::unistd::read(&self.fd, buf)
            .map_err(|e| TransportError::with_source("L2CAP read failed", e))?;
        if n == 0 {
            self.connected.store(false, Ordering::Relaxed);
            return Err(TransportError::ConnectionClosed);
        }
        Ok(n)
    }

    fn close(&self) -> Result<(), TransportError> {
        self.connected.store(false, Ordering::Relaxed);
        // The OwnedFd will close on drop; explicit close is optional.
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }
}
