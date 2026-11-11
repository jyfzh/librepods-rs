// LibrePods - AirPods liberated from Apple's ecosystem
// Copyright (C) 2025 LibrePods contributors
// SPDX-License-Identifier: AGPL-3.0-only

//! Connect to AirPods over L2CAP and stream IMU motion samples.

use std::time::Duration;

use clap::Parser;
use librepods_core::protocol::{packets, parser, tracking};
use librepods_core::transport::{DeviceEvent, DeviceMonitor, L2capTransport};

#[cfg(target_os = "linux")]
mod platform {
    pub use librepods_linux::bluez_monitor::BluezMonitor as Monitor;
    pub use librepods_linux::l2cap::L2capLinux as Transport;
}

#[cfg(not(target_os = "linux"))]
compile_error!("librepods-cli currently supports only Linux targets");

#[derive(Parser)]
#[command(name = "librepods", about = "AirPods IMU streaming daemon")]
struct Cli {
    /// Bluetooth address to connect to directly (e.g. "AA:BB:CC:DD:EE:FF").
    /// If not given, will auto-detect via BlueZ.
    #[arg(short, long)]
    address: Option<String>,

    /// Maximum connection retry attempts.
    #[arg(long, default_value = "3")]
    retries: u32,
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();

    let address = if let Some(addr) = cli.address {
        log::info!("Using provided address: {addr}");
        addr
    } else {
        log::info!("Watching for AirPods...");
        let monitor = match platform::Monitor::start() {
            Ok(m) => m,
            Err(e) => {
                log::error!("Failed to start device monitor: {e}");
                std::process::exit(1);
            }
        };
        loop {
            match monitor.next_event() {
                Ok(DeviceEvent::Connected { address, name }) => {
                    log::info!("AirPods detected: {name} ({address})");
                    break address;
                }
                Ok(DeviceEvent::Disconnected { address }) => {
                    log::info!("Device disconnected: {address}");
                }
                Err(e) => {
                    log::error!("Monitor error: {e}");
                    std::process::exit(1);
                }
            }
        }
    };

    let transport = match connect_with_retries(&address, cli.retries) {
        Some(t) => t,
        None => {
            log::error!("Failed to connect after {} attempts", cli.retries);
            std::process::exit(1);
        }
    };

    if let Err(e) = run_connection(transport.as_ref()) {
        log::error!("Connection ended: {e}");
        std::process::exit(1);
    }
    log::info!("Disconnected.");
}

fn connect_with_retries(address: &str, max_retries: u32) -> Option<Box<dyn L2capTransport>> {
    for attempt in 1..=max_retries {
        log::info!("Connection attempt {attempt}/{max_retries} to {address}");
        match platform::Transport::connect(address) {
            Ok(t) => return Some(Box::new(t)),
            Err(e) => {
                log::warn!("Attempt {attempt} failed: {e}");
                if attempt < max_retries {
                    std::thread::sleep(Duration::from_millis(1500));
                }
            }
        }
    }
    None
}

fn run_connection(transport: &dyn L2capTransport) -> Result<(), Box<dyn std::error::Error>> {
    log::info!("Sending handshake...");
    transport.send(&packets::HANDSHAKE)?;

    let mut buf = [0u8; packets::AAP_MTU as usize];
    let n = transport.recv(&mut buf)?;
    match parser::parse(&buf[..n])? {
        parser::ParsedPacket::HandshakeAck => log::info!("Handshake ACK received"),
        other => log::warn!("Expected HandshakeAck, got: {other:?}"),
    }
    transport.send(&packets::SET_SPECIFIC_FEATURES)?;

    let n = transport.recv(&mut buf)?;
    match parser::parse(&buf[..n])? {
        parser::ParsedPacket::FeaturesAck => log::info!("Features ACK received"),
        other => log::warn!("Expected FeaturesAck, got: {other:?}"),
    }
    transport.send(&packets::REQUEST_NOTIFICATIONS)?;

    // Prime activity before enabling devmotion6, then disable activity so
    // only the motion stream remains active (SensorDataWX initialization).
    let result = (|| -> Result<(), Box<dyn std::error::Error>> {
        transport.send(&tracking::stream(1, 0x0e, 40_000))?;
        std::thread::sleep(Duration::from_millis(250));
        transport.send(&tracking::stream(2, 0x10, 10_000))?;
        transport.send(&tracking::stream(3, 0x0e, 0))?;
        log::info!("IMU stream requested: devmotion6 (0x10), interval 10000 µs");

        loop {
            let n = transport.recv(&mut buf)?;
            if n == 0 {
                break;
            }
            match parser::parse(&buf[..n]) {
                Ok(parser::ParsedPacket::HeadTracking(data)) => log_motion(&data),
                Ok(_) => {}
                Err(e) => log::warn!("Parse error: {e}"),
            }
        }
        Ok(())
    })();

    // Stop both streams even if initialization or reception failed.
    let _ = transport.send(&tracking::stream(4, 0x10, 0));
    let _ = transport.send(&tracking::stream(5, 0x0e, 0));
    result
}

fn log_motion(data: &[u8]) {
    if let Some(blocks) = tracking::wx_motion_blocks(data) {
        for block in blocks {
            if let Some(sample) = tracking::MotionSample::parse(block) {
                let [ax, ay, az] = sample.acceleration;
                let [gx, gy, gz] = sample.gravity;
                let [wx, wy, wz] = sample.angular_velocity;
                let attitude = match (sample.quaternion, sample.euler_degrees()) {
                    (Some([w, x, y, z]), Some([roll, pitch, yaw])) => format!(
                        "quat(wxyz)=[{w:+.4}, {x:+.4}, {y:+.4}, {z:+.4}] \
                        rpy(deg)=[{roll:+.2}, {pitch:+.2}, {yaw:+.2}]"
                    ),
                    _ => "attitude=unavailable".to_string(),
                };
                log::info!(
                    "IMU t={}ns #{} accel(m/s²)=[{ax:+.3}, {ay:+.3}, {az:+.3}] \
                    gyro(deg/s)=[{wx:+.3}, {wy:+.3}, {wz:+.3}] \
                    gravity(m/s²)=[{gx:+.3}, {gy:+.3}, {gz:+.3}] {attitude}",
                    sample.timestamp_ns,
                    sample.counter,
                );
            }
        }
    }
    log::debug!("IMU raw AAP packet ({} bytes): {:02X?}", data.len(), data);
}
