// SPDX-License-Identifier: AGPL-3.0-only
//! OpenTrack "UDP over network": six little-endian doubles, cm and degrees.
//! Wire format reference: MousePods opentrack_out.py by Matteo Gaudioso.

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use librepods_core::protocol::tracking::MotionSample;

pub struct Output {
    socket: UdpSocket,
    reference: Option<[f64; 4]>,
    period: Duration,
    last_send: Option<Instant>,
    last_warning: Option<Instant>,
    scale: [f64; 3],
}

impl Output {
    pub fn new(target: SocketAddr, rate: u16, scale: [f64; 3]) -> io::Result<Self> {
        if target.port() == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "UDP destination port must be nonzero",
            ));
        }
        let socket = UdpSocket::bind(if target.is_ipv4() {
            "0.0.0.0:0"
        } else {
            "[::]:0"
        })?;
        socket.connect(target)?;
        socket.set_nonblocking(true)?;
        log::info!("OpenTrack UDP enabled: {target}, maximum {rate} Hz; first valid attitude defines center");
        Ok(Self {
            socket,
            reference: None,
            period: Duration::from_secs_f64(1.0 / f64::from(rate)),
            last_send: None,
            last_warning: None,
            scale,
        })
    }

    pub fn feed(&mut self, sample: &MotionSample) {
        let Some(q) = sample.quaternion else { return };
        let reference = *self.reference.get_or_insert(q);
        let now = Instant::now();
        if self
            .last_send
            .is_some_and(|last| now.duration_since(last) < self.period)
        {
            return;
        }
        self.last_send = Some(now);
        let pose = relative_pose(reference, q, self.scale);
        if !pose.iter().all(|v| v.is_finite()) {
            return;
        }
        if let Err(e) = self.socket.send(&encode(pose)) {
            // UDP errors must not terminate Bluetooth reception or flood the log.
            if self
                .last_warning
                .is_none_or(|last| now.duration_since(last) >= Duration::from_secs(5))
            {
                log::warn!("OpenTrack UDP send failed: {e}");
                self.last_warning = Some(now);
            }
        }
    }
}

fn relative_pose(reference: [f64; 4], q: [f64; 4], scale: [f64; 3]) -> [f64; 6] {
    // Inverse(reference) * current: rotation relative to the initial sensor frame.
    let [a, b, c, d] = reference;
    let [e, f, g, h] = q;
    let [w, x, y, z] = [
        a * e + b * f + c * g + d * h,
        a * f - b * e - c * h + d * g,
        a * g + b * h - c * e - d * f,
        a * h - b * g + c * f - d * e,
    ];
    // AirPods sensor X maps to head yaw, Y to roll, and negative Z to pitch.
    let pitch = -(2.0 * (w * z + x * y))
        .atan2(1.0 - 2.0 * (y * y + z * z))
        .to_degrees();
    let roll = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin().to_degrees();
    let yaw = (2.0 * (w * x + y * z))
        .atan2(1.0 - 2.0 * (x * x + y * y))
        .to_degrees();
    [
        0.0,
        0.0,
        0.0,
        yaw * scale[0],
        pitch * scale[1],
        roll * scale[2],
    ]
}

fn encode(pose: [f64; 6]) -> [u8; 48] {
    let mut packet = [0; 48];
    for (chunk, value) in packet.chunks_exact_mut(8).zip(pose) {
        chunk.copy_from_slice(&value.to_le_bytes());
    }
    packet
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn relative_rotation_center_and_axis_gains() {
        // Stay away from the Euler singularity at 90 degrees.
        let (s, c) = (std::f64::consts::FRAC_PI_8).sin_cos();
        let identity = [1.0, 0.0, 0.0, 0.0];
        for (q, axis, degrees) in [
            ([c, 0.0, 0.0, s], 4, -45.0),
            ([c, 0.0, 0.0, -s], 4, 45.0),
            ([c, 0.0, s, 0.0], 5, 45.0),
            ([c, s, 0.0, 0.0], 3, 45.0),
        ] {
            let scales = [-2.0, 0.5, 1.0];
            let pose = relative_pose(identity, q, scales);
            assert!((pose[axis] - degrees * scales[axis - 3]).abs() < 1e-5);
            for (index, value) in pose.iter().enumerate() {
                if index != axis {
                    assert!(value.abs() < 1e-5);
                }
            }
            assert!(relative_pose(q, q, [1.0; 3]).iter().all(|v| v.abs() < 1e-5));
        }
    }

    #[test]
    fn udp_packet_layout_and_rate_limit() {
        let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
        receiver
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();
        let mut output = Output::new(receiver.local_addr().unwrap(), 1, [1.0; 3]).unwrap();
        let mut sample = MotionSample {
            timestamp_ns: 0,
            counter: 0,
            acceleration: [0.0; 3],
            gravity: [0.0; 3],
            angular_velocity: [0.0; 3],
            quaternion: None,
        };
        output.feed(&sample);
        assert!(output.reference.is_none());
        sample.quaternion = Some([1.0, 0.0, 0.0, 0.0]);
        output.feed(&sample);
        let mut buf = [0; 64];
        assert_eq!(receiver.recv(&mut buf).unwrap(), 48);
        for chunk in buf[..48].chunks_exact(8) {
            assert_eq!(f64::from_le_bytes(chunk.try_into().unwrap()), 0.0);
        }
        output.feed(&sample);
        assert!(receiver.recv(&mut buf).is_err());
        let packet = encode([0.0, 0.0, 0.0, 90.0, -45.0, 10.0]);
        assert_eq!(&packet[24..32], &90f64.to_le_bytes());
        assert_eq!(&packet[32..40], &(-45f64).to_le_bytes());
    }
}
