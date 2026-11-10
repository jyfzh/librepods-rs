// LibrePods - AirPods liberated from Apple's ecosystem
// SPDX-License-Identifier: AGPL-3.0-only

//! SensorDataWX streaming and motion decoding helpers.
//!
//! Protocol reference: MousePods by Matteo Gaudioso (starman-tech),
//! https://github.com/starman-tech/mousepods (CC BY 4.0).

use super::packets::HEADER;

pub const WX_PREFIX: [u8; 10] = [4, 0, 4, 0, 0x17, 0, 0, 0, 0x10, 0];

/// Build a SensorDataWX report-interval request (microseconds).
pub fn stream(sequence: u8, service: u8, interval_us: u32) -> Vec<u8> {
    let mut body = vec![
        8,
        sequence & 0x7f,
        0x42,
        0x0b,
        8,
        service,
        0x10,
        2,
        0x1a,
        5,
        1,
    ];
    body.extend_from_slice(&interval_us.to_le_bytes());
    frame(&WX_PREFIX, &body)
}

fn frame(prefix: &[u8; 10], body: &[u8]) -> Vec<u8> {
    let mut packet = prefix.to_vec();
    packet.extend_from_slice(&(body.len() as u16).to_le_bytes());
    packet.extend_from_slice(body);
    packet
}

/// Validate the RTBuddy envelope independently of its control/data payload.
pub fn valid_envelope(packet: &[u8]) -> bool {
    packet.len() >= 12
        && packet.starts_with(&HEADER)
        && packet[4..6] == [0x17, 0]
        && packet.len() == 12 + u16::from_le_bytes([packet[10], packet[11]]) as usize
}

fn varint(bytes: &mut &[u8]) -> Option<u64> {
    let mut value = 0u64;
    for shift in (0..70).step_by(7) {
        let (&byte, rest) = bytes.split_first()?;
        *bytes = rest;
        if shift == 63 && byte > 1 {
            return None;
        }
        value |= u64::from(byte & 0x7f) << shift;
        if byte & 0x80 == 0 {
            return Some(value);
        }
    }
    None
}

// SensorDataWX is protobuf: field 7 is a sensor event, whose fields 1 and 3
// identify the service and contain the motion block respectively.
fn field<'a>(bytes: &mut &'a [u8]) -> Option<(u64, Option<u64>, &'a [u8])> {
    let key = varint(bytes)?;
    let (value, len) = match key & 7 {
        0 => return Some((key >> 3, Some(varint(bytes)?), &[])),
        1 => (None, 8),
        2 => (None, usize::try_from(varint(bytes)?).ok()?),
        5 => (None, 4),
        _ => return None,
    };
    let data = bytes.get(..len)?;
    *bytes = &bytes[len..];
    Some((key >> 3, value, data))
}

pub fn is_wx_motion(packet: &[u8]) -> bool {
    wx_motion_blocks(packet).is_some_and(|blocks| !blocks.is_empty())
}

/// Extract every motion block: a SensorDataWX packet can batch samples.
pub fn wx_motion_blocks(packet: &[u8]) -> Option<Vec<&[u8]>> {
    if !valid_envelope(packet) || !packet.starts_with(&WX_PREFIX) {
        return None;
    }
    let mut blocks = Vec::new();
    let mut body = &packet[12..];
    while !body.is_empty() {
        let (tag, _, mut event) = field(&mut body)?;
        if tag != 7 {
            continue;
        }
        let mut service = None;
        let mut block = &[][..];
        while !event.is_empty() {
            let (tag, value, data) = field(&mut event)?;
            match tag {
                1 => service = value,
                3 => block = data,
                _ => {}
            }
        }
        if matches!(service, Some(0x0e..=0x10)) && block.len() >= 58 {
            blocks.push(block);
        }
    }
    Some(blocks)
}

/// Decoded 58-byte SensorDataWX devmotion6 report.
/// Axes use the sensor's body frame; scales follow MousePods' measurements.
#[derive(Debug)]
pub struct MotionSample {
    pub timestamp_ns: u64,
    pub counter: u32,
    /// User acceleration with gravity removed, in m/s² (approximately).
    pub acceleration: [f64; 3],
    /// Gravity vector in m/s² (approximately).
    pub gravity: [f64; 3],
    /// Body-frame angular velocity in degrees/second (approximately).
    pub angular_velocity: [f64; 3],
    /// Reconstructed unit quaternion in (w, x, y, z) order.
    pub quaternion: Option<[f64; 4]>,
}

impl MotionSample {
    pub fn parse(block: &[u8]) -> Option<Self> {
        if block.len() < 58 || block[0] != 1 {
            return None;
        }
        let vector = |offset: usize, scale: f64| {
            std::array::from_fn(|i| {
                let start = offset + i * 2;
                f64::from(i16::from_le_bytes([block[start], block[start + 1]])) * scale
            })
        };
        let [x, y, z] = vector(20, 1.0 / 32768.0);
        let norm_squared = x * x + y * y + z * z;
        // MousePods' validity range excludes unavailable/zero attitude and
        // tolerates Q15 rounding near a 180-degree rotation.
        let quaternion = if 0.01 < norm_squared && norm_squared < 1.01 {
            let mut q = [(1.0 - norm_squared).max(0.0).sqrt(), x, y, z];
            let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            q.iter_mut().for_each(|v| *v /= norm);
            Some(q)
        } else {
            None
        };
        Some(Self {
            timestamp_ns: u64::from_le_bytes(block[1..9].try_into().ok()?),
            counter: u32::from_le_bytes(block[54..58].try_into().ok()?),
            acceleration: vector(38, 9.81 / 1000.0),
            gravity: vector(44, 9.81 / 1000.0),
            angular_velocity: vector(26, 0.122),
            quaternion,
        })
    }

    /// ZYX Euler angles (roll about X, pitch about Y, yaw about Z), in degrees.
    pub fn euler_degrees(&self) -> Option<[f64; 3]> {
        let [w, x, y, z] = self.quaternion?;
        Some([
            (2.0 * (w * x + y * z))
                .atan2(1.0 - 2.0 * (x * x + y * y))
                .to_degrees(),
            (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin().to_degrees(),
            (2.0 * (w * z + x * y))
                .atan2(1.0 - 2.0 * (y * y + z * z))
                .to_degrees(),
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_motion_units_signed_axes_and_attitude() {
        let mut block = [0u8; 58];
        block[0] = 1;
        block[1..9].copy_from_slice(&123_456_789u64.to_le_bytes());
        block[54..58].copy_from_slice(&42u32.to_le_bytes());
        for (offset, values) in [
            (20, [0i16, 0, 23170]), // 90 degrees about Z, Q15.
            (26, [100, -200, 0]),
            (38, [1000, -500, 0]),
            (44, [0, 0, -1000]),
        ] {
            for (i, value) in values.iter().enumerate() {
                block[offset + 2 * i..offset + 2 * i + 2].copy_from_slice(&value.to_le_bytes());
            }
        }
        let sample = MotionSample::parse(&block).unwrap();
        assert_eq!(sample.timestamp_ns, 123_456_789);
        assert_eq!(sample.counter, 42);
        for (actual, expected) in [
            (sample.acceleration[0], 9.81),
            (sample.acceleration[1], -4.905),
            (sample.gravity[2], -9.81),
            (sample.angular_velocity[0], 12.2),
            (sample.angular_velocity[1], -24.4),
        ] {
            assert!((actual - expected).abs() < 1e-9);
        }
        let [roll, pitch, yaw] = sample.euler_degrees().unwrap();
        assert!(roll.abs() < 1e-9 && pitch.abs() < 1e-9);
        assert!((yaw - 90.0).abs() < 0.01);
        block[20..26].fill(0);
        assert!(MotionSample::parse(&block).unwrap().quaternion.is_none());
        assert!(MotionSample::parse(&block[..57]).is_none());
        block[0] = 2;
        assert!(MotionSample::parse(&block).is_none());
    }

    #[test]
    fn extract_all_batched_wx_samples_and_reject_truncation() {
        let mut body = Vec::new();
        for counter in [1u32, 2] {
            body.extend_from_slice(&[0x3a, 62, 8, 0x10, 0x1a, 58]);
            let mut block = [0u8; 58];
            block[0] = 1;
            block[54..58].copy_from_slice(&counter.to_le_bytes());
            body.extend_from_slice(&block);
        }
        let packet = frame(&WX_PREFIX, &body);
        let blocks = wx_motion_blocks(&packet).unwrap();
        assert_eq!(blocks.len(), 2);
        assert_eq!(MotionSample::parse(blocks[1]).unwrap().counter, 2);
        // Preserve a valid outer envelope but truncate the nested protobuf data.
        body.pop();
        assert!(wx_motion_blocks(&frame(&WX_PREFIX, &body)).is_none());
    }
}
