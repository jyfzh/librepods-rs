# LIBREPODS-RS

A Rust tool for streaming AirPods IMU data on Linux over the Apple Accessory
Protocol (AAP). No Apple device is needed.

Streams head tracking to OpenTrack over UDP, with automatic device detection
through BlueZ. IMU support depends on the AirPods model and firmware.

## Build

Install Rust, then build on Debian/Ubuntu:

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libdbus-1-dev libudev-dev bluetooth bluez
sudo systemctl enable --now bluetooth
cargo build --release -p librepods-cli
```

## Pair and run

Pair the AirPods using `bluetoothctl`:

```bash
bluetoothctl
```

```text
scan on
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
exit
```

Bluetooth sockets require capabilities:

```bash
sudo setcap cap_net_raw,cap_net_admin+eip ./target/release/librepods-cli

# Auto-detect connected AirPods and stream to OpenTrack
RUST_LOG=info ./target/release/librepods-cli

# Or specify the paired Bluetooth address
RUST_LOG=info ./target/release/librepods-cli --address AA:BB:CC:DD:EE:FF
```

Reapply capabilities after rebuilding, or run with `sudo`. Use `RUST_LOG=debug`
to log IMU samples and raw packets, and `--help` for all CLI options.

## OpenTrack UDP output

Select **UDP over network** as the input in OpenTrack, listening on port `4242`.
The CLI sends to `127.0.0.1:4242` by default. Face forward when starting;
restart the CLI to recenter.

```bash
# OpenTrack on another machine; adjust axis sensitivity/direction
./target/release/librepods-cli --opentrack 192.168.1.100:4242 \
  --opentrack-rate 100 --yaw-scale -1 --pitch-scale 1.5 --roll-scale 1
```

For remote output, allow UDP traffic on the receiving machine.
IPv6 addresses use brackets, e.g. `--opentrack '[::1]:4242'`.

| Option | Default | Meaning |
|---|---|---|
| `--opentrack IP:PORT` | `127.0.0.1:4242` | Destination IPv4 or IPv6 address and nonzero port |
| `--opentrack-rate HZ` | `120` | Maximum send rate, from 1 to 500 Hz |
| `--yaw-scale N` | `1` | Yaw gain; negative values invert direction |
| `--pitch-scale N` | `1` | Pitch gain; negative values invert direction |
| `--roll-scale N` | `1` | Roll gain; negative values invert direction |
| `--retries N` | `3` | Maximum connection retry attempts |

Output uses the MousePods UDP format: six little-endian `f64` values
(`x, y, z, yaw, pitch, roll`), with zero translation and rotation in degrees.
Poses require valid quaternion data and follow incoming samples (approximately
100 Hz), capped by `--opentrack-rate`. Adjust axis gains in the CLI or OpenTrack;
calibrated head-axis mapping and filtering are not included.

## Architecture and testing

| Crate | Purpose |
|---|---|
| `librepods-core` | Platform-independent AAP parsing, packet builders, IMU decoding, and device types |
| `librepods-linux` | L2CAP transport, BlueZ monitoring, and BLE library APIs |
| `librepods-cli` | Connection setup, IMU initialization, and motion output |

See [ARCHITECTURE.md](ARCHITECTURE.md) for the module layout.

```bash
cargo test --workspace
cargo fmt --all -- --check
```

## Troubleshooting

- **Permission denied:** reapply the capabilities shown above or run with `sudo`.
- **Bluetooth unavailable:** check `bluetoothctl show` and that the Bluetooth
  service is running.
- **Connected but no IMU samples:** verify that the AirPods are paired and
  connected, inspect `RUST_LOG=debug` output, and check model/firmware support.

## Credits

Inspired by [LibrePods](https://github.com/kavishdevar/librepods) and its
contributors. IMU initialization and decoding draw on research from
[MousePods](https://github.com/starman-tech/mousepods).
See also the [AAP protocol documentation](https://github.com/tyalie/AAP-Protocol-Defintion).

Licensed under AGPL-3.0-only. AirPods is a trademark of Apple Inc.
This project is not affiliated with, endorsed by, or sponsored by Apple Inc.
