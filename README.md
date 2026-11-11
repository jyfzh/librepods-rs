# LIBREPODS-RS

A Rust tool for streaming AirPods IMU data on Linux over the Apple Accessory
Protocol (AAP). No Apple device is needed.

The CLI connects over Bluetooth L2CAP and outputs user acceleration, angular
velocity, gravity, quaternion, and Euler angles. It supports an explicit
Bluetooth address or automatic detection through BlueZ, with connection retries.
Battery, ear detection, and other notifications are ignored by the CLI.

## Build

Install Rust and the Linux build dependencies:

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libdbus-1-dev libudev-dev bluetooth bluez
sudo systemctl enable --now bluetooth
cargo build --release -p librepods-cli
```

## Pair and run

Pair the AirPods using `bluetoothctl`:

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

# Auto-detect connected AirPods
RUST_LOG=info ./target/release/librepods-cli

# Or specify the paired Bluetooth address
RUST_LOG=info ./target/release/librepods-cli --address AA:BB:CC:DD:EE:FF
```

Reapply capabilities after rebuilding the binary. Alternatively, run the binary
with `sudo`. IMU availability depends on the AirPods model and firmware.

## CLI options

```text
Usage: librepods-cli [OPTIONS]

Options:
  -a, --address <ADDRESS>  Bluetooth address; otherwise auto-detect via BlueZ
      --retries <RETRIES>  Maximum connection retry attempts [default: 3]
  -h, --help               Print help
```

Use `RUST_LOG=debug` to include raw IMU packet logging.

## IMU initialization

The Linux backend requires an encrypted L2CAP link (`BT_SECURITY_MEDIUM`) and
configures receive/send MTU to 2582 bytes before connecting to PSM `0x1001`.
After the AAP handshake, feature setup, and notification request, the CLI:

1. Starts SensorDataWX activity (`0x0E`) with a 40000 µs interval.
2. Waits 250 ms.
3. Starts devmotion6 (`0x10`) with a 10000 µs interval.
4. Stops the activity stream.
5. Decodes motion samples and logs timestamps, counters, acceleration in m/s²,
   angular velocity in degrees/s, gravity in m/s², and available attitude data.

When the receive loop ends or sensor initialization fails, the CLI attempts to
stop both streams. Abrupt process termination cannot perform this cleanup.

## Architecture and testing

| Crate | Purpose |
|---|---|
| `librepods-core` | Platform-independent AAP parsing, packet builders, IMU decoding, and device types |
| `librepods-linux` | L2CAP transport, BlueZ monitoring, and BLE library APIs |
| `librepods-cli` | Connection setup, IMU initialization, and motion output |

The core library retains general protocol support for use by library consumers.
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
