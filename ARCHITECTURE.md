# Architecture Overview

```
librepods-core/src/
├── lib.rs
├── transport.rs              # I/O traits + TransportError
├── protocol/
│   ├── mod.rs
│   ├── opcodes.rs            # AAP opcode constants (single source of truth)
│   ├── packets.rs            # Packet constants + builder functions
│   ├── parser.rs             # ParsedPacket enum + parse()
│   ├── tracking.rs           # SensorDataWX stream commands and IMU decoding
│   └── control_command.rs    # ControlCommand builder + ControlCommandId
├── device/
│   ├── mod.rs
│   ├── battery.rs            # BatteryInfo
│   ├── state.rs              # DeviceState, AirPodsModel, NoiseControlMode
│   ├── ble_advert.rs         # BLE advertisement decoder
│   └── listening_mode.rs
└── crossdevice/              # Cross-device sync protocol
    ├── mod.rs
    ├── packets.rs
    └── relay.rs

librepods-linux/src/
├── lib.rs
├── l2cap.rs                  # Raw AF_BLUETOOTH L2CAP socket via nix
├── bluez_monitor.rs          # BlueZ D-Bus connection events via zbus
└── ble.rs                    # BLE scanning via btleplug (library API)

librepods-cli/src/
└── main.rs                   # CLI, AAP handshake, IMU streaming and motion logging
```

The CLI connects to AirPods using an explicit Bluetooth address or BlueZ
auto-detection, initializes SensorDataWX, and logs IMU samples. Other AAP
notifications are ignored. The core library retains its protocol parsers and
device types; the Linux library provides Bluetooth I/O.
