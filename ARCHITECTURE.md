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
├── ble.rs                    # BLE scanning via btleplug
└── media.rs                  # wpctl / pactl + playerctl / MPRIS

librepods-windows/src/
├── lib.rs                    # #![cfg(target_os = "windows")]
├── l2cap.rs                  # WinSock AF_BTH L2CAP socket
├── device_monitor.rs         # WinRT Bluetooth connection events
├── ble.rs                    # BLE scanning via btleplug (WinRT backend)
└── media.rs                  # SMTC (WinRT) playback + default-render check

librepods-cli/src/
└── main.rs                   # CLI, handshake state machine, receive loop, `platform` module
```