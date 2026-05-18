# NYU Robotic Design Team Rover Firmware (2025–26)

Embedded firmware for an autonomous lunar excavation rover competing in [NASA Lunabotics 2025–26](https://www.nasa.gov/learning-resources/lunabotics-challenge/) at Kennedy Space Center, represeting @nyurdt. The rover excavates and deposits lunar regolith simulant under operator control. This repository contains the full embedded stack running on the rover's Teensy 4.1: I2C command reception, CAN-based actuator control, sensor acquisition, and a complete FreeRTOS port of the firmware. The code in this repo was copied from the private electrical engineering repo to avoid sharing access to internal design docs and discussions, hardware files etc. but all relevant information about the architecture, design decisions, and development process is included in this doc and the `RDT_25_26_RTOS/RTOS_design.md` file.

> **2026 Competition:** The NYU RDT team is currently competing at University of Central Florida and then (hopefully) Kennedy Space Center, FL - results TBD.

---

## System Architecture

The rover uses a two-MCU control architecture. The parent controller (Jetson Nano, managed by the software team) sends command bytes over I2C. The child controller (Teensy 4.1, this repo) receives them, dispatches to the appropriate subsystem, and drives brushless motor controllers over CAN at 500 kbps. The Teensy also returns an 18-byte telemetry packet on I2C request.

```
  Jetson Nano (SW team)         Teensy 4.1 (this repo)         Motor Controllers
  ┌────────────────┐  I2C      ┌──────────────────────┐  CAN   ┌──────────────────┐
  │                │─────────> │                      │───────>│ Left Loco  0x4C  │
  │  parent/       │  Wire2    │  RDT_2025_26_TEENSY  │        │ Right Loco 0x78  │
  │  commander     │  400 kHz  │  child / actuator    │        │ Excavation 0x48  │
  │  (addr master) │  pins     │  (addr 0x08)         │        └──────────────────┘
  └────────────────┘  18 / 19  └──────────────────────┘
                                         │
                               Actuators & Sensors
                               ├─ Stepper motor (excavation arm vertical)
                               ├─ Deposition door (H-bridge + PWM)
                               ├─ Vibration motor
                               ├─ 8-ch current sensing (mux ADC)
                               ├─ String potentiometer (arm position)
                               ├─ Rotary encoders (wheel odometry)
                               └─ Load cells × 4 (HX711)
```

---

## Repository Structure

| Directory | Description | Status |
|---|---|---|
| `RDT_2025_26_TEENSY4_1/` | Superloop firmware - deployed on competition rover | Active |
| `RDT_25_26_RTOS/` | FreeRTOS port - complete, hardware-tested | Complete |
| `i2c_parent/` | Jetson-side I2C test harness (WASD control + telemetry) | Test utility |
| `controller_bridge/` | Xbox controller -> I2C bridge (Python) | Test utility |

---

## Superloop Firmware (`RDT_2025_26_TEENSY4_1`)

The competition firmware. Follows a superloop architecture: `ROVER_update()` runs every loop iteration, dispatches I2C commands through a group-handler table, and updates all subsystems in sequence. PlatformIO's `build_src_filter` allows individual subsystems to be isolated into separate environments for bring-up and calibration without modifying the main source.

All hardware pin assignments and runtime constants live in a single file: `include/config.h`. Feature flags can be toggled there without touching logic.

### Build

Requires [PlatformIO](https://platformio.org/).

```bash
cd RDT_2025_26_TEENSY4_1

pio run              # build all environments
pio run -e teensy41  # main firmware only
```

### PlatformIO Environments

| Environment | Purpose |
|---|---|
| `teensy41` | Main competition firmware |
| `teensy41_sensor_test` | Current sensor + encoder bring-up → Teleplot |
| `teensy41_string_pot_test` | String potentiometer calibration |
| `teensy41_depo_door_test` | Deposition door timing and state machine verification |
| `teensy41_depo_door_raw` | Raw GPIO door bring-up (no driver layer) |
| `teensy41_load_cell_test` | HX711 load cell bring-up |

### Feature Flags

| Flag | Default | Effect |
|---|---|---|
| `RAMP_UP` | `1` | Speed ramping on all CAN motor outputs |
| `SERIAL_DEBUG` | `0` | Human-readable serial log |
| `PLOT_DATA` | `1` | Teleplot-format sensor stream (mutually exclusive with `SERIAL_DEBUG`) |
| `USE_TIMEOUT` | `1` | 500 ms I2C command timeout → auto e-stop |
| `USE_OLD_HEX_MAPPING` | `1` | Legacy command byte mapping (matches the SW team's Jetson driver) |

---

## FreeRTOS Firmware (`RDT_25_26_RTOS`)

A complete FreeRTOS port of the competition firmware, written from scratch for this rover. All design goals were met and the firmware was validated end-to-end on hardware. It was not deployed to the competition robot because the superloop had accumulated significantly more test coverage by the time hardware testing concluded. It will serve as a reference for next season's firmware development, and the architecture and design patterns are applicable to any future rover designs. 

See [`RDT_25_26_RTOS/RTOS_design.md`](RDT_25_26_RTOS/RTOS_design.md) for the full architecture, task topology, IPC contracts, and acceptance criteria.

### Task Architecture

Seven concurrent tasks communicate through a shared `DesiredState` struct (mutex-protected), a `egSafetyBits` event group, and a `qI2cRxBytes` byte queue for I2C command ingress. A hardware `IntervalTimer` ISR drives the stepper independently of all tasks.

| Task | Priority | Period | Responsibility |
|---|---:|---|---|
| `SafetyTask` | 7 | 5 ms | Relay monitoring, timeout detection, safety bit writes, hard-stop notifications |
| `MotorControlTask` | 4 | 20 ms | Speed slewing, CAN output for locomotion and excavation belt |
| `MechanismTask` | 4 | 5 ms | Stepper enable/direction, deposition door state machine, vibration motor |
| `CmdDecodeTask` | 3 | event | I2C queue drain, `DesiredState` writes, kill-switch command gating |
| `SensorTask` | 2 | 10 ms | Current mux stepping, string pot, encoder reads all into `SensorSnapshot` |
| `TelemetryTask` | 2 | 20 ms | 18-byte packet assembly, double-buffer swap |
| `DebugTask` | 1 | 50 ms | Teleplot output, diagnostic counters |

### Design Highlights

**Hardware-timed stepper pulses.** The excavation arm stepper STEP pin is driven by a Teensyduino `IntervalTimer` ISR, not toggled in a task. Pulse timing is independent of RTOS scheduling jitter which is critical for reliable stepper behavior under burst I2C command loads.

**Centralized safety ownership.** `SafetyTask` is the sole writer of the `egSafetyBits` event group. It monitors the hardware kill-switch relay, software e-stop commands, and the 500 ms comms timeout. Actuator tasks read the bits every cycle; direct task notifications provide a low-latency hard-stop path on top of the persistent bit state.

**Lock-free telemetry.** `TelemetryTask` builds packets into a double buffer and flips an atomic `gTelemetryReady` index when a new packet is ready. The I2C request ISR reads that index to pick the active buffer with no mutex or critical section in the ISR path.

**CAN IRQ disabled intentionally.** FlexCAN's TX-completion ISR was preempting FreeRTOS SysTick during back-to-back CAN writes, causing `vTaskDelayUntil()` to block indefinitely and halting the scheduler. Since 3 writes per 20 ms fit comfortably in the 58 available mailboxes without software FIFO assistance, `NVIC_DISABLE_IRQ(IRQ_CAN1)` resolved the issue at no cost to functionality.

**String pot over-travel with hysteresis.** `MechanismTask` enforces arm travel limits using hysteresis latching: once the arm reaches a limit, vertical motion is blocked until the pot reads back past the threshold by a configurable margin. This prevents limit-bounce from re-enabling motion on a noisy sensor edge.

### Build

```bash
cd RDT_25_26_RTOS
pio run -t upload
```

---

## I2C Test Harness (`i2c_parent`)

A Teensy-based I2C master for bench testing without the full Jetson software stack. Supports WASD keyboard control over USB serial, sequential command scripting, and Teleplot telemetry polling. Mirrors the Jetson's role in the comms chain. Use by uploading to separate teensy and connecting it to the rover controller over i2c, then send commands over serial monitor to the parent teensy.

```bash
cd i2c_parent
pio run -t upload
```

---

## Controller Bridge (`controller_bridge`)

A Python script that maps an Xbox controller to the rover's I2C command protocol and relays commands through `i2c_parent` over USB serial. Used for manual hardware testing sessions, as we found that the physical controller was a lot more fun to test with. uplaod i2c_parent first, then run this script on your computer with the controller connected.

---

## Command Protocol

All commands are 1-byte frames: `(group << 4) | param`.

### Groups

| Group | Value | Motion-gated | Description |
|---|---|---|---|
| Control | `0x0` | No | Software e-stop (`param=0x1`) |
| Loco Stop | `0x1` | Yes | Locomotion stop (ramped) |
| Forward | `0x2` | Yes | Drive forward |
| Backward | `0x3` | Yes | Drive backward |
| Left | `0x4` | Yes | Skid-steer left |
| Right | `0x5` | Yes | Skid-steer right |
| Excavation | `0x6` | Yes | Arm vertical + belt control |
| Deposition | `0x7` | Yes | Door open/close, vibration motor |
| Data | `0x8` | No | Request 18-byte telemetry packet |

*Motion-gated: command is rejected while the hardware kill-switch relay is disengaged.*

### Speed Parameter

For motion groups, `param[3:0]` selects a speed level:

| Value | Fraction | Effective loco speed | Effective loco speed (belt active) |
|---|---|---|---|
| `0` | 25% | ~8.3% duty | ~2.5% duty |
| `1` | 50% | ~16.5% duty | ~5% duty |
| `2` | 75% | ~25% duty | ~7.5% duty |
| `3` | 100% | ~33% duty | ~10% duty |

Locomotion duty is capped at 33% normally and 10% when the excavation belt is running.

### Telemetry Packet (18 bytes)

Returned on I2C data request:

| Bytes | Content | Notes |
|---|---|---|
| `[0–7]` | Motor currents, 8 channels | uint8, scaled; `0xFF` = sensor not active |
| `[8]` | Left wheel encoder | degrees, scaled |
| `[9]` | Right wheel encoder | degrees, scaled |
| `[10–13]` | Load cells, 4 channels | kg, scaled; `0xFF` = not active |
| `[14]` | Excavation arm position | cm from string pot |
| `[15]` | Deposition door state | `0=closed, 1=open, 2=opening, 4=closing` |
| `[16]` | Status flags | see bit map below |
| `[17]` | not used | `0xFF` |

**Status flags (byte 16, active-high):**

| Bit | Flag | Condition |
|---|---|---|
| 0 | `FLAG_ESTOP` | Hardware kill-switch relay not engaged |
| 1 | `FLAG_OVERCURRENT` | Any current channel over threshold |
| 2 | `FLAG_MACRO_ACTIVE` | MCU running an autonomous correction sequence |
| 3 | `FLAG_LOCO_STALL` | Locomotion motor stalled under load |
| 4 | `FLAG_EXCAV_ARM_OBSTRUCTED` | Stepper current spike while arm descending |
| 5 | `FLAG_EXCAV_EMPTY_CUT` | Belt running, low current — no material to excavate |
| 6 | `FLAG_EXCAV_STALL` | Belt running, high current, no mass accumulation |
| 7 | `FLAG_DEPO_BIN_EMPTY` | Bin mass below empty threshold |

---

## Contributors

This firmware was developed by members of the Electrical Engineering subteam of New York University's Robotic Design Team. I (Antoni Korycki, @tonykorycki) led this part of the project in my roles as an EE team Lead and System Engineer for Electrical Engineering, building on rover firmware I authored for the 2024–25 season, and other prior work by members of the EE subteam. The superloop firmware was a collaborative effort across the EE subteam, shaped by close coordination with members working on hardware bring-up and with Mechanical and Software team leads. The FreeRTOS architecture and implementation were designed and written by me, and - I hope - will be improved by the next generation of EEs at RDT and serve as a reference for future firmware development.
Special thanks to Jason (@jqin40), Liza (@eliz-ko), and Abdullah (@AAlduraibi) for their work on the firmware, my fellow EE leads Eric and Bonnie for making this possible, and the rest of the team for a great season! Its been a pleasure and I'm excited to see the new generation take over next year :D
