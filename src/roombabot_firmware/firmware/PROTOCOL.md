# roombabot ESP32 wire protocol (v1)

Everything the two `.ino` sketches have to agree with. The authoritative definition is
[`esp32_protocol.hpp`](../include/roombabot_firmware/esp32_protocol.hpp) — this file
describes it; that file compiles it.

Copy `esp32_protocol.hpp` into the Arduino sketch folder and include it directly (the
`esp32_motor/` and `esp32_imu/` sketch folders each carry a copy -- re-copy after any
change to the header). It is
deliberately free of ROS and POSIX headers so it compiles unchanged under the Arduino
toolchain, which removes any chance of the two ends drifting apart.

## Transport

| Board | Device (udev) | Role |
|---|---|---|
| `esp32_motor` | `/dev/esp32_motor` | wheel velocity in, encoder ticks + measured velocity out |
| `esp32_imu` | `/dev/esp32_imu` | MPU6050 gyro + accel out |

921600 baud, 8N1, no flow control, on both ends. These boards are ESP32 DevKitV1s,
whose USB port goes through a CP2102 USB-UART bridge to the ESP32's hardware UART0 -- so
the baud rate is real and **must match** (`Serial.begin(921600)` in the sketches,
`baud_rate` 921600 on the Pi). It would only be ignored on a native-USB ESP32 (S2/S3/C3
CDC).

Two CP2102 behaviours worth knowing:

- All three bridges on this robot report serial number `0001`, so udev can only tell
  them apart by physical USB port (`ATTRS{devpath}`) -- see `setup/99-roombabot.rules`.
- Right after the Pi opens a port, the link can replay one stale frame 100-200 times
  before the live stream starts. Every frame carries a `seq`, so consumers drop a frame
  whose `seq` repeats the previous one (`imu_bridge` does).

Both boards are addressed by udev symlink rather than `/dev/ttyUSB*`, because with two
ESP32s and the RPLIDAR on the same bus the kernel's enumeration order is not stable
across boots.

## Framing

```
+--------+--------+--------+-----------------+--------+
| 0xA5   | 0x5A   | TYPE   | PAYLOAD (N)     | CRC8   |
+--------+--------+--------+-----------------+--------+
```

- `N` is fixed per `TYPE` — there is **no length field on the wire**. A corrupted byte
  can therefore cost at most one frame; it can never desync the parser indefinitely.
- `CRC8` is CRC-8/ATM (poly `0x07`, init `0x00`, no reflection, no final XOR) computed
  over `TYPE` **and** `PAYLOAD`, not over the sync bytes.
- Structs are `#pragma pack(1)` and memcpy'd straight onto the wire. Both ends are
  little-endian with IEEE-754 floats, so no per-field serialization is needed.
- Velocities are `float32`. Positions are raw `int32` encoder ticks, never a float
  angle — the Pi differences successive tick counts into an `int64`, so wraparound is
  handled and position never accumulates integration error.

## Messages

### `0x01` MotorCommand — Pi → esp32_motor (10-byte payload, 14-byte frame)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `float32` | `left_rad_s` | commanded left **wheel** angular velocity |
| 4 | `float32` | `right_rad_s` | commanded right wheel angular velocity |
| 8 | `uint8` | `seq` | free-running; gaps mean the Pi dropped a frame |
| 9 | `uint8` | `flags` | bit0 `kCmdEnable` — clear to coast/brake regardless of setpoints |

Sent once per ros2_control cycle (50 Hz, `update_rate` in `roombabot_controllers.yaml`). **The firmware must run a command watchdog**:
if no `MotorCommand` arrives for ~250 ms, stop the motors and raise
`kMotorCmdWatchdog` in the next `MotorState`. Closing the serial port on the Pi does
not by itself stop a moving robot.

### `0x02` MotorState — esp32_motor → Pi (22-byte payload, 26-byte frame)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `int32` | `left_ticks` | free-running, signed, wraps freely — **do not reset** |
| 4 | `int32` | `right_ticks` | |
| 8 | `float32` | `left_rad_s` | measured, from encoder edge timing on the ESP32 |
| 12 | `float32` | `right_rad_s` | |
| 16 | `uint32` | `esp_micros` | `micros()` at sample time; diagnostics only |
| 20 | `uint8` | `seq` | |
| 21 | `uint8` | `status` | bit0 e-stop, bit1 cmd watchdog expired, bit2 encoder fault |

Send at 100 Hz. It does **not** need to be phase-locked to the incoming commands — the
Pi consumes whatever the latest frame is and treats "no new frame this cycle" as
normal. Only a sustained gap (`state_timeout`, default 0.5 s) is treated as a fault.

Sign convention: positive `rad_s` and increasing ticks must both mean the wheel driving
the robot **forward**, on both sides. Get this wrong and odometry runs backwards.

### `0x03` ImuState — esp32_imu → Pi (30-byte payload, 34-byte frame)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `float32 × 3` | `gyro_x/y/z` | rad/s, REP-103 body axes, bias-corrected |
| 12 | `float32 × 3` | `accel_x/y/z` | m/s² |
| 24 | `uint32` | `esp_micros` | |
| 28 | `uint8` | `seq` | |
| 29 | `uint8` | `status` | bit0 not initialised, bit1 read fault, bit2 calibrating |

Send at 100 Hz. Responsibilities that belong on the ESP32, not the Pi:

- Convert raw MPU6050 counts to rad/s and m/s² using the range the firmware actually
  configured. (The old Python driver had a live bug here: it set `GYRO_CONFIG = 24`
  — FS_SEL=3, ±2000 °/s — while the comment claimed ±250 °/s.)
- Rotate into REP-103 body axes (x forward, y left, z up) according to how the board is
  physically mounted.
- Average a startup gyro bias while stationary, raising `kImuCalibrating` throughout so
  the Pi ignores those samples.

**Verify the yaw sign on the bench**: rotating the robot counter-clockwise (left) must
produce a positive `gyro_z`. This is the single most consequential sign in the stack —
it feeds the EKF's only IMU input.

The Pi applies a *second*, residual bias pass of its own (`calibration_samples`,
default 200) on top of the ESP32's. That covers the case where the ESP32 booted while
the robot was being carried and so calibrated against motion.

### `0x04` SysCommand — Pi → either board (2-byte payload, 6-byte frame)

| Offset | Type | Field |
|---|---|---|
| 0 | `uint8` | `command` — 0 ping, 1 reset encoders, 2 recalibrate IMU |
| 1 | `uint8` | `seq` |

Not currently sent by any node; defined so the boards can implement it now rather than
needing a protocol revision later.

## Firmware-side notes

- **Write whole frames in one call.** A frame split across two USB transfers is still
  parsed correctly by the Pi, but batching keeps the reader thread's wakeup count down.
- **Do not print anything else to the port.** No `Serial.println` debugging on the data
  link — stray ASCII costs a resync per occurrence. Use a second UART, or add a typed
  message.
- **Keep the PID loop on the ESP32.** Velocity control belongs next to the encoders,
  running on a deterministic MCU, not on an OS-scheduled Pi. The ros2_control loop here
  only passes setpoints through; it does no wheel-level control of its own.
- The Pi's writes are non-blocking and a frame is dropped rather than queued if the TX
  buffer backs up. Firmware must therefore tolerate an occasional missing `seq`.

## Bandwidth

At 100 Hz per link: 1.4 kB/s out to the motor board, 2.6 kB/s back, 3.4 kB/s in from
the IMU board. Roughly 0.6% of a 921600-baud UART, and nothing at all over USB — the
protocol is nowhere near being the bottleneck, which is the point.
