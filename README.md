# roombabot_ws

ROS 2 Jazzy differential-drive robot on a Raspberry Pi, doing station-to-station
delivery on a SLAM-built map. Forked from `bumperbot_ws` with three changes:

- **No camera.** The ArUco fiducial stack and its marker-based docking are gone.
- **Roomba chassis.** 0.340 m diameter, 0.092 m tall, 0.064 m wheels on a 0.235 m track.
- **Two ESP32s.** The motor controllers and the MPU6050 now hang off their own ESP32s,
  which talk to the Pi over a binary serial protocol instead of the Pi driving an
  Arduino with ASCII and reading I²C itself.

For step-by-step commands to actually configure and run this (simulation and real
hardware), see [RUNNING.md](RUNNING.md).

## Hardware topology

```
                    +----------------------+
                    |  Raspberry Pi        |
                    |                      |
  /dev/esp32_motor  |  ros2_control @100Hz |   MotorCommand  -->
  <===== USB =======|    RoombabotInterface|   <-- MotorState
                    |                      |
  /dev/esp32_imu    |  imu_bridge          |   <-- ImuState
  <===== USB =======|    -> /imu/data      |
                    |                      |
  /dev/rplidar      |  rplidar_ros         |
  <===== USB =======|    -> /scan          |
                    +----------------------+
```

All three devices are addressed by **udev symlink**, never `/dev/ttyUSB*` — with two
ESP32s and the lidar on the bus, kernel enumeration order is not stable across boots,
and a swap would send wheel commands to the IMU board.

## The ESP32 link

The wire protocol is specified in
[`roombabot_firmware/firmware/PROTOCOL.md`](src/roombabot_firmware/firmware/PROTOCOL.md)
and defined in
[`esp32_protocol.hpp`](src/roombabot_firmware/include/roombabot_firmware/esp32_protocol.hpp),
which is ROS- and POSIX-free so the Arduino sketches can include it unchanged.

Four design choices, all aimed at keeping a 100 Hz control loop honest on a Pi:

1. **Nothing blocks the control loop.** A dedicated reader thread per link owns the fd
   and does the waiting in `poll()`. Decoded frames land in a
   [seqlock](src/roombabot_firmware/include/roombabot_firmware/seqlock.hpp), so
   `read()` is one atomic load and a 22-byte copy — no syscall, no allocation, no
   mutex. Writes are non-blocking and drop rather than stall; the next setpoint is
   10 ms away and strictly more correct than the one that didn't fit.
2. **Fixed-length binary frames, not ASCII lines.** No `getline`, no `stod`, no
   `stringstream` per cycle. There is no length field on the wire, so a corrupt byte
   costs at most one frame instead of desyncing the parser.
3. **Position from encoder ticks, not integrated velocity.** The old interface did
   `position += velocity * dt` with `dt` measured on the Pi, which baked both the
   velocity estimate's error and the control loop's scheduling jitter into permanent
   odometry drift. Ticks are differenced into an `int64`, wrap-safe and exact.
4. **Raw termios, not libserial.** One fewer dependency, and it allows `ASYNC_LOW_LATENCY`
   plus a single-syscall framed write.

The velocity PID loop lives on the ESP32, next to the encoders, not on an
OS-scheduled Pi. `ros2_control` here only passes setpoints through.

### Processes removed

| Gone | Why |
|---|---|
| `usb_cam` + `aruco_node` | no camera |
| `docking_controller` (ArUco) | replaced by an off-by-default stub that keeps the handoff contract |
| `mpu6050_driver.py` | 12 smbus byte-reads per cycle from Python; the ESP32 does the I²C now |
| `imu_republisher` | forwarded one topic to another unmodified — a whole process and DDS hop for nothing |

That freed enough headroom to put the local costmap back to 5 Hz, which the bumperbot
had to run at 3 Hz to stop the node missing its own lifecycle bond heartbeat.

## Stations

With no camera there is nothing to detect, so stations are **taught by driving**:
park the robot where it should end up, facing the way it should face, and call
`record_station`. What gets saved is the robot's own `map` → `base_footprint` pose at
that instant.

```bash
ros2 service call /record_station roombabot_msgs/srv/RecordStation "{id: 0, name: 'kitchen'}"
ros2 service call /save_stations std_srvs/srv/Trigger
```

Two things fall out of this for free: the recorded pose is reachable by construction
(the old standoff calculation could place a goal inside the costmap's inflated region
when a marker sat flush against a wall), and coarse navigation *is* the final approach,
so there is no residual error left for docking to remove.

`docking_controller` still exists as a pass-through stub behind `use_docking:=false`,
so the topics and sequencing survive for a future final-approach sensor.

## Chassis dimensions

Every piece of geometry is derived from the xacro properties at the top of
[`roombabot.urdf.xacro`](src/roombabot_description/urdf/roombabot.urdf.xacro). Two
values are duplicated outside that file and must be kept in step:

| Value | Also in |
|---|---|
| `wheel_radius`, `wheel_separation` | `roombabot_controllers.yaml`, `controller.launch.py` |
| `body_radius` (+ margin) | `robot_radius` in `costmap.yaml` and `planner_server.yaml` |

## Before first run

- [ ] Set `encoder_ticks_per_revolution` to the real value. There is deliberately no
      default in the hardware interface: getting it wrong scales all odometry by a
      constant, which produces a plausible-looking but wrong map rather than an obvious
      failure.
- [ ] Verify the gyro Z sign on the bench — rotating the robot left must give a
      **positive** `angular_velocity.z` on `/imu/data`. It is the EKF's only IMU input.
- [ ] Verify the wheel sign convention: positive `rad_s` and increasing ticks must both
      mean forward, on both sides.
- [ ] Check `laser_joint`'s yaw against how the RPLIDAR is physically mounted.
- [ ] Re-measure `body_diameter` against the actual shell if anything overhangs it;
      `robot_radius` carries only a 1 cm margin.

## Inherited notes worth keeping in view

- `bond_timeout` is 0.0 on both lifecycle managers — deliberate, see the comments in
  `navigation.launch.py`. Worth revisiting now that the Pi has more headroom.
- `amcl.yaml` still seeds `initial_pose` at a hardcoded (0,0,0) on every launch, and
  `recovery_alpha_slow`/`recovery_alpha_fast` are both 0.0 (augmented-MCL recovery
  disabled). Both carried over from the bumperbot unchanged.
- The undervoltage issue (`vcgencmd get_throttled` → `0x50000`) is a property of the
  power supply, not this workspace, and follows the hardware over.

## Layout

```
src/
  roombabot_bringup/       real_robot / simulated_robot launch, lidar config
  roombabot_controller/    diff_drive_controller config, twist_mux, joystick
  roombabot_description/   URDF/xacro, Gazebo, RViz
  roombabot_firmware/      ESP32 protocol, serial transport, ros2_control plugin, IMU bridge
  roombabot_localization/  EKF, AMCL
  roombabot_mapping/       slam_toolbox, maps, station_recorder
  roombabot_motion/        pure_pursuit, mission_commander, docking stub
  roombabot_msgs/          RecordStation.srv and friends
  roombabot_navigation/    planner_server, costmaps, waypoint_commander
  roombabot_planning/      DijkstraPlanner / AStarPlanner nav2 plugins
  roombabot_utils/         safety_stop
```
