# roombabot_ws

ROS 2 Jazzy differential-drive robot on a Raspberry Pi, doing station-to-station
delivery on a SLAM-built map. Forked from `bumperbot_ws` with three changes:

- **No camera.** The ArUco fiducial stack and its marker-based docking are gone.
- **Roomba chassis.** 0.340 m diameter, 0.092 m tall, 0.064 m wheels on a 0.235 m track.
- **Two ESP32s.** The motor controllers and the MPU6050 now hang off their own ESP32s,
  which talk to the Pi over a binary serial protocol instead of the Pi driving an
  Arduino with ASCII and reading I²C itself.

For step-by-step commands to actually configure and run this (simulation and real
hardware), see [RUNNING.md](RUNNING.md). For a flat cheat-sheet, see
[Common commands](#common-commands) below.

## Launch architecture

The system is split into four layers. Each layer has one job, and every transform has
exactly one owner. Simulation and real robot differ **only** in layer 1.

| Layer | Launch file | Owns |
|---|---|---|
| **1. Robot** | `simulated_robot.launch.py` or `real_robot.launch.py` (`roombabot_bringup`) | Sensors (`/scan`, `/imu/data`), wheel odometry, `cmd_vel` path (`twist_mux`, joystick), and the EKF that publishes `odom → base_footprint`. Gazebo in sim; ESP32 drivers, lidar and Foxglove on hardware |
| **2. Map provider** | inside layer 4 launches | `map → odom` and `/map`. **Either** `map_server` + AMCL (localize on a saved map) **or** `slam_toolbox` (build a map). Never both |
| **3. Navigation** | `navigation.launch.py` (`roombabot_navigation`) | Nav2 only: planner, smoother, controller, costmaps, `bt_navigator`. No localization, no RViz, so it behaves identically on top of either map provider |
| **4. Mission** | `localized_navigation.launch.py` or `mapping.launch.py` (`roombabot_bringup`) | Picks the map provider, adds Nav2, opens RViz |

```
 Robot layer (sim or real)
   └─ EKF ──────────────────────────────► odom → base_footprint
 Mission layer
   ├─ localized_navigation.launch.py
   │     map_server + AMCL ─────────────► map → odom, /map
   │     navigation.launch.py (Nav2)
   └─ mapping.launch.py
         slam_toolbox + map_saver ──────► map → odom, /map
         navigation.launch.py (Nav2)       (use_navigation:=false for teleop-only)
```

A normal session is two terminals, robot first:

```bash
# Terminal 1 - the robot (pick one)
ros2 launch roombabot_bringup simulated_robot.launch.py world_name:=small_house
ros2 launch roombabot_bringup real_robot.launch.py

# Terminal 2 - the mission (pick one)
ros2 launch roombabot_bringup mapping.launch.py map_name:=house
ros2 launch roombabot_bringup localized_navigation.launch.py map_name:=house
```

On the real robot, pass `use_sim_time:=False use_rviz:=False` to the mission launch
(`use_sim_time` defaults to `True`, which suits Gazebo; view through Foxglove instead of
RViz on the Pi).

Rules that keep it working:

- **Run exactly one mission launch**, and never two copies of anything. Two map
  providers, or two copies of the EKF or Nav2, both publish the same transforms; the map
  and lidar visibly shift and the lifecycle managers fight over the same node names.
  After stopping a session, check nothing survived:
  `ps -eo pid,lstart,comm | grep -E "ekf|amcl|lifecycle|bt_nav|gz"` should print nothing.
- **One lifecycle manager per layer**, each with its own name and node list:
  `lifecycle_manager_localization`, `lifecycle_manager_slam`,
  `lifecycle_manager_navigation`.
- **`use_sim_time` is passed down explicitly**, never assumed. The EKF config differs
  too (`ekf_simulation.yaml` vs `ekf.yaml`), but that choice is made by the robot
  launch, so nothing above layer 1 needs to know.

### Planned: autonomous exploration

The intended next step is a fifth piece in layer 4 (a `roombabot_exploration` package;
it does not exist yet). Design decisions made so that it drops in cleanly:

- **Nav2's `bt_navigator` stays stock.** The exploration logic is a separate
  BehaviorTree.CPP node that calls the `NavigateToPose` action as a client: find
  frontiers, pick one, navigate, blacklist it on failure, repeat, then call
  `map_saver_server` when none remain. Strategies can then change without touching Nav2.
  It will run alongside `mapping.launch.py`.
- **Costmaps already follow `/map`.** The global costmap's static layer subscribes to
  `/map`, so it works with `slam_toolbox`'s growing map as well as a saved one. Check
  this first when exploration is wired up.
- **Unknown space is currently impassable.** `track_unknown_space: false` and the
  A\*/Dijkstra planner treat unknown cells as walls. Frontier goals sit in free cells
  next to unknown, so they are reachable, but the planner cannot route *through*
  unexplored space.
- **Don't send goals until ready.** Wait for `lifecycle_manager_navigation` to report
  active and for `map → odom` to exist.
- **Handoff:** save the map, stop `mapping.launch.py`, start
  `localized_navigation.launch.py` with the new `map_name`.

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

`docking_controller` still exists as a pass-through stub so the topics and sequencing
survive for a future final-approach sensor. **It is not currently launched:** the
Nav2-plugin launch files don't start `waypoint_commander`, `mission_commander` or
`docking_controller` (they were started by `navigation.launch_old.py`, kept for
reference in `roombabot_navigation/launch/`). Recording stations still works, since
`station_recorder` is part of the SLAM launch.

## Chassis dimensions

Every piece of geometry is derived from the xacro properties at the top of
[`roombabot.urdf.xacro`](src/roombabot_description/urdf/roombabot.urdf.xacro). Two
values are duplicated outside that file and must be kept in step:

| Value | Also in |
|---|---|
| `wheel_radius`, `wheel_separation` | `roombabot_controllers.yaml`, `controller.launch.py` |
| `body_radius` (+ margin) | `robot_radius` in `costmap.yaml` and `planner_server.yaml` |

## Before first run

The checked items were verified on this robot on 2026-10-11 — see the calibration table in
[RUNNING.md](RUNNING.md#0-hardware-prerequisites-once-per-robot) for the values and how
each was measured. Redo the relevant check if hardware changes:

- [x] `encoder_ticks_per_revolution` = 2245 (x4 decode). Getting it wrong scales all
      odometry by a constant — a plausible-looking but wrong map, not an obvious failure.
- [x] Gyro Z sign: rotating left gives **positive** `angular_velocity.z` on `/imu/data`.
      It is the EKF's only IMU input. The IMU is mounted upside down and turned 90°;
      `esp32_imu.ino` rotates it into REP-103.
- [x] Wheel signs: positive `rad_s` and increasing ticks both mean forward, both sides.
- [x] Wheel radius 0.0322 m (calipered), effective track via
      `wheel_separation_multiplier` 0.969.
- [ ] Check `laser_joint`'s yaw against how the RPLIDAR is physically mounted.
- [ ] Re-measure `body_diameter` against the actual shell if anything overhangs it;
      `robot_radius` carries only a 1 cm margin.

## Inherited notes worth keeping in view

- `bond_timeout` is 0.0 (bond monitoring disabled) on the localization and SLAM
  lifecycle managers — deliberate, see the comments in `global_localization.launch.py`.
  `lifecycle_manager_navigation` now uses the default timeout. Worth revisiting now that
  the Pi has more headroom.
- `amcl.yaml` still seeds `initial_pose` at a hardcoded (0,0,0) on every launch, and
  `recovery_alpha_slow`/`recovery_alpha_fast` are both 0.0 (augmented-MCL recovery
  disabled). Both carried over from the bumperbot unchanged.
- The undervoltage issue (`vcgencmd get_throttled` → `0x50000`) is a property of the
  power supply, not this workspace, and follows the hardware over.

## Common commands

All commands assume `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
in the terminal. Step-by-step walkthroughs are in [RUNNING.md](RUNNING.md).

### Build

```bash
colcon build --parallel-workers 1                        # whole workspace (Pi-friendly)
colcon build --packages-select roombabot_firmware        # one package
export ROS_DOMAIN_ID=0                                   # keep multiple robots apart
```

### Launch

```bash
# Layer 1 - robot
ros2 launch roombabot_bringup simulated_robot.launch.py world_name:=small_house
ros2 launch roombabot_bringup real_robot.launch.py

# Layer 4 - mission (simulation; on hardware add use_sim_time:=False use_rviz:=False)
ros2 launch roombabot_bringup mapping.launch.py map_name:=demo
ros2 launch roombabot_bringup localized_navigation.launch.py map_name:=demo

# Teleop-only mapping (no Nav2)
ros2 launch roombabot_bringup mapping.launch.py map_name:=demo use_navigation:=false

# Keyboard teleop (sim)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=key_vel

# Foxglove bridge (started automatically by real_robot.launch.py)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Pose and frames

```bash
# Set the initial pose of the robot (or use 2D Pose Estimate in RViz/Foxglove)
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Frame convention (REP-103): red = X, green = Y, blue = Z.

### Build a map and record stations (`mapping.launch.py` running)

Drive around to build the map. Stations are taught by driving: park the robot exactly
where it should end up, facing the way it should face.

```bash
# 1. make the save folder
mkdir -p ~/roombabot_ws/src/roombabot_mapping/maps/demo

# 2. record each station (repeat, driving to each one first). The response echoes the
#    exact x/y/yaw captured; re-calling with the same id overwrites it.
ros2 service call /record_station roombabot_msgs/srv/RecordStation "{id: 0, name: 'kitchen'}"
ros2 service call /record_station roombabot_msgs/srv/RecordStation "{id: 1, name: 'desk'}"

# 3. write the stations out
ros2 service call /save_stations std_srvs/srv/Trigger

# 4. save the map itself, into the same folder
ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: '/home/seancour/roombabot_ws/src/roombabot_mapping/maps/demo/map', image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"

# 5. rebuild so the map and stations reach the install space
colcon build --packages-select roombabot_mapping
source install/setup.bash
```

Then Ctrl+C the mapping launch and start `localized_navigation.launch.py map_name:=demo`.

### ESP32 links and hardware checks

```bash
# Did both boards and the lidar come up with their udev names?
ls -l /dev/esp32_motor /dev/esp32_imu /dev/rplidar

# Wheel state from the motor board; IMU board and its rate (100 Hz expected)
ros2 topic echo /joint_states
ros2 topic echo /imu/data --once
ros2 topic hz /imu/data

# Did the hardware interface pick up the right port and encoder scaling?
ros2 control list_hardware_interfaces
ros2 control list_hardware_components

# Is the 100 Hz control loop holding? (no "missed deadline" spam = good)
ros2 topic hz /roombabot_controller/odom

# Override the encoder scaling without editing the xacro
ros2 launch roombabot_bringup real_robot.launch.py encoder_ticks_per_revolution:=1440.0

# Point at different boards (e.g. a bench ESP32 on a raw tty)
ros2 launch roombabot_bringup real_robot.launch.py motor_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1

# Raw frame sanity check straight off the wire - expect A5 5A leading every frame
xxd -c 26 /dev/esp32_motor | head
```

## Layout

```
src/
  roombabot_bringup/       layer 1 robot launches (real / simulated), layer 4 mission
                           launches (mapping, localized_navigation), lidar config
  roombabot_controller/    diff_drive_controller config, twist_mux, joystick
  roombabot_description/   URDF/xacro, Gazebo, RViz
  roombabot_firmware/      ESP32 protocol, serial transport, ros2_control plugin, IMU bridge
  roombabot_localization/  EKF (layer 1), map_server + AMCL (layer 2, localize mode)
  roombabot_mapping/       slam_toolbox, map_saver, maps, station_recorder (layer 2, map mode)
  roombabot_motion/        PurePursuitController nav2 plugin, mission_commander, docking stub
  roombabot_msgs/          RecordStation.srv and friends
  roombabot_navigation/    layer 3: Nav2 servers, costmaps, behavior trees
  roombabot_planning/      DijkstraPlanner / AStarPlanner nav2 plugins
  roombabot_utils/         safety_stop
```
