# Running roombabot — simulation and real robot

Practical step-by-step guide. For *why* things are built this way, see
[README.md](README.md); for the ESP32 wire format, see
[src/roombabot_firmware/firmware/PROTOCOL.md](src/roombabot_firmware/firmware/PROTOCOL.md);
for a flat command cheat-sheet, see [Common commands](README.md#common-commands).
For how the launch files fit together (robot / map provider / navigation / mission
layers), see [Launch architecture](README.md#launch-architecture).

Assumes ROS 2 Jazzy already installed. Commands below are for a bash shell (the Pi, or
a Linux dev machine for simulation). `<ws>` means the absolute path to this workspace.

---

## Contents

1. [One-time setup](#one-time-setup)
2. [Simulation](#simulation)
3. [Real robot](#real-robot)
4. [Teleop and Foxglove (both)](#teleop-and-foxglove-both)
5. [Troubleshooting](#troubleshooting)

---

## One-time setup

```bash
cd <ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs `source install/setup.bash` (after `source /opt/ros/jazzy/setup.bash`
if that isn't already in your shell profile). If multiple robots/workstations share a
network, set a domain ID so they don't cross-talk:

```bash
export ROS_DOMAIN_ID=2
```

Put both `source` lines and the domain ID export in `~/.bashrc` on the Pi so you don't
retype them every session.

---

## Simulation

No ESP32s, no lidar, no udev rules needed — Gazebo supplies simulated joints, IMU and
lidar through `gz_ros2_control` and the `ros_gz_bridge`.

Every session is **two terminals, robot first**: the robot launch (this section's step
1), then one mission launch (step 2 to build a map, step 3 to navigate on one). Run
exactly one mission launch at a time — see
[Launch architecture](README.md#launch-architecture) for why.

### 1. Terminal 1: the simulated robot

```bash
ros2 launch roombabot_bringup simulated_robot.launch.py world_name:=small_house
```

This starts Gazebo, spawns the robot, `diff_drive_controller`, the joystick/twist_mux
plumbing, and the EKF (pointed at `ekf_simulation.yaml`). Nothing else: no map
provider, no Nav2. Worlds shipped in this workspace are `empty` and `small_house`; drop
a `.world` file into `src/roombabot_description/worlds/` and pass `world_name:=<name>`
to use another.

### 2. Terminal 2: build a map and teach stations

```bash
ros2 launch roombabot_bringup mapping.launch.py map_name:=demo
```

This starts `slam_toolbox`, `map_saver_server`, `station_recorder`, Nav2 and RViz
(`slam.rviz`). Add `use_navigation:=false` for plain teleop mapping without Nav2.
`map_name` names the map being built; it is where `stations.yaml` gets saved.

Drive the robot around (see [Teleop](#teleop-and-foxglove-both)) until the map in RViz
covers the area you need. Then, **while still driving**, park at each delivery station
and record it:

```bash
ros2 service call /record_station roombabot_msgs/srv/RecordStation "{id: 0, name: 'kitchen'}"
ros2 service call /record_station roombabot_msgs/srv/RecordStation "{id: 1, name: 'desk'}"
```

There is no marker to find — whatever pose the robot is actually sitting at when you
call this is what gets saved (`map` → `base_footprint`). Re-call with the same `id` to
overwrite a badly-parked station. The response echoes the exact x/y/yaw it captured.

When you're happy with the map and every station:

```bash
mkdir -p <ws>/src/roombabot_mapping/maps/demo
ros2 param set /station_recorder_node stations_path <ws>/src/roombabot_mapping/maps/demo/stations.yaml
ros2 service call /save_stations std_srvs/srv/Trigger

ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: '<ws>/src/roombabot_mapping/maps/demo/map', image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

Ctrl+C the **mission** launch only (leave the robot launch running if you like), then
rebuild so the new map/stations reach the install space:

```bash
colcon build --packages-select roombabot_mapping
source install/setup.bash
```

### 3. Terminal 2: navigate on a saved map

```bash
ros2 launch roombabot_bringup localized_navigation.launch.py map_name:=demo
```

This starts `map_server` + AMCL on `demo/map.yaml`, Nav2 and RViz (`nav2_setup.rviz`).
Set the robot's starting pose first (2D Pose Estimate in RViz, or the CLI form below —
`amcl.yaml` seeds `(0,0,0)` otherwise), then send a goal with 2D Goal Pose / Nav2 Goal
in RViz.

```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

> **Station missions are currently not launched.** `mission_commander`,
> `waypoint_commander` and the `docking_controller` stub (the
> `/mission/goto_station` → `/mission/arrived` flow) were started by
> `navigation.launch_old.py`; the Nav2-plugin launch files don't start them. Stations
> can still be recorded and saved; sending the robot to one by id needs those nodes
> brought back into a launch file.

---

## Real robot

### 0. Hardware prerequisites (once per robot)

- **udev symlinks** for all three USB devices, so kernel enumeration order (which is
  *not* stable across boots) can never send wheel commands to the IMU board. All three
  are CP2102 bridges with the same serial number, so
  [setup/99-roombabot.rules](setup/99-roombabot.rules) keys on the **physical USB
  port** — each cable has to stay in its port:

  ```bash
  sudo cp setup/99-roombabot.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ls -l /dev/esp32_motor /dev/esp32_imu /dev/rplidar   # confirm all three exist
  ```

- **Joystick access**: `joy_node` opens the controller through `/dev/input/event*`,
  which must be readable by the `input` group (and your user in it):

  ```bash
  sudo cp setup/81-event.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=input
  ```

  `~/.ros/log/joy_node_*.log` should then say `Opened joystick: ...`.

- **ESP32 firmware**: flash
  [esp32_motor.ino](src/roombabot_firmware/firmware/esp32_motor/esp32_motor.ino) and
  [esp32_imu.ino](src/roombabot_firmware/firmware/esp32_imu/esp32_imu.ino)
  (ESP32 board package 3.x, "ESP32 Dev Module", no extra libraries). They implement
  [PROTOCOL.md](src/roombabot_firmware/firmware/PROTOCOL.md) at 921600 baud. Check
  each board is talking before launching — every frame starts `a5 5a`:

  ```bash
  stty -F /dev/esp32_motor 921600 raw -echo; xxd -c 26 /dev/esp32_motor | head -3   # a55a02...
  stty -F /dev/esp32_imu 921600 raw -echo; xxd -c 34 /dev/esp32_imu | head -3       # a55a03...
  ```

- **Calibration** (measured on this robot 2026-10-11 — redo if wheels, motors, or
  the IMU mount change):

  | Value | Where | Measured |
  |---|---|---|
  | encoder ticks/rev (x4) | `encoder_ticks_per_revolution` in `real_robot.launch.py`, `kTicksPerRev` in `esp32_motor.ino` | 2245 ([encoder_calibration.ino](src/roombabot_firmware/firmware/encoder_calibration/encoder_calibration.ino)) |
  | wheel radius | `roombabot_controllers.yaml`, URDF | 0.0322 m (64.35 mm calipered) |
  | effective track | `wheel_separation_multiplier` | 0.969 (driven 360s vs gyro) |
  | IMU mount | `kAxisFrom`/`kAxisSign` in `esp32_imu.ino` | upside down, X arrow to the left |
  | top speed | controller `max_velocity` | 0.27 m/s on carpet (11.1 V LiPo via L298N), limited to 0.22 |

### 1. First bring-up: bench checks before the robot drives anywhere

With the robot up on blocks (wheels off the ground):

```bash
ros2 launch roombabot_bringup real_robot.launch.py
```

- `ros2 topic hz /imu/data` — should sit near 100 Hz. If it's silent, check
  `/dev/esp32_imu` and the ESP32's power/serial connection.
- `ros2 topic echo /imu/data`, then rotate the robot by hand
  counter-clockwise (left) — `angular_velocity.z` must read **positive**. This is the
  EKF's only IMU input; get it wrong and heading estimation is backwards. While
  you're there: flat should read `linear_acceleration.z` ≈ +9.8 (this MPU6050 reads
  ~11.5), nose tipped down → `x` goes **negative**, left side down → `y` goes
  **negative**.
- Spin one wheel by hand forward — its `/joint_states` `velocity` and `position` must
  both increase. Do this for both wheels; a wired-backwards wheel drives the robot in
  circles under closed-loop control.
- `ros2 control list_hardware_interfaces` / `list_hardware_components` — confirms the
  plugin picked up the right port and joint names.
- `ros2 topic hz /roombabot_controller/odom` — should hold steady near 100 Hz with no
  "missed deadline" spam in the launch output. If it's not, see
  [Troubleshooting](#troubleshooting).

Only put the robot on the ground once these all check out.

### 2. Build a map and teach stations (on hardware)

Same procedure as simulation, minus Gazebo. Terminal 1 is the robot, Terminal 2 the
mapping mission with the hardware settings (`use_sim_time:=False`, and no RViz on the Pi
— watch through Foxglove at `ws://<pi-ip>:8765`):

```bash
# Terminal 1
ros2 launch roombabot_bringup real_robot.launch.py
# Terminal 2
ros2 launch roombabot_bringup mapping.launch.py map_name:=demo use_sim_time:=False use_rviz:=False
```

Drive with a joystick (see [Teleop](#teleop-and-foxglove-both)) past every area the
robot needs to navigate, parking at and recording each station exactly as in the
[simulation walkthrough](#2-terminal-2-build-a-map-and-teach-stations) — the
`record_station` / `save_stations` calls are identical on hardware.

```bash
mkdir -p <ws>/src/roombabot_mapping/maps/demo
ros2 param set /station_recorder_node stations_path <ws>/src/roombabot_mapping/maps/demo/stations.yaml
ros2 service call /save_stations std_srvs/srv/Trigger
ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: '<ws>/src/roombabot_mapping/maps/demo/map', image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

Ctrl+C the mapping launch, then:

```bash
colcon build --packages-select roombabot_mapping
source install/setup.bash
```

### 3. Localize and navigate (on hardware)

```bash
# Terminal 1 (if not still running)
ros2 launch roombabot_bringup real_robot.launch.py
# Terminal 2
ros2 launch roombabot_bringup localized_navigation.launch.py map_name:=demo use_sim_time:=False use_rviz:=False
```

Together these bring up: the `esp32_motor` hardware interface + `diff_drive_controller`,
the `esp32_imu` bridge, the RPLIDAR driver, the EKF and the Foxglove bridge (robot
launch), then `map_server` + AMCL against `demo/map.yaml` and the Nav2 servers
(`planner_server` with the A\*/Dijkstra plugin, `controller_server` with
`PurePursuitController`, smoother, `bt_navigator`) from the mission launch.

Set the robot's real starting pose — `amcl.yaml` seeds `(0,0,0)` on every launch, which
is very unlikely to be where the robot actually is:

```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

(or set it via 2D Pose Estimate in Foxglove, dragging to the robot's true position and
heading). Then send a goal from Foxglove, or with the `navigate_to_pose` action.

### 4. Optional: obstacle e-stop lock

`roombabot_utils` ships a `safety_stop` node (lidar-based emergency stop, wired into
`twist_mux` at priority 255 via `twist_mux_locks.yaml`) but it is **not** started by
any launch file in this workspace — run it explicitly if you want it:

```bash
ros2 run roombabot_utils safety_stop --ros-args -p danger_distance:=0.2 -p warning_distance:=0.6
```

---

## Teleop and Foxglove (both)

**Joystick** (started automatically by both `real_robot.launch.py` and
`simulated_robot.launch.py`, priority 99 in `twist_mux` — always wins over navigation):
plug in a gamepad, `device_id: 0` in `joy_config.yaml` is the default first joystick.

**Foxglove**: `real_robot.launch.py` starts the bridge automatically
(`ws://<pi-ip>:8765`, bound to `0.0.0.0` so any device on the network can connect). In
simulation, start it by hand if you want it:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Connect Foxglove Studio/desktop to `ws://<host>:8765`. Useful panels: `/scan` +
`/map` + TF for driving/mapping, `/stations/found` or `/stations/markers`
(`MarkerArray`) to see recorded stations, `/mission/arrived` to watch mission
completion, and `/imu/data` + `/roombabot_controller/odom` for the hardware sanity
checks in step 1 above.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `/roombabot_controller/odom` misses its rate, "missed deadline" spam | Check `ros2 topic hz /imu/data` and whether `/dev/esp32_motor` frames are arriving (`ros2 control list_hardware_components` should show the interface as `active`, not `unconfigured`). A dropped USB connection reports as a hardware `read()` error, not a silent hang. |
| Odometry drifts as a straight-line scale error | `encoder_ticks_per_revolution` is wrong. Re-derive it from the gearbox ratio and encoder CPR rather than guessing. |
| Robot drives backwards / spins on straight commands | A wheel's sign convention doesn't match — positive `rad_s` **and** increasing ticks must both mean "this wheel drives forward" on both sides. Recheck with the bench test in [step 1](#1-first-bring-up-bench-checks-before-the-robot-drives-anywhere). |
| Heading drifts steadily in one direction | Gyro Z sign is inverted, or the ESP32's own startup bias calibration ran while the robot was moving. Re-verify the sign check; `imu_bridge`'s own `calibration_samples` (default 200) is a *second*, residual bias pass on top of whatever the ESP32 already does. |
| `nav2_lifecycle_manager` logs a bond connect-then-dead loop | Known, carried over from the bumperbot: `bond_timeout` is set to `0.0` (disabled) in both lifecycle managers because of Pi undervoltage (`vcgencmd get_throttled`). Check power delivery before re-enabling it. |
| AMCL "shifts then suddenly jumps" | `recovery_alpha_slow`/`recovery_alpha_fast` are both `0.0` in `amcl.yaml` (augmented-MCL recovery disabled), inherited unchanged. Worth revisiting now that camera/aruco load is gone. |
| Map and lidar visibly shift, or the Nav2 panel shows Navigation "inactive" | Two copies of something are running (two mission launches, a leftover EKF/AMCL, or a stale `lifecycle_manager_navigation` from an earlier session). Stop everything, then `ps -eo pid,lstart,comm \| grep -E "ekf\|amcl\|lifecycle\|bt_nav\|gz"` must print nothing before relaunching. |
| Costmap/planner treats a big chunk of open floor as "unknown = wall" | `track_unknown_space: false` in both costmap configs is intentional (`DijkstraPlanner` treats unknown cells as impassable) — make sure SLAM actually swept the area, not just drove past it at a distance. |
| A too-large or too-small robot footprint in the costmap | `robot_radius` (0.18 m, i.e. 0.17 m body + 1 cm margin) lives in both `costmap.yaml` and `planner_server.yaml` and must match `body_diameter` in `roombabot.urdf.xacro` if you ever re-measure the chassis. |
