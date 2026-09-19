# Running roombabot — simulation and real robot

Practical step-by-step guide. For *why* things are built this way, see
[README.md](README.md); for the ESP32 wire format, see
[src/roombabot_firmware/firmware/PROTOCOL.md](src/roombabot_firmware/firmware/PROTOCOL.md);
for a flat command cheat-sheet, see [common_commands.txt](common_commands.txt).

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

### 1. Bring up Gazebo + controllers + EKF

```bash
ros2 launch roombabot_bringup simulated_robot.launch.py world_name:=empty
```

This starts Gazebo, spawns the robot, `diff_drive_controller`, joystick/twist_mux
plumbing, and the EKF (pointed at `ekf_simulation.yaml` automatically). Only the
`empty` world ships in this workspace (the AWS RoboMaker house/warehouse worlds were
dropped along with the camera demo assets) — drop a `.world` file into
`src/roombabot_description/worlds/` and pass `world_name:=<name>` to use another.

By default it also brings up **localization against a saved map** (`use_slam:=false`).
Add `use_slam:=true` instead if you're building a new map (see step 2).

> **Gotcha:** the localization launch file's own default map is `small_house`, which
> isn't in this workspace's `maps/` folder (only `bedroom`, `cube`, `demo`, `house`
> came over). Even though `simulated_robot.launch.py` doesn't explicitly forward a
> `map_name` argument, ROS 2 launch shares `LaunchConfiguration` values across included
> files — a nested `DeclareLaunchArgument("map_name", ...)` only applies its own
> default when nothing has already set that name. So passing `map_name:=` on the
> command line still works even though `--show-args` on the top-level file won't
> list it:
>
> ```bash
> ros2 launch roombabot_bringup simulated_robot.launch.py map_name:=bedroom
> ```
>
> If that ever stops working (e.g. after a launch API change), the fallback is to
> build your own map first (next section) — that avoids needing an existing one at all.

RViz opens automatically (`global_localization.rviz` or `slam.rviz`, matching
`use_slam`) with `use_sim_time:=true` already set.

### 2. Build a map and teach stations (in sim)

```bash
ros2 launch roombabot_bringup simulated_robot.launch.py use_slam:=true
```

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

Ctrl+C, rebuild so the new map/stations reach the install space, then relaunch:

```bash
colcon build --packages-select roombabot_mapping
source install/setup.bash
ros2 launch roombabot_bringup simulated_robot.launch.py map_name:=demo
```

### 3. Run the delivery mission (in sim)

With localization running (`use_slam:=false`, a `map_name` that has `stations.yaml`),
set the robot's starting pose (2D Pose Estimate in RViz, or the CLI form below), then:

```bash
ros2 topic pub /mission/goto_station std_msgs/msg/Int32 "{data: 0}" --once
ros2 topic echo /mission/arrived   # fires once the robot reaches the station
```

`use_docking` is `false` by default — coarse navigation *is* the final approach here
(stations are taught by driving, so there's no residual error to remove). See
[README.md](README.md#stations) if you want the docking stub in the loop anyway.

---

## Real robot

### 0. Hardware prerequisites (once per robot)

- **udev symlinks** for all three USB devices, so kernel enumeration order (which is
  *not* stable across boots) can never send wheel commands to the IMU board. A
  reference rule file is at
  [setup/99-roombabot.rules](setup/99-roombabot.rules) — fill in the real
  `idVendor`/`idProduct`/`serial` values with `udevadm info -a -n /dev/ttyUSB0`, then:

  ```bash
  sudo cp setup/99-roombabot.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ls -l /dev/esp32_motor /dev/esp32_imu /dev/rplidar   # confirm all three exist
  ```

- **ESP32 firmware** implementing the protocol in
  [PROTOCOL.md](src/roombabot_firmware/firmware/PROTOCOL.md) is flashed to both
  boards. This workspace doesn't include the `.ino` sketches — write them against that
  spec (frame format, message types, the command watchdog requirement, and the sign
  conventions called out below).
- **`encoder_ticks_per_revolution`** is measured for your gearbox/encoder combo. There
  is deliberately no built-in default in the hardware interface — passing the wrong
  value scales all odometry by a constant, which produces a plausible-looking but
  *wrong* map rather than an obvious failure.

### 1. First bring-up: bench checks before the robot drives anywhere

With the robot up on blocks (wheels off the ground):

```bash
ros2 launch roombabot_bringup real_robot.launch.py use_slam:=true encoder_ticks_per_revolution:=<your value>
```

- `ros2 topic hz /imu/data` — should sit near 100 Hz. If it's silent, check
  `/dev/esp32_imu` and the ESP32's power/serial connection.
- `ros2 topic echo /imu/data --once`, then rotate the robot by hand
  counter-clockwise (left) — `angular_velocity.z` must read **positive**. This is the
  EKF's only IMU input; get it wrong and heading estimation is backwards.
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

Same procedure as simulation, minus `use_sim_time`:

```bash
ros2 launch roombabot_bringup real_robot.launch.py use_slam:=true map_name:=demo
```

Drive with a joystick (see [Teleop](#teleop-and-foxglove-both)) past every area the
robot needs to navigate, parking at and recording each station exactly as in the
[simulation walkthrough](#2-build-a-map-and-teach-stations-in-sim) — the
`record_station` / `save_stations` calls are identical on hardware.

```bash
mkdir -p <ws>/src/roombabot_mapping/maps/demo
ros2 param set /station_recorder_node stations_path <ws>/src/roombabot_mapping/maps/demo/stations.yaml
ros2 service call /save_stations std_srvs/srv/Trigger
ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: '<ws>/src/roombabot_mapping/maps/demo/map', image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

Ctrl+C, then:

```bash
colcon build --packages-select roombabot_mapping
source install/setup.bash
```

### 3. Localize and navigate (on hardware)

```bash
ros2 launch roombabot_bringup real_robot.launch.py use_slam:=false map_name:=demo
```

This brings up: `esp32_motor` hardware interface + `diff_drive_controller`, the
`esp32_imu` bridge, the RPLIDAR driver, AMCL against `demo/map.yaml`, the EKF, the
Dijkstra-based `planner_server` + local costmap, `pure_pursuit`, `waypoint_commander`,
`mission_commander`, the `docking_controller` stub, and the Foxglove bridge
(`ws://<pi-ip>:8765`).

Set the robot's real starting pose — `amcl.yaml` seeds `(0,0,0)` on every launch, which
is very unlikely to be where the robot actually is:

```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

(or set it via 2D Pose Estimate in Foxglove/RViz, dragging to the robot's true
position and heading). Then run the mission:

```bash
ros2 topic pub /mission/goto_station std_msgs/msg/Int32 "{data: 0}" --once
ros2 topic echo /mission/arrived
```

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
| Simulation localization can't find its map | See the [`map_name` gotcha](#1-bring-up-gazebo--controllers--ekf) above — the default is `small_house`, which isn't in this workspace. Pass `map_name:=` explicitly. |
| Costmap/planner treats a big chunk of open floor as "unknown = wall" | `track_unknown_space: false` in both costmap configs is intentional (`DijkstraPlanner` treats unknown cells as impassable) — make sure SLAM actually swept the area, not just drove past it at a distance. |
| A too-large or too-small robot footprint in the costmap | `robot_radius` (0.18 m, i.e. 0.17 m body + 1 cm margin) lives in both `costmap.yaml` and `planner_server.yaml` and must match `body_diameter` in `roombabot.urdf.xacro` if you ever re-measure the chassis. |
