import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    map_name = LaunchConfiguration("map_name")
    use_docking = LaunchConfiguration("use_docking")
    motor_port = LaunchConfiguration("motor_port")
    imu_port = LaunchConfiguration("imu_port")
    encoder_ticks_per_revolution = LaunchConfiguration("encoder_ticks_per_revolution")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="bedroom",
        description="Name of the saved map (folder under roombabot_mapping/maps) to localize against when use_slam is false"
    )

    use_docking_arg = DeclareLaunchArgument(
        "use_docking",
        default_value="false",
        description="Whether to run docking_controller for a final approach after "
                    "coarse navigation. Defaults false: stations are taught by driving "
                    "the robot to them, so the recorded pose is already the final pose "
                    "and there is nothing left to servo onto. Kept as a hook for a "
                    "future final-approach sensor."
    )

    # All three USB devices are addressed by udev symlink rather than /dev/ttyUSB*.
    # With two ESP32s and the lidar on the bus, kernel enumeration order is not stable
    # across boots, and a swap would send wheel commands to the IMU board.
    motor_port_arg = DeclareLaunchArgument(
        "motor_port",
        default_value="/dev/esp32_motor",
        description="udev symlink for the ESP32 running the motor controllers"
    )

    imu_port_arg = DeclareLaunchArgument(
        "imu_port",
        default_value="/dev/esp32_imu",
        description="udev symlink for the ESP32 reading the MPU6050"
    )

    encoder_ticks_arg = DeclareLaunchArgument(
        "encoder_ticks_per_revolution",
        default_value="561.2",
        description="Encoder counts per full wheel revolution, after gearing"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={
            "motor_port": motor_port,
            "encoder_ticks_per_revolution": encoder_ticks_per_revolution,
        }.items(),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("roombabot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
            "use_sim_time": "False"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    # Replaces the bumperbot's mpu6050_driver.py, which read the MPU6050 over the Pi's
    # own I2C bus from Python at 100 Hz. The ESP32 does the I2C now; this node parses
    # one binary frame per sample in C++ and publishes imu/data directly -- the old
    # imu_republisher forwarding hop is gone too.
    imu_bridge = Node(
        package="roombabot_firmware",
        executable="imu_bridge",
        name="imu_bridge",
        output="screen",
        parameters=[{
            "port": imu_port,
            "frame_id": "imu_link",
            "imu_topic": "imu/data",
        }],
    )

    global_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": "False"
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_localization"),
            "launch",
            "local_localization.launch.py"
        ),
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": "False",
            "use_docking": use_docking
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": "False"
        }.items(),
        condition=IfCondition(use_slam)
    )

    # --- FOXGLOVE BRIDGE SECTION ---
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{
            "port": 8765,                  # Default WebSocket port for Foxglove Studio
            "address": "0.0.0.0",         # Bind to all interfaces so external devices can connect
            "send_buffer_limit": 10000000 # Buffer limit to prevent drops on high-bandwidth data like lasers/images
        }],
        output="screen"
    )

    return LaunchDescription([
        use_slam_arg,
        map_name_arg,
        use_docking_arg,
        motor_port_arg,
        imu_port_arg,
        encoder_ticks_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_bridge,
        local_localization,
        global_localization,
        navigation,
        slam,
        foxglove_bridge
    ])
