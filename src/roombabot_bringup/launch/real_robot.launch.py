import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Robot layer only (hardware): drivers, controllers, teleop plumbing and the EKF that
# owns odom -> base_footprint. Map provider (AMCL or SLAM) and Nav2 come from
# localized_navigation.launch.py / mapping.launch.py.


def generate_launch_description():
    motor_port = LaunchConfiguration("motor_port")
    imu_port = LaunchConfiguration("imu_port")
    encoder_ticks_per_revolution = LaunchConfiguration("encoder_ticks_per_revolution")

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
        default_value="2245.0",  # x4 quadrature; must match kTicksPerRev in esp32_motor.ino
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

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_localization"),
            "launch",
            "local_localization.launch.py"
        ),
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
        motor_port_arg,
        imu_port_arg,
        encoder_ticks_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_bridge,
        local_localization,
        foxglove_bridge
    ])
