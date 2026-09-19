import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    motor_port = LaunchConfiguration("motor_port")
    encoder_ticks_per_revolution = LaunchConfiguration("encoder_ticks_per_revolution")

    # udev symlink, not /dev/ttyUSB*. With two ESP32s and a lidar on the bus the
    # kernel's enumeration order is not stable across boots, and a swap would send
    # wheel commands to the IMU board.
    motor_port_arg = DeclareLaunchArgument(
        "motor_port",
        default_value="/dev/esp32_motor",
        description="Serial device for the ESP32 running the motor controllers",
    )

    # No default in the hardware interface on purpose -- getting this wrong scales all
    # wheel odometry by a constant factor, which looks like a plausible-but-wrong map
    # rather than an obvious failure.
    encoder_ticks_arg = DeclareLaunchArgument(
        "encoder_ticks_per_revolution",
        default_value="1440.0",
        description="Encoder counts per full revolution of the wheel, after gearing "
                    "and whatever edge-counting mode the ESP32 firmware uses",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("roombabot_description"),
                    "urdf",
                    "roombabot.urdf.xacro",
                ),
                " is_sim:=False",
                " motor_port:=", motor_port,
                " encoder_ticks_per_revolution:=", encoder_ticks_per_revolution,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("roombabot_controller"),
                "config",
                "roombabot_controllers.yaml",
            ),
        ],
    )

    return LaunchDescription(
        [
            motor_port_arg,
            encoder_ticks_arg,
            robot_state_publisher_node,
            controller_manager,
        ]
    )
