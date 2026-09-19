import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # The bumperbot ran an imu_republisher node here whose entire job was to forward
    # imu/out to the topic the EKF read, unmodified. That is now gone: on hardware
    # roombabot_firmware's imu_bridge publishes imu/data directly, and in simulation
    # the ros_gz bridge is remapped onto the same topic. One fewer process, one fewer
    # serialization, one fewer DDS hop.

    ekf_config = LaunchConfiguration("ekf_config")

    ekf_config_arg = DeclareLaunchArgument(
        "ekf_config",
        default_value=os.path.join(
            get_package_share_directory("roombabot_localization"), "config", "ekf.yaml"
        ),
        description="Full path to the robot_localization ekf yaml to load. Pass "
                    "ekf_simulation.yaml when running in Gazebo.",
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    return LaunchDescription([
        ekf_config_arg,
        robot_localization,
    ])
