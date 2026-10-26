import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Robot layer only (simulated): Gazebo, controllers, teleop plumbing and the EKF that
# owns odom -> base_footprint. Map provider (AMCL or SLAM) and Nav2 come from
# localized_navigation.launch.py / mapping.launch.py.


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="small_house",
        description="World file in roombabot_description/worlds (without .world)"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name")
        }.items(),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_localization"),
            "launch",
            "local_localization.launch.py"
        ),
        launch_arguments={
            "ekf_config": os.path.join(
                get_package_share_directory("roombabot_localization"),
                "config",
                "ekf_simulation.yaml"
            ),
        }.items(),
    )

    return LaunchDescription([
        world_name_arg,
        gazebo,
        controller,
        joystick,
        local_localization,
    ])
