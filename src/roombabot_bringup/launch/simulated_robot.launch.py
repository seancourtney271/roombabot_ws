import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

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

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("roombabot_localization"),
                "rviz",
                "global_localization.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("roombabot_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        world_name_arg,
        gazebo,
        controller,
        joystick,
        local_localization,
        localization,
        slam,
        rviz_localization,
        rviz_slam
    ])