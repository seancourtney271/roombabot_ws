import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Mission: navigate on a saved map. Map provider = map_server + AMCL, then Nav2.
# Run after simulated_robot.launch.py or real_robot.launch.py (which supply the EKF).
# Never run together with mapping.launch.py -- both publish map -> odom.


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="True for Gazebo, False on the real robot"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Map in roombabot_mapping/maps to localize and navigate against"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Open RViz with the navigation view (use False on the Pi)"
    )

    global_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "map_name": LaunchConfiguration("map_name"),
            "use_sim_time": use_sim_time
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("roombabot_localization"),
                "rviz",
                "nav2_setup.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        use_rviz_arg,
        global_localization,
        navigation,
        rviz,
    ])
