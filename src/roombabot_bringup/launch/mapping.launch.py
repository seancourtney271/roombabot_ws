import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Mission: build a map. Map provider = slam_toolbox (+ map_saver, station_recorder),
# with Nav2 alongside so goals can be sent while mapping (the future autonomous
# explorer plugs in here). Run after simulated_robot.launch.py or real_robot.launch.py
# (which supply the EKF). Never run together with localized_navigation.launch.py --
# both publish map -> odom.


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="True for Gazebo, False on the real robot"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="new_map",
        description="Name of the map being built -- station_recorder saves stations.yaml "
                    "under this map's folder"
    )

    use_navigation_arg = DeclareLaunchArgument(
        "use_navigation",
        default_value="True",
        description="Also start Nav2. False for plain teleop mapping"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Open RViz with the SLAM view (use False on the Pi)"
    )

    use_exploration_arg = DeclareLaunchArgument(
        "use_exploration",
        default_value="False",
        description="Autonomously explore frontiers via Nav2 (needs use_navigation:=True)"
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_mapping"),
            "launch",
            "slam.launch.py"
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
        condition=IfCondition(LaunchConfiguration("use_navigation"))
    )

    # Delayed so Nav2's lifecycle manager has activated bt_navigator and the global costmap
    # before the first goal is sent.
    explore = TimerAction(
        period=10.0,
        actions=[Node(
            package="roombabot_mapping",
            executable="frontier_explorer",
            name="frontier_explorer",
            output="screen",
            parameters=[
                os.path.join(
                    get_package_share_directory("roombabot_mapping"),
                    "config",
                    "explore.yaml"
                ),
                {"use_sim_time": use_sim_time}
            ]
        )],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("use_exploration"), "'.lower() == 'true' and '",
            LaunchConfiguration("use_navigation"), "'.lower() == 'true'"
        ]))
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("roombabot_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        use_navigation_arg,
        use_rviz_arg,
        use_exploration_arg,
        slam,
        navigation,
        explore,
        rviz,
    ])
