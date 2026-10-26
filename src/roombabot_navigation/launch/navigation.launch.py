import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Create use sim time arguement
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Map in roombabot_mapping/maps to localize and navigate against"
    )
    map_name = LaunchConfiguration("map_name")

    use_localization_arg = DeclareLaunchArgument(
        "use_localization",
        default_value="True",
        description="Also start the EKF, map_server and AMCL"
    )
    use_localization = LaunchConfiguration("use_localization")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Open RViz with the localization/navigation view"
    )

    ekf_config_arg = DeclareLaunchArgument(
        "ekf_config",
        default_value=PathJoinSubstitution([
            get_package_share_directory("roombabot_localization"),
            "config",
            PythonExpression([
                "'ekf_simulation.yaml' if '", use_sim_time, "'.lower() == 'true' else 'ekf.yaml'"
            ])
        ]),
        description="robot_localization yaml. Defaults to ekf_simulation.yaml when "
                    "use_sim_time is true, ekf.yaml otherwise"
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_localization"),
            "launch",
            "local_localization.launch.py"
        ),
        launch_arguments={
            "ekf_config": LaunchConfiguration("ekf_config")
        }.items(),
        condition=IfCondition(use_localization)
    )

    global_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("roombabot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": use_sim_time
        }.items(),
        condition=IfCondition(use_localization)
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

    # smoother first: the planner plugin waits for the smooth_path action server while configuring
    lifecycle_nodes = ["smoother_server", "controller_server", "planner_server", "bt_navigator"]

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name = "controller_server",
        output="screen",
        parameters=[os.path.join(
            get_package_share_directory("roombabot_navigation"),
            "config",
            "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name = "planner_server",
        output="screen",
        parameters=[os.path.join(
            get_package_share_directory("roombabot_navigation"),
            "config",
            "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name = "smoother_server",
        output="screen",
        parameters=[os.path.join(
            get_package_share_directory("roombabot_navigation"),
            "config",
            "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_bt_navigator = Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name = "bt_navigator",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("roombabot_navigation"),
                "config",
                "bt_navigator.yaml"),
                {"use_sim_time": use_sim_time}
            ]
        )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        use_localization_arg,
        use_rviz_arg,
        ekf_config_arg,
        local_localization,
        global_localization,
        rviz,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
    ])