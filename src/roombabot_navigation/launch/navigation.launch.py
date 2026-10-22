import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Create use sim time arguement
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # smoother first: the planner plugin waits for the smooth_path action server while configuring
    lifecycle_nodes = ["smoother_server", "controller_server", "planner_server"]

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
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_lifecycle_manager,
    ])