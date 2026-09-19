import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")
    map_name = LaunchConfiguration("map_name")

    ros_distro = os.environ["ROS_DISTRO"]
    lifecycle_nodes = ["map_saver_server"]
    if ros_distro != "humble":
        lifecycle_nodes.append("slam_toolbox")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("roombabot_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam yaml file to load"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="bedroom",
        description="Name of the map being built -- station_recorder saves stations.yaml under this map's folder"
    )

    # Teach-by-driving: park the robot at a station, call record_station with its id,
    # repeat, then call save_stations. There is nothing to detect and nothing to
    # configure per marker any more.
    station_recorder = Node(
        package="roombabot_mapping",
        executable="station_recorder.py",
        name="station_recorder_node",
        output="screen",
        parameters=[
            {"map_name": map_name},
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65},
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 0.0}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        map_name_arg,
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager,
        station_recorder,
    ])
