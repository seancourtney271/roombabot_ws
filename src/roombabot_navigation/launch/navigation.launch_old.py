import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


# Copy of the original launch file that got everything working.

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    planner_config = LaunchConfiguration("planner_config")
    smoother_config = LaunchConfiguration("smoother_config")
    map_name = LaunchConfiguration("map_name")
    use_docking = LaunchConfiguration("use_docking")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    use_docking_arg = DeclareLaunchArgument(
        "use_docking",
        default_value="false",
        description="Whether to run docking_controller. Defaults false: stations are "
                     "taught by driving the robot to them, so coarse navigation already "
                     "ends at the recorded pose and there is no residual error for a "
                     "final approach to remove. Kept as a hook -- see "
                     "roombabot_motion/src/docking_controller.cpp."
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="bedroom",
        description="Name of the saved map being localized against -- mission_commander loads this map's stations.yaml"
    )

    planner_config_arg = DeclareLaunchArgument(
        "planner_config",
        default_value=os.path.join(
            get_package_share_directory("roombabot_navigation"),
            "config",
            "planner_server.yaml"
        ),
        description="Full path to the planner_server yaml file to load"
    )

    smoother_config_arg = DeclareLaunchArgument(
        "smoother_config",
        default_value=os.path.join(
            get_package_share_directory("roombabot_navigation"),
            "config",
            "smoother_server.yaml"
        ),
        description="Full path to the smoother_server yaml file to load"
    )


    costmap_config = LaunchConfiguration("costmap_config")
    costmap_config_arg = DeclareLaunchArgument(
        "costmap_config",
        default_value=os.path.join(
            get_package_share_directory("roombabot_navigation"),
            "config",
            "costmap.yaml"
        ),
        description="Full path to the local costmap yaml file to load"
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            planner_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            smoother_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    local_costmap_node = Node(
        package="roombabot_navigation",
        executable="local_costmap_node",
        name="local_costmap",
        output="screen",
        parameters=[
            costmap_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": ["smoother_server", "planner_server", "local_costmap/local_costmap"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 0.0},
        ],
    )

    pure_pursuit = Node(
        package="roombabot_motion",
        executable="pure_pursuit",
        name="pure_pursuit_planner_node",
        output="screen",
        parameters=[
            {"path_subscriber": "/plan"},
            {"costmap_topic": "/local_costmap/costmap"},
            {"use_sim_time": use_sim_time},
        ],
    )

    waypoint_commander = Node(
        package="roombabot_navigation",
        executable="waypoint_commander",
        name="waypoint_commander",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    mission_commander = Node(
        package="roombabot_motion",
        executable="mission_commander.py",
        name="mission_commander_node",
        output="screen",
        parameters=[
            {"map_name": map_name},
            {"use_docking": ParameterValue(use_docking, value_type=bool)},
            {"use_sim_time": use_sim_time},
        ],
    )

    docking_controller = Node(
        package="roombabot_motion",
        executable="docking_controller",
        name="docking_controller_node",
        output="screen",
        condition=IfCondition(use_docking),
        parameters=[
            {"settle_time": 0.5},
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        planner_config_arg,
        smoother_config_arg,
        costmap_config_arg,
        map_name_arg,
        use_docking_arg,
        planner_server,
        smoother_server,
        local_costmap_node,
        lifecycle_manager,
        pure_pursuit,
        waypoint_commander,
        mission_commander,
        docking_controller,
    ])
