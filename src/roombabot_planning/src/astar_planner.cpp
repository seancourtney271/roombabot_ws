#include "roombabot_planning/astar_planner.hpp"
#include "rmw/qos_profiles.h"
#include <queue>

namespace roombabot_planning
{
    AStarPlanner::AStarPlanner() : Node("astar_node")
    {
        // Object used to grab the current pose of the robot
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        // 
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        //Map Quality of Service
        rclcpp::QoS map_qos(10);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", map_qos, std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));
        goal_pose_subscriber = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&AStarPlanner::goalCallback, this, std::placeholders::_1));

        path_publisher = create_publisher<nav_msgs::msg::Path>("/astar/path", 10);
        visited_map_pubilsher = create_publisher<nav_msgs::msg::OccupancyGrid>("/astar/visited_map", 10);
    }

    // Map recieved
    void AStarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        //Current update to map
        _map = map;
        // Configure the visited map incase it updated
        visited_map.header.frame_id = map->header.frame_id;    
        visited_map.info = map->info;
        // Calculate size of vector for visited map and populate with -1
        visited_map.data = std::vector<int8_t>(visited_map.info.height * visited_map.info.width, -1);
    }
    // Goal recieved
    void AStarPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
        RCLCPP_INFO(get_logger(), "Goal Pose Recieved.");
        if(!_map){
            RCLCPP_ERROR(get_logger(), "No map recieved!");
            return;
        }
        visited_map.data = std::vector<int8_t>(visited_map.info.height * visited_map.info.width, -1);
        geometry_msgs::msg::TransformStamped map_to_base_tf;
        try{
            map_to_base_tf = tf_buffer->lookupTransform(_map->header.frame_id, "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex){
            RCLCPP_ERROR(get_logger(), "Could not transform from map to base_footprint");
        }

        geometry_msgs::msg::Pose map_to_base_pose;
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

        auto path = plan(map_to_base_pose, pose->pose);
        if(!path.poses.empty()){
            RCLCPP_INFO(get_logger(), "Shortest path found");
            path_publisher->publish(path);
        } else {
            RCLCPP_WARN(get_logger(), "No path found to the goal.");
        }

    }

    double AStarPlanner::manhattanDistance(const GraphNode & node, const GraphNode & goal_node)
    {
        return (abs(node.x - goal_node.x) + abs(node.y - goal_node.y));
    }
    double AStarPlanner::euclideanDistance(const GraphNode & node, const GraphNode & goal_node)
    {
        return sqrt(((node.x - goal_node.x) * (node.x - goal_node.x)) + ((node.y - goal_node.y) * (node.y - goal_node.y)));
    }

    // Run the dijksrta algorithm
    nav_msgs::msg::Path AStarPlanner::plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal)
    {
        //Potential Directions to explore
        std::vector<std::pair<int, int>> explore_directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

        // Nodes not yet processed
        std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
        // Nodes that are processed
        std::vector<GraphNode> visited_nodes;

        GraphNode start_node = worldToGrid(start);
        GraphNode goal_node = worldToGrid(goal);
        start_node.heuristic = manhattanDistance(start_node, goal_node);
        pending_nodes.push(worldToGrid(start));
        GraphNode active_node;
        while(!pending_nodes.empty() && rclcpp::ok())
        {
            active_node = pending_nodes.top();
            pending_nodes.pop();

            if(worldToGrid(goal) == active_node)
            {
                break;
            }

            for(const auto & dir : explore_directions)
            {
                // Get Neighbor
                GraphNode new_node = active_node + dir;
                // Check if already visited and is within the map and the new nodes location exists
                if((std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end()) && poseOnMap(new_node) && 
                // Cells that are less than 99 and >= 0 for being navigable cells
                _map->data.at(poseToCell(new_node)) < 99 && _map->data.at(poseToCell(new_node)) >= 0)
                {
                    // Calculate Node cost
                    new_node.cost = active_node.cost + 1 + _map->data.at(poseToCell(new_node));
                    new_node.heuristic = manhattanDistance(new_node, goal_node);
                    // Assigned previous node
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    pending_nodes.push(new_node);
                    visited_nodes.push_back(new_node);
                }
            }

            // For Visualizing active node in RVIZ
            visited_map.data.at(poseToCell(active_node)) = -106;
            visited_map_pubilsher->publish(visited_map);
        }

        // Exploration finished goal mode reached or no solution found
        // Make path to goal
        nav_msgs::msg::Path path;
        path.header.frame_id = _map->header.frame_id;
        // Reconstruct path to goal backwards
        while(active_node.prev && rclcpp::ok())
        {
            geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
            geometry_msgs::msg::PoseStamped last_pose_stamped;
            last_pose_stamped.header.frame_id = _map->header.frame_id;
            last_pose_stamped.pose = last_pose;
            path.poses.push_back(last_pose_stamped);
            // Move to next node until start node is reached
            active_node = *active_node.prev;
        }

        std::reverse(path.poses.begin(), path.poses.end());
        return path;

    }

    GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
    {
        int grid_x = static_cast<int>((pose.position.x - _map->info.origin.position.x) / _map->info.resolution);
        int grid_y = static_cast<int>((pose.position.y - _map->info.origin.position.y) / _map->info.resolution);
        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose AStarPlanner::gridToWorld(GraphNode & node)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * _map->info.resolution + _map->info.origin.position.x;
        pose.position.y = node.y * _map->info.resolution + _map->info.origin.position.y;
        return pose;
    }

    bool AStarPlanner::poseOnMap(const GraphNode & node)
    {
        return node.x >= 0 && node.x < static_cast<int>(_map->info.width) && node.y >= 0 && node.y < static_cast<int>(_map->info.height);
    }

    unsigned int AStarPlanner::poseToCell(const GraphNode & node)
    {
        return node.y * _map->info.width + node.x;
    }
}

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<roombabot_planning::AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}