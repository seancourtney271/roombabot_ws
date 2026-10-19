#include "roombabot_planning/astar_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <queue>

namespace roombabot_planning
{
    void AStarPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".cost_penalty_weight", rclcpp::ParameterValue(5.0));
        node_->get_parameter(name_ + ".cost_penalty_weight", cost_penalty_weight_);
    }

    void AStarPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type AStarPlanner", name_.c_str());
    }
    void AStarPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type AStarPlanner", name_.c_str());
    }
    void AStarPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type AStarPlanner", name_.c_str());
    }

    GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
    {
        int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / (costmap_->getResolution()));
        int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / (costmap_->getResolution()));
        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose AStarPlanner::gridToWorld(GraphNode & node)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = (node.x + 0.5) * costmap_->getResolution() + costmap_->getOriginX();
        pose.position.y = (node.y + 0.5) * costmap_->getResolution() + costmap_->getOriginY();
        pose.orientation.w = 1.0;
        return pose;
    }

    bool AStarPlanner::poseOnMap(const GraphNode & node)
    {
        return node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY());
    }

    unsigned int AStarPlanner::poseToCell(const GraphNode & node)
    {
        return node.y * costmap_->getSizeInCellsX() + node.x;
    }

    double AStarPlanner::euclideanDistance(const GraphNode & node, const GraphNode & goal_node)
    {
        return sqrt(((node.x - goal_node.x) * (node.x - goal_node.x)) + ((node.y - goal_node.y) * (node.y - goal_node.y)));
    }

    // Create A Star Path Plan
    nav_msgs::msg::Path AStarPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker)
    {
        static constexpr double kDiagonalCost = 1.4142135623730951;
        std::vector<std::tuple<int, int, double>> explore_directions = {
            {-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0},
            {-1, -1, kDiagonalCost}, {-1, 1, kDiagonalCost}, {1, -1, kDiagonalCost}, {1, 1, kDiagonalCost}
        };

        const auto n_cells = costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY();
        // Load initial start node
        GraphNode start_node = worldToGrid(start.pose);
        GraphNode goal_node = worldToGrid(goal.pose);
        std::vector<double> best_cost(n_cells, std::numeric_limits<double>::infinity());  // <limits>
        std::vector<bool> closed(n_cells, false);
        if (!poseOnMap(start_node) || !poseOnMap(goal_node) ||
        costmap_->getCost(goal_node.x, goal_node.y) >= 99)
        {
            RCLCPP_WARN(node_->get_logger(), "AStarPlanner: start or goal is off the map or in an obstacle");
            return nav_msgs::msg::Path();
        }
        // Nodes not yet processed
        std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;

        start_node.heuristic = euclideanDistance(start_node, goal_node);
        GraphNode active_node;
        std::size_t iterations_since_cancel_check = 0;

        best_cost[poseToCell(start_node)] = 0.0;
        pending_nodes.push(start_node);
        while(!pending_nodes.empty() && rclcpp::ok())
        {
            // Check if the Path Planning was cancelled every 200 nodes
            if(++iterations_since_cancel_check >= 200)
            {
                iterations_since_cancel_check = 0;
                if(cancel_checker())
                {
                    RCLCPP_INFO(node_->get_logger(), "AStarPlanner: goal cancelled/superseded, aborting search");
                    return nav_msgs::msg::Path();
                }
            }

            // Grab the next node to process
            active_node = pending_nodes.top();
            pending_nodes.pop();

            // The same cell can be in the queue several times (each time a cheaper route was
            // found). Only the first pop is the cheapest; later copies are stale.
            if (closed[poseToCell(active_node)]) continue;
            closed[poseToCell(active_node)] = true;

            if (active_node == goal_node) break;

            for (const auto & [dx, dy, step_cost] : explore_directions)
            {
                GraphNode new_node = active_node + std::make_pair(dx, dy);
                if (!poseOnMap(new_node) || closed[poseToCell(new_node)] ||
                    costmap_->getCost(new_node.x, new_node.y) >= 99) continue;

                double new_cost = active_node.cost + step_cost
                                + cost_penalty_weight_ * costmap_->getCost(new_node.x, new_node.y);
                if (new_cost >= best_cost[poseToCell(new_node)]) continue;   // not an improvement

                best_cost[poseToCell(new_node)] = new_cost;
                new_node.cost = new_cost;
                new_node.heuristic = euclideanDistance(new_node, goal_node);
                new_node.prev = std::make_shared<GraphNode>(active_node);
                pending_nodes.push(new_node);
            }
        }
        // Exploration finished: either the goal was reached, or the queue ran dry
        // without ever reaching it (goal unreachable/blocked/inflated-over). Only
        // reconstruct a path in the former case, otherwise hand back empty so
        // callers get a clear planning failure instead of a path to the wrong place.
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = node_->now();

        if(!(goal_node == active_node))
        {
            RCLCPP_WARN(node_->get_logger(), "AStarPlanner: exhausted search without reaching the goal");
            return path;
        }
        // Reconstruct path to goal backwards
        while(active_node.prev && rclcpp::ok())
        {
            geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
            geometry_msgs::msg::PoseStamped last_pose_stamped;
            last_pose_stamped.header.frame_id = global_frame_;
            last_pose_stamped.pose = last_pose;
            path.poses.push_back(last_pose_stamped);
            // Move to next node until start node is reached
            active_node = *active_node.prev;
        }

        std::reverse(path.poses.begin(), path.poses.end());
        return path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roombabot_planning::AStarPlanner, nav2_core::GlobalPlanner)
