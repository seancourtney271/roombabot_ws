#include "roombabot_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"
#include "nav2_util/node_utils.hpp"
#include <queue>

namespace roombabot_planning
{
    void DijkstraPlanner::configure(
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

        smooth_client = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "smooth_path");
        if(!smooth_client->wait_for_action_server(std::chrono::seconds(3))){
            RCLCPP_ERROR(node_->get_logger(), "Action Server not available after waiting");
        }
    }

    void DijkstraPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type DijkstraPlanner", name_.c_str());
    }
    void DijkstraPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type DijkstraPlanner", name_.c_str());
    }
    void DijkstraPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type DijkstraPlanner", name_.c_str());
    }

    nav_msgs::msg::Path DijkstraPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker)
    {
        //Potential Directions to explore, paired with their per-step cost. Orthogonal-only
        // search made every path "staircase" toward diagonal goals (a run of 90-degree grid
        // steps), which fed pure_pursuit a stream of large heading errors and made it stop
        // and rotate in place constantly. Diagonal moves cost sqrt(2) instead of 1 (their
        // true relative distance), so Dijkstra still finds the genuinely shortest path --
        // it just now has a smooth diagonal option instead of only zigzagging.
        static constexpr double kDiagonalCost = 1.4142135623730951;
        std::vector<std::tuple<int, int, double>> explore_directions = {
            {-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0},
            {-1, -1, kDiagonalCost}, {-1, 1, kDiagonalCost}, {1, -1, kDiagonalCost}, {1, 1, kDiagonalCost}
        };

        // Nodes not yet processed
        std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
        // Nodes that are processed. A flat per-cell flag gives an O(1) membership
        // check -- the previous std::vector<GraphNode> + std::find was O(V) per
        // check, so on the full-size global costmap (hundreds of thousands of
        // cells) the search degenerated to roughly O(V^2), taking so long that
        // nav2's action server would try (and fail) to cancel/preempt it long
        // before createPlan ever returned.
        std::vector<bool> visited_cells(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY(), false);

        pending_nodes.push(worldToGrid(start.pose));
        GraphNode active_node;
        std::size_t iterations_since_cancel_check = 0;
        while(!pending_nodes.empty() && rclcpp::ok())
        {
            // Checking every iteration would add lock/condvar overhead per node
            // expanded; checking periodically still lets a cancelled/superseded
            // goal bail out quickly instead of running the search to exhaustion.
            if(++iterations_since_cancel_check >= 200)
            {
                iterations_since_cancel_check = 0;
                if(cancel_checker())
                {
                    RCLCPP_INFO(node_->get_logger(), "DijkstraPlanner: goal cancelled/superseded, aborting search");
                    return nav_msgs::msg::Path();
                }
            }

            active_node = pending_nodes.top();
            pending_nodes.pop();

            if(worldToGrid(goal.pose) == active_node)
            {
                break;
            }

            for(const auto & [dx, dy, step_cost] : explore_directions)
            {
                // Get Neighbor
                GraphNode new_node = active_node + std::make_pair(dx, dy);
                // Check if within the map, not already visited, and the new nodes location exists
                if(poseOnMap(new_node) && !visited_cells[poseToCell(new_node)] &&
                // Cells that are less than 99 and >= 0 for being ok cells
                costmap_->getCost(new_node.x, new_node.y) < 99)
                {
                    // Calculate Node cost. The inflation term is weighted so proximity
                    // to an edge/obstacle costs much more than a step of raw distance,
                    // making the search prefer standing off from boundaries whenever
                    // there's room to, rather than always taking the shortest route.
                    new_node.cost = active_node.cost + step_cost + cost_penalty_weight_ * costmap_->getCost(new_node.x, new_node.y);
                    // Assigned previous node
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    pending_nodes.push(new_node);
                    visited_cells[poseToCell(new_node)] = true;
                }
            }
        }

        // Exploration finished: either the goal was reached, or the queue ran dry
        // without ever reaching it (goal unreachable/blocked/inflated-over). Only
        // reconstruct a path in the former case, otherwise hand back empty so
        // callers get a clear planning failure instead of a path to the wrong place.
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;

        if(!(worldToGrid(goal.pose) == active_node))
        {
            RCLCPP_WARN(node_->get_logger(), "DijkstraPlanner: exhausted search without reaching the goal");
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

        // Initialize smoother
        nav2_msgs::action::SmoothPath::Goal path_smooth;
        path_smooth.path = path;
        path_smooth.check_for_collisions = false;
        path_smooth.smoother_id = "simple_smoother";
        path_smooth.max_smoothing_duration.sec = 10;
        // Start smoothing
        auto future = smooth_client->async_send_goal(path_smooth);
        // Check if smoothing is finished
        if(future.wait_for(std::chrono::seconds(3)) == std::future_status::ready){
            // If finished check if it worked
            auto goal_handle = future.get();
            if(goal_handle){
                // Get async result of the goal
                auto result_future = smooth_client->async_get_result(goal_handle);
                if(result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready){
                    // Check if path succeeded
                    auto result_path = result_future.get();
                    if(result_path.code == rclcpp_action::ResultCode::SUCCEEDED){
                        // If path smoothing completed set back to path
                        path = result_path.result->path;
                    }
                }
            }
        }
        return path;
    }

    GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
    {
        int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
        int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(GraphNode & node)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
        pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
        return pose;
    }

    bool DijkstraPlanner::poseOnMap(const GraphNode & node)
    {
        return node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY());
    }

    unsigned int DijkstraPlanner::poseToCell(const GraphNode & node)
    {
        return node.y * costmap_->getSizeInCellsX() + node.x;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roombabot_planning::DijkstraPlanner, nav2_core::GlobalPlanner)