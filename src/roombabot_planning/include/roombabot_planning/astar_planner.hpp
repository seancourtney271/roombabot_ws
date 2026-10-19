#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <limits>
#include <algorithm>

namespace roombabot_planning
{
    struct GraphNode
    {
        int x = 0;
        int y = 0;
        // Fractional so diagonal steps (~1.41) can be weighted differently than
        // orthogonal ones (1.0) -- see explore_directions in astar_planner.cpp.
        double cost = 0;
        double heuristic = 0;
        std::shared_ptr<GraphNode> prev;

        GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0)
        {
        }

        GraphNode() : GraphNode(0, 0)
        {
        }

        bool operator>(const GraphNode & other) const
        {
            // Operator > override to compare the cost + heuristic
            return (cost + heuristic) > (other.cost + other.heuristic);
        }

        bool operator==(const GraphNode & other) const
        {
            // Operator == override to compare both x and y components
            return x == other.x && y == other.y;
        }

        GraphNode operator+(std::pair<int, int> const & other)
        {
            // Operator + override to compare both x and y components
            GraphNode ret(x + other.first, y + other.second);
            return ret;
        }
    };

    class AStarPlanner : public nav2_core::GlobalPlanner
    {
        public:
            /* This class will be a Astar Planner
                That is a priority queue that takes in a node and gets its children. 
                And sets the nodes priority based off its cost and hueristic to traverse from the original node
            */
            void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
           
            AStarPlanner() = default;
            ~AStarPlanner() = default;

            void cleanup() override;
            void activate() override;
            void deactivate() override;

            nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancel_checker) override;

        private:
            std::shared_ptr<tf2_ros::Buffer> tf_;        
            nav2_util::LifecycleNode::SharedPtr node_;
            nav2_costmap_2d::Costmap2D * costmap_;
            std::string global_frame_, name_;

            double cost_penalty_weight_;

            GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
            geometry_msgs::msg::Pose gridToWorld(GraphNode & node);
            bool poseOnMap(const GraphNode & node);
            unsigned int poseToCell(const GraphNode & node);

            double euclideanDistance(const GraphNode & node, const GraphNode & goal_node);
    };
}
