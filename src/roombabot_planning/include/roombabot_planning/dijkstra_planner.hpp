#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace roombabot_planning
{
    struct GraphNode
    {
        int x = 0;
        int y = 0;
        // Fractional so diagonal steps (~1.41) can be weighted differently than
        // orthogonal ones (1.0) -- see explore_directions in dijkstra_planner.cpp.
        double cost = 0.0;
        std::shared_ptr<GraphNode> prev;

        GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0.0)
        {
        }

        GraphNode() : GraphNode(0, 0)
        {
        }

        bool operator>(const GraphNode & other) const
        {
            return cost > other.cost;
        }

        bool operator==(const GraphNode & other) const
        {
            return x == other.x && y == other.y;
        }

        GraphNode operator+(std::pair<int, int> const & other)
        {
            GraphNode ret(x + other.first, y + other.second);
            return ret;
        }
    };
    /* This class will be a Dijkstra Planner
        That is a priority queue that takes in a node and gets its children. And sets the nodes priority based off its cost to traverse from the original node

        Start -2- Node1 -3- Node2 -4- Final Node
        Start cost is 0
        Node1 from start will be 2
        Node2 from start will be 5
        Final Node will be 9
    */
    class DijkstraPlanner : public nav2_core::GlobalPlanner
    {
        public:
            void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

            DijkstraPlanner() = default;
           ~DijkstraPlanner() = default;

            void cleanup() override;
            void activate() override;
            void deactivate() override;

            nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancel_checker) override;

        private:    
            std::shared_ptr<tf2_ros::Buffer> tf_;        
            nav2_util::LifecycleNode::SharedPtr node_;
            nav2_costmap_2d::Costmap2D * costmap_;
            std::string global_frame_, name_;
            // Multiplies each cell's inflation cost before adding it into the path's
            // total cost. At 1.0 (the old implicit behavior) a moderately-inflated
            // cell near an edge is cheap enough that the shortest route past it beats
            // a longer detour through open space, so the planner hugs boundaries
            // whenever that's the shortest path. Raising this makes proximity to any
            // obstacle/edge cost much more, so Dijkstra prefers standing off from
            // edges whenever there's room to -- it does not change which cells are
            // passable at all (that's still the separate cost < 99 threshold), so
            // genuinely narrow corridors remain traversable.
            double cost_penalty_weight_;
            // Smoother client init
            rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_client;

            // Map recieved
            void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
            // Goal recieved
            void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

            GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
            geometry_msgs::msg::Pose gridToWorld(GraphNode & node);
            bool poseOnMap(const GraphNode & node);
            unsigned int poseToCell(const GraphNode & node);
    };
}