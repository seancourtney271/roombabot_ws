#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace roombabot_planning
{
    struct GraphNode
    {
        int x = 0;
        int y = 0;
        int cost = 0;
        double heuristic;
        std::shared_ptr<GraphNode> prev;

        GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0)
        {
        }

        GraphNode() : GraphNode(0, 0)
        {
        }

        bool operator>(const GraphNode & other) const
        {
            return (cost + heuristic) > (other.cost + other.heuristic);
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

    class AStarPlanner : public rclcpp::Node
    {
        public:
            /* This class will be a Dijkstra Planner
                That is a priority queue that takes in a node and gets its children. And sets the nodes priority based off its cost to traverse from the original node

                Start -2- Node1 -3- Node2 -4- Final Node
                Start cost is 0
                Node1 from start will be 2
                Node2 from start will be 5
                Final Node will be 9
            */
           AStarPlanner();

        private:
            // Map subscriber
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
            // Goal Pose Subscriber
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber;

            // Path to goal publisher
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
            // Visited Map Publisher
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr visited_map_pubilsher;

            // Current environment map
            nav_msgs::msg::OccupancyGrid::SharedPtr _map;
            // Visited Map
            nav_msgs::msg::OccupancyGrid visited_map;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener;
            std::unique_ptr<tf2_ros::Buffer> tf_buffer;
            
            // Map recieved
            void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
            // Goal recieved
            void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

            double manhattanDistance(const GraphNode & node, const GraphNode & goal_node);
            double euclideanDistance(const GraphNode & node, const GraphNode & goal_node);
            
            GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
            geometry_msgs::msg::Pose gridToWorld(GraphNode & node);
            bool poseOnMap(const GraphNode & node);
            unsigned int poseToCell(const GraphNode & node);

            // Run the dijksrta algorithm
            nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);
    };
}