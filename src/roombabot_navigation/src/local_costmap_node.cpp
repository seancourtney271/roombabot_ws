#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// Standalone runner for a nav2_costmap_2d::Costmap2DROS instance. Unlike the
// global costmap (owned internally by planner_server for path planning), this
// one runs as its own lifecycle node so pure_pursuit can subscribe to it for
// real-time obstacle checking while driving. Costmap2DROS is itself a
// LifecycleNode, so lifecycle transitions are handled externally by a
// nav2_lifecycle_manager, same as every other managed node in this stack.
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto costmap_node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(costmap_node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
