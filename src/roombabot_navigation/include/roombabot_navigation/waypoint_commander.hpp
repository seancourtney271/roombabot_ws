#ifndef ROOMBABOT_NAVIGATION_WAYPOINT_COMMANDER_HPP
#define ROOMBABOT_NAVIGATION_WAYPOINT_COMMANDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace roombabot_navigation
{
class WaypointCommander : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    WaypointCommander();

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
    void resultCallback(const GoalHandle::WrappedResult & result);

    rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string global_frame_;
    std::string robot_base_frame_;
    std::string planner_id_;
};
}  // namespace roombabot_navigation

#endif  // ROOMBABOT_NAVIGATION_WAYPOINT_COMMANDER_HPP
