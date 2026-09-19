#include "roombabot_navigation/waypoint_commander.hpp"

namespace roombabot_navigation
{
// Bridges RViz/Foxglove's "2D Goal Pose" tool to the planner_server's
// compute_path_to_pose action, then republishes the resulting path so
// pure_pursuit (which only knows how to follow a nav_msgs/Path topic,
// not call an action) can drive to it.
WaypointCommander::WaypointCommander() : Node("waypoint_commander")
{
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_base_frame", "base_footprint");
    declare_parameter<std::string>("planner_id", "GridBased");

    global_frame_ = get_parameter("global_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    planner_id_ = get_parameter("planner_id").as_string();

    action_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");

    // transient_local so Foxglove/RViz clients that subscribe after a path was
    // already computed (e.g. opening the panel mid-drive) still see it immediately,
    // matching the same durability convention already used for /map.
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/plan", rclcpp::QoS(1).transient_local());

    // Foxglove's "Publish"/goal-click tool defaults to this topic name
    // (carried over from classic ROS1 move_base/RViz conventions).
    goal_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&WaypointCommander::goalCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "Waypoint commander ready. Publish a goal to /goal_pose to navigate.");
}

void WaypointCommander::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(get_logger(), "compute_path_to_pose action server not available. Is planner_server running?");
        return;
    }

    geometry_msgs::msg::TransformStamped robot_tf;
    try
    {
        robot_tf = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not get robot pose in %s frame: %s", global_frame_.c_str(), ex.what());
        return;
    }

    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = global_frame_;
    // Leave stamp at zero (rclcpp::Time() default) to mean "latest available transform",
    // matching tf2::TimePointZero convention used elsewhere in this codebase. Stamping
    // this with now() caused planner_server to look up base_link->map at an exact
    // timestamp that TF publishing (often lagging under this Pi's CPU load) hadn't
    // reached yet, producing an "extrapolation into the future" error.
    start.pose.position.x = robot_tf.transform.translation.x;
    start.pose.position.y = robot_tf.transform.translation.y;
    start.pose.orientation = robot_tf.transform.rotation;

    ComputePathToPose::Goal action_goal;
    action_goal.start = start;
    action_goal.goal = *goal;
    action_goal.planner_id = planner_id_;

    auto options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    options.result_callback = std::bind(&WaypointCommander::resultCallback, this, std::placeholders::_1);
    action_client_->async_send_goal(action_goal, options);

    RCLCPP_INFO(get_logger(), "Requesting path to (%.2f, %.2f)", goal->pose.position.x, goal->pose.position.y);
}

void WaypointCommander::resultCallback(const GoalHandle::WrappedResult & result)
{
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_ERROR(get_logger(), "compute_path_to_pose did not succeed");
        return;
    }

    path_publisher_->publish(result.result->path);
    RCLCPP_INFO(get_logger(), "Path computed with %zu poses, publishing to /plan",
                result.result->path.poses.size());
}
}  // namespace roombabot_navigation

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<roombabot_navigation::WaypointCommander>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
