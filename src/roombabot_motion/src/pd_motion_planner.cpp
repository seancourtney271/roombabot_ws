#include <algorithm>
#include <cmath>
#include "roombabot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace roombabot_motion
{
    void PDMotionPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, 
                std::string name,
                std::shared_ptr<tf2_ros::Buffer> tf,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
                    _node = parent;
                    auto node = _node.lock();
                    _costmap_ros = costmap_ros;
                    _tf = tf;
                    plugin_name = name;
                    logger = node->get_logger();
                    clock = node->get_clock();

                    next_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

                    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".kp", rclcpp::ParameterValue(2.0));
                    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".kd", rclcpp::ParameterValue(0.1));
                    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".max_linear_velocity", rclcpp::ParameterValue(0.3));
                    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
                    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".step_size", rclcpp::ParameterValue(0.2));

                    node->get_parameter(plugin_name + ".kp", kp);
                    node->get_parameter(plugin_name + ".kd", kd);
                    node->get_parameter(plugin_name + ".max_linear_velocity", max_linear_velocity);
                    node->get_parameter(plugin_name + ".max_angular_velocity", max_angular_velocity);
                    node->get_parameter(plugin_name + ".step_size", step_size);
                }

    void PDMotionPlanner::cleanup(){
        RCLCPP_INFO(logger, "Cleaning up plugin PDMotionPlanner");
        next_pose_pub.reset();
    }
    void PDMotionPlanner::activate(){
        RCLCPP_INFO(logger, "Activating plugin PDMotionPlanner");
        last_cycle_time = clock->now();
    }
    void PDMotionPlanner::deactivate(){
        RCLCPP_INFO(logger, "Deactivating plugin PDMotionPlanner");
    }

    void PDMotionPlanner::setPlan(const nav_msgs::msg::Path & path){
        global_plan = path;
    }

    // Transform the stored plan into the requested coordinate frame.
    // This rewrites each pose so the global path is expressed relative to the robot's current frame.
    bool PDMotionPlanner::transformPlan(const std::string & frame)
    {
        if(global_plan.header.frame_id == frame)
        {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = _tf->lookupTransform(frame, global_plan.header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::LookupException & ex)
        {
            RCLCPP_ERROR_STREAM(logger, "Couldn't transform plan from frame " << global_plan.header.frame_id << " to " << frame);
            return false;
        }

        for(auto & pose : global_plan.poses)
        {
            // Transform each path pose into the target frame.
            tf2::doTransform(pose, pose, transform);
        }

        global_plan.header.frame_id = frame;
        return true;
    }

    geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
    {
        geometry_msgs::msg::PoseStamped next_pose = global_plan.poses.back();
        for(auto pose_it = global_plan.poses.rbegin(); pose_it != global_plan.poses.rend(); ++pose_it){
            double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
            double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            if(distance > step_size)
            {
                next_pose = *pose_it;
            }
            else{
                break;
            }
        }
        return next_pose;
    }
    
    void PDMotionPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage){}

    geometry_msgs::msg::TwistStamped PDMotionPlanner::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & robot_pose,
                const geometry_msgs::msg::Twist &,
                nav2_core::GoalChecker *){
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = robot_pose.header.frame_id;

        if(global_plan.poses.empty()){
            RCLCPP_ERROR(logger, "Empty Plan.");
            return cmd_vel;
        }
        // Convert the received global plan into the robot's current frame so a controller
        // can compute commands relative to the robot's position and orientation.
        if(!transformPlan(robot_pose.header.frame_id))
        {
            RCLCPP_ERROR(logger, "Unable to transform Plan into robot's frame.");
            return cmd_vel;
        }
        auto next_pose = getNextPose(robot_pose);

        // Publish look ahead pose
        next_pose_pub->publish(next_pose);

        // Get the error of look ahead pose and robot pose
        tf2::Transform robot_tf, next_pose_tf, next_pose_robot_tf;
        tf2::fromMsg(robot_pose.pose, robot_tf);
        tf2::fromMsg(next_pose.pose, next_pose_tf);

        // Gets the error of the pose of the robot and the look ahead pose we want to reach
        next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;
        double linear_error = next_pose_robot_tf.getOrigin().getX();
        double angular_error = next_pose_robot_tf.getOrigin().getY();

        double dt = (clock->now() - last_cycle_time).seconds();
        double linear_error_derivative = (linear_error - previous_linear_error) / dt;
        double angular_error_derivative = (angular_error - previous_angular_error) / dt;

        cmd_vel.twist.linear.x = std::clamp(kp * linear_error + kd * linear_error_derivative, -max_linear_velocity, max_linear_velocity);
        cmd_vel.twist.angular.z = std::clamp(kp * angular_error + kd * angular_error_derivative, -max_angular_velocity, max_angular_velocity);
        
        last_cycle_time = clock->now();
        previous_linear_error = linear_error;
        previous_angular_error = angular_error;

        cmd_vel.header.stamp = clock->now();
        return cmd_vel;
    }
}

PLUGINLIB_EXPORT_CLASS(roombabot_motion::PDMotionPlanner, nav2_core::Controller)
