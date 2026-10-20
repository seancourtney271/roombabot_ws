#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace roombabot_motion
{
    // Follows a global path using PD control on the error to a look-ahead point.
    // Same look-ahead selection, costmap safety stop and stuck handling as PurePursuit;
    // the difference is the control law. Instead of commanding a constant speed and
    // a curvature-derived turn rate, it runs two PD loops on the look-ahead point
    // expressed in the robot frame:
    //   angular = kp_angular * heading_error + kd_angular * d(heading_error)/dt
    //   linear  = kp_linear  * distance      + kd_linear  * d(distance)/dt
    // Linear speed is also scaled by cos(heading_error) so the robot turns toward the
    // point before driving at it.
    class PDMotionPlanner : public nav2_core::Controller
    {
        public:
            PDMotionPlanner() = default;
            ~PDMotionPlanner() = default;

            void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, 
                std::string name,
                std::shared_ptr<tf2_ros::Buffer> tf,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

            void cleanup() override;
            void activate() override;
            void deactivate() override;

            geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & robot_pose,
                const geometry_msgs::msg::Twist & velocity,
                nav2_core::GoalChecker * goal_checker) override;

            void setPlan(const nav_msgs::msg::Path & path) override;

            void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
        private:
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub;

            // PD gains
            double kp;
            double kd;
            // double kp_linear;
            // double kd_linear;
            // double kp_angular;
            // double kd_angular;

            //
            double step_size;
            double max_linear_velocity;
            double max_angular_velocity;
            nav_msgs::msg::Path global_plan;

            // Errors
            double previous_angular_error;
            double previous_linear_error;
            rclcpp::Time last_cycle_time;

            std::string plugin_name;
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
            rclcpp::Logger logger{rclcpp::get_logger("PDMotionPlanner")};
            rclcpp::Clock::SharedPtr clock;
            rclcpp_lifecycle::LifecycleNode::WeakPtr _node;
            std::shared_ptr<tf2_ros::Buffer> _tf;

            // Rebuilds working_plan from global_plan in the given frame.
            bool transformPlan(const std::string & frame);

            geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose);
    };
}
