#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace roombabot_motion
{
    // Nav2 controller plugin version of the PurePursuit node. Same behavior: carrot
    // selection that refuses chords cutting across obstacles, rotate-in-place when the
    // carrot is far off-heading, a stop when the carrot is blocked, and a give-up after
    // stuck_timeout. The differences are that the plan comes from controller_server's
    // setPlan() instead of a /plan subscription, obstacle checks read the controller's
    // own local costmap instead of a costmap topic, and goal arrival is decided by Nav2's
    // goal checker instead of the plugin.
    class PurePursuitController : public nav2_core::Controller
    {
        public:
            PurePursuitController() = default;
            ~PurePursuitController() override = default;

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
            rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pose_pub;

            double look_ahead_distance;
            double maximum_linear_velocity;
            double maximum_angular_velocity;
            // Speed limit most recently requested through setSpeedLimit (percentage of the
            // configured maximum, or absolute m/s). Applied on top of the configured max.
            double linear_velocity_limit;
            // Above this heading error (radians) rotate in place instead of curvature-driving.
            double heading_error_threshold;
            // Raw costmap cost (0-255) at/above which a cell counts as blocked. 253 is
            // inscribed-inflated obstacle, i.e. essentially only true obstacle cells.
            int occupied_threshold;
            double stuck_timeout;

            // Rotation direction (+1/-1) held while rotating in place, 0 otherwise.
            int locked_rotation_sign;
            std::size_t current_plan_index;
            rclcpp::Time blocked_start_time;
            bool is_currently_blocked;

            nav_msgs::msg::Path global_plan;
            // global_plan transformed into the robot pose's frame, rebuilt every cycle.
            nav_msgs::msg::Path working_plan;

            std::string plugin_name;
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;
            std::shared_ptr<tf2_ros::Buffer> tf;
            rclcpp::Logger logger{rclcpp::get_logger("PurePursuitController")};
            rclcpp::Clock::SharedPtr clock;

            bool transformPlan(const std::string & frame);
            geometry_msgs::msg::PoseStamped getLookAheadPose(const geometry_msgs::msg::PoseStamped & robot_pose);
            double getCurvature(const geometry_msgs::msg::Pose & look_ahead_pose);
            // Poses are assumed to be in the costmap's frame. Cells outside the costmap or
            // with no information are treated as clear.
            bool isPoseInCollision(const geometry_msgs::msg::PoseStamped & pose);
            bool isSegmentInCollision(const geometry_msgs::msg::PoseStamped & from, const geometry_msgs::msg::PoseStamped & to);
    };
}
