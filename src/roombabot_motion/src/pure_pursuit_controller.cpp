#include <algorithm>
#include <cmath>
#include "roombabot_motion/pure_pursuit_controller.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace roombabot_motion
{
    void PurePursuitController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                std::string name,
                std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
    {
        auto node = parent.lock();
        plugin_name = name;
        tf = tf_buffer;
        costmap_ros = costmap;
        logger = node->get_logger();
        clock = node->get_clock();

        nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".look_ahead_distance", rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".maximum_linear_velocity", rclcpp::ParameterValue(0.2));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".maximum_angular_velocity", rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".heading_error_threshold", rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".occupied_threshold", rclcpp::ParameterValue(253));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".stuck_timeout", rclcpp::ParameterValue(5.0));

        node->get_parameter(plugin_name + ".look_ahead_distance", look_ahead_distance);
        node->get_parameter(plugin_name + ".maximum_linear_velocity", maximum_linear_velocity);
        node->get_parameter(plugin_name + ".maximum_angular_velocity", maximum_angular_velocity);
        node->get_parameter(plugin_name + ".heading_error_threshold", heading_error_threshold);
        node->get_parameter(plugin_name + ".occupied_threshold", occupied_threshold);
        node->get_parameter(plugin_name + ".stuck_timeout", stuck_timeout);

        linear_velocity_limit = maximum_linear_velocity;
        current_plan_index = 0;
        locked_rotation_sign = 0;
        is_currently_blocked = false;

        carrot_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/purepursuit/carrot", 10);
    }

    void PurePursuitController::cleanup()
    {
        RCLCPP_INFO(logger, "Cleaning up plugin PurePursuitController");
        carrot_pose_pub.reset();
    }

    void PurePursuitController::activate()
    {
        RCLCPP_INFO(logger, "Activating plugin PurePursuitController");
        carrot_pose_pub->on_activate();
    }

    void PurePursuitController::deactivate()
    {
        RCLCPP_INFO(logger, "Deactivating plugin PurePursuitController");
        carrot_pose_pub->on_deactivate();
    }

    void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
    {
        global_plan = path;
        current_plan_index = 0;
        locked_rotation_sign = 0;
        is_currently_blocked = false;
    }

    void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
    {
        if(speed_limit <= 0.0)
        {
            // Nav2 uses a non-positive limit to mean "no limit".
            linear_velocity_limit = maximum_linear_velocity;
        }
        else if(percentage)
        {
            linear_velocity_limit = maximum_linear_velocity * speed_limit / 100.0;
        }
        else
        {
            linear_velocity_limit = std::min(speed_limit, maximum_linear_velocity);
        }
    }

    // Rebuilds working_plan from global_plan in the given frame.
    bool PurePursuitController::transformPlan(const std::string & frame)
    {
        working_plan = global_plan;
        if(global_plan.header.frame_id == frame)
        {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf->lookupTransform(frame, global_plan.header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::TransformException & ex)
        {
            RCLCPP_ERROR_STREAM(logger, "Couldn't transform plan from frame " << global_plan.header.frame_id << " to " << frame << ": " << ex.what());
            return false;
        }

        for(auto & pose : working_plan.poses)
        {
            tf2::doTransform(pose, pose, transform);
        }
        working_plan.header.frame_id = frame;
        return true;
    }

    geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & robot_pose,
                const geometry_msgs::msg::Twist &,
                nav2_core::GoalChecker *)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = robot_pose.header.frame_id;
        cmd_vel.header.stamp = clock->now();

        if(global_plan.poses.empty())
        {
            throw nav2_core::InvalidPath("Received an empty plan");
        }

        if(!transformPlan(robot_pose.header.frame_id))
        {
            throw nav2_core::ControllerTFError("Unable to transform plan into the robot's frame");
        }

        auto look_ahead_pose = getLookAheadPose(robot_pose);

        // On top of the goal: stop commanding motion and let the goal checker finish up,
        // rather than spinning toward a carrot that is under the robot.
        double dx = look_ahead_pose.pose.position.x - robot_pose.pose.position.x;
        double dy = look_ahead_pose.pose.position.y - robot_pose.pose.position.y;
        if(std::sqrt(dx * dx + dy * dy) <= 0.05)
        {
            return cmd_vel;
        }

        carrot_pose_pub->publish(look_ahead_pose);

        // Stop rather than drive into an obstacle picked up since the path was planned.
        if(isPoseInCollision(look_ahead_pose))
        {
            if(!is_currently_blocked)
            {
                is_currently_blocked = true;
                blocked_start_time = clock->now();
            }

            if((clock->now() - blocked_start_time).seconds() > stuck_timeout)
            {
                is_currently_blocked = false;
                locked_rotation_sign = 0;
                throw nav2_core::NoValidControl("Blocked by an obstacle past stuck_timeout");
            }

            RCLCPP_WARN_THROTTLE(logger, *clock, 1000, "Obstacle detected ahead, stopping.");
            return cmd_vel;
        }
        is_currently_blocked = false;

        // Express the carrot in the robot's own frame: atan2(y, x) is the heading error.
        tf2::Transform robot_tf, look_ahead_tf;
        tf2::fromMsg(robot_pose.pose, robot_tf);
        tf2::fromMsg(look_ahead_pose.pose, look_ahead_tf);
        geometry_msgs::msg::Pose look_ahead_in_robot;
        tf2::toMsg(robot_tf.inverse() * look_ahead_tf, look_ahead_in_robot);
        double heading_error = std::atan2(look_ahead_in_robot.position.y, look_ahead_in_robot.position.x);

        if(std::abs(heading_error) > heading_error_threshold)
        {
            // The curvature formula degenerates for points far to the side or behind the
            // robot, so rotate in place first. The direction is locked so noise near the
            // +-pi boundary can't flip it back and forth.
            if(locked_rotation_sign == 0)
            {
                locked_rotation_sign = (heading_error > 0) ? 1 : -1;
            }
            cmd_vel.twist.angular.z = locked_rotation_sign * maximum_angular_velocity;
        }
        else
        {
            locked_rotation_sign = 0;
            cmd_vel.twist.linear.x = linear_velocity_limit;
            cmd_vel.twist.angular.z = getCurvature(look_ahead_in_robot) * maximum_angular_velocity;
        }

        return cmd_vel;
    }

    geometry_msgs::msg::PoseStamped PurePursuitController::getLookAheadPose(const geometry_msgs::msg::PoseStamped & robot_pose)
    {
        if(current_plan_index >= working_plan.poses.size())
        {
            current_plan_index = 0;
        }

        // Fallback: the closest still-on-path pose, safer than the final goal pose which
        // could be on the far side of whatever is blocking the chord.
        geometry_msgs::msg::PoseStamped look_ahead_pose = working_plan.poses[current_plan_index];
        look_ahead_pose.header = working_plan.header;

        // Search forward for the next waypoint farther than look_ahead_distance, never
        // accepting one whose direct chord from the robot cuts across an obstacle -- a
        // point just past a sharp bend can be far away in a straight line while the path
        // swings wide to reach it, and picking it makes pure pursuit clip the corner.
        for(std::size_t i = current_plan_index; i < working_plan.poses.size(); ++i)
        {
            const auto & pose = working_plan.poses[i];
            double dx = pose.pose.position.x - robot_pose.pose.position.x;
            double dy = pose.pose.position.y - robot_pose.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if(isSegmentInCollision(robot_pose, pose))
            {
                break;
            }

            look_ahead_pose = pose;
            look_ahead_pose.header = working_plan.header;
            current_plan_index = i;

            if(distance > look_ahead_distance)
            {
                break;
            }
        }

        // Point the carrot toward the next waypoint
        if(current_plan_index + 1 < working_plan.poses.size())
        {
            const auto & next_pose = working_plan.poses[current_plan_index + 1];
            double dx = next_pose.pose.position.x - look_ahead_pose.pose.position.x;
            double dy = next_pose.pose.position.y - look_ahead_pose.pose.position.y;
            tf2::Quaternion q;
            q.setRPY(0, 0, std::atan2(dy, dx));
            look_ahead_pose.pose.orientation = tf2::toMsg(q);
        }

        return look_ahead_pose;
    }

    double PurePursuitController::getCurvature(const geometry_msgs::msg::Pose & look_ahead_pose)
    {
        const double L = (look_ahead_pose.position.x * look_ahead_pose.position.x) + (look_ahead_pose.position.y * look_ahead_pose.position.y);
        if(L > 0.001)
        {
            return 2 * look_ahead_pose.position.y / L;
        }
        return 0.0;
    }

    bool PurePursuitController::isPoseInCollision(const geometry_msgs::msg::PoseStamped & pose)
    {
        auto * costmap = costmap_ros->getCostmap();
        unsigned int mx, my;
        if(!costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
        {
            return false;
        }

        unsigned char cost = costmap->getCost(mx, my);
        if(cost == nav2_costmap_2d::NO_INFORMATION)
        {
            return false;
        }
        return cost >= occupied_threshold;
    }

    bool PurePursuitController::isSegmentInCollision(const geometry_msgs::msg::PoseStamped & from, const geometry_msgs::msg::PoseStamped & to)
    {
        double dx = to.pose.position.x - from.pose.position.x;
        double dy = to.pose.position.y - from.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Sample about every costmap cell so no obstacle-sized gap hides between samples.
        double step = costmap_ros->getCostmap()->getResolution();
        int num_samples = std::max(1, static_cast<int>(distance / step));

        for(int i = 1; i <= num_samples; ++i)
        {
            double t = static_cast<double>(i) / num_samples;
            geometry_msgs::msg::PoseStamped sample = to;
            sample.pose.position.x = from.pose.position.x + t * dx;
            sample.pose.position.y = from.pose.position.y + t * dy;

            if(isPoseInCollision(sample))
            {
                return true;
            }
        }
        return false;
    }
}

PLUGINLIB_EXPORT_CLASS(roombabot_motion::PurePursuitController, nav2_core::Controller)
