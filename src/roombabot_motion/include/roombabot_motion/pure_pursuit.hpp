#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace roombabot_motion
{
    class PurePursuit : public rclcpp::Node
    {
        public:
            PurePursuit();
        private:
            // Grabs the global path from the path planner
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber;
            // Live obstacle data used to stop before driving into something not
            // accounted for when the path was originally planned.
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber;
            // Publishes the command to the wheels
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pose_publisher;
            // Fires once per completed goal so other nodes (e.g. mission_commander) can
            // react to arrival -- otherwise "reached the goal" only ever existed as a
            // log line, with nothing else in the system able to observe it.
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_publisher_;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener;

            rclcpp::TimerBase::SharedPtr control_loop;

            // How far the look ahead distance will plan for
            double look_ahead_distance;
            double maximum_linear_velocity;
            double maximum_angular_velocity;
            // Above this heading error (radians) to the look-ahead point, rotate in
            // place instead of curvature-driving -- the curvature formula degenerates
            // for points far to the side or behind the robot.
            double heading_error_threshold;
            // Rotation direction (+1/-1) committed to while rotating in place, or 0
            // when not currently rotating. Locked in once chosen and held until the
            // heading error shrinks below threshold, so noise near the +-pi wraparound
            // boundary (target nearly behind the robot) can't flip the sign every cycle
            // and cause the robot to oscillate in place instead of turning through.
            int locked_rotation_sign;
            // Holds path
            nav_msgs::msg::Path global_plan;

            // Path planner node name
            std::string path_planner_node_name;
            // Index of the first path point still ahead of the robot.
            std::size_t current_plan_index;

            // Latest local costmap, used to check the look-ahead point for obstacles.
            nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap;
            // Occupancy value (0-100 scale) at/above which a cell is treated as blocked.
            int occupied_threshold;

            // How long the look-ahead point can stay blocked before giving up on the
            // current path entirely, rather than sitting there re-checking the same
            // blocked point forever. Guards against cases where the global plan legally
            // hugs a boundary the local costmap's (independently recomputed, rolling
            // window) view of the same spot disagrees with -- nothing will ever change
            // there on its own, so waiting indefinitely just hangs the mission.
            double stuck_timeout;
            // Set the moment the look-ahead point first became blocked; used with
            // stuck_timeout to measure how long the blockage has persisted.
            rclcpp::Time blocked_start_time;
            bool is_currently_blocked;

            void controlLoop();

            void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

            void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);

            bool transformPlan(const std::string & frame);

            geometry_msgs::msg::PoseStamped getLookAheadPose(const geometry_msgs::msg::PoseStamped & robot_pose);

            double getCurvature(const geometry_msgs::msg::Pose & look_ahead_pose);

            // Checks whether the given pose falls on an occupied/unknown-beyond-threshold
            // cell of the latest costmap, transforming it into the costmap's frame first
            // if needed. Returns false (assume clear) if no costmap has arrived yet or the
            // pose falls outside the costmap's bounds, rather than blocking the robot on
            // missing data.
            bool isPoseInCollision(const geometry_msgs::msg::PoseStamped & pose);

            // Samples points along the straight line between "from" and "to" (both
            // assumed to be in the same frame) and checks each against isPoseInCollision.
            // Used to reject look-ahead candidates whose direct chord from the robot
            // would cut across an edge/obstacle even though the point itself, and the
            // actual (curved) path to reach it, are clear -- the classic pure-pursuit
            // corner-cutting failure at sharp bends.
            bool isSegmentInCollision(const geometry_msgs::msg::PoseStamped & from, const geometry_msgs::msg::PoseStamped & to);
    };
}