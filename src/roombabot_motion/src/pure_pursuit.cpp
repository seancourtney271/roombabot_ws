// CTRL SHIFT P > ROS2: Update C++ Properties
#include <algorithm>
#include "roombabot_motion/pure_pursuit.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace roombabot_motion
{
    // Constructor for the PD motion planner node.
    // The node receives a global path, transforms it into the robot's current frame,
    // and will eventually produce velocity commands based on a PD control law.
    PurePursuit::PurePursuit() : Node("pure_pursuit_planner_node"),
        look_ahead_distance(0.5), maximum_linear_velocity(0.3), maximum_angular_velocity(1.0), path_planner_node_name("/astar/path"), current_plan_index(0), locked_rotation_sign(0),
        is_currently_blocked(false)
    {
        // Declare configurable ROS2 parameters with default values.
        declare_parameter<double>("look_ahead_distance", look_ahead_distance);  // the distance to plan path to ahead
        declare_parameter<double>("maximum_linear_velocity", maximum_linear_velocity);  // max forward speed
        declare_parameter<double>("maximum_angular_velocity", maximum_angular_velocity);  // max rotational speed

        // Topic used to subscribe to the path planner output.
        declare_parameter<std::string>("path_subscriber", path_planner_node_name);

        declare_parameter<std::string>("costmap_topic", "/local_costmap/costmap");
        // 99 (not the stricter, more intuitive-looking ~50-90) deliberately only
        // catches cost essentially at true lethal/obstacle. DijkstraPlanner's own
        // cost < 99 raw-cost cutoff (see dijkstra_planner.cpp) already keeps a
        // weighted-preferred margin from edges when planning the route; this check
        // exists to catch a *new* obstacle the local costmap picked up since planning,
        // not to re-litigate a clearance decision the planner already made with a
        // different (translated 0-100 vs raw 0-255) cost scale. A stricter threshold
        // here left only ~8cm of buffer between "Dijkstra approved this path" and
        // "pure_pursuit panics," well within normal localization/costmap jitter.
        declare_parameter<int>("occupied_threshold", 99);
        declare_parameter<double>("heading_error_threshold", 0.5);
        declare_parameter<double>("stuck_timeout", 5.0);

        // Load parameter values after declaration so runtime overrides take effect.
        look_ahead_distance = get_parameter("look_ahead_distance").as_double();
        maximum_linear_velocity = get_parameter("maximum_linear_velocity").as_double();
        maximum_angular_velocity = get_parameter("maximum_angular_velocity").as_double();
        path_planner_node_name = get_parameter("path_subscriber").as_string();
        std::string costmap_topic = get_parameter("costmap_topic").as_string();
        occupied_threshold = get_parameter("occupied_threshold").as_int();
        heading_error_threshold = get_parameter("heading_error_threshold").as_double();
        stuck_timeout = get_parameter("stuck_timeout").as_double();

        // Subscribe to the planned path. The callback saves the latest path for use by the control loop.
        path_subscriber = create_subscription<nav_msgs::msg::Path>(path_planner_node_name, 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));

        // Costmap is published with transient_local durability (same convention as /map),
        // so new subscribers still get the current costmap even if it was published before
        // this node came up.
        costmap_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic, rclcpp::QoS(1).transient_local(),
            std::bind(&PurePursuit::costmapCallback, this, std::placeholders::_1));

        // Publish velocity commands and the next selected target pose.
        command_publisher = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        carrot_pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/purepursuit/carrot", 10);
        goal_reached_publisher_ = create_publisher<std_msgs::msg::Bool>("/purepursuit/goal_reached", 10);

        // Initialize TF2 utilities for frame transforms.
        tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Create a periodic timer that calls the control loop at 10 Hz.
        control_loop = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PurePursuit::controlLoop, this));

        RCLCPP_INFO(get_logger(), "Setup Complete");
    }

    // Store the latest path received from the planner.
    // This path is treated as the global reference trajectory until a new one arrives.
    void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        RCLCPP_INFO(get_logger(), "Path Recieved");
        global_plan = *path;
        current_plan_index = 0;
        locked_rotation_sign = 0;
        is_currently_blocked = false;
    }

    // Store the latest costmap for use by isPoseInCollision().
    void PurePursuit::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
    {
        latest_costmap = costmap;
    }

    // Control loop executed on a fixed timer.
    // It verifies path availability, queries the current robot pose, transforms the plan
    // into the robot's current reference frame, and prepares for PD control computation.
    void PurePursuit::controlLoop()
    {
        // If no path has been received yet, skip this cycle.
        if(global_plan.poses.empty())
        {
            // RCLCPP_INFO(get_logger(), "Path Not Recieved Yet");
            return;
        }

        geometry_msgs::msg::TransformStamped robot_pose;
        try
        {
            // Obtain the current transform from the odom frame to the robot base.
            robot_pose = tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        }
        catch(tf2::TransformException & ex)
        {
            RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // Convert the received global plan into the robot's current frame so a controller
        // can compute commands relative to the robot's position and orientation.
        if(!transformPlan(robot_pose.header.frame_id))
        {
            RCLCPP_ERROR(get_logger(), "Unable to transform Plan into robot's frame.");
            return;
        }

        // Currently have robot pose and global plan wrt to the same reference frame


        geometry_msgs::msg::PoseStamped robot_pose_stamped;
        // Tie robot pose stamped to robot pose frame
        robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
        robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
        robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
        robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

        // Takes in as input current pose of the robot
        auto look_ahead_pose = getLookAheadPose(robot_pose_stamped);

        // Check if we reached the goal
        double dx = look_ahead_pose.pose.position.x - robot_pose_stamped.pose.position.x;
        double dy = look_ahead_pose.pose.position.y - robot_pose_stamped.pose.position.y;
        // Calculate distance of each pose
        double distance = std::sqrt(dx * dx + dy * dy);
        // Did we reach the goal position?
        if(distance <= 0.1)
        {
            // Position alone is enough -- final heading is deliberately not checked
            // here anymore. The only caller that cares about final orientation is the
            // station-docking flow, and docking_controller already does its own
            // careful, marker-based final alignment right after this; a fast blind
            // rotate-to-match-saved-yaw here was pure redundant motion.
            RCLCPP_INFO(get_logger(), "Goal Reached!");
            global_plan.poses.clear();
            locked_rotation_sign = 0;
            std_msgs::msg::Bool goal_reached_msg;
            goal_reached_msg.data = true;
            goal_reached_publisher_->publish(goal_reached_msg);
            command_publisher->publish(geometry_msgs::msg::Twist());
            return;
        }

        // Publish look ahead pose
        carrot_pose_publisher->publish(look_ahead_pose);

        // Stop rather than drive into an obstacle the local costmap has picked up
        // since this path was planned (look_ahead_pose is still in the plan's
        // frame here, before it gets rewritten into the robot-relative frame below).
        if(isPoseInCollision(look_ahead_pose))
        {
            if(!is_currently_blocked)
            {
                is_currently_blocked = true;
                blocked_start_time = get_clock()->now();
            }

            double blocked_duration = (get_clock()->now() - blocked_start_time).seconds();
            if(blocked_duration > stuck_timeout)
            {
                // The same look-ahead point has stayed blocked for too long -- nothing
                // about this situation is going to resolve itself by waiting (see the
                // comment on stuck_timeout in the header), so give up on this path
                // instead of jittering here forever.
                RCLCPP_ERROR(get_logger(), "Blocked by an obstacle for over %.1f seconds -- giving up on this path.", stuck_timeout);
                global_plan.poses.clear();
                locked_rotation_sign = 0;
                is_currently_blocked = false;
                command_publisher->publish(geometry_msgs::msg::Twist());
                return;
            }

            RCLCPP_WARN(get_logger(), "Obstacle detected ahead, stopping.");
            command_publisher->publish(geometry_msgs::msg::Twist());
            return;
        }
        is_currently_blocked = false;

        // Get the error of look ahead pose and robot pose
        tf2::Transform robot_tf, look_ahead_pose_tf, look_ahead_pose_robot_tf;
        tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
        tf2::fromMsg(look_ahead_pose.pose, look_ahead_pose_tf);
        // Gets the error of the pose of the robot and the look ahead pose we want to reach
        look_ahead_pose_robot_tf = robot_tf.inverse() * look_ahead_pose_tf;

        // look_ahead_pose is now expressed in the robot's own frame: atan2(y, x)
        // is the heading error to it.
        tf2::toMsg(look_ahead_pose_robot_tf, look_ahead_pose.pose);
        double heading_error = std::atan2(look_ahead_pose.pose.position.y, look_ahead_pose.pose.position.x);

        geometry_msgs::msg::Twist cmd_vel;
        if(std::abs(heading_error) > heading_error_threshold)
        {
            // Look-ahead point is well off to the side or behind the robot --
            // the curvature formula below degenerates for large heading errors
            // (this is what previously sent the robot the wrong way entirely when
            // a new goal required a near-180-degree turn from its current heading).
            // Rotate in place toward it first instead, locking in the direction so
            // noise near the +-pi boundary can't flip it back and forth forever.
            if(locked_rotation_sign == 0)
            {
                locked_rotation_sign = (heading_error > 0) ? 1 : -1;
            }
            cmd_vel.angular.z = locked_rotation_sign * maximum_angular_velocity;
        }
        else
        {
            locked_rotation_sign = 0;
            double curvature = getCurvature(look_ahead_pose.pose);
            cmd_vel.linear.x = maximum_linear_velocity;
            cmd_vel.angular.z = curvature * maximum_angular_velocity;
        }

        // Publish cmd_Vel
        command_publisher->publish(cmd_vel);
    }

    // Transform the stored plan into the requested coordinate frame.
    // This rewrites each pose so the global path is expressed relative to the robot's current frame.
    bool PurePursuit::transformPlan(const std::string & frame)
    {
        if(global_plan.header.frame_id == frame)
        {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer->lookupTransform(frame, global_plan.header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::LookupException & ex)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Couldn't transform plan from frame " << global_plan.header.frame_id << " to " << frame);
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

    geometry_msgs::msg::PoseStamped PurePursuit::getLookAheadPose(const geometry_msgs::msg::PoseStamped & robot_pose)
    {
        // If the saved search index is outside the plan, reset it to the beginning.
        // This can happen when a new plan is received or the current index reached the end.
        if(current_plan_index >= global_plan.poses.size())
        {
            current_plan_index = 0;
        }

        // Fallback target: the closest still-on-path pose. Used if nothing farther
        // along the path has a chord safe to drive straight at (see the loop below) --
        // a safer default than the final goal pose, which could be on the far side of
        // whatever's blocking the chord.
        geometry_msgs::msg::PoseStamped look_ahead_pose = global_plan.poses[current_plan_index];
        look_ahead_pose.header = global_plan.header;

        // Search forward from the current plan index for the next waypoint that is
        // farther than the configured lookahead distance from the robot, without ever
        // accepting a candidate whose direct chord from the robot cuts across an
        // edge/obstacle. Without this check, a point just past a sharp bend can already
        // be farther than look_ahead_distance in a straight line even though the actual
        // path swings wide to reach it -- picking it as the carrot makes pure pursuit
        // clip the corner the plan was routed around.
        for(std::size_t i = current_plan_index; i < global_plan.poses.size(); ++i)
        {
            const auto & pose = global_plan.poses[i];
            double dx = pose.pose.position.x - robot_pose.pose.position.x;
            double dy = pose.pose.position.y - robot_pose.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if(isSegmentInCollision(robot_pose, pose))
            {
                // Stop extending the carrot any farther -- keep whatever was accepted
                // last (or the closest-pose fallback above, if nothing further along
                // ever had a safe chord).
                break;
            }

            look_ahead_pose = pose;
            current_plan_index = i;

            if(distance > look_ahead_distance)
            {
                // Found the next lookahead pose.
                break;
            }
        }

        // Compute orientation for the look ahead pose to point toward the next waypoint
        if(current_plan_index + 1 < global_plan.poses.size())
        {
            const auto & next_pose = global_plan.poses[current_plan_index + 1];
            double dx = next_pose.pose.position.x - look_ahead_pose.pose.position.x;
            double dy = next_pose.pose.position.y - look_ahead_pose.pose.position.y;
            double yaw = std::atan2(dy, dx);
            
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            look_ahead_pose.pose.orientation = tf2::toMsg(q);
        }

        return look_ahead_pose;
    }
    double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & look_ahead_pose)
    {
        const double L = (look_ahead_pose.position.x * look_ahead_pose.position.x) + (look_ahead_pose.position.y * look_ahead_pose.position.y);
        if (L > 0.001)
        {
            return 2 * look_ahead_pose.position.y / L;
        }
        else
        {
            return 0.0;
        }
    }

    bool PurePursuit::isPoseInCollision(const geometry_msgs::msg::PoseStamped & pose)
    {
        if(!latest_costmap)
        {
            // No costmap data yet -- don't block the robot on missing data.
            return false;
        }

        geometry_msgs::msg::PoseStamped pose_in_costmap_frame;
        if(pose.header.frame_id == latest_costmap->header.frame_id)
        {
            pose_in_costmap_frame = pose;
        }
        else
        {
            try
            {
                auto transform = tf_buffer->lookupTransform(
                    latest_costmap->header.frame_id, pose.header.frame_id, tf2::TimePointZero);
                tf2::doTransform(pose, pose_in_costmap_frame, transform);
            }
            catch(const tf2::TransformException & ex)
            {
                RCLCPP_WARN(get_logger(), "Could not transform look-ahead pose into costmap frame: %s", ex.what());
                return false;
            }
        }

        const auto & info = latest_costmap->info;
        int grid_x = static_cast<int>((pose_in_costmap_frame.pose.position.x - info.origin.position.x) / info.resolution);
        int grid_y = static_cast<int>((pose_in_costmap_frame.pose.position.y - info.origin.position.y) / info.resolution);

        if(grid_x < 0 || grid_y < 0 || grid_x >= static_cast<int>(info.width) || grid_y >= static_cast<int>(info.height))
        {
            // Outside the costmap's known area -- nothing to check against.
            return false;
        }

        int8_t occupancy = latest_costmap->data[grid_y * info.width + grid_x];
        return occupancy >= occupied_threshold;
    }

    bool PurePursuit::isSegmentInCollision(const geometry_msgs::msg::PoseStamped & from, const geometry_msgs::msg::PoseStamped & to)
    {
        double dx = to.pose.position.x - from.pose.position.x;
        double dy = to.pose.position.y - from.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Sample roughly every costmap cell so no obstacle-sized gap can hide between
        // samples. Falls back to a reasonable step if no costmap has arrived yet --
        // isPoseInCollision itself will just report clear in that case anyway.
        double step = latest_costmap ? latest_costmap->info.resolution : 0.05;
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

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library before creating the node.
    rclcpp::init(argc, argv);

    // Create the PD motion planner node and keep it alive while ROS is running.
    auto node = std::make_shared<roombabot_motion::PurePursuit>();
    rclcpp::spin(node);

    // Cleanly shutdown ROS2 resources after node exit.
    rclcpp::shutdown();
    return 0;
}