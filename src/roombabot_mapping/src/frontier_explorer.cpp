#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// Frontier-based autonomous exploration for slam_toolbox + Nav2.
//
// A frontier is a known-free cell that touches unknown space. Each cycle the node finds
// the frontiers in /map, clusters them, picks the nearest usable cluster and sends a
// point on it to Nav2 as a NavigateToPose goal. A goal that Nav2 fails or that times out
// is blacklisted so the robot does not keep retrying an unreachable spot. When no usable
// frontier remains for several cycles in a row, exploration is finished (optionally after
// driving back to where it started).
class FrontierExplorer : public rclcpp::Node
{
public:
  FrontierExplorer()
  : Node("frontier_explorer"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("map_topic", "/map");
    declare_parameter("global_frame", "map");
    declare_parameter("robot_frame", "base_footprint");
    // Frontier clusters smaller than this (metres) are sensor noise, not openings.
    declare_parameter("min_frontier_size", 0.3);
    // Frontier cells closer than this (metres) to an obstacle are ignored so goals are
    // not placed inside the inflated costmap zone.
    declare_parameter("obstacle_clearance", 0.3);
    // Frontiers closer than this (metres) to the robot are treated as already reached.
    declare_parameter("min_goal_distance", 0.4);
    // A blacklisted goal also rules out any frontier within this radius (metres).
    declare_parameter("blacklist_radius", 0.5);
    declare_parameter("goal_timeout", 60.0);
    declare_parameter("planner_frequency", 1.0);
    // Consecutive cycles with no usable frontier before exploration is declared done.
    declare_parameter("empty_cycles_to_finish", 5);
    declare_parameter("return_to_start", true);

    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    min_frontier_size_ = get_parameter("min_frontier_size").as_double();
    obstacle_clearance_ = get_parameter("obstacle_clearance").as_double();
    min_goal_distance_ = get_parameter("min_goal_distance").as_double();
    blacklist_radius_ = get_parameter("blacklist_radius").as_double();
    goal_timeout_ = get_parameter("goal_timeout").as_double();
    empty_cycles_to_finish_ = get_parameter("empty_cycles_to_finish").as_int();
    return_to_start_ = get_parameter("return_to_start").as_bool();

    // slam_toolbox publishes /map latched (transient local).
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {map_ = std::move(msg);});

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    const double period = 1.0 / get_parameter("planner_frequency").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      [this]() {cycle();});

    RCLCPP_INFO(get_logger(), "Frontier explorer started, waiting for map and Nav2");
  }

private:
  using Point = std::pair<double, double>;

  bool robotXY(Point & xy)
  {
    try {
      const auto t = tf_buffer_.lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
      xy = {t.transform.translation.x, t.transform.translation.y};
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  // Returns {distance, x, y} for every usable frontier cluster.
  std::vector<std::array<double, 3>> findFrontiers(const Point & robot) const
  {
    const auto & info = map_->info;
    const int width = static_cast<int>(info.width);
    const int height = static_cast<int>(info.height);
    const double res = info.resolution;
    const auto & data = map_->data;
    auto at = [&](int r, int c) {return data[static_cast<size_t>(r) * width + c];};
    auto inside = [&](int r, int c) {return r >= 0 && r < height && c >= 0 && c < width;};

    const int clearance_cells = static_cast<int>(std::ceil(obstacle_clearance_ / res));
    const int min_cells = std::max(1, static_cast<int>(min_frontier_size_ / res));

    // Free cells with at least one unknown 4-neighbour, and no obstacle within the clearance.
    std::vector<uint8_t> frontier(data.size(), 0);
    const int dr4[] = {-1, 1, 0, 0};
    const int dc4[] = {0, 0, -1, 1};
    for (int r = 0; r < height; ++r) {
      for (int c = 0; c < width; ++c) {
        if (at(r, c) != 0) {
          continue;
        }
        bool touches_unknown = false;
        for (int k = 0; k < 4 && !touches_unknown; ++k) {
          const int nr = r + dr4[k];
          const int nc = c + dc4[k];
          touches_unknown = inside(nr, nc) && at(nr, nc) < 0;
        }
        if (!touches_unknown) {
          continue;
        }
        bool near_obstacle = false;
        for (int dr = -clearance_cells; dr <= clearance_cells && !near_obstacle; ++dr) {
          for (int dc = -clearance_cells; dc <= clearance_cells; ++dc) {
            if (dr * dr + dc * dc > clearance_cells * clearance_cells) {
              continue;
            }
            if (inside(r + dr, c + dc) && at(r + dr, c + dc) > 0) {
              near_obstacle = true;
              break;
            }
          }
        }
        if (!near_obstacle) {
          frontier[static_cast<size_t>(r) * width + c] = 1;
        }
      }
    }

    // Group frontier cells into 8-connected clusters.
    std::vector<std::array<double, 3>> result;
    std::vector<uint8_t> visited(data.size(), 0);
    for (int r = 0; r < height; ++r) {
      for (int c = 0; c < width; ++c) {
        const size_t start = static_cast<size_t>(r) * width + c;
        if (!frontier[start] || visited[start]) {
          continue;
        }
        std::vector<std::pair<int, int>> cells;
        std::deque<std::pair<int, int>> queue{{r, c}};
        visited[start] = 1;
        while (!queue.empty()) {
          const auto [cr, cc] = queue.front();
          queue.pop_front();
          cells.emplace_back(cr, cc);
          for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
              const int nr = cr + dr;
              const int nc = cc + dc;
              if (!inside(nr, nc)) {
                continue;
              }
              const size_t idx = static_cast<size_t>(nr) * width + nc;
              if (frontier[idx] && !visited[idx]) {
                visited[idx] = 1;
                queue.emplace_back(nr, nc);
              }
            }
          }
        }
        if (static_cast<int>(cells.size()) < min_cells) {
          continue;
        }

        // Centroid of the cluster, snapped to the nearest actual frontier cell so the
        // goal is guaranteed to lie on free space even for curved frontiers.
        double mean_r = 0.0;
        double mean_c = 0.0;
        for (const auto & [cr, cc] : cells) {
          mean_r += cr;
          mean_c += cc;
        }
        mean_r /= cells.size();
        mean_c /= cells.size();
        double best = std::numeric_limits<double>::max();
        std::pair<int, int> pick = cells.front();
        for (const auto & cell : cells) {
          const double d = std::pow(cell.first - mean_r, 2) + std::pow(cell.second - mean_c, 2);
          if (d < best) {
            best = d;
            pick = cell;
          }
        }

        const double x = info.origin.position.x + (pick.second + 0.5) * res;
        const double y = info.origin.position.y + (pick.first + 0.5) * res;
        const double distance = std::hypot(x - robot.first, y - robot.second);
        if (distance < min_goal_distance_ || isBlacklisted(x, y)) {
          continue;
        }
        result.push_back({distance, x, y});
      }
    }
    return result;
  }

  bool isBlacklisted(double x, double y) const
  {
    return std::any_of(
      blacklist_.begin(), blacklist_.end(), [&](const Point & p) {
        return std::hypot(x - p.first, y - p.second) < blacklist_radius_;
      });
  }

  void cycle()
  {
    if (finished_ || !map_) {
      return;
    }
    Point robot;
    if (!robotXY(robot)) {
      return;
    }
    if (!start_xy_) {
      start_xy_ = robot;
    }

    if (goal_pending_) {
      return;
    }

    if (goal_handle_) {
      if ((now() - goal_sent_at_).seconds() > goal_timeout_) {
        RCLCPP_WARN(get_logger(), "Goal timed out, blacklisting it");
        nav_client_->async_cancel_goal(goal_handle_);
        finishGoal(true);
      }
      return;
    }

    if (returning_home_) {
      sendGoal(robot, start_xy_->first, start_xy_->second, true);  // retries if the last was rejected
      return;
    }

    const auto frontiers = findFrontiers(robot);
    if (frontiers.empty()) {
      if (++empty_cycles_ >= empty_cycles_to_finish_) {
        explorationDone();
      }
      return;
    }
    empty_cycles_ = 0;

    const auto best = *std::min_element(
      frontiers.begin(), frontiers.end(),
      [](const auto & a, const auto & b) {return a[0] < b[0];});
    sendGoal(robot, best[1], best[2], false);
  }

  void sendGoal(const Point & robot, double x, double y, bool home)
  {
    if (!nav_client_->wait_for_action_server(100ms)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for navigate_to_pose action server");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    // Face along the direction of travel. An identity heading would force the robot to
    // rotate to map +x on arrival, which fails the goal's yaw tolerance next to walls.
    const double yaw = std::atan2(y - robot.second, x - robot.first);
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_INFO(
      get_logger(), "%s at (%.2f, %.2f)", home ? "Returning to start" : "Exploring frontier", x, y);
    goal_xy_ = Point{x, y};
    goal_pending_ = true;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        goal_pending_ = false;
        if (!handle) {
          // Usually Nav2 is not active yet; try again next cycle without blacklisting.
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Goal rejected, will retry");
          goal_xy_.reset();
          return;
        }
        goal_handle_ = handle;
        goal_sent_at_ = now();
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        // Ignore results of goals we already gave up on (timeout cancel).
        if (!goal_handle_ || result.goal_id != goal_handle_->get_goal_id()) {
          return;
        }
        const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        if (!ok) {
          RCLCPP_WARN(
            get_logger(), "Goal ended with code %d, blacklisting it", static_cast<int>(result.code));
        }
        finishGoal(!ok);
      };
    nav_client_->async_send_goal(goal, options);
  }

  void finishGoal(bool blacklist)
  {
    if (blacklist && goal_xy_ && !returning_home_) {
      blacklist_.push_back(*goal_xy_);
    }
    goal_handle_.reset();
    goal_xy_.reset();
    if (returning_home_) {
      finished_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Exploration finished: back at start. Save the map with map_saver_server/save_map");
    }
  }

  void explorationDone()
  {
    RCLCPP_INFO(
      get_logger(), "No frontiers left (%zu unreachable goals skipped)", blacklist_.size());
    if (return_to_start_ && start_xy_) {
      returning_home_ = true;  // cycle() sends the goal and retries until accepted
    } else {
      finished_ = true;
      RCLCPP_INFO(get_logger(), "Exploration finished. Save the map with map_saver_server/save_map");
    }
  }

  std::string global_frame_, robot_frame_;
  double min_frontier_size_, obstacle_clearance_, min_goal_distance_, blacklist_radius_, goal_timeout_;
  int empty_cycles_to_finish_;
  bool return_to_start_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::vector<Point> blacklist_;
  std::optional<Point> start_xy_;
  std::optional<Point> goal_xy_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Time goal_sent_at_{0, 0, RCL_ROS_TIME};
  bool goal_pending_{false};  // goal sent, waiting for accept/reject
  bool returning_home_{false};
  int empty_cycles_{0};
  bool finished_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
