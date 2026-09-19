// Placeholder final-approach controller.
//
// The bumperbot's version servoed onto an ArUco marker's pose. This robot has no
// camera, and stations are taught by driving the robot to them (see
// station_recorder), so coarse navigation already ends exactly where the station was
// recorded -- there is no residual error for a final approach to remove.
//
// This node exists to keep the handoff *contract* alive rather than the behaviour:
// the topics, the sequencing, and the /docking/docked completion signal all still
// work, so a future final-approach sensor (a lidar dock detector, a downward IR
// beacon, whatever) can be dropped in by replacing the body of onTarget() without
// touching mission_commander. It is not launched by default -- navigation.launch.py
// gates it behind use_docking, which defaults to false.
//
// It deliberately publishes no velocity commands at all. A stub that "approaches"
// nothing would be worse than one that does nothing.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <memory>

namespace roombabot_motion
{

class DockingController : public rclcpp::Node
{
public:
  DockingController()
  : Node("docking_controller_node")
  {
    // How long to wait before declaring the (nonexistent) approach finished. Non-zero
    // so the sequencing is realistic for whatever replaces this, and so a consumer
    // that expects docking to take time is not surprised by an instantaneous reply.
    settle_time_ = declare_parameter<double>("settle_time", 0.5);

    docked_publisher_ = create_publisher<std_msgs::msg::Bool>("/docking/docked", 10);
    target_subscriber_ = create_subscription<std_msgs::msg::Int32>(
      "/docking/target_station_id", 10,
      [this](const std_msgs::msg::Int32 & msg) { onTarget(msg.data); });

    RCLCPP_INFO(
      get_logger(),
      "Docking controller running as a pass-through stub -- stations are taught by "
      "driving, so coarse navigation is the final approach. Replace onTarget() when a "
      "final-approach sensor is added.");
  }

private:
  void onTarget(int32_t station_id)
  {
    RCLCPP_INFO(get_logger(), "Docking requested for station %d", station_id);

    // One-shot timer rather than a sleep: blocking the executor here would stall every
    // other callback in the process, and whatever real controller replaces this will
    // need to run its control loop on the executor anyway.
    settle_timer_ = create_wall_timer(
      std::chrono::duration<double>(settle_time_),
      [this, station_id]()
      {
        settle_timer_->cancel();
        std_msgs::msg::Bool docked;
        docked.data = true;
        docked_publisher_->publish(docked);
        RCLCPP_INFO(get_logger(), "Station %d reported docked", station_id);
      });
  }

  double settle_time_{0.5};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docked_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_subscriber_;
  rclcpp::TimerBase::SharedPtr settle_timer_;
};

}  // namespace roombabot_motion


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roombabot_motion::DockingController>());
  rclcpp::shutdown();
  return 0;
}
