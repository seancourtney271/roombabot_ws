#ifndef ROOMBABOT_FIRMWARE__ROOMBABOT_INTERFACE_HPP
#define ROOMBABOT_FIRMWARE__ROOMBABOT_INTERFACE_HPP

#include "roombabot_firmware/esp32_protocol.hpp"
#include "roombabot_firmware/seqlock.hpp"
#include "roombabot_firmware/serial_link.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace roombabot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ros2_control system interface for the esp32_motor board.
//
// The whole point of the design is that read() and write() do no blocking work. A
// reader thread inside SerialLink owns the port; read() is an atomic load out of a
// seqlock, and write() is one memcpy plus one non-blocking write() syscall. Nothing
// here allocates, takes a lock, or constructs a clock, so a 100 Hz update_rate has
// the full 10 ms budget available to the controllers rather than losing a chunk of it
// to serial I/O every cycle.
class RoombabotInterface : public hardware_interface::SystemInterface
{
public:
  RoombabotInterface() = default;
  ~RoombabotInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Index into the exported interface vectors. Resolved from joint names in on_init
  // rather than assumed from URDF ordering -- reordering the <joint> tags in the
  // xacro used to silently swap the wheels.
  static constexpr size_t kLeft = 0;
  static constexpr size_t kRight = 1;

  void sendCommand(float left_rad_s, float right_rad_s, bool enable);

  SerialLink link_;
  SeqlockSlot<protocol::MotorState> state_slot_;

  std::string port_;
  unsigned int baud_rate_{921600};
  double ticks_per_revolution_{0.0};
  double radians_per_tick_{0.0};
  double state_timeout_{0.5};

  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // Wrap-safe tick accumulation. The ESP32 sends a free-running int32; differencing
  // successive values in signed 32-bit arithmetic gives the correct delta even across
  // INT32_MAX, and the running total lives in an int64 that will not wrap in any
  // plausible lifetime (at 1000 ticks/rev and 10 rev/s that is ~29 million years).
  bool have_first_sample_{false};
  int32_t last_left_ticks_{0};
  int32_t last_right_ticks_{0};
  int64_t left_tick_total_{0};
  int64_t right_tick_total_{0};

  uint32_t last_write_count_{0};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  bool link_timed_out_{false};

  uint8_t command_sequence_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("RoombabotInterface")};
  // Held as a member so the throttled warnings below don't construct (and heap-
  // allocate) a clock on every 100 Hz cycle. Steady time so throttling still works
  // before /clock exists and doesn't jump if the system clock is stepped.
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
};

}  // namespace roombabot_firmware

#endif  // ROOMBABOT_FIRMWARE__ROOMBABOT_INTERFACE_HPP
