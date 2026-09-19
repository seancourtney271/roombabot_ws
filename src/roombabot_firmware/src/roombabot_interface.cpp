#include "roombabot_firmware/roombabot_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>

namespace roombabot_firmware
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

double paramOr(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double fallback)
{
  const auto it = params.find(key);
  if (it == params.end())
  {
    return fallback;
  }
  try
  {
    return std::stod(it->second);
  }
  catch (const std::exception &)
  {
    return fallback;
  }
}
}  // namespace


RoombabotInterface::~RoombabotInterface()
{
  link_.close();
}


CallbackReturn RoombabotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  const CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  const auto & params = info_.hardware_parameters;

  const auto port_it = params.find("port");
  if (port_it == params.end())
  {
    RCLCPP_FATAL(logger_, "No 'port' hardware parameter provided. Aborting.");
    return CallbackReturn::FAILURE;
  }
  port_ = port_it->second;

  baud_rate_ = static_cast<unsigned int>(paramOr(params, "baud_rate", 921600.0));
  state_timeout_ = paramOr(params, "state_timeout", 0.5);
  ticks_per_revolution_ = paramOr(params, "encoder_ticks_per_revolution", 0.0);

  if (ticks_per_revolution_ <= 0.0)
  {
    RCLCPP_FATAL(
      logger_,
      "'encoder_ticks_per_revolution' must be provided and positive -- wheel position "
      "is derived from raw encoder ticks, so there is no safe default.");
    return CallbackReturn::FAILURE;
  }
  radians_per_tick_ = kTwoPi / ticks_per_revolution_;

  // Fixed two-wheel differential drive. Bail loudly rather than reading past the end
  // of the joint list later.
  if (info_.joints.size() != 2)
  {
    RCLCPP_FATAL(
      logger_, "Expected exactly 2 joints (left and right wheel), got %zu.", info_.joints.size());
    return CallbackReturn::FAILURE;
  }

  // Resolve wheel side by name so the URDF's joint ordering stops being load-bearing.
  // The wire protocol is explicitly left/right, so the mapping has to be explicit too.
  size_t left_index = std::numeric_limits<size_t>::max();
  size_t right_index = std::numeric_limits<size_t>::max();
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const std::string & name = info_.joints[i].name;
    if (name.find("left") != std::string::npos)
    {
      left_index = i;
    }
    else if (name.find("right") != std::string::npos)
    {
      right_index = i;
    }
  }
  if (left_index == std::numeric_limits<size_t>::max() ||
      right_index == std::numeric_limits<size_t>::max())
  {
    RCLCPP_FATAL(
      logger_,
      "Could not identify the left and right wheel joints by name. Joint names must "
      "contain 'left' and 'right' (got '%s' and '%s').",
      info_.joints[0].name.c_str(), info_.joints[1].name.c_str());
    return CallbackReturn::FAILURE;
  }

  // Reorder info_.joints so index 0 is always left and 1 is always right; the exported
  // interfaces then line up with kLeft/kRight without a second lookup table.
  if (left_index != kLeft)
  {
    std::swap(info_.joints[kLeft], info_.joints[kRight]);
  }

  // resize(), not reserve(). export_state_interfaces() hands out pointers into these
  // vectors, and reserve() leaves size()==0, so those pointers aimed at uninitialised
  // storage until the first assignment happened to grow the vector. That worked by
  // accident; this makes it correct.
  velocity_commands_.assign(2, 0.0);
  position_states_.assign(2, 0.0);
  velocity_states_.assign(2, 0.0);

  RCLCPP_INFO(
    logger_, "Configured for %s at %u baud, %.1f ticks/rev, joints [%s, %s]", port_.c_str(),
    baud_rate_, ticks_per_revolution_, info_.joints[kLeft].name.c_str(),
    info_.joints[kRight].name.c_str());

  return CallbackReturn::SUCCESS;
}


CallbackReturn RoombabotInterface::on_configure(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!link_.open(port_, baud_rate_, error))
  {
    RCLCPP_FATAL(logger_, "Failed to open the motor link: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }

  // The frame handler runs on SerialLink's reader thread. It does exactly one thing:
  // drop the decoded struct into the seqlock. No logging, no allocation, nothing that
  // could make it slow enough to back up the serial buffer.
  link_.start(
    [this](uint8_t type, const uint8_t * payload, size_t length)
    {
      if (type != static_cast<uint8_t>(protocol::MsgType::MotorState) ||
          length != sizeof(protocol::MotorState))
      {
        return;
      }
      protocol::MotorState state;
      std::memcpy(&state, payload, sizeof(state));
      state_slot_.store(state);
    });

  RCLCPP_INFO(logger_, "Motor link open on %s", port_.c_str());
  return CallbackReturn::SUCCESS;
}


CallbackReturn RoombabotInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  link_.close();
  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> RoombabotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> RoombabotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
  }

  return command_interfaces;
}


CallbackReturn RoombabotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating motor hardware ...");

  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  have_first_sample_ = false;
  left_tick_total_ = 0;
  right_tick_total_ = 0;
  last_write_count_ = state_slot_.writeCount();
  link_timed_out_ = false;
  last_state_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  sendCommand(0.0F, 0.0F, true);

  RCLCPP_INFO(logger_, "Motor hardware active");
  return CallbackReturn::SUCCESS;
}


CallbackReturn RoombabotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating motor hardware ...");

  // Explicitly command zero with the enable bit cleared before letting go. Closing
  // the port on this end does not stop the motors -- the ESP32's own command watchdog
  // would eventually, but "eventually" is not good enough for a robot that is moving.
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  sendCommand(0.0F, 0.0F, false);

  RCLCPP_INFO(
    logger_, "Motor link stats: rx=%lu crc_err=%lu resync=%lu tx_drop=%lu read_err=%lu",
    static_cast<unsigned long>(link_.framesReceived()),
    static_cast<unsigned long>(link_.crcErrors()), static_cast<unsigned long>(link_.resyncs()),
    static_cast<unsigned long>(link_.writeDrops()),
    static_cast<unsigned long>(link_.readErrors()));

  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type RoombabotInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  protocol::MotorState state;

  // One atomic load and a 22-byte copy. Whether a new frame arrived since last cycle
  // is answered by the seqlock's write counter rather than by touching the port.
  const uint32_t write_count = state_slot_.writeCount();
  const bool have_new_sample = (write_count != last_write_count_) && state_slot_.load(state);

  if (have_new_sample)
  {
    last_write_count_ = write_count;
    last_state_time_ = time;

    if (link_timed_out_)
    {
      RCLCPP_INFO(logger_, "Motor link recovered");
      link_timed_out_ = false;
    }

    if (!have_first_sample_)
    {
      // First sample only establishes the tick baseline -- the ESP32's counters are
      // free-running and may be anywhere, so treating them as an absolute position
      // would jump the odometry on startup.
      last_left_ticks_ = state.left_ticks;
      last_right_ticks_ = state.right_ticks;
      have_first_sample_ = true;
    }
    else
    {
      // Signed 32-bit subtraction gives the right delta across wraparound as long as
      // the robot moved less than 2^31 ticks between samples, which at 100 Hz it very
      // much did.
      left_tick_total_ += static_cast<int32_t>(state.left_ticks - last_left_ticks_);
      right_tick_total_ += static_cast<int32_t>(state.right_ticks - last_right_ticks_);
      last_left_ticks_ = state.left_ticks;
      last_right_ticks_ = state.right_ticks;
    }

    // Position from tick counts, not from integrating velocity. The old interface did
    // `position += velocity * dt` with dt measured on the Pi, which accumulated both
    // the velocity estimate's error and the control loop's scheduling jitter into
    // permanent odometry drift.
    position_states_[kLeft] = static_cast<double>(left_tick_total_) * radians_per_tick_;
    position_states_[kRight] = static_cast<double>(right_tick_total_) * radians_per_tick_;

    velocity_states_[kLeft] = static_cast<double>(state.left_rad_s);
    velocity_states_[kRight] = static_cast<double>(state.right_rad_s);

    if ((state.status & protocol::kMotorEncoderFault) != 0)
    {
      RCLCPP_WARN_THROTTLE(
        logger_, throttle_clock_, 2000, "esp32_motor reports an encoder fault");
    }
    if ((state.status & protocol::kMotorCmdWatchdog) != 0)
    {
      RCLCPP_WARN_THROTTLE(
        logger_, throttle_clock_, 2000,
        "esp32_motor command watchdog expired -- the Pi is not keeping up with its own "
        "control rate, or frames are being dropped");
    }
  }
  else if (have_first_sample_ && last_state_time_.nanoseconds() > 0)
  {
    // No new frame this cycle is completely normal -- the control loop and the ESP32
    // are not phase-locked. Only a sustained silence is a fault.
    if ((time - last_state_time_).seconds() > state_timeout_)
    {
      if (!link_timed_out_)
      {
        link_timed_out_ = true;
        RCLCPP_ERROR(
          logger_, "No MotorState from %s for %.2fs -- treating the link as down", port_.c_str(),
          state_timeout_);
      }
      // Report stopped rather than the last known velocity, so the EKF and the
      // diff_drive_controller's odometry do not keep coasting on stale data.
      velocity_states_[kLeft] = 0.0;
      velocity_states_[kRight] = 0.0;
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type RoombabotInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  sendCommand(
    static_cast<float>(velocity_commands_[kLeft]), static_cast<float>(velocity_commands_[kRight]),
    true);

  // A dropped frame is not an error worth failing the controller over -- writeFrame
  // already counted it, and the next setpoint is one cycle away. Only a port that is
  // actually gone is fatal.
  if (!link_.isOpen())
  {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}


void RoombabotInterface::sendCommand(float left_rad_s, float right_rad_s, bool enable)
{
  protocol::MotorCommand command;
  command.left_rad_s = left_rad_s;
  command.right_rad_s = right_rad_s;
  command.seq = command_sequence_++;
  command.flags = enable ? protocol::kCmdEnable : 0U;

  link_.writeFrame(protocol::MsgType::MotorCommand, &command, sizeof(command));
}

}  // namespace roombabot_firmware

PLUGINLIB_EXPORT_CLASS(roombabot_firmware::RoombabotInterface, hardware_interface::SystemInterface)
