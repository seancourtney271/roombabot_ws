// Bridges the esp32_imu board onto a sensor_msgs/Imu topic.
//
// Replaces two nodes from the old robot:
//
//  * mpu6050_driver.py, which did twelve individual smbus byte reads over I2C from
//    Python at 100 Hz. That work now happens on the ESP32; the Pi parses one 34-byte
//    frame instead, in C++.
//  * imu_republisher, which existed only to forward imu/out to the topic the EKF was
//    configured to read. That was a whole extra process, a second serialization, and
//    a second DDS hop for a message that was never modified. This node publishes
//    straight onto the EKF's input topic.
//
// Messages are published from the serial reader thread the instant a frame is
// decoded, rather than polled by a timer -- no fixed-rate wakeup on an already-busy
// Pi, and no up-to-one-period latency added on top of the ESP32's own sample rate.

#include "roombabot_firmware/esp32_protocol.hpp"
#include "roombabot_firmware/serial_link.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cstring>
#include <memory>
#include <string>

namespace roombabot_firmware
{

class ImuBridge : public rclcpp::Node
{
public:
  ImuBridge()
  : Node("imu_bridge")
  {
    port_ = declare_parameter<std::string>("port", "/dev/esp32_imu");
    baud_rate_ = static_cast<unsigned int>(declare_parameter<int>("baud_rate", 921600));
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    const std::string topic = declare_parameter<std::string>("imu_topic", "imu/data");

    // The ESP32 already removes the bias it measures at its own boot. This second,
    // optional pass on the Pi catches the residual -- most usefully when the ESP32 was
    // powered up while the robot was being carried, so its own calibration window was
    // not actually stationary. Left-over gyro Z bias integrates directly into heading
    // drift, which is the one IMU error this robot genuinely cannot tolerate.
    calibration_samples_ = declare_parameter<int>("calibration_samples", 200);

    // Diagonal covariances published with every message. The EKF only consumes vyaw
    // (see ekf.yaml), but publishing honest numbers keeps the topic usable by anything
    // else that subscribes.
    const double gyro_variance = declare_parameter<double>("gyro_variance", 4e-4);
    const double accel_variance = declare_parameter<double>("accel_variance", 3e-4);

    // Prebuilt once; the reader thread only overwrites the fields that change.
    message_.header.frame_id = frame_id_;
    // No onboard sensor fusion, so there is no valid absolute orientation. -1 in
    // element 0 is the sensor_msgs/Imu convention telling consumers to ignore the
    // orientation field entirely rather than treat (0,0,0,0) as a real quaternion.
    message_.orientation_covariance[0] = -1.0;
    message_.angular_velocity_covariance = {gyro_variance, 0.0, 0.0,
                                            0.0, gyro_variance, 0.0,
                                            0.0, 0.0, gyro_variance};
    message_.linear_acceleration_covariance = {accel_variance, 0.0, 0.0,
                                               0.0, accel_variance, 0.0,
                                               0.0, 0.0, accel_variance};

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS());

    std::string error;
    if (!link_.open(port_, baud_rate_, error))
    {
      RCLCPP_FATAL(get_logger(), "Failed to open the IMU link: %s", error.c_str());
      throw std::runtime_error(error);
    }

    link_.start(
      [this](uint8_t type, const uint8_t * payload, size_t length)
      {
        onFrame(type, payload, length);
      });

    // One-second heartbeat purely for diagnostics; it does no work on the data path.
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this]() { reportDiagnostics(); });

    RCLCPP_INFO(
      get_logger(), "IMU bridge publishing %s from %s (frame '%s')", topic.c_str(), port_.c_str(),
      frame_id_.c_str());
    if (calibration_samples_ > 0)
    {
      RCLCPP_INFO(
        get_logger(), "Averaging %d samples for residual gyro bias -- keep the robot still.",
        calibration_samples_);
    }
  }

  ~ImuBridge() override { link_.close(); }

private:
  void onFrame(uint8_t type, const uint8_t * payload, size_t length)
  {
    if (type != static_cast<uint8_t>(protocol::MsgType::ImuState) ||
        length != sizeof(protocol::ImuState))
    {
      return;
    }

    protocol::ImuState state;
    std::memcpy(&state, payload, sizeof(state));

    // The CP2102 link replays the same frame 100-200 times right after the port is
    // opened. Without this, that burst alone fills the residual-bias window below
    // with a single sample. The firmware bumps seq on every frame, so a repeat is
    // never new data.
    if (have_sequence_ && state.seq == last_sequence_)
    {
      return;
    }
    have_sequence_ = true;
    last_sequence_ = state.seq;

    if ((state.status & protocol::kImuNotInitialised) != 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), throttle_clock_, 5000,
        "esp32_imu reports the MPU6050 is not initialised -- check the I2C wiring");
      return;
    }
    if ((state.status & protocol::kImuCalibrating) != 0)
    {
      return;  // the board is still averaging its own bias; nothing useful to publish
    }

    if (samples_collected_ < calibration_samples_)
    {
      bias_x_ += state.gyro_x;
      bias_y_ += state.gyro_y;
      bias_z_ += state.gyro_z;
      if (++samples_collected_ == calibration_samples_)
      {
        bias_x_ /= static_cast<double>(calibration_samples_);
        bias_y_ /= static_cast<double>(calibration_samples_);
        bias_z_ /= static_cast<double>(calibration_samples_);
        RCLCPP_INFO(
          get_logger(), "Residual gyro bias: x=%.5f y=%.5f z=%.5f rad/s", bias_x_, bias_y_,
          bias_z_);
      }
      return;
    }

    // Stamped on arrival rather than from state.esp_micros: the two clocks have no
    // common epoch, and over USB CDC the transport delay is well under a millisecond,
    // which is far smaller than the EKF's 30 Hz update period. esp_micros is carried
    // anyway so firmware-side sampling jitter can be measured if it ever matters.
    message_.header.stamp = now();

    message_.angular_velocity.x = state.gyro_x - bias_x_;
    message_.angular_velocity.y = state.gyro_y - bias_y_;
    message_.angular_velocity.z = state.gyro_z - bias_z_;

    message_.linear_acceleration.x = state.accel_x;
    message_.linear_acceleration.y = state.accel_y;
    message_.linear_acceleration.z = state.accel_z;

    publisher_->publish(message_);
  }

  void reportDiagnostics()
  {
    const uint64_t frames = link_.framesReceived();
    if (frames == last_frame_count_)
    {
      RCLCPP_WARN(
        get_logger(), "No IMU frames from %s in the last 5s -- is esp32_imu powered?",
        port_.c_str());
    }
    else if (link_.crcErrors() != last_crc_errors_)
    {
      RCLCPP_WARN(
        get_logger(), "IMU link CRC errors: %lu total (%lu new)",
        static_cast<unsigned long>(link_.crcErrors()),
        static_cast<unsigned long>(link_.crcErrors() - last_crc_errors_));
    }
    last_frame_count_ = frames;
    last_crc_errors_ = link_.crcErrors();
  }

  SerialLink link_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  sensor_msgs::msg::Imu message_;
  std::string port_;
  std::string frame_id_;
  unsigned int baud_rate_{921600};

  int calibration_samples_{200};
  int samples_collected_{0};
  double bias_x_{0.0};
  double bias_y_{0.0};
  double bias_z_{0.0};

  uint64_t last_frame_count_{0};
  uint64_t last_crc_errors_{0};

  bool have_sequence_{false};
  uint8_t last_sequence_{0};
};

}  // namespace roombabot_firmware


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<roombabot_firmware::ImuBridge>());
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("imu_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
