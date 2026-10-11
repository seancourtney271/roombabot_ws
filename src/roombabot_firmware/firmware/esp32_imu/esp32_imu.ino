/*
 * roombabot esp32_imu firmware
 *
 * Reads an MPU6050 over I2C and speaks the binary protocol in ../PROTOCOL.md:
 *   out: ImuState   (0x03)  gyro rad/s + accel m/s^2 in REP-103 body axes, 100 Hz
 *   in : SysCommand (0x04)  RecalibrateImu re-runs the startup gyro bias average
 *
 * Replaces the "AX:..,AY:..,GZ:..,T:.." text sketch. What changed and why:
 *   - Binary frames at 921600, which is what the Pi's imu_bridge parses.
 *   - Axes rotated into REP-103 (x forward, y left, z up). The board is mounted
 *     upside down and turned 90 deg; see "Mounting" below.
 *   - Startup gyro bias is averaged on the ESP32 while the robot is still, and
 *     restarted if it moves during the window.
 *   - Direct register access via Wire, no MPU6050 library. Clone chips that report a
 *     different WHO_AM_I still work.
 *   - I2C failures are reported in the status byte and the sensor is re-initialised,
 *     instead of the sketch silently printing stale or zero values.
 *
 * Arduino IDE setup: ESP32 board package 3.x, board "ESP32 Dev Module". No extra
 * libraries. esp32_protocol.hpp in this folder is a COPY of
 * ../../include/roombabot_firmware/esp32_protocol.hpp -- if that file changes,
 * copy it here again.
 *
 * Wiring (note SDA is on GPIO23, not the ESP32's default GPIO21):
 *   MPU6050 SDA -> GPIO23   MPU6050 SCL -> GPIO22   VCC -> 3.3V   GND -> GND
 */

#include <Arduino.h>
#include <Wire.h>
#include "esp32_protocol.hpp"

namespace proto = roombabot_firmware::protocol;

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
constexpr uint8_t kSdaPin = 23;
constexpr uint8_t kSclPin = 22;
constexpr uint32_t kI2cHz = 400000;
// 0x68 with AD0 low, 0x69 with AD0 high. initMpu() tries both and remembers which
// one answered.
constexpr uint8_t kMpuAddresses[2] = {0x68, 0x69};
uint8_t g_mpu_address = kMpuAddresses[0];

// MPU6050 registers
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;  // start of the 14-byte accel/temp/gyro block
constexpr uint8_t kRegPwrMgmt1 = 0x6B;

// Ranges. +/-500 deg/s covers the robot's 2 rad/s (115 deg/s) yaw limit with room
// for bumps; +/-4 g keeps a bump from clipping accel.
constexpr uint8_t kGyroConfig = 0x08;    // FS_SEL=1, +/-500 deg/s
constexpr float kGyroLsbPerDps = 65.5f;
constexpr uint8_t kAccelConfig = 0x08;   // AFS_SEL=1, +/-4 g
constexpr float kAccelLsbPerG = 8192.0f;
constexpr float kGravity = 9.80665f;
// DLPF_CFG=3: 44 Hz bandwidth, 1 kHz internal sample rate. Reading at 100 Hz then
// always gets a fresh, already-anti-aliased sample.
constexpr uint8_t kDlpfConfig = 0x03;

constexpr float kGyroScale = (PI / 180.0f) / kGyroLsbPerDps;  // LSB -> rad/s
constexpr float kAccelScale = kGravity / kAccelLsbPerG;        // LSB -> m/s^2

// ---------------------------------------------------------------------------
// Mounting: body axis i = kAxisSign[i] * sensor axis kAxisFrom[i]  (0=x 1=y 2=z)
//
// This robot's board is mounted upside down AND turned 90 deg, with its X arrow
// pointing to the robot's left. Measured on the bench 2026-10-11:
//   body x (forward) = +sensor y
//   body y (left)    = +sensor x
//   body z (up)      = -sensor z
// Expected readings in `ros2 topic echo /imu/data` (accel reads the push AGAINST
// gravity, so a tipped-down axis reads negative):
//   - Robot flat and still:   linear_acceleration.z ~ +9.8
//   - Nose tipped down:       linear_acceleration.x goes NEGATIVE
//   - Left side tipped down:  linear_acceleration.y goes NEGATIVE
//   - Rotate robot to the LEFT (counter-clockwise from above): angular_velocity.z > 0
// The mapping must stay a proper rotation (determinant +1): if you flip one sign,
// you must flip or swap another.
// ---------------------------------------------------------------------------
constexpr uint8_t kAxisFrom[3] = {1, 0, 2};
constexpr float kAxisSign[3] = {+1.0f, +1.0f, -1.0f};

// ---------------------------------------------------------------------------
// Timing / calibration
// ---------------------------------------------------------------------------
constexpr uint32_t kSamplePeriodUs = 10000;  // 100 Hz
constexpr int kCalibrationSamples = 200;     // 2 s
// If any gyro axis spans more than this during calibration, the robot was moving:
// start over. 0.05 rad/s ~ 3 deg/s, well above the MPU6050's noise at 44 Hz DLPF.
constexpr float kCalibrationMaxSpread = 0.05f;
// Give up being picky after this many restarts and use what we have; the Pi runs its
// own residual-bias pass on top.
constexpr int kCalibrationMaxRestarts = 5;
constexpr int kFailuresBeforeReinit = 3;
constexpr uint32_t kReinitPeriodMs = 500;

// ---------------------------------------------------------------------------
// Framing (identical to esp32_motor.ino)
// ---------------------------------------------------------------------------
class FrameParser
{
public:
  bool feed(uint8_t byte)
  {
    switch (state_)
    {
      case State::Sync0:
        if (byte == proto::kSync0) state_ = State::Sync1;
        return false;
      case State::Sync1:
        if (byte == proto::kSync1) state_ = State::Type;
        else if (byte != proto::kSync0) state_ = State::Sync0;
        return false;
      case State::Type:
        length_ = proto::payloadSize(byte);
        if (length_ == 0)
        {
          state_ = (byte == proto::kSync0) ? State::Sync1 : State::Sync0;
          return false;
        }
        type_ = byte;
        index_ = 0;
        state_ = State::Payload;
        return false;
      case State::Payload:
        payload_[index_++] = byte;
        if (index_ == length_) state_ = State::Crc;
        return false;
      case State::Crc:
      {
        state_ = State::Sync0;
        const uint8_t crc = proto::crc8(payload_, length_, proto::crc8(&type_, 1));
        return crc == byte;
      }
    }
    return false;
  }

  uint8_t type() const { return type_; }
  const uint8_t * payload() const { return payload_; }

private:
  enum class State : uint8_t { Sync0, Sync1, Type, Payload, Crc };
  State state_ = State::Sync0;
  uint8_t type_ = 0;
  size_t length_ = 0;
  size_t index_ = 0;
  uint8_t payload_[proto::kMaxPayloadSize];
};

template <typename T>
void sendFrame(proto::MsgType type, const T & payload)
{
  constexpr size_t kFrameSize = sizeof(T) + proto::kFrameOverhead;
  uint8_t frame[kFrameSize];
  frame[0] = proto::kSync0;
  frame[1] = proto::kSync1;
  frame[2] = static_cast<uint8_t>(type);
  memcpy(frame + 3, &payload, sizeof(T));
  frame[kFrameSize - 1] = proto::crc8(frame + 2, sizeof(T) + 1);
  if (Serial.availableForWrite() >= static_cast<int>(kFrameSize))
  {
    Serial.write(frame, kFrameSize);
  }
}

// ---------------------------------------------------------------------------
// MPU6050
// ---------------------------------------------------------------------------
bool writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(g_mpu_address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t * out, size_t length)
{
  Wire.beginTransmission(g_mpu_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(g_mpu_address, length) != length) return false;
  for (size_t i = 0; i < length; ++i) out[i] = Wire.read();
  return true;
}

bool initMpuAt(uint8_t address)
{
  g_mpu_address = address;
  if (!writeRegister(kRegPwrMgmt1, 0x80)) return false;  // device reset
  delay(100);
  if (!writeRegister(kRegPwrMgmt1, 0x01)) return false;  // wake, PLL on gyro X
  delay(10);
  if (!writeRegister(kRegSmplrtDiv, 0x00)) return false;
  if (!writeRegister(kRegConfig, kDlpfConfig)) return false;
  if (!writeRegister(kRegGyroConfig, kGyroConfig)) return false;
  if (!writeRegister(kRegAccelConfig, kAccelConfig)) return false;

  // Read back one config register: proves the writes landed, not just that something
  // ACKed the address.
  uint8_t check = 0;
  return readRegisters(kRegGyroConfig, &check, 1) && check == kGyroConfig;
}

bool initMpu()
{
  for (uint8_t address : kMpuAddresses)
  {
    if (initMpuAt(address)) return true;
  }
  return false;
}

// Reads one sample, already scaled and rotated into body axes.
bool readMpu(float gyro[3], float accel[3])
{
  uint8_t raw[14];
  if (!readRegisters(kRegAccelXoutH, raw, sizeof(raw))) return false;

  float sensor_accel[3];
  float sensor_gyro[3];
  for (int i = 0; i < 3; ++i)
  {
    // Big-endian on the chip. raw[6..7] is temperature, skipped.
    sensor_accel[i] = static_cast<int16_t>((raw[2 * i] << 8) | raw[2 * i + 1]) * kAccelScale;
    sensor_gyro[i] = static_cast<int16_t>((raw[8 + 2 * i] << 8) | raw[9 + 2 * i]) * kGyroScale;
  }
  for (int i = 0; i < 3; ++i)
  {
    accel[i] = kAxisSign[i] * sensor_accel[kAxisFrom[i]];
    gyro[i] = kAxisSign[i] * sensor_gyro[kAxisFrom[i]];
  }
  return true;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
FrameParser g_parser;
bool g_initialised = false;
int g_consecutive_failures = 0;
uint32_t g_last_reinit_ms = 0;

bool g_calibrating = true;
int g_cal_count = 0;
int g_cal_restarts = 0;
float g_cal_sum[3];
float g_cal_min[3];
float g_cal_max[3];
float g_bias[3] = {0.0f, 0.0f, 0.0f};

uint32_t g_last_sample_us = 0;
uint8_t g_seq = 0;

void startCalibration(bool count_as_restart)
{
  g_calibrating = true;
  g_cal_count = 0;
  if (count_as_restart) ++g_cal_restarts;
  else g_cal_restarts = 0;
  for (int i = 0; i < 3; ++i)
  {
    g_cal_sum[i] = 0.0f;
    g_cal_min[i] = INFINITY;
    g_cal_max[i] = -INFINITY;
  }
}

void calibrationStep(const float gyro[3])
{
  bool moved = false;
  for (int i = 0; i < 3; ++i)
  {
    g_cal_sum[i] += gyro[i];
    g_cal_min[i] = min(g_cal_min[i], gyro[i]);
    g_cal_max[i] = max(g_cal_max[i], gyro[i]);
    moved |= (g_cal_max[i] - g_cal_min[i]) > kCalibrationMaxSpread;
  }
  if (moved && g_cal_restarts < kCalibrationMaxRestarts)
  {
    startCalibration(true);
    return;
  }
  if (++g_cal_count >= kCalibrationSamples)
  {
    for (int i = 0; i < 3; ++i) g_bias[i] = g_cal_sum[i] / g_cal_count;
    g_calibrating = false;
  }
}

void handleFrame(uint8_t type, const uint8_t * payload)
{
  if (type != static_cast<uint8_t>(proto::MsgType::SysCommand)) return;
  proto::SysCommandPayload cmd;
  memcpy(&cmd, payload, sizeof(cmd));
  if (cmd.command == static_cast<uint8_t>(proto::SysCommand::RecalibrateImu))
  {
    startCalibration(false);
  }
}

void readSerial()
{
  for (int n = Serial.available(); n > 0; --n)
  {
    if (g_parser.feed(static_cast<uint8_t>(Serial.read())))
    {
      handleFrame(g_parser.type(), g_parser.payload());
    }
  }
}

void sampleStep(uint32_t now_us)
{
  proto::ImuState state{};
  state.esp_micros = now_us;

  if (!g_initialised)
  {
    // Try to bring the sensor back, but not every cycle -- initMpu() blocks ~110 ms.
    if (millis() - g_last_reinit_ms >= kReinitPeriodMs)
    {
      g_last_reinit_ms = millis();
      g_initialised = initMpu();
      if (g_initialised)
      {
        g_consecutive_failures = 0;
        startCalibration(false);
      }
    }
    if (!g_initialised)
    {
      // Keep talking so the Pi logs "check the I2C wiring" rather than "no frames".
      state.seq = g_seq++;
      state.status = proto::kImuNotInitialised;
      sendFrame(proto::MsgType::ImuState, state);
      return;
    }
  }

  float gyro[3];
  float accel[3];
  if (!readMpu(gyro, accel))
  {
    // One bad transaction: skip the frame (the Pi tolerates gaps). Several in a row:
    // assume the sensor is gone and re-initialise it.
    if (++g_consecutive_failures >= kFailuresBeforeReinit)
    {
      g_initialised = false;
      g_last_reinit_ms = millis();
      state.seq = g_seq++;
      state.status = proto::kImuNotInitialised | proto::kImuReadFault;
      sendFrame(proto::MsgType::ImuState, state);
    }
    return;
  }
  g_consecutive_failures = 0;

  if (g_calibrating) calibrationStep(gyro);

  state.gyro_x = gyro[0] - g_bias[0];
  state.gyro_y = gyro[1] - g_bias[1];
  state.gyro_z = gyro[2] - g_bias[2];
  state.accel_x = accel[0];
  state.accel_y = accel[1];
  state.accel_z = accel[2];
  state.seq = g_seq++;
  state.status = g_calibrating ? proto::kImuCalibrating : 0;
  sendFrame(proto::MsgType::ImuState, state);
}

void setup()
{
  Serial.setRxBufferSize(256);
  Serial.begin(921600);

  Wire.begin(kSdaPin, kSclPin, kI2cHz);
  // A wedged bus must not stall the loop for the default 50 ms.
  Wire.setTimeOut(5);

  g_last_reinit_ms = millis();
  g_initialised = initMpu();
  startCalibration(false);
  g_last_sample_us = micros();
}

void loop()
{
  readSerial();

  const uint32_t now = micros();
  if (now - g_last_sample_us < kSamplePeriodUs) return;
  g_last_sample_us += kSamplePeriodUs;
  if (now - g_last_sample_us > kSamplePeriodUs) g_last_sample_us = now;

  sampleStep(now);
}
