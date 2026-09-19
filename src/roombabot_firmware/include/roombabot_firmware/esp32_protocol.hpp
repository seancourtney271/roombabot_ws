#ifndef ROOMBABOT_FIRMWARE__ESP32_PROTOCOL_HPP
#define ROOMBABOT_FIRMWARE__ESP32_PROTOCOL_HPP

// Wire protocol shared by the Pi and both ESP32s (esp32_motor, esp32_imu).
//
// The previous robot spoke an ASCII protocol ("rp01.23,ln00.45,") over libserial's
// ReadLine. That cost a getline + stod per wheel per cycle and, worse, blocked the
// 100 Hz ros2_control loop for up to the read timeout every single cycle. This
// protocol is fixed-length binary instead: a frame is a memcpy on both ends, and the
// length is implied by the type byte, so a corrupted length field can never desync
// the parser for more than one frame.
//
// Endianness/layout: both the Pi (aarch64) and the ESP32 (Xtensa LX6/LX7 and RISC-V)
// are little-endian with IEEE-754 floats and the same natural alignment for these
// scalar types, so the packed structs below can be memcpy'd straight to and from the
// wire with no per-field serialization. static_asserts below pin the sizes so a
// compiler that disagrees fails the build rather than the robot.
//
// Floats (not scaled fixed-point integers) are used for velocities deliberately: at
// 100 Hz the difference is ~400 B/s, which is nothing over USB CDC, and it removes a
// whole class of silent scaling/range bugs. Wheel *position* is sent as raw encoder
// ticks instead, so the Pi integrates nothing and position never drifts.

#include <cstddef>
#include <cstdint>
#include <array>

namespace roombabot_firmware
{
namespace protocol
{

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------
//
//   [0] SYNC0 = 0xA5
//   [1] SYNC1 = 0x5A
//   [2] TYPE
//   [3 .. 3+N-1] PAYLOAD (N fixed per TYPE)
//   [3+N] CRC8 over TYPE and PAYLOAD
//
inline constexpr uint8_t kSync0 = 0xA5;
inline constexpr uint8_t kSync1 = 0x5A;

// SYNC0 + SYNC1 + TYPE + CRC8
inline constexpr size_t kFrameOverhead = 4;

enum class MsgType : uint8_t
{
  // Pi -> ESP32
  MotorCommand = 0x01,
  SysCommand = 0x04,

  // ESP32 -> Pi
  MotorState = 0x02,
  ImuState = 0x03,
};

// Status bit flags carried in MotorState::status.
enum MotorStatusBits : uint8_t
{
  kMotorEStop = 1u << 0,        // hardware/software e-stop asserted on the ESP32
  kMotorCmdWatchdog = 1u << 1,  // no MotorCommand within the ESP32's watchdog window
  kMotorEncoderFault = 1u << 2, // encoder read failed or is implausible
};

// Status bit flags carried in ImuState::status.
enum ImuStatusBits : uint8_t
{
  kImuNotInitialised = 1u << 0, // I2C bring-up of the MPU6050 has not succeeded
  kImuReadFault = 1u << 1,      // last I2C transaction failed
  kImuCalibrating = 1u << 2,    // ESP32 is still averaging its startup gyro bias
};

// Flags carried in MotorCommand::flags.
enum MotorCommandFlags : uint8_t
{
  kCmdEnable = 1u << 0, // clear to coast/brake the motors regardless of the setpoints
};

enum class SysCommand : uint8_t
{
  Ping = 0,
  ResetEncoders = 1,
  RecalibrateImu = 2,
};

// ---------------------------------------------------------------------------
// Payloads
// ---------------------------------------------------------------------------

#pragma pack(push, 1)

// Pi -> esp32_motor, sent once per ros2_control cycle (100 Hz).
struct MotorCommand
{
  float left_rad_s;   // commanded left wheel angular velocity
  float right_rad_s;  // commanded right wheel angular velocity
  uint8_t seq;        // wraps freely; lets the ESP32 spot dropped commands
  uint8_t flags;      // MotorCommandFlags
};

// esp32_motor -> Pi, at whatever rate the firmware chooses (100 Hz recommended).
struct MotorState
{
  // Signed tick counters, free-running. The Pi differences successive values into a
  // 64-bit accumulator, so wraparound at INT32_MIN/MAX is handled and never needs a
  // reset. Position comes from these, never from integrating velocity.
  int32_t left_ticks;
  int32_t right_ticks;
  // Velocity measured on the ESP32, where the encoder edge timestamps actually live.
  float left_rad_s;
  float right_rad_s;
  uint32_t esp_micros;  // ESP32 micros() at sample time -- for jitter diagnostics
  uint8_t seq;
  uint8_t status;  // MotorStatusBits
};

// esp32_imu -> Pi, at whatever rate the firmware chooses (100 Hz recommended).
// Already converted to REP-103 body axes (x forward, y left, z up) and bias-corrected
// on the ESP32; the Pi applies only an optional residual bias it measures at startup.
struct ImuState
{
  float gyro_x, gyro_y, gyro_z;     // rad/s
  float accel_x, accel_y, accel_z;  // m/s^2
  uint32_t esp_micros;
  uint8_t seq;
  uint8_t status;  // ImuStatusBits
};

// Pi -> either ESP32.
struct SysCommandPayload
{
  uint8_t command;  // SysCommand
  uint8_t seq;
};

#pragma pack(pop)

static_assert(sizeof(MotorCommand) == 10, "MotorCommand must be packed to 10 bytes");
static_assert(sizeof(MotorState) == 22, "MotorState must be packed to 22 bytes");
static_assert(sizeof(ImuState) == 30, "ImuState must be packed to 30 bytes");
static_assert(sizeof(SysCommandPayload) == 2, "SysCommandPayload must be packed to 2 bytes");

// Largest payload any type carries -- sizes the parser's frame buffer.
inline constexpr size_t kMaxPayloadSize = sizeof(ImuState);
inline constexpr size_t kMaxFrameSize = kMaxPayloadSize + kFrameOverhead;

// Payload length for a type byte, or 0 if the type is unknown. Driving the parser
// from this table (rather than a length field on the wire) means a bit-flip can
// corrupt at most the frame it lands in.
inline constexpr size_t payloadSize(uint8_t type)
{
  switch (static_cast<MsgType>(type))
  {
    case MsgType::MotorCommand:
      return sizeof(MotorCommand);
    case MsgType::MotorState:
      return sizeof(MotorState);
    case MsgType::ImuState:
      return sizeof(ImuState);
    case MsgType::SysCommand:
      return sizeof(SysCommandPayload);
    default:
      return 0;
  }
}

// ---------------------------------------------------------------------------
// CRC-8/ATM (poly 0x07, init 0x00, no reflection, no final xor)
// ---------------------------------------------------------------------------
//
// Table is built at compile time: 256 bytes stays resident in L1, and checking a
// 26-byte frame costs 26 table lookups instead of 208 shift/xor steps. At 100 Hz on
// two links that is the difference between "free" and "still basically free", but it
// also keeps the reader thread's inner loop tight enough to stay out of the way of
// the control loop.
inline constexpr std::array<uint8_t, 256> makeCrc8Table()
{
  std::array<uint8_t, 256> table{};
  for (size_t i = 0; i < 256; ++i)
  {
    uint8_t crc = static_cast<uint8_t>(i);
    for (int bit = 0; bit < 8; ++bit)
    {
      crc = static_cast<uint8_t>((crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1));
    }
    table[i] = crc;
  }
  return table;
}

inline constexpr std::array<uint8_t, 256> kCrc8Table = makeCrc8Table();

inline uint8_t crc8(const uint8_t * data, size_t length, uint8_t crc = 0x00)
{
  for (size_t i = 0; i < length; ++i)
  {
    crc = kCrc8Table[static_cast<uint8_t>(crc ^ data[i])];
  }
  return crc;
}

}  // namespace protocol
}  // namespace roombabot_firmware

#endif  // ROOMBABOT_FIRMWARE__ESP32_PROTOCOL_HPP
