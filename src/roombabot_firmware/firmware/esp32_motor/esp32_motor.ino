/*
 * roombabot esp32_motor firmware
 *
 * Speaks the binary protocol in ../PROTOCOL.md:
 *   in : MotorCommand (0x01)  wheel velocity setpoints in rad/s, 100 Hz from the Pi
 *   out: MotorState   (0x02)  cumulative encoder ticks + measured rad/s, 100 Hz
 *
 * Replaces robot_control_esp32.ino. What changed and why:
 *   - Binary frames at 921600 instead of "rp1.23,ln0.45," text at 115200. The Pi's
 *     hardware interface only understands the binary frames.
 *   - Reports cumulative signed tick counts. The Pi derives wheel position from these,
 *     so odometry no longer depends on integrating a noisy velocity.
 *   - x4 quadrature decode (every edge on A and B) instead of x1 (A rising only):
 *     4x the resolution, and direction comes from the decode itself rather than from
 *     a String assigned inside an ISR.
 *   - Signed PI + feedforward controller written out here instead of PID_v1. PID_v1
 *     was configured for 0..255 output with direction handled separately, so the loop
 *     could never push back against an overshoot through zero.
 *   - 100 Hz control loop (was 10 Hz) and a 250 ms command watchdog (was 500 ms).
 *   - Nothing but protocol frames is ever written to Serial. One stray println costs
 *     the Pi a resync.
 *
 * Arduino IDE setup: ESP32 board package 3.x, board "ESP32 Dev Module". No extra
 * libraries. esp32_protocol.hpp in this folder is a COPY of
 * ../../include/roombabot_firmware/esp32_protocol.hpp -- if that file changes,
 * copy it here again.
 *
 * Wiring is unchanged from robot_control_esp32.ino, except that the right encoder's
 * A/B match encoder_calibration.ino (A=35, B=34), which is the assignment that gives
 * increasing ticks when driving forward.
 *
 *   GPIO25 -> L298N ENA (right PWM)    GPIO14 -> L298N ENB (left PWM)
 *   GPIO26 -> L298N IN1 (right dir)    GPIO13 -> L298N IN3 (left dir)
 *   GPIO27 -> L298N IN2 (right dir)    GPIO4  -> L298N IN4 (left dir)
 *   GPIO35 <- right encoder A (green)  GPIO36 <- left encoder A (green)
 *   GPIO34 <- right encoder B (white)  GPIO39 <- left encoder B (white)
 */

#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp32_protocol.hpp"

namespace proto = roombabot_firmware::protocol;

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
constexpr uint8_t kRightPwmPin = 25;
constexpr uint8_t kRightIn1Pin = 26;
constexpr uint8_t kRightIn2Pin = 27;
constexpr uint8_t kLeftPwmPin = 14;
constexpr uint8_t kLeftIn3Pin = 13;
constexpr uint8_t kLeftIn4Pin = 4;

constexpr uint8_t kRightEncA = 35;
constexpr uint8_t kRightEncB = 34;
constexpr uint8_t kLeftEncA = 36;
constexpr uint8_t kLeftEncB = 39;

// The encoder ISRs read GPIO_IN1_REG directly (fast and IRAM-safe), which only
// covers GPIO32-39. All four encoder pins are input-only pins in that range anyway.
static_assert(kRightEncA >= 32 && kRightEncB >= 32 && kLeftEncA >= 32 && kLeftEncB >= 32,
              "encoder ISRs assume GPIO32-39");

// ---------------------------------------------------------------------------
// Sign conventions -- PROTOCOL.md: positive rad/s and increasing ticks must both
// mean that wheel is driving the robot FORWARD. Flip these on the bench rather than
// rewiring.
// ---------------------------------------------------------------------------
constexpr bool kLeftEncoderReversed = false;
constexpr bool kRightEncoderReversed = false;
// false: IN1=HIGH/IN2=LOW (right) and IN3=HIGH/IN4=LOW (left) drive forward, which is
// what robot_control_esp32.ino assumed.
constexpr bool kLeftMotorReversed = false;
constexpr bool kRightMotorReversed = false;

// ---------------------------------------------------------------------------
// Encoder scaling
// ---------------------------------------------------------------------------
// x4 counts per wheel revolution, measured with encoder_calibration.ino
// (left 2248.4, right 2241.6). MUST match encoder_ticks_per_revolution on the Pi
// (real_robot.launch.py). This firmware uses it only to turn ticks into the measured
// rad/s that the PI loop and the Pi's velocity state see.
constexpr float kTicksPerRev = 2245.0f;
constexpr float kRadPerTick = 2.0f * PI / kTicksPerRev;

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
constexpr uint32_t kControlPeriodUs = 10000;  // 100 Hz control + MotorState
constexpr uint32_t kCommandTimeoutMs = 250;   // watchdog, per PROTOCOL.md
// Velocity is measured over the last kVelocityWindow control periods (40 ms). One
// period alone would be only ~3.5 ticks at 1 rad/s -- too coarse to control on.
constexpr int kVelocityWindow = 4;

// ---------------------------------------------------------------------------
// PWM / controller tuning (all in 8-bit PWM counts)
// ---------------------------------------------------------------------------
constexpr uint32_t kPwmFreqHz = 20000;  // above hearing; L298N copes fine
constexpr uint8_t kPwmBits = 8;
constexpr float kPwmMax = 255.0f;

// Output = feedforward + P + I, then a fixed offset to get past static friction.
//   kFreeSpeedRadS: wheel speed at full PWM, unloaded. Feedforward assumes speed is
//     roughly linear in PWM, so this does most of the work and the PI only trims.
//   kPwmDeadband: the PWM below which the wheels don't turn at all.
// These are starting guesses. To tune: put the robot on blocks, drive with the
// joystick, and compare commanded vs measured wheel speed
// (/roombabot_controller/cmd_vel_out vs velocity in /joint_states). Set
// kPwmDeadband first (lowest PWM that reliably starts a wheel), then kFreeSpeedRadS
// until steady-state error is small with kKi = 0, then bring kKi back.
constexpr float kFreeSpeedRadS = 20.0f;
constexpr float kFeedforward = (kPwmMax - 40.0f) / kFreeSpeedRadS;
constexpr float kPwmDeadband = 40.0f;
constexpr float kKp = 20.0f;          // PWM per rad/s of error
constexpr float kKi = 200.0f;         // PWM per rad/s of error, per second
// Full PWM range. A cap of 100 left the loop ~40% short of setpoint on carpet
// (2.4 vs 4.2 rad/s, 2026-10-11): the motors need far more than the feedforward
// guess under load, and the integrator is what has to supply it. Windup is handled
// by not integrating while the output is saturated (see update()).
constexpr float kIntegralLimit = kPwmMax;
// Setpoints smaller than this are treated as "stop": motors off, integrator cleared.
constexpr float kMinCommandRadS = 0.05f;

// Encoder fault: commanding near-full PWM for this long with no ticks at all means the
// encoder is unplugged or dead. Without this the PI winds up to full speed forever.
constexpr float kFaultPwm = 200.0f;
constexpr uint32_t kFaultTimeMs = 500;

// ---------------------------------------------------------------------------
// Encoders (x4 quadrature)
// ---------------------------------------------------------------------------
// Index: (previous AB << 2) | new AB. Value: tick delta. 0 for no change or an illegal
// two-bit jump (a missed edge) -- losing a tick is better than counting the wrong way.
static const int8_t kQuadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0,
};

// 32-bit aligned writes from the ISR and reads from loop() are atomic on the ESP32,
// and both run on core 1, so no lock is needed.
volatile int32_t g_left_ticks = 0;
volatile int32_t g_right_ticks = 0;
volatile uint8_t g_left_ab = 0;
volatile uint8_t g_right_ab = 0;

static inline uint8_t IRAM_ATTR readAB(uint8_t pin_a, uint8_t pin_b)
{
  const uint32_t in = REG_READ(GPIO_IN1_REG);
  return static_cast<uint8_t>((((in >> (pin_a - 32)) & 1u) << 1) | ((in >> (pin_b - 32)) & 1u));
}

void IRAM_ATTR leftEncoderIsr()
{
  const uint8_t ab = readAB(kLeftEncA, kLeftEncB);
  const int8_t delta = kQuadTable[(g_left_ab << 2) | ab];
  g_left_ticks += kLeftEncoderReversed ? -delta : delta;
  g_left_ab = ab;
}

void IRAM_ATTR rightEncoderIsr()
{
  const uint8_t ab = readAB(kRightEncA, kRightEncB);
  const int8_t delta = kQuadTable[(g_right_ab << 2) | ab];
  g_right_ticks += kRightEncoderReversed ? -delta : delta;
  g_right_ab = ab;
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------
class FrameParser
{
public:
  // Feed one byte. Returns true when a complete, CRC-valid frame has just arrived;
  // type()/payload() then describe it until the next call.
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
  size_t length() const { return length_; }

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
  // One write per frame. Drop rather than block if the UART is backed up -- the next
  // frame is 10 ms away and the control loop must not stall.
  if (Serial.availableForWrite() >= static_cast<int>(kFrameSize))
  {
    Serial.write(frame, kFrameSize);
  }
}

// ---------------------------------------------------------------------------
// Motors
// ---------------------------------------------------------------------------
struct Motor
{
  uint8_t pwm_pin;
  uint8_t fwd_pin;   // HIGH (with rev_pin LOW) = forward, before kXMotorReversed
  uint8_t rev_pin;
  bool reversed;

  // pwm in [-255, 255]; 0 coasts (both direction pins LOW).
  void drive(float pwm) const
  {
    if (reversed) pwm = -pwm;
    const uint32_t duty = static_cast<uint32_t>(constrain(fabsf(pwm), 0.0f, kPwmMax));
    if (duty == 0)
    {
      digitalWrite(fwd_pin, LOW);
      digitalWrite(rev_pin, LOW);
    }
    else
    {
      digitalWrite(fwd_pin, pwm > 0 ? HIGH : LOW);
      digitalWrite(rev_pin, pwm > 0 ? LOW : HIGH);
    }
    ledcWrite(pwm_pin, duty);
  }
};

const Motor kLeftMotor{kLeftPwmPin, kLeftIn3Pin, kLeftIn4Pin, kLeftMotorReversed};
const Motor kRightMotor{kRightPwmPin, kRightIn1Pin, kRightIn2Pin, kRightMotorReversed};

// ---------------------------------------------------------------------------
// Velocity controller
// ---------------------------------------------------------------------------
struct WheelController
{
  float integral = 0.0f;
  float stall_ms = 0.0f;

  void reset()
  {
    integral = 0.0f;
    stall_ms = 0.0f;
  }

  // Returns PWM in [-255, 255].
  float update(float target, float measured, float dt)
  {
    if (fabsf(target) < kMinCommandRadS)
    {
      reset();
      return 0.0f;
    }
    const float error = target - measured;
    const float offset = kFeedforward * target + ((target > 0.0f) ? kPwmDeadband : -kPwmDeadband);
    const float unsaturated = offset + kKp * error + integral;
    // Conditional integration: once the output is pinned at full PWM, more integral in
    // the same direction can't help and would only have to unwind later (overshoot
    // when the load drops, e.g. coming off carpet).
    const bool saturated_same_way = (unsaturated >= kPwmMax && error > 0.0f) ||
                                    (unsaturated <= -kPwmMax && error < 0.0f);
    if (!saturated_same_way)
    {
      integral = constrain(integral + kKi * error * dt, -kIntegralLimit, kIntegralLimit);
    }
    return constrain(offset + kKp * error + integral, -kPwmMax, kPwmMax);
  }
};

WheelController g_left_ctrl;
WheelController g_right_ctrl;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
FrameParser g_parser;
float g_left_target = 0.0f;
float g_right_target = 0.0f;
bool g_enabled = false;
uint32_t g_last_command_ms = 0;
bool g_have_command = false;
bool g_encoder_fault = false;

uint32_t g_last_control_us = 0;
uint8_t g_state_seq = 0;

// Ring buffer of the last kVelocityWindow+1 samples, for windowed velocity.
int32_t g_hist_left[kVelocityWindow + 1];
int32_t g_hist_right[kVelocityWindow + 1];
uint32_t g_hist_us[kVelocityWindow + 1];
uint8_t g_hist_head = 0;

void handleFrame(uint8_t type, const uint8_t * payload)
{
  if (type == static_cast<uint8_t>(proto::MsgType::MotorCommand))
  {
    proto::MotorCommand cmd;
    memcpy(&cmd, payload, sizeof(cmd));
    // NaN/inf would poison the integrator permanently; treat them as stop.
    g_left_target = isfinite(cmd.left_rad_s) ? cmd.left_rad_s : 0.0f;
    g_right_target = isfinite(cmd.right_rad_s) ? cmd.right_rad_s : 0.0f;
    g_enabled = (cmd.flags & proto::kCmdEnable) != 0;
    g_last_command_ms = millis();
    g_have_command = true;
  }
  // SysCommand: ResetEncoders is deliberately ignored -- PROTOCOL.md says ticks are
  // free-running and never reset, and the Pi would read a reset as a huge jump.
}

void readSerial()
{
  // Bounded so a flood of bytes can't starve the control loop.
  for (int n = Serial.available(); n > 0; --n)
  {
    if (g_parser.feed(static_cast<uint8_t>(Serial.read())))
    {
      handleFrame(g_parser.type(), g_parser.payload());
    }
  }
}

void controlStep(uint32_t now_us)
{
  const int32_t left_ticks = g_left_ticks;
  const int32_t right_ticks = g_right_ticks;

  // Windowed velocity: compare against the oldest sample in the ring.
  const uint8_t oldest = (g_hist_head + 1) % (kVelocityWindow + 1);
  const float window_s = (now_us - g_hist_us[oldest]) * 1e-6f;
  float left_rad_s = 0.0f;
  float right_rad_s = 0.0f;
  if (window_s > 0.0f)
  {
    left_rad_s = static_cast<int32_t>(left_ticks - g_hist_left[oldest]) * kRadPerTick / window_s;
    right_rad_s = static_cast<int32_t>(right_ticks - g_hist_right[oldest]) * kRadPerTick / window_s;
  }
  // Store this sample at oldest's position: it becomes the newest.
  const int32_t prev_left = g_hist_left[g_hist_head];
  const int32_t prev_right = g_hist_right[g_hist_head];
  g_hist_head = oldest;
  g_hist_left[g_hist_head] = left_ticks;
  g_hist_right[g_hist_head] = right_ticks;
  g_hist_us[g_hist_head] = now_us;

  const bool watchdog_expired =
    !g_have_command || (millis() - g_last_command_ms) > kCommandTimeoutMs;
  const bool stopped_by_pi = !g_enabled ||
    (fabsf(g_left_target) < kMinCommandRadS && fabsf(g_right_target) < kMinCommandRadS);
  // Fault latches until the Pi asks for a stop, so it can't re-arm mid-runaway.
  if (stopped_by_pi) g_encoder_fault = false;

  const float dt = kControlPeriodUs * 1e-6f;
  float left_pwm = 0.0f;
  float right_pwm = 0.0f;
  if (watchdog_expired || !g_enabled || g_encoder_fault)
  {
    g_left_ctrl.reset();
    g_right_ctrl.reset();
  }
  else
  {
    left_pwm = g_left_ctrl.update(g_left_target, left_rad_s, dt);
    right_pwm = g_right_ctrl.update(g_right_target, right_rad_s, dt);

    // Encoder fault detection: hard drive, zero ticks, for kFaultTimeMs.
    const float period_ms = kControlPeriodUs * 1e-3f;
    g_left_ctrl.stall_ms = (fabsf(left_pwm) >= kFaultPwm && left_ticks == prev_left)
                           ? g_left_ctrl.stall_ms + period_ms : 0.0f;
    g_right_ctrl.stall_ms = (fabsf(right_pwm) >= kFaultPwm && right_ticks == prev_right)
                            ? g_right_ctrl.stall_ms + period_ms : 0.0f;
    if (g_left_ctrl.stall_ms >= kFaultTimeMs || g_right_ctrl.stall_ms >= kFaultTimeMs)
    {
      g_encoder_fault = true;
      left_pwm = 0.0f;
      right_pwm = 0.0f;
      g_left_ctrl.reset();
      g_right_ctrl.reset();
    }
  }

  kLeftMotor.drive(left_pwm);
  kRightMotor.drive(right_pwm);

  proto::MotorState state;
  state.left_ticks = left_ticks;
  state.right_ticks = right_ticks;
  state.left_rad_s = left_rad_s;
  state.right_rad_s = right_rad_s;
  state.esp_micros = now_us;
  state.seq = g_state_seq++;
  state.status = (watchdog_expired ? proto::kMotorCmdWatchdog : 0) |
                 (g_encoder_fault ? proto::kMotorEncoderFault : 0);
  sendFrame(proto::MsgType::MotorState, state);
}

void setup()
{
  // Motors off before anything else, so a reset never twitches the wheels.
  for (uint8_t pin : {kRightIn1Pin, kRightIn2Pin, kLeftIn3Pin, kLeftIn4Pin})
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  ledcAttach(kRightPwmPin, kPwmFreqHz, kPwmBits);
  ledcAttach(kLeftPwmPin, kPwmFreqHz, kPwmBits);
  ledcWrite(kRightPwmPin, 0);
  ledcWrite(kLeftPwmPin, 0);

  Serial.setRxBufferSize(512);
  Serial.begin(921600);

  // GPIO34-39 have no internal pull resistors; the encoders drive push-pull.
  for (uint8_t pin : {kRightEncA, kRightEncB, kLeftEncA, kLeftEncB})
  {
    pinMode(pin, INPUT);
  }
  g_left_ab = readAB(kLeftEncA, kLeftEncB);
  g_right_ab = readAB(kRightEncA, kRightEncB);
  attachInterrupt(digitalPinToInterrupt(kLeftEncA), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kLeftEncB), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncA), rightEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncB), rightEncoderIsr, CHANGE);

  const uint32_t now = micros();
  for (int i = 0; i <= kVelocityWindow; ++i)
  {
    g_hist_left[i] = 0;
    g_hist_right[i] = 0;
    g_hist_us[i] = now;
  }
  g_last_control_us = now;
}

void loop()
{
  readSerial();

  const uint32_t now = micros();
  if (now - g_last_control_us < kControlPeriodUs) return;
  g_last_control_us += kControlPeriodUs;
  // If something stalled us for more than a period, resync instead of bursting
  // several back-to-back control steps to catch up.
  if (now - g_last_control_us > kControlPeriodUs) g_last_control_us = now;

  controlStep(now);
}
