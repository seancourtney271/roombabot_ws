/*
  Differential-drive robot controller — ESP32 DEVKITV1 port
  -----------------------------------------------------------
  Ported from an Arduino Uno version of this script. Three things
  changed to make it work on the ESP32; everything else (PID logic,
  serial command parsing, direction handling) is unchanged:

  1. PIN NUMBERS
     The Uno's pins 2/3/4/5/7/8/9/11/12/13 don't exist in the same
     form on the ESP32. Remapped to GPIOs that are safe to use on a
     DEVKITV1 (avoiding flash pins 6-11, boot-strapping pins 0/2/15,
     and UART pins 1/3).

  2. PWM OUTPUT
     The Uno's analogWrite() drives PWM directly on pins 9/11 via
     built-in hardware timers. The ESP32 instead uses its LEDC
     peripheral, configured explicitly with ledcSetup/ledcAttachPin/
     ledcWrite. (Newer arduino-esp32 core versions, 3.0+, do support
     calling analogWrite() directly — but the ledc calls used here
     work on any core version, so this is the safer choice.)

  3. IRAM_ATTR ON INTERRUPT HANDLERS
     ESP32 interrupt service routines (ISRs) should be marked
     IRAM_ATTR so they're kept in fast on-chip RAM rather than flash,
     which the ESP32 sometimes has to pause access to internally.
     Without this, encoder interrupts can occasionally be missed or
     the board can crash. The Uno has no equivalent requirement.

  CONFIRMED WIRING (from your encoder identification test):
    Yellow -> Encoder VCC (3.3V)
    Black  -> Encoder GND
    Green  -> Encoder channel A
    White  -> Encoder channel B
  Both encoders share the same color convention below. "Right" and
  "left" are just labels — wire whichever physical wheel you want to
  each set of pins, it doesn't matter electrically.

  FULL WIRING TABLE:
    ESP32 GPIO25 -> L298N ENA   (PWM speed, right motor)
    ESP32 GPIO26 -> L298N IN1   (direction, right motor)
    ESP32 GPIO27 -> L298N IN2   (direction, right motor)
    ESP32 GPIO14 -> L298N ENB   (PWM speed, left motor)
    ESP32 GPIO13 -> L298N IN3   (direction, left motor)
    ESP32 GPIO4  -> L298N IN4   (direction, left motor)
    ESP32 3.3V   -> Yellow wires, BOTH encoders (shared VCC)
    ESP32 GND    -> Black wires, BOTH encoders, AND L298N GND (common ground)
    ESP32 GPIO34 -> Green wire,  right encoder (channel A, interrupt)
    ESP32 GPIO35 -> White wire,  right encoder (channel B)
    ESP32 GPIO36 -> Green wire,  left encoder  (channel A, interrupt)
    ESP32 GPIO39 -> White wire,  left encoder  (channel B)

  L298N OUT1/OUT2 and OUT3/OUT4 connect ONLY to the motors' red/gray
  power wires -- the encoders never connect to the L298N at all, they
  wire directly to the ESP32.

  IMPORTANT — before uploading:
   - Install the "PID" library by Brett Beauregard (PID_v1) via the
     Arduino Library Manager if you haven't already — it's fully
     compatible with ESP32, no changes needed there.
   - Remove the ENA/ENB jumpers on the L298N board so these PWM pins
     actually control speed.
   - Update PULSES_PER_REV below to YOUR measured value (see the
     comment on that line) — the placeholder of 494 is carried over
     from the original script and is very unlikely to match your
     motors.
*/

#include <PID_v1.h>

// ---------- L298N H-Bridge connection pins ----------
#define L298N_enA 25  // PWM, right motor speed
#define L298N_enB 14  // PWM, left motor speed
#define L298N_in4 4   // Direction, left motor
#define L298N_in3 13  // Direction, left motor
#define L298N_in2 27  // Direction, right motor
#define L298N_in1 26  // Direction, right motor

// ---------- LEDC (ESP32 PWM) setup ----------
// Newer arduino-esp32 cores (3.0+) removed the old channel-based
// ledcSetup()/ledcAttachPin() functions in favor of a simpler
// pin-based API: ledcAttach(pin, freq, resolution) once in setup(),
// then ledcWrite(pin, duty) anywhere you'd have called analogWrite().
const int PWM_FREQ_HZ    = 5000;
const int PWM_RESOLUTION = 8;      // 8-bit -> duty cycle range 0-255

// ---------- Wheel encoder connection pins ----------
#define right_encoder_phaseA 34  // Green wire, interrupt-capable input-only pin
#define right_encoder_phaseB 35  // White wire
#define left_encoder_phaseA  36  // Green wire, interrupt-capable input-only pin
#define left_encoder_phaseB  39  // White wire

// ---------- Encoder scaling ----------
// How many encoder pulses correspond to one full wheel revolution, as
// counted by THIS file's decode scheme: one pulse per RISING edge on
// phase A only (x1 decoding), not per quadrature edge.
//
// Measured with firmware/encoder_calibration/encoder_calibration.ino,
// which does x4 decoding (counts every edge on both A and B), then
// divided by 4 to match this file's x1 counting:
//   left:  2248.4 ticks/rev (x4) / 4 = 562.1
//   right: 2241.6 ticks/rev (x4) / 4 = 560.4
// Averaged across both wheels since this file uses a single shared
// constant for both.
const float PULSES_PER_REV = 561.2;

// ---------- Encoders ----------
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";   // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;  // ms between velocity reports

// Safety watchdog: if no velocity command arrives within this window (ROS was
// Ctrl+C'd, crashed, or the serial link dropped), stop the motors instead of
// continuing to execute the last command indefinitely.
unsigned long last_command_millis = 0;
const unsigned long command_timeout = 500;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;     // rad/s
double left_wheel_cmd_vel = 0.0;      // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0;    // rad/s
double left_wheel_meas_vel = 0.0;     // rad/s
// Output - Command
double right_wheel_cmd = 0.0;             // 0-255
double left_wheel_cmd = 0.0;              // 0-255
// Tuning — these were tuned for the ORIGINAL motors on the Uno build.
// Expect to re-tune these for your motors; treat these as a starting
// point only.
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;
// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void IRAM_ATTR rightEncoderCallback();
void IRAM_ATTR leftEncoderCallback();

void setup() {
  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Set up PWM on the enable pins (new pin-based ledc API)
  ledcAttach(L298N_enA, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(L298N_enB, PWM_FREQ_HZ, PWM_RESOLUTION);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);

  // Init encoders
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Motor
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = true;
      }
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = false;
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
        last_command_millis = millis();
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Command Value
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    // Convert raw pulse counts (accumulated over `interval` ms) into
    // an angular velocity in rad/s:
    //   pulses/interval -> pulses/sec -> revs/sec -> revs/min -> rad/s
    float sample_hz = 1000.0 / interval;
    const float RPM_TO_RAD_S = 0.10472; // 2*pi/60
    right_wheel_meas_vel = (sample_hz * right_encoder_counter * (60.0 / PULSES_PER_REV)) * RPM_TO_RAD_S;
    left_wheel_meas_vel  = (sample_hz * left_encoder_counter  * (60.0 / PULSES_PER_REV)) * RPM_TO_RAD_S;

    // No command received recently -- force a stop rather than keep driving.
    if(current_millis - last_command_millis > command_timeout)
    {
      right_wheel_cmd_vel = 0.0;
      left_wheel_cmd_vel = 0.0;
    }

    rightMotor.Compute();
    leftMotor.Compute();

    // Ignore commands smaller than inertia
    if(right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    ledcWrite(L298N_enA, (int)right_wheel_cmd);
    ledcWrite(L298N_enB, (int)left_wheel_cmd);
  }
}

// New pulse from Right Wheel Encoder
// IRAM_ATTR keeps this function in fast on-chip RAM, required for
// reliable ESP32 interrupt handlers (see note at top of file).
void IRAM_ATTR rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_wheel_sign = "p";
  }
  else
  {
    right_wheel_sign = "n";
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void IRAM_ATTR leftEncoderCallback()
{
  if(digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_wheel_sign = "n";
  }
  else
  {
    left_wheel_sign = "p";
  }
  left_encoder_counter++;
}
