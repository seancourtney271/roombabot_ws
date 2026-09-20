/*
 * roombabot encoder ticks-per-revolution calibration
 *
 * Bench-only sketch for esp32_motor. It does NOT speak the wire protocol in
 * PROTOCOL.md -- it prints plain text over the same USB-CDC port so you can
 * drive it from the Arduino Serial Monitor before the real esp32_motor.ino
 * firmware exists. Flash the real firmware back afterwards; this sketch has
 * no motor control or command watchdog and should never drive the robot.
 *
 * Wiring (edit these four defines if your harness differs):
 *   left  A (green) -> GPIO36   left  B (white) -> GPIO39
 *   right A (green) -> GPIO34   right B (white) -> GPIO35
 *
 * GPIO 34-39 are input-only ESP32 pins with no internal pull resistors, so
 * this assumes the encoder outputs are actively driven (push-pull), which
 * holds for the stock Roomba 600 / Create 2 drive encoders.
 *
 * Procedure:
 *   1. Flash this sketch. Open Serial Monitor at 115200 baud, newline
 *      line ending.
 *   2. Stick a piece of tape on each wheel as a rotation reference mark.
 *   3. Type "r" + Enter to zero both counters.
 *   4. Rotate one wheel by hand through an exact number of full turns --
 *      10+ turns keeps stopping on a fractional tick from mattering much.
 *      Turning the wheel FORWARD must make its count increase; if it goes
 *      negative, that wheel's A/B pair is swapped from the PROTOCOL.md sign
 *      convention (increasing ticks == forward on both sides).
 *   5. Type "c <revolutions>" + Enter, e.g. "c 10", to print ticks/rev for
 *      both wheels from the counts accumulated since the last reset.
 *   6. Repeat a few times per wheel and average. Gearbox backlash and an
 *      imprecise hand-turn count dominate the error here, not the encoder.
 *
 * The result goes into encoder_ticks_per_revolution in
 * roombabot_controllers.yaml (see RUNNING.md).
 */

#define LEFT_ENC_A  36
#define LEFT_ENC_B  39
#define RIGHT_ENC_A 34
#define RIGHT_ENC_B 35

volatile int64_t left_ticks = 0;
volatile int64_t right_ticks = 0;
volatile uint8_t left_state = 0;
volatile uint8_t right_state = 0;

// x4 quadrature decode: table index is (previous 2-bit AB state << 2) | new
// 2-bit AB state; value is the tick delta for that transition (0 if the
// transition is illegal/bounced).
static const int8_t kQuadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR leftIsr() {
  uint8_t new_state = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  left_ticks += kQuadTable[(left_state << 2) | new_state];
  left_state = new_state;
}

void IRAM_ATTR rightIsr() {
  uint8_t new_state = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);
  right_ticks += kQuadTable[(right_state << 2) | new_state];
  right_state = new_state;
}

uint32_t last_print_ms = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  left_state = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  right_state = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), leftIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightIsr, CHANGE);

  Serial.println();
  Serial.println("roombabot encoder calibration");
  Serial.println("  r          reset both counters to zero");
  Serial.println("  c <revs>   compute ticks/rev from counts since last reset");
  Serial.println("  (counts also print automatically twice a second)");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      // ignore blank lines
    } else if (line == "r") {
      noInterrupts();
      left_ticks = 0;
      right_ticks = 0;
      interrupts();
      Serial.println("counters reset");
    } else if (line.startsWith("c")) {
      float revs = line.substring(1).toFloat();
      if (revs <= 0) {
        Serial.println("usage: c <revolutions>, e.g. \"c 10\"");
      } else {
        int64_t lt, rt;
        noInterrupts();
        lt = left_ticks;
        rt = right_ticks;
        interrupts();
        Serial.print("left:  ");
        Serial.print((long)lt);
        Serial.print(" ticks / ");
        Serial.print(revs);
        Serial.print(" rev = ");
        Serial.print(fabs((double)lt) / revs, 2);
        Serial.println(" ticks/rev");
        Serial.print("right: ");
        Serial.print((long)rt);
        Serial.print(" ticks / ");
        Serial.print(revs);
        Serial.print(" rev = ");
        Serial.print(fabs((double)rt) / revs, 2);
        Serial.println(" ticks/rev");
      }
    } else {
      Serial.println("unknown command, use 'r' or 'c <revs>'");
    }
  }

  uint32_t now = millis();
  if (now - last_print_ms >= 500) {
    last_print_ms = now;
    int64_t lt, rt;
    noInterrupts();
    lt = left_ticks;
    rt = right_ticks;
    interrupts();
    Serial.print("left=");
    Serial.print((long)lt);
    Serial.print("  right=");
    Serial.println((long)rt);
  }
}
