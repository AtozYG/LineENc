// cpp
#include <Arduino.h>
#include <QTRSensors.h>

// PID
float Kp = 0.052;
float Ki = 0.0;
float Kd = 0.837;
int P;
int I;
int D;
int lastError = 0;
// Timer for straight line detection
unsigned long straightStartTime = 0;
bool inStraightRange = false;
const unsigned long straightTimeThreshold = 300; // millis fra 200

const uint8_t maxSpeedA = 250;
const uint8_t maxSpeedB = 250;
const uint8_t baseSpeedA = 190; //fra 190
const uint8_t baseSpeedB = 190;

const int PWMA = GPIO_NUM_4;
const int AIN2 = GPIO_NUM_17;
const int AIN1 = GPIO_NUM_16;

const int BIN1 = GPIO_NUM_5;
const int BIN2 = GPIO_NUM_15;
const int PWMB = GPIO_NUM_32;


QTRSensors qtr;

const uint8_t SensorCount = 11;
uint16_t sensorValues[SensorCount];

// Map sensors to ESP32 ADC-capable GPIOs
const uint8_t sensorPins[SensorCount] = {GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14,
  GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_23, GPIO_NUM_22, GPIO_NUM_21,
  GPIO_NUM_19};

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// PWM channels for ESP32 ledc
const uint8_t PWMA_channel = 0;
const uint8_t PWMB_channel = 1;
const uint32_t PWM_freq = 5000;
const uint8_t PWM_resolution = 8; // 8-bit -> 0..255

// ── Encoder pins (Pololu 5157 – quadrature, 12 CPR) ─────────────────────────
// GPIO 34-39 are input-only on ESP32 and are well-suited for encoder signals.
const int ENC_A_RIGHT = 34; // Right motor encoder channel A
const int ENC_B_RIGHT = 35; // Right motor encoder channel B
const int ENC_A_LEFT  = 36; // Left motor encoder channel A
const int ENC_B_LEFT  = 39; // Left motor encoder channel B

volatile long encoderCountRight = 0;
volatile long encoderCountLeft  = 0;

// ISRs – called on rising edge of channel A; channel B gives direction
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENC_B_RIGHT) == HIGH) encoderCountRight++;
  else                                   encoderCountRight--;
}
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENC_B_LEFT) == HIGH) encoderCountLeft++;
  else                                  encoderCountLeft--;
}

// Atomic snapshot of both encoder counts (called from main loop)
long encoderAvg() {
  noInterrupts();
  long r = encoderCountRight;
  long l = encoderCountLeft;
  interrupts();
  return (r + l) / 2;
}

// ── State machine ────────────────────────────────────────────────────────────
enum RobotState { STATE_LEARNING, STATE_RACING };
RobotState robotState = STATE_LEARNING;

// Encoder count at the beginning of the current lap
long lapStartEncoder = 0;

// Distance travelled since the start of this lap (encoder ticks)
inline long lapPosition() { return encoderAvg() - lapStartEncoder; }

// ── Turn log ─────────────────────────────────────────────────────────────────
struct TurnEvent {
  long lapPos;      // Encoder ticks from lap start when turn began
  int  direction;   // > 0 means right turn, < 0 means left turn
};

const int MAX_TURNS = 60;
TurnEvent turnLog[MAX_TURNS];
int turnCount = 0;
int nextTurnIdx = 0; // index into turnLog for the next upcoming turn (racing mode)

// Turn detection state
bool  inTurn            = false;
long  turnStartPos      = 0;
const int TURN_THRESHOLD = 2000; // |error| above this = we are in a turn

// ── Lap-completion detection ──────────────────────────────────────────────────
// The start/finish is marked by a thick black stripe that places all sensors on
// the line at the same time (all sensorValues[] high after calibration).
const uint16_t LAP_SENSOR_THRESHOLD  = 800;  // calibrated value above which sensor is "on black"
const long     LAP_MIN_TICKS         = 2000; // minimum ticks before we check for lap end
bool lapDetectionArmed = false; // armed once we've travelled LAP_MIN_TICKS

// ── Pre-brake parameters ──────────────────────────────────────────────────────
const long    PRE_BRAKE_TICKS  = 250; // slow down this many ticks before a logged turn
const uint8_t APPROACH_SPEED   = 170; // speed during the pre-brake phase

void rightMotor(int motorSpeed);
void leftMotor(int motorSpeed);
void PID_control();
void checkLapCompletion();
void detectTurn(int error);
void printTurnLog();

void setup()
{
  // Motor A pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // Motor B pins
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // PWM setup (remove duplicate pinMode for PWMA/PWMB)
  ledcSetup(PWMA_channel, PWM_freq, PWM_resolution);
  ledcAttachPin(PWMA, PWMA_channel);
  ledcSetup(PWMB_channel, PWM_freq, PWM_resolution);
  ledcAttachPin(PWMB, PWMB_channel);

  // Encoder pins – GPIO 34-39 are input-only (no internal pull-up needed
  // because the Pololu 5157 encoder has push-pull outputs).
  pinMode(ENC_A_RIGHT, INPUT);
  pinMode(ENC_B_RIGHT, INPUT);
  pinMode(ENC_A_LEFT,  INPUT);
  pinMode(ENC_B_LEFT,  INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT),  leftEncoderISR,  RISING);

  // Ensure motors are stopped during calibration
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA_channel, 0);
  ledcWrite(PWMB_channel, 0);

  Serial.begin(115200);

  Serial.println("QTR calibration");

  // Use analog mode for ADC pins on ESP32
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(255); // use pin 17 to control the IR LEDs

  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //test
  Serial.println("Calibrating - robot will sway automatically");

  // Calibration parameters
    // Calibration parameters
  const int calibrationSpeed = 50;      // Speed for swaying
  const int swayDuration = 5;          // Time to sway in one direction (ms)
  const int calibrationCycles = 4;      // Number of left-right cycles

  for (int cycle = 0; cycle < calibrationCycles; cycle++) {
    // === SWAY LEFT from center ===
    digitalWrite(AIN1, HIGH);  // Right forward
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);   // Left backward
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMA_channel, calibrationSpeed);
    ledcWrite(PWMB_channel, calibrationSpeed);

    // Calibrate while swaying left
    for (int i = 0; i < 20; i++) {
      qtr.calibrate();
      delay(swayDuration / 20);
    }
    Serial.print('<');

    // === RETURN TO CENTER from left (also calibrating) ===
    digitalWrite(AIN1, LOW);   // Right backward
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);  // Left forward
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMA_channel, calibrationSpeed);
    ledcWrite(PWMB_channel, calibrationSpeed);

    // Calibrate while returning to center
    for (int i = 0; i < 20; i++) {
      qtr.calibrate();
      delay(swayDuration / 20);
    }
    Serial.print('|');

    // === SWAY RIGHT from center ===
    digitalWrite(AIN1, LOW);   // Right backward
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);  // Left forward
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMA_channel, calibrationSpeed);
    ledcWrite(PWMB_channel, calibrationSpeed);

    // Calibrate while swaying right
    for (int i = 0; i < 20; i++) {
      qtr.calibrate();
      delay(swayDuration / 20);
    }
    Serial.print('>');

    // === RETURN TO CENTER from right (also calibrating) ===
    digitalWrite(AIN1, HIGH);  // Right forward
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);   // Left backward
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMA_channel, calibrationSpeed);
    ledcWrite(PWMB_channel, calibrationSpeed);

    // Calibrate while returning to center
    for (int i = 0; i < 20; i++) {
      qtr.calibrate();
      delay(swayDuration / 20);
    }
    Serial.print('|');
  }

  // Stop motors
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA_channel, 0);
  ledcWrite(PWMB_channel, 0);

  // Center back over the line by reading position
  Serial.println("\nCentering...");
  delay(200);

  // Quick centering: read position and adjust
  for (int i = 0; i < 50; i++) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = 5000 - position;

    if (abs(error) < 300) {
      // Close enough to center, stop
      break;
    }

    int centerSpeed = 80;
    if (error > 0) {
      // Line is to the right, turn right
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    } else {
      // Line is to the left, turn left
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
    ledcWrite(PWMA_channel, centerSpeed);
    ledcWrite(PWMB_channel, centerSpeed);
    delay(20);
  }

  // Stop motors - ready to start
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA_channel, 0);
  ledcWrite(PWMB_channel, 0);

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration complete! Starting in soon");
  delay(500);

  // Reset encoder counts and lap reference so the first lap starts at 0
  encoderCountRight = 0;
  encoderCountLeft  = 0;
  lapStartEncoder   = 0;
  lapDetectionArmed = false;
  Serial.println("LEARNING LAP – logging turn positions");
/* // Tester det over fra der det står test
  Serial.println("Calibrating");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    if ((i & 31) == 0) Serial.print('.'); // sparse progress
  }
  digitalWrite(LED_BUILTIN, LOW);

*/
  // print calibration min/max
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(100);
}

void loop()
{
  checkLapCompletion();
  PID_control();
}

void PID_control()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 5000 - position;
  //Serial.println(error);
  P = error;
  I = I + error;
  if (I > 5000) I = 5000;
  if (I < -5000) I = -5000;
  D = error - lastError;
  lastError = error;

  float motorSpeedF = P * Kp + I * Ki + D * Kd;
  int motorspeed = (int)motorSpeedF;

  int motorSpeedA = baseSpeedA + motorspeed;
  int motorspeedB = baseSpeedB - motorspeed;

  if (motorSpeedA > maxSpeedA) motorSpeedA = maxSpeedA;
  if (motorspeedB > maxSpeedB) motorspeedB = maxSpeedB;
  if (motorSpeedA < 0) motorSpeedA = -100;
  if (motorspeedB < 0) motorspeedB = -100;

  // ── Learning lap: log turns based on PID error ───────────────────────────
  if (robotState == STATE_LEARNING) {
    detectTurn(error);
  }

  // ── Racing laps: pre-brake before logged turns ───────────────────────────
  bool approachingTurn = false;
  if (robotState == STATE_RACING) {
    long pos = lapPosition();
    // Advance past turns we have already completed
    while (nextTurnIdx < turnCount && pos > turnLog[nextTurnIdx].lapPos) {
      nextTurnIdx++;
    }
    // Check if the very next turn is within the pre-brake window
    if (nextTurnIdx < turnCount) {
      long dist = turnLog[nextTurnIdx].lapPos - pos;
      if (dist > 0 && dist <= PRE_BRAKE_TICKS) {
        approachingTurn = true;
      }
    }
  }

  if (approachingTurn) {
    // Slow both motors to APPROACH_SPEED while still applying PID steering
    int scaledA = (motorSpeedA > 0) ? min((int)APPROACH_SPEED, motorSpeedA) : motorSpeedA;
    int scaledB = (motorspeedB > 0) ? min((int)APPROACH_SPEED, motorspeedB) : motorspeedB;
    rightMotor(scaledA);
    leftMotor(scaledB);
  } else if ((error >= -1000) && (error <= 1000)) {
    // Sjekker om bilen er innenfor rangen
    if (!inStraightRange) {
      // Starter tiden
      straightStartTime = millis();
      inStraightRange = true;
    }

    // Hvis tiden er mer en satt verdi - Aktiver maxspeed
    if (millis() - straightStartTime >= straightTimeThreshold) {
      rightMotor(maxSpeedA);
      leftMotor(maxSpeedB);
    } else {
      // Bruker normal PID fart frem til maxspeed er aktivert
      rightMotor(motorSpeedA);
      leftMotor(motorspeedB);
    }
  }
  else {
    // Out of range, reset timer and use normal PID
    inStraightRange = false;
    rightMotor(motorSpeedA);
    leftMotor(motorspeedB);
  }
}

void rightMotor(int motorSpeed)
{
  if (motorSpeed > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (motorSpeed < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  uint8_t pwm = (uint8_t)min(255, abs(motorSpeed));
  ledcWrite(PWMA_channel, pwm);
}

void leftMotor(int motorSpeed)
{
  if (motorSpeed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if (motorSpeed < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  uint8_t pwm = (uint8_t)min(255, abs(motorSpeed));
  ledcWrite(PWMB_channel, pwm);
}

// ── detectTurn ───────────────────────────────────────────────────────────────
// Called every PID cycle during the learning lap.
// Records the lap-encoder position and direction when a turn begins.
void detectTurn(int error)
{
  if (abs(error) > TURN_THRESHOLD) {
    if (!inTurn) {
      inTurn       = true;
      turnStartPos = lapPosition();
      if (turnCount < MAX_TURNS) {
        turnLog[turnCount].lapPos    = turnStartPos;
        turnLog[turnCount].direction = (error > 0) ? 1 : -1;
        Serial.print("Turn #");
        Serial.print(turnCount + 1);
        Serial.print(" detected at pos=");
        Serial.print(turnStartPos);
        Serial.println((error > 0) ? " [RIGHT]" : " [LEFT]");
        turnCount++;
      }
    }
  } else {
    inTurn = false;
  }
}

// ── checkLapCompletion ───────────────────────────────────────────────────────
// Detects the start/finish line (all sensors on the black stripe) after a
// minimum distance, then transitions from learning to racing or resets the
// lap reference for the next racing lap.
void checkLapCompletion()
{
  long pos = lapPosition();

  // Arm detection once the robot has travelled far enough from the start
  if (!lapDetectionArmed && pos > LAP_MIN_TICKS) {
    lapDetectionArmed = true;
  }

  if (!lapDetectionArmed) return;

  // Check if all sensors are on the black start/finish stripe
  bool allBlack = true;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < LAP_SENSOR_THRESHOLD) {
      allBlack = false;
      break;
    }
  }
  if (!allBlack) return;

  // ── Lap complete ────────────────────────────────────────────────────────
  if (robotState == STATE_LEARNING) {
    Serial.println("=== LEARNING LAP COMPLETE ===");
    printTurnLog();
    robotState = STATE_RACING;
    Serial.println("Switching to RACING mode");
  } else {
    Serial.print("Racing lap complete. Lap ticks=");
    Serial.println(pos);
  }

  // Reset lap reference
  lapStartEncoder   = encoderAvg();
  lapDetectionArmed = false;
  nextTurnIdx       = 0; // restart turn scan from the beginning each lap
}

// ── printTurnLog ─────────────────────────────────────────────────────────────
void printTurnLog()
{
  Serial.print("Logged ");
  Serial.print(turnCount);
  Serial.println(" turns:");
  for (int t = 0; t < turnCount; t++) {
    Serial.print("  Turn ");
    Serial.print(t + 1);
    Serial.print(": pos=");
    Serial.print(turnLog[t].lapPos);
    Serial.println(turnLog[t].direction > 0 ? " RIGHT" : " LEFT");
  }
}