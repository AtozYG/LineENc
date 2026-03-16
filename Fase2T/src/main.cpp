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

void rightMotor(int motorSpeed);
void leftMotor(int motorSpeed);
void PID_control();

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
  /*
  Serial.print("HøyreM=");
  Serial.println(motorSpeedA);
  Serial.print("VenstreM=");
  Serial.println(motorspeedB);
  */
  if (motorSpeedA > maxSpeedA) motorSpeedA = maxSpeedA;
  if (motorspeedB > maxSpeedB) motorspeedB = maxSpeedB;
  if (motorSpeedA < 0) motorSpeedA = -100;
  if (motorspeedB < 0) motorspeedB = -100;

  //test
  /*
  if (error == -5000) {
    leftMotor(maxSpeedB);
    rightMotor(-100);
  }
  if (error == 5000) {
    leftMotor(-100);
    rightMotor(motorSpeedA);
  }
*/
  // Sjekker om bilen er innenfor rangen
  if ((error >= -1000) && (error <= 1000)) {
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