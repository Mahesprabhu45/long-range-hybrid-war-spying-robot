/*
  Project : Long Range Hybrid Control War Spying Robot
  File    : rover_receiver_main.ino
  Board   : Arduino Mega 2560

  Description:
  Main receiver-side firmware running on the rover.
  - Receives commands via LoRa (long range) and Bluetooth (short range)
  - Controls DC motors via motor driver
  - Supports manual and automatic obstacle avoidance modes
  - Dual ultrasonic sensing (left & right)
  - Pan–tilt camera control using PCA9685
  - Displays system status on I2C LCD
  - Reads temperature and humidity using DHT11
*/

// ========================== Libraries ==========================
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ===================== Motor & Control Pins ====================
const int A_RPWM = 7, A_LPWM = 8, A_REN = 26, A_LEN = 27;
const int B_RPWM = 5, B_LPWM = 6, B_REN = 23, B_LEN = 22;

int speedLevel = 6;
const int MAX_PWM = 255;
char currentCmd = 'S';
const bool invertRight = true;

// ======================== LoRa Settings ========================
const long LORA_FREQ = 433E6;
const int LORA_CS = 53;
const int LORA_RST = 49;
const int LORA_DIO0 = 2;

// ======================== Timers ===============================
unsigned long lastBTtime = 0;
unsigned long lastLoRAtime = 0;
unsigned long lastAutoPoll = 0;
unsigned long lastAutoToggleTime = 0;

// ====================== Drive Thresholds =======================
const int DRIVE_DEADZONE = 6;
const int DRIVE_ACTIVE_THRESH = 10;

// =================== PCA9685 / Pan–Tilt ========================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t PAN_CH = 0;
const uint8_t TILT_CH = 6;
const uint8_t SENSOR_L_SV_CH = 4;
const uint8_t SENSOR_R_SV_CH = 5;

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_FREQ = 50;

// ===================== Ultrasonic Setup ========================
bool autoMode = false;

const uint8_t UL_L_TRIG = 32;
const uint8_t UL_L_ECHO = 33;
const uint8_t UL_R_TRIG = 38;
const uint8_t UL_R_ECHO = 39;

const int SAFE_DIST_CM = 30;
const int AUTO_FORWARD_SPEED = 5;
const unsigned long AUTO_POLL_MS = 350;

const int SENSOR_CENTER_ANGLE = 90;
const int SENSOR_LEFT_ANGLE = 45;
const int SENSOR_RIGHT_ANGLE = 135;
const int SENSOR_SETTLE_MS = 200;

// ===================== LCD & DHT ===============================
LiquidCrystal_I2C *lcd = nullptr;
#define DHTPIN 41
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

String line0 = "INIT...";
String line1 = "DHT:---";

// ===================== Helper Functions ========================
void stopAll() {
  analogWrite(A_RPWM, 0); analogWrite(A_LPWM, 0);
  analogWrite(B_RPWM, 0); analogWrite(B_LPWM, 0);
}

int pwmForLevel(int lvl) {
  return map(constrain(lvl, 0, 9), 0, 9, 0, MAX_PWM);
}

void driveForward(int pwmv) {
  analogWrite(A_RPWM, 0); analogWrite(B_RPWM, 0);
  analogWrite(A_LPWM, pwmv); analogWrite(B_LPWM, pwmv);
}

void driveBackward(int pwmv) {
  analogWrite(A_LPWM, 0); analogWrite(B_LPWM, 0);
  analogWrite(A_RPWM, pwmv); analogWrite(B_RPWM, pwmv);
}

void pivotLeft(int pwmv) {
  analogWrite(A_RPWM, pwmv);
  analogWrite(B_LPWM, pwmv);
}

void pivotRight(int pwmv) {
  analogWrite(A_LPWM, pwmv);
  analogWrite(B_RPWM, pwmv);
}

long readUltrasonicCM(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 30000);
  if (d == 0) return 9999;
  return d / 58;
}

// ===================== Auto Mode Logic =========================
void runAutoModeOnce() {
  long leftDist = readUltrasonicCM(UL_L_TRIG, UL_L_ECHO);
  delay(40);
  long rightDist = readUltrasonicCM(UL_R_TRIG, UL_R_ECHO);

  if (leftDist > SAFE_DIST_CM && rightDist > SAFE_DIST_CM) {
    currentCmd = 'F';
    speedLevel = AUTO_FORWARD_SPEED;
  } else if (leftDist < rightDist) {
    currentCmd = 'R';
  } else {
    currentCmd = 'L';
  }
}

// =========================== Setup =============================
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  pinMode(A_RPWM, OUTPUT); pinMode(A_LPWM, OUTPUT);
  pinMode(B_RPWM, OUTPUT); pinMode(B_LPWM, OUTPUT);
  pinMode(A_REN, OUTPUT);  pinMode(A_LEN, OUTPUT);
  pinMode(B_REN, OUTPUT);  pinMode(B_LEN, OUTPUT);

  digitalWrite(A_REN, HIGH); digitalWrite(A_LEN, HIGH);
  digitalWrite(B_REN, HIGH); digitalWrite(B_LEN, HIGH);

  stopAll();

  pinMode(UL_L_TRIG, OUTPUT); pinMode(UL_L_ECHO, INPUT);
  pinMode(UL_R_TRIG, OUTPUT); pinMode(UL_R_ECHO, INPUT);

  // LoRa Init
  SPI.begin();
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  LoRa.begin(LORA_FREQ);

  // Servo Controller
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  dht.begin();
}

// ============================ Loop =============================
void loop() {

  // Read LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    char c = (char)LoRa.read();
    currentCmd = c;
  }

  // Auto Mode
  if (autoMode && millis() - lastAutoPoll > AUTO_POLL_MS) {
    lastAutoPoll = millis();
    runAutoModeOnce();
  }

  // Apply command
  int pwmv = pwmForLevel(speedLevel);
  switch (currentCmd) {
    case 'F': driveForward(pwmv); break;
    case 'B': driveBackward(pwmv); break;
    case 'L': pivotLeft(pwmv); break;
    case 'R': pivotRight(pwmv); break;
    default: stopAll(); break;
  }

  delay(10);
}
