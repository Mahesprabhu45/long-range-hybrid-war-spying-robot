/*
  Project : Long Range Hybrid Control War Spying Robot
  File    : lora_transmitter.ino
  Board   : Arduino Nano / Arduino Uno (with LoRa SX1278)

  Description:
  - Reads joystick X/Y
  - Reads buttons for Manual / Auto toggle
  - Sends CSV packet over LoRa to rover
  - Compatible with rover_receiver_main.ino
*/

#include <SPI.h>
#include <LoRa.h>

// ======================= LoRa Pins =======================
#define LORA_CS    10
#define LORA_RST   9
#define LORA_DIO0  2
#define LORA_FREQ  433E6

// ===================== Joystick Pins =====================
#define JOY_X A0
#define JOY_Y A1

// ====================== Button Pins ======================
#define BTN_MANUAL 4
#define BTN_AUTO   5

// ===================== Variables =========================
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 100; // ms

// ======================= Setup ===========================
void setup() {
  Serial.begin(9600);

  pinMode(BTN_MANUAL, INPUT_PULLUP);
  pinMode(BTN_AUTO, INPUT_PULLUP);

  SPI.begin();
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  Serial.println("LoRa Transmitter Ready");
}

// ======================== Loop ===========================
void loop() {
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = millis();
    sendControlPacket();
  }
}

// ===================== Send Packet =======================
void sendControlPacket() {

  int joyX = analogRead(JOY_X);
  int joyY = analogRead(JOY_Y);

  // Map joystick to -100 to +100
  int mvx = map(joyX, 0, 1023, -100, 100);
  int mvy = map(joyY, 0, 1023, -100, 100);

  bool manualBtn = !digitalRead(BTN_MANUAL);
  bool autoBtn   = !digitalRead(BTN_AUTO);

  /*
    CSV Format (must match receiver):
    mvx,mvy,pan,tilt,armx,army,bManual,bAuto,bReturn,bArmSw
  */

  String packet = "";
  packet += String(mvx) + ",";
  packet += String(mvy) + ",";
  packet += "0,0,0,0,";                 // pan, tilt, armx, army (unused)
  packet += String(manualBtn) + ",";
  packet += String(autoBtn) + ",";
  packet += "0,0";                      // return, arm switch

  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

  Serial.print("Sent: ");
  Serial.println(packet);
}

