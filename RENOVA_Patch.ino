/*
  ============================================================
   RENOVA Patch - Prototype Firmware
   MEDATHON 2026 - Medical Track
  ------------------------------------------------------------
   Wearable monitor for dialysis patients.
   - Real sensors : MAX30102 (heart rate), DS18B20 (skin temp)
   - Simulated    : blood markers via potentiometers
                    (K+ on A0, pH on A1, Creatinine on A3)
   - Calculated   : HCO3 (Henderson-Hasselbalch)
                    eGFR (CKD-EPI 2021)
   - Output       : DNI score (Dialysis Need Index)
                    LEDs + buzzer + Bluetooth (HC-05) to app
  ============================================================
*/

#include <Wire.h>
#include "MAX30105.h"          // SparkFun MAX3010x library
#include "heartRate.h"         // Beat detection algorithm
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>

// ---------------- Pins ----------------
const int LED_GREEN  = 7;
const int LED_YELLOW = 8;
const int LED_RED    = 6;
const int BUZZER     = 5;
const int TEMP_PIN   = 2;      // DS18B20 data

const int POT_K   = A0;        // Potassium
const int POT_PH  = A1;        // pH
const int POT_BUN = A2;        // BUN (not connected)
const int POT_CR  = A3;        // Creatinine
// A4 = SDA, A5 = SCL -> MAX30102 (do not use)

// HC-05: Arduino pin 10 <- TXD, Arduino pin 11 -> RXD (through divider)
SoftwareSerial bt(10, 11);     // RX, TX

// ---------------- Which pots are connected ----------------
// K+ is always read from A0
const bool USE_POT_PH  = true;    // A1
const bool USE_POT_BUN = false;   // A2 -> fixed value below
const bool USE_POT_CR  = true;    // A3

// ---------------- Default patient ----------------
const int  AGE    = 60;
const bool FEMALE = false;     // male

float pH  = 7.40;              // used if pot is off
float BUN = 60.0;              // mg/dL (fixed)
float Cr  = 6.0;               // mg/dL
const float PCO2 = 40.0;       // mmHg (assumed normal)

// ---------------- Thresholds (normal -> danger) ----------------
const float K_NORMAL    = 5.5,  K_DANGER    = 6.5;
const float PH_NORMAL   = 7.35, PH_DANGER   = 7.10;
const float HCO3_NORMAL = 22.0, HCO3_DANGER = 12.0;
const float BUN_NORMAL  = 20.0, BUN_DANGER  = 100.0;
const float EGFR_NORMAL = 30.0, EGFR_DANGER = 15.0;

// ---------------- DNI weights ----------------
const int W_K = 3, W_PH = 2, W_HCO3 = 2, W_BUN = 1, W_EGFR = 1;
const float MAX_TOTAL = 2.0 * (W_K + W_PH + W_HCO3 + W_BUN + W_EGFR);  // = 18

// ---------------- Heart rate (MAX30102) ----------------
MAX30105 particleSensor;
bool maxFound = false;
const long NO_FINGER_IR = 50000;   // IR below this = no finger

const byte AVG_SIZE = 8;           // average of last 8 beats
byte rates[AVG_SIZE];
byte rateSpot = 0;
byte rateCount = 0;
long lastBeat = 0;
int  beatAvg = 0;
byte jumpCount = 0;                // counts big jumps in a row

// ---------------- Temperature (DS18B20) ----------------
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
float skinTemp = 0;
unsigned long lastTempRequest = 0;

// ---------------- Timing ----------------
unsigned long lastReport = 0;
const unsigned long REPORT_EVERY = 1000;   // ms
unsigned long lastBuzz = 0;
bool buzzState = false;
int currentLevel = 0;

// ============================================================
void setup() {
  Serial.begin(115200);
  bt.begin(9600);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Temperature sensor: non-blocking mode (so pulse reading is not delayed)
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  tempSensor.requestTemperatures();
  lastTempRequest = millis();

  // Heart rate sensor: STANDARD speed (FAST did not work)
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    maxFound = true;
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("MAX30102 found");
  } else {
    Serial.println("MAX30102 not found - HR will be 0");
  }

  Serial.println("RENOVA Patch ready");
}

// ============================================================
void loop() {
  readHeartRate();       // must run as fast as possible
  readTemperature();     // every 1 s, non-blocking

  if (millis() - lastReport >= REPORT_EVERY) {
    lastReport = millis();
    report();
  }

  updateBuzzer();
}

// ---------------- Heart rate with filter ----------------
void readHeartRate() {
  if (!maxFound) return;

  long ir = particleSensor.getIR();

  // No finger -> reset everything
  if (ir < NO_FINGER_IR) {
    rateCount = 0;
    rateSpot = 0;
    beatAvg = 0;
    jumpCount = 0;
    return;
  }

  if (checkForBeat(ir)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    float bpm = 60.0 / (delta / 1000.0);

    // Reject impossible values
    if (bpm < 40 || bpm > 180) return;

    // Reject sudden jumps > 30 BPM, unless repeated 3 times
    if (beatAvg > 0 && abs(bpm - beatAvg) > 30) {
      jumpCount++;
      if (jumpCount < 3) return;
    }
    jumpCount = 0;

    // Add to rolling average
    rates[rateSpot++] = (byte)bpm;
    rateSpot %= AVG_SIZE;
    if (rateCount < AVG_SIZE) rateCount++;

    int sum = 0;
    for (byte i = 0; i < rateCount; i++) sum += rates[i];
    beatAvg = sum / rateCount;
  }
}

// ---------------- Temperature (non-blocking) ----------------
void readTemperature() {
  if (millis() - lastTempRequest >= 1000) {
    float t = tempSensor.getTempCByIndex(0);
    skinTemp = (t == DEVICE_DISCONNECTED_C) ? 0 : t;   // -127 = not connected
    tempSensor.requestTemperatures();
    lastTempRequest = millis();
  }
}

// ---------------- Graded score: 0 (normal) -> 2 (danger) ----------------
// Works for both directions (higher-is-worse or lower-is-worse)
float rampScore(float value, float normal, float danger) {
  float s = 2.0 * (value - normal) / (danger - normal);
  if (s < 0) s = 0;
  if (s > 2) s = 2;
  return s;
}

// Integer score for the emergency rule: 0, 1 or 2
int stepScore(float value, float normal, float danger) {
  bool higherIsWorse = danger > normal;
  if (higherIsWorse) {
    if (value >= danger) return 2;
    if (value >= normal) return 1;
  } else {
    if (value <= danger) return 2;
    if (value <= normal) return 1;
  }
  return 0;
}

// ---------------- eGFR (CKD-EPI 2021) ----------------
float calcEGFR(float scr) {
  float kappa = FEMALE ? 0.7 : 0.9;
  float alpha = FEMALE ? -0.241 : -0.302;
  float ratio = scr / kappa;

  float egfr = 142.0
             * pow(min(ratio, 1.0), alpha)
             * pow(max(ratio, 1.0), -1.2)
             * pow(0.9938, AGE);
  if (FEMALE) egfr *= 1.012;
  return egfr;
}

// ---------------- Main calculation + output ----------------
void report() {
  // 1) Read potentiometers
  float K = 3.0 + (analogRead(POT_K) / 1023.0) * 6.0;                      // 3.0 - 9.0
  if (USE_POT_PH)  pH  = 6.9 + (analogRead(POT_PH)  / 1023.0) * 0.7;      // 6.9 - 7.6
  if (USE_POT_BUN) BUN = 5   + (analogRead(POT_BUN) / 1023.0) * 145;      // 5 - 150
  if (USE_POT_CR)  Cr  = 0.5 + (analogRead(POT_CR)  / 1023.0) * 9.5;      // 0.5 - 10

  // 2) Calculated markers
  float HCO3 = 0.03 * PCO2 * pow(10, pH - 6.1);   // Henderson-Hasselbalch
  float eGFR = calcEGFR(Cr);

  // 3) DNI (graded scores x weights)
  float total = rampScore(K,    K_NORMAL,    K_DANGER)    * W_K
              + rampScore(pH,   PH_NORMAL,   PH_DANGER)   * W_PH
              + rampScore(HCO3, HCO3_NORMAL, HCO3_DANGER) * W_HCO3
              + rampScore(BUN,  BUN_NORMAL,  BUN_DANGER)  * W_BUN
              + rampScore(eGFR, EGFR_NORMAL, EGFR_DANGER) * W_EGFR;
  int dni = round(total / MAX_TOTAL * 100);

  // 4) Level: 0 = green, 1 = yellow, 2 = red
  int level = 0;
  if (dni >= 60)      level = 2;
  else if (dni >= 30) level = 1;

  // Emergency rule: any critical marker raises the level
  int emergency = max(stepScore(K, K_NORMAL, K_DANGER),
                  max(stepScore(pH, PH_NORMAL, PH_DANGER),
                      stepScore(HCO3, HCO3_NORMAL, HCO3_DANGER)));
  if (emergency > level) level = emergency;
  currentLevel = level;

  // 5) LEDs
  digitalWrite(LED_GREEN,  level == 0);
  digitalWrite(LED_YELLOW, level == 1);
  digitalWrite(LED_RED,    level == 2);

  // 6) Serial Monitor (labeled, for testing)
  Serial.print("DNI: ");    Serial.print(dni);
  Serial.print(" | Level: ");
  Serial.print(level == 0 ? "GREEN" : level == 1 ? "YELLOW" : "RED");
  Serial.print(" | K: ");    Serial.print(K, 1);
  Serial.print(" | pH: ");   Serial.print(pH, 2);
  Serial.print(" | HCO3: "); Serial.print(HCO3, 1);
  Serial.print(" | BUN: ");  Serial.print(BUN, 0);
  Serial.print(" | Cr: ");   Serial.print(Cr, 1);
  Serial.print(" | eGFR: "); Serial.print(eGFR, 0);
  Serial.print(" | HR: ");
  if (beatAvg > 0) Serial.print(beatAvg); else Serial.print("--");
  Serial.print(" | Temp: "); Serial.println(skinTemp, 1);

  // 7) Bluetooth CSV line for the app:
  // DNI,level,K,pH,BUN,eGFR,HR,SpO2,Temp
  bt.print(dni);          bt.print(",");
  bt.print(level);        bt.print(",");
  bt.print(K, 1);         bt.print(",");
  bt.print(pH, 2);        bt.print(",");
  bt.print(BUN, 0);       bt.print(",");
  bt.print(eGFR, 0);      bt.print(",");
  bt.print(beatAvg);      bt.print(",");
  bt.print(0);            bt.print(",");    // SpO2 not measured
  bt.println(skinTemp, 1);
}

// ---------------- Buzzer: beeps only on RED ----------------
void updateBuzzer() {
  if (currentLevel == 2) {
    if (millis() - lastBuzz >= 400) {
      lastBuzz = millis();
      buzzState = !buzzState;
      digitalWrite(BUZZER, buzzState);
    }
  } else {
    buzzState = false;
    digitalWrite(BUZZER, LOW);
  }
}
