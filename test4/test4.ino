/*
  MAX30105 – Heart-Rate + SpO₂ on ESP8266 (NodeMCU)
  SparkFun library + Ratio-of-Ratios algorithm
  -------------------------------------------------
  IMPORTANT: Rename BUFFER_LENGTH → SPO2_BUF_LEN
*/

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 sensor;

// ---------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------
const uint8_t BPM_AVG_SIZE = 8;          // moving-average length for BPM
const uint8_t SPO2_BUF_LEN = 50;         // samples for AC/DC (≈1 s @ 50 Hz)
const uint8_t SAMPLE_RATE  = 50;         // Hz (default of the library)

// ---------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------
uint8_t  bpmBuffer[BPM_AVG_SIZE];
uint8_t  bpmIdx = 0;
long     lastBeat = 0;

float    bpm = 0.0;
int      avgBPM = 0;
int      spo2 = 0;
bool     spo2Valid = false;

// AC/DC circular buffers
int32_t  irBuf[SPO2_BUF_LEN];
int32_t  redBuf[SPO2_BUF_LEN];
uint8_t  bufPos = 0;
bool     bufFull = false;

// ---------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 not found!"));
    while (1);
  }

  uint8_t ledBright = 60;
  uint8_t sampleAvg = 4;
  uint8_t ledMode   = 2;
  int     sampleRt  = 100;   // 100 Hz = better SNR
  int     pulseW    = 411;
  int     adcRange  = 4096;

  sensor.setup(ledBright, sampleAvg, ledMode, sampleRt, pulseW, adcRange);

  // CRITICAL: Lower IR to avoid saturation
  sensor.setPulseAmplitudeRed(50);   // ~15 mA
  sensor.setPulseAmplitudeIR(30);    // ~9 mA  ← KEY FIX

  Serial.println(F("\nMAX30105 ready – Place finger firmly!"));
}

// ---------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------
void loop() {
  uint32_t ir  = sensor.getIR();
  uint32_t red = sensor.getRed();

  // ---- Finger detection -------------------------------------------------
  if (ir < 50000) {
    Serial.println(F("No finger"));
    resetAll();
    return;
  }

  // ---- Fill circular buffer for SpO₂ ------------------------------------
  irBuf[bufPos]  = ir;
  redBuf[bufPos] = red;
  bufPos = (bufPos + 1) % SPO2_BUF_LEN;
  if (bufPos == 0) bufFull = true;

  // ---- Heart-rate (IR only) ---------------------------------------------
  if (checkForBeat(ir)) {
    long now = millis();
    if (lastBeat) {
      long delta = now - lastBeat;
      bpm = 60.0f * 1000.0f / delta;

      if (bpm > 30 && bpm < 220) {
        bpmBuffer[bpmIdx++] = (uint8_t)bpm;
        bpmIdx %= BPM_AVG_SIZE;

        uint32_t sum = 0;
        for (uint8_t i = 0; i < BPM_AVG_SIZE; ++i) sum += bpmBuffer[i];
        avgBPM = sum / BPM_AVG_SIZE;
      }
    }
    lastBeat = now;
  }

  // ---- SpO₂ (only when we have a full window) ---------------------------
  if (bufFull) {
    spo2Valid = calcSpO2(irBuf, redBuf, SPO2_BUF_LEN, &spo2);
  }

  // ---- Serial output ----------------------------------------------------
  Serial.print(F("IR="));   Serial.print(ir);
  Serial.print(F("\tRed="));Serial.print(red);
  Serial.print(F("\tBPM="));Serial.print(bpm, 1);
  Serial.print(F("\tAvg="));Serial.print(avgBPM);
  Serial.print(F("\tSpO2="));
  if (spo2Valid) {
    Serial.print(spo2); Serial.print(F("%"));
  } else {
    Serial.print(F("--"));
  }
  Serial.println();
}

// ---------------------------------------------------------------------
// SpO₂ – Ratio-of-Ratios (same as Maxim reference)
// ---------------------------------------------------------------------
bool calcSpO2(int32_t *ir, int32_t *red, int len, int *out) {
  int32_t irMin =  1000000, irMax = 0;
  int32_t redMin = 1000000, redMax = 0;
  float   irDC = 0, redDC = 0;

  for (int i = 0; i < len; ++i) {
    int32_t v = ir[i];
    irDC += v;
    if (v < irMin) irMin = v;
    if (v > irMax) irMax = v;

    v = red[i];
    redDC += v;
    if (v < redMin) redMin = v;
    if (v > redMax) redMax = v;
  }

  irDC  /= len;
  redDC /= len;

  float irAC  = irMax  - irMin;
  float redAC = redMax - redMin;

  // weak signal → discard
  if (irAC < 500 || redAC < 500 || irDC < 50000 || redDC < 50000) return false;

  float R = (redAC / redDC) / (irAC / irDC);
  *out = (int)(110.0 - 25.0 * R);
  if (*out > 100) *out = 100;
  if (*out <  70) *out =  70;

  return (*out >= 90 && *out <= 100);
}

// ---------------------------------------------------------------------
// Reset everything when the finger is removed
// ---------------------------------------------------------------------
void resetAll() {
  bufPos = 0;
  bufFull = false;
  lastBeat = 0;
  spo2Valid = false;
  spo2 = 0;
  bpm = 0;
  avgBPM = 0;
}