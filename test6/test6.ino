/*
  Fixed ESP8266-safe MAX30102 + SPO2 + HeartRate sketch
  - Aligns RED/IR samples correctly (no double-read)
  - Non-blocking wait with timeout and yield()
  - Uses sampleRate = 100 and bufferLength = 100
  - Avoids long blocking loops that cause WDT resets
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

MAX30105 particleSensor;

const int BUFFER_LEN = 100;
uint32_t irBuffer[BUFFER_LEN];
uint32_t redBuffer[BUFFER_LEN];

int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// Heart rate averaging
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

// Pins
byte pulseLED = 2;      // D4 on many ESP8266 boards
byte readLED = 16;      // D0 (onboard LED on some boards) — confirm for your board

// Sensor config
const byte LED_BRIGHTNESS = 60;  // 0..255
const byte SAMPLE_AVERAGE = 4;   // 1,2,4,8,16,32
const byte LED_MODE = 2;         // 1 = Red only, 2 = Red+IR
const byte SAMPLE_RATE = 100;    // 50,100,200,400,800,1000,1600,3200
const int PULSE_WIDTH = 411;     // 69,118,215,411
const int ADC_RANGE = 4096;      // 2048,4096,8192,16384

// Finger detection threshold (tune for your hardware)
const uint32_t IR_FINGER_THRESHOLD = 50000UL;

// Wait for a sample with timeout; returns true if sample is ready
bool waitForSample(const char *stage, unsigned long timeoutMs = 1200)
{
  unsigned long start = millis();
  while (!particleSensor.available()) {
    particleSensor.check(); // library call (may be quick)
    yield();
    if (millis() - start > timeoutMs) {
      Serial.print(F("⚠ TIMEOUT in stage: "));
      Serial.println(stage);
      return false;
    }
  }
  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== ESP8266 MAX30102 SAFE START ==="));

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("❌ MAX30102 not found! Check wiring/power."));
    while (1) { yield(); } // block but feed WDT
  }

  // set LED amplitudes (helps ensure good AC amplitude)
  particleSensor.setPulseAmplitudeRed(0x1F); // about mid-range; tune if needed
  particleSensor.setPulseAmplitudeIR(0x1F);

  // Use setup() helper - parameters: ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange
  particleSensor.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE, SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);

  Serial.println(F("Sensor initialized. Attach finger. Press any key to start."));
  while (Serial.available() == 0) { yield(); }
  Serial.read();

  // warm up: clear rates
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
}

void loop()
{
  bufferLength = BUFFER_LEN;

  // ------------ Stage 1: fill initial buffer -------------
  Serial.println(F("Stage 1: filling initial buffer..."));
  for (int i = 0; i < bufferLength; i++) {
    if (!waitForSample("INITIAL_READ")) {
      // On timeout, allow loop() to restart cleanly
      Serial.println(F("Initial read timeout, restarting loop."));
      delay(50);
      return;
    }

    // Read RED and IR once and store to buffers (aligned)
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    // occasionally print to avoid flooding serial
    if ((i % 20) == 0) {
      Serial.print(F("i="));
      Serial.print(i);
      Serial.print(F(" red="));
      Serial.print(redBuffer[i]);
      Serial.print(F(" ir="));
      Serial.println(irBuffer[i]);
    }

    yield();
  }

  // First calculation
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHeartRate
  );

  Serial.print(F("Initial calc -> HR: "));
  Serial.print(heartRate);
  Serial.print(F("  HRvalid: "));
  Serial.print(validHeartRate);
  Serial.print(F("  SpO2: "));
  Serial.print(spo2);
  Serial.print(F("  SpO2Valid: "));
  Serial.println(validSPO2);

  // ------------ Stage 2: continuous rolling updates -------------
  // We'll perform one window-shift + 25-sample read then recalc, then return to loop() so OS can run
  // This avoids very long blocking loops.
  // Shift last 75 samples up
  for (int i = 25; i < bufferLength; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  // Read 25 new samples (indexes 75..99)
  for (int i = 75; i < BUFFER_LEN; i++) {
    if (!waitForSample("CONTINUOUS_READ")) {
      Serial.println(F("Continuous read timeout, returning to loop."));
      delay(20);
      return; // exit loop() to let WiFi/OS breathe; on return we'll re-enter and continue
    }

    digitalWrite(readLED, !digitalRead(readLED)); // blink indicator each sample (non-blocking)

    // read aligned RED & IR (single read per channel)
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    uint32_t irValue = irBuffer[i]; // use buffer value (aligned)
    // Heart beat check uses IR buffer value
    if (checkForBeat((long)irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      float instantBpm = 60.0 / (delta / 1000.0);
      if (instantBpm > 20 && instantBpm < 255) {
        rates[rateSpot++] = (byte)instantBpm;
        rateSpot %= RATE_SIZE;
        // average rates
        long avg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) avg += rates[x];
        beatAvg = (int)(avg / RATE_SIZE);
        beatsPerMinute = instantBpm;
      }
    }

    // print occasional debug: every 5th new sample
    if ((i % 5) == 0) {
      Serial.print(F("idx="));
      Serial.print(i);
      Serial.print(F(" IR="));
      Serial.print(irValue);
      if (irValue < IR_FINGER_THRESHOLD) {
        Serial.print(F("  [No finger]"));
      } else {
        Serial.print(F("  BPM="));
        Serial.print(beatsPerMinute, 1);
        Serial.print(F("  Avg="));
        Serial.print(beatAvg);
      }
      Serial.println();
    }

    yield(); // feed watchdog
  }

  // Recalculate SPO2 + HR for the new window
  Serial.println(F("Recalculating SPO2 + HR..."));
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHeartRate
  );

  // Print result summary (one concise line)
  Serial.print(F("RESULT -> HR: "));
  Serial.print(heartRate);
  Serial.print(F(" (valid="));
  Serial.print(validHeartRate);
  Serial.print(F(")  BPMavg: "));
  Serial.print(beatAvg);
  Serial.print(F("  SpO2: "));
  Serial.print(spo2);
  Serial.print(F(" (valid="));
  Serial.print(validSPO2);
  Serial.print(F(")  IRmean: "));
  // compute mean IR quickly
  unsigned long s = 0;
  for (int i = 0; i < BUFFER_LEN; i++) s += irBuffer[i];
  Serial.println(s / BUFFER_LEN);

  // Small pause to keep serial readable and OS happy, then loop() will run again
  delay(50);
  return;
}
