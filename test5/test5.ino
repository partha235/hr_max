/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
*/
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
const uint8_t SPO2_BUF_LEN = 50; // samples for AC/DC (≈1 s @ 50 Hz)
float beatsPerMinute;
int beatAvg;
int spo2 = 0;
bool spo2Valid = false;
// AC/DC circular buffers
int32_t irBuf[SPO2_BUF_LEN];
int32_t redBuf[SPO2_BUF_LEN];
uint8_t bufPos = 0;
bool bufFull = false;
void setup()
{
Serial.begin(115200);
Serial.println("Initializing...");
  // Initialize sensor
if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
Serial.println("MAX30105 was not found. Please check wiring/power. ");
while (1);
  }
Serial.println("Place your index finger on the sensor with steady pressure.");
particleSensor.setup(); //Configure sensor with default settings
particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}
void loop()
{
uint32_t ir = particleSensor.getIR();
uint32_t red = particleSensor.getRed();
long irValue = particleSensor.getIR();
if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      //Take average of readings
      beatAvg = 0;
for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  // ---- Fill circular buffer for SpO₂
irBuf[bufPos] = ir;
redBuf[bufPos] = red;
  bufPos = (bufPos + 1) % SPO2_BUF_LEN;
if (bufPos == 0) bufFull = true;
  // ---- SpO₂ (only when we have a full window) ---------------------------
if (bufFull) {
    spo2Valid = calcSpO2(irBuf, redBuf, SPO2_BUF_LEN, &spo2);
  }
Serial.print("IR=");
Serial.print(irValue);
Serial.print(", BPM=");
Serial.print(beatsPerMinute);
Serial.print(", Avg BPM=");
Serial.print(beatAvg);
if (ir < 50000) {
Serial.println(F("No finger"));
resetAll();
return;
  }
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
int32_t irMin = 1000000, irMax = 0;
int32_t redMin = 1000000, redMax = 0;
float irDC = 0, redDC = 0;
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
  irDC /= len;
  redDC /= len;
float irAC = irMax - irMin;
float redAC = redMax - redMin;
  // weak signal → discard
if (irAC < 500 || redAC < 500 || irDC < 50000 || redDC < 50000) return false;
float R = (redAC / redDC) / (irAC / irDC);
  *out = (int)(110.0 - 25.0 * R);
if (*out > 100) *out = 100;
if (*out < 70) *out = 70;
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
  beatsPerMinute=0.0;
  beatAvg=0;
}