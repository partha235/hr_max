/*
  Optical SP02 Detection 
*/
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

uint32_t irBuffer[100];
uint32_t redBuffer[100];

int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

byte pulseLED = 2;      // ESP8266 safe pin (D4)
byte readLED = 16;      // ESP8266 onboard LED (D0)


// =========================
// 🟦 DEBUG WAIT FUNCTION
// =========================
bool waitForSample(const char *stage)
{
    unsigned long start = millis();

    while (!particleSensor.available()) {
        particleSensor.check();
        yield();

        if (millis() - start > 1200) {  // 1.2 sec timeout
            Serial.print("⚠ TIMEOUT in stage: ");
            Serial.println(stage);
            return false;
        }
    }

    return true;
}


void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println("\n=== ESP8266 MAX30102 DEBUG START ===");

    pinMode(pulseLED, OUTPUT);
    pinMode(readLED, OUTPUT);

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println("❌ MAX30102 not found!");
        while (1) yield();
    }

    Serial.println("Sensor Found. Press any key to continue...");
    while (Serial.available() == 0) yield();
    Serial.read();

    particleSensor.setup(60, 4, 2, 50, 411, 4096); 
    Serial.println("Sensor Initialized.");
}



void loop()
{
    bufferLength = 100;

    Serial.println("🟩 Stage 1: Reading first 100 samples...");

    for (byte i = 0 ; i < bufferLength ; i++)
    {
        if (!waitForSample("INITIAL READ")) return;

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i]  = particleSensor.getIR();
        particleSensor.nextSample();

        if (i % 10 == 0) {  // print every 10th sample
            Serial.print("i=");
            Serial.print(i);
            Serial.print("  red=");
            Serial.print(redBuffer[i]);
            Serial.print("  ir=");
            Serial.println(irBuffer[i]);
        }
    }

    Serial.println("🟩 Stage 2: Running SPO2 algorithm...");
    maxim_heart_rate_and_oxygen_saturation(
        irBuffer, bufferLength, redBuffer, &spo2, &validSPO2,
        &heartRate, &validHeartRate
    );


    Serial.println("🟩 Stage 3: Entering continuous loop...");

    // ============================
    //   MAIN SAFE AND DEBUG LOOP
    // ============================

    while (true)
    {
        // Shift old samples
        for (byte i = 25; i < 100; i++)
        {
            redBuffer[i - 25] = redBuffer[i];
            irBuffer[i - 25]  = irBuffer[i];
        }

        // Take 25 new samples
        for (byte i = 75; i < 100; i++)
        {
            if (!waitForSample("CONTINUOUS READ")) {
                Serial.println("⚠ BREAK TO AVOID WDT");
                return;
            }

            digitalWrite(readLED, !digitalRead(readLED));

            redBuffer[i] = particleSensor.getRed();
            irBuffer[i]  = particleSensor.getIR();
            particleSensor.nextSample();

            if (i % 5 == 0) {
                Serial.print("idx=");
                Serial.print(i);
                Serial.print("  HR=");
                Serial.print(heartRate);
                Serial.print("  SpO2=");
                Serial.println(spo2);
            }

            yield();
        }

        // Recalculate
        Serial.println("🔄 Recalculating SPO2 + HR...");
        maxim_heart_rate_and_oxygen_saturation(
            irBuffer, bufferLength, redBuffer, &spo2, &validSPO2,
            &heartRate, &validHeartRate
        );

        yield();
    }
}
