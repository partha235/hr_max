#include <SoftWire.h>

// Your pins on ESP32-CAM
#define SDA_PIN  16
#define SCL_PIN  13

SoftWire sw(SDA_PIN, SCL_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);           // Wait for serial monitor
  delay(500);
  
  Serial.println("\n=== SoftI2C Scanner (SoftWire) ===");
  Serial.println("Using SDA = GPIO" + String(SDA_PIN) + "   SCL = GPIO" + String(SCL_PIN));
  
  // Important: enable internal pull-ups (ESP32-CAM often has no external ones)
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  
  sw.setTimeout(1000);
  sw.begin();
  
  delay(100);

  byte error, address;
  int devicesFound = 0;

  Serial.println("Scanning I2C bus (7-bit addresses 0x08 to 0x77)...\n");

  for (address = 8; address < 120; address++) {
    sw.beginTransmission(address);
    error = sw.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Common MAX30102 addresses
      if (address == 0x57) Serial.print("  <-- MAX30102 / MAX30105 (most common)");
      if (address == 0xAE) Serial.print("  <-- Some MAX30102 boards (write address)");
      
      Serial.println();
      devicesFound++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (devicesFound == 0) {
    Serial.println("No I2C devices found!\n");
    Serial.println("Possible causes:");
    Serial.println("  • Wiring error (SDA/SCL swapped?)");
    Serial.println("  • Missing or too-weak pull-up resistors (try adding 4.7kΩ)");
    Serial.println("  • Sensor not powered (check 3.3V and GND)");
    Serial.println("  • Sensor damaged");
  } else {
    Serial.printf("\nScan complete: %d device(s) found.\n", devicesFound);
  }
}

void loop() {
  // Nothing here – scanner runs once
}