#include <Wire.h>

void scanI2C() {
  Serial.println("I2C scan starting...");

  int count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("Check: SDA=A4, SCL=A5, common GND, 3.3V power.");
  } else {
    Serial.print("Total I2C devices found: ");
    Serial.println(count);
  }

  Serial.println("I2C scan done.\n");
}