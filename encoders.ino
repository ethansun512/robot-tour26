#include <Wire.h>

#define MT6701_ADDR 0x06
#define MT6701_ANGLE_REG 0x03

void initEncoders() {
  // nothing required for analog encoder
  Serial.println("Encoders initialized");
}

void readEncoders(float &leftDeg, float &rightDeg) {

  //left analog encoder
  int rawLeft = 0;
  for(int i=0; i<4; i++) rawLeft += analogRead(A0);
  rawLeft /= 4;
  leftDeg = (rawLeft / 1023.0) * 360.0;

  //right i2c encoder
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(MT6701_ANGLE_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(MT6701_ADDR, 2);

  if (Wire.available() == 2) {
    uint16_t raw = (Wire.read() << 8) | Wire.read();
    raw &= 0x3FFF;
    rightDeg = raw * 360.0 / 16384.0;
  } else {
    rightDeg = 0.0;  // ✅ Add this
  }
}