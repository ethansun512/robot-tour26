#include <Wire.h>

#define MT6701_ADDR    0x06
#define MT6701_ANGLE_REG 0x03

static bool rightEncoderOK = true;

void initEncoders() {
  Wire.beginTransmission(MT6701_ADDR);
  byte err = Wire.endTransmission();
  rightEncoderOK = (err == 0);

  if (!rightEncoderOK) {
    Serial.println(F("WARNING: Right encoder (MT6701) not found on I2C!"));
  } else {
    Serial.println(F("Encoders initialized"));
  }
}

bool rightEncoderHealthy() {
  return rightEncoderOK;
}

void readEncoders(float &leftDeg, float &rightDeg) {
  int rawLeft = 0;
  for (int i = 0; i < 16; i++) rawLeft += analogRead(A0);
  rawLeft /= 16;
  leftDeg = (rawLeft / 1023.0f) * 360.0f;

  if (!rightEncoderOK) {
    rightDeg = 0.0f;
    return;
  }

  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(MT6701_ANGLE_REG);
  byte err = Wire.endTransmission(false);

  if (err != 0) {
    rightEncoderOK = false;
    rightDeg = 0.0f;
    Serial.println(F("ERROR: Right encoder I2C read failed!"));
    return;
  }

  Wire.requestFrom(MT6701_ADDR, 2);
  if (Wire.available() == 2) {
    uint16_t raw = (Wire.read() << 8) | Wire.read();
    raw &= 0x3FFF;
    rightDeg = raw * 360.0f / 16384.0f;
  } else {
    rightEncoderOK = false;
    rightDeg = 0.0f;
    Serial.println(F("ERROR: Right encoder returned no data!"));
  }
}