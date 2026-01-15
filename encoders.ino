#include <MT6701.h>

const int ENC_LEFT_PIN = A0;

// Create the right encoder object
MT6701 rightEnc;

void initEncoders() {
  // If the library needs Wire, it's already started in setup()
  rightEnc.begin();
}

void readEncoders(float &leftDeg, float &rightDeg) {
  // Left: analog 0–1023 mapped to 0–360 degrees (basic test)
  int rawLeft = analogRead(ENC_LEFT_PIN);
  leftDeg = (rawLeft / 1023.0f) * 360.0f;

  // Right: I2C MT6701 in degrees
  rightDeg = rightEnc.getAngleDegrees();
}