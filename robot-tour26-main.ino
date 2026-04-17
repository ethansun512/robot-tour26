#include <Wire.h>

const int START_BUTTON_PIN = 2;

void initMotors();
void initEncoders();
void initIMU();
void scanI2C();
void setMotors(int leftSpeed, int rightSpeed);

void calibrateGyroZ(unsigned long ms);
void refreshGyroBias(int samples);
void resetDistanceError();
void setPose(float x, float y, float facingDeg);

void startRunTimer(float targetSeconds);
float elapsedRunSec();
void runProgramTimed(const char* cmd);

extern float g_posX;
extern float g_posY;
extern float g_facingDeg;

void waitForStartButton() {
  Serial.println(F("================================="));
  Serial.println(F("Press START button to begin"));
  Serial.println(F("================================="));

  // Make sure button is released first (wait for stable HIGH)
  unsigned long releasedSince = millis();
  while (millis() - releasedSince < 100) {
    if (digitalRead(START_BUTTON_PIN) == LOW) {
      releasedSince = millis();  // reset if pressed
    }
  }

  // Now wait for stable press (LOW for at least 50ms)
  unsigned long pressedSince = 0;
  while (true) {
    if (digitalRead(START_BUTTON_PIN) == LOW) {
      if (pressedSince == 0) pressedSince = millis();
      if (millis() - pressedSince > 50) break;
    } else {
      pressedSince = 0;
    }
  }

  // Wait for release before returning, so next call sees clean state
  while (digitalRead(START_BUTTON_PIN) == LOW) { }
  delay(50);

  Serial.println(F("Starting!"));
}

const char* program = "fn50 fn100 rn90 fn50 bn50 ln90 bn50";

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();
  Serial.println(F("Booting..."));
  scanI2C();

  initMotors();
  initEncoders();
  initIMU();
  delay(500);

  Serial.println(F("Ready."));
}

void loop() {
  waitForStartButton();

  resetDistanceError();
  setPose(0.0f, 0.0f, 0.0f);

  delay(2000);
  calibrateGyroZ(3000);

  startRunTimer(20.0f);
  runProgramTimed(program);

  Serial.print(F("Total time: ")); Serial.println(elapsedRunSec(), 3);
  Serial.print(F("Final X="));    Serial.print(g_posX, 1);
  Serial.print(F(" Y="));         Serial.print(g_posY, 1);
  Serial.print(F(" Facing="));    Serial.println(g_facingDeg, 1);
}