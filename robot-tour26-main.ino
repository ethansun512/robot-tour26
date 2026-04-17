#include <Wire.h>

const int START_BUTTON_PIN = 2;

void initMotors();
void initEncoders();
void initIMU();
void scanI2C();
void setMotors(int leftSpeed, int rightSpeed);

void calibrateGyroZ(unsigned long ms = 2000);
void resetDistanceError();

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

  while (digitalRead(START_BUTTON_PIN) == LOW) { }
  delay(50);
  while (digitalRead(START_BUTTON_PIN) == HIGH) { }
  delay(200);

  Serial.println(F("Starting!"));
}

const char* program = "fn50 rn90 fn50 bn50 ln90 bn50";

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
  delay(2000);
  calibrateGyroZ(4000);

  Serial.println(F("Ready."));
}

void loop() {
  waitForStartButton();

  resetDistanceError();
  g_posX      = 0.0f;
  g_posY      = 0.0f;
  g_facingDeg = 0.0f;

  calibrateGyroZ(3000);

  startRunTimer(15.0f);
  runProgramTimed(program);

  Serial.print(F("Total time: ")); Serial.println(elapsedRunSec(), 3);
  Serial.print(F("Final X="));    Serial.print(g_posX, 1);
  Serial.print(F(" Y="));         Serial.print(g_posY, 1);
  Serial.print(F(" Facing="));    Serial.println(g_facingDeg, 1);
}