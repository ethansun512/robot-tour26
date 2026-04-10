#include <Wire.h>

const int START_BUTTON_PIN = 2;

// ---- Prototypes (implemented in other tabs) ----
void initMotors();
void initEncoders();
void initIMU();
void scanI2C();
void setMotors(int leftSpeed, int rightSpeed);

void calibrateGyroZ(unsigned long ms = 2000);
void resetDistanceError();  // ✅ Add this line


void startRunTimer(float targetSeconds);
float elapsedRunSec();
void runProgramTimed(const char* cmd);

void waitForStartButton() {
  Serial.println(F("================================="));
  Serial.println(F("Press START button to begin"));
  Serial.println(F("================================="));
  
  // Wait for button to be released (HIGH = not pressed with INPUT_PULLUP)
  while (digitalRead(START_BUTTON_PIN) == LOW) { }
  delay(50);
  
  // Wait for button to be pressed (LOW = pressed with INPUT_PULLUP)
  while (digitalRead(START_BUTTON_PIN) == HIGH) { }
  delay(200);
  
  Serial.println(F("Starting!"));
}
//first start - 38
//last back - 13
//const char* program = "fn38 ln90 fn100 rn90 fn100 rn90 fn50 ln90 fn100 rn180 fn150 ln90 fn100 ln90 fn50 b13";   // example
const char* program = "fn50 rn90 ln90 b50";
//const char* program = "f38 fn50 ln90 fn50 ln90 fn50 rn90 fn50 rn90 fn100 rn90 fn50 ln90 fn100 rn90 fn50 rn90 fn100 ln90 fn50 b13"

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();
  Serial.println("Booting...");
  scanI2C();

  initMotors();
  initEncoders();
  initIMU();

  calibrateGyroZ(4000);

  Serial.println(F("Ready."));
}

void loop() {

  waitForStartButton();

  resetDistanceError();

  startRunTimer(15.0f);        // target run time (seconds)
  runProgramTimed(program);
  
  Serial.print(F("Total time: "));
  Serial.println(elapsedRunSec(), 3);


}