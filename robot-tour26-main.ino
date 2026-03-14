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
  
  // First, wait for button to be RELEASED (if it was pressed)
  while (digitalRead(START_BUTTON_PIN) == HIGH) { }
  delay(50);
  
  // Now wait for button to be PRESSED
  while (digitalRead(START_BUTTON_PIN) == LOW) { }
  delay(200); // debounce
  
  Serial.println("Starting!");
}

const char* program = "fn50 rn90";   // example

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

  calibrateGyroZ(2000);

  Serial.println(F("Ready."));
}

void loop() {

  waitForStartButton();

  resetDistanceError();

  startRunTimer(5.0f);        // target run time (seconds)
  runProgramTimed(program);
  
  Serial.print(F("Total time: "));
  Serial.println(elapsedRunSec(), 3);


}