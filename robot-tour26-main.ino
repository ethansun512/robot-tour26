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
void runProgramTimed(String cmd);

void waitForStartButton() {
  Serial.println("Waiting for start button (D2 -> GND)...");
  while (digitalRead(START_BUTTON_PIN) == HIGH) { }
  delay(200); // debounce
  Serial.println("Start!");
}

String program = "f8 f50";   // example

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

  Serial.println("Ready.");
}

void loop() {

  waitForStartButton();

  resetDistanceError();

  startRunTimer(30.0f);        // target run time (seconds)
  runProgramTimed(program);
  
  Serial.print("Total time: ");
  Serial.println(elapsedRunSec(), 3);

}