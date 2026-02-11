#include <Wire.h>

const int START_BUTTON_PIN = 2;

// Function declarations (implemented in other tabs)
void initMotors();
void setMotors(int leftSpeed, int rightSpeed);

void initEncoders();
void readEncoders(float &leftDeg, float &rightDeg);

void initIMU();
void readIMU(float &gyroZ);

void scanI2C();  // debug tool

void waitForStartButton() {
  Serial.println("Waiting for start button (D2 -> GND)...");
  while (digitalRead(START_BUTTON_PIN) == HIGH) {
    // wait (HIGH means not pressed because INPUT_PULLUP)
  }
  Serial.println("Start button pressed!");
  delay(200); // simple debounce so it doesn't double-trigger instantly
}

void setup() {
  Serial.begin(9600);
  delay(500); // give Serial time

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);   // must be before reading the pin

  Serial.println("Booting...");

  Wire.begin();
  Serial.println("I2C started.");

  // Debug: confirm the button is wired correctly
  Serial.print("Button state right now (HIGH=not pressed, LOW=pressed): ");
  Serial.println(digitalRead(START_BUTTON_PIN));

  // Now do other init
  scanI2C();

  Serial.println("About to initMotors"); initMotors(); Serial.println("Done initMotors");
  Serial.println("About to initEncoders"); initEncoders(); Serial.println("Done initEncoders");
  Serial.println("About to initIMU"); initIMU(); Serial.println("Done initIMU");
  Serial.println("About to calibrate"); calibrateGyroZ(); Serial.println("Done calibration");
  Serial.println("System Initialized");

}

String program = "f50 l90 f30 r90 b20";

void loop() {

  waitForStartButton();

  startRunTimer(30.0f);        // target total run time (seconds)
  runProgramTimed(program);

  Serial.print("Total time: ");
  Serial.println(elapsedRunSec(), 3);

  while(true); // stop after one run
}
}