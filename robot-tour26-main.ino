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

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);   // ✅ must be before reading the pin

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
  Serial.println("System Initialized");
  Serial.println("About to waitForStartButton"); waitForStartButton(); Serial.println("Done wait");

}

void loop() {
  
  setMotors(120, 120);

  float leftEncDeg = 0, rightEncDeg = 0;
  float gyroZ = 0;

  readEncoders(leftEncDeg, rightEncDeg);
  readIMU(gyroZ);

  Serial.print("L_Enc: ");
  Serial.print(leftEncDeg, 1);
  Serial.print(" deg | R_Enc: ");
  Serial.print(rightEncDeg, 1);
  Serial.print(" deg | GyroZ(rad/s): ");
  Serial.println(gyroZ, 4);

  delay(100);
}