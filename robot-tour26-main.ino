#include <Wire.h>

// We declare these functions here so Arduino knows they exist across tabs.
void initMotors();
void setMotors(int leftSpeed, int rightSpeed);

void initEncoders();
void readEncoders(float &leftDeg, float &rightDeg);

void initIMU();
void readIMU(float &gyroZ);

void scanI2C();  // debug tool

void setup() {
  Serial.begin(9600);
  Wire.begin();

  delay(500);
  Serial.println("Booting...");

  // 1) Scan I2C so you can confirm devices are actually visible
  scanI2C();

  // 2) Init subsystems
  initMotors();
  initEncoders();
  initIMU();

  Serial.println("System Initialized");
}

void loop() {
  // Motor test: forward
  setMotors(120, 120);

  // Read sensors
  float leftEncDeg = 0, rightEncDeg = 0;
  float gyroZ = 0;

  readEncoders(leftEncDeg, rightEncDeg);
  readIMU(gyroZ);

  // Print
  Serial.print("L_Enc: ");
  Serial.print(leftEncDeg, 1);
  Serial.print(" deg | R_Enc: ");
  Serial.print(rightEncDeg, 1);
  Serial.print(" deg | GyroZ(rad/s): ");
  Serial.println(gyroZ, 4);
