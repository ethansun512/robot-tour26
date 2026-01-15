#include <Wire.h>
#include <MPU6050.h>

// ================== PIN DEFINITIONS ==================

// Left Motor (DRV8871)
const int LM_IN1 = 5;
const int LM_IN2 = 6;

// Right Motor (DRV8871)
const int RM_IN1 = 9;
const int RM_IN2 = 10;

// Left Encoder (Analog MT6701)
const int ENC_LEFT_PIN = A0;

// ================== OBJECTS ==================

MPU6050 imu;

// ================== SETUP ==================

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Motor pins
  pinMode(LM_IN1, OUTPUT);
  pinMode(LM_IN2, OUTPUT);
  pinMode(RM_IN1, OUTPUT);
  pinMode(RM_IN2, OUTPUT);

  // IMU
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("MPU6500 NOT CONNECTED");
    while (1);
  }

  Serial.println("System Initialized");
}

// ================== MOTOR CONTROL ==================

void setMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

// ================== LOOP ==================

void loop() {

  // ---- MOTOR TEST ----
  setMotor(LM_IN1, LM_IN2, 120);
  setMotor(RM_IN1, RM_IN2, 120);

  // ---- LEFT ENCODER (ANALOG) ----
  int rawLeft = analogRead(ENC_LEFT_PIN);
  float angleLeft = (rawLeft / 1023.0) * 360.0;

  // ---- RIGHT ENCODER (I2C MT6701) ----
  // MT6701 angle register read
  Wire.beginTransmission(0x06); // common MT6701 address
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(0x06, 2);

  int angleRightRaw = 0;
  if (Wire.available() == 2) {
    angleRightRaw = (Wire.read() << 8) | Wire.read();
    angleRightRaw &= 0x3FFF; // 14-bit
  }
  float angleRight = angleRightRaw * 360.0 / 16384.0;

  // ---- IMU ----
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // ---- PRINT ----
  Serial.print("L_Enc: ");
  Serial.print(angleLeft, 1);
  Serial.print(" deg | R_Enc: ");
  Serial.print(angleRight, 1);
  Serial.print(" deg | GyroZ: ");
  Serial.println(gz);

  delay(100);
}