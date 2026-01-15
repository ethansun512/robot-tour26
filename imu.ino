#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void initIMU() {
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU-6500 not found on I2C!");
    Serial.println("Check wiring: VCC->3.3V, GND->GND, SDA->A4, SCL->A5");
    while (1) {}
  }

  // Optional: set ranges for stability
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void readIMU(float &gyroZ) {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // gyro.gyro is radians/sec
  gyroZ = gyro.gyro.z;
}