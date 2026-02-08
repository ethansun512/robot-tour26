#include <Wire.h>

static const uint8_t MPU_ADDR = 0x68;   // you confirmed this in the scan
static bool imuOK = false;

// MPU registers (MPU6050/MPU6500 compatible for basics)
static const uint8_t REG_WHO_AM_I     = 0x75;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_GYRO_ZOUT_H  = 0x47;

static uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

static void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void initIMU() {
  // Read identity
  uint8_t who = i2cRead8(MPU_ADDR, REG_WHO_AM_I);
  Serial.print("MPU WHO_AM_I = 0x");
  Serial.println(who, HEX);

  // Wake up (chip starts in sleep)
  i2cWrite8(MPU_ADDR, REG_PWR_MGMT_1, 0x00);
  delay(50);

  // Gyro full scale ±250 dps
  i2cWrite8(MPU_ADDR, REG_GYRO_CONFIG, 0x00);
  delay(10);

  // Basic sanity check (if reads aren't bogus)
  if (who != 0xFF && who != 0x00) {
    imuOK = true;
    Serial.println("IMU init OK");
  } else {
    imuOK = false;
    Serial.println("ERROR: IMU WHO_AM_I read failed");
  }
}

// Returns gyroZ in rad/s (so your print label is true)
void readIMU(float &gyroZ) {
  if (!imuOK) {
    gyroZ = 0.0f;
    return;
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);

  int16_t gzRaw = 0;
  if (Wire.available() == 2) {
    gzRaw = (int16_t)((Wire.read() << 8) | Wire.read());
  }

  // For ±250 dps, sensitivity is 131 LSB/(deg/s)
  float gz_dps = (float)gzRaw / 131.0f;
  gyroZ = gz_dps * 0.0174532925f; // deg/s -> rad/s
}