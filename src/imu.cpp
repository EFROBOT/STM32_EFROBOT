#include "imu.h"
#include "motor.h"

float RateRoll, RatePitch, RateYaw;
float RateCalibrationYaw = 0;
float AngleYaw = 0;
unsigned long previousTime;

void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

void init_imu() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.println("Calibration Gyro...");
  for (int i = 0; i < 2000; i++) {
    gyro_signals();
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationYaw /= 2000;
  previousTime = micros();
}

void update_imu() {
  gyro_signals();
  RateYaw -= RateCalibrationYaw;
  
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  if (abs(RateYaw) > 0.7) {
    AngleYaw += RateYaw * dt;
  }
  angle_robot = AngleYaw * PI / 180.0;
}