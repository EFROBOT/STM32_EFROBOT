#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

extern float AngleYaw;

void init_imu();
void update_imu();

#endif