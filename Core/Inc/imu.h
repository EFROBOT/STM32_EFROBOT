#ifndef IMU_H
#define IMU_H

#include "main.h"

void imu_init(I2C_HandleTypeDef *hi2c, IWDG_HandleTypeDef *hiwdg);
int16_t imu_get_heading(I2C_HandleTypeDef *hi2c);

#endif
