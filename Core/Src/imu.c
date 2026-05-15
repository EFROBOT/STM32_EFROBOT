#include "imu.h"

#define BNO055_ADDR (0x28 << 1)

void imu_init(I2C_HandleTypeDef *hi2c, IWDG_HandleTypeDef *hiwdg) {
    HAL_IWDG_Refresh(hiwdg);
    HAL_Delay(500);
    HAL_IWDG_Refresh(hiwdg);
    HAL_Delay(200);

    uint8_t mode = 0x00;
    HAL_I2C_Mem_Write(hi2c, BNO055_ADDR, 0x3D, 1, &mode, 1, 100);
    HAL_Delay(50);

    mode = 0x0C;
    HAL_I2C_Mem_Write(hi2c, BNO055_ADDR, 0x3D, 1, &mode, 1, 100);
    HAL_Delay(50);
}

int16_t imu_get_heading(I2C_HandleTypeDef *hi2c) {
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(hi2c, BNO055_ADDR, 0x1A, 1, buf, 2, 100);

    if (s != HAL_OK) {
        HAL_I2C_DeInit(hi2c);
        HAL_Delay(100);
        HAL_I2C_Init(hi2c);
        HAL_Delay(100);
        return -1;
    }

    return (int16_t)((buf[1] << 8) | buf[0]) / 16;
}

