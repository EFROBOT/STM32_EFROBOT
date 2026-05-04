#include "pince.h"


// Piston

void piston(Piston state)
{
    if (state == Ouvert)
    {
        HAL_GPIO_WritePin(pist_0_GPIO_Port, pist_0_Pin, GPIO_PIN_SET); // In1
        HAL_GPIO_WritePin(pist_1_GPIO_Port, pist_1_Pin, GPIO_PIN_RESET); // In2
    }
    else
    {
        HAL_GPIO_WritePin(pist_0_GPIO_Port, pist_0_Pin, GPIO_PIN_RESET); // In1
        HAL_GPIO_WritePin(pist_1_GPIO_Port, pist_1_Pin, GPIO_PIN_RESET); // In2
    }
}


//-------------------------------------------------------------
// Stepper

void activer_stepper(Stepper x)
{
    if (x == Stepper_lever_pince)
        HAL_GPIO_WritePin(en1_GPIO_Port, en1_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(en2_GPIO_Port, en2_Pin, GPIO_PIN_RESET);
}

void desactiver_stepper(Stepper x)
{
    if (x == Stepper_lever_pince)
        HAL_GPIO_WritePin(en1_GPIO_Port, en1_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(en2_GPIO_Port, en2_Pin, GPIO_PIN_SET);
}

void nombre_pas_stepper(Stepper x, Stepper_dir dir, uint32_t steps, uint32_t delay_ms)
{
    if (x == Stepper_lever_pince) {
        if (dir == Horraire)
            HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_SET);
    }
    else {
        if (dir == Horraire)
            HAL_GPIO_WritePin(dir2_GPIO_Port, dir2_Pin, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(dir2_GPIO_Port, dir2_Pin, GPIO_PIN_SET);
    }

    for (uint32_t i = 0; i < steps; i++)
    {
        if (x == Stepper_lever_pince) {
            HAL_GPIO_WritePin(step1_GPIO_Port, step1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(step1_GPIO_Port, step1_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(step2_GPIO_Port, step2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(step2_GPIO_Port, step2_Pin, GPIO_PIN_RESET);
        }

        HAL_Delay(delay_ms);
    }
}

//-------------------------------------------------------------
// Servo

void controle_angle_servo(TIM_HandleTypeDef *htim, uint32_t channel, float angle)
{
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint32_t pulse = (uint32_t)(1000.0f + (angle / 180.0f) * 1000.0f);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}
