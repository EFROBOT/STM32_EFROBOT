/*
 * servo.c
 *
 *  Created on: 10 mai 2026
 *      Author: samue
 */


#include "servo.h"

void Servo_SetAngle(float angle)
{
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    float pulse = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180.0f);

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, (uint32_t)pulse);
}


void securiser_caisses(){
	  Servo_SetAngle(180);
	  safe_delay(280);
	  Servo_SetAngle(100);
}

void lacher_caisses(){
	  Servo_SetAngle(0);
	  safe_delay(450);
	  Servo_SetAngle(90);
}

