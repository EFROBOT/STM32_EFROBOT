#ifndef SERVO_H
#define SERVO_H

#include "main.h"

extern TIM_HandleTypeDef htim16;

#define SERVO_MIN_PULSE  600
#define SERVO_MAX_PULSE  2400

void Servo_SetAngle(float angle);
void lacher_caisses();
void securiser_caisses();

#endif
