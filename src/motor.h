
#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "A4988.h"

extern float x_robot;
extern float y_robot;
extern float angle_robot;


void init_motors();
void sync_4_driver(long steps1, long steps2, long steps3, long steps4);
long calcul_steps(float distance);


void avancer(float distance);
void reculer(float distance);

void droite(float distance);
void gauche(float distance);

void rotation_droite(float distance);
void rotation_gauche(float distance);

void diagonale_gauche(float distance);
void diagonale_droite(float distance);

void aller_a_coord(float x, float y);
void tourner_vers_angle(float angle_cible_deg);
#endif
