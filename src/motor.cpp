#include "motor.h"
#include <Arduino.h>

const float L = 0.0115;
const float LAMBDA = 0.0115;
const float R = 0.024;

const int STEPS_PER_REV = 200;
const int MICROSTEPPING = 16;
const float NB_PAS_PAR_METRE = (STEPS_PER_REV * MICROSTEPPING) / (2 * PI * R);

float x_robot   = 0.0;
float y_robot   = 0.0;
float angle_robot = 0.0;  // en radians

A4988 step_1(STEPS_PER_REV, D2,  D3);
A4988 step_2(STEPS_PER_REV, D6,  D7);
A4988 step_3(STEPS_PER_REV, D8,  D9);
A4988 step_4(STEPS_PER_REV, D10, D11);

void init_motors() {
    step_1.begin(100, MICROSTEPPING);
    step_2.begin(100, MICROSTEPPING);
    step_3.begin(100, MICROSTEPPING);
    step_4.begin(100, MICROSTEPPING);

    step_1.setSpeedProfile(step_1.LINEAR_SPEED, 1000, 1000);
    step_2.setSpeedProfile(step_2.LINEAR_SPEED, 1000, 1000);
    step_3.setSpeedProfile(step_3.LINEAR_SPEED, 1000, 1000);
    step_4.setSpeedProfile(step_4.LINEAR_SPEED, 1000, 1000);
}

long calcul_steps(float distance) {
    return (long)(distance * NB_PAS_PAR_METRE);
}

void sync_4_driver(long s1, long s2, long s3, long s4) {
    step_1.startMove(s1);
    step_2.startMove(s2);
    step_3.startMove(s3);
    step_4.startMove(s4);

    while (step_1.getStepsRemaining() || step_2.getStepsRemaining() ||
           step_3.getStepsRemaining() || step_4.getStepsRemaining()) {
        step_1.nextAction();
        step_2.nextAction();
        step_3.nextAction();
        step_4.nextAction();
    }
}

void avancer(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(s, -s, s, -s);
    x_robot += distance * cos(angle_robot);
    y_robot += distance * sin(angle_robot);
}

void reculer(float distance) {
    avancer(-distance);
}

void gauche(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(-s, -s, s, s);
    x_robot -= distance * sin(angle_robot);
    y_robot += distance * cos(angle_robot);
}

void droite(float distance) {
    gauche(-distance);
}

void rotation_gauche(float angle_deg) {
    float angle_rad = angle_deg * PI / 180.0;
    float arc = angle_rad * (L + LAMBDA);
    long s = calcul_steps(arc);
    sync_4_driver(-s, -s, -s, -s);
    angle_robot += angle_rad;
    angle_robot = atan2(sin(angle_robot), cos(angle_robot));
}

void rotation_droite(float angle_deg) {
    float angle_rad = angle_deg * PI / 180.0;
    float arc = angle_rad * (L + LAMBDA);
    long s = calcul_steps(arc);
    sync_4_driver(s, s, s, s);
    angle_robot -= angle_rad;
    angle_robot = atan2(sin(angle_robot), cos(angle_robot));
}

void diagonale_droite(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(s, 0, 0, -s);
    x_robot += distance * cos(angle_robot + PI / 4.0);
    y_robot += distance * sin(angle_robot + PI / 4.0);
}

void diagonale_gauche(float distance) {
    long s = calcul_steps(distance);
    sync_4_driver(0, -s, s, 0);
    x_robot += distance * cos(angle_robot - PI / 4.0);
    y_robot += distance * sin(angle_robot - PI / 4.0);
}


void aller_a_coord(float x_cible, float y_cible) {
    float dx = x_cible - x_robot;
    float dy = y_cible - y_robot;
    float angle_cible = atan2(dy, dx);

    float diff = angle_cible - angle_robot;
    diff = atan2(sin(diff), cos(diff));

    if (diff > 0.01) {
        rotation_gauche(diff * 180.0 / PI);
    } else if (diff < -0.01) {
        rotation_droite(-diff * 180.0 / PI);
    }

    float distance = sqrt(dx * dx + dy * dy);
    avancer(distance);
}

void tourner_vers_angle(float angle_cible_deg) {
    float angle_cible_rad = angle_cible_deg * PI / 180.0;
    float diff = angle_cible_rad - angle_robot;
    diff = atan2(sin(diff), cos(diff));

    if (diff > 0.01) {
        rotation_gauche(diff * 180.0 / PI);
    } else if (diff < -0.01) {
        rotation_droite(-diff * 180.0 / PI);
    }
}
