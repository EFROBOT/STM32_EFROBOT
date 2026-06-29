/* motor.h */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "imu.h"

#define PPR           1180
#define DIAMETRE_M    0.08f
#define PI            3.14159f

#define V_MAX         2.0f
#define V_MIN         0.4f
#define SEUIL         100
#define ZONE_DECEL    800
#define ACCEL_STEP    0.1

#define PWM_MAX       60000
#define PWM_MIN       20000

#define V             2.0f

#define RAYON_ROBOT_M   0.2

#define DIR1 (-1)
#define DIR2 (-1)
#define DIR3 (-1)
#define DIR4 (+1)



// PID
typedef struct {
    float kp, ki, kd;
    float integrale;
    float erreur_precedente;
} PID_Vitesse_t;

// position robot
typedef struct {
    float x;
    float y;
    float angle;
} Robot_Pos;

extern Robot_Pos robot_pos;
extern volatile uint8_t stop_mouvement;


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;
extern I2C_HandleTypeDef hi2c1;

// PID define
extern PID_Vitesse_t pid1, pid2, pid3, pid4;


int32_t distance_en_ticks(float metres);

void position_update(int32_t d1, int32_t d2, int32_t d3, int32_t d4);

void stop_motors(void);
void set_motor_pwm(TIM_HandleTypeDef *htim, uint32_t canal_0, uint32_t canal_1, float commande);

void PID_Vitesse_Init(PID_Vitesse_t *p, float kp, float ki, float kd);
float calculer_commande_PID(PID_Vitesse_t *p, float consigne, float vitesse_mesuree, float dt);
float calculer_vitesse(int32_t erreur, int32_t cible);


void test_PID_5s(float consigne_vitesse);
void mecanum_move_position(float v1, float v2, float v3, float v4, float metres);

void avancer(float metre);
void reculer(float metre);
void droite(float metre);
void gauche(float metre);
void diagonale_droite(float metre);
void diagonale_gauche(float metre);
void rotation_gauche(float angle);
void rotation_droite(float angle);
void arc_de_cercle(float rayon_cm, float angle_deg);

void tourner_vers_angle(float angle_cible);
void aller_a_coord(float x_cible, float y_cible);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
