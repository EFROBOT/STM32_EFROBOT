/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Counter_Period 65535
#define vitesse_moteur 30000



#define PPR          1320
#define DIAMETRE_M   0.08 // = 80 cm
#define PI           3.14159

#define V_MAX  2.0   // tr/s vitesse max
#define V_MIN  0.4   // tr/s vitesse minimum (juste assez pour bouger)
#define SEUIL  100    // ticks d'arrêt
#define ZONE_DECEL 800


// #define Vitesse (PI * diametre_roue * RPM)/60)
#define Temps_test 3000
#define L 0.0115
#define Lambda 0.0115

#define V 2.0

#define PWM_MAX 60000
#define PWM_MIN 20000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int32_t encoder1, encoder2, encoder3, encoder4;

// asservissement
typedef struct {
    float kp, ki, kd;
    float integrale;
    float erreur_precedente;
} PID_Vitesse_t;

PID_Vitesse_t pid1, pid2, pid3, pid4;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t distance_en_ticks(float metres) {
    float tours = metres / (PI * DIAMETRE_M);
    return (int32_t)(tours * PPR);
}


void safe_delay(uint32_t ms) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(10);
    }
}

// set motors

void stop_motors(){
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // In1 -- avancer
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // In2 = Low -- reculer

	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); // In1
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // In2 = Low

	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); // In1
	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0); // In2 = Low

	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0); // In1
	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0); // In2 = Low
}

void set_one_motor(TIM_HandleTypeDef *htim, uint32_t canal_0, uint32_t canal_1, int dir){
	if (dir > 0){
		__HAL_TIM_SET_COMPARE(htim, canal_0, 0);
		__HAL_TIM_SET_COMPARE(htim, canal_1, vitesse_moteur);
	} else if (dir < 0){
		__HAL_TIM_SET_COMPARE(htim, canal_1, 0);
		__HAL_TIM_SET_COMPARE(htim, canal_0, vitesse_moteur);
	} else if (dir == 0) {
		__HAL_TIM_SET_COMPARE(htim, canal_0, 0);
		__HAL_TIM_SET_COMPARE(htim, canal_1, 0);
	}
}

void run_motors(int dir1, int dir2, int dir3, int dir4, float temps){
	set_one_motor(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, dir1);
	set_one_motor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, dir2);

	set_one_motor(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, dir3);
	set_one_motor(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, dir4);

	safe_delay(temps);
	stop_motors();
}


// asservissement

void PID_Vitesse_Init(PID_Vitesse_t *p, float kp, float ki, float kd) {
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integrale = 0;
    p->erreur_precedente = 0;
}

float calculer_commande_PID(PID_Vitesse_t *p, float consigne, float vitesse_mesuree, float dt) {
    float erreur = consigne - vitesse_mesuree;
    p->integrale += erreur * dt;
    float derivee = (erreur - p->erreur_precedente) / dt;
    p->erreur_precedente = erreur;

    float commande = (p->kp * erreur) + (p->ki * p->integrale) + (p->kd * derivee);
    return commande;
}

float calculer_commande_PID_position(PID_Vitesse_t *p, int32_t consigne_ticks,
                                      int32_t position_actuelle, float dt)
{
    float erreur   = (float)(consigne_ticks - position_actuelle);
    p->integrale  += erreur * dt;
    float derivee  = (erreur - p->erreur_precedente) / dt;
    p->erreur_precedente = erreur;

    float commande = (p->kp * erreur)
                   + (p->ki * p->integrale)
                   + (p->kd * derivee);
    return commande;
}


void set_motor_pwm(TIM_HandleTypeDef *htim, uint32_t canal_0,
                   uint32_t canal_1, float commande)
{
    // Clamp avant tout
    if (commande >  PWM_MAX) commande =  PWM_MAX;
    if (commande < -PWM_MAX) commande = -PWM_MAX;

    if (commande > 0   && commande < 8000)  commande = 0;
    if (commande < 0   && commande > -8000) commande = 0;

    if (commande >= 8000  && commande < PWM_MIN) commande = PWM_MIN;
    if (commande <= -8000 && commande > -PWM_MIN) commande = -PWM_MIN;


    uint32_t pwm_val = (uint32_t)fabsf(commande);

    if (commande > 0) {
        __HAL_TIM_SET_COMPARE(htim, canal_0, 0);
        __HAL_TIM_SET_COMPARE(htim, canal_1, pwm_val);
    } else if (commande < 0) {
        __HAL_TIM_SET_COMPARE(htim, canal_1, 0);
        __HAL_TIM_SET_COMPARE(htim, canal_0, pwm_val);
    } else {
        __HAL_TIM_SET_COMPARE(htim, canal_0, 0);
        __HAL_TIM_SET_COMPARE(htim, canal_1, 0);
    }
}

// fonction test pour regler les pid

void test_PID_5s(float consigne_vitesse) {
    uint32_t start_time = HAL_GetTick();
    uint32_t last_pid_time = start_time;

    int32_t last_enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t last_enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t last_enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t last_enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

    while ((HAL_GetTick() - start_time) < Temps_test) {

        HAL_IWDG_Refresh(&hiwdg);

        uint32_t current_time = HAL_GetTick();
        uint32_t delta_t_ms = current_time - last_pid_time;

        if (delta_t_ms >= 50) {
            float dt = delta_t_ms / 1000.0f;

            int32_t enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            int32_t enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
            int32_t enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
            int32_t enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

            int32_t d1 = enc1 - last_enc1;
            int32_t d2 = enc2 - last_enc2;
            int32_t d3 = enc3 - last_enc3;
            int32_t d4 = enc4 - last_enc4;

            if (d1 >  32767) d1 -= 65536;  if (d1 < -32768) d1 += 65536;
            if (d2 >  32767) d2 -= 65536;  if (d2 < -32768) d2 += 65536;
            if (d3 >  32767) d3 -= 65536;  if (d3 < -32768) d3 += 65536;
            if (d4 >  32767) d4 -= 65536;  if (d4 < -32768) d4 += 65536;

            float v1 = (d1 / 1320.0f) / dt;
            float v2 = (d2 / 1320.0f) / dt;
            float v3 = (d3 / 1320.0f) / dt;
            float v4 = (d4 / 1320.0f) / dt;

            float cmd1 = calculer_commande_PID(&pid1, consigne_vitesse, v1, dt);
            float cmd2 = calculer_commande_PID(&pid2, consigne_vitesse, v2, dt);
            float cmd3 = calculer_commande_PID(&pid3, consigne_vitesse, v3, dt);
            float cmd4 = calculer_commande_PID(&pid4, consigne_vitesse, v4, dt);

            set_motor_pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd1);
            set_motor_pwm(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd2);
            set_motor_pwm(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd3);
            set_motor_pwm(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd4);

            last_enc1 = enc1;
            last_enc2 = enc2;
            last_enc3 = enc3;
            last_enc4 = enc4;
            last_pid_time = current_time;

            char buf[128];
            int len = snprintf(
                buf, sizeof(buf),
                "V1:%.2f V2:%.2f V3:%.2f V4:%.2f | Enc1:%ld Enc2:%ld Enc3:%ld Enc4:%ld \r\n",
                v1, v2, v3, v4,
                enc1, enc2, enc3, enc4
            );
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 5);

        }

        HAL_Delay(1);
    }

    stop_motors();
}




void mecanum_move_position(float vHR, float vBR, float vHL, float vBL,
                           float metres)
{
    int32_t ticks_cible = (int32_t)(metres / (PI * DIAMETRE_M) * PPR);

    // Reset PIDs
    pid1.integrale = 0; pid1.erreur_precedente = 0;
    pid2.integrale = 0; pid2.erreur_precedente = 0;
    pid3.integrale = 0; pid3.erreur_precedente = 0;
    pid4.integrale = 0; pid4.erreur_precedente = 0;

    // Reset encodeurs
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    int32_t pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    int32_t last_enc1 = 0, last_enc2 = 0;
    int32_t last_enc3 = 0, last_enc4 = 0;
    uint32_t last_time = HAL_GetTick();

    // Cible signée par roue selon le mouvement
    int32_t cible1 = (int32_t)(vHR * ticks_cible);
    int32_t cible2 = (int32_t)(vBR * ticks_cible);
    int32_t cible3 = (int32_t)(vHL * ticks_cible);
    int32_t cible4 = (int32_t)(vBL * ticks_cible);


    while (1) {
        HAL_IWDG_Refresh(&hiwdg);

        uint32_t now      = HAL_GetTick();
        uint32_t delta_ms = now - last_time;

        if (delta_ms >= 50) {
            float dt = delta_ms / 1000.0f;

            int32_t enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            int32_t enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
            int32_t enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
            int32_t enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

            int32_t d1 = enc1 - last_enc1;
            int32_t d2 = enc2 - last_enc2;
            int32_t d3 = enc3 - last_enc3;
            int32_t d4 = enc4 - last_enc4;

            if (d1 >  32767) d1 -= 65536; if (d1 < -32768) d1 += 65536;
            if (d2 >  32767) d2 -= 65536; if (d2 < -32768) d2 += 65536;
            if (d3 >  32767) d3 -= 65536; if (d3 < -32768) d3 += 65536;
            if (d4 >  32767) d4 -= 65536; if (d4 < -32768) d4 += 65536;

            pos1 += d1; pos2 += d2;
            pos3 += d3; pos4 += d4;

            // PID position → PWM directement
            float cmd1 = calculer_commande_PID_position(&pid1, cible1, pos1, dt);
            float cmd2 = calculer_commande_PID_position(&pid2, cible2, pos2, dt);
            float cmd3 = calculer_commande_PID_position(&pid3, cible3, pos3, dt);
            float cmd4 = calculer_commande_PID_position(&pid4, cible4, pos4, dt);

            if (abs(cible1 - pos1) < SEUIL) cmd1 = 0;
            if (abs(cible2 - pos2) < SEUIL) cmd2 = 0;
            if (abs(cible3 - pos3) < SEUIL) cmd3 = 0;
            if (abs(cible4 - pos4) < SEUIL) cmd4 = 0;

            set_motor_pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd1);
            set_motor_pwm(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd2);
            set_motor_pwm(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd3);
            set_motor_pwm(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd4);

            last_enc1 = enc1; last_enc2 = enc2;
            last_enc3 = enc3; last_enc4 = enc4;
            last_time = now;

            char buf[200];
            int len = snprintf(buf, sizeof(buf),
                "Enc1:%ld Enc2:%ld Enc3:%ld Enc4:%ld | E1:%ld E2:%ld E3:%ld E4:%ld \r\n",
				enc1, enc2, enc3, enc4,
                cible1-pos1, cible2-pos2, cible3-pos3, cible4-pos4);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 5);

            // Arrêt quand toutes les roues ont atteint leur cible
            if (abs(cible1-pos1) < SEUIL && abs(cible2-pos2) < SEUIL &&
                abs(cible3-pos3) < SEUIL && abs(cible4-pos4) < SEUIL) {
                break;
            }
        }
        HAL_Delay(1);
    }
    stop_motors();
}



void mecanum_move_position_v2(float vHR, float vBR, float vHL, float vBL,
                               float metres)
{
    int32_t ticks_cible = (int32_t)(metres / (PI * DIAMETRE_M) * PPR);

    // Reset PIDs
    pid1.integrale = 0; pid1.erreur_precedente = 0;
    pid2.integrale = 0; pid2.erreur_precedente = 0;
    pid3.integrale = 0; pid3.erreur_precedente = 0;
    pid4.integrale = 0; pid4.erreur_precedente = 0;

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    int32_t pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    int32_t last_enc1 = 0, last_enc2 = 0;
    int32_t last_enc3 = 0, last_enc4 = 0;
    uint32_t last_time = HAL_GetTick();
    uint32_t timeout   = HAL_GetTick();

    int32_t cible1 = (int32_t)(vHR * ticks_cible);
    int32_t cible2 = (int32_t)(vBR * ticks_cible);
    int32_t cible3 = (int32_t)(vHL * ticks_cible);
    int32_t cible4 = (int32_t)(vBL * ticks_cible);


    while (1) {
        HAL_IWDG_Refresh(&hiwdg);

        if ((HAL_GetTick() - timeout) > 10000) break;

        uint32_t now      = HAL_GetTick();
        uint32_t delta_ms = now - last_time;

        if (delta_ms >= 50) {
            float dt = delta_ms / 1000.0f;

            int32_t enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            int32_t enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
            int32_t enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
            int32_t enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

            int32_t d1 = enc1 - last_enc1;
            int32_t d2 = enc2 - last_enc2;
            int32_t d3 = enc3 - last_enc3;
            int32_t d4 = enc4 - last_enc4;

            if (d1 >  32767) d1 -= 65536; if (d1 < -32768) d1 += 65536;
            if (d2 >  32767) d2 -= 65536; if (d2 < -32768) d2 += 65536;
            if (d3 >  32767) d3 -= 65536; if (d3 < -32768) d3 += 65536;
            if (d4 >  32767) d4 -= 65536; if (d4 < -32768) d4 += 65536;

            pos1 += d1; pos2 += d2;
            pos3 += d3; pos4 += d4;

            float v1 = (d1 / 1320.0f) / dt;
            float v2 = (d2 / 1320.0f) / dt;
            float v3 = (d3 / 1320.0f) / dt;
            float v4 = (d4 / 1320.0f) / dt;

            int32_t err1 = abs(cible1 - pos1);
            int32_t err2 = abs(cible2 - pos2);
            int32_t err3 = abs(cible3 - pos3);
            int32_t err4 = abs(cible4 - pos4);

            float vc1, vc2, vc3, vc4;

            // Roue 1
            if      (err1 < SEUIL)      vc1 = 0;
            else if (err1 > ZONE_DECEL) vc1 = V_MAX;
            else                        vc1 = V_MAX * ((float)err1 / ZONE_DECEL);
            if (vc1 > 0 && vc1 < V_MIN) vc1 = V_MIN;
            if (cible1 < 0) vc1 = -vc1;

            // Roue 2
            if      (err2 < SEUIL)      vc2 = 0;
            else if (err2 > ZONE_DECEL) vc2 = V_MAX;
            else                        vc2 = V_MAX * ((float)err2 / ZONE_DECEL);
            if (vc2 > 0 && vc2 < V_MIN) vc2 = V_MIN;
            if (cible2 < 0) vc2 = -vc2;

            // Roue 3
            if      (err3 < SEUIL)      vc3 = 0;
            else if (err3 > ZONE_DECEL) vc3 = V_MAX;
            else                        vc3 = V_MAX * ((float)err3 / ZONE_DECEL);
            if (vc3 > 0 && vc3 < V_MIN) vc3 = V_MIN;
            if (cible3 < 0) vc3 = -vc3;

            // Roue 4
            if      (err4 < SEUIL)      vc4 = 0;
            else if (err4 > ZONE_DECEL) vc4 = V_MAX;
            else                        vc4 = V_MAX * ((float)err4 / ZONE_DECEL);
            if (vc4 > 0 && vc4 < V_MIN) vc4 = V_MIN;
            if (cible4 < 0) vc4 = -vc4;

            float cmd1 = (vc1 != 0) ? calculer_commande_PID(&pid1, vc1, v1, dt) : 0;
            float cmd2 = (vc2 != 0) ? calculer_commande_PID(&pid2, vc2, v2, dt) : 0;
            float cmd3 = (vc3 != 0) ? calculer_commande_PID(&pid3, vc3, v3, dt) : 0;
            float cmd4 = (vc4 != 0) ? calculer_commande_PID(&pid4, vc4, v4, dt) : 0;

            set_motor_pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd1);
            set_motor_pwm(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd2);
            set_motor_pwm(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd3);
            set_motor_pwm(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd4);

            last_enc1 = enc1; last_enc2 = enc2;
            last_enc3 = enc3; last_enc4 = enc4;
            last_time = now;

            char buf[200];
            int len = snprintf(buf, sizeof(buf),
                "Enc1:%ld Enc2:%ld Enc3:%ld Enc4:%ld\r\n",
                enc1, enc2, enc3, enc4);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 5);

            if (err1 < SEUIL && err2 < SEUIL && err3 < SEUIL && err4 < SEUIL) break;
        }
        HAL_Delay(1);
    }
    stop_motors();
}



// mouvement

// fonction test

void mecanum_move_pid(float vFL, float vFR, float vBL, float vBR, uint32_t duree_ms)
{
    // Reset PIDs
    pid1.integrale = 0; pid1.erreur_precedente = 0;
    pid2.integrale = 0; pid2.erreur_precedente = 0;
    pid3.integrale = 0; pid3.erreur_precedente = 0;
    pid4.integrale = 0; pid4.erreur_precedente = 0;

    uint32_t start_time    = HAL_GetTick();
    uint32_t last_pid_time = start_time;

    int32_t last_enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t last_enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t last_enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t last_enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

    while ((HAL_GetTick() - start_time) < duree_ms) {
        HAL_IWDG_Refresh(&hiwdg);

        uint32_t current_time = HAL_GetTick();
        uint32_t delta_t_ms   = current_time - last_pid_time;

        if (delta_t_ms >= 50) {
            float dt = delta_t_ms / 1000.0f;

            int32_t enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            int32_t enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
            int32_t enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
            int32_t enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

            int32_t d1 = enc1 - last_enc1;
            int32_t d2 = enc2 - last_enc2;
            int32_t d3 = enc3 - last_enc3;
            int32_t d4 = enc4 - last_enc4;

            if (d1 >  32767) d1 -= 65536; if (d1 < -32768) d1 += 65536;
            if (d2 >  32767) d2 -= 65536; if (d2 < -32768) d2 += 65536;
            if (d3 >  32767) d3 -= 65536; if (d3 < -32768) d3 += 65536;
            if (d4 >  32767) d4 -= 65536; if (d4 < -32768) d4 += 65536;

            float v1 = (d1 / 1320.0f) / dt;
            float v2 = (d2 / 1320.0f) / dt;
            float v3 = (d3 / 1320.0f) / dt;
            float v4 = (d4 / 1320.0f) / dt;

            // Consigne différente par roue
            float cmd1 = calculer_commande_PID(&pid1, vFL, v1, dt);
            float cmd2 = calculer_commande_PID(&pid2, vFR, v2, dt);
            float cmd3 = calculer_commande_PID(&pid3, vBL, v3, dt);
            float cmd4 = calculer_commande_PID(&pid4, vBR, v4, dt);

            set_motor_pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd1);
            set_motor_pwm(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd2);
            set_motor_pwm(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd3);
            set_motor_pwm(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd4);

            last_enc1 = enc1; last_enc2 = enc2;
            last_enc3 = enc3; last_enc4 = enc4;
            last_pid_time = current_time;


            char buf[200];
            int len = snprintf(buf, sizeof(buf),
                "HR:%ld BR:%ld HL:%ld BL:%ld \r\n",
                enc1, enc2, enc3, enc4);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 5);

        }
        HAL_Delay(1);
    }
    stop_motors();
}



void avancer(uint32_t time){
	mecanum_move_pid(V,  V,  V,  V, time);
}

void reculer(uint32_t time){
	mecanum_move_pid(-V,  -V,  -V,  -V, time);
}

void droite(uint32_t time){
	mecanum_move_pid(V,  -V,  -V,  V, time);
}

void gauche(uint32_t time){
	mecanum_move_pid(-V,  V,  V,  -V, time);
}

void diagonale_droite(uint32_t time){
	mecanum_move_pid(V,  0,  0,  V, time);
}

void diagonale_gauche(uint32_t time){
	mecanum_move_pid(0 ,  V,  V,  0, time);
}




/*
void rotation_droite(float angle){
	float angle_rad = angle * PI / 180.0;
	float arc = angle_rad * (L + LAMBDA);
	run_motors(1, 1, 1, 1, Temps_test);
}

void rotation_gauche(float angle){
	float angle_rad = angle * PI / 180.0;
	float arc = angle_rad * (L + LAMBDA);
	run_motors(-1, -1, -1 -1, Temps_test);
}*/


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Timer 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // motor 1 In1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // motor 1 In2
  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // motor 2 In1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // motor 2 In2
  __HAL_TIM_MOE_ENABLE(&htim1);

  // Timer 2 & 3 & 4 & 5 --> encodeur
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // Timer 8
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // motor 3 In1
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // motor 3 In2
  __HAL_TIM_MOE_ENABLE(&htim8);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // motor 4 In1
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // motor 4 In2
  __HAL_TIM_MOE_ENABLE(&htim8);

  // PIDS init

  PID_Vitesse_Init(&pid1, 60000.0, 33000.0, 0.0); // ok
  PID_Vitesse_Init(&pid2, 60000.0, 35000.0, 0.0); // ok
  PID_Vitesse_Init(&pid3, 60000.0, 25000.0, 0.0); // ok
  PID_Vitesse_Init(&pid4, 50000.0, 25000.0, 0.0); // ok


  //PID_Vitesse_Init(&pid1, 33.0, 5.0, 5.5);
  //PID_Vitesse_Init(&pid2, 30.0, 2.0, 5.5);
  //PID_Vitesse_Init(&pid3, 23.0, 0.0, 5.5);
  //PID_Vitesse_Init(&pid4, 25.0, 0.0, 5.5);

  // Imu i2c
  //safe_delay(1000);
  //avancer(150);

    //safe_delay(1000);
    //test_PID_5s(2.0);
    //safe_delay(1000);
    //test_PID_5s_reculer(2.0);

  	safe_delay(1000);
  	//mecanum_move_position(1, 1, 1, 0, 0.50);
  	mecanum_move_position_v2(1, 1, 1, 1, 2);
  	safe_delay(1000);
  	mecanum_move_position_v2(-1, -1, -1, -1, 2);

  	//mecanum_move_position(-1, -1, -1, -1, 0.50);


    //mecanum_move_pid(2.0, 2.0, 2.0, 2.0, 2000);
    //safe_delay(500);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_IWDG_Refresh(&hiwdg);


      HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535 ;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|led_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 led_Pin PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|led_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : interrupteur_Pin */
  GPIO_InitStruct.Pin = interrupteur_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(interrupteur_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
