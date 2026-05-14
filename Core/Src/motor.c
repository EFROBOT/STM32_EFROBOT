/* motor.c */
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// init
PID_Vitesse_t pid1, pid2, pid3, pid4;
Robot_Pos robot_pos = {0.0f, 0.0f, 0.0f};


int32_t distance_en_ticks(float cm) {
    float tours = cm / (PI * DIAMETRE_M);
    return (int32_t)(tours * PPR);
}


// motor control

void stop_motors(void) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
}


void set_motor_pwm(TIM_HandleTypeDef *htim, uint32_t canal_0,
                   uint32_t canal_1, float commande)
{
    if (commande >  PWM_MAX) commande =  PWM_MAX;
    if (commande < -PWM_MAX) commande = -PWM_MAX;

    if (commande >  0    && commande <  8000) commande = 0;
    if (commande <  0    && commande > -8000) commande = 0;

    if (commande >=  8000 && commande <  PWM_MIN) commande =  PWM_MIN;
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
// --------------------------------------------------------------------------------------

// PID
void PID_Vitesse_Init(PID_Vitesse_t *p, float kp, float ki, float kd) {
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integrale = 0;
    p->erreur_precedente = 0;
}

float calculer_commande_PID(PID_Vitesse_t *p, float consigne,
                             float vitesse_mesuree, float dt)
{
    float erreur = consigne - vitesse_mesuree;
    p->integrale += erreur * dt;
    float derivee = (erreur - p->erreur_precedente) / dt;
    p->erreur_precedente = erreur;
    return (p->kp * erreur) + (p->ki * p->integrale) + (p->kd * derivee);
}

// --------------------------------------------------------------------------------------

// Trouver les pid de chaque moteurs
void test_PID_5s(float consigne_vitesse) {
    uint32_t start_time    = HAL_GetTick();
    uint32_t last_pid_time = start_time;

    int32_t last_enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t last_enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t last_enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t last_enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

    while ((HAL_GetTick() - start_time) < 3000) {
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

            float v1 = - (d1 / 1320.0f) / dt;
            float v2 = - (d2 / 1320.0f) / dt;
            float v3 = - (d3 / 1320.0f) / dt;
            float v4 = (d4 / 1320.0f) / dt;

            float cmd1 = calculer_commande_PID(&pid1, consigne_vitesse, v1, dt);
            float cmd2 = calculer_commande_PID(&pid2, consigne_vitesse, v2, dt);
            float cmd3 = calculer_commande_PID(&pid3, consigne_vitesse, v3, dt);
            float cmd4 = calculer_commande_PID(&pid4, consigne_vitesse, v4, dt);

            set_motor_pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd1);
            //set_motor_pwm(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd2);
            //set_motor_pwm(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd3);
            //set_motor_pwm(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd4);

            last_enc1 = enc1; last_enc2 = enc2;
            last_enc3 = enc3; last_enc4 = enc4;
            last_pid_time = current_time;

            char buf[128];
            int len = snprintf(buf, sizeof(buf),
                "V1:%.2f V2:%.2f V3:%.2f V4:%.2f | Enc1:%ld Enc2:%ld Enc3:%ld Enc4:%ld\r\n",
                v1, v2, v3, v4, enc1, enc2, enc3, enc4);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
        }
        HAL_Delay(1);
    }
    stop_motors();
}


// --------------------------------------------------------------------------------------


float calculer_vitesse(int32_t erreur, int32_t cible)
{
    float vc = 0.0f;

    if (erreur < SEUIL) {
        vc = 0.0f;
    }
    else if (erreur > ZONE_DECEL) {
        vc = V_MAX;
    }
    else {
        vc = V_MAX * ((float)erreur / ZONE_DECEL);
    }

    if (vc > 0.0f && vc < V_MIN) {
        vc = V_MIN;
    }
    if (cible < 0) {
        vc = -vc;
    }

    return vc;
}


void move(float vinit1, float vinit2, float vinit3, float vinit4, float cm) {
	stop_mouvement = 0;
    int32_t ticks_cible = distance_en_ticks(cm);

    pid1.integrale = 0; pid1.erreur_precedente = 0;
    pid2.integrale = 0; pid2.erreur_precedente = 0;
    pid3.integrale = 0; pid3.erreur_precedente = 0;
    pid4.integrale = 0; pid4.erreur_precedente = 0;

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    int32_t last_enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t last_enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t last_enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t last_enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

    int32_t pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    uint32_t last_time = HAL_GetTick();
    uint32_t timeout = HAL_GetTick();

    int32_t cible1 = (int32_t)(vinit1 * ticks_cible);
    int32_t cible2 = (int32_t)(vinit2 * ticks_cible);
    int32_t cible3 = (int32_t)(vinit3 * ticks_cible);
    int32_t cible4 = (int32_t)(vinit4 * ticks_cible);

    // Variable pour l'anti-dépassement
    int32_t err_max_precedent = ticks_cible;

    while (1) {
        HAL_IWDG_Refresh(&hiwdg);

        if (stop_mouvement) break;

        if ((HAL_GetTick() - timeout) > 10000) break;

        uint32_t now = HAL_GetTick();
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

            position_update(d1, d2, d3, d4);

            d1 *= DIR1;
            d2 *= DIR2;
            d3 *= DIR3;
            d4 *= DIR4;

            pos1 += d1; pos2 += d2; pos3 += d3; pos4 += d4;

            float v1 = (d1 / 1320.0f) / dt;
            float v2 = (d2 / 1320.0f) / dt;
            float v3 = (d3 / 1320.0f) / dt;
            float v4 = (d4 / 1320.0f) / dt;

            int32_t err1 = abs(cible1 - pos1);
            int32_t err2 = abs(cible2 - pos2);
            int32_t err3 = abs(cible3 - pos3);
            int32_t err4 = abs(cible4 - pos4);

            int32_t err_max = err1;
            if (err2 > err_max) err_max = err2;
            if (err3 > err_max) err_max = err3;
            if (err4 > err_max) err_max = err4;

            float vitesse_base = calculer_vitesse(err_max, ticks_cible);

            float vc1 = vitesse_base * vinit1;
            float vc2 = vitesse_base * vinit2;
            float vc3 = vitesse_base * vinit3;
            float vc4 = vitesse_base * vinit4;

            float cmd1 = calculer_commande_PID(&pid1, vc1, v1, dt);
            float cmd2 = calculer_commande_PID(&pid2, vc2, v2, dt);
            float cmd3 = calculer_commande_PID(&pid3, vc3, v3, dt);
            float cmd4 = calculer_commande_PID(&pid4, vc4, v4, dt);

            set_motor_pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd1);
            set_motor_pwm(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd2);
            set_motor_pwm(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd3);
            set_motor_pwm(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd4);

            last_enc1 = enc1; last_enc2 = enc2;
            last_enc3 = enc3; last_enc4 = enc4;
            last_time = now;

            char buf[200];
            int len = snprintf(buf, sizeof(buf), "cmd1:%.0f cmd2:%.0f cmd3:%.0f cmd4:%.0f enc1:%f enc2:%f enc3:%f enc4:%f\r\n",
                     cmd1, cmd2, cmd3, cmd4, enc1, enc2, enc3, enc4);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);

            // Conditions d'arret (soit on est sous le seuil; soit le robot accelere aprés etre passé juste à cote du seuil
            if (err_max < SEUIL) {
                break;
            }
            if (err_max > err_max_precedent && err_max < 300) {
                break;
            }

            err_max_precedent = err_max;
        }
        HAL_Delay(1);
    }
    stop_motors();
}

// --------------------------------------------------------------------------------------

void avancer(float metre){
	move( 1,  1,  1,  1, metre);
}

void reculer(float metre){
	move(-1, -1, -1, -1, metre);
}

void droite(float metre){
	move( 1, -1, -1,  1, metre);
}

void gauche(float metre){
	move(-1,  1,  1, -1, metre);
}

void diagonale_droite(float metre){
	move( 1,  0,  0,  1, metre);
}

void diagonale_gauche(float metre){
	move( 0,  1,  1,  0, metre);
}

void rotation_gauche(float angle_deg) {
    float distance_cm = (angle_deg / 360.0f) * (2.0f * PI * RAYON_ROBOT_M);
    move(1, 1, -1, -1, distance_cm);
}

void rotation_droite(float angle_deg) {
    float distance_cm = (angle_deg / 360.0f) * (2.0f * PI * RAYON_ROBOT_M);
    move(-1, -1, 1, 1, distance_cm);
}


// --------------------------------------------------------------------------------------

void tourner_vers_angle(float angle_cible) {
    float diff = angle_cible - robot_pos.angle;

    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    if (diff > 0.0f) {
        rotation_gauche(diff);
    } else if (diff < 0.0f) {
        rotation_droite(-diff);
    }

    robot_pos.angle = angle_cible;
}

void aller_a_coord(float x_cible, float y_cible) {
    float dx = x_cible - robot_pos.x;
    float dy = y_cible - robot_pos.y;

    float distance_cm = sqrtf(dx * dx + dy * dy);
    if (distance_cm < 1.0f) return;

    float angle_cible_rad = atan2f(dy, dx);
    float angle_robot_rad = robot_pos.angle * PI / 180.0f;
    float diff_angle = angle_cible_rad - angle_robot_rad;

    float vx_local = cosf(diff_angle);
    float vy_local = sinf(diff_angle);

    float v1 = vx_local - vy_local;
    float v2 = vx_local + vy_local;
    float v3 = vx_local + vy_local;
    float v4 = vx_local - vy_local;

    char buf[200];
	int len = snprintf(buf, sizeof(buf),
		"x_cible:%.2f robot_pos.x:%.2f y_cible:%.2f robot_pos.y:%.2f \r\n",
		x_cible, robot_pos.x, y_cible, robot_pos.y);
	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 5);


    // On passe la distance en mètres à move()
    move(v1, v2, v3, v4, distance_cm / 100.0f);
}



/*
void aller_a_coord(float x_cible, float y_cible) {
	float dx = x_cible - robot_pos.x;
	float dy = y_cible - robot_pos.y;

	float diff = atan2(sin(atan2(dy, dx) - robot_pos.angle), cos(atan2(dy, dx) - robot_pos.angle));

    if (diff > 0.01f) {
    	rotation_gauche(diff * 180.0f / PI);
    }
    else if (diff < -0.01f) {
    	rotation_droite(-diff * 180.0f / PI);
    }

    avancer(sqrt(dx * dx + dy * dy));
}*/


void arc_de_cercle(float rayon_m, float angle_deg) {
    float distance_arc_m = fabsf((angle_deg / 360.0f) * 2.0f * PI * rayon_m);

    float v_rot = RAYON_ROBOT_M / rayon_m;

    float v1, v2, v3, v4;

    if (angle_deg > 0.0f) {
        v1 = 1.0f + v_rot;
        v2 = 1.0f + v_rot;
        v3 = 1.0f - v_rot;
        v4 = 1.0f - v_rot;
    } else {
        v1 = 1.0f - v_rot;
        v2 = 1.0f - v_rot;
        v3 = 1.0f + v_rot;
        v4 = 1.0f + v_rot;
    }

    move(v1, v2, v3, v4, distance_arc_m);
}

// --------------------------------------------------------------------------------------
// recuperer posititon
void position_update(int32_t d1, int32_t d2, int32_t d3, int32_t d4) {

    float r1 = (- d1 / (float)PPR) * (PI * DIAMETRE_M) * 100;
    float r2 = (- d2 / (float)PPR) * (PI * DIAMETRE_M) * 100;
    float r3 = (- d3 / (float)PPR) * (PI * DIAMETRE_M) * 100;
    float r4 = (d4 / (float)PPR) * (PI * DIAMETRE_M) * 100;

    float vx = ( r1 + r2 + r3 + r4) / 4.0f;
    float vy = (-r1 + r2 + r3 - r4) / 4.0f;

    float angle_rad = robot_pos.angle * PI / 180.0f;

    robot_pos.x += vx * cosf(angle_rad) - vy * sinf(angle_rad);
    robot_pos.y += vx * sinf(angle_rad) + vy * cosf(angle_rad);
}
