#include "trajectory.h"
#include "motor.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
// ── helpers ─────────────────────────────────────────────────────────────────

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Interpolation Bézier cubique : t ∈ [0,1]
// P0=départ, P3=arrivée, P1=P0+tangente_entrée, P2=P3-tangente_sortie
static float bezier_cubic(float p0, float p1, float p2, float p3, float t) {
    float u = 1.0f - t;
    return u*u*u*p0 + 3.0f*u*u*t*p1 + 3.0f*u*t*t*p2 + t*t*t*p3;
}

// ── init ────────────────────────────────────────────────────────────────────

void traj_init(Trajectory *t) {
    memset(t, 0, sizeof(Trajectory));
}

void traj_add(Trajectory *t, float v1, float v2, float v3, float v4, float dist_m) {
    if (t->count >= MAX_SEGMENTS) return;
    TrajectorySegment *s = &t->segments[t->count++];
    s->v1 = v1; s->v2 = v2; s->v3 = v3; s->v4 = v4;
    s->distance_m = dist_m;
}

// ── raccourcis ───────────────────────────────────────────────────────────────

void traj_avancer      (Trajectory *t, float m) { traj_add(t,  1,  1,  1,  1, m); }
void traj_reculer      (Trajectory *t, float m) { traj_add(t, -1, -1, -1, -1, m); }
void traj_gauche       (Trajectory *t, float m) { traj_add(t,  1, -1, -1,  1, m); }
void traj_droite       (Trajectory *t, float m) { traj_add(t, -1,  1,  1, -1, m); }
void traj_diag_droite  (Trajectory *t, float m) { traj_add(t,  1,  0,  0,  1, m); }
void traj_diag_gauche  (Trajectory *t, float m) { traj_add(t,  0,  1,  1,  0, m); }

void traj_rotation_gauche(Trajectory *t, float deg) {
    float d = (deg / 360.0f) * (2.0f * PI * RAYON_ROBOT_M);
    traj_add(t,  1,  1, -1, -1, d);
}
void traj_rotation_droite(Trajectory *t, float deg) {
    float d = (deg / 360.0f) * (2.0f * PI * RAYON_ROBOT_M);
    traj_add(t, -1, -1,  1,  1, d);
}

// ── exécution ────────────────────────────────────────────────────────────────

// Zone de transition (en ticks) sur laquelle on interpole entre deux segments
#define BLEND_TICKS  200     // ~15 cm selon ton PPR
#define PID_DT_MS    50

void traj_execute(Trajectory *t) {
    if (t->count == 0) return;

    // Reset PIDs
    pid1.integrale = 0; pid1.erreur_precedente = 0;
    pid2.integrale = 0; pid2.erreur_precedente = 0;
    pid3.integrale = 0; pid3.erreur_precedente = 0;
    pid4.integrale = 0; pid4.erreur_precedente = 0;

    float v_ramp = V_MIN;

    for (uint8_t seg_i = 0; seg_i < t->count; seg_i++) {
        TrajectorySegment *cur  = &t->segments[seg_i];
        TrajectorySegment *next = (seg_i + 1 < t->count) ? &t->segments[seg_i + 1] : NULL;

        int32_t ticks_cible = distance_en_ticks(cur->distance_m * 100.0f); // m→cm

        // Reset encodeurs relatifs au segment
        int32_t last_enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t last_enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
        int32_t last_enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
        int32_t last_enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

        int32_t pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
        uint32_t last_time = HAL_GetTick();
        uint32_t timeout   = HAL_GetTick();

        // Dernier segment : on freine normalement
        int is_last = (next == NULL);

        while (1) {
            HAL_IWDG_Refresh(&hiwdg);
            if ((HAL_GetTick() - timeout) > 10000) goto end;

            uint32_t now      = HAL_GetTick();
            uint32_t delta_ms = now - last_time;

            if (delta_ms >= PID_DT_MS) {
                float dt = delta_ms / 1000.0f;

                int32_t enc1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
                int32_t enc2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
                int32_t enc3 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
                int32_t enc4 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

                int32_t d1 = enc1 - last_enc1;
                int32_t d2 = enc2 - last_enc2;
                int32_t d3 = enc3 - last_enc3;
                int32_t d4 = enc4 - last_enc4;

                // overflow 16-bit
                if (d1 >  32767) d1 -= 65536; if (d1 < -32768) d1 += 65536;
                if (d2 >  32767) d2 -= 65536; if (d2 < -32768) d2 += 65536;
                if (d3 >  32767) d3 -= 65536; if (d3 < -32768) d3 += 65536;
                if (d4 >  32767) d4 -= 65536; if (d4 < -32768) d4 += 65536;

                position_update(d1, d2, d3, d4);

                d1 *= DIR1; d2 *= DIR2; d3 *= DIR3; d4 *= DIR4;
                pos1 += d1; pos2 += d2; pos3 += d3; pos4 += d4;

                float v1_mes = (d1 / 1320.0f) / dt;
                float v2_mes = (d2 / 1320.0f) / dt;
                float v3_mes = (d3 / 1320.0f) / dt;
                float v4_mes = (d4 / 1320.0f) / dt;

                // Avancement moyen normalisé [0..1] dans ce segment
                int32_t pos_avg = (abs(pos1) + abs(pos2) + abs(pos3) + abs(pos4)) / 4;
                float progress = clampf((float)pos_avg / (float)ticks_cible, 0.0f, 1.0f);

                // ── Calcul de la vitesse scalaire ────────────────────────────
                int32_t err_max = ticks_cible - pos_avg;
                if (err_max < 0) err_max = 0;

                float vitesse_base;

                if (!is_last && err_max < BLEND_TICKS) {
                    // Zone de blend : on NE freine PAS, on maintient V_MAX
                    vitesse_base = V_MAX;
                } else {
                    vitesse_base = calculer_vitesse(err_max, ticks_cible);
                }

                // Ramp-up global (partagé entre segments)
                if (vitesse_base > v_ramp) vitesse_base = v_ramp;
                v_ramp += ACCEL_STEP;
                if (v_ramp > V_MAX) v_ramp = V_MAX;

                // ── Interpolation Bézier des vecteurs moteurs ────────────────
                float vc1, vc2, vc3, vc4;

                if (!is_last && progress > (1.0f - (float)BLEND_TICKS / ticks_cible)) {
                    // t local dans la zone de blend [0..1]
                    float t_blend = (progress - (1.0f - (float)BLEND_TICKS / ticks_cible))
                                    / ((float)BLEND_TICKS / ticks_cible);
                    t_blend = clampf(t_blend, 0.0f, 1.0f);

                    // Bézier : P0=cur, P3=next, tangentes = direction courante
                    // (Schéma simple : point de contrôle = 1/3 de la transition)
                    vc1 = bezier_cubic(cur->v1, cur->v1, next->v1, next->v1, t_blend) * vitesse_base;
                    vc2 = bezier_cubic(cur->v2, cur->v2, next->v2, next->v2, t_blend) * vitesse_base;
                    vc3 = bezier_cubic(cur->v3, cur->v3, next->v3, next->v3, t_blend) * vitesse_base;
                    vc4 = bezier_cubic(cur->v4, cur->v4, next->v4, next->v4, t_blend) * vitesse_base;
                } else {
                    vc1 = cur->v1 * vitesse_base;
                    vc2 = cur->v2 * vitesse_base;
                    vc3 = cur->v3 * vitesse_base;
                    vc4 = cur->v4 * vitesse_base;
                }

                // ── PID ──────────────────────────────────────────────────────
                float cmd1 = calculer_commande_PID(&pid1, vc1, v1_mes, dt);
                float cmd2 = calculer_commande_PID(&pid2, vc2, v2_mes, dt);
                float cmd3 = calculer_commande_PID(&pid3, vc3, v3_mes, dt);
                float cmd4 = calculer_commande_PID(&pid4, vc4, v4_mes, dt);

                set_motor_pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd1);
                set_motor_pwm(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd2);
                set_motor_pwm(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, cmd3);
                set_motor_pwm(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, cmd4);

                last_enc1 = enc1; last_enc2 = enc2;
                last_enc3 = enc3; last_enc4 = enc4;
                last_time = now;

                // ── Condition de sortie du segment ───────────────────────────
                if (!is_last && pos_avg >= (ticks_cible - BLEND_TICKS / 2)) {
                    // On passe au segment suivant AVANT l'arrêt complet
                    break;
                }
                if (is_last && err_max < SEUIL) {
                    break;
                }
            }
            HAL_Delay(1);
        }
    }

end:
    stop_motors();
}
