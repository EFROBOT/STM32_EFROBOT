#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

#define MAX_SEGMENTS 16

typedef struct {
    float v1, v2, v3, v4;   // vecteurs normalisés [-1, 1]
    float distance_m;        // distance en mètres
} TrajectorySegment;

typedef struct {
    TrajectorySegment segments[MAX_SEGMENTS];
    uint8_t count;
} Trajectory;

// API publique
void traj_init(Trajectory *t);
void traj_add(Trajectory *t, float v1, float v2, float v3, float v4, float dist_m);

// Raccourcis (même logique que tes fonctions actuelles)
void traj_avancer      (Trajectory *t, float m);
void traj_reculer      (Trajectory *t, float m);
void traj_gauche       (Trajectory *t, float m);
void traj_droite       (Trajectory *t, float m);
void traj_diag_droite  (Trajectory *t, float m);
void traj_diag_gauche  (Trajectory *t, float m);
void traj_rotation_gauche(Trajectory *t, float deg);
void traj_rotation_droite(Trajectory *t, float deg);

// Exécution continue
void traj_execute(Trajectory *t);

#endif
