#ifndef PINCE_H
#define PINCE_H

#include "main.h"

// piston
typedef enum {
    Ferme = 0,
    Ouvert  = 1
} Piston;

void piston(Piston state);

//-------------------------------------------------------------
// Stepper

typedef enum {
    Stepper_lever_pince = 1,
	Stepper_rotation_bloc = 2
} Stepper;

typedef enum {
    Horraire  = 0,
    Anti_horraire = 1
} Stepper_dir;

void activer_stepper (Stepper x);
void desactiver_stepper (Stepper x);
void nombre_pas_stepper (Stepper x, Stepper_dir dir, uint32_t steps, uint32_t delay_us);

//-------------------------------------------------------------
//servo

void controle_angle_servo(TIM_HandleTypeDef *htim, uint32_t channel, float angle);

#endif /* PINCE_H */
