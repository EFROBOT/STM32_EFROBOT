#ifndef PINCE_H
#define PINCE_H

#include "main.h"

typedef enum {
  GRIPPER_OPEN = 0,
  GRIPPER_CLOSED,
  GRIPPER_HALF_OPEN,
  GRIPPER_HALF_CLOSED
} GripperJawState_t;

typedef enum {
  POS_GROUND = 0,
  POS_NAVIGATION,
  POS_UNLOAD,
  POS_UNLOAD_SAFE_CLOSE,
  POS_ROTATE_INTERMEDIATE
} GripperPosition_t;

#define ANGLE_SOL_DEG           0.0f
#define ANGLE_NAV_DEG           136.0f
#define ANGLE_ROTATION_DEG      0.0f
#define ANGLE_HOMOLOGATION_DEG  45.0f
#define ANGLE_UNLOAD_DEG        170.0f
#define ANGLE_UNLOAD_SAFE_CLOSE_DEG 150.0f

#define M2_ROTATION_DEG         135.0f
#define M2_ALIGNEMENT_DEG       30.0f

void Pince_Init(void);
void Pince_UpdateSchedulerTick(void);
uint8_t Pince_RecupererEtStocker(uint8_t rotation_active);
uint8_t Pince_GoToNav(void);
uint8_t Pince_Pos_Homolog(void);

#endif

