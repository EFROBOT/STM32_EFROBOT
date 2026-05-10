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
  POS_ROTATE_INTERMEDIATE
} GripperPosition_t;

typedef struct {
  GPIO_TypeDef *step_port;
  uint16_t step_pin;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef *en_port;
  uint16_t en_pin;
  uint8_t enabled, busy, direction_cw;
  uint32_t total_steps, steps_done, phase_accel_end, phase_decel_start;
  float v_min_sps, v_max_sps, accel_sps2, current_sps, tick_accum;
  uint8_t step_high_ticks;
} StepperMotor_t;

typedef struct {
  GripperJawState_t jaw_state;
  GripperPosition_t position;
  uint8_t has_object;
  uint8_t m2_lock_with_object;
  float m1_angle_deg;
} GripperState_t;


#define ANGLE_SOL_DEG           0.0f
#define ANGLE_NAV_DEG           90.0f
#define ANGLE_INTERMEDIATE_DEG  45.0f
#define ANGLE_UNLOAD_DEG        120.0f
#define M2_ROTATION_DEG         180.0f



#define SCHEDULER_TICK_US 50.0f
#define STEP_PULSE_HIGH_TICKS 1U
#define MOTOR_FULL_STEPS_PER_REV 200.0f
#define DRIVER_MICROSTEPS 16.0f
#define STEPS_PER_REV (MOTOR_FULL_STEPS_PER_REV * DRIVER_MICROSTEPS)
#define MIN_STEP_RATE_SPS 120.0f
#define MAX_STEP_RATE_SPS 1200.0f
#define DEFAULT_ACCEL_SPS2 1800.0f
#define M1_MAX_STEP_RATE_SPS 267.0f
#define M1_ACCEL_SPS2 400.0f
#define ACTION_TIMEOUT_MS 20000U
#define JAW_FULL_OPEN_MS 750U
#define JAW_FULL_CLOSE_MS 750U
#define JAW_HALF_OPEN_MS 400U
#define JAW_HALF_CLOSE_MS 400U
#define ROUTINE_RELEASE_DELAY_MS 2000U

static StepperMotor_t motors[2];
static GripperState_t gripper;
static const float M1_GEAR_RATIO = 3.0f;
static const float M2_GEAR_RATIO = 1.6666667f;

void Pince_Init(void);
void Pince_UpdateSchedulerTick(void);
uint8_t Pince_RecupererEtStocker(uint8_t rotation_active);

extern IWDG_HandleTypeDef hiwdg;



#endif

