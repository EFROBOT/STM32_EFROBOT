#include "pince.h"
#include "motor.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  GPIO_TypeDef *step_port;
  uint16_t step_pin;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef *en_port;
  uint16_t en_pin;
  uint8_t enabled, busy, direction_cw;
  uint32_t total_steps, steps_done;
  float v_min_sps, v_max_sps, accel_sps2;
  float current_sps, tick_accum;
  uint32_t phase_accel_end, phase_decel_start;
  uint8_t step_high_ticks;
} StepperMotor_t;

typedef struct {
  GripperJawState_t jaw_state;
  GripperPosition_t position;
  uint8_t has_object;
  uint8_t m2_lock_with_object;
  float m1_angle_deg;
} GripperState_t;

typedef struct {
  uint8_t active;
  GripperJawState_t target;
  uint32_t release_tick;
} JawCommand_t;

#define SCHEDULER_TICK_US 50.0f
#define STEP_PULSE_HIGH_TICKS 1U
#define MOTOR_FULL_STEPS_PER_REV 200.0f
#define DRIVER_MICROSTEPS 16.0f
#define STEPS_PER_REV (MOTOR_FULL_STEPS_PER_REV * DRIVER_MICROSTEPS)
static float moteur1_rpm = 150.0f;
static float moteur2_rpm = 75.0f;
static float moteur_min_rpm = 4.0f;
static float moteur_accel_rpm_s = 225.0f;
static float avance_distance_m = 0.065f;
#define ACTION_TIMEOUT_MS 20000U
#define JAW_FULL_OPEN_MS 750U
#define JAW_FULL_CLOSE_MS 750U
#define JAW_HALF_OPEN_MS 400U
#define JAW_HALF_CLOSE_MS 400U
#define GROUND_SETTLE_DELAY_MS 50U

static StepperMotor_t motors[2];
static GripperState_t gripper;
static JawCommand_t jaw_cmd;
static const float M1_GEAR_RATIO = 3.0f;
static const float M2_GEAR_RATIO = 1.6666667f;
extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;

static void Gripper_UpdateHoldPolicy(void);
static void Jaw_Update(void);

static void Watchdog_RefreshSafe(void) { (void)HAL_IWDG_Refresh(&hiwdg); }
static void DelayWithWatchdog(uint32_t delay_ms) {
  while (delay_ms-- > 0U) {
    Jaw_Update();
    Watchdog_RefreshSafe();
    HAL_Delay(1U);
  }
}

static void Debug_Log(const char *msg) {
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 50U);
}


static void Pince_NotifierFinMouvement(uint8_t succes) {
  const char *msg_ok = "Mouv Pince Ok\r\n";
  const char *msg_err = "Mouv Pince Err\r\n";
  const char *msg = succes ? msg_ok : msg_err;
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 50U);
}
static void Debug_Logf(const char *prefix, float value) {
  char buf[96];
  int len = snprintf(buf, sizeof(buf), "%s%.2f\r\n", prefix, (double)value);
  if (len > 0) {
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 50U);
  }
}

static void Motor_SetEnable(uint8_t motor_id, uint8_t enable) {
  if (motor_id > 1U)
    return;
  motors[motor_id].enabled = enable ? 1U : 0U;
  HAL_GPIO_WritePin(motors[motor_id].en_port, motors[motor_id].en_pin,
                    enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static float Motor_RpmToSps(float rpm) { return (rpm * STEPS_PER_REV) / 60.0f; }
static void Motor_StartMoveAngle(uint8_t motor_id, float angle_deg,
                                 uint8_t cw) {
  if (motor_id > 1U)
    return;
  StepperMotor_t *m = &motors[motor_id];
  uint32_t steps =
      (uint32_t)lroundf(fabsf(angle_deg) * (STEPS_PER_REV / 360.0f));
  if (!steps)
    return;
  m->direction_cw = cw ? 1U : 0U;
  HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                    m->direction_cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
  Gripper_UpdateHoldPolicy();
  Motor_SetEnable(motor_id, 1U);
  m->total_steps = steps;
  m->steps_done = 0U;
  m->current_sps = m->v_min_sps;
  m->tick_accum = 0.0f;
  m->step_high_ticks = 0U;
  uint32_t accel_steps = (uint32_t)(((m->v_max_sps * m->v_max_sps) -
                                     (m->v_min_sps * m->v_min_sps)) /
                                    (2.0f * m->accel_sps2));
  if ((2U * accel_steps) >= steps) {
    accel_steps = steps / 2U;
  }
  m->phase_accel_end = accel_steps;
  m->phase_decel_start = steps - accel_steps;
  m->busy = 1U;
}
static void Motor_Update(StepperMotor_t *m) {
  if (!m->busy)
    return;
  if (m->step_high_ticks > 0U) {
    m->step_high_ticks--;
    if (!m->step_high_ticks)
      HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
  }
  if (m->steps_done < m->phase_accel_end) {
    m->current_sps += m->accel_sps2 * (SCHEDULER_TICK_US * 1e-6f);
    if (m->current_sps > m->v_max_sps) {
      m->current_sps = m->v_max_sps;
    }
  } else if (m->steps_done >= m->phase_decel_start) {
    if (m->current_sps > m->v_min_sps) {
      m->current_sps -= m->accel_sps2 * (SCHEDULER_TICK_US * 1e-6f);
      if (m->current_sps < m->v_min_sps) {
        m->current_sps = m->v_min_sps;
      }
    }
  }
  m->tick_accum += m->current_sps * (SCHEDULER_TICK_US * 1e-6f);
  while (m->tick_accum >= 1.0f && m->steps_done < m->total_steps) {
    m->tick_accum -= 1.0f;
    HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_SET);
    m->step_high_ticks = STEP_PULSE_HIGH_TICKS;
    m->steps_done++;
  }
  if (m->steps_done >= m->total_steps) {
    m->busy = 0U;
    Gripper_UpdateHoldPolicy();
  }
}
static void Gripper_UpdateHoldPolicy(void) {
  uint8_t m1_should_hold = (gripper.position != POS_GROUND) ? 1U : 0U;
  uint8_t m2_should_hold =
      (gripper.has_object && gripper.m2_lock_with_object) ? 1U : 0U;
  if (!motors[0].busy)
    Motor_SetEnable(0U, m1_should_hold);
  if (!motors[1].busy)
    Motor_SetEnable(1U, m2_should_hold);
}
static uint8_t Motor_WaitIdle(uint8_t motor_id, uint32_t timeout_ms) {
  uint32_t t0 = HAL_GetTick();
  if (motor_id > 1U)
    return 0U;
  while (motors[motor_id].busy) {
    Jaw_Update();
    Watchdog_RefreshSafe();
    if ((HAL_GetTick() - t0) > timeout_ms) {
      return 0U;
    }
    HAL_Delay(1U);
  }
  return 1U;
}
static void Jaw_StopOutputs(void) {
  HAL_GPIO_WritePin(VERRIN_0_GPIO_Port, VERRIN_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VERRIN_1_GPIO_Port, VERRIN_1_Pin, GPIO_PIN_RESET);
}
static uint32_t Jaw_DurationMs(GripperJawState_t target) {
  return (target == GRIPPER_OPEN)
             ? JAW_FULL_OPEN_MS
             : (target == GRIPPER_CLOSED)
                   ? JAW_FULL_CLOSE_MS
                   : (target == GRIPPER_HALF_OPEN ? JAW_HALF_OPEN_MS
                                                  : JAW_HALF_CLOSE_MS);
}
static void Jaw_ApplyOutputs(GripperJawState_t target) {
  switch (target) {
  case GRIPPER_OPEN:
  case GRIPPER_HALF_OPEN:
    HAL_GPIO_WritePin(VERRIN_0_GPIO_Port, VERRIN_0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(VERRIN_1_GPIO_Port, VERRIN_1_Pin, GPIO_PIN_RESET);
    break;
  case GRIPPER_CLOSED:
  case GRIPPER_HALF_CLOSED:
    HAL_GPIO_WritePin(VERRIN_0_GPIO_Port, VERRIN_0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VERRIN_1_GPIO_Port, VERRIN_1_Pin, GPIO_PIN_SET);
    break;
  }
}
static void Jaw_StartAsync(GripperJawState_t target) {
  Jaw_ApplyOutputs(target);
  jaw_cmd.active = 1U;
  jaw_cmd.target = target;
  jaw_cmd.release_tick = HAL_GetTick() + Jaw_DurationMs(target);
}
static void Jaw_Update(void) {
  if (jaw_cmd.active && ((int32_t)(HAL_GetTick() - jaw_cmd.release_tick) >= 0)) {
    Jaw_StopOutputs();
    jaw_cmd.active = 0U;
    gripper.jaw_state = jaw_cmd.target;
    Gripper_UpdateHoldPolicy();
  }
}
static uint8_t Jaw_WaitDone(uint32_t timeout_ms) {
  uint32_t t0 = HAL_GetTick();
  while (jaw_cmd.active) {
    Jaw_Update();
    Watchdog_RefreshSafe();
    if ((HAL_GetTick() - t0) > timeout_ms) {
      return 0U;
    }
    HAL_Delay(1U);
  }
  return 1U;
}
static uint8_t Motor_MoveOutputAngle(uint8_t motor_id, float output_angle_deg,
                                     uint8_t cw) {
  float motor_angle = fabsf(output_angle_deg) *
                      (motor_id == 0U ? M1_GEAR_RATIO : M2_GEAR_RATIO);
  Motor_StartMoveAngle(motor_id, motor_angle, cw);
  return Motor_WaitIdle(motor_id, ACTION_TIMEOUT_MS);
}
static void Gripper_ActuateJaw(GripperJawState_t target) {
  if (gripper.jaw_state == target)
    return;
  Jaw_StartAsync(target);
  (void)Jaw_WaitDone(ACTION_TIMEOUT_MS);
}
static void Gripper_EnsureClosed(void) {
  if (gripper.jaw_state != GRIPPER_CLOSED) {
    Gripper_ActuateJaw(GRIPPER_CLOSED);
  }
}
static void Gripper_RecalibrateZeroAtGround(void) {
  if ((gripper.position == POS_GROUND) &&
      (gripper.jaw_state == GRIPPER_CLOSED)) {
    gripper.m1_angle_deg = 0.0f;
  }
}
static uint8_t Gripper_MoveTo(GripperPosition_t target) {
  if (target == gripper.position)
    return 1U;
  if ((target == POS_NAVIGATION) || (target == POS_UNLOAD)) {
    Gripper_EnsureClosed();
  }
  float target_angle = (target == POS_GROUND)
                           ? ANGLE_SOL_DEG
                           : (target == POS_NAVIGATION
                                  ? ANGLE_NAV_DEG
                                  : (target == POS_UNLOAD
                                         ? ANGLE_UNLOAD_DEG
                                         : (target == POS_UNLOAD_SAFE_CLOSE
                                                ? ANGLE_UNLOAD_SAFE_CLOSE_DEG
                                                : ANGLE_HOMOLOGATION_DEG)));
  float delta = target_angle - gripper.m1_angle_deg;
  if (fabsf(delta) > 0.01f) {
    if (!Motor_MoveOutputAngle(0U, fabsf(delta), delta >= 0.0f ? 1U : 0U))
      return 0U;
    gripper.m1_angle_deg = target_angle;
  }
  gripper.position = target;
  Gripper_UpdateHoldPolicy();
  return 1U;
}
static uint8_t Gripper_RetrieveAndGoUnload(uint8_t do_rotate) {
  float descend_start = gripper.m1_angle_deg;
  float descend_delta = ANGLE_SOL_DEG - descend_start;
  Debug_Log("[PINCE] start retrieve cycle\r\n");
  Debug_Logf("[PINCE] descend_start_deg=", descend_start);
  Debug_Logf("[PINCE] descend_delta_deg=", descend_delta);
  if (fabsf(descend_delta) > 0.01f) {
    Motor_StartMoveAngle(0U, fabsf(descend_delta) * M1_GEAR_RATIO,
                         descend_delta >= 0.0f ? 1U : 0U);
    uint8_t opened = 0U;
    uint32_t t0 = HAL_GetTick();
    while (motors[0].busy) {
      Jaw_Update();
      float progress = motors[0].total_steps
                           ? ((float)motors[0].steps_done / (float)motors[0].total_steps)
                           : 1.0f;
      float current_angle = descend_start + descend_delta * progress;
      if (!opened && current_angle < (ANGLE_NAV_DEG - 20) && gripper.jaw_state != GRIPPER_OPEN &&
          !jaw_cmd.active) {
        Jaw_StartAsync(GRIPPER_OPEN);
        Debug_Logf("[PINCE] jaw open started at m1_deg=", current_angle);
        opened = 1U;
      }
      if ((HAL_GetTick() - t0) > ACTION_TIMEOUT_MS)
        return 0U;
      Watchdog_RefreshSafe();
      HAL_Delay(1U);
    }
    gripper.m1_angle_deg = ANGLE_SOL_DEG;
  }
  gripper.position = POS_GROUND;
  if (jaw_cmd.active && !Jaw_WaitDone(ACTION_TIMEOUT_MS))
    return 0U;
  if (gripper.jaw_state != GRIPPER_OPEN)
    Gripper_ActuateJaw(GRIPPER_OPEN);
  Debug_Log("[PINCE] m1 reached ground, disable m1\r\n");
  Motor_SetEnable(0U, 0U);
  DelayWithWatchdog(GROUND_SETTLE_DELAY_MS);
  Debug_Log("[PINCE] settle done, closing jaw\r\n");
  Gripper_ActuateJaw(GRIPPER_CLOSED);
  Gripper_RecalibrateZeroAtGround();
  Debug_Log("[PINCE] recalibrated at ground\r\n");
  gripper.has_object = 1U;
  gripper.m2_lock_with_object = 1U;
  Gripper_UpdateHoldPolicy();
  float target_angle = ANGLE_UNLOAD_DEG;
  float rise_delta = target_angle - gripper.m1_angle_deg;
  Debug_Logf("[PINCE] rise_target_deg=", target_angle);
  if (fabsf(rise_delta) > 0.01f) {
    Motor_StartMoveAngle(0U, fabsf(rise_delta) * M1_GEAR_RATIO,
                         rise_delta >= 0.0f ? 1U : 0U);
    uint8_t rotation_started = 0U;
    uint32_t t0 = HAL_GetTick();
    while (motors[0].busy) {
      Jaw_Update();
      float progress = motors[0].total_steps
                           ? ((float)motors[0].steps_done / (float)motors[0].total_steps)
                           : 1.0f;
      float current_angle = gripper.m1_angle_deg + rise_delta * progress;
      if (!rotation_started && current_angle > ANGLE_ROTATION_DEG) {
        Motor_StartMoveAngle(1U, (do_rotate ? M2_ROTATION_DEG : M2_ALIGNEMENT_DEG) *
                                     M2_GEAR_RATIO,
                             do_rotate ? 1U : 0U);
        Debug_Logf("[PINCE] m2 rotation started at m1_deg=", current_angle);
        rotation_started = 1U;
      }
      if (current_angle > ANGLE_NAV_DEG) {
        gripper.m2_lock_with_object = 0U;
        Gripper_UpdateHoldPolicy();
      }
      if ((HAL_GetTick() - t0) > ACTION_TIMEOUT_MS)
        return 0U;
      Watchdog_RefreshSafe();
      HAL_Delay(1U);
    }
    gripper.m1_angle_deg = target_angle;
  }
  if (!Motor_WaitIdle(1U, ACTION_TIMEOUT_MS))
    return 0U;
  Debug_Log("[PINCE] m1+m2 done, unload sequence\r\n");
  gripper.position = POS_UNLOAD;
  gripper.m2_lock_with_object = 0U;
  Gripper_UpdateHoldPolicy();
  Gripper_ActuateJaw(GRIPPER_HALF_OPEN);
  if (!Motor_MoveOutputAngle(0U, fabsf(ANGLE_UNLOAD_SAFE_CLOSE_DEG - gripper.m1_angle_deg),
                             ANGLE_UNLOAD_SAFE_CLOSE_DEG >= gripper.m1_angle_deg ? 1U : 0U))
    return 0U;
  gripper.m1_angle_deg = ANGLE_UNLOAD_SAFE_CLOSE_DEG;
  gripper.position = POS_UNLOAD_SAFE_CLOSE;
  Gripper_UpdateHoldPolicy();
  Gripper_ActuateJaw(GRIPPER_HALF_CLOSED);
  gripper.has_object = 0U;
  gripper.m2_lock_with_object = 0U;
  Gripper_UpdateHoldPolicy();
  return 1U;
}
uint8_t Pince_RecupererEtStocker(uint8_t rotation_active) {
  uint8_t succes = Gripper_RetrieveAndGoUnload(rotation_active ? 1U : 0U);
  return succes;
}



uint8_t Pince_GoToNav(void) {
  return Gripper_MoveTo(POS_NAVIGATION);
}

uint8_t Pince_Pos_Homolog(void) {
  if (!Motor_MoveOutputAngle(0U, fabsf(ANGLE_HOMOLOGATION_DEG - gripper.m1_angle_deg),
                             ANGLE_HOMOLOGATION_DEG >= gripper.m1_angle_deg ? 1U : 0U))
    return 0U;
  Gripper_ActuateJaw(GRIPPER_OPEN);
  gripper.m1_angle_deg = ANGLE_HOMOLOGATION_DEG;
  gripper.position = POS_ROTATE_INTERMEDIATE;
  gripper.has_object = 0U;
  gripper.m2_lock_with_object = 0U;
  Gripper_UpdateHoldPolicy();
  return 1U;
}

void Pince_UpdateSchedulerTick(void) {
  Motor_Update(&motors[0]);
  Motor_Update(&motors[1]);
}

void Pince_Init(void) {
  motors[0] =
      (StepperMotor_t){M1_STEP_GPIO_Port, M1_STEP_Pin,     M1_DIR_GPIO_Port,
                       M1_DIR_Pin,        M1_EN_GPIO_Port, M1_EN_Pin};
  motors[1] =
      (StepperMotor_t){M2_STEP_GPIO_Port, M2_STEP_Pin,     M2_DIR_GPIO_Port,
                       M2_DIR_Pin,        M2_EN_GPIO_Port, M2_EN_Pin};
  for (uint8_t i = 0; i < 2; i++) {
    motors[i].v_min_sps = Motor_RpmToSps(moteur_min_rpm);
    motors[i].accel_sps2 = Motor_RpmToSps(moteur_accel_rpm_s);
    Motor_SetEnable(i, 0U);
    HAL_GPIO_WritePin(motors[i].step_port, motors[i].step_pin, GPIO_PIN_RESET);
  }
  motors[0].v_max_sps = Motor_RpmToSps(moteur1_rpm);
  motors[1].v_max_sps = Motor_RpmToSps(moteur2_rpm);
  gripper.position = POS_GROUND;
  gripper.jaw_state = GRIPPER_OPEN;
  gripper.has_object = 0U;
  gripper.m2_lock_with_object = 0U;
  gripper.m1_angle_deg = 0.0f;
  Gripper_ActuateJaw(GRIPPER_CLOSED);
  Gripper_UpdateHoldPolicy();
  Gripper_RecalibrateZeroAtGround();
  (void)Gripper_MoveTo(POS_NAVIGATION);
  gripper.position = POS_NAVIGATION;
  gripper.m1_angle_deg = ANGLE_NAV_DEG;
  gripper.has_object = 0U;
  gripper.m2_lock_with_object = 0U;
  Gripper_UpdateHoldPolicy();
}
