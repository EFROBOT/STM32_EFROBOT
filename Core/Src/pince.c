#include "pince.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

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
#define JAW_FULL_OPEN_MS 1000U
#define JAW_FULL_CLOSE_MS 1250U
#define JAW_HALF_OPEN_MS 375U
#define JAW_HALF_CLOSE_MS 500U
#define JAW_REGRIP_DELAY_MS 1000U
#define ROUTINE_RELEASE_DELAY_MS 2000U

static StepperMotor_t motors[2];
static GripperState_t gripper;
static char uart_tx_buf[96];
static const float M1_GEAR_RATIO = 3.0f;
static const float M2_GEAR_RATIO = 1.6666667f;

static void Gripper_UpdateHoldPolicy(void);

static void Uart_Send(const char *msg) {
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg),
                    HAL_MAX_DELAY);
}
static void Motor_SetEnable(uint8_t motor_id, uint8_t enable) {
  if (motor_id > 1U)
    return;
  motors[motor_id].enabled = enable ? 1U : 0U;
  HAL_GPIO_WritePin(motors[motor_id].en_port, motors[motor_id].en_pin,
                    enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
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
  if ((2U * accel_steps) >= steps)
    accel_steps = steps / 2U;
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
    if (m->current_sps > m->v_max_sps)
      m->current_sps = m->v_max_sps;
  } else if (m->steps_done >= m->phase_decel_start) {
    if (m->current_sps > m->v_min_sps) {
      m->current_sps -= m->accel_sps2 * (SCHEDULER_TICK_US * 1e-6f);
      if (m->current_sps < m->v_min_sps)
        m->current_sps = m->v_min_sps;
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
    if ((HAL_GetTick() - t0) > timeout_ms) {
      (void)snprintf(uart_tx_buf, sizeof(uart_tx_buf), "TIMEOUT M%u\r\n",
                     (unsigned)(motor_id + 1U));
      Uart_Send(uart_tx_buf);
      return 0U;
    }
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
  switch (target) {
  case GRIPPER_OPEN:
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_Delay(JAW_FULL_OPEN_MS);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    break;
  case GRIPPER_CLOSED:
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_Delay(JAW_FULL_CLOSE_MS);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    break;
  case GRIPPER_HALF_OPEN:
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_Delay(JAW_HALF_OPEN_MS);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    break;
  case GRIPPER_HALF_CLOSED:
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_Delay(JAW_HALF_CLOSE_MS);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    break;
  }
  gripper.jaw_state = target;
  Gripper_UpdateHoldPolicy();
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
    Uart_Send("M1 zero recalibrated\r\n");
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
                                  : (target == POS_UNLOAD ? ANGLE_UNLOAD_DEG
                                                          : ANGLE_INTERMEDIATE_DEG));
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
static uint8_t Gripper_MoveToUnloadWithM2Release(void) {
  if ((gripper.m1_angle_deg < ANGLE_NAV_DEG) && !Gripper_MoveTo(POS_NAVIGATION))
    return 0U;
  gripper.m2_lock_with_object = 0U;
  Gripper_UpdateHoldPolicy();
  return Gripper_MoveTo(POS_UNLOAD);
}
static uint8_t Gripper_RetrieveAndGoUnload(uint8_t do_rotate) {
  if (!Gripper_MoveTo(POS_ROTATE_INTERMEDIATE))
    return 0U;
  if (gripper.jaw_state != GRIPPER_OPEN)
    Gripper_ActuateJaw(GRIPPER_OPEN);
  if (!Gripper_MoveTo(POS_GROUND))
    return 0U;
  Motor_SetEnable(0U, 0U);
  Gripper_ActuateJaw(GRIPPER_CLOSED);
  Gripper_RecalibrateZeroAtGround();
  gripper.has_object = 1U;
  gripper.m2_lock_with_object = 1U;
  Gripper_UpdateHoldPolicy();
  if (do_rotate) {
    if (!Gripper_MoveTo(POS_ROTATE_INTERMEDIATE))
      return 0U;
    if (!Motor_MoveOutputAngle(1U, M2_ROTATION_DEG, 1U))
      return 0U;
  }
  if (!Gripper_MoveToUnloadWithM2Release())
    return 0U;
  Gripper_ActuateJaw(GRIPPER_HALF_OPEN);
  HAL_Delay(ROUTINE_RELEASE_DELAY_MS);
  Gripper_ActuateJaw(GRIPPER_HALF_CLOSED);
  gripper.has_object = 0U;
  gripper.m2_lock_with_object = 0U;
  Gripper_UpdateHoldPolicy();
  return 1U;
}
static uint8_t Gripper_Unload(void) {
  if (!Gripper_MoveTo(POS_UNLOAD))
    return 0U;
  Gripper_ActuateJaw(GRIPPER_HALF_OPEN);
  HAL_Delay(JAW_REGRIP_DELAY_MS);
  Gripper_ActuateJaw(GRIPPER_HALF_CLOSED);
  Gripper_ActuateJaw(GRIPPER_CLOSED);
  gripper.has_object = 0U;
  gripper.m2_lock_with_object = 0U;
  Gripper_UpdateHoldPolicy();
  return 1U;
}

void Pince_PrintStatus(void) {
  const char *pos = (gripper.position == POS_GROUND)
                        ? "SOL"
                        : (gripper.position == POS_NAVIGATION ? "NAV" : "DECH");
  const char *jaw =
      (gripper.jaw_state == GRIPPER_OPEN)
          ? "OPEN"
          : (gripper.jaw_state == GRIPPER_CLOSED
                 ? "CLOSED"
                 : (gripper.jaw_state == GRIPPER_HALF_OPEN ? "HALF_OPEN"
                                                           : "HALF_CLOSE"));
  (void)snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                 "STATE pos=%s jaw=%s obj=%u m1_en=%u m2_en=%u\r\n", pos, jaw,
                 (unsigned)gripper.has_object, (unsigned)motors[0].enabled,
                 (unsigned)motors[1].enabled);
  Uart_Send(uart_tx_buf);
}

void Pince_HandleCommand(const char *line) {
  char motor[4] = {0}, dir[4] = {0};
  float angle = 0.0f;
  if (!strcmp(line, "S") || !strcmp(line, "s")) {
    (void)Gripper_MoveTo(POS_GROUND);
    Pince_PrintStatus();
    return;
  }
  if (!strcmp(line, "N") || !strcmp(line, "n")) {
    (void)Gripper_MoveTo(POS_NAVIGATION);
    Pince_PrintStatus();
    return;
  }
  if (!strcmp(line, "D") || !strcmp(line, "d")) {
    (void)Gripper_MoveTo(POS_UNLOAD);
    Pince_PrintStatus();
    return;
  }
  if (!strcmp(line, "O") || !strcmp(line, "o")) {
    if (gripper.position == POS_GROUND) {
      Gripper_ActuateJaw(GRIPPER_OPEN);
      Gripper_UpdateHoldPolicy();
    } else
      Uart_Send("REFUSED: open only at ground\r\n");
    Pince_PrintStatus();
    return;
  }
  if (!strcmp(line, "F") || !strcmp(line, "f")) {
    Gripper_ActuateJaw(GRIPPER_CLOSED);
    if (gripper.position == POS_GROUND) {
      Gripper_RecalibrateZeroAtGround();
      Gripper_UpdateHoldPolicy();
    }
    Pince_PrintStatus();
    return;
  }
  if (!strcmp(line, "C") || !strcmp(line, "c")) {
    if (gripper.position == POS_UNLOAD) {
      (void)Gripper_Unload();
    } else
      Uart_Send("REFUSED: half-open only at unload\r\n");
    Pince_PrintStatus();
    return;
  }
  if (!strcmp(line, "R0") || !strcmp(line, "r0")) {
    (void)Gripper_RetrieveAndGoUnload(0U);
    Pince_PrintStatus();
    return;
  }
  if (!strcmp(line, "R1") || !strcmp(line, "r1")) {
    (void)Gripper_RetrieveAndGoUnload(1U);
    Pince_PrintStatus();
    return;
  }
  if (sscanf(line, "%3s %f %3s", motor, &angle, dir) == 3) {
    uint8_t mid = 0xFFU, cw = 1U;
    if (!strcmp(motor, "M1"))
      mid = 0U;
    else if (!strcmp(motor, "M2"))
      mid = 1U;
    if (!strcmp(dir, "CW") || !strcmp(dir, "cw"))
      cw = 1U;
    else if (!strcmp(dir, "CCW") || !strcmp(dir, "ccw"))
      cw = 0U;
    else
      mid = 0xFFU;
    if (mid <= 1U) {
      Motor_StartMoveAngle(mid, angle, cw);
      (void)Motor_WaitIdle(mid, ACTION_TIMEOUT_MS);
      return;
    }
  }
  Uart_Send("ERR\r\n");
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
    motors[i].v_min_sps = MIN_STEP_RATE_SPS;
    motors[i].v_max_sps = MAX_STEP_RATE_SPS;
    motors[i].accel_sps2 = DEFAULT_ACCEL_SPS2;
    Motor_SetEnable(i, 0U);
    HAL_GPIO_WritePin(motors[i].step_port, motors[i].step_pin, GPIO_PIN_RESET);
  }
  gripper.position = POS_GROUND;
  gripper.jaw_state = GRIPPER_OPEN;
  gripper.has_object = 0U;
  gripper.m2_lock_with_object = 0U;
  gripper.m1_angle_deg = 0.0f;
  Gripper_ActuateJaw(GRIPPER_CLOSED);
  Uart_Send("Fermeture de la pince \r\n");
  motors[0].v_max_sps = M1_MAX_STEP_RATE_SPS;
  motors[0].accel_sps2 = M1_ACCEL_SPS2;
  Gripper_UpdateHoldPolicy();
  Gripper_RecalibrateZeroAtGround();
  (void)Gripper_MoveTo(POS_NAVIGATION);
  gripper.position = POS_NAVIGATION;
  gripper.m1_angle_deg = ANGLE_NAV_DEG;
  gripper.has_object = 0U;
  gripper.m2_lock_with_object = 0U;
  Gripper_UpdateHoldPolicy();
}
