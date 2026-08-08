#ifndef IR_MODULE_H
#define IR_MODULE_H

#include <stdint.h>

#include "board.h"
#include "task2_config.h"
#include "ti_msp_dl_config.h"

/*
 * Front eight-channel CD4051 sensor.
 *
 * Vehicle-front connector order, left to right: OUT, AD0, AD1, AD2.
 * Address AD2:AD1:AD0 = 000..111 selects X1..X8.
 * X1 is the vehicle-left probe; X8 is the vehicle-right probe.
 */
#define IR_CHANNEL_COUNT              (8U)
#define IR_BLACK_ACTIVE_LEVEL         TASK2_FRONT_BLACK_ACTIVE_LEVEL
#define IR_SENSOR_ORDER_REVERSED      TASK2_FRONT_ORDER_REVERSED
#define IR_CHANNEL_SETTLE_US          TASK2_FRONT_CHANNEL_SETTLE_US
#define IR_CONTROL_PERIOD_MS          TASK2_LINE_PERIOD_MS

#define IR_LOST_SPEED_MM_S            TASK2_SEARCH_SPEED_MM_S
#define IR_LOST_TURN_MM_S             TASK2_SEARCH_TURN_MM_S

#define IR_WRITE_PIN(port, pin, state) do { \
    if ((state) != 0U) { \
        DL_GPIO_setPins((port), (pin)); \
    } else { \
        DL_GPIO_clearPins((port), (pin)); \
    } \
} while (0)

#define IR_AD0_WRITE(state) \
    IR_WRITE_PIN(IR_AD0_PORT, IR_AD0_PIN_9_PIN, (state))
#define IR_AD1_WRITE(state) \
    IR_WRITE_PIN(IR_AD1_PORT, IR_AD1_PIN_27_PIN, (state))
#define IR_AD2_WRITE(state) \
    IR_WRITE_PIN(IR_AD2_PORT, IR_AD2_PIN_24_PIN, (state))
#define IR_OUT_READ() \
    (DL_GPIO_readPins(IR_OUT_PORT, IR_OUT_PIN_12_PIN) ? 1U : 0U)

/* Cached sensor and controller diagnostics. */
extern uint8_t ir_raw_state;
extern uint8_t ir_black_mask;
extern uint8_t ir_line_valid;
extern uint8_t ir_front_cross_detected;
extern uint16_t ir_lost_sample_count;
extern float ir_line_raw_error_mm;
extern float ir_line_position;       /* filtered front error, mm */
extern float ir_line_derivative;     /* filtered derivative, mm/s */
extern float ir_line_turn_target;    /* infrared PD only, mm/s */
extern float ir_curve_feedforward;   /* known-track curvature term, mm/s */
extern float ir_total_turn_target;   /* PD plus curvature feedforward, mm/s */
extern float ir_line_turn_diff;      /* target after slew limit, mm/s */
extern float ir_yaw_rate_target_dps;
extern float ir_yaw_rate_measured_dps;
extern float ir_gyro_correction;     /* yaw-rate tracking correction, mm/s */
extern float ir_motor_steering_sign; /* +1 because MotorA is verified left wheel */

/* Existing public speed/turn names retained for other project modules. */
extern float base_speed_mm;
extern float BaseSpeed;
extern float ForwardLimit;
extern float turn_diff;
extern float Turn90Angle;
extern float TurnMaxAngle;
extern float TurnMidAngle;
extern float TurnMinAngle;

void IR_Module_Init(void);
void IR_Module_Read(void);
void IRDM_line_inspection(void);
void IR_SetDiagnosticMode(uint8_t enabled);
void IR_SetMotorSteeringSign(float sign);
void IR_SetSearchTurnMmS(float turn_mm_s);
void IR_SetCurveFeedforward(float turn_mm_s);
void IR_ApplyYawRateTracking(
    float yaw_rate_dps, uint8_t gyro_valid);
void IR_Module_Update(
    float yaw_rate_dps,
    uint8_t gyro_valid,
    float curve_feedforward_mm_s);

/* One complete scan; all consumers use the resulting cached frame. */
uint8_t IR_Scan(void);
uint8_t IR_ReadRawState(void);
uint8_t IR_ReadBlackMask(void);
uint8_t IR_GetBlackMask(void);
uint8_t IR_LineDetected(void);

uint8_t IR_CountBlack(uint8_t black_mask);
uint8_t IR_IsWideMarker(uint8_t black_mask);
uint8_t IR_IsFinishMarkerSignature(uint8_t black_mask);
void IR_LineReset(void);
void IR_LineReacquire(void);

#endif /* IR_MODULE_H */
