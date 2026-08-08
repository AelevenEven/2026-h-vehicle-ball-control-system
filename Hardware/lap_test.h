#ifndef LAP_TEST_H
#define LAP_TEST_H

#include <stdint.h>

#include "task2_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TASK2_IDLE = 0,
    TASK2_READY,
    TASK2_STARTING,
    TASK2_RUNNING,
    TASK2_PASS_B,
    TASK2_APPROACH_STOP,
    TASK2_FINAL_CREEP,
    TASK2_BRAKING,
    TASK2_STOPPED,
    TASK2_LOST_LINE,
    TASK2_FAULT
} Task2State;

typedef enum
{
    TASK2_FAULT_NONE = 0,
    TASK2_FAULT_ABORTED,
    TASK2_FAULT_START_LINE,
    TASK2_FAULT_LINE_TIMEOUT,
    TASK2_FAULT_RUN_TIMEOUT,
    TASK2_FAULT_ENCODER_STALL,
    TASK2_FAULT_WHEEL_MISMATCH,
    TASK2_FAULT_CREEP_TIMEOUT,
    TASK2_FAULT_BALL_LOST,
    TASK2_FAULT_X42S
} Task2Fault;

/* Compatibility type retained for the existing application entry. */
typedef Task2State LapTestState;

#define LAP_DEBUG_PRINT_ENABLE       TASK2_DEBUG_CSV_ENABLE
#define H2_STEERING_DIAG_MODE        (0U)
#define H2_MPU6050_ENABLE            TASK2_MPU6050_ENABLE

void LapTest_Init(void);
void LapTest_Update(void);
void LapTest_SetTask4Mode(uint8_t enabled);
void LapTest_SetExternalStartReady(uint8_t ready);
void LapTest_SetTask4BallErrorMm(int16_t error_mm);
void LapTest_SetTask4BallVisionHold(uint8_t held);
void LapTest_SetTask4ExternalFault(uint8_t fault);
LapTestState LapTest_GetState(void);
uint32_t LapTest_GetElapsedMs(void);
Task2Fault LapTest_GetFault(void);

#ifdef __cplusplus
}
#endif

#endif /* LAP_TEST_H */
