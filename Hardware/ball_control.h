#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BALL_AUTO_HOLD_FAULT_NONE = 0,
    BALL_AUTO_HOLD_FAULT_VISION_LOST,
    BALL_AUTO_HOLD_FAULT_STEPPER
} BallAutoHoldFault;

/* 初始化K230串口接收与X42S控制。 */
void BallControl_Init(void);

/* Configure boot behaviour; normal application startup passes false. */
void BallControl_SetBootHomeCalibration(bool calibrate_current_position);

/* Register the current X42S shaft angle as home while the task is stopped. */
bool BallControl_RegisterCurrentHome(void);

/* 非阻塞更新函数，必须在主循环中持续调用。 */
void BallControl_Update(void);

/*
 * 手动设置滚球目标，单位mm。
 * 启用题目第3项自动流程后，此接口会返回false。
 */
bool BallControl_SetTargetMm(int16_t target_mm);

/* 获取当前滚球目标，单位mm。 */
int16_t BallControl_GetTargetMm(void);

/* 启用或禁止本模块刷新OLED。 */
void BallControl_SetDisplayEnabled(bool enabled);

/*
 * 启用题目第3项自动流程：
 *
 * 中心O -> +50mm -> -50mm并稳定。
 */
void BallControl_SetTask3Enabled(bool enabled);

/*
 * Enable the combined line-following + ball-hold mode.
 * Every STOP-to-RUN edge records the current valid vision position as O,
 * then the normal position loop holds 0 mm relative to that reading.
 */
void BallControl_SetAutoZeroHoldEnabled(bool enabled);

/* TASK4: true only after a fresh O-point frame and X42S enable are ready. */
bool BallControl_IsAutoZeroHoldReady(void);

/* Short vision gaps are allowed; a stale frame or X42S fault is unsafe. */
bool BallControl_IsAutoZeroHoldHealthy(void);

/* TASK4 transient/fault diagnostics used by the chassis state machine. */
bool BallControl_IsAutoZeroHoldVisionHeld(void);
BallAutoHoldFault BallControl_GetAutoZeroHoldFault(void);

/* 题目第3项是否已经完成。 */
bool BallControl_IsTask3Finished(void);

/* 获取题目第3项已用时间，单位ms。 */
uint32_t BallControl_GetTask3ElapsedMs(void);

/* 保留给摆杆角度标定模式使用。 */
void BallControl_SetRodCalibrationEnabled(bool enabled);

/*
 * 当任务3已经通过初始中心容差判断后返回true。
 *
 * 初始中心不要求视觉结果严格等于0。
 */
bool BallControl_IsStartReady(void);

/*
 * 获取当前目标误差：
 *
 * target_mm - measured_position_mm
 */
int16_t BallControl_GetPositionErrorMm(void);

/*
 * 获取最后收到的X42S回复状态。
 * 返回0表示尚未收到回复。
 */
uint8_t BallControl_GetLastStepperReply(void);

#ifdef __cplusplus
}
#endif

#endif /* BALL_CONTROL_H */
