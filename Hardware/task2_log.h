#ifndef TASK2_LOG_H
#define TASK2_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Compact 20 Hz TASK2 diagnostic sample.
 * All speed/turn fields use mm/s, angles and yaw rates use 0.1 degree units.
 */
typedef struct
{
    uint16_t time_ms;
    uint16_t odometry_mm;
    int16_t line_error_mm_x10;
    int16_t line_turn_mm_s;
    int16_t final_turn_mm_s;
    int16_t base_speed_mm_s;
    int16_t left_ref_mm_s;
    int16_t right_ref_mm_s;
    int16_t left_meas_mm_s;
    int16_t right_meas_mm_s;
    int16_t left_pwm;
    int16_t right_pwm;
    int16_t yaw_deg_x10;
    int16_t yaw_rate_target_x10;
    int16_t yaw_rate_measured_x10;
    int16_t gyro_correction_mm_s;
    int16_t curve_feedforward_mm_s;
    uint16_t curve_blend_x1000;
    uint8_t state;
    uint8_t front_mask;
    uint8_t imu_status;
} Task2LogFrame;

void Task2Log_Init(void);
void Task2Log_Start(void);
uint8_t Task2Log_Capture(const Task2LogFrame *frame);
void Task2Log_CaptureFinal(const Task2LogFrame *frame);
void Task2Log_Freeze(uint32_t now_ms);
void Task2Log_UpdateReplay(uint32_t now_ms);
uint16_t Task2Log_Count(void);
uint8_t Task2Log_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK2_LOG_H */
