#include "task2_log.h"

#include <stddef.h>
#include <stdio.h>

#include "task2_config.h"

typedef enum
{
    TASK2_LOG_REPLAY_IDLE = 0,
    TASK2_LOG_REPLAY_BEGIN,
    TASK2_LOG_REPLAY_HEADER,
    TASK2_LOG_REPLAY_ROWS,
    TASK2_LOG_REPLAY_END,
    TASK2_LOG_REPLAY_WAIT
} Task2LogReplayState;

static Task2LogFrame s_frames[TASK2_BLACKBOX_MAX_SAMPLES];
static uint16_t s_frame_count;
static uint16_t s_replay_index;
static uint8_t s_frozen;
static uint8_t s_ready;
static Task2LogReplayState s_replay_state;
static uint32_t s_next_replay_ms;

void Task2Log_Init(void)
{
    s_frame_count = 0U;
    s_replay_index = 0U;
    s_frozen = 0U;
    s_ready = 0U;
    s_replay_state = TASK2_LOG_REPLAY_IDLE;
    s_next_replay_ms = 0U;
}

void Task2Log_Start(void)
{
    s_frame_count = 0U;
    s_replay_index = 0U;
    s_frozen = 0U;
    s_ready = 0U;
    s_replay_state = TASK2_LOG_REPLAY_IDLE;
    s_next_replay_ms = 0U;
}

uint8_t Task2Log_Capture(const Task2LogFrame *frame)
{
    if ((frame == NULL) ||
        (s_frozen != 0U) ||
        (s_frame_count >= TASK2_BLACKBOX_MAX_SAMPLES)) {
        return 0U;
    }

    s_frames[s_frame_count] = *frame;
    s_frame_count++;
    return 1U;
}

void Task2Log_CaptureFinal(const Task2LogFrame *frame)
{
    if ((frame == NULL) || (s_frozen != 0U)) {
        return;
    }
    if (s_frame_count < TASK2_BLACKBOX_MAX_SAMPLES) {
        s_frames[s_frame_count] = *frame;
        s_frame_count++;
    } else if (TASK2_BLACKBOX_MAX_SAMPLES != 0U) {
        s_frames[TASK2_BLACKBOX_MAX_SAMPLES - 1U] = *frame;
    }
}

void Task2Log_Freeze(uint32_t now_ms)
{
    if (s_frozen != 0U) {
        return;
    }

    s_frozen = 1U;
    s_ready = (s_frame_count != 0U) ? 1U : 0U;
    s_replay_index = 0U;
    s_replay_state =
        (s_ready != 0U) ?
            TASK2_LOG_REPLAY_BEGIN : TASK2_LOG_REPLAY_IDLE;
    s_next_replay_ms = now_ms + TASK2_BLACKBOX_REPLAY_DELAY_MS;
}

void Task2Log_UpdateReplay(uint32_t now_ms)
{
    const Task2LogFrame *frame;

    if ((s_ready == 0U) ||
        ((int32_t)(now_ms - s_next_replay_ms) < 0)) {
        return;
    }

    switch (s_replay_state) {
    case TASK2_LOG_REPLAY_BEGIN:
        printf(
            "#TASK2_LOG_BEGIN,%u,%u\r\n",
            (unsigned int)s_frame_count,
            (unsigned int)sizeof(Task2LogFrame));
        s_replay_state = TASK2_LOG_REPLAY_HEADER;
        s_next_replay_ms =
            now_ms + TASK2_BLACKBOX_REPLAY_ROW_MS;
        break;

    case TASK2_LOG_REPLAY_HEADER:
        printf(
            "#time_ms,state,front_bits,front_error_mm_x10,"
            "line_turn,turn_output,base_speed,left_ref,right_ref,"
            "left_meas,right_meas,left_pwm,right_pwm,odometry_mm,"
            "yaw_deg_x10,yaw_rate_target_x10,yaw_rate_measured_x10,"
            "gyro_correction,curve_feedforward,curve_blend_x1000,"
            "imu_status\r\n");
        s_replay_state = TASK2_LOG_REPLAY_ROWS;
        s_replay_index = 0U;
        s_next_replay_ms =
            now_ms + TASK2_BLACKBOX_REPLAY_ROW_MS;
        break;

    case TASK2_LOG_REPLAY_ROWS:
        frame = &s_frames[s_replay_index];
        printf(
            "%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%d,%d,%d,%d,%d,%u,%u\r\n",
            (unsigned int)frame->time_ms,
            (unsigned int)frame->state,
            (unsigned int)frame->front_mask,
            (int)frame->line_error_mm_x10,
            (int)frame->line_turn_mm_s,
            (int)frame->final_turn_mm_s,
            (int)frame->base_speed_mm_s,
            (int)frame->left_ref_mm_s,
            (int)frame->right_ref_mm_s,
            (int)frame->left_meas_mm_s,
            (int)frame->right_meas_mm_s,
            (int)frame->left_pwm,
            (int)frame->right_pwm,
            (unsigned int)frame->odometry_mm,
            (int)frame->yaw_deg_x10,
            (int)frame->yaw_rate_target_x10,
            (int)frame->yaw_rate_measured_x10,
            (int)frame->gyro_correction_mm_s,
            (int)frame->curve_feedforward_mm_s,
            (unsigned int)frame->curve_blend_x1000,
            (unsigned int)frame->imu_status);
        s_replay_index++;
        if (s_replay_index >= s_frame_count) {
            s_replay_state = TASK2_LOG_REPLAY_END;
        }
        s_next_replay_ms =
            now_ms + TASK2_BLACKBOX_REPLAY_ROW_MS;
        break;

    case TASK2_LOG_REPLAY_END:
        printf(
            "#TASK2_LOG_END,%u\r\n",
            (unsigned int)s_frame_count);
        s_replay_state = TASK2_LOG_REPLAY_WAIT;
        s_next_replay_ms =
            now_ms + TASK2_BLACKBOX_REPLAY_REPEAT_MS;
        break;

    case TASK2_LOG_REPLAY_WAIT:
        s_replay_state = TASK2_LOG_REPLAY_BEGIN;
        s_next_replay_ms = now_ms;
        break;

    default:
        s_replay_state = TASK2_LOG_REPLAY_IDLE;
        break;
    }
}

uint16_t Task2Log_Count(void)
{
    return s_frame_count;
}

uint8_t Task2Log_IsReady(void)
{
    return s_ready;
}
