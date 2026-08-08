#include "ball_control.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef BALL_DEBUG_PRINT_ENABLE
#define BALL_DEBUG_PRINT_ENABLE                (0U)
#endif

#if BALL_DEBUG_PRINT_ENABLE == 1U
#include <stdio.h>
#endif

#include "board.h"
#include "control.h"
#include "k230_uart.h"
#include "k230_uart_stats.h"
#include "oled.h"
#include "stepper_emm.h"

/*
 * 改成-1可以反转升降机构控制方向。
 *
 * 球向正方向偏离时，如果水管倾斜方向错误，
 * 只修改这里，不要同时修改K230的位置正负。
 */
#define BALL_STEPPER_DIRECTION                 (-1)

/*
 * ============================================================
 * 滚球位置控制参数
 * ============================================================
 */

/* 每毫米位置误差对应的步进电机脉冲数。 */
#define BALL_POSITION_KP_PULSES_PER_MM         (1.00f)

/* 球速度阻尼。 */
#define BALL_VELOCITY_KD_PULSES_PER_MM_S       (0.22f)

#define BALL_OUTBOUND_KP_PULSES_PER_MM           (1.00f)
#define BALL_OUTBOUND_KD_PULSES_PER_MM_S         (0.65f)
#define BALL_OUTBOUND_KI_PULSES_PER_MM_S         (0.12f)
#define BALL_OUTBOUND_START_MIN_PULSES            (120)
#define BALL_OUTBOUND_START_ZONE_MM               (5)
#define BALL_OUTBOUND_LAUNCH_DELTA_PULSES          (120)
#define BALL_OUTBOUND_RELEASE_DELTA_PULSES          (60)
#define BALL_RETURN_KI_PULSES_PER_MM_S           (0.45f)
#define BALL_TRANSPORT_STALL_CONFIRM_MS         (100U)
#define BALL_OUTBOUND_INTEGRAL_LIMIT_PULSES      (10.0f)
#define BALL_RETURN_INTEGRAL_LIMIT_PULSES        (40.0f)

/* 进入端点捕获区后降低位置增益、增强速度阻尼。 */
#define BALL_CAPTURE_POSITION_TO_SPEED_GAIN    (1.00f)
#define BALL_CAPTURE_SPEED_LIMIT_MM_S          (45.0f)
#define BALL_CAPTURE_SPEED_KP_PULSES_PER_MM_S  (0.20f)
#define BALL_CAPTURE_KI_PULSES_PER_MM_S        (0.18f)
#define BALL_CAPTURE_ZONE_MM                   (20.0f)
#define BALL_RETURN_CAPTURE_ZONE_MM            (30.0f)
#define BALL_CAPTURE_INTEGRAL_LIMIT_PULSES     (4.0f)
#define BALL_CAPTURE_INTEGRAL_SPEED_MM_S       (25.0f)

/* 位置环死区，单位mm。 */
#define BALL_POSITION_DEADBAND_MM              (2.0f)
#define BALL_VELOCITY_DEADBAND_MM_S            (20.0f)

/*
 * 球速低通滤波系数。
 *
 * 越大响应越快；
 * 越小滤波越强。
 */
#define BALL_VELOCITY_FILTER_ALPHA             (0.50f)
#define BALL_VELOCITY_LIMIT_MM_S                (600.0f)
#define BALL_VELOCITY_MIN_INTERVAL_MS           (20U)
#define BALL_VELOCITY_MAX_INTERVAL_MS           (200U)

/*
 * ============================================================
 * 目标位置和步进电机限制
 * ============================================================
 */

/* 25cm水管允许的滚球目标范围。 */
#define BALL_TARGET_MIN_MM                     (-120)
#define BALL_TARGET_MAX_MM                     (120)

/* 升降机构允许的最大逻辑偏移脉冲。 */
#define BALL_STEPPER_TRANSPORT_LIMIT_PULSES     (90)
#define BALL_STEPPER_HOLD_LIMIT_PULSES          (40)
#define BALL_STEPPER_OUTBOUND_LIMIT_PULSES      (120)

/* 每次更新允许变化的最大脉冲。 */
#define BALL_STEPPER_NORMAL_DELTA_PULSES        (8)
#define BALL_STEPPER_BRAKE_DELTA_PULSES         (16)
#define BALL_RETURN_BRAKE_DELTA_PULSES          (8)
#define BALL_CAPTURE_NEAR_OVERSHOOT_MM          (10.0f)
#define BALL_CAPTURE_NEAR_DRIVE_LIMIT_PULSES    (6)
#define BALL_CAPTURE_FAR_DRIVE_LIMIT_PULSES     (10)

/* 回程运输允许的最大安全角度；实际角度仍由视觉PID计算。 */
#define BALL_STEPPER_RETURN_LIMIT_PULSES        (96)

/* 正常滚球控制速度。 */
#define BALL_STEPPER_SPEED_RPM                 (40U)

/* 视觉丢失后回水平位置的速度。 */
#define BALL_STEPPER_CENTER_SPEED_RPM          (20U)

/* X42S加速度参数。 */
#define BALL_STEPPER_ACCELERATION              (180U)

/*
 * TASK4 longitudinal acceleration feedforward.
 * The lift end points toward the car front: forward acceleration requires a
 * negative motor offset so gravity counters the ball's rearward inertia.
 * 3200/(2*pi*g) is approximately 0.052 pulse per (mm/s^2).
 */
#define BALL_HOLD_ACCEL_FF_SIGN                (-1.0f)
#define BALL_HOLD_ACCEL_FF_GAIN                (0.060f)
#define BALL_HOLD_ACCEL_FF_ALPHA               (0.30f)
#define BALL_HOLD_ACCEL_FF_LIMIT_MM_S2         (300.0f)
#define BALL_HOLD_ACCEL_FF_DEADBAND_MM_S2      (25.0f)
#define BALL_HOLD_ACCEL_FF_LIMIT_PULSES        (24)
#define BALL_HOLD_ACCEL_FF_MIN_INTERVAL_MS     (20U)

/*
 * TASK4 has its own centre-hold controller.  Do not reuse the heavily tuned
 * O -> +5 -> -5 controller: TASK4 needs small, critically damped corrections
 * around O plus an integral term for the real rod-level bias.
 */
#define BALL_HOLD_POSITION_KP_INNER            (1.40f)
#define BALL_HOLD_POSITION_KP_MIDDLE           (2.40f)
#define BALL_HOLD_POSITION_KP_OUTER            (3.20f)
#define BALL_HOLD_POSITION_KI_PULSES_PER_MM_S  (0.50f)
#define BALL_HOLD_VELOCITY_KD_INNER            (1.10f)
#define BALL_HOLD_VELOCITY_KD_MIDDLE           (0.90f)
#define BALL_HOLD_VELOCITY_KD_OUTER            (0.70f)
#define BALL_HOLD_INNER_ZONE_MM                (3.0f)
#define BALL_HOLD_MIDDLE_ZONE_MM               (8.0f)
#define BALL_HOLD_INNER_LIMIT_PULSES           (45)
#define BALL_HOLD_MIDDLE_LIMIT_PULSES          (80)
#define BALL_HOLD_INTEGRAL_LIMIT_PULSES        (24.0f)
#define BALL_HOLD_INTEGRAL_ZONE_MM             (20.0f)
#define BALL_HOLD_INTEGRAL_SPEED_MM_S          (60.0f)
#define BALL_HOLD_POSITION_DEADBAND_MM         (1.0f)
#define BALL_HOLD_VELOCITY_DEADBAND_MM_S       (6.0f)
#define BALL_HOLD_VELOCITY_FILTER_ALPHA        (0.35f)
#define BALL_HOLD_STATIC_DRIVE_ERROR_MM        (3.0f)
#define BALL_HOLD_STATIC_DRIVE_SPEED_MM_S      (18.0f)
#define BALL_HOLD_STATIC_DRIVE_PULSES          (32)
#define BALL_HOLD_MAX_OFFSET_PULSES            (120)
#define BALL_HOLD_MAX_DELTA_PULSES             (30)

/* MCU复位后等待X42S启动，再把当前电机轴角度定义为逻辑零点。 */
#define BALL_STEPPER_CLEAR_FUNCTION            (0x0AU)
#define BALL_STEPPER_BOOT_ZERO_DELAY_MS        (300U)
#define BALL_STEPPER_BOOT_COMMAND_GAP_MS       (80U)
#define BALL_STEPPER_BOOT_ZERO_RETRY_MS        (300U)
#define BALL_STEPPER_BOOT_ZERO_MAX_ATTEMPTS    (3U)
#define BALL_STEPPER_BOOT_RETURN_SETTLE_MS      (800U)

/*
 * ============================================================
 * K230数据有效性参数
 * ============================================================
 */

/* 最低识别置信度，千分数。 */
#define BALL_MIN_CONFIDENCE_PERMILLE           (400U)

/* 超过此时间未收到有效位置，认为数据过期。 */
#define BALL_MESSAGE_STALE_MS                  (200U)

/* 少量视觉漏帧期间保持当前电机角度，不用旧位置重复计算PD。 */
#define BALL_VISION_HOLD_MS                    (150U)
#define BALL_VISION_INVALID_FRAME_LIMIT        (4U)

/* 丢球超过此时间后，机构返回逻辑水平位置。 */
#define BALL_CENTER_AFTER_LOST_MS              (200U)

/* TASK4 tolerates brief K230 misses while slowing the chassis safely. */
#define BALL_TASK4_VISION_HOLD_MS               (250U)
#define BALL_TASK4_MESSAGE_STALE_MS             (700U)
#define BALL_TASK4_INVALID_FRAME_LIMIT          (14U)
#define BALL_TASK4_CENTER_AFTER_LOST_MS          (700U)

/* 步进控制更新周期。 */
#define BALL_STEPPER_UPDATE_MS                 (50U)

/* OLED刷新周期。 */
#define BALL_DISPLAY_UPDATE_MS                 (200U)

/* 给K230发送电机轴目标角度的周期。 */
#define BALL_ANGLE_SEND_PERIOD_MS              (50U)

/* 当前X42S细分设置：3200脉冲对应电机轴一圈。 */
#define BALL_STEPPER_PULSES_PER_REVOLUTION     (3200L)

/*
 * ============================================================
 * 题目第3项：O -> +5cm -> -5cm并稳定
 * ============================================================
 */

/* 三个目标位置，单位mm。 */
#define BALL_TASK3_CENTER_TARGET_MM            (0)
#define BALL_TASK3_POSITIVE_TARGET_MM          (50)
#define BALL_TASK3_NEGATIVE_TARGET_MM          (-50)
#define BALL_TASK3_NEGATIVE_FINISH_TARGET_MM   (-62)
#define BALL_TASK3_NEGATIVE_CONTROL_TARGET_MM  (-68.0f)
#define BALL_TASK3_PID_NEGATIVE_DRIVE_TARGET_MM (-84.0f)
#define BALL_TASK3_PID_NEGATIVE_HOLD_TARGET_MM  (-64.0f)
#define BALL_TASK3_POSITIVE_SWITCH_MM          (12)
#define BALL_TASK3_POSITIVE_PREDICT_MM         (24.0f)
#define BALL_TASK3_POSITIVE_PREDICT_MIN_MM     (3.0f)
#define BALL_TASK3_POSITIVE_LOOKAHEAD_S         (0.60f)
#define BALL_TASK3_POSITIVE_MAX_PREDICT_SPEED  (200.0f)
#define BALL_TASK3_POSITIVE_CONFIRM_FRAMES      (1U)

/*
 * 到达判定允许误差。
 *
 * 这里使用±5mm，不再要求K230必须精确显示：
 *
 * 0.0mm
 * +50.0mm
 * -50.0mm
 */
#define BALL_TASK3_CENTER_TOLERANCE_MM         (5)
#define BALL_TASK3_NEGATIVE_TOLERANCE_MM       (8)

/*
 * 到达目标时允许的最大球速。
 *
 * 中心和最终-5cm需要稳定，所以速度限制较小。
 * +5cm是折返点，可以允许更快。
 */
#define BALL_TASK3_CENTER_MAX_SPEED_MM_S       (25.0f)
#define BALL_TASK3_NEGATIVE_MAX_SPEED_MM_S     (35.0f)

/*
 * Mode 2 stopwatch freezes only after the ball is physically stopped inside
 * the scoring window (-5 cm +/-1 cm).  The confirmation interval rejects a
 * momentary zero-velocity sample at either end of an oscillation.
 */
#define BALL_TASK3_STOP_WINDOW_RIGHT_MM         (-40)
#define BALL_TASK3_STOP_WINDOW_LEFT_MM          (-60)
#define BALL_TASK3_STOP_MAX_SPEED_MM_S          (12.0f)
#define BALL_TASK3_STOP_HOLD_MS                 (300U)

/*
 * 连续满足条件的保持时间。
 */
#define BALL_TASK3_CENTER_HOLD_MS              (200U)
#define BALL_TASK3_NEGATIVE_HOLD_MS            (100U)
#define BALL_TASK3_FRAME_GAP_MAX_MS             (100U)
#define BALL_TASK3_LATE_MS                      (5000U)
#define BALL_TASK3_SAFETY_TIMEOUT_MS            (8000U)

/* 调试时只显示LATE并继续闭环；正式比赛前可改回1U恢复8秒回平。 */
#define BALL_TASK3_TIME_SAFETY_ENABLE           (0U)

/*
 * TASK3 uses a dedicated predictive relay controller.  The +5 cm point only
 * needs to be reached before reversal; the -5 cm point uses pulse-and-coast
 * capture so a correction cannot remain applied for an entire half cycle.
 */
#define BALL_TASK3_RELAY_ENABLE                 (0U)

/*
 * Dedicated mode-2 visual PID.  Its output is the absolute X42S shaft offset
 * in pulses; the derivative is taken from the filtered K230 ball velocity.
 * The same PID law is used throughout, with gain/output scheduling only to
 * match the transport and final-capture parts of the motion.
 */
#define BALL_TASK3_PID_OUTBOUND_KP              (1.65f)
#define BALL_TASK3_PID_OUTBOUND_KI              (0.00f)
#define BALL_TASK3_PID_OUTBOUND_KD              (0.65f)
#define BALL_TASK3_PID_RETURN_KP                (1.40f)
#define BALL_TASK3_PID_RETURN_KI                (0.00f)
#define BALL_TASK3_PID_RETURN_KD                (0.50f)
#define BALL_TASK3_PID_CAPTURE_KP               (1.20f)
#define BALL_TASK3_PID_CAPTURE_KI               (0.30f)
#define BALL_TASK3_PID_CAPTURE_KD               (1.00f)
#define BALL_TASK3_PID_RECOVERY_KP              (0.15f)
#define BALL_TASK3_PID_RECOVERY_KD              (1.25f)
#define BALL_TASK3_PID_CAPTURE_ZONE_MM          (55.0f)
#define BALL_TASK3_PID_CAPTURE_INNER_ZONE_MM    (15.0f)
#define BALL_TASK3_PID_INTEGRAL_ZONE_MM         (10.0f)
#define BALL_TASK3_PID_INTEGRAL_SPEED_MM_S      (25.0f)
#define BALL_TASK3_PID_INTEGRAL_LIMIT_PULSES    (12.0f)
#define BALL_TASK3_PID_POSITION_DEADBAND_MM     (1.5f)
#define BALL_TASK3_PID_SPEED_DEADBAND_MM_S      (4.0f)
#define BALL_TASK3_PID_VELOCITY_ALPHA           (0.60f)
#define BALL_TASK3_PID_OUTBOUND_LIMIT_PULSES    (90)
#define BALL_TASK3_PID_RETURN_LIMIT_PULSES      (92)
#define BALL_TASK3_PID_CAPTURE_OUTER_LIMIT      (140)
#define BALL_TASK3_PID_CAPTURE_INNER_LIMIT      (28)
#define BALL_TASK3_PID_RECOVERY_OUTER_LIMIT     (0)
#define BALL_TASK3_PID_RECOVERY_INNER_LIMIT     (0)
#define BALL_TASK3_PID_RIGHT_MOTION_BRAKE_SPEED (5.0f)
#define BALL_TASK3_PID_RIGHT_MOTION_BRAKE_MIN   (45)
#define BALL_TASK3_PID_RIGHT_MOTION_BRAKE_GAIN  (0.50f)
#define BALL_TASK3_PID_RIGHT_MOTION_BRAKE_LIMIT (90)
#define BALL_TASK3_PID_REDRIVE_THRESHOLD_MM     (-55.0f)
#define BALL_TASK3_PID_REDRIVE_SPEED_MM_S       (-5.0f)
#define BALL_TASK3_PID_REDRIVE_MIN_PULSES       (90)
#define BALL_TASK3_PID_REDRIVE_MAX_PULSES       (140)
#define BALL_TASK3_PID_REDRIVE_RAMP_PULSES      (5)
#define BALL_TASK3_PID_REDRIVE_RELEASE_PULSES   (10)
#define BALL_TASK3_PID_HOLD_BIAS_BACKOFF_PULSES (10)
#define BALL_TASK3_PID_HOLD_BIAS_MIN_PULSES     (70)
#define BALL_TASK3_PID_HOLD_BIAS_MAX_PULSES     (95)
#define BALL_TASK3_PID_HOLD_BRAKE_RANGE_PULSES  (35)
#define BALL_TASK3_PID_HOLD_DRIVE_RANGE_PULSES  (45)
#define BALL_TASK3_PID_RIGHT_RECAPTURE_MM       (-52.0f)
#define BALL_TASK3_PID_RECAPTURE_PREDICT_MM     (-62.0f)
#define BALL_TASK3_PID_RECAPTURE_LOOKAHEAD_S    (0.30f)
#define BALL_TASK3_PID_RECAPTURE_MAX_SPEED      (180.0f)
#define BALL_TASK3_PID_RECAPTURE_MIN_PULSES     (40)
#define BALL_TASK3_PID_CAPTURE_TRIGGER_MM       (-52.0f)
#define BALL_TASK3_PID_CAPTURE_MIN_MEASURED_MM  (-20.0f)
#define BALL_TASK3_PID_CAPTURE_LOOKAHEAD_S      (0.25f)
#define BALL_TASK3_PID_CAPTURE_MAX_SPEED        (200.0f)
#define BALL_TASK3_PID_PREDICTIVE_BRAKE_LIMIT  (0)
#define BALL_TASK3_PID_PREDICTIVE_LEVEL_DELTA  (45)
#define BALL_TASK3_PID_NORMAL_DELTA_PULSES      (15)
#define BALL_TASK3_PID_REVERSE_DELTA_PULSES     (28)
#define BALL_TASK3_PID_LAUNCH_DELTA_PULSES      (30)
#define BALL_TASK3_PID_LAUNCH_ZONE_MM           (5)
#define BALL_RELAY_OUTBOUND_LOOKAHEAD_S         (0.25f)
#define BALL_RELAY_RETURN_LOOKAHEAD_S           (0.42f)
#define BALL_RELAY_MAX_PREDICT_SPEED_MM_S       (160.0f)
#define BALL_RELAY_VELOCITY_DEADBAND_MM_S       (12.0f)
#define BALL_RELAY_TRANSPORT_FAR_PULSES         (120)
#define BALL_RELAY_TRANSPORT_MID_PULSES         (80)
#define BALL_RELAY_TRANSPORT_NEAR_PULSES        (50)
#define BALL_RELAY_RETURN_FAR_PULSES            (100)
#define BALL_RELAY_RETURN_MID_PULSES            (68)
#define BALL_RELAY_RETURN_NEAR_PULSES           (42)
#define BALL_RELAY_TRANSPORT_FAR_ERROR_MM       (50.0f)
#define BALL_RELAY_TRANSPORT_MID_ERROR_MM       (25.0f)
#define BALL_RELAY_CAPTURE_ZONE_MM              (80.0f)
#define BALL_RELAY_CAPTURE_INNER_ZONE_MM        (10.0f)
#define BALL_RELAY_CAPTURE_PREDICT_KP           (1.7f)
#define BALL_RELAY_CAPTURE_OUTER_LIMIT_PULSES   (68)
#define BALL_RELAY_CAPTURE_INNER_LIMIT_PULSES   (32)
#define BALL_RELAY_POSITIVE_BRAKE_OUTER_PULSES  (20)
#define BALL_RELAY_POSITIVE_BRAKE_INNER_PULSES  (10)
#define BALL_RELAY_TRANSPORT_DELTA_PULSES       (96)
#define BALL_RELAY_RETURN_DELTA_PULSES          (64)
#define BALL_RELAY_CAPTURE_OUTER_DELTA_PULSES   (40)
#define BALL_RELAY_CAPTURE_INNER_DELTA_PULSES   (16)
#define BALL_RELAY_CAPTURE_DEADBAND_MM          (2.0f)
#define BALL_RELAY_RETURN_POSITION_GATE_MM      (25.0f)
#define BALL_RELAY_RETURN_MIN_DRIVE_PULSES      (44)
#define BALL_RELAY_RETURN_REACQUIRE_ERROR_MM    (12.0f)
#define BALL_RELAY_RETURN_REACQUIRE_SPEED_MM_S  (-8.0f)
#define BALL_RELAY_RETURN_REACQUIRE_PULSES      (24)
#define BALL_RELAY_MAX_OFFSET_PULSES            (132)
#define BALL_RELAY_BIAS_LIMIT_PULSES             (24)
#define BALL_RELAY_BIAS_LEARN_STEP_PULSES         (2)
#define BALL_RELAY_BIAS_LEARN_ERROR_MM           (3.0f)
#define BALL_RELAY_BIAS_LEARN_SPEED_MM_S        (15.0f)
#define BALL_RELAY_BIAS_LEARN_PERIOD_MS         (100U)

#define BALL_DIRECTION_CHECK_MS                (600U)
#define BALL_DIRECTION_ERROR_INCREASE_MM       (20)
#define BALL_DEBUG_PRINT_PERIOD_MS             (50U)


typedef enum
{
    BALL_CONTROL_STOPPED = 0,
    BALL_CONTROL_SEARCHING,
    BALL_CONTROL_TRACKING,
    BALL_CONTROL_VISION_HOLD,
    BALL_CONTROL_LOST,
    BALL_CONTROL_K230_OVERFLOW,
    BALL_CONTROL_STEPPER_FAULT,
    BALL_CONTROL_DIRECTION_FAULT
} BallControlState;


typedef enum
{
    BALL_TASK3_DISABLED = 0,
    BALL_TASK3_WAIT_CENTER,
    BALL_TASK3_MOVE_POSITIVE,
    BALL_TASK3_MOVE_NEGATIVE,
    BALL_TASK3_SETTLE_NEGATIVE,
    BALL_TASK3_FINISHED,
    BALL_TASK3_SAFETY_STOP
} BallTask3State;


typedef enum
{
    BALL_REGION_IDLE = 0,
    BALL_REGION_OUTBOUND,
    BALL_REGION_RETURN_DRIVE,
    BALL_REGION_RETURN_CAPTURE
} BallControlRegion;


/*
 * ============================================================
 * 模块状态变量
 * ============================================================
 */

static BallControlState s_state;
static BallTask3State s_task3_state;

static K230_BallMessage s_message;

static uint32_t s_last_valid_ms;
static uint32_t s_last_measurement_interval_ms;
static uint32_t s_last_stepper_ms;
static uint32_t s_last_display_ms;

static uint32_t s_task3_condition_since_ms;
static uint32_t s_task3_condition_last_frame_ms;
static uint32_t s_task3_start_ms;
static uint32_t s_task3_elapsed_ms;
static uint32_t s_direction_check_start_ms;
static uint32_t s_last_debug_ms;
static uint32_t s_transport_stall_since_ms;
static uint32_t s_stepper_boot_start_ms;
static uint32_t s_stepper_last_zero_ms;

static int16_t s_previous_position_mm;
static int16_t s_target_position_mm;
static int16_t s_task3_origin_sensor_mm;

static int32_t s_stepper_offset_pulses;
static int32_t s_last_raw_pd_output_pulses;
static int32_t s_last_pd_output_pulses;
static int32_t s_last_p_output_pulses;
static int32_t s_last_i_output_pulses;
static int32_t s_last_d_output_pulses;
static int32_t s_direction_initial_error_mm;
static int32_t s_task3_hold_bias_pulses;
static int32_t s_task3_redrive_floor_pulses;
static int32_t s_task3_redrive_peak_pulses;

static float s_velocity_mm_s;
static float s_position_integral_mm_s;
static float s_task3_pid_velocity_mm_s;

static uint8_t s_have_measurement;
static uint8_t s_have_ever_received;
static uint8_t s_previous_running;
static uint8_t s_stepper_enabled;
static uint8_t s_stepper_fault_latched;
static uint8_t s_last_stepper_reply;

static uint8_t s_display_enabled;
static uint8_t s_rod_calibration_enabled;

static uint8_t s_task3_enabled;
static uint8_t s_task3_started;
static uint8_t s_auto_zero_hold_enabled;
static uint8_t s_auto_zero_hold_pending;
static uint8_t s_task3_condition_active;
static uint8_t s_task3_origin_valid;
static uint8_t s_direction_check_active;
static uint8_t s_direction_fault_latched;
static uint8_t s_new_measurement_this_update;
static uint8_t s_vision_hold_active;
static uint8_t s_invalid_frame_count;
static uint8_t s_transport_direction_gate_active;
static uint8_t s_transport_stall_active;
static uint8_t s_stepper_zero_attempts;
static uint8_t s_stepper_zero_done;
static uint8_t s_stepper_boot_enable_sent;
static uint8_t s_boot_home_calibration_requested;
static uint8_t s_task3_late;
static uint8_t s_outbound_switch_candidate_frames;
static uint8_t s_task3_negative_capture_latched;
static uint8_t s_task3_redrive_completed;
static BallControlRegion s_control_region;

static uint32_t s_task3_bias_learn_since_ms;

static uint32_t s_hold_ff_last_ms;
static float s_hold_ff_previous_speed_mm_s;
static float s_hold_ff_acceleration_mm_s2;
static int32_t s_hold_ff_pulses;
static float s_hold_velocity_mm_s;

static uint16_t s_last_ball_sequence;
static uint8_t s_have_ball_sequence;

static uint16_t s_angle_sequence;
static uint32_t s_last_angle_send_ms;


/*
 * ============================================================
 * 基础辅助函数
 * ============================================================
 */

static int32_t BallControl_RoundToInt(float value)
{
    return (value >= 0.0f) ?
        (int32_t)(value + 0.5f) :
        (int32_t)(value - 0.5f);
}


static int32_t BallControl_ClampInt32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static float BallControl_ClampFloat(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static bool BallControl_IsSequenceNewer(
    uint16_t candidate,
    uint16_t reference)
{
    uint16_t difference =
        (uint16_t)(candidate - reference);

    return (difference != 0U) &&
           (difference < 0x8000U);
}


static int32_t BallControl_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}


static float BallControl_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}


static uint32_t BallControl_MessageStaleLimitMs(void)
{
    return (s_auto_zero_hold_enabled != 0U) ?
        BALL_TASK4_MESSAGE_STALE_MS : BALL_MESSAGE_STALE_MS;
}


static uint32_t BallControl_VisionHoldLimitMs(void)
{
    return (s_auto_zero_hold_enabled != 0U) ?
        BALL_TASK4_VISION_HOLD_MS : BALL_VISION_HOLD_MS;
}


static uint8_t BallControl_InvalidFrameLimit(void)
{
    return (s_auto_zero_hold_enabled != 0U) ?
        BALL_TASK4_INVALID_FRAME_LIMIT : BALL_VISION_INVALID_FRAME_LIMIT;
}


static uint32_t BallControl_CenterAfterLostMs(void)
{
    return (s_auto_zero_hold_enabled != 0U) ?
        BALL_TASK4_CENTER_AFTER_LOST_MS : BALL_CENTER_AFTER_LOST_MS;
}


static int32_t BallControl_HoldAccelerationFeedforward(
    uint32_t now_ms)
{
    uint32_t interval_ms;
    float target_speed_mm_s;
    float raw_acceleration_mm_s2;

    if (s_auto_zero_hold_enabled == 0U) {
        return 0;
    }

    interval_ms =
        (uint32_t)(now_ms - s_hold_ff_last_ms);
    if (interval_ms <
        BALL_HOLD_ACCEL_FF_MIN_INTERVAL_MS) {
        return s_hold_ff_pulses;
    }

    target_speed_mm_s =
        (MotorA.Target_Encoder +
         MotorB.Target_Encoder) * 500.0f;
    raw_acceleration_mm_s2 =
        (target_speed_mm_s -
         s_hold_ff_previous_speed_mm_s) *
        (1000.0f / (float)interval_ms);
    raw_acceleration_mm_s2 =
        BallControl_ClampFloat(
            raw_acceleration_mm_s2,
            -BALL_HOLD_ACCEL_FF_LIMIT_MM_S2,
            BALL_HOLD_ACCEL_FF_LIMIT_MM_S2);

    s_hold_ff_acceleration_mm_s2 =
        BALL_HOLD_ACCEL_FF_ALPHA *
        raw_acceleration_mm_s2 +
        (1.0f - BALL_HOLD_ACCEL_FF_ALPHA) *
        s_hold_ff_acceleration_mm_s2;

    if (BallControl_AbsFloat(
            s_hold_ff_acceleration_mm_s2) <
        BALL_HOLD_ACCEL_FF_DEADBAND_MM_S2) {
        s_hold_ff_acceleration_mm_s2 = 0.0f;
    }

    s_hold_ff_previous_speed_mm_s =
        target_speed_mm_s;
    s_hold_ff_last_ms = now_ms;
    s_hold_ff_pulses =
        BallControl_ClampInt32(
            BallControl_RoundToInt(
                BALL_HOLD_ACCEL_FF_SIGN *
                BALL_HOLD_ACCEL_FF_GAIN *
                s_hold_ff_acceleration_mm_s2),
            -BALL_HOLD_ACCEL_FF_LIMIT_PULSES,
            BALL_HOLD_ACCEL_FF_LIMIT_PULSES);

    return s_hold_ff_pulses;
}


/*
 * 题目3按键时把球所在的物理O点记录为视觉零点。
 * 后续位置统一减去该固定偏差，不要求K230恰好显示0.0。
 */
static int16_t BallControl_MeasuredPositionMm(void)
{
    int32_t position_mm =
        (int32_t)s_message.position_mm;

    if (s_task3_origin_valid != 0U) {
        position_mm -=
            (int32_t)s_task3_origin_sensor_mm;
    }

    position_mm = BallControl_ClampInt32(
        position_mm,
        -32768,
        32767);

    return (int16_t)position_mm;
}


static int16_t BallControl_StepperAngleCentidegrees(void)
{
    int32_t scaled =
        s_stepper_offset_pulses * 36000L;

    if (scaled >= 0) {
        scaled +=
            BALL_STEPPER_PULSES_PER_REVOLUTION / 2L;
    }
    else {
        scaled -=
            BALL_STEPPER_PULSES_PER_REVOLUTION / 2L;
    }

    scaled /=
        BALL_STEPPER_PULSES_PER_REVOLUTION;

    scaled = BallControl_ClampInt32(
        scaled,
        -32768,
        32767);

    return (int16_t)scaled;
}


static void BallControl_SendAngleHeartbeat(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_angle_send_ms) <
        BALL_ANGLE_SEND_PERIOD_MS) {
        return;
    }

    s_last_angle_send_ms = now_ms;

    K230_UART_SendAngle(
        s_angle_sequence,
        BallControl_StepperAngleCentidegrees());

    s_angle_sequence++;
}


/*
 * ============================================================
 * 题目第3项状态机
 * ============================================================
 */

static void BallControl_Task3ResetCondition(void)
{
    s_task3_condition_active = 0U;
    s_task3_condition_since_ms = 0U;
    s_task3_condition_last_frame_ms = 0U;
}


static void BallControl_ResetPidMemory(void)
{
    s_position_integral_mm_s = 0.0f;
    s_task3_pid_velocity_mm_s = 0.0f;
    s_last_measurement_interval_ms = 0U;
    s_last_p_output_pulses = 0;
    s_last_i_output_pulses = 0;
    s_last_d_output_pulses = 0;
    s_last_raw_pd_output_pulses = 0;
    s_last_pd_output_pulses = 0;
    s_transport_direction_gate_active = 0U;
    s_transport_stall_since_ms = 0U;
    s_transport_stall_active = 0U;
    s_outbound_switch_candidate_frames = 0U;
    s_task3_redrive_floor_pulses = 0;
    s_task3_redrive_peak_pulses = 0;
    s_task3_redrive_completed = 0U;
    s_task3_hold_bias_pulses = 0;
    s_hold_velocity_mm_s = 0.0f;
}


static void BallControl_Task3UpdateTime(uint32_t now_ms)
{
    if ((s_task3_enabled == 0U) ||
        (s_task3_started == 0U) ||
        (s_task3_state == BALL_TASK3_FINISHED) ||
        (s_task3_state == BALL_TASK3_SAFETY_STOP)) {
        return;
    }

    s_task3_elapsed_ms =
        (uint32_t)(now_ms - s_task3_start_ms);

    if (s_task3_elapsed_ms >= BALL_TASK3_LATE_MS) {
        s_task3_late = 1U;
    }

#if BALL_TASK3_TIME_SAFETY_ENABLE == 1U
    if (s_task3_elapsed_ms < BALL_TASK3_SAFETY_TIMEOUT_MS) {
        return;
    }

    s_task3_elapsed_ms = BALL_TASK3_SAFETY_TIMEOUT_MS;
    s_task3_started = 0U;
    s_task3_state = BALL_TASK3_SAFETY_STOP;
    s_target_position_mm = BALL_TASK3_CENTER_TARGET_MM;
    s_control_region = BALL_REGION_IDLE;
    s_direction_check_active = 0U;
    BallControl_ResetPidMemory();
    BallControl_Task3ResetCondition();
#endif
}


static void BallControl_Task3ConfigureWaiting(void)
{
    s_task3_started = 0U;
    s_task3_elapsed_ms = 0U;
    s_task3_origin_valid = 0U;
    s_task3_origin_sensor_mm = 0;
    s_direction_check_active = 0U;
    s_task3_late = 0U;
    s_task3_negative_capture_latched = 0U;
    s_control_region = BALL_REGION_IDLE;
    s_task3_hold_bias_pulses = 0;
    s_task3_bias_learn_since_ms = 0U;
    BallControl_ResetPidMemory();

    BallControl_Task3ResetCondition();

    if (s_task3_enabled != 0U) {
        s_task3_state = BALL_TASK3_WAIT_CENTER;
        s_target_position_mm = BALL_TASK3_CENTER_TARGET_MM;
    }
    else {
        s_task3_state = BALL_TASK3_DISABLED;
    }
}


static void BallControl_Task3Begin(uint32_t now_ms)
{
    if ((s_task3_enabled == 0U) ||
        (s_have_measurement == 0U) ||
        (s_vision_hold_active != 0U) ||
        ((uint32_t)(now_ms - s_last_valid_ms) >
         BALL_MESSAGE_STALE_MS)) {
        return;
    }

    /* 用户把球物理放在O点后按键，当前视觉读数即零偏。 */
    s_task3_origin_sensor_mm =
        s_message.position_mm;
    s_task3_origin_valid = 1U;

    /* Start each scored run with clean velocity and direction histories. */
    s_previous_position_mm = s_message.position_mm;
    s_velocity_mm_s = 0.0f;
    s_task3_late = 0U;
    s_task3_negative_capture_latched = 0U;
    s_control_region = BALL_REGION_OUTBOUND;
    s_task3_hold_bias_pulses = 0;
    s_task3_bias_learn_since_ms = 0U;
    BallControl_ResetPidMemory();
    s_last_ball_sequence = 0U;
    s_have_ball_sequence = 0U;

    s_task3_started = 1U;

    s_task3_state =
        BALL_TASK3_MOVE_POSITIVE;

    s_target_position_mm =
        BALL_TASK3_POSITIVE_TARGET_MM;

    s_task3_start_ms = now_ms;
    s_task3_elapsed_ms = 0U;

    s_direction_check_start_ms = now_ms;
    s_direction_initial_error_mm =
        BallControl_AbsInt32(
            (int32_t)BALL_TASK3_POSITIVE_TARGET_MM -
            (int32_t)BallControl_MeasuredPositionMm());
    s_direction_check_active = 1U;

    BallControl_Task3ResetCondition();
}


static bool BallControl_Task3NearTarget(
    int16_t target_mm,
    int16_t tolerance_mm,
    float maximum_speed_mm_s)
{
    int32_t error_mm;

    if (s_have_measurement == 0U) {
        return false;
    }

    error_mm =
        (int32_t)target_mm -
        (int32_t)BallControl_MeasuredPositionMm();

    return
        (BallControl_AbsInt32(error_mm) <=
         (int32_t)tolerance_mm) &&
        (BallControl_AbsFloat(s_velocity_mm_s) <=
         maximum_speed_mm_s);
}


static bool BallControl_Task3ConditionHeld(
    bool condition,
    uint32_t now_ms,
    uint32_t hold_ms)
{
    /*
     * Count only newly received, monotonically increasing BALL frames.
     * Re-running the fast main loop must not turn one 20 Hz frame into
     * several apparent confirmations.
     */
    if (s_new_measurement_this_update == 0U) {
        if ((s_task3_condition_active != 0U) &&
            ((uint32_t)(now_ms - s_task3_condition_last_frame_ms) >
             BALL_TASK3_FRAME_GAP_MAX_MS)) {

            BallControl_Task3ResetCondition();
        }
        return false;
    }

    if ((s_task3_condition_active != 0U) &&
        ((uint32_t)(now_ms - s_task3_condition_last_frame_ms) >
         BALL_TASK3_FRAME_GAP_MAX_MS)) {

        BallControl_Task3ResetCondition();
    }

    if (!condition) {
        BallControl_Task3ResetCondition();
        return false;
    }

    if (s_task3_condition_active == 0U) {
        s_task3_condition_active = 1U;
        s_task3_condition_since_ms = now_ms;
        s_task3_condition_last_frame_ms = now_ms;
        return false;
    }

    s_task3_condition_last_frame_ms = now_ms;

    return
        (uint32_t)(now_ms - s_task3_condition_since_ms) >=
        hold_ms;
}


static void BallControl_Task3Update(uint32_t now_ms)
{
    bool reached;
    float projected_position_mm;
    float prediction_speed_mm_s;
    int16_t measured_position_mm;

    if ((s_task3_enabled == 0U) ||
        (s_task3_started == 0U) ||
        (s_have_measurement == 0U)) {

        BallControl_Task3ResetCondition();
        return;
    }

    switch (s_task3_state) {
    case BALL_TASK3_WAIT_CENTER:

        /*
         * 第一步：
         *
         * 先控制球回到中心O附近。
         */
        s_target_position_mm =
            BALL_TASK3_CENTER_TARGET_MM;

        reached = BallControl_Task3NearTarget(
            BALL_TASK3_CENTER_TARGET_MM,
            BALL_TASK3_CENTER_TOLERANCE_MM,
            BALL_TASK3_CENTER_MAX_SPEED_MM_S);

        /*
         * 球在±5mm范围内、速度足够小，
         * 连续保持200ms后开始前往+5cm。
         */
        if (BallControl_Task3ConditionHeld(
                reached,
                now_ms,
                BALL_TASK3_CENTER_HOLD_MS)) {

            s_task3_state =
                BALL_TASK3_MOVE_POSITIVE;

            s_target_position_mm =
                BALL_TASK3_POSITIVE_TARGET_MM;

            s_outbound_switch_candidate_frames = 0U;
            BallControl_Task3ResetCondition();
        }
        break;


    case BALL_TASK3_MOVE_POSITIVE:

        /*
         * 第二步：
         *
         * 控制球前往+50mm。
         */
        s_target_position_mm =
            BALL_TASK3_POSITIVE_TARGET_MM;

        /*
         * +5cm只要求到达后折返，不要求在该点稳定。
         * 使用目标减去允许误差作为单向到达阈值，可避免20Hz视觉
         * 采样恰好跳过+50mm附近时漏掉折返事件。
         */
        if (s_new_measurement_this_update != 0U) {
            measured_position_mm =
                BallControl_MeasuredPositionMm();

            prediction_speed_mm_s =
                BallControl_ClampFloat(
                    s_velocity_mm_s,
                    0.0f,
                    BALL_TASK3_POSITIVE_MAX_PREDICT_SPEED);

            projected_position_mm =
                (float)measured_position_mm +
                prediction_speed_mm_s *
                BALL_TASK3_POSITIVE_LOOKAHEAD_S;

            /*
             * A fast ball reverses earlier than a slow ball. The predicted
             * crossing anticipates inertia before the measured fallback;
             * +12 mm is the measured-position fallback, allowing the existing
             * ball inertia to carry it close to the physical +5 cm mark.
             */
            if (measured_position_mm >=
                BALL_TASK3_POSITIVE_SWITCH_MM) {
                s_outbound_switch_candidate_frames =
                    BALL_TASK3_POSITIVE_CONFIRM_FRAMES;
            }
            else if (((float)measured_position_mm >=
                      BALL_TASK3_POSITIVE_PREDICT_MIN_MM) &&
                     (projected_position_mm >=
                      BALL_TASK3_POSITIVE_PREDICT_MM)) {
                if (s_outbound_switch_candidate_frames < 255U) {
                    s_outbound_switch_candidate_frames++;
                }
            }
            else {
                s_outbound_switch_candidate_frames = 0U;
            }
        }

        if (s_outbound_switch_candidate_frames >=
            BALL_TASK3_POSITIVE_CONFIRM_FRAMES) {

            s_task3_state =
                BALL_TASK3_MOVE_NEGATIVE;

            s_control_region =
                BALL_REGION_RETURN_DRIVE;

            s_task3_negative_capture_latched = 0U;

            s_direction_check_active = 0U;

            s_target_position_mm =
                BALL_TASK3_NEGATIVE_TARGET_MM;

            BallControl_ResetPidMemory();
            /* Preserve the measured positive-going momentum for PID braking. */
            s_task3_pid_velocity_mm_s = s_velocity_mm_s;
            BallControl_Task3ResetCondition();
        }
        break;


    case BALL_TASK3_MOVE_NEGATIVE:
    case BALL_TASK3_SETTLE_NEGATIVE:

        /*
         * 第三步：
         *
         * 控制球前往-50mm。
         */
        s_target_position_mm =
            BALL_TASK3_NEGATIVE_TARGET_MM;

        measured_position_mm =
            BallControl_MeasuredPositionMm();
        reached =
            (s_task3_redrive_completed != 0U) &&
            (measured_position_mm >=
             BALL_TASK3_STOP_WINDOW_LEFT_MM) &&
            (measured_position_mm <=
             BALL_TASK3_STOP_WINDOW_RIGHT_MM) &&
            (BallControl_AbsFloat(s_velocity_mm_s) <=
             BALL_TASK3_STOP_MAX_SPEED_MM_S) &&
            (BallControl_AbsFloat(s_task3_pid_velocity_mm_s) <=
             BALL_TASK3_STOP_MAX_SPEED_MM_S);

        if ((s_new_measurement_this_update != 0U) && reached) {
            s_task3_state = BALL_TASK3_SETTLE_NEGATIVE;
        }

        /* -50mm同样要求位置和速度稳定300ms。 */
        if (BallControl_Task3ConditionHeld(
                reached,
                now_ms,
                BALL_TASK3_STOP_HOLD_MS)) {

            s_task3_state =
                BALL_TASK3_FINISHED;

            s_control_region =
                BALL_REGION_RETURN_CAPTURE;

            /*
             * 冻结最终完成时间。
             */
            s_task3_elapsed_ms =
                (uint32_t)(
                    s_task3_condition_since_ms -
                    s_task3_start_ms);

            BallControl_Task3ResetCondition();
        }
        else if ((s_task3_condition_active == 0U) &&
                 (s_task3_state == BALL_TASK3_SETTLE_NEGATIVE)) {
            s_task3_state = BALL_TASK3_MOVE_NEGATIVE;
        }
        break;


    case BALL_TASK3_FINISHED:

        /*
         * 完成后仍然将目标保持在-50mm，
         * 使钢球继续稳定在最终位置。
         */
        s_target_position_mm =
            BALL_TASK3_NEGATIVE_TARGET_MM;

        BallControl_Task3ResetCondition();
        break;


    case BALL_TASK3_SAFETY_STOP:

        s_target_position_mm = BALL_TASK3_CENTER_TARGET_MM;
        BallControl_Task3ResetCondition();
        break;


    case BALL_TASK3_DISABLED:
    default:

        BallControl_Task3ResetCondition();
        break;
    }
}


/*
 * ============================================================
 * OLED显示辅助函数
 * ============================================================
 */

static void BallControl_ClearFrame(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < 8U; page++) {
        for (column = 0U; column < 128U; column++) {
            OLED_GRAM[column][page] = 0U;
        }
    }
}


static void BallControl_DrawText(
    uint8_t x,
    uint8_t y,
    const char *text)
{
    while ((*text != '\0') &&
           (x <= 120U)) {

        OLED_ShowChar(
            x,
            y,
            (uint8_t)*text,
            12U,
            1U);

        x = (uint8_t)(x + 8U);
        text++;
    }
}


static void BallControl_DrawUnsigned(
    uint8_t x,
    uint8_t y,
    uint32_t value,
    uint8_t digits)
{
    uint32_t divisor = 1U;
    uint8_t index;

    for (index = 1U;
         index < digits;
         index++) {

        divisor *= 10U;
    }

    for (index = 0U;
         index < digits;
         index++) {

        uint8_t digit =
            (uint8_t)(
                (value / divisor) %
                10U
            );

        OLED_ShowChar(
            x,
            y,
            (uint8_t)('0' + digit),
            12U,
            1U);

        x = (uint8_t)(x + 8U);

        if (divisor > 1U) {
            divisor /= 10U;
        }
    }
}


static void BallControl_DrawSigned(
    uint8_t x,
    uint8_t y,
    int32_t value,
    uint8_t digits)
{
    uint32_t magnitude;

    if (value < 0) {
        OLED_ShowChar(
            x,
            y,
            (uint8_t)'-',
            12U,
            1U);

        magnitude =
            (uint32_t)(
                -(value + 1)
            ) + 1U;
    }
    else {
        OLED_ShowChar(
            x,
            y,
            (uint8_t)'+',
            12U,
            1U);

        magnitude =
            (uint32_t)value;
    }

    BallControl_DrawUnsigned(
        (uint8_t)(x + 8U),
        y,
        magnitude,
        digits);
}


static const char *BallControl_Task3Text(void)
{
    switch (s_task3_state) {
    case BALL_TASK3_WAIT_CENTER:
        return "T3 WAIT O";

    case BALL_TASK3_MOVE_POSITIVE:
        return (s_task3_late != 0U) ?
               "T3 LATE +50" : "T3 TO +50";

    case BALL_TASK3_MOVE_NEGATIVE:
        return (s_task3_late != 0U) ?
               "T3 LATE -50" : "T3 TO -50";

    case BALL_TASK3_SETTLE_NEGATIVE:
        return (s_task3_late != 0U) ?
               "T3 LATE -50" : "T3 SET -50";

    case BALL_TASK3_FINISHED:
        return (s_task3_late != 0U) ?
               "T3 FIN LATE" : "T3 FINISHED";

    case BALL_TASK3_SAFETY_STOP:
        return "T3 SAFETY STOP";

    case BALL_TASK3_DISABLED:
    default:
        return "TRACKING";
    }
}


static const char *BallControl_StateText(void)
{
    K230UartStats uart_stats;

    K230_UART_GetStats(&uart_stats);

    switch (s_state) {
    case BALL_CONTROL_STOPPED:
        if (s_stepper_zero_done == 0U) {
            return (s_boot_home_calibration_requested != 0U) ?
                   "SET HOME ZERO" : "RETURN TO HOME";
        }
        if (s_task3_enabled != 0U) {
            if (s_vision_hold_active != 0U) {
                return "K230 HOLD WAIT";
            }
            if (s_have_measurement != 0U) {
                return "PLACE O PRESS KEY";
            }
            if (uart_stats.rx_bytes == 0U) {
                return "RX0 CHECK PB7";
            }
            if (uart_stats.valid_ball_frames == 0U) {
                return "RX BAD FRAME";
            }
            if (s_message.status != K230_BALL_TRACKING) {
                return "K230 STATUS ZERO";
            }
            if (s_message.confidence_permille <
                BALL_MIN_CONFIDENCE_PERMILLE) {
                return "K230 LOW CONF";
            }
            return "K230 FRAME STALE";
        }
        return "STOP KEY TO RUN";

    case BALL_CONTROL_SEARCHING:
        if ((s_task3_enabled != 0U) &&
            (Flag_Stop == 0)) {
            return "WAIT VALID BALL";
        }
        return "SEARCH BALL";

    case BALL_CONTROL_TRACKING:

        if (s_task3_enabled != 0U) {
            return BallControl_Task3Text();
        }

        if (s_rod_calibration_enabled != 0U) {
            return "ROD CAL MODE";
        }

        return "TRACKING";

    case BALL_CONTROL_VISION_HOLD:
        return "BALL HOLD";

    case BALL_CONTROL_LOST:
        return "BALL LOST";

    case BALL_CONTROL_K230_OVERFLOW:
        return "K230 OVERFLOW";

    case BALL_CONTROL_STEPPER_FAULT:
        return "X42S FAULT";

    case BALL_CONTROL_DIRECTION_FAULT:
        return "CONTROL DIR";

    default:
        return "ERROR";
    }
}


static void BallControl_UpdateDisplay(
    uint32_t now_ms,
    uint8_t force)
{
    if (s_display_enabled == 0U) {
        return;
    }

    if ((force == 0U) &&
        ((uint32_t)(
            now_ms -
            s_last_display_ms
        ) < BALL_DISPLAY_UPDATE_MS)) {

        return;
    }

    s_last_display_ms = now_ms;

    BallControl_ClearFrame();

    BallControl_DrawText(
        0U,
        0U,
        "BALL X42S EMM");

    BallControl_DrawText(
        0U,
        16U,
        BallControl_StateText());

    /*
     * 当前球位置。
     */
    BallControl_DrawText(
        0U,
        32U,
        "P:");

    if (s_have_measurement != 0U) {
        BallControl_DrawSigned(
            16U,
            32U,
            BallControl_MeasuredPositionMm(),
            3U);
    }
    else {
        BallControl_DrawText(
            16U,
            32U,
            "----");
    }

    /*
     * 当前目标位置。
     */
    BallControl_DrawText(
        56U,
        32U,
        "T:");

    BallControl_DrawSigned(
        72U,
        32U,
        s_target_position_mm,
        3U);

    if ((s_task3_enabled != 0U) &&
        ((s_state == BALL_CONTROL_STOPPED) ||
         (s_state == BALL_CONTROL_SEARCHING))) {
        K230UartStats uart_stats;

        K230_UART_GetStats(&uart_stats);

        /* B=received bytes, V=valid BALL frames, O=overflows. */
        BallControl_DrawText(0U, 48U, "B:");
        BallControl_DrawUnsigned(
            16U,
            48U,
            uart_stats.rx_bytes % 10000U,
            4U);

        BallControl_DrawText(48U, 48U, "V:");
        BallControl_DrawUnsigned(
            64U,
            48U,
            uart_stats.valid_ball_frames % 1000U,
            3U);

        BallControl_DrawText(88U, 48U, "O:");
        BallControl_DrawUnsigned(
            104U,
            48U,
            uart_stats.overflows % 100U,
            2U);
    }
    else if (s_task3_enabled != 0U) {
        uint32_t shown_time_ms = s_task3_elapsed_ms;
        uint32_t shown_seconds;
        uint32_t shown_centiseconds;

        if (shown_time_ms > 99990U) {
            shown_time_ms = 99990U;
        }

        shown_seconds = shown_time_ms / 1000U;
        shown_centiseconds =
            (shown_time_ms % 1000U) / 10U;

        /* Show the complete mode-2 stopwatch as seconds.centiseconds. */
        BallControl_DrawText(0U, 48U, "TM:");
        BallControl_DrawUnsigned(
            24U, 48U, shown_seconds, 2U);
        BallControl_DrawText(40U, 48U, ".");
        BallControl_DrawUnsigned(
            48U, 48U, shown_centiseconds, 2U);

        /* Keep the motor offset visible without hiding the timer. */
        BallControl_DrawText(72U, 48U, "O:");
        BallControl_DrawSigned(
            88U, 48U,
            s_stepper_offset_pulses,
            3U);
    }
    else {
        BallControl_DrawText(0U, 48U, "C:");
        BallControl_DrawUnsigned(
            16U, 48U,
            s_message.confidence_permille,
            4U);

        BallControl_DrawText(56U, 48U, "O:");
        BallControl_DrawSigned(
            72U, 48U,
            s_stepper_offset_pulses,
            3U);
    }

    OLED_Refresh_Gram();
}


/*
 * ============================================================
 * X42S回复与故障处理
 * ============================================================
 */

static void BallControl_LatchStepperFault(
    uint8_t reply_status)
{
    if (s_stepper_fault_latched == 0U) {
        (void)StepperEmm_Stop(
            STEPPER_EMM_DEFAULT_ADDRESS);
    }

    s_last_stepper_reply =
        reply_status;

    s_stepper_fault_latched =
        1U;

    s_stepper_enabled =
        0U;

    s_have_measurement =
        0U;

    s_state =
        BALL_CONTROL_STEPPER_FAULT;

    BallControl_Task3ResetCondition();
}


static bool BallControl_ServiceBootZero(uint32_t now_ms)
{
    if (s_stepper_zero_done != 0U) {
        return true;
    }

    if ((uint32_t)(now_ms - s_stepper_boot_start_ms) <
        BALL_STEPPER_BOOT_ZERO_DELAY_MS) {
        return false;
    }

    if (s_stepper_boot_enable_sent == 0U) {
        if (StepperEmm_Enable(
                STEPPER_EMM_DEFAULT_ADDRESS,
                true)) {
            s_stepper_boot_enable_sent = 1U;
            s_stepper_enabled = 1U;
            s_stepper_last_zero_ms = now_ms;
        }
        return false;
    }

    if ((s_stepper_zero_attempts == 0U) &&
        ((uint32_t)(now_ms - s_stepper_last_zero_ms) <
         BALL_STEPPER_BOOT_COMMAND_GAP_MS)) {
        return false;
    }

    if (s_stepper_zero_attempts != 0U) {
        if (s_boot_home_calibration_requested == 0U) {
            if ((uint32_t)(now_ms - s_stepper_last_zero_ms) <
                BALL_STEPPER_BOOT_RETURN_SETTLE_MS) {
                return false;
            }
            s_stepper_zero_done = 1U;
            s_stepper_offset_pulses = 0;
            return true;
        }

        if ((uint32_t)(now_ms - s_stepper_last_zero_ms) <
            BALL_STEPPER_BOOT_ZERO_RETRY_MS) {
            return false;
        }
    }

    if (s_stepper_zero_attempts >=
        BALL_STEPPER_BOOT_ZERO_MAX_ATTEMPTS) {
        s_stepper_zero_done = 1U;
        s_stepper_offset_pulses = 0;
        return true;
    }

    if (((s_boot_home_calibration_requested != 0U) &&
         StepperEmm_ClearPosition(
             STEPPER_EMM_DEFAULT_ADDRESS)) ||
        ((s_boot_home_calibration_requested == 0U) &&
         StepperEmm_MoveAbsolute(
             STEPPER_EMM_DEFAULT_ADDRESS,
             0,
             BALL_STEPPER_CENTER_SPEED_RPM,
             BALL_STEPPER_ACCELERATION))) {
        s_stepper_zero_attempts++;
        s_stepper_last_zero_ms = now_ms;
        s_stepper_offset_pulses = 0;
    }

    return false;
}


static void BallControl_CheckStepperReplies(void)
{
    StepperEmmEvent event;

    /*
     * X42S接收缓冲区溢出。
     */
    if (StepperEmm_TakeRxOverflow()) {

        StepperEmm_ClearRx();

        BallControl_LatchStepperFault(
            STEPPER_EMM_REPLY_FORMAT_ERROR);

        return;
    }

    /*
     * 读取所有完整回复。
     */
    while (StepperEmm_ReadEvent(&event)) {

        if (event.address !=
            STEPPER_EMM_DEFAULT_ADDRESS) {

            continue;
        }

        if (event.type != STEPPER_EMM_EVENT_REPLY) {
            continue;
        }

        /*
         * 注意：
         *
         * 这里不能多写一个右花括号。
         * 你原来的代码就是在continue后提前结束了while作用域。
         */
        s_last_stepper_reply =
            event.status;

        if ((event.function ==
             BALL_STEPPER_CLEAR_FUNCTION) &&
            (event.status == STEPPER_EMM_REPLY_OK)) {
            s_stepper_zero_done = 1U;
            s_stepper_offset_pulses = 0;
        }

        if ((event.status ==
             STEPPER_EMM_REPLY_PARAMETER_ERROR) ||

            (event.status ==
             STEPPER_EMM_REPLY_FORMAT_ERROR) ||

            (event.status ==
             STEPPER_EMM_REPLY_ZERO_LIMIT) ||

            (event.status ==
             STEPPER_EMM_REPLY_LIMIT)) {

            BallControl_LatchStepperFault(
                event.status);

            return;
        }
    }
}


/*
 * ============================================================
 * K230位置消息处理
 * ============================================================
 */

static bool BallControl_AcceptMessage(
    const K230_BallMessage *message,
    uint32_t now_ms)
{
    uint32_t interval_ms =
        (uint32_t)(
            now_ms -
            s_last_valid_ms
        );

    s_have_ever_received = 1U;

    /*
     * 状态和置信度必须有效。
     */
    if ((message->status !=
         K230_BALL_TRACKING) ||

        (message->confidence_permille <
         BALL_MIN_CONFIDENCE_PERMILLE)) {

        if (s_invalid_frame_count < 255U) {
            s_invalid_frame_count++;
        }

        s_vision_hold_active = 1U;

        BallControl_Task3ResetCondition();

        if ((s_invalid_frame_count >=
             BallControl_InvalidFrameLimit()) ||
            (interval_ms >= BallControl_MessageStaleLimitMs())) {

            s_have_measurement = 0U;
            s_velocity_mm_s = 0.0f;
        }

        return false;
    }

    s_message = *message;
    s_last_measurement_interval_ms = interval_ms;
    s_invalid_frame_count = 0U;
    s_vision_hold_active = 0U;

    /*
     * 根据相邻两次位置计算速度，并做低通滤波。
     */
    if ((s_have_measurement != 0U) &&
        (interval_ms >= BALL_VELOCITY_MIN_INTERVAL_MS) &&
        (interval_ms <= BALL_VELOCITY_MAX_INTERVAL_MS)) {

        float instantaneous_velocity_mm_s;

        instantaneous_velocity_mm_s =
            (
                (float)message->position_mm -
                (float)s_previous_position_mm
            ) *
            (
                1000.0f /
                (float)interval_ms
            );

        instantaneous_velocity_mm_s =
            BallControl_ClampFloat(
                instantaneous_velocity_mm_s,
                -BALL_VELOCITY_LIMIT_MM_S,
                BALL_VELOCITY_LIMIT_MM_S);

        s_velocity_mm_s =
            BALL_VELOCITY_FILTER_ALPHA *
            instantaneous_velocity_mm_s +

            (1.0f -
             BALL_VELOCITY_FILTER_ALPHA) *
            s_velocity_mm_s;
    }
    else {
        s_velocity_mm_s = 0.0f;
    }

    s_previous_position_mm =
        message->position_mm;

    s_last_valid_ms =
        now_ms;

    s_have_measurement =
        1U;

    return true;
}


/*
 * ============================================================
 * 步进电机命令生成
 * ============================================================
 */

static void BallControl_CommandOffset(
    int32_t requested_offset_pulses,
    uint16_t speed_rpm,
    int32_t maximum_offset_pulses,
    int32_t maximum_delta_pulses)
{
    int32_t delta_pulses;
    int32_t next_offset_pulses;

    /*
     * 限制绝对逻辑偏移。
     */
    requested_offset_pulses =
        BallControl_ClampInt32(
            requested_offset_pulses,
            -maximum_offset_pulses,
            maximum_offset_pulses);

    /*
     * 绝对目标转换为本次相对增量。
     */
    delta_pulses =
        requested_offset_pulses -
        s_stepper_offset_pulses;

    /*
     * 限制单次命令变化量。
     */
    delta_pulses =
        BallControl_ClampInt32(
            delta_pulses,
            -maximum_delta_pulses,
            maximum_delta_pulses);

    if (delta_pulses == 0) {
        return;
    }

    next_offset_pulses =
        s_stepper_offset_pulses +
        delta_pulses;

    /*
     * The validated lift-test path uses absolute X42S positions.  Use the
     * same mode here so frequent control updates cannot lose relative moves
     * while the previous command is still executing.
     */
    if (StepperEmm_MoveAbsolute(
            STEPPER_EMM_DEFAULT_ADDRESS,
            next_offset_pulses,
            speed_rpm,
            BALL_STEPPER_ACCELERATION)) {

        s_stepper_offset_pulses =
            next_offset_pulses;
    }
    else {
        BallControl_LatchStepperFault(
            STEPPER_EMM_REPLY_PARAMETER_ERROR);
    }
}


static void BallControl_RunPositionLoop(
    uint32_t now_ms)
{
    float position_error;
    float velocity_feedback;
    float absolute_position_error;
    float kp_gain;
    float ki_gain;
    float kd_gain;
    float active_capture_zone_mm;
    float integral_limit_pulses;
    float p_term;
    float i_term;
    float d_term;
    float integral_candidate;
    float integral_delta_output;
    float candidate_requested_offset;
    float requested_offset;
    float desired_velocity_mm_s;
    int32_t vehicle_accel_ff_pulses;
    int32_t requested_offset_pulses;
    int32_t maximum_offset_pulses;
    int32_t maximum_delta_pulses;
    int32_t position_drive_sign;
    BallControlRegion previous_control_region;
    uint8_t integral_update_enabled;
    uint8_t transport_control_active;

    if ((uint32_t)(
            now_ms -
            s_last_stepper_ms
        ) < BALL_STEPPER_UPDATE_MS) {

        return;
    }

    s_last_stepper_ms = now_ms;

    /*
     * 位置误差：
     *
     * 目标位置 - 当前球位置
     */
    position_error =
        (float)s_target_position_mm -
        (float)BallControl_MeasuredPositionMm();

    velocity_feedback = s_velocity_mm_s;
    absolute_position_error =
        BallControl_AbsFloat(position_error);

    previous_control_region = s_control_region;
    active_capture_zone_mm = BALL_CAPTURE_ZONE_MM;

    if (s_task3_state == BALL_TASK3_MOVE_POSITIVE) {
        s_control_region = BALL_REGION_OUTBOUND;
    }
    else if (((s_task3_state == BALL_TASK3_MOVE_NEGATIVE) ||
              (s_task3_state == BALL_TASK3_SETTLE_NEGATIVE) ||
              (s_task3_state == BALL_TASK3_FINISHED)) &&
             (position_error < -BALL_RETURN_CAPTURE_ZONE_MM)) {
        s_control_region = BALL_REGION_RETURN_DRIVE;
        active_capture_zone_mm = BALL_RETURN_CAPTURE_ZONE_MM;
    }
    else if ((s_task3_state == BALL_TASK3_MOVE_NEGATIVE) ||
             (s_task3_state == BALL_TASK3_SETTLE_NEGATIVE) ||
             (s_task3_state == BALL_TASK3_FINISHED)) {
        s_control_region = BALL_REGION_RETURN_CAPTURE;
        active_capture_zone_mm = BALL_RETURN_CAPTURE_ZONE_MM;
    }
    else {
        s_control_region = BALL_REGION_IDLE;
    }

    if ((s_control_region == BALL_REGION_RETURN_DRIVE) &&
        (previous_control_region != BALL_REGION_RETURN_DRIVE)) {
        s_position_integral_mm_s = 0.0f;
        s_transport_stall_since_ms = 0U;
        s_transport_stall_active = 0U;
    }

    transport_control_active =
        (((s_control_region == BALL_REGION_OUTBOUND) &&
          (absolute_position_error > BALL_RETURN_CAPTURE_ZONE_MM)) ||
         (s_control_region == BALL_REGION_RETURN_DRIVE)) ?
        1U : 0U;

    ki_gain = 0.0f;
    integral_limit_pulses = 0.0f;
    integral_update_enabled = 0U;
    desired_velocity_mm_s = 0.0f;

    if (absolute_position_error <= active_capture_zone_mm) {
        kp_gain = BALL_POSITION_KP_PULSES_PER_MM;
        kd_gain = BALL_VELOCITY_KD_PULSES_PER_MM_S;
        maximum_offset_pulses = BALL_STEPPER_HOLD_LIMIT_PULSES;

        if (s_control_region == BALL_REGION_RETURN_CAPTURE) {
            ki_gain = BALL_CAPTURE_KI_PULSES_PER_MM_S;
            integral_limit_pulses =
                BALL_CAPTURE_INTEGRAL_LIMIT_PULSES;

            if (previous_control_region !=
                BALL_REGION_RETURN_CAPTURE) {
                s_position_integral_mm_s = 0.0f;
                s_transport_stall_since_ms = 0U;
                s_transport_stall_active = 0U;
            }

            integral_update_enabled =
                (BallControl_AbsFloat(velocity_feedback) <=
                 BALL_CAPTURE_INTEGRAL_SPEED_MM_S) ?
                1U : 0U;
        }
    }
    else {
        kp_gain = BALL_POSITION_KP_PULSES_PER_MM;
        kd_gain = BALL_VELOCITY_KD_PULSES_PER_MM_S;
        maximum_offset_pulses = BALL_STEPPER_TRANSPORT_LIMIT_PULSES;

        if (s_control_region == BALL_REGION_OUTBOUND) {
            kp_gain = BALL_OUTBOUND_KP_PULSES_PER_MM;
            kd_gain = BALL_OUTBOUND_KD_PULSES_PER_MM_S;
            maximum_offset_pulses =
                BALL_STEPPER_OUTBOUND_LIMIT_PULSES;
        }
        else if (s_control_region == BALL_REGION_RETURN_DRIVE) {
            maximum_offset_pulses = BALL_STEPPER_RETURN_LIMIT_PULSES;
        }

        if (transport_control_active != 0U) {
            if (s_control_region == BALL_REGION_OUTBOUND) {
                ki_gain = BALL_OUTBOUND_KI_PULSES_PER_MM_S;
                integral_limit_pulses =
                    BALL_OUTBOUND_INTEGRAL_LIMIT_PULSES;
            }
            else {
                ki_gain = BALL_RETURN_KI_PULSES_PER_MM_S;
                integral_limit_pulses =
                    BALL_RETURN_INTEGRAL_LIMIT_PULSES;
            }

            if ((s_last_measurement_interval_ms >=
                 BALL_VELOCITY_MIN_INTERVAL_MS) &&
                (s_last_measurement_interval_ms <=
                 BALL_TASK3_FRAME_GAP_MAX_MS)) {

                if (s_transport_stall_since_ms == 0U) {
                    s_transport_stall_since_ms = now_ms;
                }

                if ((uint32_t)(now_ms -
                    s_transport_stall_since_ms) >=
                    BALL_TRANSPORT_STALL_CONFIRM_MS) {
                    s_transport_stall_active = 1U;
                }
            }
            else {
                s_transport_stall_since_ms = 0U;
                s_transport_stall_active = 0U;
            }

            integral_update_enabled =
                s_transport_stall_active;
        }
        else {
            s_transport_stall_since_ms = 0U;
            s_transport_stall_active = 0U;
        }
    }

    /*
     * 位置死区只影响步进控制输出，
     * 不影响任务3的±5mm到达判断。
     */
    if ((BallControl_AbsFloat(position_error) <
         BALL_POSITION_DEADBAND_MM) &&
        (BallControl_AbsFloat(velocity_feedback) <
         BALL_VELOCITY_DEADBAND_MM_S)) {

        position_error = 0.0f;
        velocity_feedback = 0.0f;
    }

    if (s_control_region == BALL_REGION_RETURN_CAPTURE) {
        desired_velocity_mm_s =
            BallControl_ClampFloat(
                BALL_CAPTURE_POSITION_TO_SPEED_GAIN *
                position_error,
                -BALL_CAPTURE_SPEED_LIMIT_MM_S,
                BALL_CAPTURE_SPEED_LIMIT_MM_S);
    }

    /* Only fresh K230 frames may update either stall or capture integral. */
    if ((integral_update_enabled != 0U) &&
        (ki_gain > 0.0f) &&
        (s_new_measurement_this_update != 0U) &&
        (s_last_measurement_interval_ms >=
         BALL_VELOCITY_MIN_INTERVAL_MS) &&
        (s_last_measurement_interval_ms <=
         BALL_TASK3_FRAME_GAP_MAX_MS)) {

        integral_candidate =
            s_position_integral_mm_s +
            position_error *
            ((float)s_last_measurement_interval_ms / 1000.0f);

        integral_candidate =
            BallControl_ClampFloat(
                integral_candidate,
                -integral_limit_pulses / ki_gain,
                integral_limit_pulses / ki_gain);

        if (s_control_region == BALL_REGION_RETURN_CAPTURE) {
            candidate_requested_offset =
                (float)BALL_STEPPER_DIRECTION *
                (BALL_CAPTURE_SPEED_KP_PULSES_PER_MM_S *
                 (desired_velocity_mm_s - velocity_feedback) +
                 ki_gain * integral_candidate);
        }
        else {
            candidate_requested_offset =
                (float)BALL_STEPPER_DIRECTION *
                (kp_gain * position_error +
                 ki_gain * integral_candidate -
                 kd_gain * velocity_feedback);
        }

        integral_delta_output =
            (float)BALL_STEPPER_DIRECTION *
            ki_gain *
            (integral_candidate - s_position_integral_mm_s);

        if (((candidate_requested_offset <=
              (float)maximum_offset_pulses) &&
             (candidate_requested_offset >=
              (float)-maximum_offset_pulses)) ||
            ((candidate_requested_offset >
              (float)maximum_offset_pulses) &&
             (integral_delta_output < 0.0f)) ||
            ((candidate_requested_offset <
              (float)-maximum_offset_pulses) &&
             (integral_delta_output > 0.0f))) {

            s_position_integral_mm_s = integral_candidate;
        }
    }

    if (s_control_region == BALL_REGION_RETURN_CAPTURE) {
        p_term =
            (float)BALL_STEPPER_DIRECTION *
            BALL_CAPTURE_SPEED_KP_PULSES_PER_MM_S *
            desired_velocity_mm_s;

        d_term =
            (float)BALL_STEPPER_DIRECTION *
            (-BALL_CAPTURE_SPEED_KP_PULSES_PER_MM_S *
             velocity_feedback);
    }
    else {
        p_term =
            (float)BALL_STEPPER_DIRECTION *
            kp_gain * position_error;

        d_term =
            (float)BALL_STEPPER_DIRECTION *
            (-kd_gain * velocity_feedback);
    }

    i_term =
        (float)BALL_STEPPER_DIRECTION *
        ki_gain * s_position_integral_mm_s;

    vehicle_accel_ff_pulses =
        BallControl_HoldAccelerationFeedforward(now_ms);
    requested_offset =
        p_term + i_term + d_term +
        (float)vehicle_accel_ff_pulses;

    s_last_p_output_pulses = BallControl_RoundToInt(p_term);
    s_last_i_output_pulses = BallControl_RoundToInt(i_term);
    s_last_d_output_pulses = BallControl_RoundToInt(d_term);

    if ((s_task3_state == BALL_TASK3_FINISHED) &&
        (absolute_position_error <=
         BALL_RETURN_CAPTURE_ZONE_MM)) {
        maximum_offset_pulses = BALL_STEPPER_HOLD_LIMIT_PULSES;
    }

    requested_offset_pulses =
        BallControl_ClampInt32(
            BallControl_RoundToInt(requested_offset),
            -maximum_offset_pulses,
            maximum_offset_pulses);

    /*
     * The tube needs a finite launch angle to overcome static friction.
     * Apply it unconditionally near O so one noisy K230 velocity sample cannot
     * cancel the launch.  After the ball has moved 5 mm, velocity feedback is
     * solely responsible for braking and the predictive reversal takes over.
     */
    if ((s_control_region == BALL_REGION_OUTBOUND) &&
        (BallControl_MeasuredPositionMm() <=
         BALL_OUTBOUND_START_ZONE_MM)) {

        requested_offset_pulses =
            BALL_STEPPER_DIRECTION *
            BALL_OUTBOUND_START_MIN_PULSES;
    }

    s_last_raw_pd_output_pulses = requested_offset_pulses;
    s_transport_direction_gate_active = 0U;
    position_drive_sign =
        (((float)BALL_STEPPER_DIRECTION * position_error) >= 0.0f) ?
        1 : -1;

    /* 运输区中D项可以减速，但不能把坡度翻到远离目标的方向。 */
    if (((s_control_region == BALL_REGION_OUTBOUND) &&
         (absolute_position_error > active_capture_zone_mm)) ||
        (s_control_region == BALL_REGION_RETURN_DRIVE)) {

        if (((position_drive_sign > 0) &&
             (requested_offset_pulses < 0)) ||
            ((position_drive_sign < 0) &&
             (requested_offset_pulses > 0))) {

            requested_offset_pulses = 0;
            s_transport_direction_gate_active = 1U;
        }
    }

    /*
     * The last 30 mm uses the cascaded position/speed controller above.
     * Limit its angle symmetrically; speed feedback changes the sign before
     * target crossing instead of waiting for a large position overshoot.
     */
    if (s_control_region == BALL_REGION_RETURN_CAPTURE) {
        int32_t capture_limit_pulses =
            (absolute_position_error <=
                 BALL_CAPTURE_NEAR_OVERSHOOT_MM) ?
            BALL_CAPTURE_NEAR_DRIVE_LIMIT_PULSES :
            BALL_CAPTURE_FAR_DRIVE_LIMIT_PULSES;

        requested_offset_pulses =
            BallControl_ClampInt32(
                requested_offset_pulses,
                -capture_limit_pulses,
                capture_limit_pulses);
    }

    /* 捕获区允许更快的反向角度变化，用于接近-5cm时主动制动。 */
    if ((s_control_region == BALL_REGION_OUTBOUND) &&
        (BallControl_MeasuredPositionMm() <=
         BALL_OUTBOUND_START_ZONE_MM)) {
        maximum_delta_pulses =
            BALL_OUTBOUND_LAUNCH_DELTA_PULSES;
    }
    else if ((s_control_region == BALL_REGION_OUTBOUND) &&
             (s_stepper_offset_pulses < 0) &&
             (requested_offset_pulses >
              s_stepper_offset_pulses)) {
        maximum_delta_pulses =
            BALL_OUTBOUND_RELEASE_DELTA_PULSES;
    }
    else if (((s_stepper_offset_pulses > 0) &&
         (requested_offset_pulses < 0)) ||
        ((s_stepper_offset_pulses < 0) &&
         (requested_offset_pulses > 0))) {

        maximum_delta_pulses =
            (s_control_region == BALL_REGION_RETURN_CAPTURE) ?
            BALL_RETURN_BRAKE_DELTA_PULSES :
            BALL_STEPPER_BRAKE_DELTA_PULSES;
    }
    else {
        maximum_delta_pulses = BALL_STEPPER_NORMAL_DELTA_PULSES;
    }

    s_last_pd_output_pulses =
        requested_offset_pulses;

    BallControl_CommandOffset(
        requested_offset_pulses,
        BALL_STEPPER_SPEED_RPM,
        maximum_offset_pulses,
        maximum_delta_pulses);
}


static void BallControl_RunAutoZeroHoldLoop(
    uint32_t now_ms)
{
    float position_error;
    float absolute_position_error;
    float velocity_feedback;
    float kp_gain;
    float kd_gain;
    float p_term;
    float i_term;
    float d_term;
    float integral_candidate;
    float integral_limit_mm_s;
    float requested_offset;
    float candidate_offset;
    int32_t feedforward_pulses;
    int32_t active_offset_limit_pulses;
    int32_t requested_offset_pulses;

    if ((uint32_t)(now_ms - s_last_stepper_ms) <
        BALL_STEPPER_UPDATE_MS) {
        return;
    }
    s_last_stepper_ms = now_ms;
    s_control_region = BALL_REGION_IDLE;

    position_error =
        -(float)BallControl_MeasuredPositionMm();
    s_hold_velocity_mm_s =
        BALL_HOLD_VELOCITY_FILTER_ALPHA * s_velocity_mm_s +
        (1.0f - BALL_HOLD_VELOCITY_FILTER_ALPHA) *
        s_hold_velocity_mm_s;
    velocity_feedback = s_hold_velocity_mm_s;

    if (BallControl_AbsFloat(velocity_feedback) <
        BALL_HOLD_VELOCITY_DEADBAND_MM_S) {
        velocity_feedback = 0.0f;
    }
    absolute_position_error =
        BallControl_AbsFloat(position_error);

    if (absolute_position_error <=
        BALL_HOLD_INNER_ZONE_MM) {
        kp_gain = BALL_HOLD_POSITION_KP_INNER;
        kd_gain = BALL_HOLD_VELOCITY_KD_INNER;
        active_offset_limit_pulses =
            BALL_HOLD_INNER_LIMIT_PULSES;
    }
    else if (absolute_position_error <=
             BALL_HOLD_MIDDLE_ZONE_MM) {
        kp_gain = BALL_HOLD_POSITION_KP_MIDDLE;
        kd_gain = BALL_HOLD_VELOCITY_KD_MIDDLE;
        active_offset_limit_pulses =
            BALL_HOLD_MIDDLE_LIMIT_PULSES;
    }
    else {
        kp_gain = BALL_HOLD_POSITION_KP_OUTER;
        kd_gain = BALL_HOLD_VELOCITY_KD_OUTER;
        active_offset_limit_pulses =
            BALL_HOLD_MAX_OFFSET_PULSES;
    }

    /*
     * Integrate only stable, fresh vision samples near O.  This learns the
     * small physical angle at which the real rod is level without winding up
     * while the ball is already moving quickly.
     */
    integral_limit_mm_s =
        BALL_HOLD_INTEGRAL_LIMIT_PULSES /
        BALL_HOLD_POSITION_KI_PULSES_PER_MM_S;
    if ((s_new_measurement_this_update != 0U) &&
        (s_last_measurement_interval_ms >=
         BALL_VELOCITY_MIN_INTERVAL_MS) &&
        (s_last_measurement_interval_ms <=
         BALL_TASK3_FRAME_GAP_MAX_MS) &&
        (BallControl_AbsFloat(position_error) <=
         BALL_HOLD_INTEGRAL_ZONE_MM) &&
        (BallControl_AbsFloat(velocity_feedback) <=
         BALL_HOLD_INTEGRAL_SPEED_MM_S)) {

        integral_candidate =
            s_position_integral_mm_s +
            position_error *
            ((float)s_last_measurement_interval_ms / 1000.0f);
        integral_candidate = BallControl_ClampFloat(
            integral_candidate,
            -integral_limit_mm_s,
            integral_limit_mm_s);

        candidate_offset =
            (float)BALL_STEPPER_DIRECTION *
            (kp_gain * position_error +
             BALL_HOLD_POSITION_KI_PULSES_PER_MM_S *
                 integral_candidate -
             kd_gain *
                 velocity_feedback);

        /* Simple anti-windup: accept I only while the feedback part is safe. */
        if (BallControl_AbsFloat(candidate_offset) <=
            (float)active_offset_limit_pulses) {
            s_position_integral_mm_s = integral_candidate;
        }
    }

    if ((BallControl_AbsFloat(position_error) <=
         BALL_HOLD_POSITION_DEADBAND_MM) &&
        (BallControl_AbsFloat(velocity_feedback) <=
         BALL_HOLD_VELOCITY_DEADBAND_MM_S)) {
        p_term = 0.0f;
        d_term = 0.0f;
    }
    else {
        p_term =
            (float)BALL_STEPPER_DIRECTION *
            kp_gain *
            position_error;
        d_term =
            (float)BALL_STEPPER_DIRECTION *
            (-kd_gain *
             velocity_feedback);
    }

    i_term =
        (float)BALL_STEPPER_DIRECTION *
        BALL_HOLD_POSITION_KI_PULSES_PER_MM_S *
        s_position_integral_mm_s;
    feedforward_pulses =
        BallControl_HoldAccelerationFeedforward(now_ms);
    requested_offset =
        p_term + i_term + d_term +
        (float)feedforward_pulses;
    requested_offset_pulses =
        BallControl_ClampInt32(
            BallControl_RoundToInt(requested_offset),
            -active_offset_limit_pulses,
            active_offset_limit_pulses);

    /*
     * The linkage and tube need a finite angle before the ball starts moving.
     * Apply it only when the ball is outside 3 mm and is stationary or moving
     * away from O.  Once it moves toward O, velocity damping takes over and
     * the minimum drive is removed before the centre is crossed.
     */
    if ((absolute_position_error >=
         BALL_HOLD_STATIC_DRIVE_ERROR_MM) &&
        ((position_error * velocity_feedback) <= 0.0f) &&
        (BallControl_AbsFloat(velocity_feedback) <=
         BALL_HOLD_STATIC_DRIVE_SPEED_MM_S)) {
        int32_t required_sign =
            (((float)BALL_STEPPER_DIRECTION *
              position_error) >= 0.0f) ? 1 : -1;

        if ((required_sign > 0) &&
            (requested_offset_pulses <
             BALL_HOLD_STATIC_DRIVE_PULSES)) {
            requested_offset_pulses =
                BALL_HOLD_STATIC_DRIVE_PULSES;
        }
        else if ((required_sign < 0) &&
                 (requested_offset_pulses >
                  -BALL_HOLD_STATIC_DRIVE_PULSES)) {
            requested_offset_pulses =
                -BALL_HOLD_STATIC_DRIVE_PULSES;
        }
    }

    s_last_p_output_pulses = BallControl_RoundToInt(p_term);
    s_last_i_output_pulses = BallControl_RoundToInt(i_term);
    s_last_d_output_pulses = BallControl_RoundToInt(d_term);
    s_last_raw_pd_output_pulses = requested_offset_pulses;
    s_last_pd_output_pulses = requested_offset_pulses;

    BallControl_CommandOffset(
        requested_offset_pulses,
        BALL_STEPPER_SPEED_RPM,
        active_offset_limit_pulses,
        BALL_HOLD_MAX_DELTA_PULSES);
}


#if BALL_TASK3_RELAY_ENABLE == 0U
static void BallControl_RunTask3PidLoop(
    uint32_t now_ms)
{
    float measured_position_mm;
    float position_error_mm;
    float absolute_error_mm;
    float velocity_mm_s;
    float kp_gain;
    float ki_gain;
    float kd_gain;
    float integral_candidate;
    float integral_limit_mm_s;
    float candidate_output;
    float integral_delta_output;
    float p_term;
    float i_term;
    float d_term;
    float raw_output;
    float recapture_velocity_mm_s;
    float recapture_projected_position_mm;
    float capture_velocity_mm_s;
    float capture_projected_position_mm;
    int32_t maximum_offset_pulses;
    int32_t maximum_delta_pulses;
    int32_t requested_offset_pulses;
    uint8_t capture_active;
    uint8_t integral_update_enabled;

    if ((uint32_t)(now_ms - s_last_stepper_ms) <
        BALL_STEPPER_UPDATE_MS) {
        return;
    }

    s_last_stepper_ms = now_ms;
    measured_position_mm =
        (float)BallControl_MeasuredPositionMm();

    /* A second light low-pass removes K230 one-pixel velocity spikes. */
    s_task3_pid_velocity_mm_s =
        BALL_TASK3_PID_VELOCITY_ALPHA * s_velocity_mm_s +
        (1.0f - BALL_TASK3_PID_VELOCITY_ALPHA) *
        s_task3_pid_velocity_mm_s;
    velocity_mm_s = s_task3_pid_velocity_mm_s;

    if (BallControl_AbsFloat(velocity_mm_s) <
        BALL_TASK3_PID_SPEED_DEADBAND_MM_S) {
        velocity_mm_s = 0.0f;
    }

    capture_active = 0U;
    integral_update_enabled = 0U;

    if (s_task3_state == BALL_TASK3_MOVE_POSITIVE) {
        position_error_mm =
            (float)BALL_TASK3_POSITIVE_TARGET_MM -
            measured_position_mm;
        kp_gain = BALL_TASK3_PID_OUTBOUND_KP;
        ki_gain = BALL_TASK3_PID_OUTBOUND_KI;
        kd_gain = BALL_TASK3_PID_OUTBOUND_KD;
        maximum_offset_pulses =
            BALL_TASK3_PID_OUTBOUND_LIMIT_PULSES;
        s_control_region = BALL_REGION_OUTBOUND;
    }
    else if ((s_task3_state == BALL_TASK3_MOVE_NEGATIVE) ||
             (s_task3_state == BALL_TASK3_SETTLE_NEGATIVE) ||
             (s_task3_state == BALL_TASK3_FINISHED)) {
        capture_velocity_mm_s =
            BallControl_ClampFloat(
                velocity_mm_s,
                -BALL_TASK3_PID_CAPTURE_MAX_SPEED,
                0.0f);
        capture_projected_position_mm =
            measured_position_mm +
            capture_velocity_mm_s *
            BALL_TASK3_PID_CAPTURE_LOOKAHEAD_S;

        /*
         * Return at full authority until the measured position and velocity
         * predict that the ball is about to enter the scoring window.  Once
         * latched, use a target inside the finish window and high damping.
         * Capture remains latched for the rest of the run: a rebound is
         * corrected by the bounded capture controller, never by the original
         * full-authority transport drive.
         */
        if ((s_task3_negative_capture_latched == 0U) &&
            (measured_position_mm <=
             BALL_TASK3_PID_CAPTURE_MIN_MEASURED_MM) &&
            ((capture_projected_position_mm <=
              BALL_TASK3_PID_CAPTURE_TRIGGER_MM) ||
             (measured_position_mm <=
              BALL_TASK3_NEGATIVE_FINISH_TARGET_MM))) {
            s_task3_negative_capture_latched = 1U;
            s_position_integral_mm_s = 0.0f;
        }

        if (s_task3_negative_capture_latched == 0U) {
            position_error_mm =
                BALL_TASK3_PID_NEGATIVE_DRIVE_TARGET_MM -
                measured_position_mm;
            kp_gain = BALL_TASK3_PID_RETURN_KP;
            ki_gain = BALL_TASK3_PID_RETURN_KI;
            kd_gain = BALL_TASK3_PID_RETURN_KD;
            maximum_offset_pulses =
                BALL_TASK3_PID_RETURN_LIMIT_PULSES;
            s_control_region = BALL_REGION_RETURN_DRIVE;
        }
        else {
            position_error_mm =
                BALL_TASK3_PID_NEGATIVE_HOLD_TARGET_MM -
                measured_position_mm;
            absolute_error_mm =
                BallControl_AbsFloat(position_error_mm);
            capture_active = 1U;
            kp_gain = BALL_TASK3_PID_CAPTURE_KP;
            ki_gain = BALL_TASK3_PID_CAPTURE_KI;
            kd_gain = BALL_TASK3_PID_CAPTURE_KD;
            maximum_offset_pulses =
                (absolute_error_mm <=
                 BALL_TASK3_PID_CAPTURE_INNER_ZONE_MM) ?
                BALL_TASK3_PID_CAPTURE_INNER_LIMIT :
                BALL_TASK3_PID_CAPTURE_OUTER_LIMIT;

            /*
             * Once the ball has passed the negative target, use a gentler P
             * term and stronger velocity damping for the correction toward
             * the positive side.  This prevents the recovery itself from
             * carrying the ball all the way back to about -4 cm.
             */
            if (position_error_mm > 0.0f) {
                kp_gain = BALL_TASK3_PID_RECOVERY_KP;
                kd_gain = BALL_TASK3_PID_RECOVERY_KD;
                maximum_offset_pulses =
                    (absolute_error_mm <=
                     BALL_TASK3_PID_CAPTURE_INNER_ZONE_MM) ?
                    BALL_TASK3_PID_CAPTURE_INNER_LIMIT :
                    BALL_TASK3_PID_CAPTURE_OUTER_LIMIT;
            }
            s_control_region = BALL_REGION_RETURN_CAPTURE;

            integral_update_enabled =
                ((absolute_error_mm <=
                  BALL_TASK3_PID_INTEGRAL_ZONE_MM) &&
                 (BallControl_AbsFloat(velocity_mm_s) <=
                  BALL_TASK3_PID_INTEGRAL_SPEED_MM_S)) ? 1U : 0U;
        }
    }
    else {
        return;
    }

    /* Never carry left-driving integral across the final negative target. */
    if ((capture_active != 0U) &&
        (position_error_mm >= 0.0f)) {
        s_position_integral_mm_s = 0.0f;
    }

    absolute_error_mm =
        BallControl_AbsFloat(position_error_mm);

    /*
     * The first successful trip into the negative capture window identifies
     * the real mechanism bias.  Keep a little less than the breakaway angle
     * as the physical-level feedforward, then regulate around that value.
     * This latch prevents every rebound to -3 cm from restarting the original
     * 90-to-140-pulse launch sequence.
     */
    if ((capture_active != 0U) &&
        (s_task3_redrive_completed == 0U) &&
        (measured_position_mm <=
         BALL_TASK3_PID_REDRIVE_THRESHOLD_MM)) {
        int32_t hold_bias_magnitude =
            s_task3_redrive_peak_pulses -
            BALL_TASK3_PID_HOLD_BIAS_BACKOFF_PULSES;

        hold_bias_magnitude =
            BallControl_ClampInt32(
                hold_bias_magnitude,
                BALL_TASK3_PID_HOLD_BIAS_MIN_PULSES,
                BALL_TASK3_PID_HOLD_BIAS_MAX_PULSES);
        s_task3_hold_bias_pulses =
            (BALL_STEPPER_DIRECTION * -1) *
            hold_bias_magnitude;
        s_task3_redrive_floor_pulses = 0;
        s_task3_redrive_completed = 1U;
    }

    if ((capture_active != 0U) &&
        (s_task3_redrive_completed != 0U)) {
        maximum_offset_pulses =
            BALL_TASK3_PID_CAPTURE_OUTER_LIMIT;
    }

    /*
     * Integral is a final-position level trim, not a transport drive.  It is
     * updated only by fresh, regularly timed K230 samples in the capture zone.
     */
    if ((capture_active != 0U) &&
        (integral_update_enabled != 0U) &&
        (s_new_measurement_this_update != 0U) &&
        (s_last_measurement_interval_ms >=
         BALL_VELOCITY_MIN_INTERVAL_MS) &&
        (s_last_measurement_interval_ms <=
         BALL_TASK3_FRAME_GAP_MAX_MS)) {
        integral_limit_mm_s =
            BALL_TASK3_PID_INTEGRAL_LIMIT_PULSES /
            ki_gain;
        integral_candidate =
            s_position_integral_mm_s +
            position_error_mm *
            ((float)s_last_measurement_interval_ms / 1000.0f);
        integral_candidate =
            BallControl_ClampFloat(
                integral_candidate,
                -integral_limit_mm_s,
                integral_limit_mm_s);

        candidate_output =
            (float)BALL_STEPPER_DIRECTION *
            (kp_gain * position_error_mm +
             ki_gain * integral_candidate -
             kd_gain * velocity_mm_s);
        integral_delta_output =
            (float)BALL_STEPPER_DIRECTION *
            ki_gain *
            (integral_candidate - s_position_integral_mm_s);

        /* Conditional integration prevents wind-up at the angle limit. */
        if ((BallControl_AbsFloat(candidate_output) <=
             (float)maximum_offset_pulses) ||
            ((candidate_output >
              (float)maximum_offset_pulses) &&
             (integral_delta_output < 0.0f)) ||
            ((candidate_output <
              (float)-maximum_offset_pulses) &&
             (integral_delta_output > 0.0f))) {
            s_position_integral_mm_s = integral_candidate;
        }
    }

    if ((absolute_error_mm <=
         BALL_TASK3_PID_POSITION_DEADBAND_MM) &&
        (BallControl_AbsFloat(velocity_mm_s) <=
         BALL_TASK3_PID_SPEED_DEADBAND_MM_S)) {
        p_term = 0.0f;
        d_term = 0.0f;
    }
    else {
        p_term =
            (float)BALL_STEPPER_DIRECTION *
            kp_gain * position_error_mm;
        d_term =
            (float)BALL_STEPPER_DIRECTION *
            (-kd_gain * velocity_mm_s);
    }

    i_term =
        (float)BALL_STEPPER_DIRECTION *
        ki_gain * s_position_integral_mm_s;
    raw_output = p_term + i_term + d_term;
    if ((capture_active != 0U) &&
        (s_task3_redrive_completed != 0U)) {
        raw_output += (float)s_task3_hold_bias_pulses;
    }
    requested_offset_pulses =
        BallControl_ClampInt32(
            BallControl_RoundToInt(raw_output),
            -maximum_offset_pulses,
            maximum_offset_pulses);

    /*
     * Also cap a velocity-predicted positive correction that starts just
     * before the measured position crosses the target.  The opposite command
     * toward -5 cm keeps its full authority.
     */
    if ((s_task3_state != BALL_TASK3_MOVE_POSITIVE) &&
        ((((BALL_STEPPER_DIRECTION) < 0) &&
          (requested_offset_pulses < 0)) ||
         (((BALL_STEPPER_DIRECTION) > 0) &&
          (requested_offset_pulses > 0)))) {
        int32_t recovery_limit;

        /*
         * While approaching the target at speed, return rapidly to physical
         * level but do not cross level into a rightward tilt.  Friction and
         * the level rod remove momentum without launching a new half-cycle.
         * After crossing the target, keep the rod level and use no active
         * rightward recovery; the left-driving controller resumes only if the
         * measured ball returns to the right side of the hold target.
         */
        if ((s_task3_negative_capture_latched != 0U) &&
            (position_error_mm < 0.0f) &&
            (velocity_mm_s <
             -BALL_TASK3_PID_INTEGRAL_SPEED_MM_S)) {
            recovery_limit =
                BALL_TASK3_PID_PREDICTIVE_BRAKE_LIMIT;
        }
        else {
            recovery_limit =
                (absolute_error_mm <=
                 BALL_TASK3_PID_CAPTURE_INNER_ZONE_MM) ?
                BALL_TASK3_PID_RECOVERY_INNER_LIMIT :
                BALL_TASK3_PID_RECOVERY_OUTER_LIMIT;
        }

        if (requested_offset_pulses < -recovery_limit) {
            requested_offset_pulses = -recovery_limit;
        }
        else if (requested_offset_pulses > recovery_limit) {
            requested_offset_pulses = recovery_limit;
        }
    }

    /*
     * A ball that has crossed left of the target can coast back to the right
     * even though the rod itself is no longer tilted right.  Brake that
     * rightward velocity early with a bounded left tilt, before the position
     * has travelled all the way back through the target.
     */
    if ((s_task3_negative_capture_latched != 0U) &&
        (s_task3_redrive_completed == 0U) &&
        (position_error_mm > 0.0f) &&
        (velocity_mm_s >
         BALL_TASK3_PID_RIGHT_MOTION_BRAKE_SPEED)) {
        int32_t left_brake_sign =
            BALL_STEPPER_DIRECTION * -1;
        int32_t brake_magnitude =
            BallControl_RoundToInt(
                (float)BALL_TASK3_PID_RIGHT_MOTION_BRAKE_MIN +
                BALL_TASK3_PID_RIGHT_MOTION_BRAKE_GAIN *
                (velocity_mm_s -
                 BALL_TASK3_PID_RIGHT_MOTION_BRAKE_SPEED));

        brake_magnitude =
            BallControl_ClampInt32(
                brake_magnitude,
                BALL_TASK3_PID_RIGHT_MOTION_BRAKE_MIN,
                BALL_TASK3_PID_RIGHT_MOTION_BRAKE_LIMIT);

        /* Velocity braking may temporarily exceed the quiet inner-zone cap. */
        if (maximum_offset_pulses < brake_magnitude) {
            maximum_offset_pulses = brake_magnitude;
        }

        if (left_brake_sign > 0) {
            requested_offset_pulses = brake_magnitude;
        }
        else {
            requested_offset_pulses = -brake_magnitude;
        }
    }

    /*
     * If the ball is still to the right of the final window, adapt the
     * minimum left-driving tilt to the real static-friction requirement.
     * A stopped ball ramps from 90 pulses toward 140 pulses.  As soon as the
     * ball makes useful leftward progress, withdraw that extra floor twice
     * as fast so the normal velocity damping can capture without launching
     * another large oscillation.
     */
    if ((s_task3_negative_capture_latched != 0U) &&
        (s_task3_redrive_completed == 0U) &&
        (measured_position_mm >
         BALL_TASK3_PID_REDRIVE_THRESHOLD_MM)) {
        int32_t redrive_sign =
            BALL_STEPPER_DIRECTION * -1;

        if (velocity_mm_s >=
            BALL_TASK3_PID_REDRIVE_SPEED_MM_S) {
            if (s_task3_redrive_floor_pulses <
                BALL_TASK3_PID_REDRIVE_MIN_PULSES) {
                s_task3_redrive_floor_pulses =
                    BALL_TASK3_PID_REDRIVE_MIN_PULSES;
            }
            else {
                s_task3_redrive_floor_pulses +=
                    BALL_TASK3_PID_REDRIVE_RAMP_PULSES;
                if (s_task3_redrive_floor_pulses >
                    BALL_TASK3_PID_REDRIVE_MAX_PULSES) {
                    s_task3_redrive_floor_pulses =
                        BALL_TASK3_PID_REDRIVE_MAX_PULSES;
                }
            }

            if (s_task3_redrive_peak_pulses <
                s_task3_redrive_floor_pulses) {
                s_task3_redrive_peak_pulses =
                    s_task3_redrive_floor_pulses;
            }
        }
        else if (s_task3_redrive_floor_pulses > 0) {
            s_task3_redrive_floor_pulses -=
                BALL_TASK3_PID_REDRIVE_RELEASE_PULSES;
            if (s_task3_redrive_floor_pulses < 0) {
                s_task3_redrive_floor_pulses = 0;
            }
        }

        if (s_task3_redrive_floor_pulses > 0) {
            if (maximum_offset_pulses <
                s_task3_redrive_floor_pulses) {
                maximum_offset_pulses =
                    s_task3_redrive_floor_pulses;
            }

            if ((redrive_sign > 0) &&
                (requested_offset_pulses <
                 s_task3_redrive_floor_pulses)) {
                requested_offset_pulses =
                    s_task3_redrive_floor_pulses;
            }
            else if ((redrive_sign < 0) &&
                     (requested_offset_pulses >
                      -s_task3_redrive_floor_pulses)) {
                requested_offset_pulses =
                    -s_task3_redrive_floor_pulses;
            }
        }
    }
    else {
        s_task3_redrive_floor_pulses = 0;
    }

    /*
     * After the first arrival, all corrections are centred on the learned
     * physical-level bias.  Limiting both sides of that bias removes the
     * former 0 <-> +90 pulse step that sustained the -3 cm to -6 cm cycle.
     */
    if ((capture_active != 0U) &&
        (s_task3_redrive_completed != 0U)) {
        int32_t hold_minimum =
            s_task3_hold_bias_pulses -
            BALL_TASK3_PID_HOLD_BRAKE_RANGE_PULSES;
        int32_t hold_maximum =
            s_task3_hold_bias_pulses +
            BALL_TASK3_PID_HOLD_DRIVE_RANGE_PULSES;

        requested_offset_pulses =
            BallControl_ClampInt32(
                requested_offset_pulses,
                hold_minimum,
                hold_maximum);
    }

    /*
     * The filtered velocity can still indicate leftward motion briefly after
     * the ball has already started rolling right.  Real position therefore
     * has priority while the ball is still to the right of -5 cm.  The short
     * velocity projection releases the minimum drive early when a fast ball
     * is already expected to cross the target, so the D term can brake it
     * without waiting for a large negative overshoot.
     */
    recapture_velocity_mm_s =
        BallControl_ClampFloat(
            velocity_mm_s,
            -BALL_TASK3_PID_RECAPTURE_MAX_SPEED,
            0.0f);
    recapture_projected_position_mm =
        measured_position_mm +
        recapture_velocity_mm_s *
        BALL_TASK3_PID_RECAPTURE_LOOKAHEAD_S;

    if ((s_task3_state != BALL_TASK3_MOVE_POSITIVE) &&
        (s_task3_negative_capture_latched == 0U) &&
        (measured_position_mm >
         BALL_TASK3_PID_RIGHT_RECAPTURE_MM) &&
        (recapture_projected_position_mm >
         BALL_TASK3_PID_RECAPTURE_PREDICT_MM)) {
        int32_t negative_drive_sign =
            BALL_STEPPER_DIRECTION * -1;

        if ((negative_drive_sign > 0) &&
            (requested_offset_pulses <
             BALL_TASK3_PID_RECAPTURE_MIN_PULSES)) {
            requested_offset_pulses =
                BALL_TASK3_PID_RECAPTURE_MIN_PULSES;
        }
        else if ((negative_drive_sign < 0) &&
                 (requested_offset_pulses >
                  -BALL_TASK3_PID_RECAPTURE_MIN_PULSES)) {
            requested_offset_pulses =
                -BALL_TASK3_PID_RECAPTURE_MIN_PULSES;
        }
    }

    if ((s_task3_state == BALL_TASK3_MOVE_POSITIVE) &&
        (BallControl_MeasuredPositionMm() <=
         BALL_TASK3_PID_LAUNCH_ZONE_MM)) {
        maximum_delta_pulses =
            BALL_TASK3_PID_LAUNCH_DELTA_PULSES;
    }
    else if ((s_task3_negative_capture_latched != 0U) &&
             (s_task3_redrive_completed == 0U) &&
             (measured_position_mm >
              BALL_TASK3_PID_REDRIVE_THRESHOLD_MM) &&
             (velocity_mm_s >=
              BALL_TASK3_PID_REDRIVE_SPEED_MM_S)) {
        maximum_delta_pulses =
            BALL_TASK3_PID_PREDICTIVE_LEVEL_DELTA;
    }
    else if ((s_task3_negative_capture_latched != 0U) &&
             (velocity_mm_s >
              BALL_TASK3_PID_RIGHT_MOTION_BRAKE_SPEED)) {
        maximum_delta_pulses =
            BALL_TASK3_PID_PREDICTIVE_LEVEL_DELTA;
    }
    else if ((s_task3_negative_capture_latched != 0U) &&
             (requested_offset_pulses == 0) &&
             (BallControl_AbsFloat(velocity_mm_s) >
              BALL_TASK3_PID_INTEGRAL_SPEED_MM_S)) {
        maximum_delta_pulses =
            BALL_TASK3_PID_PREDICTIVE_LEVEL_DELTA;
    }
    else if (((s_stepper_offset_pulses > 0) &&
              (requested_offset_pulses < 0)) ||
             ((s_stepper_offset_pulses < 0) &&
              (requested_offset_pulses > 0))) {
        maximum_delta_pulses =
            BALL_TASK3_PID_REVERSE_DELTA_PULSES;
    }
    else {
        maximum_delta_pulses =
            BALL_TASK3_PID_NORMAL_DELTA_PULSES;
    }

    s_last_p_output_pulses = BallControl_RoundToInt(p_term);
    s_last_i_output_pulses = BallControl_RoundToInt(i_term);
    s_last_d_output_pulses = BallControl_RoundToInt(d_term);
    s_last_raw_pd_output_pulses = BallControl_RoundToInt(raw_output);
    s_last_pd_output_pulses = requested_offset_pulses;
    s_transport_direction_gate_active = 0U;
    s_transport_stall_active = 0U;

    BallControl_CommandOffset(
        requested_offset_pulses,
        BALL_STEPPER_SPEED_RPM,
        maximum_offset_pulses,
        maximum_delta_pulses);
}
#else
static void BallControl_RunTask3RelayLoop(
    uint32_t now_ms)
{
    float measured_position_mm;
    float measured_velocity_mm_s;
    float relay_target_position_mm;
    float relay_lookahead_s;
    float predicted_position_mm;
    float predicted_error_mm;
    float actual_error_mm;
    float absolute_predicted_error_mm;
    float absolute_actual_error_mm;
    int32_t requested_offset_pulses;
    int32_t relay_pulses;
    int32_t bias_step;
    int32_t output_limit_pulses;
    int32_t output_delta_limit_pulses;
    int32_t positive_brake_limit_pulses;
    uint8_t capture_active;

    if ((uint32_t)(now_ms - s_last_stepper_ms) <
        BALL_STEPPER_UPDATE_MS) {
        return;
    }

    s_last_stepper_ms = now_ms;
    measured_position_mm =
        (float)BallControl_MeasuredPositionMm();
    measured_velocity_mm_s = s_velocity_mm_s;

    if (BallControl_AbsFloat(measured_velocity_mm_s) <
        BALL_RELAY_VELOCITY_DEADBAND_MM_S) {
        measured_velocity_mm_s = 0.0f;
    }

    measured_velocity_mm_s =
        BallControl_ClampFloat(
            measured_velocity_mm_s,
            -BALL_RELAY_MAX_PREDICT_SPEED_MM_S,
            BALL_RELAY_MAX_PREDICT_SPEED_MM_S);

    if (s_task3_state == BALL_TASK3_MOVE_POSITIVE) {
        relay_target_position_mm =
            (float)BALL_TASK3_POSITIVE_TARGET_MM;
        relay_lookahead_s =
            BALL_RELAY_OUTBOUND_LOOKAHEAD_S;
    }
    else {
        relay_target_position_mm =
            BALL_TASK3_NEGATIVE_CONTROL_TARGET_MM;
        relay_lookahead_s =
            BALL_RELAY_RETURN_LOOKAHEAD_S;
    }

    predicted_position_mm =
        measured_position_mm +
        measured_velocity_mm_s *
        relay_lookahead_s;

    actual_error_mm =
        relay_target_position_mm -
        measured_position_mm;
    predicted_error_mm =
        relay_target_position_mm -
        predicted_position_mm;
    absolute_actual_error_mm =
        BallControl_AbsFloat(actual_error_mm);
    absolute_predicted_error_mm =
        BallControl_AbsFloat(predicted_error_mm);

    capture_active =
        (((s_task3_state == BALL_TASK3_MOVE_NEGATIVE) ||
          (s_task3_state == BALL_TASK3_SETTLE_NEGATIVE) ||
          (s_task3_state == BALL_TASK3_FINISHED)) &&
         (absolute_actual_error_mm <=
          BALL_RELAY_CAPTURE_ZONE_MM)) ? 1U : 0U;

    if (s_task3_state == BALL_TASK3_MOVE_POSITIVE) {
        s_control_region = BALL_REGION_OUTBOUND;
    }
    else if (capture_active != 0U) {
        s_control_region = BALL_REGION_RETURN_CAPTURE;
    }
    else {
        s_control_region = BALL_REGION_RETURN_DRIVE;
    }

    /* Learn only the small physical level bias after the ball is nearly still. */
    if ((capture_active != 0U) &&
        (absolute_actual_error_mm >
         BALL_RELAY_BIAS_LEARN_ERROR_MM) &&
        (BallControl_AbsFloat(measured_velocity_mm_s) <=
         BALL_RELAY_BIAS_LEARN_SPEED_MM_S)) {

        if (s_task3_bias_learn_since_ms == 0U) {
            s_task3_bias_learn_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms -
                 s_task3_bias_learn_since_ms) >=
                 BALL_RELAY_BIAS_LEARN_PERIOD_MS) {

            bias_step =
                BALL_STEPPER_DIRECTION *
                ((actual_error_mm > 0.0f) ?
                 BALL_RELAY_BIAS_LEARN_STEP_PULSES :
                 -BALL_RELAY_BIAS_LEARN_STEP_PULSES);
            s_task3_hold_bias_pulses =
                BallControl_ClampInt32(
                    s_task3_hold_bias_pulses + bias_step,
                    -BALL_RELAY_BIAS_LIMIT_PULSES,
                    BALL_RELAY_BIAS_LIMIT_PULSES);
            s_task3_bias_learn_since_ms = now_ms;
        }
    }
    else {
        s_task3_bias_learn_since_ms = 0U;
    }

    relay_pulses = 0;
    output_limit_pulses =
        BALL_RELAY_MAX_OFFSET_PULSES;
    output_delta_limit_pulses =
        (s_task3_state == BALL_TASK3_MOVE_POSITIVE) ?
        BALL_RELAY_TRANSPORT_DELTA_PULSES :
        BALL_RELAY_RETURN_DELTA_PULSES;

    if (capture_active != 0U) {
        /*
         * Continuous predictive damping.  The old pulse/coast controller
         * intentionally returned to level every other vision frame, which
         * made the last centimetre take several seconds.  Position plus the
         * measured ball velocity now predicts the landing point, so the same
         * continuous command accelerates toward the target and changes sign
         * early enough to brake before overshoot.
         */
        output_limit_pulses =
            (absolute_actual_error_mm <=
             BALL_RELAY_CAPTURE_INNER_ZONE_MM) ?
            BALL_RELAY_CAPTURE_INNER_LIMIT_PULSES :
            BALL_RELAY_CAPTURE_OUTER_LIMIT_PULSES;
        output_delta_limit_pulses =
            (absolute_actual_error_mm <=
             BALL_RELAY_CAPTURE_INNER_ZONE_MM) ?
            BALL_RELAY_CAPTURE_INNER_DELTA_PULSES :
            BALL_RELAY_CAPTURE_OUTER_DELTA_PULSES;

        if (absolute_predicted_error_mm >
            BALL_RELAY_CAPTURE_DEADBAND_MM) {
            relay_pulses =
                BallControl_RoundToInt(
                    BALL_RELAY_CAPTURE_PREDICT_KP *
                    absolute_predicted_error_mm);
            relay_pulses =
                BallControl_ClampInt32(
                    relay_pulses,
                    0,
                    output_limit_pulses);

        }
    }
    else {
        if (absolute_predicted_error_mm >
            BALL_RELAY_TRANSPORT_FAR_ERROR_MM) {
            relay_pulses =
                (s_task3_state ==
                 BALL_TASK3_MOVE_POSITIVE) ?
                BALL_RELAY_TRANSPORT_FAR_PULSES :
                BALL_RELAY_RETURN_FAR_PULSES;
        }
        else if (absolute_predicted_error_mm >
                 BALL_RELAY_TRANSPORT_MID_ERROR_MM) {
            relay_pulses =
                (s_task3_state ==
                 BALL_TASK3_MOVE_POSITIVE) ?
                BALL_RELAY_TRANSPORT_MID_PULSES :
                BALL_RELAY_RETURN_MID_PULSES;
        }
        else if (absolute_predicted_error_mm >
                 BALL_RELAY_CAPTURE_DEADBAND_MM) {
            relay_pulses =
                (s_task3_state ==
                 BALL_TASK3_MOVE_POSITIVE) ?
                BALL_RELAY_TRANSPORT_NEAR_PULSES :
                BALL_RELAY_RETURN_NEAR_PULSES;
        }
    }

    if (relay_pulses == 0) {
        requested_offset_pulses =
            (capture_active != 0U) ?
            s_task3_hold_bias_pulses : 0;
    }
    else {
        requested_offset_pulses =
            BALL_STEPPER_DIRECTION *
            ((predicted_error_mm > 0.0f) ?
             relay_pulses : -relay_pulses);

        if (capture_active != 0U) {
            requested_offset_pulses +=
                s_task3_hold_bias_pulses;
        }
    }

    /*
     * K230 position jitter can create a false velocity large enough to predict
     * that the ball will reach -5 cm while it is physically still near -1 cm.
     * Outside the last 25 mm, real position therefore has authority: velocity
     * prediction may reduce the drive but may not level the rod or reverse it
     * away from the target.
     */
    s_transport_direction_gate_active = 0U;
    if ((s_task3_state != BALL_TASK3_MOVE_POSITIVE) &&
        (absolute_actual_error_mm >
         BALL_RELAY_RETURN_POSITION_GATE_MM)) {
        int32_t required_drive_sign =
            BALL_STEPPER_DIRECTION *
            ((actual_error_mm > 0.0f) ? 1 : -1);

        if ((required_drive_sign > 0) &&
            (requested_offset_pulses <
             BALL_RELAY_RETURN_MIN_DRIVE_PULSES)) {
            requested_offset_pulses =
                BALL_RELAY_RETURN_MIN_DRIVE_PULSES;
            s_transport_direction_gate_active = 1U;
        }
        else if ((required_drive_sign < 0) &&
                 (requested_offset_pulses >
                  -BALL_RELAY_RETURN_MIN_DRIVE_PULSES)) {
            requested_offset_pulses =
                -BALL_RELAY_RETURN_MIN_DRIVE_PULSES;
            s_transport_direction_gate_active = 1U;
        }
    }

    /*
     * Mode 2 uses a strong command to carry the ball from +5 cm toward the
     * negative end.  Once prediction asks for the opposite (positive-position)
     * correction, however, the same 68-pulse authority pushes the ball too far
     * back to the right and adds another long settling cycle.  Keep the early
     * predictive braking, but make that one direction asymmetric: 20 pulses in
     * the outer capture zone and 10 pulses in the last 10 mm.  The drive toward
     * -5 cm, the outbound leg and the learned level bias are otherwise intact.
     */
    if ((s_task3_state != BALL_TASK3_MOVE_POSITIVE) &&
        ((((BALL_STEPPER_DIRECTION) < 0) &&
          (requested_offset_pulses < 0)) ||
         (((BALL_STEPPER_DIRECTION) > 0) &&
          (requested_offset_pulses > 0)))) {

        positive_brake_limit_pulses =
            (absolute_actual_error_mm <=
             BALL_RELAY_CAPTURE_INNER_ZONE_MM) ?
            BALL_RELAY_POSITIVE_BRAKE_INNER_PULSES :
            BALL_RELAY_POSITIVE_BRAKE_OUTER_PULSES;

        if (requested_offset_pulses <
            -positive_brake_limit_pulses) {
            requested_offset_pulses =
                -positive_brake_limit_pulses;
        }
        else if (requested_offset_pulses >
                 positive_brake_limit_pulses) {
            requested_offset_pulses =
                positive_brake_limit_pulses;
        }
    }

    /*
     * After the positive-position correction has removed the negative-going
     * momentum, take control back toward -5 cm without waiting for the learned
     * bias to unwind.  This small 24-pulse floor is enabled only when the ball
     * is still more than 12 mm to the right of the relay control target and is
     * no longer moving left faster than 8 mm/s.  It therefore cannot fight the
     * normal negative-going transport or its predictive brake.
     */
    if ((s_task3_state != BALL_TASK3_MOVE_POSITIVE) &&
        (actual_error_mm <
         -BALL_RELAY_RETURN_REACQUIRE_ERROR_MM) &&
        (measured_velocity_mm_s >=
         BALL_RELAY_RETURN_REACQUIRE_SPEED_MM_S)) {
        int32_t negative_drive_sign =
            BALL_STEPPER_DIRECTION * -1;

        if ((negative_drive_sign > 0) &&
            (requested_offset_pulses <
             BALL_RELAY_RETURN_REACQUIRE_PULSES)) {
            requested_offset_pulses =
                BALL_RELAY_RETURN_REACQUIRE_PULSES;
        }
        else if ((negative_drive_sign < 0) &&
                 (requested_offset_pulses >
                  -BALL_RELAY_RETURN_REACQUIRE_PULSES)) {
            requested_offset_pulses =
                -BALL_RELAY_RETURN_REACQUIRE_PULSES;
        }
    }

    requested_offset_pulses =
        BallControl_ClampInt32(
            requested_offset_pulses,
            -output_limit_pulses,
            output_limit_pulses);

    s_last_p_output_pulses =
        requested_offset_pulses -
        s_task3_hold_bias_pulses;
    s_last_i_output_pulses =
        s_task3_hold_bias_pulses;
    s_last_d_output_pulses = 0;
    s_last_raw_pd_output_pulses =
        requested_offset_pulses;
    s_last_pd_output_pulses =
        requested_offset_pulses;

    BallControl_CommandOffset(
        requested_offset_pulses,
        BALL_STEPPER_SPEED_RPM,
        BALL_RELAY_MAX_OFFSET_PULSES,
        output_delta_limit_pulses);
}
#endif


static void BallControl_ReturnToLogicalCenter(
    uint32_t now_ms)
{
    if ((uint32_t)(
            now_ms -
            s_last_stepper_ms
        ) < BALL_STEPPER_UPDATE_MS) {

        return;
    }

    s_last_stepper_ms = now_ms;

    BallControl_CommandOffset(
        0,
        BALL_STEPPER_CENTER_SPEED_RPM,
        BALL_STEPPER_TRANSPORT_LIMIT_PULSES,
        BALL_STEPPER_BRAKE_DELTA_PULSES);
}

static bool BallControl_CheckDirectionFault(uint32_t now_ms)
{
    int32_t current_error_mm;

    if (s_direction_check_active == 0U) {
        return false;
    }

    if (s_task3_state != BALL_TASK3_MOVE_POSITIVE) {
        s_direction_check_active = 0U;
        return false;
    }

    if ((uint32_t)(now_ms - s_direction_check_start_ms) <
        BALL_DIRECTION_CHECK_MS) {
        return false;
    }

    s_direction_check_active = 0U;
    current_error_mm =
        BallControl_AbsInt32(
            (int32_t)s_target_position_mm -
            (int32_t)BallControl_MeasuredPositionMm());

    if (current_error_mm <=
        (s_direction_initial_error_mm +
         BALL_DIRECTION_ERROR_INCREASE_MM)) {
        return false;
    }

    s_direction_fault_latched = 1U;
    s_task3_started = 0U;
    s_last_pd_output_pulses = 0;
    s_state = BALL_CONTROL_DIRECTION_FAULT;
    BallControl_ReturnToLogicalCenter(now_ms);
    return true;
}

static void BallControl_DebugPrint(uint32_t now_ms)
{
#if BALL_DEBUG_PRINT_ENABLE == 1U
    int16_t position_mm =
        (s_have_measurement != 0U) ?
        BallControl_MeasuredPositionMm() : 0;
    int32_t error_mm =
        (int32_t)s_target_position_mm -
        (int32_t)position_mm;
    uint32_t settle_ms =
        (s_task3_condition_active != 0U) ?
        (uint32_t)(now_ms - s_task3_condition_since_ms) :
        0U;

    if ((uint32_t)(now_ms - s_last_debug_ms) <
        BALL_DEBUG_PRINT_PERIOD_MS) {
        return;
    }

    s_last_debug_ms = now_ms;
    printf(
        "%lu,%u,%u,%u,%d,%d,%d,%ld,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%lu\r\n",
        (unsigned long)now_ms,
        (unsigned int)s_state,
        (unsigned int)s_task3_state,
        (unsigned int)s_control_region,
        (int)s_target_position_mm,
        (int)position_mm,
        (int)s_velocity_mm_s,
        (long)error_mm,
        (unsigned int)s_transport_stall_active,
        (long)s_last_p_output_pulses,
        (long)s_last_i_output_pulses,
        (long)s_last_d_output_pulses,
        (long)s_last_raw_pd_output_pulses,
        (long)s_last_pd_output_pulses,
        (long)s_stepper_offset_pulses,
        (unsigned int)s_transport_direction_gate_active,
        (unsigned int)s_invalid_frame_count,
        (unsigned int)s_vision_hold_active,
        (unsigned int)s_task3_late,
        (unsigned long)settle_ms);
#else
    (void)now_ms;
#endif
}


/*
 * ============================================================
 * 对外接口
 * ============================================================
 */

void BallControl_Init(void)
{
    uint32_t now_ms =
        Board_GetMillis();

#if BALL_DEBUG_PRINT_ENABLE == 1U
    Board_EnableProgrammingUartTx();
#endif
    K230_UART_Init();
    StepperEmm_Init();

    s_state =
        BALL_CONTROL_STOPPED;

    s_task3_state =
        BALL_TASK3_DISABLED;

    s_message.sequence = 0U;
    s_message.status = K230_BALL_LOST;
    s_message.position_mm = 0;
    s_message.error_x_px = 0;
    s_message.error_y_px = 0;
    s_message.confidence_permille = 0U;
    s_message.center_x_px = -1;
    s_message.center_y_px = -1;

    s_last_valid_ms = now_ms;
    s_last_measurement_interval_ms = 0U;
    s_last_stepper_ms = now_ms;
    s_last_display_ms = now_ms;
    s_last_debug_ms = now_ms;
    s_direction_check_start_ms = 0U;

    s_task3_condition_since_ms = 0U;
    s_task3_condition_last_frame_ms = 0U;
    s_task3_start_ms = 0U;
    s_task3_elapsed_ms = 0U;
    s_transport_stall_since_ms = 0U;
    s_stepper_boot_start_ms = now_ms;
    s_stepper_last_zero_ms = 0U;

    s_previous_position_mm = 0;
    s_target_position_mm = 0;
    s_task3_origin_sensor_mm = 0;
    s_stepper_offset_pulses = 0;
    s_last_raw_pd_output_pulses = 0;
    s_last_pd_output_pulses = 0;
    s_last_p_output_pulses = 0;
    s_last_i_output_pulses = 0;
    s_last_d_output_pulses = 0;
    s_direction_initial_error_mm = 0;
    s_task3_hold_bias_pulses = 0;
    s_task3_redrive_floor_pulses = 0;
    s_task3_redrive_peak_pulses = 0;

    s_velocity_mm_s = 0.0f;
    s_position_integral_mm_s = 0.0f;

    s_have_measurement = 0U;
    s_have_ever_received = 0U;
    s_previous_running = 0U;
    s_stepper_enabled = 0U;
    s_stepper_fault_latched = 0U;
    s_last_stepper_reply = 0U;

    s_display_enabled = 1U;
    s_rod_calibration_enabled = 0U;

    s_task3_enabled = 0U;
    s_task3_started = 0U;
    s_auto_zero_hold_enabled = 0U;
    s_auto_zero_hold_pending = 0U;
    s_task3_condition_active = 0U;
    s_task3_origin_valid = 0U;
    s_direction_check_active = 0U;
    s_direction_fault_latched = 0U;
    s_new_measurement_this_update = 0U;
    s_vision_hold_active = 0U;
    s_invalid_frame_count = 0U;
    s_transport_direction_gate_active = 0U;
    s_transport_stall_active = 0U;
    s_stepper_zero_attempts = 0U;
    s_stepper_zero_done = 0U;
    s_stepper_boot_enable_sent = 0U;
    s_boot_home_calibration_requested = 0U;
    s_task3_late = 0U;
    s_outbound_switch_candidate_frames = 0U;
    s_task3_negative_capture_latched = 0U;
    s_task3_redrive_completed = 0U;
    s_control_region = BALL_REGION_IDLE;
    s_task3_bias_learn_since_ms = 0U;

    s_hold_ff_last_ms = now_ms;
    s_hold_ff_previous_speed_mm_s = 0.0f;
    s_hold_ff_acceleration_mm_s2 = 0.0f;
    s_hold_ff_pulses = 0;
    s_hold_velocity_mm_s = 0.0f;

    s_last_ball_sequence = 0U;
    s_have_ball_sequence = 0U;

    s_angle_sequence = 0U;
    s_last_angle_send_ms = now_ms;

    BallControl_UpdateDisplay(
        now_ms,
        1U);
}


void BallControl_SetBootHomeCalibration(
    bool calibrate_current_position)
{
    s_boot_home_calibration_requested =
        calibrate_current_position ? 1U : 0U;
    s_stepper_zero_attempts = 0U;
    s_stepper_zero_done = 0U;
    s_stepper_boot_enable_sent = 0U;
    s_stepper_boot_start_ms = Board_GetMillis();
    s_stepper_last_zero_ms = 0U;
}


bool BallControl_RegisterCurrentHome(void)
{
    uint32_t now_ms;

    if ((Flag_Stop == 0) ||
        (s_stepper_fault_latched != 0U)) {
        return false;
    }

    now_ms = Board_GetMillis();
    (void)StepperEmm_Stop(
        STEPPER_EMM_DEFAULT_ADDRESS);
    StepperEmm_ClearRx();

    s_boot_home_calibration_requested = 1U;
    s_stepper_zero_attempts = 0U;
    s_stepper_zero_done = 0U;
    s_stepper_boot_enable_sent = 0U;
    s_stepper_boot_start_ms = now_ms;
    s_stepper_last_zero_ms = 0U;
    s_stepper_enabled = 0U;
    s_stepper_offset_pulses = 0;
    s_state = BALL_CONTROL_STOPPED;
    s_task3_started = 0U;
    BallControl_ResetPidMemory();
    BallControl_Task3ResetCondition();
    BallControl_UpdateDisplay(now_ms, 1U);
    return true;
}


void BallControl_Update(void)
{
    K230_BallMessage incoming;
    K230_BallMessage latest_incoming;
    uint8_t have_latest_incoming = 0U;

    uint32_t now_ms =
        Board_GetMillis();

    uint8_t running =
        (Flag_Stop == 0) ?
        1U :
        0U;

    s_new_measurement_this_update = 0U;

    /* 视觉短时丢失不暂停题目计时。 */
    BallControl_Task3UpdateTime(now_ms);

    /*
     * 本模块不再清零：
     *
     * MotorA.Target_Encoder
     * MotorB.Target_Encoder
     *
     * 底盘轮速目标应由底盘任务负责。
     */
    K230_UART_ServiceRx();
    BallControl_CheckStepperReplies();
    BallControl_SendAngleHeartbeat(now_ms);

    /*
     * K230接收缓冲区溢出。
     */
    if (K230_UART_TakeOverflow()) {

        /* Drop only the damaged backlog and resynchronise on new lines. */
        K230_UART_ClearRx();

        if (s_invalid_frame_count < 255U) {
            s_invalid_frame_count++;
        }
        s_vision_hold_active = 1U;

        if (((uint32_t)(now_ms - s_last_valid_ms) >=
             BallControl_MessageStaleLimitMs()) ||
            (s_invalid_frame_count >=
             BallControl_InvalidFrameLimit())) {
            s_have_measurement = 0U;
        }
        s_velocity_mm_s = 0.0f;

        BallControl_Task3ResetCondition();
    }

    if ((uint32_t)(now_ms - s_last_valid_ms) >
        BallControl_MessageStaleLimitMs()) {
        /* Allow immediate resynchronisation if the K230 has rebooted. */
        s_have_ball_sequence = 0U;
    }

    /* Drain the queue, but update the controller only from the newest frame. */
    while (K230_UART_ReadBall(
        &incoming)) {

        if (have_latest_incoming != 0U) {
            if (BallControl_IsSequenceNewer(
                    incoming.sequence,
                    latest_incoming.sequence)) {
                latest_incoming = incoming;
            }
        }
        else if ((s_have_ball_sequence == 0U) ||
                 BallControl_IsSequenceNewer(
                    incoming.sequence,
                    s_last_ball_sequence)) {
            latest_incoming = incoming;
            have_latest_incoming = 1U;
        }
    }

    if (have_latest_incoming != 0U) {
        s_last_ball_sequence = latest_incoming.sequence;
        s_have_ball_sequence = 1U;
        if (BallControl_AcceptMessage(
                &latest_incoming,
                now_ms)) {
            s_new_measurement_this_update = 1U;
        }
    }

    if ((s_stepper_fault_latched == 0U) &&
        !BallControl_ServiceBootZero(now_ms)) {
        s_state = BALL_CONTROL_STOPPED;
        BallControl_UpdateDisplay(now_ms, 0U);
        return;
    }

    BallControl_DebugPrint(now_ms);

    /*
     * 按键停止状态。
     */
    if (running == 0U) {

        if (s_previous_running != 0U) {
            (void)StepperEmm_Stop(
                STEPPER_EMM_DEFAULT_ADDRESS);
        }

        s_previous_running = 0U;
        s_stepper_enabled = 0U;
        s_stepper_fault_latched = 0U;
        s_direction_fault_latched = 0U;
        s_direction_check_active = 0U;

        StepperEmm_ClearRx();

        s_state =
            BALL_CONTROL_STOPPED;

        /*
         * Keep the newest valid K230 measurement while stopped.
         * TASK3 captures that reading as the physical O-point offset
         * on the next STOP-to-RUN edge. Clearing it every loop made
         * startup depend on a new UART frame arriving in that exact loop.
         */
        if ((s_have_measurement != 0U) &&
            ((uint32_t)(now_ms - s_last_valid_ms) >
             BallControl_MessageStaleLimitMs())) {

            s_have_measurement = 0U;
            s_velocity_mm_s = 0.0f;
        }

        /*
         * 停止后将题目3恢复到等待中心状态，
         * 下次按键重新测试。
         */
        if (s_task3_enabled != 0U) {
            BallControl_Task3ConfigureWaiting();
        }
        else {
            BallControl_Task3ResetCondition();

            if (s_auto_zero_hold_enabled != 0U) {
                s_auto_zero_hold_pending = 1U;
                s_task3_origin_valid = 0U;
                s_hold_ff_last_ms = now_ms;
                s_hold_ff_previous_speed_mm_s = 0.0f;
                s_hold_ff_acceleration_mm_s2 = 0.0f;
                s_hold_ff_pulses = 0;
            }
        }

        BallControl_UpdateDisplay(
            now_ms,
            0U);

        return;
    }

    /*
     * 检测STOP -> RUN边沿。
     *
     * 在按键启动瞬间开始题目第3项计时。
     */
    if ((s_previous_running == 0U) &&
        (s_task3_enabled != 0U)) {

        BallControl_Task3Begin(
            now_ms);

        if (s_task3_started == 0U) {
            /*
             * Keep the key request latched while waiting for the next valid
             * BALL frame.  A second key press still toggles Flag_Stop back to
             * STOP and cancels the request.  Previously this line forced
             * Flag_Stop to STOP immediately, so X42S enable was unreachable
             * whenever the exact key loop did not contain a fresh frame.
             */
            s_state = BALL_CONTROL_SEARCHING;
            BallControl_UpdateDisplay(now_ms, 1U);
            return;
        }
    }

    /*
     * Combined line-following + ball-hold mode. Capture the physical O
     * position on every start edge and hold zero relative to that reading.
     * The lift remains disabled until a fresh BALL position is available.
     */
    if ((s_previous_running == 0U) &&
        (s_task3_enabled == 0U) &&
        (s_auto_zero_hold_enabled != 0U)) {
        s_auto_zero_hold_pending = 1U;
        s_task3_origin_valid = 0U;
    }

    if ((s_auto_zero_hold_enabled != 0U) &&
        (s_auto_zero_hold_pending != 0U)) {

        if ((s_have_measurement == 0U) ||
            (s_vision_hold_active != 0U) ||
            ((uint32_t)(now_ms - s_last_valid_ms) >
             BallControl_MessageStaleLimitMs())) {
            s_state = BALL_CONTROL_SEARCHING;
            BallControl_UpdateDisplay(now_ms, 1U);
            return;
        }

        s_task3_origin_sensor_mm =
            s_message.position_mm;
        s_task3_origin_valid = 1U;
        s_auto_zero_hold_pending = 0U;
        s_previous_position_mm =
            s_message.position_mm;
        s_velocity_mm_s = 0.0f;
        s_target_position_mm = 0;
        s_hold_ff_last_ms = now_ms;
        s_hold_ff_previous_speed_mm_s = 0.0f;
        s_hold_ff_acceleration_mm_s2 = 0.0f;
        s_hold_ff_pulses = 0;
        s_hold_velocity_mm_s = 0.0f;
        BallControl_ResetPidMemory();
    }

    /*
     * X42S故障锁存。
     */
    if (s_stepper_fault_latched != 0U) {

        s_state =
            BALL_CONTROL_STEPPER_FAULT;

        BallControl_Task3ResetCondition();

        BallControl_UpdateDisplay(
            now_ms,
            0U);

        return;
    }

    if (s_direction_fault_latched != 0U) {
        s_state = BALL_CONTROL_DIRECTION_FAULT;
        BallControl_ReturnToLogicalCenter(now_ms);
        BallControl_UpdateDisplay(now_ms, 0U);
        return;
    }

    /*
     * 第一次进入运行状态时使能X42S。
     */
    if (s_stepper_enabled == 0U) {

        if (!StepperEmm_Enable(
                STEPPER_EMM_DEFAULT_ADDRESS,
                true)) {

            BallControl_LatchStepperFault(
                STEPPER_EMM_REPLY_PARAMETER_ERROR);
        }
        else {
            s_stepper_enabled = 1U;
            s_previous_running = 1U;
            s_last_stepper_ms = now_ms;
        }

        BallControl_UpdateDisplay(
            now_ms,
            1U);

        return;
    }

    s_previous_running = 1U;

    if (s_task3_state == BALL_TASK3_SAFETY_STOP) {
        BallControl_ReturnToLogicalCenter(now_ms);
        BallControl_UpdateDisplay(now_ms, 1U);
        return;
    }

    /*
     * 有效且未过期的K230数据。
     */
    if ((s_have_measurement != 0U) &&
        ((uint32_t)(
            now_ms -
            s_last_valid_ms
        ) <= BallControl_MessageStaleLimitMs())) {

        if ((s_vision_hold_active != 0U) ||
            ((uint32_t)(now_ms - s_last_valid_ms) >
             BallControl_VisionHoldLimitMs())) {

            s_state = BALL_CONTROL_VISION_HOLD;
            BallControl_Task3ResetCondition();
            BallControl_UpdateDisplay(now_ms, 0U);
            return;
        }

        s_state = BALL_CONTROL_TRACKING;

        /*
         * 先更新任务3目标，
         * 再执行位置控制。
         */
        BallControl_Task3Update(
            now_ms);

        if (BallControl_CheckDirectionFault(now_ms)) {
            BallControl_UpdateDisplay(now_ms, 1U);
            return;
        }

        if (s_new_measurement_this_update != 0U) {
#if BALL_TASK3_RELAY_ENABLE == 1U
            if (s_task3_enabled != 0U) {
                BallControl_RunTask3RelayLoop(now_ms);
            }
            else if (s_auto_zero_hold_enabled != 0U) {
                BallControl_RunAutoZeroHoldLoop(now_ms);
            }
            else {
                BallControl_RunPositionLoop(now_ms);
            }
#else
            if (s_task3_enabled != 0U) {
                BallControl_RunTask3PidLoop(now_ms);
            }
            else if (s_auto_zero_hold_enabled != 0U) {
                BallControl_RunAutoZeroHoldLoop(now_ms);
            }
            else {
                BallControl_RunPositionLoop(now_ms);
            }
#endif
        }
    }
    else {
        /*
         * 视觉无效或超时。
         */
        s_have_measurement = 0U;
        s_velocity_mm_s = 0.0f;
        s_direction_check_active = 0U;

        BallControl_ResetPidMemory();
        BallControl_Task3ResetCondition();

        if (s_have_ever_received == 0U) {
            s_state =
                BALL_CONTROL_SEARCHING;
        }
        else {
            s_state =
                BALL_CONTROL_LOST;
        }

        /*
         * 长时间丢球后回到逻辑水平位置。
         */
        if ((uint32_t)(
                now_ms -
                s_last_valid_ms
            ) >= BallControl_CenterAfterLostMs()) {

            BallControl_ReturnToLogicalCenter(
                now_ms);
        }
    }

    BallControl_UpdateDisplay(
        now_ms,
        0U);
}


bool BallControl_SetTargetMm(
    int16_t target_mm)
{
    if ((target_mm <
         BALL_TARGET_MIN_MM) ||

        (target_mm >
         BALL_TARGET_MAX_MM)) {

        return false;
    }

    /*
     * 自动任务3运行时，
     * 不允许外部手动修改目标。
     */
    if (s_task3_enabled != 0U) {
        return false;
    }

    s_target_position_mm =
        target_mm;

    return true;
}


int16_t BallControl_GetTargetMm(void)
{
    return s_target_position_mm;
}


void BallControl_SetDisplayEnabled(
    bool enabled)
{
    s_display_enabled =
        enabled ?
        1U :
        0U;

    if (s_display_enabled != 0U) {
        s_last_display_ms = 0U;
    }
}


void BallControl_SetTask3Enabled(
    bool enabled)
{
    uint32_t now_ms =
        Board_GetMillis();

    s_task3_enabled =
        enabled ?
        1U :
        0U;

    if (s_task3_enabled != 0U) {

        s_auto_zero_hold_enabled = 0U;
        s_auto_zero_hold_pending = 0U;

        /*
         * 题目3与摆杆标定模式互斥。
         */
        s_rod_calibration_enabled = 0U;

        BallControl_Task3ConfigureWaiting();

        /*
         * 如果启用时已经处于RUN状态，
         * 立即开始测试。
         */
        if (Flag_Stop == 0) {
            BallControl_Task3Begin(
                now_ms);
        }
    }
    else {
        s_task3_state =
            BALL_TASK3_DISABLED;

        s_control_region = BALL_REGION_IDLE;

        s_task3_started = 0U;
        s_task3_elapsed_ms = 0U;

        s_target_position_mm = 0;

        BallControl_Task3ResetCondition();
    }
}


void BallControl_SetAutoZeroHoldEnabled(
    bool enabled)
{
    s_auto_zero_hold_enabled =
        enabled ? 1U : 0U;
    s_auto_zero_hold_pending =
        enabled ? 1U : 0U;
    s_task3_origin_valid = 0U;
    s_hold_ff_last_ms = Board_GetMillis();
    s_hold_ff_previous_speed_mm_s = 0.0f;
    s_hold_ff_acceleration_mm_s2 = 0.0f;
    s_hold_ff_pulses = 0;

    if (s_auto_zero_hold_enabled != 0U) {
        s_task3_enabled = 0U;
        s_task3_started = 0U;
        s_task3_state = BALL_TASK3_DISABLED;
        s_rod_calibration_enabled = 0U;
        s_control_region = BALL_REGION_IDLE;
        s_target_position_mm = 0;
        BallControl_Task3ResetCondition();
        BallControl_ResetPidMemory();
    }
}


bool BallControl_IsAutoZeroHoldReady(void)
{
    uint32_t now_ms = Board_GetMillis();

    return
        (s_auto_zero_hold_enabled != 0U) &&
        (s_auto_zero_hold_pending == 0U) &&
        (s_task3_origin_valid != 0U) &&
        (s_have_measurement != 0U) &&
        (s_vision_hold_active == 0U) &&
        (s_stepper_zero_done != 0U) &&
        (s_stepper_enabled != 0U) &&
        (s_stepper_fault_latched == 0U) &&
        ((uint32_t)(now_ms - s_last_valid_ms) <=
         BALL_MESSAGE_STALE_MS);
}


bool BallControl_IsAutoZeroHoldHealthy(void)
{
    uint32_t now_ms = Board_GetMillis();

    return
        (s_auto_zero_hold_enabled != 0U) &&
        (s_task3_origin_valid != 0U) &&
        (s_stepper_zero_done != 0U) &&
        (s_stepper_enabled != 0U) &&
        (s_stepper_fault_latched == 0U) &&
        (s_direction_fault_latched == 0U) &&
        ((uint32_t)(now_ms - s_last_valid_ms) <=
         BALL_TASK4_MESSAGE_STALE_MS);
}


bool BallControl_IsAutoZeroHoldVisionHeld(void)
{
    uint32_t now_ms = Board_GetMillis();

    if (s_auto_zero_hold_enabled == 0U) {
        return false;
    }

    return
        (s_vision_hold_active != 0U) ||
        ((uint32_t)(now_ms - s_last_valid_ms) >
         BALL_TASK4_VISION_HOLD_MS);
}


BallAutoHoldFault BallControl_GetAutoZeroHoldFault(void)
{
    uint32_t now_ms = Board_GetMillis();

    if ((s_stepper_fault_latched != 0U) ||
        (s_direction_fault_latched != 0U)) {
        return BALL_AUTO_HOLD_FAULT_STEPPER;
    }

    if ((s_auto_zero_hold_enabled != 0U) &&
        (s_task3_origin_valid != 0U) &&
        ((uint32_t)(now_ms - s_last_valid_ms) >
         BALL_TASK4_MESSAGE_STALE_MS)) {
        return BALL_AUTO_HOLD_FAULT_VISION_LOST;
    }

    return BALL_AUTO_HOLD_FAULT_NONE;
}


bool BallControl_IsTask3Finished(void)
{
    return
        (s_task3_enabled != 0U) &&
        (s_task3_state ==
         BALL_TASK3_FINISHED);
}


uint32_t BallControl_GetTask3ElapsedMs(void)
{
    return s_task3_elapsed_ms;
}


void BallControl_SetRodCalibrationEnabled(
    bool enabled)
{
    s_rod_calibration_enabled =
        enabled ?
        1U :
        0U;

    if (s_rod_calibration_enabled != 0U) {

        s_task3_enabled = 0U;
        s_auto_zero_hold_enabled = 0U;
        s_auto_zero_hold_pending = 0U;
        s_task3_state =
            BALL_TASK3_DISABLED;

        s_control_region = BALL_REGION_IDLE;

        s_task3_started = 0U;
        s_target_position_mm = 0;

        BallControl_Task3ResetCondition();
    }
}


bool BallControl_IsStartReady(void)
{
    /*
     * 通过初始中心判断以后，
     * 状态会离开WAIT_CENTER。
     */
    return
        (s_task3_enabled != 0U) &&

        (s_task3_state !=
         BALL_TASK3_WAIT_CENTER) &&

        (s_task3_state !=
         BALL_TASK3_DISABLED);
}


int16_t BallControl_GetPositionErrorMm(void)
{
    int32_t error_mm;

    if (s_have_measurement == 0U) {
        return 0;
    }

    error_mm =
        (int32_t)s_target_position_mm -
        (int32_t)BallControl_MeasuredPositionMm();

    if (error_mm > 32767) {
        return (int16_t)32767;
    }

    if (error_mm < -32768) {
        return (int16_t)-32768;
    }

    return (int16_t)error_mm;
}


uint8_t BallControl_GetLastStepperReply(void)
{
    return s_last_stepper_reply;
}
