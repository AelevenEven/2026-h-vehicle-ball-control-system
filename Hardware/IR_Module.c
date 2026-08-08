#include "IR_Module.h"

#include <math.h>

#include "control.h"

/* Legacy public calibration variables retained for source compatibility. */
float Turn90Angle  = 70.0f;
float TurnMaxAngle = 45.0f;
float TurnMidAngle = 20.0f;
float TurnMinAngle = 15.0f;
float ForwardLimit = TASK2_START_SPEED_MM_S;

float BaseSpeed = TASK2_START_SPEED_MM_S;
float base_speed_mm;
float turn_diff;

uint8_t ir_raw_state;
uint8_t ir_black_mask;
uint8_t ir_line_valid;
uint8_t ir_front_cross_detected;
uint16_t ir_lost_sample_count;
float ir_line_raw_error_mm;
float ir_line_position;
float ir_line_derivative;
float ir_line_turn_target;
float ir_curve_feedforward;
float ir_total_turn_target;
float ir_line_turn_diff;
float ir_yaw_rate_target_dps;
float ir_yaw_rate_measured_dps;
float ir_gyro_correction;
float ir_motor_steering_sign = 1.0f;

static const float s_front_position_mm[IR_CHANNEL_COUNT] =
    TASK2_FRONT_POSITION_MM;
static uint8_t s_scan_history[3];
static uint8_t s_scan_history_valid;
static uint8_t s_cross_sample_count;
static uint8_t s_line_filter_valid;
static uint8_t s_diagnostic_mode;
static float s_previous_filtered_error_mm;
static float s_last_valid_error_mm;
static float s_speed_limit_mm_s;
static float s_prev_curve_feedforward;    /* 用于检测弯道→直线过渡 */
static float s_search_turn_mm_s = TASK2_SEARCH_TURN_MM_S;

static float IR_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float IR_Approach(float current, float target, float maximum_change)
{
    if (target > current + maximum_change) {
        return current + maximum_change;
    }
    if (target < current - maximum_change) {
        return current - maximum_change;
    }
    return target;
}

static void IR_SelectChannel(uint8_t channel)
{
    IR_AD0_WRITE((channel >> 0) & 0x01U);
    IR_AD1_WRITE((channel >> 1) & 0x01U);
    IR_AD2_WRITE((channel >> 2) & 0x01U);
}

#if IR_SENSOR_ORDER_REVERSED == 1U
static uint8_t IR_ReverseBits(uint8_t value)
{
    value = (uint8_t)(((value & 0x55U) << 1U) |
                      ((value & 0xAAU) >> 1U));
    value = (uint8_t)(((value & 0x33U) << 2U) |
                      ((value & 0xCCU) >> 2U));
    return (uint8_t)((value << 4U) | (value >> 4U));
}
#endif

uint8_t IR_CountBlack(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t)(count + (value & 1U));
        value >>= 1;
    }
    return count;
}

uint8_t IR_IsFinishMarkerSignature(uint8_t black_mask)
{
    uint8_t index;
    uint8_t current_run = 0U;
    uint8_t longest_run = 0U;

    for (index = 0U; index < IR_CHANNEL_COUNT; index++) {
        if ((black_mask & (uint8_t)(0x80U >> index)) != 0U) {
            current_run++;
            if (current_run > longest_run) {
                longest_run = current_run;
            }
        } else {
            current_run = 0U;
        }
    }

    return ((IR_CountBlack(black_mask) >=
             TASK2_FRONT_CROSS_MIN_COUNT) &&
            ((black_mask & 0x18U) == 0x18U) &&
            (longest_run >= TASK2_FRONT_CROSS_MIN_RUN)) ? 1U : 0U;
}

uint8_t IR_IsWideMarker(uint8_t black_mask)
{
    return IR_IsFinishMarkerSignature(black_mask);
}

static float IR_ErrorSpeedLimit(float error_mm, float yaw_rate_dps)
{
    float magnitude = fabsf(error_mm);
    float yaw_magnitude = fabsf(yaw_rate_dps);
    float error_limit;
    float yaw_limit;
    float ratio;

    if (magnitude <= TASK2_ERROR_INNER_MM) {
        error_limit = TASK2_STRAIGHT_SPEED_MM_S;
    } else if (magnitude <= TASK2_ERROR_MIDDLE_MM) {
        ratio = (magnitude - TASK2_ERROR_INNER_MM) /
            (TASK2_ERROR_MIDDLE_MM - TASK2_ERROR_INNER_MM);
        error_limit = TASK2_INNER_SPEED_MM_S +
            ratio * (TASK2_MIDDLE_SPEED_MM_S -
                     TASK2_INNER_SPEED_MM_S);
    } else if (magnitude <= TASK2_ERROR_OUTER_MM) {
        ratio = (magnitude - TASK2_ERROR_MIDDLE_MM) /
            (TASK2_ERROR_OUTER_MM - TASK2_ERROR_MIDDLE_MM);
        error_limit = TASK2_MIDDLE_SPEED_MM_S +
            ratio * (TASK2_CURVE_SPEED_MM_S -
                     TASK2_MIDDLE_SPEED_MM_S);
    } else {
        error_limit = TASK2_CURVE_SPEED_MM_S;
    }

    if (yaw_magnitude <= TASK2_YAW_SPEED_START_DPS) {
        yaw_limit = TASK2_STRAIGHT_SPEED_MM_S;
    } else if (yaw_magnitude >= TASK2_YAW_SPEED_FULL_DPS) {
        yaw_limit = TASK2_CURVE_SPEED_MM_S;
    } else {
        ratio = (yaw_magnitude - TASK2_YAW_SPEED_START_DPS) /
            (TASK2_YAW_SPEED_FULL_DPS -
             TASK2_YAW_SPEED_START_DPS);
        yaw_limit = TASK2_STRAIGHT_SPEED_MM_S +
            ratio * (TASK2_CURVE_SPEED_MM_S -
                     TASK2_STRAIGHT_SPEED_MM_S);
    }

    return (error_limit < yaw_limit) ? error_limit : yaw_limit;
}

static void IR_ApplyMotorTargets(void)
{
    float actuator_turn = turn_diff * ir_motor_steering_sign;
    float left_speed_mm_s = base_speed_mm - actuator_turn;
    float right_speed_mm_s = base_speed_mm + actuator_turn;

    left_speed_mm_s = IR_Clamp(
        left_speed_mm_s, 0.0f, TASK2_WHEEL_REF_MAX_MM_S);
    right_speed_mm_s = IR_Clamp(
        right_speed_mm_s, 0.0f, TASK2_WHEEL_REF_MAX_MM_S);
    MotorA.Target_Encoder = left_speed_mm_s * 0.001f;
    MotorB.Target_Encoder = right_speed_mm_s * 0.001f;
}

void IR_SetCurveFeedforward(float turn_mm_s)
{
    ir_curve_feedforward = IR_Clamp(
        turn_mm_s,
        -TASK2_TURN_MAX_MM_S,
        TASK2_TURN_MAX_MM_S);
}

void IR_ApplyYawRateTracking(
    float yaw_rate_dps, uint8_t gyro_valid)
{
    float correction_limit = TASK2_GYRO_CORRECTION_LIMIT_MM_S;
    float final_turn_limit = TASK2_TURN_MAX_MM_S;

    if (s_diagnostic_mode != 0U) {
        correction_limit = 20.0f;
        final_turn_limit = 45.0f;
    }

    ir_yaw_rate_measured_dps =
        TASK2_GYRO_Z_SIGN * yaw_rate_dps;
    ir_yaw_rate_target_dps =
        (2.0f * ir_line_turn_diff /
         TASK2_WHEEL_TRACK_MM) * TASK2_RAD_TO_DEG;
    if (gyro_valid != 0U) {
        ir_gyro_correction = IR_Clamp(
            TASK2_GYRO_RATE_K_MM_S_PER_DPS *
                (ir_yaw_rate_target_dps -
                 ir_yaw_rate_measured_dps),
            -correction_limit,
            correction_limit);
    } else {
        ir_gyro_correction = 0.0f;
        ir_yaw_rate_measured_dps = 0.0f;
    }
    turn_diff = IR_Clamp(
        ir_line_turn_diff + ir_gyro_correction,
        -final_turn_limit,
        final_turn_limit);
    IR_ApplyMotorTargets();
}

static void IR_UpdateSpecialState(uint8_t black_count)
{
    uint8_t cross_now = IR_IsWideMarker(ir_black_mask);
    static uint8_t s_cross_clear_count;

    if (cross_now != 0U) {
        s_cross_clear_count = 0U;
        if (s_cross_sample_count < TASK2_SPECIAL_CONFIRM_SAMPLES) {
            s_cross_sample_count++;
        }
    } else {
        /*
         * 滞后清除：需要连续多帧无交叉才清零，
         * 防止单帧噪声导致 ir_front_cross_detected 意外翻转，
         * 进而造成按键启动被"吃掉"。
         */
        if (s_cross_clear_count < TASK2_SPECIAL_CONFIRM_SAMPLES) {
            s_cross_clear_count++;
        }
        if (s_cross_clear_count >= TASK2_SPECIAL_CONFIRM_SAMPLES) {
            s_cross_sample_count = 0U;
        }
    }
    ir_front_cross_detected =
        (s_cross_sample_count >= TASK2_SPECIAL_CONFIRM_SAMPLES) ? 1U : 0U;

    if (black_count == 0U) {
        if (ir_lost_sample_count < 65535U) {
            ir_lost_sample_count++;
        }
    } else {
        ir_lost_sample_count = 0U;
    }
}

static void IR_UpdateLineError(void)
{
    uint8_t index;
    uint8_t black_count = IR_CountBlack(ir_black_mask);
    float weighted_sum_mm = 0.0f;
    float raw_error_mm;
    float raw_derivative_mm_s;

    IR_UpdateSpecialState(black_count);

    if ((black_count == 0U) ||
        (ir_front_cross_detected != 0U)) {
        ir_line_valid = 0U;
        ir_line_raw_error_mm = s_last_valid_error_mm;
        ir_line_derivative = 0.0f;
        return;
    }

    for (index = 0U; index < IR_CHANNEL_COUNT; index++) {
        if ((ir_black_mask & (uint8_t)(0x80U >> index)) != 0U) {
            weighted_sum_mm += s_front_position_mm[index];
        }
    }
    raw_error_mm = weighted_sum_mm / (float)black_count -
        TASK2_FRONT_CENTER_OFFSET_MM;
    ir_line_raw_error_mm = raw_error_mm;
    s_last_valid_error_mm = raw_error_mm;
    ir_line_valid = 1U;

    if (s_line_filter_valid == 0U) {
        ir_line_position = raw_error_mm;
        s_previous_filtered_error_mm = raw_error_mm;
        ir_line_derivative = 0.0f;
        s_line_filter_valid = 1U;
        return;
    }

    ir_line_position += TASK2_LINE_ERROR_ALPHA *
        (raw_error_mm - ir_line_position);
    raw_derivative_mm_s =
        (ir_line_position - s_previous_filtered_error_mm) *
        (1000.0f / (float)TASK2_LINE_PERIOD_MS);
    raw_derivative_mm_s = IR_Clamp(
        raw_derivative_mm_s,
        -TASK2_LINE_DERIVATIVE_LIMIT_MM_S,
        TASK2_LINE_DERIVATIVE_LIMIT_MM_S);
    ir_line_derivative += TASK2_LINE_DERIVATIVE_ALPHA *
        (raw_derivative_mm_s - ir_line_derivative);
    s_previous_filtered_error_mm = ir_line_position;
}

static void IR_ApplyLineControl(void)
{
    float control_error_mm;
    float error_magnitude_mm;
    float proportional_gain;
    float speed_limit_mm_s;

    IR_UpdateLineError();

    if (ir_line_valid != 0U) {
        control_error_mm = ir_line_position;
        error_magnitude_mm = fabsf(control_error_mm);

        /*
         * 弯道→直线过渡时，复位误差滤波器和微分项，
         * 防止弯道残留的历史值在直线上引发振荡。
         */
        if (fabsf(ir_curve_feedforward) < 3.0f &&
            fabsf(s_prev_curve_feedforward) > 10.0f) {
            ir_line_derivative = 0.0f;
            s_line_filter_valid = 0U;
        }
        s_prev_curve_feedforward = ir_curve_feedforward;

        if (error_magnitude_mm >=
            TASK2_LINE_KP_OUTER_ERROR_MM) {
            proportional_gain = TASK2_LINE_KP_OUTER;
        } else {
            proportional_gain = TASK2_LINE_KP_INNER +
                (TASK2_LINE_KP_OUTER -
                 TASK2_LINE_KP_INNER) *
                (error_magnitude_mm /
                 TASK2_LINE_KP_OUTER_ERROR_MM);
        }
        ir_line_turn_target =
            proportional_gain * control_error_mm +
            TASK2_LINE_KD * ir_line_derivative;
        ir_line_turn_target = IR_Clamp(
            ir_line_turn_target,
            -TASK2_TURN_MAX_MM_S,
            TASK2_TURN_MAX_MM_S);
        ir_total_turn_target = IR_Clamp(
            ir_line_turn_target + ir_curve_feedforward,
            -TASK2_TURN_MAX_MM_S,
            TASK2_TURN_MAX_MM_S);
        ir_line_turn_diff = IR_Approach(
            ir_line_turn_diff,
            ir_total_turn_target,
            TASK2_TURN_RATE_MM_S_PER_TICK);

        speed_limit_mm_s = IR_ErrorSpeedLimit(
            ir_line_position, ir_yaw_rate_measured_dps);
        s_speed_limit_mm_s = IR_Approach(
            s_speed_limit_mm_s,
            speed_limit_mm_s,
            TASK2_SPEED_LIMIT_RATE_PER_TICK);
        base_speed_mm =
            (BaseSpeed < s_speed_limit_mm_s) ?
                BaseSpeed : s_speed_limit_mm_s;
    } else if (ir_front_cross_detected != 0U) {
        /*
         * A transverse marker is not a steering error. Hold the last turn
         * briefly until the task state decides whether this is the finish.
         */
        ir_line_turn_target = ir_line_turn_diff;
        ir_total_turn_target = ir_line_turn_diff;
        base_speed_mm = BaseSpeed;
    } else if (ir_lost_sample_count >= TASK2_LOST_CONFIRM_SAMPLES) {
        ir_line_derivative = 0.0f;
        if (fabsf(ir_curve_feedforward) >= 3.0f) {
            /* Continue the known curve direction during a short line loss. */
            ir_line_turn_target =
                (ir_curve_feedforward > 0.0f) ?
                    s_search_turn_mm_s : -s_search_turn_mm_s;
        } else {
            ir_line_turn_target =
                (s_last_valid_error_mm > 0.0f) ?
                    s_search_turn_mm_s :
                (s_last_valid_error_mm < 0.0f) ?
                    -s_search_turn_mm_s : 0.0f;
        }
        ir_total_turn_target = ir_line_turn_target;
        ir_line_turn_diff = IR_Approach(
            ir_line_turn_diff,
            ir_total_turn_target,
            TASK2_TURN_RATE_MM_S_PER_TICK);
        base_speed_mm = TASK2_SEARCH_SPEED_MM_S;
    } else {
        /* One invalid frame is ignored rather than interpreted as zero error. */
        ir_line_derivative = 0.0f;
        base_speed_mm =
            (BaseSpeed < TASK2_CURVE_SPEED_MM_S) ?
                BaseSpeed : TASK2_CURVE_SPEED_MM_S;
    }
}

void IR_Module_Init(void)
{
    s_diagnostic_mode = 0U;
    ir_motor_steering_sign = 1.0f;
    s_search_turn_mm_s = TASK2_SEARCH_TURN_MM_S;
    IR_SelectChannel(0U);
    IR_LineReset();
}

void IR_SetDiagnosticMode(uint8_t enabled)
{
    s_diagnostic_mode = (enabled != 0U) ? 1U : 0U;
    turn_diff = 0.0f;
    ir_line_turn_diff = 0.0f;
}

void IR_SetMotorSteeringSign(float sign)
{
    ir_motor_steering_sign = (sign < 0.0f) ? -1.0f : 1.0f;
    turn_diff = 0.0f;
    ir_line_turn_diff = 0.0f;
}


void IR_SetSearchTurnMmS(float turn_mm_s)
{
    s_search_turn_mm_s = IR_Clamp(
        turn_mm_s, 0.0f, TASK2_TURN_MAX_MM_S);
}

uint8_t IR_Scan(void)
{
    uint8_t channel;
    uint8_t raw_state = 0U;
    uint8_t active_mask;

    for (channel = 0U; channel < IR_CHANNEL_COUNT; channel++) {
        IR_SelectChannel(channel);
        delay_us(IR_CHANNEL_SETTLE_US);
        if (IR_OUT_READ() != 0U) {
            raw_state |= (uint8_t)(0x80U >> channel);
        }
    }

    ir_raw_state = raw_state;
#if IR_BLACK_ACTIVE_LEVEL == 1U
    active_mask = raw_state;
#else
    active_mask = (uint8_t)(~raw_state);
#endif
#if IR_SENSOR_ORDER_REVERSED == 1U
    active_mask = IR_ReverseBits(active_mask);
#endif

#if TASK2_FRONT_TEMPORAL_FILTER == 1U
    if (s_scan_history_valid == 0U) {
        s_scan_history[0] = active_mask;
        s_scan_history[1] = active_mask;
        s_scan_history[2] = active_mask;
        s_scan_history_valid = 1U;
    } else {
        s_scan_history[2] = s_scan_history[1];
        s_scan_history[1] = s_scan_history[0];
        s_scan_history[0] = active_mask;
    }
    ir_black_mask = (uint8_t)(
        (s_scan_history[0] & s_scan_history[1]) |
        (s_scan_history[0] & s_scan_history[2]) |
        (s_scan_history[1] & s_scan_history[2]));
#else
    ir_black_mask = active_mask;
#endif
    return ir_black_mask;
}

uint8_t IR_ReadRawState(void)
{
    (void)IR_Scan();
    return ir_raw_state;
}

uint8_t IR_ReadBlackMask(void)
{
    return IR_Scan();
}

uint8_t IR_GetBlackMask(void)
{
    return ir_black_mask;
}

uint8_t IR_LineDetected(void)
{
    return (ir_black_mask != 0U) ? 1U : 0U;
}

void IR_LineReset(void)
{
    ir_raw_state = 0U;
    ir_black_mask = 0U;
    ir_line_valid = 0U;
    ir_front_cross_detected = 0U;
    ir_lost_sample_count = 0U;
    ir_line_raw_error_mm = 0.0f;
    ir_line_position = 0.0f;
    ir_line_derivative = 0.0f;
    ir_line_turn_target = 0.0f;
    ir_curve_feedforward = 0.0f;
    ir_total_turn_target = 0.0f;
    ir_line_turn_diff = 0.0f;
    ir_yaw_rate_target_dps = 0.0f;
    ir_yaw_rate_measured_dps = 0.0f;
    ir_gyro_correction = 0.0f;
    base_speed_mm = 0.0f;
    turn_diff = 0.0f;
    s_previous_filtered_error_mm = 0.0f;
    s_last_valid_error_mm = 0.0f;
    s_scan_history[0] = 0U;
    s_scan_history[1] = 0U;
    s_scan_history[2] = 0U;
    s_scan_history_valid = 0U;
    s_cross_sample_count = 0U;
    s_line_filter_valid = 0U;
    s_speed_limit_mm_s = TASK2_STRAIGHT_SPEED_MM_S;
    s_prev_curve_feedforward = 0.0f;
}

void IR_LineReacquire(void)
{
    ir_line_derivative = 0.0f;
    s_previous_filtered_error_mm = ir_line_position;
    s_line_filter_valid = (ir_line_valid != 0U) ? 1U : 0U;
}

void IRDM_line_inspection(void)
{
    IR_Module_Update(0.0f, 0U, 0.0f);
}

void IR_Module_Update(
    float yaw_rate_dps,
    uint8_t gyro_valid,
    float curve_feedforward_mm_s)
{
    /*
     * Publish the measured rate before line control so its speed scheduler
     * uses this cycle's IMU sample, then apply the final yaw-rate correction.
     */
    ir_yaw_rate_measured_dps =
        (gyro_valid != 0U) ?
            (TASK2_GYRO_Z_SIGN * yaw_rate_dps) : 0.0f;
    IR_SetCurveFeedforward(curve_feedforward_mm_s);
    (void)IR_Scan();
    IR_ApplyLineControl();
    IR_ApplyYawRateTracking(yaw_rate_dps, gyro_valid);
}

void IR_Module_Read(void)
{
    IRDM_line_inspection();
}
