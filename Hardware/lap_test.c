#include "lap_test.h"

#include <math.h>
#include <stdio.h>

#include "IR_Module.h"
#include "MPU6050.h"
#include "board.h"
#include "control.h"
#include "oled.h"

static Task2State s_state;
static Task2State s_lost_resume_state;
static Task2Fault s_fault;
static uint32_t s_start_ms;
static uint32_t s_final_time_ms;
static uint32_t s_last_line_ms;
static uint32_t s_last_state_ms;
static uint32_t s_last_display_ms;
static uint32_t s_last_ramp_ms;
static uint32_t s_starting_ms;
static uint32_t s_guide_candidate_ms;
static uint32_t s_line_lost_ms;
static uint32_t s_line_recover_ms;
static uint32_t s_creep_start_ms;
static uint32_t s_brake_start_ms;
static uint32_t s_stall_start_ms;
static uint32_t s_mismatch_start_ms;
static float s_base_command_mm_s;
static float s_cross_distance_mm;
static float s_start_reference_mm;
static float s_stop_reference_mm;
static float s_stop_learned_reference_mm;
static float s_stop_brake_compensation_mm;
static float s_curve_blend;
static float s_curve_feedforward_mm_s;
static uint8_t s_marker_armed;
static uint8_t s_finish_window;
static uint8_t s_finish_heading_ok;
static uint8_t s_finish_distance_ok;
static uint8_t s_marker_candidate;
static uint8_t s_marker_reject_reason;
static uint8_t s_previous_stop;
static uint8_t s_start_marker_phase;
static uint8_t s_start_reference_valid;
static uint8_t s_start_marker_mask;
static uint8_t s_finish_match_count;
static uint8_t s_task4_mode;
static uint8_t s_external_start_ready = 1U;
static float s_task4_ball_error_mm;
static uint8_t s_task4_ball_vision_hold;
static uint8_t s_task4_external_fault;

static uint8_t s_imu_detected;
static uint8_t s_imu_online;
static uint8_t s_yaw_reference_set;
static uint32_t s_last_imu_update_ms;
static uint32_t s_last_imu_success_ms;
static float s_gyro_rate_z_dps;

#define TASK2_MARKER_REJECT_NOT_ARMED       (0x01U)
#define TASK2_MARKER_REJECT_TIME             (0x02U)
#define TASK2_MARKER_REJECT_WINDOW           (0x04U)
#define TASK2_MARKER_REJECT_HEADING          (0x08U)
#define TASK2_MARKER_REJECT_OFFLINE_DISTANCE (0x10U)

static float Task2_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Task2_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float Task2_OdometryMm(void)
{
    return Control_GetAverageDistance() * 1000.0f;
}

static float Task2_StopReferenceMm(void)
{
    float reference_mm = TASK2_STOP_SENSOR_TO_REFERENCE_MM;

#if TASK2_STOP_AUTO_REFERENCE_ENABLE == 1U
    if (s_start_reference_valid != 0U) {
        reference_mm = s_start_reference_mm;
    }
#endif
    reference_mm += TASK2_STOP_REFERENCE_TRIM_MM;
    return Task2_Clamp(
        reference_mm,
        TASK2_STOP_REFERENCE_MIN_MM,
        TASK2_STOP_REFERENCE_MAX_MM);
}

static float Task2_StopBrakeCompensationMm(void)
{
    float measured_speed_mm_s =
        (Task2_Abs(left_speed_meas) +
         Task2_Abs(right_speed_meas)) * 0.5f;
    float estimated_speed_mm_s = measured_speed_mm_s;
    float compensation_mm;

    /* Encoder samples can briefly read low; never estimate below the current
     * commanded base speed at the marker. */
    if (estimated_speed_mm_s < s_base_command_mm_s) {
        estimated_speed_mm_s = s_base_command_mm_s;
    }
    estimated_speed_mm_s = Task2_Clamp(
        estimated_speed_mm_s, 0.0f, TASK2_WHEEL_REF_MAX_MM_S);
    compensation_mm =
        (estimated_speed_mm_s * estimated_speed_mm_s) /
        (2.0f * TASK2_STOP_BRAKE_DECEL_MM_S2);
    compensation_mm += TASK2_STOP_BRAKE_MARGIN_MM;
    return Task2_Clamp(
        compensation_mm, 0.0f, TASK2_STOP_BRAKE_COMP_MAX_MM);
}

static uint8_t Task2_IsStartMarkerMatch(uint8_t mask)
{
    uint8_t expanded_start;
    uint8_t overlap;

    if (s_start_marker_mask == 0U) {
        return 0U;
    }

    /* Allow a one-probe lateral shift when returning to A. */
    expanded_start = (uint8_t)(
        s_start_marker_mask |
        (uint8_t)(s_start_marker_mask << 1U) |
        (s_start_marker_mask >> 1U));
    overlap = (uint8_t)(mask & expanded_start);

    return ((IR_CountBlack(mask) >= TASK2_FINISH_MATCH_MIN_COUNT) &&
            (IR_CountBlack(overlap) >=
             TASK2_FINISH_MATCH_MIN_OVERLAP)) ? 1U : 0U;
}

static float Task2_CurveBlendForInterval(
    float odometry_mm,
    float curve_start_mm,
    float curve_end_mm,
    float transition_mm)
{
    float half_transition_mm = 0.5f * transition_mm;

    if ((odometry_mm <= curve_start_mm - half_transition_mm) ||
        (odometry_mm >= curve_end_mm + half_transition_mm)) {
        return 0.0f;
    }
    if (odometry_mm < curve_start_mm + half_transition_mm) {
        return (odometry_mm -
                (curve_start_mm - half_transition_mm)) /
            transition_mm;
    }
    if (odometry_mm <= curve_end_mm - half_transition_mm) {
        return 1.0f;
    }
    return ((curve_end_mm + half_transition_mm) -
            odometry_mm) / transition_mm;
}

static float Task2_CurveBlend(float odometry_mm)
{
    float first_curve_start_mm = TASK2_STRAIGHT_LENGTH_MM;
    float first_curve_end_mm =
        first_curve_start_mm + TASK2_CURVE_LENGTH_MM;
    float second_curve_start_mm =
        first_curve_end_mm + TASK2_STRAIGHT_LENGTH_MM;
    float second_curve_end_mm =
        second_curve_start_mm + TASK2_CURVE_LENGTH_MM;
    float first_blend = Task2_CurveBlendForInterval(
        odometry_mm,
        first_curve_start_mm,
        first_curve_end_mm,
        TASK2_CURVE_TRANSITION_MM);
    float second_blend = Task2_CurveBlendForInterval(
        odometry_mm,
        second_curve_start_mm,
        second_curve_end_mm,
        TASK2_SECOND_CURVE_TRANSITION_MM);

    return (first_blend > second_blend) ?
        first_blend : second_blend;
}

static uint8_t Task2_IsSecondCurveWindow(float odometry_mm)
{
    float second_curve_start_mm =
        TASK2_STRAIGHT_LENGTH_MM +
        TASK2_CURVE_LENGTH_MM +
        TASK2_STRAIGHT_LENGTH_MM;
    float second_curve_end_mm =
        second_curve_start_mm + TASK2_CURVE_LENGTH_MM;
    float half_transition_mm =
        0.5f * TASK2_SECOND_CURVE_TRANSITION_MM;

    return
        ((odometry_mm >=
          second_curve_start_mm - half_transition_mm) &&
         (odometry_mm <=
          second_curve_end_mm + half_transition_mm)) ? 1U : 0U;
}


static float Task2_CurveFeedforward(
    float base_speed_mm_s,
    float curve_blend,
    float odometry_mm)
{
    float feedforward_gain = 1.0f;

    /*
     * Positive turn means left wheel slower/right wheel faster (left turn).
     * Both TASK2 semicircles are clockwise, therefore the feedforward is
     * negative and remains covered by infrared feedback.
     */
    if ((s_task4_mode == 0U) &&
        (Task2_IsSecondCurveWindow(odometry_mm) != 0U)) {
        feedforward_gain = TASK2_SECOND_CURVE_FF_GAIN;
    }

    return -feedforward_gain * curve_blend * base_speed_mm_s *
        TASK2_WHEEL_TRACK_MM /
        (2.0f * TASK2_CURVE_RADIUS_MM);
}

static uint32_t Task2_CurrentElapsedMs(uint32_t now_ms)
{
    if ((s_task4_mode != 0U) &&
        (s_final_time_ms != 0U)) {
        return s_final_time_ms;
    }

    switch (s_state) {
    case TASK2_STARTING:
    case TASK2_RUNNING:
    case TASK2_APPROACH_STOP:
    case TASK2_FINAL_CREEP:
    case TASK2_BRAKING:
    case TASK2_LOST_LINE:
        return (uint32_t)(now_ms - s_start_ms);
    default:
        return s_final_time_ms;
    }
}

static void Task2_ClearFrame(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < 8U; page++) {
        for (column = 0U; column < 128U; column++) {
            OLED_GRAM[column][page] = 0U;
        }
    }
}

static void Task2_DrawText(uint8_t x, uint8_t y, const char *text)
{
    while ((*text != '\0') && (x <= 120U)) {
        OLED_ShowChar(x, y, (uint8_t)*text, 12U, 1U);
        x = (uint8_t)(x + 8U);
        text++;
    }
}

static void Task2_DrawDigits(
    uint8_t x, uint8_t y, uint32_t value, uint8_t digits)
{
    uint32_t divisor = 1U;
    uint8_t index;

    for (index = 1U; index < digits; index++) {
        divisor *= 10U;
    }
    for (index = 0U; index < digits; index++) {
        OLED_ShowChar(
            x, y,
            (uint8_t)('0' + (uint8_t)((value / divisor) % 10U)),
            12U, 1U);
        x = (uint8_t)(x + 8U);
        if (divisor > 1U) {
            divisor /= 10U;
        }
    }
}

static void Task2_DrawSigned3(
    uint8_t x, uint8_t y, float value)
{
    uint32_t magnitude;

    if (value < 0.0f) {
        OLED_ShowChar(x, y, (uint8_t)'-', 12U, 1U);
        value = -value;
    }
    else {
        OLED_ShowChar(x, y, (uint8_t)'+', 12U, 1U);
    }

    magnitude = (uint32_t)(value + 0.5f);
    if (magnitude > 999U) {
        magnitude = 999U;
    }
    Task2_DrawDigits((uint8_t)(x + 8U), y, magnitude, 3U);
}

static void Task2_DrawMask(uint8_t x, uint8_t y, uint8_t mask)
{
    uint8_t index;

    for (index = 0U; index < 8U; index++) {
        OLED_ShowChar(
            x, y,
            ((mask & (uint8_t)(0x80U >> index)) != 0U) ?
                (uint8_t)'1' : (uint8_t)'0',
            12U, 1U);
        x = (uint8_t)(x + 8U);
    }
}

static const char *Task2_StateText(void)
{
    switch (s_state) {
    case TASK2_IDLE:
        return "SELF CHECK";
    case TASK2_READY:
        return (ir_front_cross_detected != 0U) ?
            "START OK" : "ALIGN A LINE";
    case TASK2_STARTING:
        return "STARTING";
    case TASK2_RUNNING:
        if ((s_task4_mode != 0U) &&
            (s_task4_ball_vision_hold != 0U)) {
            return "BALL HOLD SLOW";
        }
        return (s_task4_mode != 0U) ?
            "TO B BALL HOLD" : "RUNNING";
    case TASK2_PASS_B:
        return "PASSED B";
    case TASK2_APPROACH_STOP:
        return "APPROACH";
    case TASK2_FINAL_CREEP:
        return "FINAL CREEP";
    case TASK2_BRAKING:
        return (s_task4_mode != 0U) ?
            "B PASSED STOP" : "BRAKING";
    case TASK2_STOPPED:
        return (s_task4_mode != 0U) ?
            "TASK4 DONE" : "STOPPED";
    case TASK2_LOST_LINE:
        return "SEARCH LINE";
    case TASK2_FAULT:
        switch (s_fault) {
        case TASK2_FAULT_ABORTED:
            return "FAULT ABORT";
        case TASK2_FAULT_START_LINE:
            return "FAULT START";
        case TASK2_FAULT_LINE_TIMEOUT:
            return "FAULT LINE";
        case TASK2_FAULT_RUN_TIMEOUT:
            return "FAULT TIME";
        case TASK2_FAULT_ENCODER_STALL:
            return "FAULT ENCODER";
        case TASK2_FAULT_WHEEL_MISMATCH:
            return "FAULT WHEELS";
        case TASK2_FAULT_CREEP_TIMEOUT:
            return "FAULT CREEP";
        case TASK2_FAULT_BALL_LOST:
            return "FAULT BALL LOST";
        case TASK2_FAULT_X42S:
            return "FAULT X42S";
        default:
            return "FAULT";
        }
    default:
        return "FAULT";
    }
}

static void Task2_UpdateDisplay(uint32_t now_ms, uint8_t force)
{
    uint32_t elapsed_ms;
    uint32_t seconds;
    uint32_t milliseconds;

    if ((force == 0U) &&
        ((uint32_t)(now_ms - s_last_display_ms) <
         TASK2_DISPLAY_PERIOD_MS)) {
        return;
    }
    s_last_display_ms = now_ms;
    elapsed_ms = Task2_CurrentElapsedMs(now_ms);
    seconds = elapsed_ms / 1000U;
    milliseconds = elapsed_ms % 1000U;
    if (seconds > 99U) {
        seconds = 99U;
    }

    Task2_ClearFrame();
    Task2_DrawText(
        0U, 0U,
        (s_task4_mode != 0U) ?
            ((s_imu_online != 0U) ?
                "TASK4 MPU+BALL" : "TASK4 LINE+BALL") :
            ((s_imu_online != 0U) ? "TASK2 MPU" : "TASK2 LINE"));
    Task2_DrawText(0U, 16U, Task2_StateText());
    Task2_DrawText(0U, 32U, "TIME:");
    Task2_DrawDigits(40U, 32U, seconds, 2U);
    OLED_ShowChar(56U, 32U, (uint8_t)'.', 12U, 1U);
    Task2_DrawDigits(64U, 32U, milliseconds, 3U);
    if ((s_state == TASK2_READY) &&
        (s_task4_mode != 0U) &&
        (s_external_start_ready == 0U)) {
        Task2_DrawText(0U, 48U, "WAIT BALL O");
    } else if (s_state == TASK2_READY) {
        Task2_DrawText(0U, 48U, "IR:");
        Task2_DrawMask(24U, 48U, IR_GetBlackMask());
    } else if ((s_state == TASK2_STOPPED) &&
               (s_task4_mode != 0U)) {
        Task2_DrawText(0U, 48U, "AB MM:1500");
    } else if (s_state == TASK2_STOPPED) {
        Task2_DrawText(0U, 48U, "GO:");
        Task2_DrawDigits(
            24U, 48U,
            (uint32_t)Task2_Clamp(
                s_stop_reference_mm + 0.5f, 0.0f, 99.0f),
            2U);
        Task2_DrawText(40U, 48U, " B:");
        Task2_DrawDigits(
            64U, 48U,
            (uint32_t)Task2_Clamp(
                s_stop_brake_compensation_mm + 0.5f,
                0.0f, 99.0f),
            2U);
    } else if (s_task4_mode != 0U) {
        /* Live ball error and travelled distance while TASK4 is running. */
        Task2_DrawText(0U, 48U, "B:");
        Task2_DrawSigned3(16U, 48U, s_task4_ball_error_mm);
        Task2_DrawText(48U, 48U, " D:");
        Task2_DrawDigits(
            72U, 48U,
            (uint32_t)Task2_Clamp(Task2_OdometryMm(), 0.0f, 9999.0f),
            4U);
    } else {
        Task2_DrawText(0U, 48U, "MM:");
        Task2_DrawDigits(
            24U, 48U,
            (uint32_t)Task2_Clamp(Task2_OdometryMm(), 0.0f, 9999.0f),
            4U);
    }
    OLED_Refresh_Gram();
}

static void Task2_UpdateImu(uint32_t now_ms, uint8_t stationary)
{
#if TASK2_MPU6050_ENABLE == 1U
    if (s_imu_detected == 0U) {
        s_imu_online = 0U;
        s_gyro_rate_z_dps = 0.0f;
        return;
    }

    if ((uint32_t)(now_ms - s_last_imu_update_ms) >=
        TASK2_MPU_UPDATE_PERIOD_MS) {
        s_last_imu_update_ms = now_ms;
        if (MPU6050_Update(stationary) != 0U) {
            s_last_imu_success_ms = now_ms;
            s_gyro_rate_z_dps = mpu6050_status.yaw_rate;
        }
    }

    s_imu_online =
        ((mpu6050_status.calibrated != 0U) &&
         ((uint32_t)(now_ms - s_last_imu_success_ms) <=
          TASK2_MPU_ONLINE_TIMEOUT_MS)) ? 1U : 0U;
    if (s_imu_online == 0U) {
        s_gyro_rate_z_dps = 0.0f;
        return;
    }
#else
    (void)now_ms;
    (void)stationary;
    s_imu_online = 0U;
    s_gyro_rate_z_dps = 0.0f;
#endif
}

static void Task2_SetWheelTargetsMm(float left_mm_s, float right_mm_s)
{
    left_mm_s = Task2_Clamp(
        left_mm_s, 0.0f, TASK2_WHEEL_REF_MAX_MM_S);
    right_mm_s = Task2_Clamp(
        right_mm_s, 0.0f, TASK2_WHEEL_REF_MAX_MM_S);
    MotorA.Target_Encoder = left_mm_s * 0.001f;
    MotorB.Target_Encoder = right_mm_s * 0.001f;
}

static void Task2_RampBaseSpeed(uint32_t now_ms, float target_mm_s)
{
    uint32_t delta_ms = (uint32_t)(now_ms - s_last_ramp_ms);
    float rate_mm_s2;
    float maximum_change;

    if (s_last_ramp_ms == 0U) {
        s_last_ramp_ms = now_ms;
        return;
    }
    if (delta_ms == 0U) {
        return;
    }
    if (delta_ms > 20U) {
        delta_ms = 20U;
    }
    s_last_ramp_ms = now_ms;

    if (s_task4_mode != 0U) {
        rate_mm_s2 =
            (target_mm_s > s_base_command_mm_s) ?
                TASK4_ACCEL_LIMIT_MM_S2 :
                TASK4_DECEL_LIMIT_MM_S2;
    }
    else {
        rate_mm_s2 =
            (target_mm_s > s_base_command_mm_s) ?
                TASK2_ACCEL_LIMIT_MM_S2 :
                TASK2_DECEL_LIMIT_MM_S2;
    }
    maximum_change = rate_mm_s2 * (float)delta_ms * 0.001f;
    if (target_mm_s > s_base_command_mm_s) {
        s_base_command_mm_s += maximum_change;
        if (s_base_command_mm_s > target_mm_s) {
            s_base_command_mm_s = target_mm_s;
        }
    } else {
        s_base_command_mm_s -= maximum_change;
        if (s_base_command_mm_s < target_mm_s) {
            s_base_command_mm_s = target_mm_s;
        }
    }
    BaseSpeed = s_base_command_mm_s;
}

static uint8_t Task2_IsCenteredGuide(void)
{
    uint8_t count = IR_CountBlack(IR_GetBlackMask());

    return ((ir_line_valid != 0U) &&
            (count >= 1U) && (count <= 4U) &&
            (Task2_Abs(ir_line_position) <=
             TASK2_ERROR_INNER_MM)) ? 1U : 0U;
}

static void Task2_EnterFault(
    Task2Fault reason, uint32_t now_ms)
{
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    Flag_Stop = 1;
    Control_ResetSpeedControllers();
    s_final_time_ms = (uint32_t)(now_ms - s_start_ms);
    s_fault = reason;
    s_state = TASK2_FAULT;
    Task2_UpdateDisplay(now_ms, 1U);
}

static void Task2_Start(uint32_t now_ms)
{
    uint8_t start_marker_mask = IR_GetBlackMask();
    float start_speed_mm_s =
        (s_task4_mode != 0U) ?
            TASK4_START_SPEED_MM_S :
            TASK2_START_SPEED_MM_S;
    float initial_speed_mm_s =
        (s_task4_mode != 0U) ? 0.0f : start_speed_mm_s;

    Control_ResetOdometry();
    Control_ResetSpeedControllers();
    IR_LineReset();
    IR_SetMotorSteeringSign(1.0f); /* MotorA verified as left wheel. */
    s_yaw_reference_set = 0U;
    if (s_imu_online != 0U) {
        MPU6050_ResetYaw(0.0f);
        s_yaw_reference_set = 1U;
    }

    s_start_ms = now_ms;
    s_final_time_ms = 0U;
    s_starting_ms = now_ms;
    s_guide_candidate_ms = 0U;
    s_line_lost_ms = 0U;
    s_line_recover_ms = 0U;
    s_creep_start_ms = 0U;
    s_brake_start_ms = 0U;
    s_stall_start_ms = 0U;
    s_mismatch_start_ms = 0U;
    s_cross_distance_mm = 0.0f;
    s_start_reference_mm = TASK2_STOP_SENSOR_TO_REFERENCE_MM;
    s_stop_reference_mm = TASK2_STOP_SENSOR_TO_REFERENCE_MM;
    s_stop_learned_reference_mm = TASK2_STOP_SENSOR_TO_REFERENCE_MM;
    s_stop_brake_compensation_mm = 0.0f;
    s_curve_blend = 0.0f;
    s_curve_feedforward_mm_s = 0.0f;
    s_base_command_mm_s = initial_speed_mm_s;
    s_last_ramp_ms = now_ms;
    s_marker_armed = 0U;
    s_finish_window = 0U;
    s_finish_heading_ok = 0U;
    s_finish_distance_ok = 0U;
    s_marker_candidate = 0U;
    s_marker_reject_reason = 0U;
    s_start_marker_phase = 0U;
    s_start_reference_valid = 0U;
    s_start_marker_mask = start_marker_mask;
    s_finish_match_count = 0U;
    s_fault = TASK2_FAULT_NONE;
    s_state = TASK2_STARTING;
    BaseSpeed = s_base_command_mm_s;
    Task2_SetWheelTargetsMm(
        initial_speed_mm_s, initial_speed_mm_s);
    Flag_Stop = 0;
    Task2_UpdateDisplay(now_ms, 1U);
}

static void Task2_BeginBrake(uint32_t now_ms)
{
    s_state = TASK2_BRAKING;
    s_brake_start_ms = now_ms;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    Control_ResetSpeedControllers();
}


static void Task2_UpdateBrake(uint32_t now_ms)
{
    uint32_t brake_ms = (uint32_t)(now_ms - s_brake_start_ms);
    uint8_t stopped =
        ((Task2_Abs(left_speed_meas) <=
          TASK2_STOP_SPEED_THRESHOLD_MM_S) &&
         (Task2_Abs(right_speed_meas) <=
          TASK2_STOP_SPEED_THRESHOLD_MM_S)) ? 1U : 0U;

    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    if (((brake_ms >= TASK2_STOP_SETTLE_MIN_MS) &&
         (stopped != 0U)) ||
        (brake_ms >= TASK2_STOP_SETTLE_MAX_MS)) {
        Flag_Stop = 1;
        Control_ResetSpeedControllers();
        if ((s_task4_mode == 0U) ||
            (s_final_time_ms == 0U)) {
            s_final_time_ms =
                (uint32_t)(now_ms - s_start_ms);
        }
        s_state = TASK2_STOPPED;
        Task2_UpdateDisplay(now_ms, 1U);
    }
}

static uint8_t Task2_CheckSafety(uint32_t now_ms)
{
    float left_ref_abs = Task2_Abs(left_speed_ref);
    float right_ref_abs = Task2_Abs(right_speed_ref);
    float tracking_difference =
        Task2_Abs((left_speed_ref - left_speed_meas) -
                  (right_speed_ref - right_speed_meas));
    uint8_t stall_now =
        (((left_ref_abs >= TASK2_STALL_TARGET_MIN_MM_S) &&
          (Task2_Abs(left_speed_meas) <=
           TASK2_STALL_MEASURED_MAX_MM_S)) ||
         ((right_ref_abs >= TASK2_STALL_TARGET_MIN_MM_S) &&
          (Task2_Abs(right_speed_meas) <=
           TASK2_STALL_MEASURED_MAX_MM_S))) ? 1U : 0U;

    if (stall_now != 0U) {
        if (s_stall_start_ms == 0U) {
            s_stall_start_ms = now_ms;
        } else if ((uint32_t)(now_ms - s_stall_start_ms) >=
                   TASK2_STALL_TIMEOUT_MS) {
            Task2_EnterFault(
                TASK2_FAULT_ENCODER_STALL, now_ms);
            return 1U;
        }
    } else {
        s_stall_start_ms = 0U;
    }

    if (tracking_difference >= TASK2_WHEEL_MISMATCH_MAX_MM_S) {
        if (s_mismatch_start_ms == 0U) {
            s_mismatch_start_ms = now_ms;
        } else if ((uint32_t)(now_ms - s_mismatch_start_ms) >=
                   TASK2_WHEEL_MISMATCH_TIMEOUT_MS) {
            Task2_EnterFault(
                TASK2_FAULT_WHEEL_MISMATCH, now_ms);
            return 1U;
        }
    } else {
        s_mismatch_start_ms = 0U;
    }
    return 0U;
}

static void Task2_UpdateStarting(uint32_t now_ms)
{
    float odometry_mm;
    float start_speed_mm_s =
        (s_task4_mode != 0U) ?
            TASK4_START_SPEED_MM_S :
            TASK2_START_SPEED_MM_S;

    if ((s_yaw_reference_set == 0U) &&
        (s_imu_online != 0U)) {
        MPU6050_ResetYaw(0.0f);
        s_yaw_reference_set = 1U;
    }
    IR_Module_Update(
        s_gyro_rate_z_dps, s_imu_online, 0.0f);
    if (s_task4_mode != 0U) {
        Task2_RampBaseSpeed(now_ms, start_speed_mm_s);
    }
    else {
        s_base_command_mm_s = start_speed_mm_s;
    }
    BaseSpeed = s_base_command_mm_s;
    base_speed_mm = s_base_command_mm_s;
    ir_line_turn_diff = 0.0f;
    turn_diff = 0.0f;
    Task2_SetWheelTargetsMm(
        s_base_command_mm_s, s_base_command_mm_s);

    /*
     * Reconfirm the wide marker after IR_LineReset(), then capture the first
     * confirmed transition from the A marker to the normal guide. This is the
     * actual centre-to-edge distance for the current placement and track.
     */
    odometry_mm = Task2_OdometryMm();
    if (ir_front_cross_detected != 0U) {
        if (s_start_marker_phase == 0U) {
            s_start_marker_phase = 1U;
        }
    } else if (s_start_marker_phase == 1U) {
        if ((odometry_mm >= TASK2_STOP_REFERENCE_MIN_MM) &&
            (odometry_mm <= TASK2_STOP_REFERENCE_MAX_MM)) {
            s_start_reference_mm = odometry_mm;
            s_start_reference_valid = 1U;
        }
        s_start_marker_phase = 2U;
    }

    if (Task2_IsCenteredGuide() != 0U) {
        if (s_guide_candidate_ms == 0U) {
            s_guide_candidate_ms = now_ms;
        } else if ((uint32_t)(now_ms - s_guide_candidate_ms) >=
                   TASK2_START_GUIDE_CONFIRM_MS) {
            s_state = TASK2_RUNNING;
            if (s_task4_mode == 0U) {
                s_base_command_mm_s = start_speed_mm_s;
            }
            s_guide_candidate_ms = 0U;
            IR_LineReacquire();
        }
    } else {
        s_guide_candidate_ms = 0U;
    }

    if ((s_state == TASK2_STARTING) &&
        ((uint32_t)(now_ms - s_starting_ms) >=
         ((s_task4_mode != 0U) ?
          TASK4_START_LEAVE_TIMEOUT_MS :
          TASK2_START_LEAVE_TIMEOUT_MS))) {
        Task2_EnterFault(TASK2_FAULT_START_LINE, now_ms);
    }
}

static void Task2_EnterLostLine(uint32_t now_ms)
{
    s_lost_resume_state = s_state;
    s_state = TASK2_LOST_LINE;
    s_line_lost_ms = now_ms;
    s_line_recover_ms = 0U;
    s_base_command_mm_s = TASK2_SEARCH_SPEED_MM_S;
    BaseSpeed = s_base_command_mm_s;
}

static void Task2_UpdateLostLine(uint32_t now_ms)
{
    float odometry_mm = Task2_OdometryMm();
    uint32_t lost_timeout_ms = TASK2_LOST_TIMEOUT_MS;

    Task2_RampBaseSpeed(now_ms, TASK2_SEARCH_SPEED_MM_S);
    /*
     * Keep the known clockwise curvature while the front row is briefly off
     * the line.  Clearing this feedforward made the vehicle drive almost
     * straight in the second semicircle and guaranteed a line-timeout.
     */
    s_curve_blend =
        (s_task4_mode != 0U) ? 0.0f : Task2_CurveBlend(odometry_mm);
    s_curve_feedforward_mm_s = Task2_CurveFeedforward(
        s_base_command_mm_s, s_curve_blend, odometry_mm);
    IR_Module_Update(
        s_gyro_rate_z_dps,
        s_imu_online,
        s_curve_feedforward_mm_s);

    if ((s_task4_mode == 0U) &&
        (Task2_IsSecondCurveWindow(odometry_mm) != 0U)) {
        lost_timeout_ms = TASK2_SECOND_CURVE_LOST_TIMEOUT_MS;
    }

    if (ir_line_valid != 0U) {
        if (s_line_recover_ms == 0U) {
            s_line_recover_ms = now_ms;
        } else if ((uint32_t)(now_ms - s_line_recover_ms) >=
                   TASK2_RECOVER_CONFIRM_MS) {
            s_state = s_lost_resume_state;
            s_line_lost_ms = 0U;
            s_line_recover_ms = 0U;
            IR_LineReacquire();
        }
    } else {
        s_line_recover_ms = 0U;
    }

    if ((s_state == TASK2_LOST_LINE) &&
        ((uint32_t)(now_ms - s_line_lost_ms) >=
         lost_timeout_ms)) {
        Task2_EnterFault(
            TASK2_FAULT_LINE_TIMEOUT, now_ms);
    }
}

static void Task2_UpdateLineMotion(uint32_t now_ms)
{
    float odometry_mm = Task2_OdometryMm();
    float route_speed_mm_s;
    float curve_speed_mm_s = TASK2_CURVE_SPEED_MM_S;
    float target_speed_mm_s;
    uint32_t minimum_lap_time_ms = TASK2_MIN_LAP_TIME_MS;
    uint8_t adaptive_marker_now;
    uint8_t adaptive_marker_confirmed;
    uint8_t finish_qualified;

    if ((s_task4_mode != 0U) &&
        (s_state == TASK2_RUNNING) &&
        (odometry_mm >= TASK4_AB_DISTANCE_MM)) {
        /* Freeze the scored AB time exactly when the reference passes B. */
        s_final_time_ms =
            (uint32_t)(now_ms - s_start_ms);
        s_state = TASK2_PASS_B;
    }

    if ((s_task4_mode != 0U) &&
        (s_state == TASK2_PASS_B) &&
        (odometry_mm >=
         (TASK4_AB_DISTANCE_MM +
          TASK4_PASS_B_MARGIN_MM))) {
        Task2_BeginBrake(now_ms);
        return;
    }

    /* AB is straight; curve feedforward starts only after B was passed. */
    s_curve_blend =
        ((s_task4_mode != 0U) &&
         (s_state != TASK2_PASS_B)) ?
        0.0f : Task2_CurveBlend(odometry_mm);
    if ((s_task4_mode == 0U) &&
        (Task2_IsSecondCurveWindow(odometry_mm) != 0U)) {
        curve_speed_mm_s = TASK2_SECOND_CURVE_SPEED_MM_S;
    }
    route_speed_mm_s = TASK2_STRAIGHT_SPEED_MM_S +
        s_curve_blend *
            (curve_speed_mm_s -
             TASK2_STRAIGHT_SPEED_MM_S);

    if ((s_task4_mode == 0U) &&
        (s_state == TASK2_RUNNING) &&
        (odometry_mm >= TASK2_APPROACH_DISTANCE_MM)) {
        s_state = TASK2_APPROACH_STOP;
    }

    if (s_task4_mode != 0U) {
        target_speed_mm_s =
            (s_state == TASK2_PASS_B) ?
            TASK4_POST_B_SPEED_MM_S :
            TASK4_CRUISE_SPEED_MM_S;

        if ((s_state != TASK2_PASS_B) &&
            (s_task4_ball_vision_hold != 0U)) {
            target_speed_mm_s = TASK4_BALL_HOLD_SPEED_MM_S;
        }
        else if (s_state != TASK2_PASS_B) {
            float ball_error_abs =
                Task2_Abs(s_task4_ball_error_mm);

            if (ball_error_abs >=
                TASK4_BALL_RECOVERY_ERROR_MM) {
                target_speed_mm_s =
                    TASK4_BALL_RECOVERY_SPEED_MM_S;
            }
            else if (ball_error_abs >
                     TASK4_BALL_SLOW_START_MM) {
                float blend =
                    (ball_error_abs -
                     TASK4_BALL_SLOW_START_MM) /
                    (TASK4_BALL_RECOVERY_ERROR_MM -
                     TASK4_BALL_SLOW_START_MM);

                target_speed_mm_s =
                    TASK4_CRUISE_SPEED_MM_S +
                    blend *
                    (TASK4_BALL_RECOVERY_SPEED_MM_S -
                     TASK4_CRUISE_SPEED_MM_S);
            }
        }
    }
    else {
        target_speed_mm_s =
            (s_state == TASK2_APPROACH_STOP) ?
            ((odometry_mm >= TASK2_FINAL_APPROACH_DISTANCE_MM) ?
                TASK2_APPROACH_SPEED_MM_S :
                TASK2_PREAPPROACH_SPEED_MM_S) :
#if TASK2_STOP_LINE_TEST == 1U
            TASK2_APPROACH_SPEED_MM_S;
#elif TASK2_FRONT_LINE_ONLY_TEST == 1U
            TASK2_DEBUG_SPEED_MM_S;
#else
            route_speed_mm_s;
#endif
    }
    Task2_RampBaseSpeed(now_ms, target_speed_mm_s);
    s_curve_feedforward_mm_s = Task2_CurveFeedforward(
        s_base_command_mm_s, s_curve_blend, odometry_mm);
    IR_Module_Update(
        s_gyro_rate_z_dps,
        s_imu_online,
        s_curve_feedforward_mm_s);

    if (s_task4_mode != 0U) {
        if ((ir_lost_sample_count >=
             TASK2_LOST_CONFIRM_SAMPLES) &&
            (ir_front_cross_detected == 0U)) {
            Task2_EnterLostLine(now_ms);
        }
        return;
    }

    if ((s_marker_armed == 0U) &&
        (odometry_mm >= TASK2_MARKER_ARM_DISTANCE_MM) &&
        (ir_front_cross_detected == 0U)) {
        s_marker_armed = 1U;
    }

#if TASK2_STOP_LINE_TEST == 1U
    minimum_lap_time_ms = TASK2_STOP_TEST_MIN_TIME_MS;
    s_finish_window =
        (odometry_mm >= TASK2_STOP_TEST_MIN_DISTANCE_MM) ? 1U : 0U;
#else
    s_finish_window =
        ((odometry_mm >= TASK2_FINISH_WINDOW_START_MM) &&
         (odometry_mm <= TASK2_FINISH_WINDOW_END_MM)) ? 1U : 0U;
#endif

    adaptive_marker_now = Task2_IsStartMarkerMatch(IR_GetBlackMask());
    if (adaptive_marker_now != 0U) {
        if (s_finish_match_count <
            TASK2_FINISH_MATCH_CONFIRM_SAMPLES) {
            s_finish_match_count++;
        }
    }
    else {
        s_finish_match_count = 0U;
    }
    adaptive_marker_confirmed =
        (s_finish_match_count >=
         TASK2_FINISH_MATCH_CONFIRM_SAMPLES) ? 1U : 0U;
    s_marker_candidate =
        ((ir_front_cross_detected != 0U) ||
         (adaptive_marker_confirmed != 0U)) ? 1U : 0U;

    if ((s_imu_online != 0U) &&
        (s_yaw_reference_set != 0U)) {
        s_finish_heading_ok =
            (Task2_Abs(mpu6050.yaw) <=
             TASK2_FINISH_HEADING_LIMIT_DEG) ? 1U : 0U;
        s_finish_distance_ok =
            ((odometry_mm >= TASK2_MIN_LAP_DISTANCE_MM) &&
             ((s_finish_window != 0U) ||
              ((odometry_mm >= TASK2_FINISH_OFFLINE_MIN_MM) &&
               (s_finish_heading_ok != 0U)))) ? 1U : 0U;
    } else {
        s_finish_heading_ok = 1U;
        s_finish_distance_ok =
            (odometry_mm >=
             TASK2_FINISH_OFFLINE_MIN_MM) ? 1U : 0U;
    }

    s_marker_reject_reason = 0U;
    if (s_marker_candidate != 0U) {
        if (s_marker_armed == 0U) {
            s_marker_reject_reason |=
                TASK2_MARKER_REJECT_NOT_ARMED;
        }
        if (Task2_CurrentElapsedMs(now_ms) <
            minimum_lap_time_ms) {
            s_marker_reject_reason |=
                TASK2_MARKER_REJECT_TIME;
        }
        if (s_finish_distance_ok == 0U) {
            s_marker_reject_reason |=
                TASK2_MARKER_REJECT_WINDOW;
        }
        if ((s_finish_window == 0U) &&
            (s_finish_heading_ok == 0U)) {
            s_marker_reject_reason |=
                TASK2_MARKER_REJECT_HEADING;
        }
        if (((s_imu_online == 0U) ||
             (s_yaw_reference_set == 0U)) &&
            (odometry_mm < TASK2_FINISH_OFFLINE_MIN_MM)) {
            s_marker_reject_reason |=
                TASK2_MARKER_REJECT_OFFLINE_DISTANCE;
        }
    }

    finish_qualified =
        ((s_marker_candidate != 0U) &&
         (s_marker_reject_reason == 0U)) ? 1U : 0U;

    if (finish_qualified != 0U) {
        s_state = TASK2_FINAL_CREEP;
        s_cross_distance_mm = odometry_mm;
        s_stop_learned_reference_mm = Task2_StopReferenceMm();
        s_stop_brake_compensation_mm =
            Task2_StopBrakeCompensationMm();
        s_stop_reference_mm = Task2_Clamp(
            s_stop_learned_reference_mm -
                s_stop_brake_compensation_mm,
            0.0f,
            TASK2_STOP_TARGET_MAX_MM);
        s_creep_start_ms = now_ms;
        s_base_command_mm_s =
            (s_base_command_mm_s < TASK2_FINAL_CREEP_SPEED_MM_S) ?
                s_base_command_mm_s :
                TASK2_FINAL_CREEP_SPEED_MM_S;
        if ((TASK2_STOP_POSITION_CORRECTION == 0U) ||
            (s_stop_reference_mm <= 0.0f)) {
            Task2_BeginBrake(now_ms);
        }
        return;
    }

    if ((ir_lost_sample_count >= TASK2_LOST_CONFIRM_SAMPLES) &&
        (ir_front_cross_detected == 0U)) {
        Task2_EnterLostLine(now_ms);
    }
}

static void Task2_UpdateFinalCreep(uint32_t now_ms)
{
    float odometry_mm = Task2_OdometryMm();
    float creep_distance_mm =
        odometry_mm - s_cross_distance_mm;

    Task2_RampBaseSpeed(now_ms, TASK2_FINAL_CREEP_SPEED_MM_S);
    s_curve_blend = Task2_CurveBlend(odometry_mm);
    s_curve_feedforward_mm_s = Task2_CurveFeedforward(
        s_base_command_mm_s, s_curve_blend, odometry_mm);
    IR_Module_Update(
        s_gyro_rate_z_dps,
        s_imu_online,
        s_curve_feedforward_mm_s);
    ir_line_turn_diff = Task2_Clamp(
        ir_line_turn_diff,
        -TASK2_FINAL_TURN_MAX_MM_S,
        TASK2_FINAL_TURN_MAX_MM_S);
    IR_ApplyYawRateTracking(
        s_gyro_rate_z_dps, s_imu_online);

    if (creep_distance_mm >= s_stop_reference_mm) {
        Task2_BeginBrake(now_ms);
    } else if ((uint32_t)(now_ms - s_creep_start_ms) >=
               TASK2_FINAL_CREEP_TIMEOUT_MS) {
        Task2_EnterFault(
            TASK2_FAULT_CREEP_TIMEOUT, now_ms);
    }
}


void LapTest_SetTask4Mode(uint8_t enabled)
{
    s_task4_mode = (enabled != 0U) ? 1U : 0U;
    s_external_start_ready = 1U;
    s_task4_ball_error_mm = 0.0f;
    s_task4_ball_vision_hold = 0U;
    s_task4_external_fault = 0U;
    IR_SetSearchTurnMmS(
        (s_task4_mode != 0U) ?
            TASK2_SEARCH_TURN_MM_S :
            TASK2_LAP_SEARCH_TURN_MM_S);
}


void LapTest_SetExternalStartReady(uint8_t ready)
{
    s_external_start_ready = (ready != 0U) ? 1U : 0U;
}


void LapTest_SetTask4BallErrorMm(int16_t error_mm)
{
    s_task4_ball_error_mm = (float)error_mm;
}


void LapTest_SetTask4BallVisionHold(uint8_t held)
{
    s_task4_ball_vision_hold = (held != 0U) ? 1U : 0U;
}


void LapTest_SetTask4ExternalFault(uint8_t fault)
{
    s_task4_external_fault = fault;
}

void LapTest_Init(void)
{
    uint32_t now_ms;

    s_state = TASK2_IDLE;
#if TASK2_MPU6050_ENABLE == 1U
    OLED_Clear();
    OLED_ShowString(
        0U, 0U, (const uint8_t *)"MPU CALIBRATE");
    OLED_ShowString(
        0U, 24U, (const uint8_t *)"KEEP CAR STILL");
    s_imu_detected = MPU6050_initialize();
#else
    s_imu_detected = 0U;
#endif
    s_imu_online = s_imu_detected;
    s_yaw_reference_set = 0U;
    s_last_imu_update_ms = Board_GetMillis();
    s_last_imu_success_ms =
        (s_imu_detected != 0U) ? s_last_imu_update_ms : 0U;
    s_gyro_rate_z_dps = 0.0f;

    Flag_Stop = 1;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    Control_ResetOdometry();
    Control_ResetSpeedControllers();
    IR_LineReset();
    IR_SetDiagnosticMode(0U);
    IR_SetMotorSteeringSign(1.0f);

    now_ms = Board_GetMillis();
    s_state = TASK2_READY;
    s_fault = TASK2_FAULT_NONE;
    s_start_ms = now_ms;
    s_final_time_ms = 0U;
    s_last_line_ms = now_ms;
    s_last_state_ms = now_ms;
    s_last_display_ms = 0U;
    s_last_ramp_ms = now_ms;
    s_base_command_mm_s = 0.0f;
    s_start_reference_mm = TASK2_STOP_SENSOR_TO_REFERENCE_MM;
    s_stop_reference_mm = TASK2_STOP_SENSOR_TO_REFERENCE_MM;
    s_stop_learned_reference_mm = TASK2_STOP_SENSOR_TO_REFERENCE_MM;
    s_stop_brake_compensation_mm = 0.0f;
    s_curve_blend = 0.0f;
    s_curve_feedforward_mm_s = 0.0f;
    s_marker_armed = 0U;
    s_finish_window = 0U;
    s_finish_heading_ok = 0U;
    s_finish_distance_ok = 0U;
    s_marker_candidate = 0U;
    s_marker_reject_reason = 0U;
    s_start_marker_phase = 0U;
    s_start_reference_valid = 0U;
    s_start_marker_mask = 0U;
    s_finish_match_count = 0U;
    s_previous_stop = 1U;
    s_task4_ball_vision_hold = 0U;
    s_task4_external_fault = 0U;
    (void)IR_Module_Read();
    Task2_UpdateDisplay(now_ms, 1U);
}

void LapTest_Update(void)
{
    uint32_t now_ms = Board_GetMillis();
    uint8_t stopped_state =
        ((s_state == TASK2_READY) ||
         (s_state == TASK2_STOPPED) ||
         (s_state == TASK2_FAULT)) ? 1U : 0U;

    Task2_UpdateImu(now_ms, stopped_state);

    if (stopped_state != 0U) {
        if ((uint32_t)(now_ms - s_last_state_ms) >=
            TASK2_STATE_PERIOD_MS) {
            s_last_state_ms = now_ms;
            IR_Module_Read();
        }

        /*
         * 启动逻辑：当 Flag_Stop 被按键翻转为 0 时，等待交叉标志确认后起步。
         * 不再因为交叉暂时为 0 而立即复位 Flag_Stop —— 用户不需要反复按。
         * 按一次 = 请求启动，交叉满足 = 实际起步。再按一次 = 取消。
         */
        if (Flag_Stop == 0) {
            if ((s_external_start_ready != 0U) &&
                (ir_front_cross_detected != 0U)) {
                Task2_Start(now_ms);
            }
            /* else: keep waiting, Flag_Stop stays 0 */
        } else {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
        }
        Task2_UpdateDisplay(now_ms, 0U);
        s_previous_stop = (Flag_Stop != 0) ? 1U : 0U;
        return;
    }

    if ((s_task4_mode != 0U) &&
        (s_task4_external_fault != 0U)) {
        Task2_EnterFault(
            (s_task4_external_fault == 1U) ?
                TASK2_FAULT_BALL_LOST : TASK2_FAULT_X42S,
            now_ms);
        s_previous_stop = 1U;
        return;
    }

    if (Flag_Stop != 0) {
        Task2_EnterFault(TASK2_FAULT_ABORTED, now_ms);
        s_previous_stop = 1U;
        return;
    }

    if (Task2_CurrentElapsedMs(now_ms) >=
        ((s_task4_mode != 0U) ?
         TASK4_MAX_RUN_TIME_MS :
         TASK2_MAX_RUN_TIME_MS)) {
        Task2_EnterFault(TASK2_FAULT_RUN_TIMEOUT, now_ms);
        s_previous_stop = 1U;
        return;
    }

    if (s_state == TASK2_BRAKING) {
        Task2_UpdateBrake(now_ms);
    } else if ((uint32_t)(now_ms - s_last_line_ms) >=
               TASK2_LINE_PERIOD_MS) {
        s_last_line_ms = now_ms;
        switch (s_state) {
        case TASK2_STARTING:
            Task2_UpdateStarting(now_ms);
            break;
        case TASK2_RUNNING:
        case TASK2_PASS_B:
        case TASK2_APPROACH_STOP:
            Task2_UpdateLineMotion(now_ms);
            break;
        case TASK2_FINAL_CREEP:
            Task2_UpdateFinalCreep(now_ms);
            break;
        case TASK2_LOST_LINE:
            Task2_UpdateLostLine(now_ms);
            break;
        default:
            break;
        }
    }

    if ((s_state != TASK2_FAULT) &&
        (s_state != TASK2_BRAKING) &&
        (s_state != TASK2_STARTING)) {
        (void)Task2_CheckSafety(now_ms);
    }

    Task2_UpdateDisplay(now_ms, 0U);
    s_previous_stop = (Flag_Stop != 0) ? 1U : 0U;
}

LapTestState LapTest_GetState(void)
{
    return s_state;
}

uint32_t LapTest_GetElapsedMs(void)
{
    return Task2_CurrentElapsedMs(Board_GetMillis());
}

Task2Fault LapTest_GetFault(void)
{
    return s_fault;
}
