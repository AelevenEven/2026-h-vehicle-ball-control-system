#ifndef TASK2_CONFIG_H
#define TASK2_CONFIG_H

/*
 * 2026 H-problem item 2 configuration.
 *
 * Units:
 *   distance / line error / wheel command: mm or mm/s
 *   acceleration: mm/s^2
 *   time: ms
 *   gyro rate: deg/s
 *
 * X1 is the leftmost front probe and X8 is the rightmost probe when viewed
 * in the vehicle's forward direction. Positive line error means that the
 * black line is to the vehicle's left.
 */

/* Build and debug selection. */
#define TASK2_DEBUG_CSV_ENABLE             (0U)
#define TASK2_DEBUG_PERIOD_MS              (50U)
#define TASK2_DISPLAY_PERIOD_MS            (50U)
#define TASK2_FRONT_LINE_ONLY_TEST         (0U)
#define TASK2_STOP_LINE_TEST               (0U)
#define TASK2_FULL_TEST                    (1U)

/* Tether-free RAM black-box and stopped-state CSV replay. */
#define TASK2_BLACKBOX_ENABLE              (0U)
#define TASK2_BLACKBOX_MAX_SAMPLES         (400U)
#define TASK2_BLACKBOX_SAMPLE_PERIOD_MS    (50U)
#define TASK2_BLACKBOX_REPLAY_DELAY_MS     (1000U)
#define TASK2_BLACKBOX_REPLAY_ROW_MS       (20U)
#define TASK2_BLACKBOX_REPLAY_REPEAT_MS    (3000U)

/* Optional rear four-probe row. No rear pins exist in the current SysConfig. */
#define LINE_REAR_ENABLE                   (0U)
#define TASK2_REAR_POSITION_MM             {18.0f, 6.0f, -6.0f, -18.0f}
#define TASK2_REAR_CENTER_OFFSET_MM        (0.0f)
#define TASK2_HEADING_KH                   (0.0f)

/* Front CD4051 digital sensor interface. */
#define TASK2_FRONT_BLACK_ACTIVE_LEVEL     (1U)
#define TASK2_FRONT_ORDER_REVERSED         (0U)
#define TASK2_FRONT_CHANNEL_SETTLE_US      (100U)
#define TASK2_FRONT_POSITION_MM            \
    {35.0f, 25.0f, 15.0f, 5.0f, -5.0f, -15.0f, -25.0f, -35.0f}
#define TASK2_FRONT_CENTER_OFFSET_MM       (0.0f)
#define TASK2_FRONT_TEMPORAL_FILTER        (1U)
#define TASK2_SPECIAL_CONFIRM_SAMPLES      (3U)

/* Main-loop scheduling. Wheel PI remains on the existing 10 ms timer. */
#define TASK2_LINE_PERIOD_MS               (5U)
#define TASK2_STATE_PERIOD_MS              (10U)

/* Front-line PD controller. No line integral term is used.
 * 振荡根因是增益过大，不是死区设计问题。
 * 大幅降低 Kp，靠微分阻尼 + 低增益自然稳定。
 */
#define TASK2_LINE_ERROR_ALPHA             (0.25f)
#define TASK2_LINE_DERIVATIVE_ALPHA        (0.25f)
#define TASK2_LINE_DERIVATIVE_LIMIT_MM_S   (400.0f)
#define TASK2_LINE_CENTER_ENTER_MM         (6.0f)
#define TASK2_LINE_CENTER_EXIT_MM          (10.0f)
#define TASK2_LINE_KP_INNER                (0.20f)
#define TASK2_LINE_KP_OUTER                (3.5f)
#define TASK2_LINE_KP_OUTER_ERROR_MM       (18.0f)
#define TASK2_LINE_KD                      (0.150f)
#define TASK2_TURN_MAX_MM_S                (380.0f)
#define TASK2_TURN_RATE_MM_S_PER_TICK      (5.0f)

/* MPU6050 follows the infrared/feedforward yaw-rate target. */
#define TASK2_MPU6050_ENABLE               (1U)
#define TASK2_WIT_GYRO_ENABLE              (0U)
#define TASK2_GYRO_Z_SIGN                  (1.0f)
#define TASK2_GYRO_RATE_K_MM_S_PER_DPS     (0.22f)
#define TASK2_GYRO_CORRECTION_LIMIT_MM_S   (18.0f)
#define TASK2_MPU_UPDATE_PERIOD_MS         (5U)
#define TASK2_MPU_ONLINE_TIMEOUT_MS        (50U)
#define TASK2_WHEEL_TRACK_MM               (150.0f)
#define TASK2_RAD_TO_DEG                   (57.2957795f)

/* Speed planning — 重车全线降速 */
#define TASK2_DEBUG_SPEED_MM_S             (120.0f)
#define TASK2_START_SPEED_MM_S             (200.0f)
#define TASK2_STRAIGHT_SPEED_MM_S          (420.0f)
#define TASK2_INNER_SPEED_MM_S             (380.0f)
#define TASK2_MIDDLE_SPEED_MM_S            (340.0f)
#define TASK2_CURVE_SPEED_MM_S             (280.0f)
#define TASK2_SEARCH_SPEED_MM_S            (100.0f)
#define TASK2_SEARCH_TURN_MM_S             (80.0f)
#define TASK2_LAP_SEARCH_TURN_MM_S         (110.0f)
/*
 * Two-stage finish approach.  The first stage removes most of the kinetic
 * energy without adding several seconds to the lap.  The second stage makes
 * the A-line leading edge repeatable before the position correction starts.
 */
#define TASK2_PREAPPROACH_SPEED_MM_S       (220.0f)
#define TASK2_APPROACH_SPEED_MM_S          (80.0f)
#define TASK2_ACCEL_LIMIT_MM_S2            (450.0f)
#define TASK2_DECEL_LIMIT_MM_S2            (1000.0f)
#define TASK2_WHEEL_REF_MAX_MM_S           (600.0f)
#define TASK2_SPEED_LIMIT_RATE_PER_TICK     (10.0f)
#define TASK2_YAW_SPEED_START_DPS           (10.0f)
#define TASK2_YAW_SPEED_FULL_DPS            (35.0f)

/* Error breakpoints for continuous speed scheduling. */
#define TASK2_ERROR_INNER_MM               (10.0f)
#define TASK2_ERROR_MIDDLE_MM              (22.0f)
#define TASK2_ERROR_OUTER_MM               (32.0f)

/* A-line and lap qualification. */
#define TASK2_FRONT_CROSS_MIN_COUNT        (4U)
#define TASK2_FRONT_CROSS_MIN_RUN          (3U)
#define TASK2_FINISH_MATCH_MIN_COUNT       (3U)
#define TASK2_FINISH_MATCH_MIN_OVERLAP     (3U)
#define TASK2_FINISH_MATCH_CONFIRM_SAMPLES (2U)
#define TASK2_MARKER_ARM_DISTANCE_MM       (400.0f)
#define TASK2_MIN_LAP_DISTANCE_MM          (5000.0f)
#define TASK2_MIN_LAP_TIME_MS              (8000U)
#define TASK2_APPROACH_DISTANCE_MM         (5700.0f)
#define TASK2_FINAL_APPROACH_DISTANCE_MM   (6000.0f)
#define TASK2_MAX_RUN_TIME_MS              (25000U)
#define TASK2_FINISH_WINDOW_START_MM       (5700.0f)
#define TASK2_FINISH_WINDOW_END_MM         (6700.0f)
#define TASK2_FINISH_OFFLINE_MIN_MM        (5950.0f)
#define TASK2_FINISH_HEADING_LIMIT_DEG     (30.0f)

/* Known TASK2 racetrack geometry, measured from A in the clockwise direction. */
#define TASK2_STRAIGHT_LENGTH_MM           (1500.0f)
#define TASK2_CURVE_RADIUS_MM              (500.0f)
#define TASK2_CURVE_LENGTH_MM              (1570.7963f)
#define TASK2_TRACK_LENGTH_MM              (6141.5926f)
#define TASK2_CURVE_TRANSITION_MM          (200.0f)

/*
 * MODE 1 LOCKED FINISH BASELINE
 *
 * This is the real-car-tested A-line stop path used before the three modes
 * were packaged.  Mode selection and TASK4 must not replace this finish source
 * with odometry-only or alternate marker logic.
 */

/*
 * The heavier replacement chassis under-steers in the final semicircle.
 * Keep the already stable first curve unchanged; only the second curve uses
 * a slightly lower speed and stronger geometric right-turn feedforward.
 */
#define TASK2_SECOND_CURVE_SPEED_MM_S      (245.0f)
#define TASK2_SECOND_CURVE_FF_GAIN         (1.85f)
#define TASK2_SECOND_CURVE_TRANSITION_MM   (360.0f)

/* TASK4: A-to-B is the 1.5 m upper straight in the official track. */
#define TASK4_AB_DISTANCE_MM               (1500.0f)
#define TASK4_PASS_B_MARGIN_MM             (80.0f)
#define TASK4_POST_B_SPEED_MM_S            (180.0f)
#define TASK4_MAX_RUN_TIME_MS              (8000U)
#define TASK4_START_SPEED_MM_S              (100.0f)
#define TASK4_CRUISE_SPEED_MM_S             (230.0f)
#define TASK4_ACCEL_LIMIT_MM_S2             (180.0f)
#define TASK4_DECEL_LIMIT_MM_S2             (150.0f)
#define TASK4_BALL_SLOW_START_MM             (3.0f)
#define TASK4_BALL_RECOVERY_ERROR_MM         (7.0f)
#define TASK4_BALL_RECOVERY_SPEED_MM_S       (170.0f)
#define TASK4_BALL_HOLD_SPEED_MM_S           (120.0f)
#define TASK4_START_LEAVE_TIMEOUT_MS         (1300U)

/* Start-line departure and lost-line handling. */
#define TASK2_START_GUIDE_CONFIRM_MS       (30U)
#define TASK2_START_LEAVE_TIMEOUT_MS       (1000U)
#define TASK2_LOST_CONFIRM_SAMPLES         (2U)
#define TASK2_RECOVER_CONFIRM_MS           (20U)
#define TASK2_LOST_TIMEOUT_MS              (300U)
#define TASK2_SECOND_CURVE_LOST_TIMEOUT_MS (800U)

/*
 * A-line centre calibration.
 *
 * The vehicle is placed with the front sensor-row centre on A. During the
 * start departure, the controller measures the distance from that reference
 * position until the wide A-line signature disappears. On the return lap it
 * advances by the same distance after detecting the leading edge, bringing
 * the same sensor-row centre back to the original reference.
 *
 * The learned value is accepted only inside the limits below. The fixed value
 * is a fallback for an invalid departure sample. TRIM is the only value that
 * normally needs track testing; positive stops later, negative stops earlier.
 */
#define TASK2_STOP_AUTO_REFERENCE_ENABLE   (1U)
#define TASK2_STOP_REFERENCE_MIN_MM        (4.0f)
#define TASK2_STOP_REFERENCE_MAX_MM        (40.0f)
#define TASK2_STOP_TARGET_MAX_MM           (30.0f)
#define TASK2_STOP_SENSOR_TO_REFERENCE_MM  (9.0f)
#define TASK2_STOP_REFERENCE_TRIM_MM       (0.0f)
#define TASK2_FINAL_CREEP_SPEED_MM_S        (40.0f)
#define TASK2_FINAL_TURN_MAX_MM_S           (30.0f)
#define TASK2_FINAL_CREEP_TIMEOUT_MS        (1200U)
/*
 * Estimate the distance travelled after the zero-speed command with
 * d = v^2 / (2*a), then begin braking that distance before the learned A
 * reference.  A smaller deceleration value brakes earlier; the margin is an
 * additional fixed early-stop allowance.
 */
#define TASK2_STOP_BRAKE_DECEL_MM_S2        (1200.0f)
#define TASK2_STOP_BRAKE_MARGIN_MM          (2.0f)
#define TASK2_STOP_BRAKE_COMP_MAX_MM        (30.0f)
#define TASK2_STOP_SETTLE_MIN_MS            (40U)
#define TASK2_STOP_SETTLE_MAX_MS            (220U)
#define TASK2_STOP_SPEED_THRESHOLD_MM_S     (25.0f)
#define TASK2_REVERSE_BRAKE_ENABLE          (0U)
#define TASK2_STOP_POSITION_CORRECTION      (1U)

/* STOP_LINE_TEST relaxes the full-lap guard only in that explicit mode. */
#define TASK2_STOP_TEST_MIN_TIME_MS          (1000U)
#define TASK2_STOP_TEST_MIN_DISTANCE_MM      (200.0f)

/* Speed-loop settings and future hardware compensation interfaces. */
#define TASK2_SPEED_KP_LEFT                 (4.0f)
#define TASK2_SPEED_KI_LEFT                 (3.0f)
#define TASK2_SPEED_KD_LEFT                 (1.5f)
#define TASK2_SPEED_KP_RIGHT                (4.8f)
#define TASK2_SPEED_KI_RIGHT                (3.5f)
#define TASK2_SPEED_KD_RIGHT                (1.0f)
#define TASK2_MOTOR_DEADZONE_PWM_LEFT       (0)
#define TASK2_MOTOR_DEADZONE_PWM_RIGHT      (0)
#define TASK2_PWM_LIMIT                     (7800)

/* Safety monitoring; disabled below very low target speeds. */
#define TASK2_STALL_TARGET_MIN_MM_S         (120.0f)
#define TASK2_STALL_MEASURED_MAX_MM_S       (20.0f)
#define TASK2_STALL_TIMEOUT_MS              (700U)
#define TASK2_WHEEL_MISMATCH_MAX_MM_S       (220.0f)
#define TASK2_WHEEL_MISMATCH_TIMEOUT_MS     (500U)

#endif /* TASK2_CONFIG_H */
