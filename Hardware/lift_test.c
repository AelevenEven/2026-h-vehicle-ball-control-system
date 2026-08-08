#include "lift_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "board.h"
#include "control.h"
#include "motor.h"
#include "oled.h"
#include "stepper_emm.h"

#define LIFT_TEST_SPEED_RPM             (30U)
#define LIFT_TEST_ACCELERATION          (100U)
#define LIFT_TEST_COMMAND_GAP_MS        (50U)
#define LIFT_TEST_ENABLE_SETTLE_MS      (100U)
#define LIFT_TEST_MOTION_GUARD_MS       (300U)
#define LIFT_TEST_REPLY_WAIT_MS         (300U)
#define LIFT_TEST_DISPLAY_PERIOD_MS     (200U)

/* 3200 motor pulses/revolution: 40 pulses are 4.5 motor-shaft degrees. */
#define LIFT_TEST_DIRECTION_SIGN        (1)
#define LIFT_TEST_MAX_ABS_PULSES        (40)

typedef enum
{
    LIFT_TEST_WAIT_HORIZONTAL = 0,
    LIFT_TEST_WAIT_STOP_GAP,
    LIFT_TEST_WAIT_ZERO_GAP,
    LIFT_TEST_WAIT_ENABLE_SETTLE,
    LIFT_TEST_READY,
    LIFT_TEST_MOTION_GUARD,
    LIFT_TEST_STOPPED_BY_KEY,
    LIFT_TEST_FAULT
} LiftTestState;

static const int32_t s_position_sequence[] =
{
    40,
    0,
    -40,
    0
};

static LiftTestState s_state;
static uint8_t s_sequence_index;
static int32_t s_target_pulses;
static uint32_t s_state_since_ms;
static uint32_t s_last_command_ms;
static uint32_t s_last_display_ms;
static uint8_t s_last_reply;
static bool s_reply_seen_for_command;
static bool s_rx_overflow_seen;
static bool s_force_display;

extern uint8_t OLED_GRAM[128][8];

static void LiftTest_ClearFrame(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < 8U; page++) {
        for (column = 0U; column < 128U; column++) {
            OLED_GRAM[column][page] = 0U;
        }
    }
}

static void LiftTest_DrawText(uint8_t x, uint8_t y, const char *text)
{
    while ((*text != '\0') && (x <= 120U)) {
        OLED_ShowChar(x, y, (uint8_t)*text, 12U, 1U);
        x = (uint8_t)(x + 8U);
        text++;
    }
}

static bool LiftTest_IsFaultReply(uint8_t reply)
{
    return (reply == STEPPER_EMM_REPLY_ZERO_LIMIT) ||
           (reply == STEPPER_EMM_REPLY_LIMIT) ||
           (reply == STEPPER_EMM_REPLY_PARAMETER_ERROR) ||
           (reply == STEPPER_EMM_REPLY_FORMAT_ERROR);
}

static void LiftTest_KeepChassisStopped(void)
{
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    MotorA.Motor_Pwm = 0.0f;
    MotorB.Motor_Pwm = 0.0f;
    Set_PWM(0, 0);
}

static bool LiftTest_ConsumeKeyPress(void)
{
    if (Flag_Stop == 0) {
        /*
         * Key() toggles Flag_Stop in the 10 ms control interrupt.  Consume
         * that RUN request locally, then restore STOP so the chassis can
         * never be started by this test mode.
         */
        Flag_Stop = 1;
        return true;
    }

    return false;
}

static void LiftTest_RecordCommand(bool sent, uint32_t now_ms)
{
    s_last_command_ms = now_ms;
    s_reply_seen_for_command = false;
    s_rx_overflow_seen = false;
    s_force_display = true;

    if (!sent) {
        s_last_reply = STEPPER_EMM_REPLY_PARAMETER_ERROR;
        s_state = LIFT_TEST_FAULT;
        (void)StepperEmm_Stop(STEPPER_EMM_DEFAULT_ADDRESS);
    }
}

static void LiftTest_EnterFault(uint8_t reply, uint32_t now_ms)
{
    s_last_reply = reply;
    s_reply_seen_for_command = true;
    s_state = LIFT_TEST_FAULT;
    s_state_since_ms = now_ms;
    s_force_display = true;
    (void)StepperEmm_Stop(STEPPER_EMM_DEFAULT_ADDRESS);
}

static void LiftTest_ProcessReplies(uint32_t now_ms)
{
    StepperEmmEvent event;

    if (StepperEmm_TakeRxOverflow()) {
        StepperEmm_ClearRx();
        s_rx_overflow_seen = true;
        s_force_display = true;
    }

    while (StepperEmm_ReadEvent(&event)) {
        if ((event.type == STEPPER_EMM_EVENT_REPLY) ||
            (event.type == STEPPER_EMM_EVENT_PERIODIC_ACK)) {
            /* Preserve the first real fault code after the stop reply. */
            if (s_state == LIFT_TEST_FAULT) {
                continue;
            }
            s_last_reply = event.status;
            s_reply_seen_for_command = true;
            s_force_display = true;

            if (LiftTest_IsFaultReply(event.status)) {
                LiftTest_EnterFault(event.status, now_ms);
                return;
            }
        }
    }
}

static const char *LiftTest_StateText(void)
{
    switch (s_state) {
    case LIFT_TEST_WAIT_HORIZONTAL:
        return "SET HORIZONTAL";
    case LIFT_TEST_WAIT_STOP_GAP:
        return "INIT STOP";
    case LIFT_TEST_WAIT_ZERO_GAP:
        return "SET ZERO";
    case LIFT_TEST_WAIT_ENABLE_SETTLE:
        return "ENABLE MOTOR";
    case LIFT_TEST_READY:
        if (s_target_pulses > 0) {
            return "HOLD +4.5 DEG";
        }
        if (s_target_pulses < 0) {
            return "HOLD -4.5 DEG";
        }
        return "ZERO READY";
    case LIFT_TEST_MOTION_GUARD:
        if (s_target_pulses > 0) {
            return "MOVE +4.5 DEG";
        }
        if (s_target_pulses < 0) {
            return "MOVE -4.5 DEG";
        }
        return "MOVE ZERO";
    case LIFT_TEST_STOPPED_BY_KEY:
        return "STOPPED BY KEY";
    case LIFT_TEST_FAULT:
        return "X42S FAULT";
    default:
        return "STATE ERROR";
    }
}

static void LiftTest_Show(uint32_t now_ms)
{
    char line[18];
    const char *angle_text;
    int32_t display_pulses = s_target_pulses * LIFT_TEST_DIRECTION_SIGN;

    if ((!s_force_display) &&
        ((uint32_t)(now_ms - s_last_display_ms) <
         LIFT_TEST_DISPLAY_PERIOD_MS)) {
        return;
    }

    s_force_display = false;
    s_last_display_ms = now_ms;

    LiftTest_ClearFrame();
    LiftTest_DrawText(0U, 0U, "X42S LIFT TEST");
    LiftTest_DrawText(0U, 16U, LiftTest_StateText());

    angle_text = (display_pulses > 0) ? "+4.5" :
                 ((display_pulses < 0) ? "-4.5" : "0.0");
    (void)snprintf(
        line,
        sizeof(line),
        "P:%+04ld A:%s",
        (long)display_pulses,
        angle_text);
    LiftTest_DrawText(0U, 32U, line);

    if (s_state == LIFT_TEST_FAULT) {
        (void)snprintf(
            line, sizeof(line), "FAULT %02X C:%03lu",
            (unsigned int)s_last_reply,
            (unsigned long)StepperEmm_GetCommandCount());
    }
    else if (s_rx_overflow_seen) {
        (void)snprintf(
            line, sizeof(line), "RX OVER C:%03lu",
            (unsigned long)StepperEmm_GetCommandCount());
    }
    else if (s_reply_seen_for_command) {
        (void)snprintf(
            line, sizeof(line), "REPLY %02X C:%03lu",
            (unsigned int)s_last_reply,
            (unsigned long)StepperEmm_GetCommandCount());
    }
    else if ((s_last_command_ms != 0U) &&
             ((uint32_t)(now_ms - s_last_command_ms) >=
              LIFT_TEST_REPLY_WAIT_MS)) {
        (void)snprintf(
            line, sizeof(line), "NO REPLY C:%03lu",
            (unsigned long)StepperEmm_GetCommandCount());
    }
    else {
        (void)snprintf(
            line, sizeof(line), "CMD %03lu WAIT",
            (unsigned long)StepperEmm_GetCommandCount());
    }
    LiftTest_DrawText(0U, 48U, line);
    OLED_Refresh_Gram();
}

static bool LiftTest_SendAbsolute(int32_t logical_pulses, uint32_t now_ms)
{
    int32_t motor_pulses = logical_pulses * LIFT_TEST_DIRECTION_SIGN;
    bool sent;

    if ((motor_pulses > LIFT_TEST_MAX_ABS_PULSES) ||
        (motor_pulses < -LIFT_TEST_MAX_ABS_PULSES)) {
        LiftTest_EnterFault(STEPPER_EMM_REPLY_PARAMETER_ERROR, now_ms);
        return false;
    }

    sent = StepperEmm_MoveAbsolute(
        STEPPER_EMM_DEFAULT_ADDRESS,
        motor_pulses,
        LIFT_TEST_SPEED_RPM,
        LIFT_TEST_ACCELERATION);
    LiftTest_RecordCommand(sent, now_ms);
    return sent;
}

void LiftTest_Init(void)
{
    uint32_t now_ms = Board_GetMillis();

    StepperEmm_Init();
    Flag_Stop = 1;
    LiftTest_KeepChassisStopped();

    s_state = LIFT_TEST_WAIT_HORIZONTAL;
    s_sequence_index = 0U;
    s_target_pulses = 0;
    s_state_since_ms = now_ms;
    s_last_command_ms = 0U;
    s_last_display_ms = 0U;
    s_last_reply = 0U;
    s_reply_seen_for_command = false;
    s_rx_overflow_seen = false;
    s_force_display = true;

    LiftTest_Show(now_ms);
}

void LiftTest_Update(void)
{
    uint32_t now_ms = Board_GetMillis();
    bool key_pressed;

    LiftTest_KeepChassisStopped();
    LiftTest_ProcessReplies(now_ms);
    key_pressed = LiftTest_ConsumeKeyPress();

    if (s_state == LIFT_TEST_FAULT) {
        LiftTest_Show(now_ms);
        return;
    }

    if ((s_state == LIFT_TEST_WAIT_STOP_GAP) ||
        (s_state == LIFT_TEST_WAIT_ZERO_GAP) ||
        (s_state == LIFT_TEST_WAIT_ENABLE_SETTLE)) {
        if (key_pressed) {
            (void)StepperEmm_Stop(STEPPER_EMM_DEFAULT_ADDRESS);
            s_state = LIFT_TEST_STOPPED_BY_KEY;
            s_state_since_ms = now_ms;
            s_force_display = true;
        }
        else if ((uint32_t)(now_ms - s_state_since_ms) >=
                 ((s_state == LIFT_TEST_WAIT_ENABLE_SETTLE) ?
                  LIFT_TEST_ENABLE_SETTLE_MS :
                  LIFT_TEST_COMMAND_GAP_MS)) {
            if (s_state == LIFT_TEST_WAIT_STOP_GAP) {
                LiftTest_RecordCommand(
                    StepperEmm_ClearPosition(
                        STEPPER_EMM_DEFAULT_ADDRESS),
                    now_ms);
                if (s_state != LIFT_TEST_FAULT) {
                    s_state = LIFT_TEST_WAIT_ZERO_GAP;
                    s_state_since_ms = now_ms;
                }
            }
            else if (s_state == LIFT_TEST_WAIT_ZERO_GAP) {
                LiftTest_RecordCommand(
                    StepperEmm_Enable(
                        STEPPER_EMM_DEFAULT_ADDRESS, true),
                    now_ms);
                if (s_state != LIFT_TEST_FAULT) {
                    s_state = LIFT_TEST_WAIT_ENABLE_SETTLE;
                    s_state_since_ms = now_ms;
                }
            }
            else {
                s_state = LIFT_TEST_READY;
                s_state_since_ms = now_ms;
                s_force_display = true;
            }
        }
    }
    else if (s_state == LIFT_TEST_WAIT_HORIZONTAL) {
        if (key_pressed) {
            LiftTest_RecordCommand(
                StepperEmm_Stop(STEPPER_EMM_DEFAULT_ADDRESS),
                now_ms);
            if (s_state != LIFT_TEST_FAULT) {
                s_state = LIFT_TEST_WAIT_STOP_GAP;
                s_state_since_ms = now_ms;
            }
        }
    }
    else if (s_state == LIFT_TEST_MOTION_GUARD) {
        if (key_pressed) {
            LiftTest_RecordCommand(
                StepperEmm_Stop(STEPPER_EMM_DEFAULT_ADDRESS),
                now_ms);
            if (s_state != LIFT_TEST_FAULT) {
                s_state = LIFT_TEST_STOPPED_BY_KEY;
                s_state_since_ms = now_ms;
            }
        }
        else if ((uint32_t)(now_ms - s_state_since_ms) >=
                 LIFT_TEST_MOTION_GUARD_MS) {
            s_state = LIFT_TEST_READY;
            s_state_since_ms = now_ms;
            s_force_display = true;
        }
    }
    else if ((s_state == LIFT_TEST_READY) ||
             (s_state == LIFT_TEST_STOPPED_BY_KEY)) {
        if (key_pressed) {
            int32_t next_target =
                s_position_sequence[s_sequence_index];

            if (LiftTest_SendAbsolute(next_target, now_ms)) {
                s_target_pulses = next_target;
                s_sequence_index = (uint8_t)(
                    (s_sequence_index + 1U) %
                    (sizeof(s_position_sequence) /
                     sizeof(s_position_sequence[0])));
                s_state = LIFT_TEST_MOTION_GUARD;
                s_state_since_ms = now_ms;
            }
        }
    }

    LiftTest_Show(now_ms);
}
