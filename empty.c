#include "ti_msp_dl_config.h"

#include <stdint.h>

#include "IR_Module.h"
#include "ball_control.h"
#include "board.h"
#include "control.h"
#include "key.h"
#include "lap_test.h"
#include "oled.h"

/*
 * One firmware image contains the three evaluator-facing modes:
 *
 * M1 - TASK2 chassis lap and precise stop at A.
 * M2 - TASK3 stationary O -> +5 cm -> -5 cm ball motion.
 * M3 - TASK4 A-to-B line following while the ball is held at physical O.
 *
 * The mode is selected only at boot.  A reset is required to select another
 * mode, preventing an accidental key press from changing the active hardware
 * state machine during an evaluated run.
 */
typedef enum
{
    APP_MODE_TASK2_LAP = 0,
    APP_MODE_TASK3_BALL,
    APP_MODE_TASK4_AB_BALL,
    APP_MODE_COUNT
} AppRunMode;

#define APP_MODE_KEY_DEBOUNCE_MS       (30U)
#define APP_MODE_KEY_HOLD_MS           (800U)
#define APP_MODE_LOCKED_DISPLAY_MS     (300U)
#define APP_HOME_KEY2_ACTIVE_LEVEL     (1U)
#define APP_HOME_KEY2_DEBOUNCE_MS      (30U)

static uint8_t s_home_key_raw;
static uint8_t s_home_key_stable;
static uint32_t s_home_key_change_ms;


static uint8_t App_HomeCalibrationKeyPressed(void)
{
    uint8_t pin_high =
        ((DL_GPIO_readPins(KEY2_PORT, KEY2_PIN_8_PIN) &
          KEY2_PIN_8_PIN) != 0U) ? 1U : 0U;

    return (pin_high == APP_HOME_KEY2_ACTIVE_LEVEL) ? 1U : 0U;
}


static void App_HomeCalibrationKeyInit(void)
{
    s_home_key_raw = App_HomeCalibrationKeyPressed();
    s_home_key_stable = s_home_key_raw;
    s_home_key_change_ms = Board_GetMillis();
}


static void App_HomeCalibrationKeyUpdate(void)
{
    uint32_t now_ms = Board_GetMillis();
    uint8_t raw_pressed = App_HomeCalibrationKeyPressed();

    if (raw_pressed != s_home_key_raw) {
        s_home_key_raw = raw_pressed;
        s_home_key_change_ms = now_ms;
    }

    if ((s_home_key_stable != s_home_key_raw) &&
        ((uint32_t)(now_ms - s_home_key_change_ms) >=
         APP_HOME_KEY2_DEBOUNCE_MS)) {
        s_home_key_stable = s_home_key_raw;
        if (s_home_key_stable != 0U) {
            (void)BallControl_RegisterCurrentHome();
        }
    }
}


static void App_OledClearFrame(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < 8U; page++) {
        for (column = 0U; column < 128U; column++) {
            OLED_GRAM[column][page] = 0U;
        }
    }
}


static void App_OledDrawText(
    uint8_t x,
    uint8_t y,
    const char *text)
{
    while ((*text != '\0') && (x <= 120U)) {
        OLED_ShowChar(x, y, (uint8_t)*text, 12U, 1U);
        x = (uint8_t)(x + 8U);
        text++;
    }
}


static const char *App_ModeName(AppRunMode mode)
{
    switch (mode) {
    case APP_MODE_TASK2_LAP:
        return "M1 TASK2 LAP";

    case APP_MODE_TASK3_BALL:
        return "M2 TASK3 BALL";

    case APP_MODE_TASK4_AB_BALL:
        return "M3 TASK4 A-B";

    default:
        return "MODE ERROR";
    }
}


static void App_ShowModeMenu(
    AppRunMode mode,
    uint8_t release_to_enter)
{
    App_OledClearFrame();
    App_OledDrawText(0U, 0U, "SELECT TEST MODE");
    App_OledDrawText(0U, 16U, App_ModeName(mode));
    App_OledDrawText(0U, 32U, "CLICK: NEXT");

    if (release_to_enter != 0U) {
        App_OledDrawText(0U, 48U, "RELEASE: ENTER");
    }
    else {
        App_OledDrawText(0U, 48U, "HOLD: ENTER");
    }
    OLED_Display_On();
    OLED_Refresh_Gram();
}


static void App_ShowModeLocked(AppRunMode mode)
{
    App_OledClearFrame();
    App_OledDrawText(0U, 0U, "MODE LOCKED");
    App_OledDrawText(0U, 20U, App_ModeName(mode));
    App_OledDrawText(0U, 44U, "KEY: START/STOP");
    OLED_Display_On();
    OLED_Refresh_Gram();
}


static AppRunMode App_SelectRunMode(void)
{
    AppRunMode selected_mode =
        APP_MODE_TASK2_LAP;
    uint8_t raw_previous;
    uint8_t stable_pressed = 0U;
    uint8_t hold_hint_shown = 0U;
    uint32_t raw_change_ms;
    uint32_t press_start_ms = 0U;

    /*
     * Draw before waiting for the key to be released. OLED_Init() clears the
     * panel, so the old ordering left a black screen when PA18 was high during
     * reset or programming.
     */
    App_ShowModeMenu(selected_mode, 0U);

    /* Ignore a key that was already held while power was applied. */
    while (keyValue() != 0U) {
        __WFI();
    }
    delay_ms(APP_MODE_KEY_DEBOUNCE_MS);

    raw_previous = keyValue();
    raw_change_ms = Board_GetMillis();
    App_ShowModeMenu(selected_mode, 0U);

    while (1) {
        uint32_t now_ms = Board_GetMillis();
        uint8_t raw_pressed = keyValue();

        if (raw_pressed != raw_previous) {
            raw_previous = raw_pressed;
            raw_change_ms = now_ms;
        }

        if (((uint32_t)(now_ms - raw_change_ms) >=
             APP_MODE_KEY_DEBOUNCE_MS) &&
            (stable_pressed != raw_previous)) {

            stable_pressed = raw_previous;

            if (stable_pressed != 0U) {
                press_start_ms = now_ms;
                hold_hint_shown = 0U;
            }
            else {
                uint32_t held_ms =
                    (uint32_t)(now_ms - press_start_ms);

                if (held_ms >= APP_MODE_KEY_HOLD_MS) {
                    App_ShowModeLocked(selected_mode);
                    delay_ms(APP_MODE_LOCKED_DISPLAY_MS);

                    /* Start key scanning later from a known released level. */
                    while (keyValue() != 0U) {
                        __WFI();
                    }
                    delay_ms(APP_MODE_KEY_DEBOUNCE_MS);
                    return selected_mode;
                }

                selected_mode =
                    (AppRunMode)(
                        ((uint8_t)selected_mode + 1U) %
                        (uint8_t)APP_MODE_COUNT);
                App_ShowModeMenu(selected_mode, 0U);
            }
        }

        if ((stable_pressed != 0U) &&
            (hold_hint_shown == 0U) &&
            ((uint32_t)(now_ms - press_start_ms) >=
             APP_MODE_KEY_HOLD_MS)) {
            hold_hint_shown = 1U;
            App_ShowModeMenu(selected_mode, 1U);
        }

        __WFI();
    }
}


static void App_InitSelectedMode(AppRunMode mode)
{
    if ((mode == APP_MODE_TASK2_LAP) ||
        (mode == APP_MODE_TASK4_AB_BALL)) {
        IR_Module_Init();
    }

    switch (mode) {
    case APP_MODE_TASK2_LAP:
        LapTest_SetTask4Mode(0U);
        LapTest_Init();
        break;

    case APP_MODE_TASK3_BALL:
        BallControl_Init();
        BallControl_SetBootHomeCalibration(false);
        BallControl_SetTask3Enabled(true);
        break;

    case APP_MODE_TASK4_AB_BALL:
        BallControl_Init();
        BallControl_SetBootHomeCalibration(false);
        BallControl_SetAutoZeroHoldEnabled(true);
        BallControl_SetDisplayEnabled(false);
        LapTest_SetTask4Mode(1U);
        LapTest_Init();
        break;

    default:
        Flag_Stop = 1;
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        break;
    }

    App_HomeCalibrationKeyInit();

#if LAP_DEBUG_PRINT_ENABLE == 1U
    if ((mode == APP_MODE_TASK2_LAP) ||
        (mode == APP_MODE_TASK4_AB_BALL)) {
        Board_EnableProgrammingUartTx();
    }
#endif
}


static void App_UpdateSelectedMode(AppRunMode mode)
{
    switch (mode) {
    case APP_MODE_TASK2_LAP:
        LapTest_Update();
        break;

    case APP_MODE_TASK3_BALL:
        App_HomeCalibrationKeyUpdate();
        BallControl_Update();
        break;

    case APP_MODE_TASK4_AB_BALL:
        App_HomeCalibrationKeyUpdate();
        BallControl_Update();
        LapTest_SetExternalStartReady(
            BallControl_IsAutoZeroHoldReady() ? 1U : 0U);
        LapTest_SetTask4BallErrorMm(
            BallControl_GetPositionErrorMm());
        LapTest_SetTask4BallVisionHold(
            BallControl_IsAutoZeroHoldVisionHeld() ? 1U : 0U);
        LapTest_SetTask4ExternalFault(
            (uint8_t)BallControl_GetAutoZeroHoldFault());
        LapTest_Update();
        break;

    default:
        Flag_Stop = 1;
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        break;
    }
}


int main(void)
{
    AppRunMode run_mode;

    SYSCFG_DL_init();
    SysTick_Init();
    OLED_Init();
    Control_Init();

    run_mode = App_SelectRunMode();
    App_InitSelectedMode(run_mode);

    DL_Timer_startCounter(PWM_0_INST);

    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
        App_UpdateSelectedMode(run_mode);
    }
}
