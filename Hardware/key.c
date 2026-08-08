#include "key.h"

/*
 * The WHEELTEC PA18 key is wired high when pressed, so it is active-high.
 * If a different switch board is fitted, change this one value to 0U.
 */
#ifndef KEY_ACTIVE_LEVEL
#define KEY_ACTIVE_LEVEL                 (1U)
#endif

#define KEY_DEBOUNCE_SAMPLES             (5U) /* 50 ms at the 100 Hz timer */

uint8_t keyValue(void)
{
    uint8_t level =
        ((DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) & KEY_key_PIN) != 0U) ?
        1U : 0U;

    /* Return 1 while the key is physically pressed. */
    return (level == KEY_ACTIVE_LEVEL) ? 1U : 0U;
}

UserKeyState_t key_scan(uint16_t freq)
{
    static uint8_t initialized = 0U;
    static uint8_t raw_previous = 0U;
    static uint8_t stable_state = 0U;
    static uint8_t debounce_count = 0U;
    uint8_t raw_pressed = keyValue();

    (void)freq;

    /* Do not manufacture a key event from the level present at power-up. */
    if (initialized == 0U) {
        initialized = 1U;
        raw_previous = raw_pressed;
        stable_state = raw_pressed;
        debounce_count = 0U;
        return USEKEY_stateless;
    }

    if (raw_pressed != raw_previous) {
        raw_previous = raw_pressed;
        debounce_count = 0U;
        return USEKEY_stateless;
    }

    if (debounce_count < KEY_DEBOUNCE_SAMPLES) {
        debounce_count++;
    }

    if ((debounce_count >= KEY_DEBOUNCE_SAMPLES) &&
        (stable_state != raw_previous)) {
        stable_state = raw_previous;

        /* Generate one event on the debounced press edge. */
        if (stable_state != 0U) {
            return USEKEY_single_click;
        }
    }

    return USEKEY_stateless;
}
