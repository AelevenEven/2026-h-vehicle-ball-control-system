#ifndef STEPPER_EMM_H
#define STEPPER_EMM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ZDT X42S, Emm firmware, TTL serial protocol.
 *
 * UART_2 pin replacement for the removed serial-bus servo:
 *   PB17 / UART2 TX -> X42S R/A/H
 *   PB16 / UART2 RX <- X42S T/B/L
 *   MCU GND          -> X42S GND
 *
 * UART format is 115200 baud, 8 data bits, no parity, 1 stop bit. The driver
 * uses the factory free-protocol checksum byte 0x6B.
 */

#define STEPPER_EMM_DEFAULT_ADDRESS       (1U)
#define STEPPER_EMM_MAX_SPEED_RPM         (3000U)
#define STEPPER_EMM_RX_BUFFER_SIZE        (256U)

#define STEPPER_EMM_REPLY_OK              (0x02U)
#define STEPPER_EMM_REPLY_ZERO_LIMIT      (0x12U)
#define STEPPER_EMM_REPLY_LIMIT           (0x22U)
#define STEPPER_EMM_REPLY_PARAMETER_ERROR (0xE2U)
#define STEPPER_EMM_REPLY_FORMAT_ERROR    (0xEEU)
#define STEPPER_EMM_REPLY_REACHED         (0x9FU)

typedef enum
{
    /* Position is relative to the previous target position. */
    STEPPER_EMM_MOTION_RELATIVE_TARGET = 0x00U,
    /* Position is absolute relative to the software zero position. */
    STEPPER_EMM_MOTION_ABSOLUTE_ZERO = 0x01U,
    /* Position is relative to the current measured position. */
    STEPPER_EMM_MOTION_RELATIVE_CURRENT = 0x02U
} StepperEmmMotionMode;

typedef enum
{
    STEPPER_EMM_EVENT_REPLY = 0,
    STEPPER_EMM_EVENT_POSITION,
    STEPPER_EMM_EVENT_PERIODIC_ACK
} StepperEmmEventType;

typedef struct
{
    StepperEmmEventType type;
    uint8_t address;
    uint8_t function;
    uint8_t status;
    bool position_negative;
    uint32_t position_raw;
} StepperEmmEvent;

/*
 * Initializes the UART2 receive path and software state only. It sends no
 * command, so the mechanism does not move at power-up.
 */
void StepperEmm_Init(void);

/* F3 AB command. enable=true locks/enables the motor; false releases it. */
bool StepperEmm_Enable(uint8_t address, bool enable);

/*
 * 0A 6D command. Defines the motor's current shaft angle as position zero.
 * The mechanism must be manually placed at its horizontal reference first.
 */
bool StepperEmm_ClearPosition(uint8_t address);

/*
 * FD position command. Positive pulses select CW and negative pulses select
 * CCW. The command is executed immediately (sync flag 00).
 */
bool StepperEmm_MovePosition(
    uint8_t address,
    int32_t pulses,
    uint16_t speed_rpm,
    uint8_t acceleration,
    StepperEmmMotionMode motion_mode);

/*
 * FD position command using Emm motion mode 00: move relative to the previous
 * target position. Positive pulses select CW; negative pulses select CCW.
 * The command is executed immediately (sync flag 00).
 */
bool StepperEmm_MoveRelative(
    uint8_t address,
    int32_t pulses,
    uint16_t speed_rpm,
    uint8_t acceleration);

/*
 * FD position command using Emm motion mode 01: move to an absolute position
 * relative to the zero set by StepperEmm_ClearPosition().
 */
bool StepperEmm_MoveAbsolute(
    uint8_t address,
    int32_t position_pulses,
    uint16_t speed_rpm,
    uint8_t acceleration);

/* FE 98 command, immediate stop without releasing the shaft. */
bool StepperEmm_Stop(uint8_t address);

/* 36 6B command: request the motor shaft's current real-time position. */
bool StepperEmm_RequestPosition(uint8_t address);

/*
 * 11 18 36 command: period_ms=0 disables automatic position reports;
 * otherwise X42S returns one 0x36 position frame at the requested period.
 */
bool StepperEmm_SetPositionReportPeriod(
    uint8_t address, uint16_t period_ms);

/*
 * Non-blocking parser for both ordinary four-byte replies and the eight-byte
 * 0x36 position response. position_raw uses the motor's 0..65535/revolution
 * scale; the separate sign byte selects positive/negative direction.
 */
bool StepperEmm_ReadEvent(StepperEmmEvent *event);

/* Convert a 0x36 position result to signed motor-shaft centidegrees. */
int32_t StepperEmm_PositionCentidegrees(
    bool position_negative, uint32_t position_raw);

bool StepperEmm_TakeRxOverflow(void);
void StepperEmm_ClearRx(void);

uint32_t StepperEmm_GetCommandCount(void);
uint32_t StepperEmm_GetRxByteCount(void);
uint32_t StepperEmm_GetEventCount(void);
uint32_t StepperEmm_GetOverflowCount(void);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_EMM_H */
