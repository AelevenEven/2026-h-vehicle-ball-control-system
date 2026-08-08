#include "stepper_emm.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#if (UART_2_BAUD_RATE != 115200)
#error "X42S Emm requires UART_2 at 115200 baud"
#endif

#define STEPPER_EMM_CHECK_BYTE            (0x6BU)
#define STEPPER_EMM_ENABLE_FUNCTION       (0xF3U)
#define STEPPER_EMM_ENABLE_AUXILIARY      (0xABU)
#define STEPPER_EMM_CLEAR_FUNCTION        (0x0AU)
#define STEPPER_EMM_CLEAR_AUXILIARY       (0x6DU)
#define STEPPER_EMM_POSITION_FUNCTION     (0xFDU)
#define STEPPER_EMM_QUERY_POSITION        (0x36U)
#define STEPPER_EMM_PERIODIC_FUNCTION     (0x11U)
#define STEPPER_EMM_PERIODIC_AUXILIARY    (0x18U)
#define STEPPER_EMM_STOP_FUNCTION         (0xFEU)
#define STEPPER_EMM_STOP_AUXILIARY        (0x98U)
#define STEPPER_EMM_DIRECTION_CW          (0x00U)
#define STEPPER_EMM_DIRECTION_CCW         (0x01U)
#define STEPPER_EMM_EXECUTE_IMMEDIATELY   (0x00U)

#define STEPPER_EMM_ENABLE_FRAME_SIZE     (6U)
#define STEPPER_EMM_CLEAR_FRAME_SIZE      (4U)
#define STEPPER_EMM_POSITION_FRAME_SIZE   (13U)
#define STEPPER_EMM_STOP_FRAME_SIZE       (5U)
#define STEPPER_EMM_REPLY_FRAME_SIZE      (4U)
#define STEPPER_EMM_QUERY_FRAME_SIZE      (3U)
#define STEPPER_EMM_PERIODIC_FRAME_SIZE   (7U)
#define STEPPER_EMM_PERIODIC_REPLY_SIZE   (3U)
#define STEPPER_EMM_POSITION_REPLY_SIZE   (8U)
#define STEPPER_EMM_RX_BUFFER_MASK        (STEPPER_EMM_RX_BUFFER_SIZE - 1U)

#if (STEPPER_EMM_RX_BUFFER_SIZE == 0U) || \
    ((STEPPER_EMM_RX_BUFFER_SIZE & STEPPER_EMM_RX_BUFFER_MASK) != 0U)
#error "STEPPER_EMM_RX_BUFFER_SIZE must be a power of two"
#endif

static volatile uint8_t s_rx_buffer[STEPPER_EMM_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile bool s_rx_overflow;
static uint32_t s_command_count;
static volatile uint32_t s_rx_byte_count;
static uint32_t s_event_count;
static volatile uint32_t s_overflow_count;

static uint16_t StepperEmm_NextRxIndex(uint16_t index)
{
    return (uint16_t)((index + 1U) & STEPPER_EMM_RX_BUFFER_MASK);
}

static void StepperEmm_StoreRxByte(uint8_t data)
{
    uint16_t next_head = StepperEmm_NextRxIndex(s_rx_head);

    s_rx_byte_count++;
    if (next_head == s_rx_tail) {
        s_rx_overflow = true;
        s_overflow_count++;
        return;
    }

    s_rx_buffer[s_rx_head] = data;
    s_rx_head = next_head;
}

static uint16_t StepperEmm_RxAvailable(void)
{
    uint16_t head = s_rx_head;
    uint16_t tail = s_rx_tail;

    return (uint16_t)((head - tail) & STEPPER_EMM_RX_BUFFER_MASK);
}

static uint8_t StepperEmm_PeekRx(uint16_t offset)
{
    uint16_t index =
        (uint16_t)((s_rx_tail + offset) & STEPPER_EMM_RX_BUFFER_MASK);

    return s_rx_buffer[index];
}

static bool StepperEmm_IsReplyStatus(uint8_t status)
{
    return (status == STEPPER_EMM_REPLY_OK) ||
           (status == STEPPER_EMM_REPLY_ZERO_LIMIT) ||
           (status == STEPPER_EMM_REPLY_LIMIT) ||
           (status == STEPPER_EMM_REPLY_PARAMETER_ERROR) ||
           (status == STEPPER_EMM_REPLY_FORMAT_ERROR) ||
           (status == STEPPER_EMM_REPLY_REACHED);
}

static void StepperEmm_Send(const uint8_t *data, uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; index++) {
        DL_UART_Main_transmitDataBlocking(UART_2_INST, data[index]);
    }
    s_command_count++;
}

static bool StepperEmm_IsValidAddress(uint8_t address)
{
    /* Address 0 is broadcast and is deliberately not used by this driver. */
    return address != 0U;
}

void StepperEmm_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow = false;
    s_command_count = 0U;
    s_rx_byte_count = 0U;
    s_event_count = 0U;
    s_overflow_count = 0U;

    DL_UART_Main_clearInterruptStatus(
        UART_2_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_2_INST, DL_UART_MAIN_INTERRUPT_RX);

    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

bool StepperEmm_Enable(uint8_t address, bool enable)
{
    uint8_t frame[STEPPER_EMM_ENABLE_FRAME_SIZE];

    if (!StepperEmm_IsValidAddress(address)) {
        return false;
    }

    frame[0] = address;
    frame[1] = STEPPER_EMM_ENABLE_FUNCTION;
    frame[2] = STEPPER_EMM_ENABLE_AUXILIARY;
    frame[3] = enable ? 0x01U : 0x00U;
    frame[4] = STEPPER_EMM_EXECUTE_IMMEDIATELY;
    frame[5] = STEPPER_EMM_CHECK_BYTE;
    StepperEmm_Send(frame, (uint8_t)sizeof(frame));
    return true;
}

bool StepperEmm_ClearPosition(uint8_t address)
{
    uint8_t frame[STEPPER_EMM_CLEAR_FRAME_SIZE];

    if (!StepperEmm_IsValidAddress(address)) {
        return false;
    }

    frame[0] = address;
    frame[1] = STEPPER_EMM_CLEAR_FUNCTION;
    frame[2] = STEPPER_EMM_CLEAR_AUXILIARY;
    frame[3] = STEPPER_EMM_CHECK_BYTE;
    StepperEmm_Send(frame, (uint8_t)sizeof(frame));
    return true;
}

bool StepperEmm_MovePosition(
    uint8_t address,
    int32_t pulses,
    uint16_t speed_rpm,
    uint8_t acceleration,
    StepperEmmMotionMode motion_mode)
{
    uint8_t frame[STEPPER_EMM_POSITION_FRAME_SIZE];
    uint32_t magnitude;

    if (!StepperEmm_IsValidAddress(address) ||
        (speed_rpm == 0U) ||
        (speed_rpm > STEPPER_EMM_MAX_SPEED_RPM) ||
        ((motion_mode != STEPPER_EMM_MOTION_RELATIVE_TARGET) &&
         (motion_mode != STEPPER_EMM_MOTION_ABSOLUTE_ZERO) &&
         (motion_mode != STEPPER_EMM_MOTION_RELATIVE_CURRENT)) ||
        ((pulses == 0) &&
         (motion_mode != STEPPER_EMM_MOTION_ABSOLUTE_ZERO))) {
        return false;
    }

    if (pulses < 0) {
        frame[2] = STEPPER_EMM_DIRECTION_CCW;
        magnitude = (uint32_t)(-(pulses + 1)) + 1U;
    }
    else {
        frame[2] = STEPPER_EMM_DIRECTION_CW;
        magnitude = (uint32_t)pulses;
    }

    frame[0] = address;
    frame[1] = STEPPER_EMM_POSITION_FUNCTION;
    frame[3] = (uint8_t)(speed_rpm >> 8U);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(magnitude >> 24U);
    frame[7] = (uint8_t)(magnitude >> 16U);
    frame[8] = (uint8_t)(magnitude >> 8U);
    frame[9] = (uint8_t)magnitude;
    frame[10] = (uint8_t)motion_mode;
    frame[11] = STEPPER_EMM_EXECUTE_IMMEDIATELY;
    frame[12] = STEPPER_EMM_CHECK_BYTE;
    StepperEmm_Send(frame, (uint8_t)sizeof(frame));
    return true;
}

bool StepperEmm_MoveRelative(
    uint8_t address,
    int32_t pulses,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    return StepperEmm_MovePosition(
        address,
        pulses,
        speed_rpm,
        acceleration,
        STEPPER_EMM_MOTION_RELATIVE_TARGET);
}

bool StepperEmm_MoveAbsolute(
    uint8_t address,
    int32_t position_pulses,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    return StepperEmm_MovePosition(
        address,
        position_pulses,
        speed_rpm,
        acceleration,
        STEPPER_EMM_MOTION_ABSOLUTE_ZERO);
}

bool StepperEmm_Stop(uint8_t address)
{
    uint8_t frame[STEPPER_EMM_STOP_FRAME_SIZE];

    if (!StepperEmm_IsValidAddress(address)) {
        return false;
    }

    frame[0] = address;
    frame[1] = STEPPER_EMM_STOP_FUNCTION;
    frame[2] = STEPPER_EMM_STOP_AUXILIARY;
    frame[3] = STEPPER_EMM_EXECUTE_IMMEDIATELY;
    frame[4] = STEPPER_EMM_CHECK_BYTE;
    StepperEmm_Send(frame, (uint8_t)sizeof(frame));
    return true;
}

bool StepperEmm_RequestPosition(uint8_t address)
{
    uint8_t frame[STEPPER_EMM_QUERY_FRAME_SIZE];

    if (!StepperEmm_IsValidAddress(address)) {
        return false;
    }

    frame[0] = address;
    frame[1] = STEPPER_EMM_QUERY_POSITION;
    frame[2] = STEPPER_EMM_CHECK_BYTE;
    StepperEmm_Send(frame, (uint8_t)sizeof(frame));
    return true;
}

bool StepperEmm_SetPositionReportPeriod(
    uint8_t address, uint16_t period_ms)
{
    uint8_t frame[STEPPER_EMM_PERIODIC_FRAME_SIZE];

    if (!StepperEmm_IsValidAddress(address)) {
        return false;
    }

    frame[0] = address;
    frame[1] = STEPPER_EMM_PERIODIC_FUNCTION;
    frame[2] = STEPPER_EMM_PERIODIC_AUXILIARY;
    frame[3] = STEPPER_EMM_QUERY_POSITION;
    frame[4] = (uint8_t)(period_ms >> 8U);
    frame[5] = (uint8_t)period_ms;
    frame[6] = STEPPER_EMM_CHECK_BYTE;
    StepperEmm_Send(frame, (uint8_t)sizeof(frame));
    return true;
}

bool StepperEmm_ReadEvent(StepperEmmEvent *event)
{
    if (event == NULL) {
        return false;
    }

    while (StepperEmm_RxAvailable() >=
           STEPPER_EMM_PERIODIC_REPLY_SIZE) {
        uint8_t address = StepperEmm_PeekRx(0U);
        uint8_t function = StepperEmm_PeekRx(1U);
        uint8_t index;

        if ((address != 0U) &&
            (function == STEPPER_EMM_PERIODIC_FUNCTION) &&
            (StepperEmm_PeekRx(2U) == STEPPER_EMM_CHECK_BYTE)) {
            for (index = 0U;
                 index < STEPPER_EMM_PERIODIC_REPLY_SIZE;
                 index++) {
                s_rx_tail = StepperEmm_NextRxIndex(s_rx_tail);
            }
            event->type = STEPPER_EMM_EVENT_PERIODIC_ACK;
            event->address = address;
            event->function = function;
            event->status = STEPPER_EMM_REPLY_OK;
            event->position_negative = false;
            event->position_raw = 0U;
            s_event_count++;
            return true;
        }
        else if ((address != 0U) &&
            (function == STEPPER_EMM_QUERY_POSITION)) {
            uint8_t sign;
            uint32_t raw_position;

            if (StepperEmm_RxAvailable() <
                STEPPER_EMM_POSITION_REPLY_SIZE) {
                return false;
            }
            sign = StepperEmm_PeekRx(2U);
            if (((sign == 0U) || (sign == 1U)) &&
                (StepperEmm_PeekRx(7U) ==
                 STEPPER_EMM_CHECK_BYTE)) {
                raw_position =
                    ((uint32_t)StepperEmm_PeekRx(3U) << 24U) |
                    ((uint32_t)StepperEmm_PeekRx(4U) << 16U) |
                    ((uint32_t)StepperEmm_PeekRx(5U) << 8U) |
                    (uint32_t)StepperEmm_PeekRx(6U);
                for (index = 0U;
                     index < STEPPER_EMM_POSITION_REPLY_SIZE;
                     index++) {
                    s_rx_tail = StepperEmm_NextRxIndex(s_rx_tail);
                }
                event->type = STEPPER_EMM_EVENT_POSITION;
                event->address = address;
                event->function = function;
                event->status = 0U;
                event->position_negative = (sign != 0U);
                event->position_raw = raw_position;
                s_event_count++;
                return true;
            }
        }
        else {
            if (StepperEmm_RxAvailable() <
                STEPPER_EMM_REPLY_FRAME_SIZE) {
                return false;
            }
            uint8_t status = StepperEmm_PeekRx(2U);
            uint8_t check = StepperEmm_PeekRx(3U);

            if ((address != 0U) &&
                StepperEmm_IsReplyStatus(status) &&
                (check == STEPPER_EMM_CHECK_BYTE)) {
                for (index = 0U;
                     index < STEPPER_EMM_REPLY_FRAME_SIZE;
                     index++) {
                    s_rx_tail = StepperEmm_NextRxIndex(s_rx_tail);
                }
                event->type = STEPPER_EMM_EVENT_REPLY;
                event->address = address;
                event->function = function;
                event->status = status;
                event->position_negative = false;
                event->position_raw = 0U;
                s_event_count++;
                return true;
            }
        }

        /* Discard one byte and resynchronize with the next possible frame. */
        s_rx_tail = StepperEmm_NextRxIndex(s_rx_tail);
    }

    return false;
}

int32_t StepperEmm_PositionCentidegrees(
    bool position_negative, uint32_t position_raw)
{
    uint32_t magnitude = position_raw;
    int32_t centidegrees;

    /* 65536 counts = 360 degrees. Round to the nearest 0.01 degree. */
    magnitude = (uint32_t)
        ((((uint64_t)magnitude * 36000ULL) + 32768ULL) / 65536ULL);
    if (magnitude > 2147483647UL) {
        magnitude = 2147483647UL;
    }
    centidegrees = (int32_t)magnitude;
    return position_negative ? -centidegrees : centidegrees;
}

bool StepperEmm_TakeRxOverflow(void)
{
    bool overflow = s_rx_overflow;

    s_rx_overflow = false;
    return overflow;
}

void StepperEmm_ClearRx(void)
{
    s_rx_tail = s_rx_head;
}

uint32_t StepperEmm_GetCommandCount(void)
{
    return s_command_count;
}

uint32_t StepperEmm_GetRxByteCount(void)
{
    return s_rx_byte_count;
}

uint32_t StepperEmm_GetEventCount(void)
{
    return s_event_count;
}

uint32_t StepperEmm_GetOverflowCount(void)
{
    return s_overflow_count;
}

void UART_2_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) {
            StepperEmm_StoreRxByte(
                (uint8_t)DL_UART_Main_receiveData(UART_2_INST));
        }
        break;

    default:
        break;
    }
}
