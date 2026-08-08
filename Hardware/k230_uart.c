#include "k230_uart.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "k230_uart_stats.h"
#include "ti_msp_dl_config.h"

#define K230_UART_INTERNAL_RX_BUFFER_SIZE  (1024U)
#define K230_UART_RX_BUFFER_MASK \
    (K230_UART_INTERNAL_RX_BUFFER_SIZE - 1U)

#if (K230_UART_INTERNAL_RX_BUFFER_SIZE == 0U) || \
    ((K230_UART_INTERNAL_RX_BUFFER_SIZE & \
      K230_UART_RX_BUFFER_MASK) != 0U)
#error "K230 UART RX buffer size must be a power of two"
#endif

static volatile uint8_t
    g_k230_rx_buffer[K230_UART_INTERNAL_RX_BUFFER_SIZE];
static volatile uint16_t g_k230_rx_head;
static volatile uint16_t g_k230_rx_tail;
static volatile bool g_k230_rx_overflow;

static volatile uint32_t g_k230_rx_byte_count;
static volatile uint32_t g_k230_complete_line_count;
static volatile uint32_t g_k230_valid_ball_count;
static volatile uint32_t g_k230_parse_error_count;
static volatile uint32_t g_k230_overflow_count;

static void K230_UART_StoreRxByte(uint8_t data);

static void K230_UART_DrainHardwareRx(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        K230_UART_StoreRxByte(
            (uint8_t)DL_UART_Main_receiveData(UART_1_INST));
    }
}

static bool K230_UART_ParseInteger(
    const char **cursor, int32_t *value, bool last_field)
{
    const char *text;
    int32_t result = 0;
    int32_t sign = 1;
    bool has_digit = false;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL)) {
        return false;
    }

    text = *cursor;
    if (*text == '-') {
        sign = -1;
        text++;
    }
    else if (*text == '+') {
        text++;
    }

    while ((*text >= '0') && (*text <= '9')) {
        int32_t digit = (int32_t)(*text - '0');
        has_digit = true;
        if (result > (INT32_MAX - digit) / 10) {
            return false;
        }
        result = result * 10 + digit;
        text++;
    }

    if (!has_digit) {
        return false;
    }
    if (last_field) {
        if (*text != '\0') {
            return false;
        }
    }
    else {
        if (*text != ',') {
            return false;
        }
        text++;
    }

    *value = result * sign;
    *cursor = text;
    return true;
}

/*
 * K230 formats position_mm with two decimal places (for example 48.72).
 * The controller works in integer millimetres, so parse the decimal field
 * without pulling floating-point scanf into the firmware and round it to the
 * nearest millimetre.  Plain integer fields remain accepted as well.
 */
static bool K230_UART_ParsePositionMm(
    const char **cursor, int32_t *value)
{
    const char *text;
    int32_t whole = 0;
    int32_t sign = 1;
    uint8_t first_decimal = 0U;
    uint8_t decimal_count = 0U;
    bool has_digit = false;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL)) {
        return false;
    }

    text = *cursor;
    if (*text == '-') {
        sign = -1;
        text++;
    }
    else if (*text == '+') {
        text++;
    }

    while ((*text >= '0') && (*text <= '9')) {
        int32_t digit = (int32_t)(*text - '0');

        has_digit = true;
        if (whole > (INT32_MAX - digit) / 10) {
            return false;
        }
        whole = whole * 10 + digit;
        text++;
    }
    if (!has_digit) {
        return false;
    }

    if (*text == '.') {
        text++;
        while ((*text >= '0') && (*text <= '9')) {
            if (decimal_count == 0U) {
                first_decimal = (uint8_t)(*text - '0');
            }
            decimal_count++;
            text++;
        }
        if (decimal_count == 0U) {
            return false;
        }
    }

    if (*text != ',') {
        return false;
    }
    text++;

    /* The first fractional digit is sufficient for nearest-mm rounding. */
    if (first_decimal >= 5U) {
        if (whole == INT32_MAX) {
            return false;
        }
        whole++;
    }

    *value = whole * sign;
    *cursor = text;
    return true;
}

static uint16_t K230_UART_NextIndex(uint16_t index)
{
    return (uint16_t)((index + 1U) & K230_UART_RX_BUFFER_MASK);
}

static void K230_UART_StoreRxByte(uint8_t data)
{
    uint16_t next_head = K230_UART_NextIndex(g_k230_rx_head);

    g_k230_rx_byte_count++;

    if (next_head == g_k230_rx_tail) {
        if (!g_k230_rx_overflow) {
            g_k230_overflow_count++;
        }
        g_k230_rx_overflow = true;
        return;
    }

    g_k230_rx_buffer[g_k230_rx_head] = data;
    g_k230_rx_head = next_head;
}

void K230_UART_Init(void)
{
    g_k230_rx_head = 0U;
    g_k230_rx_tail = 0U;
    K230_UART_ClearRx();
    g_k230_rx_overflow = false;

    g_k230_rx_byte_count = 0U;
    g_k230_complete_line_count = 0U;
    g_k230_valid_ball_count = 0U;
    g_k230_parse_error_count = 0U;
    g_k230_overflow_count = 0U;

    DL_UART_Main_clearInterruptStatus(
        UART_1_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    DL_UART_Main_enableInterrupt(
        UART_1_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

void K230_UART_ServiceRx(void)
{
    const uint32_t rx_interrupts =
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR;

    /*
     * Prevent the ISR and the polling fallback from reading the FIFO at the
     * same instant.  This critical section lasts only while already-received
     * UART bytes are copied into the software ring buffer.
     */
    DL_UART_Main_disableInterrupt(UART_1_INST, rx_interrupts);
    K230_UART_DrainHardwareRx();
    DL_UART_Main_clearInterruptStatus(UART_1_INST, rx_interrupts);
    DL_UART_Main_enableInterrupt(UART_1_INST, rx_interrupts);
}

void K230_UART_SendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(UART_1_INST, data);
}

void K230_UART_Send(const uint8_t *data, size_t length)
{
    size_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0U; index < length; index++) {
        K230_UART_SendByte(data[index]);
    }
}

void K230_UART_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        K230_UART_SendByte((uint8_t)*text);
        text++;
    }
}

void K230_UART_SendLine(const char *text)
{
    K230_UART_SendString(text);
    K230_UART_SendByte((uint8_t)'\r');
    K230_UART_SendByte((uint8_t)'\n');
}

void K230_UART_SendAngle(uint16_t sequence, int16_t angle_cdeg)
{
    char frame[32];
    int length = snprintf(
        frame,
        sizeof(frame),
        "ANGLE,%u,%d\r\n",
        (unsigned int)sequence,
        (int)angle_cdeg);

    if ((length > 0) && ((size_t)length < sizeof(frame))) {
        K230_UART_Send((const uint8_t *)frame, (size_t)length);
    }
}

size_t K230_UART_Available(void)
{
    uint16_t head = g_k230_rx_head;
    uint16_t tail = g_k230_rx_tail;

    return (size_t)((head - tail) & K230_UART_RX_BUFFER_MASK);
}

size_t K230_UART_Read(uint8_t *data, size_t capacity)
{
    size_t count = 0U;

    if (data == NULL) {
        return 0U;
    }

    while ((count < capacity) && (g_k230_rx_tail != g_k230_rx_head)) {
        data[count] = g_k230_rx_buffer[g_k230_rx_tail];
        g_k230_rx_tail = K230_UART_NextIndex(g_k230_rx_tail);
        count++;
    }

    return count;
}

bool K230_UART_ReadLine(char *line, size_t capacity)
{
    uint16_t scan;
    uint16_t head_snapshot;
    size_t length = 0U;

    if ((line == NULL) || (capacity == 0U)) {
        return false;
    }

    scan = g_k230_rx_tail;
    head_snapshot = g_k230_rx_head;

    while (scan != head_snapshot) {
        uint8_t data = g_k230_rx_buffer[scan];
        scan = K230_UART_NextIndex(scan);

        if (data == (uint8_t)'\n') {
            while (g_k230_rx_tail != scan) {
                data = g_k230_rx_buffer[g_k230_rx_tail];
                g_k230_rx_tail = K230_UART_NextIndex(g_k230_rx_tail);

                if (data == (uint8_t)'\n') {
                    break;
                }
                if ((data != (uint8_t)'\r') &&
                    ((length + 1U) < capacity)) {
                    line[length] = (char)data;
                    length++;
                }
            }

            line[length] = '\0';
            g_k230_complete_line_count++;
            return true;
        }
    }

    line[0] = '\0';
    return false;
}

bool K230_UART_ReadColor(K230_Color *color)
{
    char line[16];

    if (color == NULL) {
        return false;
    }

    /*
     * 没有收到完整的一行。
     */
    if (!K230_UART_ReadLine(line, sizeof(line))) {
        return false;
    }

    /*
     * K230发送的是小写字符串。
     * K230_UART_ReadLine已经自动去除了 \r 和 \n。
     */
    if (strcmp(line, "red") == 0) {
        *color = K230_COLOR_RED;
    }
    else if (strcmp(line, "green") == 0) {
        *color = K230_COLOR_GREEN;
    }
    else if (strcmp(line, "blue") == 0) {
        *color = K230_COLOR_BLUE;
    }
    else if (strcmp(line, "none") == 0) {
        *color = K230_COLOR_NONE;
    }
    else {
        *color = K230_COLOR_INVALID;
    }

    return true;
}

bool K230_UART_ReadBall(K230_BallMessage *message)
{
    char line[96];

    if (message == NULL) {
        return false;
    }

    /*
     * Consume malformed/noise lines and continue searching in this call.
     * The former implementation stopped at the first bad line, so a valid
     * BALL frame already queued behind it was delayed until a later loop.
     */
    while (K230_UART_ReadLine(line, sizeof(line))) {
        const char *cursor;
        int32_t values[8];
        uint8_t index;
        bool valid = true;

        if (strncmp(line, "BALL,", 5U) != 0) {
            valid = false;
        }

        cursor = &line[5];
        if (valid) {
            for (index = 0U; index < 8U; index++) {
                bool parsed = (index == 2U) ?
                    K230_UART_ParsePositionMm(
                        &cursor, &values[index]) :
                    K230_UART_ParseInteger(
                        &cursor,
                        &values[index],
                        (index == 7U));

                if (!parsed) {
                    valid = false;
                    break;
                }
            }
        }

        if (valid &&
            ((values[0] < 0) || (values[0] > 65535) ||
             (values[1] < 0) || (values[1] > 1) ||
             (values[2] < -1000) || (values[2] > 1000) ||
             (values[3] < INT16_MIN) ||
             (values[3] > INT16_MAX) ||
             (values[4] < INT16_MIN) ||
             (values[4] > INT16_MAX) ||
             (values[5] < 0) || (values[5] > 1000) ||
             (values[6] < -1) || (values[6] > 4095) ||
             (values[7] < -1) || (values[7] > 4095))) {
            valid = false;
        }

        if (!valid) {
            g_k230_parse_error_count++;
            continue;
        }

        message->sequence = (uint16_t)values[0];
        message->status = (K230_BallStatus)values[1];
        message->position_mm = (int16_t)values[2];
        message->error_x_px = (int16_t)values[3];
        message->error_y_px = (int16_t)values[4];
        message->confidence_permille = (uint16_t)values[5];
        message->center_x_px = (int16_t)values[6];
        message->center_y_px = (int16_t)values[7];

        g_k230_valid_ball_count++;
        return true;
    }

    return false;
}

void K230_UART_GetStats(K230UartStats *stats)
{
    if (stats == NULL) {
        return;
    }

    stats->rx_bytes = g_k230_rx_byte_count;
    stats->complete_lines = g_k230_complete_line_count;
    stats->valid_ball_frames = g_k230_valid_ball_count;
    stats->parse_errors = g_k230_parse_error_count;
    stats->overflows = g_k230_overflow_count;
}

bool K230_UART_TakeOverflow(void)
{
    bool overflow = g_k230_rx_overflow;
    g_k230_rx_overflow = false;
    return overflow;
}

void K230_UART_ClearRx(void)
{
    g_k230_rx_tail = g_k230_rx_head;
}

void UART_1_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    do {
        interrupt_index =
            DL_UART_Main_getPendingInterrupt(UART_1_INST);

        switch (interrupt_index) {
        case DL_UART_MAIN_IIDX_RX:
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            K230_UART_DrainHardwareRx();
            break;

        default:
            break;
        }
    } while (interrupt_index != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}
