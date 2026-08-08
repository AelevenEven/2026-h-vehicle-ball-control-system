#ifndef K230_UART_H
#define K230_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * K230 vision-module UART transport.
 *
 * Hardware mapping is provided by SysConfig:
 *   UART_1, 115200 baud, 8 data bits, no parity, 1 stop bit
 *   PB6 = MCU TX (connect to K230 RX)
 *   PB7 = MCU RX (connect to K230 TX)
 *
 * K230_UART_Read() and K230_UART_ReadLine() consume the same RX buffer. Use
 * one receive style for a protocol instead of mixing both APIs.
 */

#define K230_UART_BAUD_RATE       (115200U)
#define K230_UART_RX_BUFFER_SIZE  (256U)

typedef enum
{
    K230_COLOR_NONE = 0,
    K230_COLOR_RED,
    K230_COLOR_GREEN,
    K230_COLOR_BLUE,
    K230_COLOR_INVALID
} K230_Color;

typedef enum
{
    K230_BALL_LOST = 0,
    K230_BALL_TRACKING = 1
} K230_BallStatus;

/*
 * Decoded K230 frame:
 * BALL,seq,status,position_mm,error_x,error_y,confidence_permille,cx,cy
 */
typedef struct
{
    uint16_t sequence;
    K230_BallStatus status;
    int16_t position_mm;
    int16_t error_x_px;
    int16_t error_y_px;
    uint16_t confidence_permille;
    int16_t center_x_px;
    int16_t center_y_px;
} K230_BallMessage;

void K230_UART_Init(void);

/*
 * Polling fallback for the UART1 hardware FIFO.  The RX interrupt remains
 * enabled, but BallControl calls this once per update so a missed interrupt
 * cannot leave valid BALL bytes stranded in the peripheral.
 */
void K230_UART_ServiceRx(void);

void K230_UART_SendByte(uint8_t data);
void K230_UART_Send(const uint8_t *data, size_t length);
void K230_UART_SendString(const char *text);
void K230_UART_SendLine(const char *text);

/* Send ANGLE,seq,angle_cdeg for K230 dynamic-perspective calibration. */
void K230_UART_SendAngle(uint16_t sequence, int16_t angle_cdeg);

size_t K230_UART_Available(void);
size_t K230_UART_Read(uint8_t *data, size_t capacity);

/*
 * Reads one '\n'-terminated ASCII message without blocking. A preceding '\r'
 * is removed. The output is always NUL-terminated when capacity is nonzero.
 * Returns true only when a complete line was consumed.
 */
bool K230_UART_ReadLine(char *line, size_t capacity);

/*
 * 从串口缓冲区读取并解析一条颜色信息。
 *
 * K230发送：
 *   red\r\n
 *   green\r\n
 *   blue\r\n
 *   none\r\n
 *
 * 返回true：成功读取到一条完整串口消息。
 * 返回false：当前还没有完整的一行。
 *
 * 当收到无法识别的字符串时：
 *   *color = K230_COLOR_INVALID
 */
bool K230_UART_ReadColor(K230_Color *color);

/*
 * Reads and strictly validates one steel-ball message. position_mm may be an
 * integer or a decimal value and is rounded to the nearest integer millimetre.
 * A malformed complete line is consumed and returns false. Call repeatedly
 * while data is available so the controller uses the newest measurement.
 */
bool K230_UART_ReadBall(K230_BallMessage *message);

/*
 * Returns and clears the RX overflow indication. An overflow means one or
 * more newly received bytes were discarded because the ring buffer was full.
 */
bool K230_UART_TakeOverflow(void);
void K230_UART_ClearRx(void);

#ifdef __cplusplus
}
#endif

#endif /* K230_UART_H */
