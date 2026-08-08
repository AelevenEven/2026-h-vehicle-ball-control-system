#ifndef K230_UART_STATS_H
#define K230_UART_STATS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Runtime diagnostics for the K230 -> MSPM0 BALL UART stream. */
typedef struct
{
    uint32_t rx_bytes;
    uint32_t complete_lines;
    uint32_t valid_ball_frames;
    uint32_t parse_errors;
    uint32_t overflows;
} K230UartStats;

/* Copies a coherent-enough diagnostic snapshot for OLED display. */
void K230_UART_GetStats(K230UartStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* K230_UART_STATS_H */
