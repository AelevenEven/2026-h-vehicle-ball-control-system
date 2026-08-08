#ifndef _ENCODER_H
#define _ENCODER_H
#include "ti_msp_dl_config.h"
#include "board.h"

/*
 * 修改原因：计数值由 GPIO 中断修改、由定时器中断读取；volatile 防止编译器缓存旧值，
 * int32_t 则保证高速运行时有足够的计数范围。
 */
extern volatile int32_t Get_Encoder_countA;
extern volatile int32_t Get_Encoder_countB;

#endif
