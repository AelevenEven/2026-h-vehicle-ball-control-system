#include "ti_msp_dl_config.h"
#include "board.h"

volatile uint32_t tick_ms;

#define BOARD_DEBUG_TX_BUFFER_SIZE    (512U)

static volatile uint8_t s_debug_tx_buffer[BOARD_DEBUG_TX_BUFFER_SIZE];
static volatile uint16_t s_debug_tx_head;
static volatile uint16_t s_debug_tx_tail;
static volatile uint32_t s_debug_tx_dropped;

/*
 * 这里将 SysTick 配置成 1 ms 中断，统一提供系统时间。
 */
void SysTick_Init(void)//开启每1ms的系统计时
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 3);
}

void SysTick_Handler(void)
{
    tick_ms++;
}

uint32_t Board_GetMillis(void)//获取上电到现在的毫秒数
{
    return tick_ms;
}

/*
 * The on-board CH9102 receives UART0_TX on PA10. PA0/PA1 are reserved for
 * MPU6050 I2C, so UART0 uses its PA10/PA11 alternate pin pair.
 */
void Board_EnableProgrammingUartTx(void)
{
    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

uint8_t Board_DebugWriteByte(uint8_t data)
{
    uint16_t next_head;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    next_head = (uint16_t)(s_debug_tx_head + 1U);
    if (next_head >= BOARD_DEBUG_TX_BUFFER_SIZE) {
        next_head = 0U;
    }
    if (next_head == s_debug_tx_tail) {
        s_debug_tx_dropped++;
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    s_debug_tx_buffer[s_debug_tx_head] = data;
    s_debug_tx_head = next_head;
    /*
     * Prime the hardware FIFO immediately. Relying only on an initially
     * empty-FIFO interrupt is not guaranteed to fire when a new software
     * transmission begins.
     */
    Board_DebugTxIrqHandler();
    if (s_debug_tx_tail != s_debug_tx_head) {
        DL_UART_Main_enableInterrupt(
            UART_0_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

void Board_DebugTxIrqHandler(void)
{
    while ((s_debug_tx_tail != s_debug_tx_head) &&
           (!DL_UART_Main_isTXFIFOFull(UART_0_INST))) {
        DL_UART_Main_transmitData(
            UART_0_INST, s_debug_tx_buffer[s_debug_tx_tail]);
        s_debug_tx_tail++;
        if (s_debug_tx_tail >= BOARD_DEBUG_TX_BUFFER_SIZE) {
            s_debug_tx_tail = 0U;
        }
    }

    if (s_debug_tx_tail == s_debug_tx_head) {
        DL_UART_Main_disableInterrupt(
            UART_0_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

uint32_t Board_DebugDroppedBytes(void)
{
    return s_debug_tx_dropped;
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
            (void)DL_UART_Main_receiveData(UART_0_INST);
        }
        break;

    case DL_UART_MAIN_IIDX_TX:
    default:
        break;
    }
    Board_DebugTxIrqHandler();
}

//返回SysTick计数值
uint32_t Systick_getTick(void)
{
	return (SysTick->VAL);
}


//ms阻塞延迟
void delay_ms(uint32_t ms)
{
	/* 使用毫秒计数等待，避免 CPU 主频变化后原来的空循环延时严重不准。 */
	uint32_t start = Board_GetMillis();
	while ((uint32_t)(Board_GetMillis() - start) < ms) {
		__WFI();
	}
}


void delay_us(uint32_t us)
{
	/* 按 CPU 实际频率换算周期，使微秒延时不再依赖固定魔数。 */
	if (us != 0U) {
		delay_cycles(us * (CPUCLK_FREQ / 1000000U));
	}
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
#endif

/*
 * Redirect printf to UART0 TX on PA10 for the on-board CH9102.
 */
int fputc(int ch, FILE *stream)
{
    (void)stream;
    (void)Board_DebugWriteByte((uint8_t)ch);
    return ch;
}
