/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_2
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM15)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM15_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_3
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM16)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM16_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (12499U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                400000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_1_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOB
#define GPIO_UART_2_TX_PORT                                                GPIOB
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_16
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_17
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM33)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM43)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM33_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM43_PF_UART2_TX
#define UART_2_BAUD_RATE                                                (115200)
#define UART_2_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_2_FBRD_40_MHZ_115200_BAUD                                      (45)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for led: GPIOB.9 with pinCMx 26 on package pin 61 */
#define LED_led_PIN                                              (DL_GPIO_PIN_9)
#define LED_led_IOMUX                                            (IOMUX_PINCM26)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOA)

/* Defines for key: GPIOA.18 with pinCMx 40 on package pin 11 */
#define KEY_key_PIN                                             (DL_GPIO_PIN_18)
#define KEY_key_IOMUX                                            (IOMUX_PINCM40)
/* Port definition for Pin Group OLED_RST */
#define OLED_RST_PORT                                                    (GPIOB)

/* Defines for PIN_RST: GPIOB.14 with pinCMx 31 on package pin 2 */
#define OLED_RST_PIN_RST_PIN                                    (DL_GPIO_PIN_14)
#define OLED_RST_PIN_RST_IOMUX                                   (IOMUX_PINCM31)
/* Port definition for Pin Group OLED_DC */
#define OLED_DC_PORT                                                     (GPIOB)

/* Defines for PIN_DC: GPIOB.15 with pinCMx 32 on package pin 3 */
#define OLED_DC_PIN_DC_PIN                                      (DL_GPIO_PIN_15)
#define OLED_DC_PIN_DC_IOMUX                                     (IOMUX_PINCM32)
/* Port definition for Pin Group OLED_SCL */
#define OLED_SCL_PORT                                                    (GPIOA)

/* Defines for PIN_SCL: GPIOA.28 with pinCMx 3 on package pin 35 */
#define OLED_SCL_PIN_SCL_PIN                                    (DL_GPIO_PIN_28)
#define OLED_SCL_PIN_SCL_IOMUX                                    (IOMUX_PINCM3)
/* Port definition for Pin Group OLED_SDA */
#define OLED_SDA_PORT                                                    (GPIOA)

/* Defines for PIN_SDA: GPIOA.31 with pinCMx 6 on package pin 39 */
#define OLED_SDA_PIN_SDA_PIN                                    (DL_GPIO_PIN_31)
#define OLED_SDA_PIN_SDA_IOMUX                                    (IOMUX_PINCM6)
/* Port definition for Pin Group IR_AD0 */
#define IR_AD0_PORT                                                      (GPIOA)

/* Defines for PIN_9: GPIOA.9 with pinCMx 20 on package pin 55 */
#define IR_AD0_PIN_9_PIN                                         (DL_GPIO_PIN_9)
#define IR_AD0_PIN_9_IOMUX                                       (IOMUX_PINCM20)
/* Port definition for Pin Group IR_AD1 */
#define IR_AD1_PORT                                                      (GPIOA)

/* Defines for PIN_27: GPIOA.27 with pinCMx 60 on package pin 31 */
#define IR_AD1_PIN_27_PIN                                       (DL_GPIO_PIN_27)
#define IR_AD1_PIN_27_IOMUX                                      (IOMUX_PINCM60)
/* Port definition for Pin Group IR_AD2 */
#define IR_AD2_PORT                                                      (GPIOA)

/* Defines for PIN_24: GPIOA.24 with pinCMx 54 on package pin 25 */
#define IR_AD2_PIN_24_PIN                                       (DL_GPIO_PIN_24)
#define IR_AD2_PIN_24_IOMUX                                      (IOMUX_PINCM54)
/* Port definition for Pin Group IR_OUT */
#define IR_OUT_PORT                                                      (GPIOA)

/* Defines for PIN_12: GPIOA.12 with pinCMx 34 on package pin 5 */
#define IR_OUT_PIN_12_PIN                                       (DL_GPIO_PIN_12)
#define IR_OUT_PIN_12_IOMUX                                      (IOMUX_PINCM34)
/* Port definition for Pin Group GPIO_GRP_14 */
#define GPIO_GRP_14_PORT                                                 (GPIOA)

/* Defines for PIN_14: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_GRP_14_PIN_14_PIN                                  (DL_GPIO_PIN_22)
#define GPIO_GRP_14_PIN_14_IOMUX                                 (IOMUX_PINCM47)
/* Port definition for Pin Group KEY2 */
#define KEY2_PORT                                                        (GPIOB)

/* Defines for PIN_8: GPIOB.8 with pinCMx 25 on package pin 60 */
#define KEY2_PIN_8_PIN                                           (DL_GPIO_PIN_8)
#define KEY2_PIN_8_IOMUX                                         (IOMUX_PINCM25)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.25 with pinCMx 55 on package pin 26 */
// pins affected by this interrupt request:["E1A","E1B"]
#define ENCODERA_INT_IRQN                                       (GPIOA_INT_IRQn)
#define ENCODERA_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO25)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_25)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM55)
/* Defines for E1B: GPIOA.26 with pinCMx 59 on package pin 30 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO26)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_26)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM59)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for E2A: GPIOB.20 with pinCMx 48 on package pin 19 */
// pins affected by this interrupt request:["E2A","E2B"]
#define ENCODERB_INT_IRQN                                       (GPIOB_INT_IRQn)
#define ENCODERB_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO20)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_20)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM48)
/* Defines for E2B: GPIOB.24 with pinCMx 52 on package pin 23 */
#define ENCODERB_E2B_IIDX                                   (DL_GPIO_IIDX_DIO24)
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_24)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM52)
/* Port definition for Pin Group AIN */
#define AIN_PORT                                                         (GPIOA)

/* Defines for AIN1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_14)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM36)
/* Defines for AIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define AIN_AIN2_PIN                                            (DL_GPIO_PIN_13)
#define AIN_AIN2_IOMUX                                           (IOMUX_PINCM35)
/* Port definition for Pin Group BIN */
#define BIN_PORT                                                         (GPIOA)

/* Defines for BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define BIN_BIN1_PIN                                            (DL_GPIO_PIN_16)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM38)
/* Defines for BIN2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define BIN_BIN2_PIN                                            (DL_GPIO_PIN_17)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM39)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_DEBUG_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_2_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
