#include "encoder.h"

volatile int32_t Get_Encoder_countA;
volatile int32_t Get_Encoder_countB;
/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/
void GROUP1_IRQHandler(void)
{
	uint32_t status;

	
	switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
	case ENCODERA_INT_IIDX://GPIOA
		status = DL_GPIO_getEnabledInterruptStatus(
			GPIOA, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
		//如果B相为高电平，超前A相90度，则正转
		if ((status & ENCODERA_E1A_PIN) != 0U) {//A相上升沿
			Get_Encoder_countA += DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN) ? 1 : -1;
		}
		if ((status & ENCODERA_E1B_PIN) != 0U) {//B相上升沿
			Get_Encoder_countA += DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN) ? -1 : 1;
		}
		DL_GPIO_clearInterruptStatus(GPIOA, status);
		break;

	case ENCODERB_INT_IIDX://GPIOB
		status = DL_GPIO_getEnabledInterruptStatus(
			ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
		if ((status & ENCODERB_E2A_PIN) != 0U) {
			Get_Encoder_countB += DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN) ? 1 : -1;
		}
		if ((status & ENCODERB_E2B_PIN) != 0U) {
			Get_Encoder_countB += DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN) ? -1 : 1;
		}
		DL_GPIO_clearInterruptStatus(ENCODERB_PORT, status);
		break;

	default:
		break;
	}
}
