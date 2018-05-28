/*
 * inpcap_tim1.c
 *
 *  Created on: May 15, 2018
 *      Author: abel
 */

#include "timer1_inpcap_drv.h"

void init_capture_gpio(void)
{

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_IC_InitStr;

	/* PIN A8 is used for input capture*/

	GPIO_IC_InitStr.GPIO_Mode = GPIO_Mode_AF;
	GPIO_IC_InitStr.GPIO_OType = GPIO_OType_PP;
	GPIO_IC_InitStr.GPIO_Pin = GPIO_Pin_8;
	GPIO_IC_InitStr.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_IC_InitStr.GPIO_Speed = GPIO_Fast_Speed;

	GPIO_Init(GPIOA, &GPIO_IC_InitStr);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

//	GPIO_ResetBits(GPIOA,GPIO_Pin_8);

}

void init_inp_capture_module(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_ICInitTypeDef TIM1_ICInitStruct;

	TIM1_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM1_ICInitStruct.TIM_ICFilter = 0x0F;
	TIM1_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM1_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM1_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;

	TIM_ICInit(TIM1, &TIM1_ICInitStruct);

	/*Output Freq = Fclock/ ( (ARR+1)*(Prescaler+1)*(RCR+1) )*/
	TIM_TimeBaseInitTypeDef TIM1_InitStruct;
	TIM1_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM1_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM1_InitStruct.TIM_Period = 65535; /*ARR=65535*/
	TIM1_InitStruct.TIM_Prescaler = IC_PRESCALER - 1;
	TIM1_InitStruct.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM1_InitStruct);

	TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

	TIM_Cmd(TIM1, ENABLE);

}

void enable_inp_capture_irq(void)
{
	NVIC_InitTypeDef IC_IRQStruct;

	IC_IRQStruct.NVIC_IRQChannel = TIM1_CC_IRQn;
	IC_IRQStruct.NVIC_IRQChannelCmd = ENABLE;
	IC_IRQStruct.NVIC_IRQChannelPreemptionPriority = 2;
	IC_IRQStruct.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&IC_IRQStruct);
}
