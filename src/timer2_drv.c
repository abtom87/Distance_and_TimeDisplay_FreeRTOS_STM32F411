/*
 * timer2_src.c
 *
 *  Created on: May 15, 2018
 *      Author: abel
 */

#include "timer2_drv.h"

void enable_alt_func_gpio(void) {
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

     /* PB10 TIM2-CH3 */
     GPIO_InitTypeDef OC_GPIO_Struct;
     OC_GPIO_Struct.GPIO_Mode  = GPIO_Mode_AF;
     OC_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
     OC_GPIO_Struct.GPIO_Pin   = GPIO_Pin_10;
     OC_GPIO_Struct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
     OC_GPIO_Struct.GPIO_Speed = GPIO_Fast_Speed;

     GPIO_Init(GPIOB, &OC_GPIO_Struct);

     GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
}

void init_timer2(void) {
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

     TIM_TimeBaseInitTypeDef TimInitStruct;
     TIM_OCInitTypeDef       Tim2OCInitStr;

     /*Output Freq = Fclock/ ( (ARR+1)*(Prescaler+1)*(RCR+1) )*/

     TimInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
     TimInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;
     TimInitStruct.TIM_Period            = TIMER2_ARR - 1;       /*2499*/
     TimInitStruct.TIM_Prescaler         = TIMER2_PRESCALER - 1; /*99*/
     TimInitStruct.TIM_RepetitionCounter = 0;

     TIM_TimeBaseInit(TIM2, &TimInitStruct);

     Tim2OCInitStr.TIM_Pulse       = (uint32_t)((DUTY_CYCLE * TIMER2_ARR) / 100);
     Tim2OCInitStr.TIM_OCMode      = TIM_OCMode_PWM2;
     Tim2OCInitStr.TIM_OCPolarity  = TIM_OCPolarity_Low;
     Tim2OCInitStr.TIM_OutputState = TIM_OutputState_Enable;
     TIM_OC3Init(TIM2, &Tim2OCInitStr);
     TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

     TIM_Cmd(TIM2, ENABLE);

     // TIM2->CCR3 = (DUTY_CYCLE * TIMER2_ARR / TIMER2_PRESCALER);
}

void enable_timer_irq(void) {
     NVIC_InitTypeDef TIM2_IRQ_Struct;

     TIM2_IRQ_Struct.NVIC_IRQChannel                   = TIM2_IRQn;
     TIM2_IRQ_Struct.NVIC_IRQChannelCmd                = ENABLE;
     TIM2_IRQ_Struct.NVIC_IRQChannelPreemptionPriority = 0;
     TIM2_IRQ_Struct.NVIC_IRQChannelSubPriority        = 0;

     NVIC_Init(&TIM2_IRQ_Struct);
}
