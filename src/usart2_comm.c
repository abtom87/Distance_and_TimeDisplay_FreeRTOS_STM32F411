/*
 * usart2_comm.c
 *
 *  Created on: Apr 1, 2018
 *      Author: abel
 */

#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOSConfig.h"

void USART_TX_byte(uint8_t byte)
{
	while ((SET != USART_GetFlagStatus(USART2, USART_FLAG_TXE)))
		;
	USART_SendData(USART2, (uint16_t) byte);

}

void USART_TX_string(char *str_to_transmit)
{
	int i=0;


	while(*(str_to_transmit + i)!=0)
	{
		USART_TX_byte(*(str_to_transmit + i));
		++i;
	}

}
/*
 * Configure USART2(PD5 Tx, PD6 Rx) to redirect printf data to host PC.
 */
void init_USART2(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStruct);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	USART_Cmd(USART2, ENABLE);

}

void enable_usart2_irq(void)
{

	NVIC_InitTypeDef NVIC_USARTStruct;

	NVIC_USARTStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_USARTStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_USARTStruct.NVIC_IRQChannelPreemptionPriority =
			configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1;
	NVIC_USARTStruct.NVIC_IRQChannelSubPriority = 0;

	//NVIC_SetPriority(USART2_IRQn,5);
	NVIC_Init(&NVIC_USARTStruct);

}

