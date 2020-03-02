/*
 * inpcap_tim1.h
 *
 *  Created on: May 15, 2018
 *      Author: abel
 */

#ifndef TIMER1_INPCAP_DRV_H_
#define TIMER1_INPCAP_DRV_H_

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#define IC_PRESCALER    200lu
#define INP_CAPT_FACTOR (100000000lu / IC_PRESCALER)


void init_inp_capture_module (void);
void enable_inp_capture_irq (void);
void init_capture_gpio (void);

#endif /* TIMER1_INPCAP_DRV_H_ */
