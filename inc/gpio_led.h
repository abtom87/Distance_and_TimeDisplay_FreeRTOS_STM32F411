/*
 * gpio_led.h
 *
 *  Created on: Jan 31, 2018
 *      Author: abel
 */

#ifndef GPIO_LED_H_
#define GPIO_LED_H_



#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_gpio.h"


void init_led_gpios(void);
void toggle_leds(void);
void init_debug_pin(void);

#endif /* GPIO_LED_H_ */
