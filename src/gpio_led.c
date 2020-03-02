/*
 * gpio_led.c
 *
 *  Created on: Jan 31, 2018
 *      Author: abel
 */

#include "gpio_led.h"

void init_debug_pin(void) {
     GPIO_InitTypeDef gpioDebugInitStr;
     /* Initialise PORTC */
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

     gpioDebugInitStr.GPIO_Pin   = GPIO_Pin_10;
     gpioDebugInitStr.GPIO_Mode  = GPIO_Mode_OUT;
     gpioDebugInitStr.GPIO_OType = GPIO_OType_PP;
     gpioDebugInitStr.GPIO_PuPd  = GPIO_PuPd_NOPULL;
     gpioDebugInitStr.GPIO_Speed = GPIO_Speed_50MHz;

     GPIO_Init(GPIOC, &gpioDebugInitStr);
     //	GPIO_ResetBits(GPIOC, GPIO_Pin_10);
}

void init_led_gpios(void) {
     GPIO_InitTypeDef gpioInitStr;
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

     gpioInitStr.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
     gpioInitStr.GPIO_Mode  = GPIO_Mode_OUT;
     gpioInitStr.GPIO_OType = GPIO_OType_PP;
     gpioInitStr.GPIO_PuPd  = GPIO_PuPd_NOPULL;
     gpioInitStr.GPIO_Speed = GPIO_Speed_50MHz;

     GPIO_Init(GPIOD, &gpioInitStr);
}

void toggle_leds() {
     GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
     GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
     GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
     // GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
}
