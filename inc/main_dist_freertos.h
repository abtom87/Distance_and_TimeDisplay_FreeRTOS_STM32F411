/*
 * main.h
 *
 *  Created on: Jan 27, 2018
 *      Author: abel
 */

#ifndef MAIN_DIST_FREERTOS_H_
#define MAIN_DIST_FREERTOS_H_
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_rcc.h"

#include <i2c1_drv.h>
#include <i2c_lcd_dist.h>
#include "comm_service.h"
#include "dma2_driver.h"
#include "gpio_led.h"
#include "math.h"
#include "semphr.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "task.h"
#include "timer1_inpcap_drv.h"
#include "timer2_drv.h"
#include "usart2_comm.h"
#include "usart6_dma_comm.h"

typedef enum { e_initial = 0, e_rising = 1, e_falling = 2 } edge_t;

typedef struct {
     uint32_t rising_tick;
     uint32_t falling_tick;
     uint32_t difference;
     edge_t   current_edge;
     edge_t   next_edge;

} timestamp_t;

typedef struct {
     uint32_t begin;
     uint32_t end;
     uint32_t duration;

} task_dur_t;

extern void Delay_ms(uint16_t);
void        init_HW_peripherals(void);
void        print_elapsed_time_ticks(TickType_t* start, TickType_t* end);
#define TRUE 1
#define FALSE 0

void initialiseTimer(void);
void SysTick_Init(void);

void enable_Tim_Interrupts(void);

// GPIO_InitTypeDef GPIO_InitDef;

#endif /* MAIN_DIST_FREERTOS_H_ */
