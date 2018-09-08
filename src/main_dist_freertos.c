/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include <main_dist_freertos.h>

// Macro to use CCM (Core Coupled Memory) in STM32F4
#define CCM_RAM __attribute__((section(".ccmram")))

#define USE_FREERTOS_ISR_API
//#define ENABLE_DEBUG

#define GPS_BUFFER_SIZE 400

#define TASK1_STACK_SIZE 128
#define TASK2_STACK_SIZE 256
#define TASK3_STACK_SIZE 256
#define TASK4_STACK_SIZE 512
#define TASK5_STACK_SIZE 512

#define TASK1_PRIO 1
#define TASK2_PRIO 3
#define TASK3_PRIO 5
#define TASK4_PRIO 2
#define TASK5_PRIO 1

StackType_t Task1_Stack[TASK1_STACK_SIZE] CCM_RAM; /* Put task stack in CCM */
StackType_t Task2_Stack[TASK2_STACK_SIZE] CCM_RAM; /* Put task stack in CCM */
StackType_t Task3_Stack[TASK3_STACK_SIZE] CCM_RAM;
StackType_t Task4_Stack[TASK4_STACK_SIZE] CCM_RAM;
StackType_t Task5_Stack[TASK4_STACK_SIZE] CCM_RAM;

StaticTask_t Task1Buff CCM_RAM; /* Put TCB in CCM */
StaticTask_t Task2Buff CCM_RAM; /* Put TCB in CCM */
StaticTask_t Task3Buff CCM_RAM; /* Put TCB in CCM */
StaticTask_t Task4Buff CCM_RAM; /* Put TCB in CCM */
StaticTask_t Task5Buff CCM_RAM; /* Put TCB in CCM */

SemaphoreHandle_t xBinarySemaphore_InpCap;
SemaphoreHandle_t xBinarySemaphore_DMA;
SemaphoreHandle_t xBinarySemaphore_InpCapDone;
SemaphoreHandle_t xBinarySem_ParsingDone;
SemaphoreHandle_t xBinarySem_LCD;

QueueHandle_t xQueueTimeStamp;
QueueHandle_t xQueueDispDistBuff;
QueueHandle_t xQueueTimeBuff;
QueueHandle_t xQueueTimeDispBuff;

//volatile static task_dur_t task_duration;

#define DISP_BUFF_LEN 15

char buffer[DISP_BUFF_LEN] = { 0 };
char time_buff[DISP_BUFF_LEN] = { 0 };

void Delay_ms(uint16_t millisec)
{
	uint32_t i, result;
	result = 25000 * millisec; /*measured using Logic-analyzer; corresponds to ~1ms*/
	for (i = 0; i < result; i++)
	{
		__asm__("nop");
	}
}

/*TASK 1*/
void vTask1_toggleLED(void* p)
{

	while (1)
	{
		toggle_leds();
		vTaskDelay(500);

	}

}

/*TASK 2
 * Calculates the distance of the object from ultrasound sensor
 *  based on the content in CCR register
 *
 * */
void vTask_HandlerInpCapture(void* p)
{

	static timestamp_t rcvd_tmpstamp;
	float distance = 0;
	float temp_dist = 0;
	unsigned long characteristic = 0;
	unsigned long mantissa = 0;

	char *distance_buf_ptr;
	distance_buf_ptr = &buffer[0];

	char timestampbuf[10] = { 0 };

	while (1)
	{
		

		//	task_duration.begin = xTaskGetTickCount();
		if (xSemaphoreTake(xBinarySemaphore_InpCap, portMAX_DELAY) == pdTRUE)
		{
			TIM_Cmd(TIM2, DISABLE);
			xQueueReceive(xQueueTimeStamp, &rcvd_tmpstamp, 0);
//			distance = (float) ((float) rcvd_tmpstamp.difference
//					/ (float) INP_CAPT_FACTOR);
			distance = (float) rcvd_tmpstamp.difference;
			distance /= (float) (INP_CAPT_FACTOR);

			distance *= (float) 17000;
			characteristic = (unsigned long) distance;
			temp_dist = (distance - (float) characteristic) * (float) 1000;
			//mantissa = ( distance - characteristic) * 1000;
			mantissa = (unsigned long) temp_dist;
			sprintf(timestampbuf, "%lu", rcvd_tmpstamp.difference);
			sprintf(buffer, "Dist: %lu.%lucm", characteristic, mantissa);

			xQueueSend(xQueueDispDistBuff, &distance_buf_ptr, 0);

			xSemaphoreGive(xBinarySemaphore_InpCapDone);

			TIM_Cmd(TIM2, ENABLE);

#ifdef ENABLE_DEBUG
			USART_TX_string(timestampbuf);
			USART_TX_string(" ");
#endif
		}
		vTaskDelay(200);
	}

}

/*TASK 3*//*Disabled for now*/
void vTask_DisplayDistance(void *p)
{
	char *Rx_Buff_ptr;
	while (1)
	{
		if (xSemaphoreTake(xBinarySemaphore_InpCapDone, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(xQueueDispDistBuff, &Rx_Buff_ptr, 0);

			LCD_Goto(1, 2);
			LCD_Write_String((const char*) Rx_Buff_ptr);

			vTaskDelay(150);
			xSemaphoreGive(xBinarySem_LCD);
		}
	}
}

/* TASK 4
 * Extracts the string for time from the GPS buffer
 * This buffer is received when DMA interrupt triggers
 * */

void vTask_parseTime(void *p)
{
	char *TimeBuf;
	char *buff_to_display_ptr;

	buff_to_display_ptr = &time_buff[0];
	while (1)
	{

		if (xSemaphoreTake(xBinarySemaphore_DMA, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(xQueueTimeBuff, &TimeBuf, 0);

			//parse time
			parse_time(TimeBuf, "$GPRMC,", buff_to_display_ptr, 7);

			xQueueSend(xQueueTimeDispBuff, &buff_to_display_ptr, 0);
			xSemaphoreGive(xBinarySem_ParsingDone);

		}
		vTaskDelay(100);

	}
}

/* TASK 5
 * Receives respective buffers for Distance and Time
 * and displays sequentially over LCD
 * */

void vTask_DispTimeandDist(void *p)
{
	char *Disp_Time_Buff_ptr;
	char *Rx_Buff_ptr;
	while (1)
	{

		if (xSemaphoreTake(xBinarySemaphore_InpCapDone, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(xQueueDispDistBuff, &Rx_Buff_ptr, 0);

			LCD_Goto(1, 2);
			LCD_Write_String((const char*) Rx_Buff_ptr);
		}

		if (xSemaphoreTake(xBinarySem_ParsingDone, portMAX_DELAY) == pdTRUE)
		{
			{
				xQueueReceive(xQueueTimeDispBuff, &Disp_Time_Buff_ptr, 0);
				LCD_Goto(1, 1);
				LCD_Write_String("Time: ");
				LCD_Write_String((const char*) Disp_Time_Buff_ptr);
			}
		}		
		vTaskDelay(300);

	}

}


static void init_HW_peripherals(void)
{
	/*Initialize USART2 peripheral */
		init_USART2();
		init_led_gpios();
		//init_debug_pin();  /* Initialize Debug pin LogicAnalyzer*/

		/* Initialize I2C1 peripheral*/
		init_i2c_bus_config();
		init_i2c_gpio();

		/* Initialize the LCD using initialization commands */
		LCD_Init();
		LCD_Clear();

		/* Initialize Timer2 to trigger the Ultrasound HC04 module*/
		init_timer2();
		enable_alt_func_gpio();

		/* Initialize Timer1 to initialize the input capture to receive echo from
		 *  the Ultrasound HC04 module*/
		init_capture_gpio();
		init_inp_capture_module();
		enable_inp_capture_irq();

		/* initialize DMA module for receiving GPS data */
		init_dma2();
		enable_dma2_irq();

		/* Initialize USART module to receive GPS data*/
		init_usart6_comm_module();
		init_usart6_gpio();

}

int main(void)
{
	SystemInit();

	/* Initialise all the relevant peripherals*/
	init_HW_peripherals();

	/*Create a semaphore for synchronizing Input capture interrupt and Distance calculation*/
	xBinarySemaphore_InpCap = xSemaphoreCreateBinary();
	/* Create semaphore to synchronize DMA interrupt and parser task*/
	xBinarySemaphore_DMA = xSemaphoreCreateBinary();
	/* Create semaphore to synchronize distance calculation task and display task*/
	xBinarySemaphore_InpCapDone = xSemaphoreCreateBinary();
	/*  Create semaphore to synchronize parser task and display task*/
	xBinarySem_ParsingDone = xSemaphoreCreateBinary();
	xBinarySem_LCD = xSemaphoreCreateBinary();

	/* Create  a queue to send timestamp structure to Distance calculation task*/
	xQueueTimeStamp = xQueueCreate(1, sizeof(timestamp_t));
	/* Create  a queue to send a char buffer (calculated distance) to Display task*/
	xQueueDispDistBuff = xQueueCreate(DISP_BUFF_LEN, sizeof(char *));
	/* Create  a queue to send a char buffer(GPS Data) to parser task*/
	xQueueTimeBuff = xQueueCreate(GPS_BUFFER_SIZE, sizeof(char*));
	/* Create  a queue to send a char buffer (Time) to Display task*/
	xQueueTimeDispBuff = xQueueCreate(DISP_BUFF_LEN, sizeof(char*));

	if (xQueueTimeStamp == NULL)
		USART_TX_string("Queue Creation xQueueTimeStamp Failed!\r\n");

	if (xQueueDispDistBuff == NULL)
		USART_TX_string("Queue Creation xQueueDispDistBuff Failed!\r\n");

	if (xQueueTimeBuff == NULL)
		USART_TX_string("Queue Creation xQueueTimeBuff Failed!\r\n");

	if (xQueueTimeDispBuff == NULL)
		USART_TX_string("Queue Creation xQueueTimeDispBuff Failed!\r\n");

	/*Make sure Semaphore creation was successful*/
	if ((xBinarySemaphore_InpCap != NULL) && (xBinarySemaphore_DMA != NULL)
			&& (xBinarySemaphore_InpCapDone != NULL)
			&& (xBinarySem_ParsingDone != NULL))
	{

		/*Create a task */
		/*Stack and TCB are placed in CCM of STM32F4 */
		/* The CCM block is connected directly to the core, which leads to zero wait states */

		xTaskCreateStatic(vTask1_toggleLED, "tsk_toggleLEDs", TASK1_STACK_SIZE,
		NULL,
		TASK1_PRIO, Task1_Stack, &Task1Buff);

		xTaskCreateStatic(vTask_HandlerInpCapture, "tsk_IC_Hndlr",
		TASK2_STACK_SIZE,
		NULL,
		TASK2_PRIO, Task2_Stack, &Task2Buff);

//		xTaskCreateStatic(vTask_DisplayDistance, "tsk_Disp_Dist",
//		TASK3_STACK_SIZE,
//		NULL, TASK3_PRIO, Task3_Stack, &Task3Buff);

		xTaskCreateStatic(vTask_parseTime, "tsk_Parse_Time",
		TASK4_STACK_SIZE,
		NULL, TASK4_PRIO, Task4_Stack, &Task4Buff);

		xTaskCreateStatic(vTask_DispTimeandDist, "tsk_Disp_Time",
		TASK5_STACK_SIZE,
		NULL, TASK5_PRIO, Task5_Stack, &Task5Buff);

		USART_TX_string("Starting scheduler...\r\n");

		vTaskStartScheduler();  // should never return
	}

	while (1)
	{
	}
}

/*DMA IRQ HANDLER*/
static char gps_string[GPS_BUFFER_SIZE] = { 0 };
void DMA2_Stream2_IRQHandler(void)
{

	char *time_buf_ptr;

	time_buf_ptr = gps_string;

	static BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	if (DMA_GetITStatus(DMA2_USART6_STREAM, DMA_IT_TCIF2) != RESET)
	{
		GPIO_ToggleBits(GPIOD, GPIO_Pin_15); /* Toggles Blue LED on board*/

		memcpy(gps_string, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);

		DMA_ClearITPendingBit(DMA2_USART6_STREAM, DMA_IT_TCIF2);
		DMA_ClearITPendingBit(DMA2_USART6_STREAM, DMA_IT_HTIF2);
		DMA_ClearITPendingBit(DMA2_USART6_STREAM, DMA_IT_FEIF2);
		DMA_ClearITPendingBit(DMA2_USART6_STREAM, DMA_IT_DMEIF2);
		DMA_ClearITPendingBit(DMA2_USART6_STREAM, DMA_IT_TEIF2);

		/* Pass the gps buffer to handler task  */
		xQueueSendFromISR(xQueueTimeBuff, &time_buf_ptr,
				&xHigherPriorityTaskWoken);

		xSemaphoreGiveFromISR(xBinarySemaphore_DMA, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}

}

void TIM1_CC_IRQHandler(void)
{
	static timestamp_t pulse;



	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if ((TIM_GetITStatus(TIM1, TIM_IT_CC1)) != RESET)
	{

		switch (pulse.next_edge)
		{
		case e_initial:
			pulse.rising_tick = TIM_GetCapture1(TIM1);
			pulse.next_edge = e_falling;
			break;

		case e_rising:

			pulse.rising_tick = TIM_GetCapture1(TIM1);
			pulse.current_edge = e_rising;
			pulse.next_edge = e_falling;
			break;

		case e_falling:
			pulse.falling_tick = TIM_GetCapture1(TIM1);
			TIM1->CCR1 = 0;
			pulse.current_edge = e_falling;
			pulse.next_edge = e_rising;

			break;

		default:
			break;

		}

		if (pulse.current_edge == e_falling)
		{
			if (pulse.falling_tick > pulse.rising_tick)
				pulse.difference = (pulse.falling_tick - pulse.rising_tick);
			else
			{
				TIM1->CCR1 = 0;
			}
		}

		/* Pass the pulse structure to handler task  */
		xQueueSendFromISR(xQueueTimeStamp, &pulse, &xHigherPriorityTaskWoken);

		/* Give the semaphore to handler task */
		xSemaphoreGiveFromISR(xBinarySemaphore_InpCap,
				&xHigherPriorityTaskWoken);

		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
		//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}


}

void vApplicationTickHook(void)
{
}

/* vApplicationMallocFailedHook() will only be called if
 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
 function that will get called if a call to pvPortMalloc() fails.
 pvPortMalloc() is called internally by the kernel whenever a task, queue,
 timer or semaphore is created.  It is also called by various parts of the
 demo application.  If heap_1.c or heap_2.c are used, then the size of the
 heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
 FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
 to query the size of free heap space that remains (although it does not
 provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
 to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
 task.  It is essential that code added to this hook function never attempts
 to block in any way (for example, call xQueueReceive() with a block time
 specified, or call vTaskDelay()).  If the application makes use of the
 vTaskDelete() API function (as this demo application does) then it is also
 important that vApplicationIdleHook() is permitted to return to its calling
 function, because it is the responsibility of the idle task to clean up
 memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void)
{
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

StaticTask_t xIdleTaskTCB CCM_RAM;
StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE] CCM_RAM;

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
	 state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	 Note that, as the array is necessarily of type StackType_t,
	 configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB CCM_RAM;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] CCM_RAM;

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 application must provide an implementation of vApplicationGetTimerTaskMemory()
 to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
		StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

