/*
 * comm_service.h
 *
 *  Created on: Jan 28, 2018
 *      Author: abel
 */

#ifndef COMM_SERVICE_H_
#define COMM_SERVICE_H_

#include "usart2_comm.h"
#include "dma2_driver.h"
#include "i2c_lcd_dist.h"
#include <string.h>

#define PARSE_BUFF_SIZE       (DMA_RX_BUFFER_SIZE + 10)
#define OFFSET_BUFF_SIZE      10
#define TIME_EXTRACTION_SIZE  13
#define EXTRACTED_STRING_SIZE 20

void parser_gprmc(char *);
extern void Display_Time_LCD(char *str_to_parse, char *str_to_extract, const uint8_t ofsset);
extern void parse_time(char *, char *, char *, uint8_t );

#endif /* COMM_SERVICE_H_ */
