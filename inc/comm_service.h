/*
 * comm_service.h
 *
 *  Created on: Jan 28, 2018
 *      Author: abel
 */

#ifndef COMM_SERVICE_H_
#define COMM_SERVICE_H_

#include "usart2_comm.h"
#include "i2c_lcd_dist.h"
#include <string.h>

void parser_gprmc(char *);
extern void Display_Time_LCD(char *str_to_parse, char *str_to_extract, const uint8_t ofsset);
extern void parse_time(char *, char *, char *, uint8_t );

#endif /* COMM_SERVICE_H_ */
