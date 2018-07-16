/*
 * comm_service.c
 *
 *  Created on: Jan 28, 2018
 *      Author: abel
 */

#include "comm_service.h"

#define SUMMER_TIME

void parser_gprmc(char *str_to_parse)
{
	static int k = 0;
	char *substr;
	substr = strstr(str_to_parse, "$GPRMC");

	while (substr[k] != '\n')
	{
		//USART_TX_byte(substr[k]);
		k++;
	}
	substr[k] = 0;

	USART_TX_byte('\n');

}

void Display_Time_LCD(char *str_to_parse, char *str_to_extract,
		const uint8_t offset)
{
	int k = 0;
	unsigned char *substr;
	unsigned char Time_buffer[10] = { 0 };

	substr = strstr((const char *) str_to_parse, (const char *) str_to_extract);
	k = offset - 1;
	while (k < 13)
	{
		Time_buffer[k - offset] = (unsigned char) (substr[k]);
		k++;
	}

	if (Time_buffer[1] == 0x39)
	{
		Time_buffer[1] = 0x30;
		Time_buffer[0] += 1;
	}
	else
		Time_buffer[1] += 1;

	LCD_Goto(7, 1);
	LCD_Write_String((const char *) Time_buffer);

}

void parse_time(char *str_to_parse, char *str_to_extract,
		char *time_to_disp_buff, uint8_t offset)
{
	unsigned char *substr;
	int k = 0;
	substr = strstr((const char *) str_to_parse, (const char *) str_to_extract);
	k = offset - 1;
	while (k < 13)
	{
		time_to_disp_buff[k - offset] = (unsigned char) (substr[k]);
		k++;
	}

	if (time_to_disp_buff[1] == 0x39)
	{
#ifdef  SUMMER_TIME
		time_to_disp_buff[1] = 0x30;
		time_to_disp_buff[0] += 2;
#else
		time_to_disp_buff[1] = 0x30;
		time_to_disp_buff[0] += 1;


#endif


	}
	else
#ifdef  SUMMER_TIME
		time_to_disp_buff[1] += 2;
#else
	time_to_disp_buff[1] += 1;
#endif
}

