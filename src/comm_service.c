/*
 * comm_service.c
 *
 *  Created on: Jan 28, 2018
 *      Author: abel
 */

#include "comm_service.h"

#define SUMMER_TIME

void parser_gprmc (char *str_to_parse)
{
    static int k = 0;
    char *substr;
    substr = strstr (str_to_parse, "$GPRMC");

    while (substr[k] != '\n')
    {
        // USART_TX_byte(substr[k]);
        k++;
    }
    substr[k] = 0;

    USART_TX_byte ('\n');
}

void Display_Time_LCD (char *str_to_parse, char *str_to_extract, const uint8_t offset)
{
    int k = 0;
    unsigned char *substr;
    unsigned char Time_buffer[10] = { 0 };

    substr = strstr ((const char *)str_to_parse, (const char *)str_to_extract);
    k = offset - 1;
    while (k < 13)
    {
        Time_buffer[k - offset] = (unsigned char)(substr[k]);
        k++;
    }


    if (Time_buffer[1] == 0x39)
    {
        Time_buffer[1] = 0x30;
        Time_buffer[0] += 1;
    }
    else
        Time_buffer[1] += 1;

    LCD_Goto (7, 1);
    LCD_Write_String ((const char *)Time_buffer);
}

char parse_buffer[PARSE_BUFF_SIZE] = { 0 };
char offset_string[OFFSET_BUFF_SIZE] = { 0 };

void parse_time (char *str_to_parse, char *str_to_extract, char *time_to_disp_buff, uint8_t offset)
{
    char *ptr_extracted_string = NULL;
    ptr_extracted_string = parse_buffer;
    char extracted_string[EXTRACTED_STRING_SIZE] = { 0 };
    char TimeDisplayBuff[OFFSET_BUFF_SIZE] = { 0 };
    int k;

    strncpy (parse_buffer, str_to_parse, strlen (str_to_parse));
    parse_buffer[strlen (str_to_parse)] = 0; /* NULL_Terminate the string */
    strncpy (offset_string, str_to_extract, strlen (str_to_extract));
    offset_string[strlen (str_to_extract)] = 0; /* NULL_Terminate the string */

    ptr_extracted_string = strstr ((const char *)&parse_buffer, (const char *)&offset_string);
    *(ptr_extracted_string + strlen (ptr_extracted_string)) = 0;
    strncpy (extracted_string, ptr_extracted_string, TIME_EXTRACTION_SIZE);
    // USART_TX_string(extracted_string);

    for (k = 0; k < offset; k++)
    {
        TimeDisplayBuff[k] = extracted_string[k + offset];
    }
    strncpy (time_to_disp_buff, TimeDisplayBuff, OFFSET_BUFF_SIZE);

#ifdef SUMMER_TIME
    if (time_to_disp_buff[1] == 0x38)
    {
        time_to_disp_buff[1] = 0x30;
        time_to_disp_buff[0] += 1;
    }
    else if (time_to_disp_buff[1] == 0x39)
    {
        time_to_disp_buff[1] = 0x31;
        time_to_disp_buff[0] += 1;
    }
    else
        time_to_disp_buff[1] += 2;
#else
    time_to_disp_buff[1] += 1;
#endif
}
