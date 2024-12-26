/*
 * 02uart_case.c
 *
 *  Created on: Dec 21, 2024
 *      Author: sleman
 */


#include<stdio.h>
#include<string.h>
#include "stm32f401xx.h"
#include "ds1307.h"

#define DS1307_SET		0



void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	if(ds1307_init())
	{
			while(1)
		{
			__asm("NOP"); // error
		}
	}


#if (DS1307_SET)
	current_date.day = WEDNESDAY;
	current_date.date = 25;
	current_date.month = 12;
	current_date.year = 24;

	current_time.hours = 19;
	current_time.minutes = 3;
	current_time.seconds = 0;
	current_time.time_format = TIME_FORMAT_24HRS;

	ds1307_set_current_time(&current_time);
	ds1307_set_current_date(&current_date);

#endif

	ds1307_get_current_time(&current_time);
	ds1307_get_current_date(&current_date);


	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format ? "PM" : "AM");
	}

    while(1)
    {
    	ds1307_get_current_date(&current_date);
		ds1307_get_current_time(&current_time);
    	__asm("NOP");
    }

}

