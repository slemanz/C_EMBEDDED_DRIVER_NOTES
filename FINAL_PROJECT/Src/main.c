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

	current_date.day = TUESDAY;
	current_date.date = 24;
	current_date.month = 12;
	current_date.year = 24;

	current_time.hours = 16;
	current_time.minutes = 8;
	current_time.seconds = 0;
	current_time.time_format = TIME_FORMAT_24HRS;

	//ds1307_set_current_time(&current_time);
	//ds1307_set_current_date(&current_date);


	ds1307_get_current_date(&current_date);
	ds1307_get_current_time(&current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format ? "PM" : "AM");
	}

    while(1)
    {
    	__asm("NOP");
    }

}

