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

#define DS1307_SET		1


void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

	/* calculation of reload */
}


char* get_day_of_week(uint8_t i)
{
	char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

	return days[i-1];
}

void number_to_string(uint8_t num, char *buf)
{
	if(num < 10)
	{
		buf[0] = '0';
		buf[1] = num+48;
	}else if(num >= 10 && num < 99)
	{
		buf[0] = (num/10) + 48;
		buf[1] = (num % 10) + 48;
	}
}

// hh:mm:ss
char* time_to_string(RTC_time_t *rtc_time)
{
	static char buf[9];

	buf[2] = ':';
	buf[5] = ':';

	number_to_string(rtc_time->hours, &buf[0]);
	number_to_string(rtc_time->minutes, &buf[3]);
	number_to_string(rtc_time->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;
}

// dd/mm/yy
char* date_to_string(RTC_date_t *rtc_date)
{
	static char buf[9];

	buf[2] = '/';
	buf[5] = '/';

	number_to_string(rtc_date->date, &buf[0]);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);

	buf[8] = '\0';

	return buf;

}


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
	current_date.day = THURSDAY;
	current_date.date = 26;
	current_date.month = 12;
	current_date.year = 24;

	current_time.hours = 4;
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



