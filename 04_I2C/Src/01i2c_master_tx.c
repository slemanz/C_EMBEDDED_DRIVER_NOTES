/*
 * Exercise:
 *
 * When button on the STM32 board (master) is pressed, master should send data to the
 * Arduino board (slave). The data received by the arduino board will be displayed on
 * the serial monitor terminal of the Arduino IDE.
 *
 * 1. Use 12C SCL = 100kHz (Standard mode)
 * 2. Use external pull up resistors (3.3k) for SDA and SCL line
 *
 */

/*
 * ARDUINO UNO PINS:
 * SCL: A5
 * SDA: A4
 *
 * STM32 Board: (I2C1)
 *
 */


#include<stdio.h>
#include<string.h>
#include "stm32f401xx.h"

extern void initialise_monitor_handles(void);


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{
	initialise_monitor_handles();

	printf("Running\n");

	while(1){




	}

	return 0;

}

