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
 * ARDUINO UNO PINS// ESP32:
 * SCL: A5
 * SDA: A4
 *
 * STM32 Board: (I2C1)
 *
 * I2C1 SDA: PB7
 * I2C1 SCLK: PB6
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

I2C_Handle_t I2C1Handle;

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&I2CPins);



}

void I2C1_Inits(void)
{
	I2C1Handle.pI2C = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;

}



void GPIO_Button_init(void)
{
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GpioButton);

}


int main(void)
{
	initialise_monitor_handles();

	printf("Running\n");

	while(1){




	}

	return 0;

}

