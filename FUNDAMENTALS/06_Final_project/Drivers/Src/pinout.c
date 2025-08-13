#include "stm32f401xx.h"


void pinout_init(void)
{
	/*
	 *	GPIO Configs
	 */

	GPIO_Handle_t GpioLed1;
	GpioLed1.pGPIOx = PORT_LED1;
	GpioLed1.GPIO_PinConfig.GPIO_PinNumber = PIN_LED1;
	GpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t GpioLed2;
	GpioLed2.pGPIOx = PORT_LED2;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber = PIN_LED2;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/*
	 *  Init peripherals pins
	 */

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed1);
	GPIO_Init(&GpioLed2);




	/*
	 *  Initial state of gpio's
	 */

	GPIO_WriteToOutputPin(PORT_LED1, PIN_LED1, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(PORT_LED2, PIN_LED2, GPIO_PIN_RESET);
}
