#include <string.h>
#include "stm32f401xx.h"


void delay(void)
{
	for(uint32_t i = 0; i < 5000000; i++);
}

void debounce(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed1, GpioLed2, GpioButton;

	memset(&GpioLed1, 0, sizeof(GpioLed1)); // set each member element to 0
	memset(&GpioLed2, 0, sizeof(GpioLed2));
	memset(&GpioButton, 0, sizeof(GpioButton));


	GpioLed1.pGPIOx = GPIOC;
	GpioLed1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioLed2.pGPIOx = GPIOC;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;




	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed1);
	GPIO_Init(&GpioLed2);
	GPIO_Init(&GpioButton);
	GPIO_WriteToOutputPin(GPIOC,GPIO_PIN_NO_11,GPIO_PIN_RESET);

	// IRQ config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQITConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_12);
		delay();
	}
}

void EXTI15_10_IRQHandler(void)
{
	debounce(); // before clear trigger
	GPIO_IRQHandling(GPIO_PIN_NO_10);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_11);
}

