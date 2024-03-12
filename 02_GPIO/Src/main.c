#include "stm32f401xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 1000000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	//uint32_t *RCC_AHB1ENR = (uint32_t *)(0x40023800 + 0x30); // have to typecast that is a pointer
		//*RCC_AHB1ENR = 0x04; // 0x04 == 0b0100
	//*RCC_AHB1ENR = *RCC_AHB1ENR  | (1 << 2);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		//GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_10, 1);
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_10);
		delay();
	}
}

