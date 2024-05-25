#include "stm32f401xx.h"

// test the SPI_sendData API, to send the string
// "hello world"
// SPI Master Mode
// SCLK = max possible
// DFF = 0 and DFF = 1

/*
 *  PINs:
 *  PB12 -> SPI2_NSS
 *  PB13 -> SPI2_SCK
 *  PB14 -> SPI2 MISO
 *  PB15 -> SPI2 MOSI
 *  Alt function: 5
 */

void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);


}


int main(void)
{
	// this funtciton is used to initialize the GPIO pins to behave as SPI2 pins
	SPI_GPIOInits();


	while(1)
	{

	}
}
