#include "stm32f401xx.h"
#include "string.h"

// test the SPI_sendData API, to send the string
// "hello world"
// SPI Master Mode
// SCLK = 2Mhz
// DFF = 0 and DFF = 1

/*
 *  PINs:
 *  PB12 -> SPI2_NSS
 *  PB13 -> SPI2_SCK
 *  PB14 -> SPI2 MISO
 *  PB15 -> SPI2 MOSI
 *  Alt function: 5
 */

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

void SPI2_GPIOInits(void)
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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);


}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // 8Mhz serial clock
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;	// will use hardware slave management

	SPI_Init(&SPI2handle);
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
	GPIO_Button_init();
	// user button


	char user_data[] = "Hello world";


	// this funtciton is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//
	SPI_SSOEConfig(SPI2, ENABLE);




	while(1)
	{
		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		// to avoid button bouncing
		delay();
		// enable the SPI2 peripheral
		SPI_PeipheralControl(SPI2, ENABLE);

		// first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// to send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeipheralControl(SPI2, DISABLE);

	}
}
