#include "stm32f401xx.h"
#include <string.h>
#include <stdio.h>

// test the SPI_sendData API, to send the string
// "hello world"
// SPI Master Mode
// SCLK = 2Mhz
// DFF = 0 and DFF = 1

// semihosting
// linker flag: -specs=rdimon.specs -lc -lrdimon
// startup debug: monitor arm semihosting enable

extern void initialise_monitor_handles(void);


/*
 *  PINs:
 *  PB12 -> SPI2_NSS
 *  PB13 -> SPI2_SCK
 *  PB14 -> SPI2 MISO
 *  PB15 -> SPI2 MOSI
 *  Alt function: 5
 */


// command codes
#define COMMAND_LED_CTRL         	 0x50
#define COMMAND_SENSOR_READ     	 0x51
#define COMMAND_LED_READ         	 0x52
#define COMMAND_PRINT          		 0x53
#define COMMAND_ID_READ       		 0x54

#define LED_ON							1
#define LED_OFF	 						0

// arduino analog pins
#define ANALOG_PIN0						0
#define ANALOG_PIN1						1
#define ANALOG_PIN2						2
#define ANALOG_PIN3						3
#define ANALOG_PIN4						4

// arduino led
#define LED_PIN 						9


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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; // 2Mhz serial clock
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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		// ack
		return 1;
	}
	return 0;
}


int main(void)
{
	initialise_monitor_handles();

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	uint8_t ackbyte;
	uint8_t led_state = 0;

	printf("Application is running\n");

	// user button
	GPIO_Button_init();


	// this funtciton is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//
	SPI_SSOEConfig(SPI2, ENABLE);




	while(1)
	{


		// wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));




		// to avoid button bouncing
		delay();
		// enable the SPI2 peripheral
		SPI_PeipheralControl(SPI2, ENABLE);

		// 1. CMD_LED_CTRL <pin no(1)>		<value (1)>
		printf("Command: 1\n");

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t args[2];

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);



		// send some dummy bits (1byte), to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1 );

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte) & !led_state)
		{
			led_state = 1;
			printf("Led on\n");

			// send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);

		}else{
			led_state = 0;
			printf("Led off\n");

			// send arguments
			args[0] = LED_PIN;
			args[1] = LED_OFF;
			SPI_SendData(SPI2, args, 2);
		}
		printf("\n");

		//2. CMD_SENSOR_READ	<analog pin number(1)>


		// wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();
		printf("Command: 2\n");

		commandcode = COMMAND_SENSOR_READ;

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);



		// send some dummy bits (1byte), to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1 );

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			// send arguments
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// insert some delay, for the adc conversion
			delay();


			// send some dummy bits (1byte), to fetch the response from the slave.
			SPI_SendData(SPI2, &dummy_write, 1 );

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("Analog read: %d\n", analog_read);

		}
		printf("\n");


		//3. COMMAND_LED_READ 	<led pin number(1)>


		// wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();
		printf("Command: 3\n");

		commandcode = COMMAND_LED_READ;

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);



		// send some dummy bits (1byte), to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1 );

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			// send arguments
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with the data
			delay();


			// send some dummy bits (1byte), to fetch the response from the slave.
			SPI_SendData(SPI2, &dummy_write, 1 );

			uint8_t pin_read;
			SPI_ReceiveData(SPI2, &pin_read, 1);
			printf("Pin read: %d\n", pin_read);

		}
		printf("\n");



		// 4. CMD PRINT 	<len(2) message(len)>


		// wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();
		printf("Command: 4\n");

		commandcode = COMMAND_PRINT;

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);



		// send some dummy bits (1byte), to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1 );

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t msg[] = "Hello! Who are you?";
		if(SPI_VerifyResponse(ackbyte))
		{
			// send arguments
			args[0] = strlen((char*)msg);
			SPI_SendData(SPI2, args, 1);

			delay();

			for(uint8_t i = 0; i < args[0]; i++)
			{
				SPI_SendData(SPI2, &msg[i], 1);
				SPI_ReceiveData(SPI2, &dummy_read, 1);
			}
			printf("COMMAND_PRINT Executed \n");
		}
		printf("\n");

		//5. CMD_ID_READ

		// wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();
		printf("Command: 5\n");

		commandcode = COMMAND_PRINT;

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);



		// send some dummy bits (1byte), to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1 );

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}
		printf("\n");















		// lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeipheralControl(SPI2, DISABLE);


	}
}

