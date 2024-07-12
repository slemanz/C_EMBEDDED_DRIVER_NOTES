/*
 * Exercise: I2C master(STM) and I2C slavve (ESP) communication
 *
 * When button on the STM32 board (master) is pressed, master should read and display
 * data from ESP slave connected. First master has to get the length of the data from
 * from the slave to read subsequent data from the slave.
 *
 * 1. Use 12C SCL = 100kHz (Standard mode)
 * 2. Use external pull up resistors (3.3k) for SDA and SCL line
 *
 */

/*
 * ESP32:
 * SCL: A5 // GPIO22
 * SDA: A4 // GPIO21
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

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68



void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

// rcv buffer
uint8_t rcv_buf[32];

// some data
uint8_t some_data[] = "\n";

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
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);



}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}



void GPIO_Button_init(void)
{
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	//GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GpioButton);

}


int main(void)
{
	//initialise_monitor_handles();
	//printf("Hello World!\n");


	uint8_t commandcode;
	uint8_t len;

	GPIO_Button_init();

	// i2c pin inits
	I2C1_GPIOInits();

	// i2c peripheral configuration
	I2C1_Inits();








	while(1){

		// wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();
		//printf("Ok!\n");

		// enable the i2c peripheral
		I2C_PeripheralControl(I2C1, ENABLE);

		// ack bit is made 1 after PE=1
		//I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);


		commandcode = 0x51;

		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR);
		I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR);
		//I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR);

		//printf("%d\n", len);


		//commandcode = 0x52;
		//I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR);
		//I2C_MasterReceiveData(&I2C1Handle, rcv_buf, len, SLAVE_ADDR);
		//rcv_buf[len+1] = '\0';

		//printf("Data : %s",rcv_buf);

		// enable the i2c peripheral
		I2C_PeripheralControl(I2C1, DISABLE);

	}

	return 0;

}

