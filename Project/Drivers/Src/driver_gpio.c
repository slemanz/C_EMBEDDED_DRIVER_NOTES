#include "driver_gpio.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if		(pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if	(pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if	(pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if	(pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if	(pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if	(pGPIOx == GPIOH) GPIOH_PCLK_EN();
	}else
	{
		if		(pGPIOx == GPIOA) GPIOA_PCLK_DI();
		else if	(pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if	(pGPIOx == GPIOC) GPIOC_PCLK_DI();
		else if	(pGPIOx == GPIOD) GPIOD_PCLK_DI();
		else if	(pGPIOx == GPIOE) GPIOE_PCLK_DI();
		else if	(pGPIOx == GPIOH) GPIOH_PCLK_DI();
	}
}

void GPIO_Init(GPIO_Config_t *pGPIOConfig)
{
    // enable clock to the port
    GPIO_PeriClockControl(pGPIOConfig->pGPIOx, ENABLE);

	uint32_t temp = 0; // temporary register

	// 1. configure the mode of the gpio pin
	
	if(pGPIOConfig->GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOConfig->GPIO_PinMode << (2 * pGPIOConfig->GPIO_PinNumber));
		pGPIOConfig->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOConfig->GPIO_PinNumber)); //clearing
		pGPIOConfig->pGPIOx->MODER |= temp; // setting
	}else
	{
		//1. configure interrupt mode
	}

	temp = 0;

	// 2. configure the speed
	temp = (pGPIOConfig->GPIO_PinSpeed << (2 * pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->OSPEEDR |= temp;
	temp = 0;


	// 3. configure the pupd settings
	temp = (pGPIOConfig->GPIO_PinPuPdControl << (2 * pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->PUPDR |= temp;
	temp = 0;


	// 4. configure the output type
	temp = (pGPIOConfig->GPIO_PinOPType << (pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->OTYPER &= ~(0x1 << (pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->OTYPER |= temp;
	temp = 0;


	// 5. configure the alt functionality
	if(pGPIOConfig->GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;

		temp1 = pGPIOConfig->GPIO_PinNumber / 8;
		temp2 = pGPIOConfig->GPIO_PinNumber % 8;
		pGPIOConfig->pGPIOx->AFR[temp1] &= ~(0x0F << (4*temp2));
		pGPIOConfig->pGPIOx->AFR[temp1] |= (pGPIOConfig->GPIO_PinAltFunMode << (4*temp2));
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if		(pGPIOx == GPIOA) GPIOA_REG_RESET();
	else if	(pGPIOx == GPIOB) GPIOB_REG_RESET();
	else if	(pGPIOx == GPIOC) GPIOC_REG_RESET();
	else if	(pGPIOx == GPIOD) GPIOD_REG_RESET();
	else if	(pGPIOx == GPIOE) GPIOE_REG_RESET();
	else if	(pGPIOx == GPIOH) GPIOH_REG_RESET();
}

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);

	return value;
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bitfield corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}