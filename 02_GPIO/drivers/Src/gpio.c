#include "stm32f401xx.h"
#include "gpio.h"

/*
 *  Peripheral clock setup
 */

/**************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- this functions enable and disables peripheral clock
 * 					  for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 */


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if		(pGPIOx == GPIOA) GPIOA_PCLCK_EN();
		else if	(pGPIOx == GPIOB) GPIOB_PCLCK_EN();
		else if	(pGPIOx == GPIOC) GPIOC_PCLCK_EN();
		else if	(pGPIOx == GPIOD) GPIOD_PCLCK_EN();
		else if	(pGPIOx == GPIOE) GPIOE_PCLCK_EN();
		else if	(pGPIOx == GPIOH) GPIOH_PCLCK_EN();
	}else
	{
		if		(pGPIOx == GPIOA) GPIOA_PCLCK_DI();
		else if (pGPIOx == GPIOB) GPIOB_PCLCK_DI();
		else if (pGPIOx == GPIOC) GPIOC_PCLCK_DI();
		else if (pGPIOx == GPIOD) GPIOD_PCLCK_DI();
		else if (pGPIOx == GPIOE) GPIOE_PCLCK_DI();
		else if (pGPIOx == GPIOH) GPIOH_PCLCK_DI();
	}
}



/*
 * Init and De-init
 */

/**************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temporary register

	// 1. configure the mode of the gpio pin
	
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER = temp;
	}else
	{
		//interrupt mode
	}

	temp = 0;

	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR = temp;
	temp = 0;


	// 3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR = temp;
	temp = 0;


	// 4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER = temp;
	temp = 0;


	// 5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		
	}
}


/**************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{


}




























