#include "stm32f401xx.h"

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
		if(pGPIOx == GPIOA) GPIOA_PCLCK_EN();
		else if(pGPIOx == GPIOB) GPIOB_PCLCK_EN();
		else if(pGPIOx == GPIOC) GPIOC_PCLCK_EN();
		else if(pGPIOx == GPIOD) GPIOD_PCLCK_EN();
		else if(pGPIOx == GPIOE) GPIOE_PCLCK_EN();
		else if(pGPIOx == GPIOH) GPIOH_PCLCK_EN();
	}else
	{
		if(pGPIOx == GPIOA) GPIOA_PCLCK_DI();
		else if(pGPIOx == GPIOB) GPIOB_PCLCK_DI();
		else if(pGPIOx == GPIOC) GPIOC_PCLCK_DI();
		else if(pGPIOx == GPIOD) GPIOD_PCLCK_DI();
		else if(pGPIOx == GPIOE) GPIOE_PCLCK_DI();
		else if(pGPIOx == GPIOH) GPIOH_PCLCK_DI();
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




























