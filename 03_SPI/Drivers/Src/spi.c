#include "stm32f401xx.h"

/**************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}else
	{
		// to do
	}
}


/**************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Configure the SPI_CR1 register

	uint32_t tempreg = 0;

	// 1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_BusConfig << 2;

	// 2. configure the bus config
	if(pSPIHandle->SPIConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << 15);
	}else if(pSPIHandle->SPIConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << 15);
	}else if(pSPIHandle->SPIConfig == SPI_BUS_SIMPLEX_RXONLY)
	{
		// bidi mode should be cleared
		tempreg &= ~(1 << 15);
		// RXONLY bit should be set
		tempreg |= (1 << 10);
	}

	// 3. configure the spi serial clock speed (baudrate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	pSPIHandle->pSPIx->CR1 = tempreg;

}




/**************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if		(pSPIx == SPI1) SPI1_REG_RESET();
	else if	(pSPIx == SPI2) SPI2_REG_RESET();
	else if	(pSPIx == SPI3) SPI3_REG_RESET();
	else if	(pSPIx == SPI4) SPI4_REG_RESET();
}
























