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
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. configure the spi serial clock speed (baudrate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

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


/**************************************************************************
 * @fn				- SPI_GetFlagStatus
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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**************************************************************************
 * @fn				- SPI_SendData
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

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
	}
}





















