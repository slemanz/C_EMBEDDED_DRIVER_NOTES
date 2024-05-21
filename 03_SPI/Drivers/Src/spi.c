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



























