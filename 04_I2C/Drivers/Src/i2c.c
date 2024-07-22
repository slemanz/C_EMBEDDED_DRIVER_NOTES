#include "i2c.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[8] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);





static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // SlaveAddr is Slave address + r/nw bit = 0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; // SlaveAddr is Slave address + r/nw bit = 1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;

}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - Enables or disables the specified I2C peripheral.
 *
 * @param[in]         - pI2Cx: Pointer to the I2C peripheral register definition structure
 * @param[in]         - EnorDi: Enable or disable the I2C peripheral
 *
 * @return            - None
 *
 * @Note              - This function controls the Peripheral Enable (PE) bit of the I2C_CR1 register.

 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}




/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - Enables or disables the peripheral clock for the specified I2C port.
 *
 * @param[in]         - pI2Cx: Pointer to the I2C peripheral register definition structure
 * @param[in]         - EnorDi: Enable or disable the peripheral clock
 *
 * @return            - None
 *
 * @Note              - None

 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}



uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp;
	uint16_t ahbp, apb1p;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		// dont have external oscilator in my board
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}


	// ahb
	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}


	// apb1
	temp = (RCC->CFGR >> 10) & 0x7;

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk/ahbp)/apb1p;

	return pclk1;
}


/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - Initializes the specified I2C peripheral with the given configuration settings.
 *
 * @param[in]         - pI2CHandle: Pointer to the I2C handle structure containing the I2C configuration settings
 *
 * @return            - None
 *
 * @Note              - his function configures the I2C peripheral including clock enabling, addressing mode, and speed settings.

 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// enable the clock for the i2c peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);


	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address;
	tempreg = 0;
	tempreg = (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14); // always be 1 - reference manual
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = (RCC_GetPCLK1Value()/(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= ccr_value & 0xFFF;
	}else
	{
		// mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle <<14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value()/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value()/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}else
	{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300)/ 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);


}





/**
 * @fn         - I2C_DeInit
 *
 * @brief      - Resets the specified I2C peripheral and its registers to their default values.
 *
 * @param[in]  - pI2Cx: Pointer to the I2C peripheral register definition structure
 *
 * @return     - None
 *
 * @Note       - This function resets the specified I2C peripheral using the appropriate reset macro.
 */


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if		(pI2Cx == I2C1) I2C1_REG_RESET();
	else if	(pI2Cx == I2C2) I2C2_REG_RESET();
	else if	(pI2Cx == I2C3) I2C3_REG_RESET();
}

/**
 * @fn         - I2C_ManageAcking
 *
 * @brief      -
 *
 * @param[in]  -
 * @param[in]  -
 *
 * @return     -
 *
 * @Note       -
 */


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		// disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/**
 * @fn         - I2C_GetFlagStatus
 *
 * @brief      - Checks the status of the specified I2C flag.
 *
 * @param[in]  - pI2Cx: Pointer to the I2C peripheral register definition structure
 * @param[in]  - FlagName: Name of the flag whose status needs to be checked
 *
 * @return     - uint8_t: FLAG_SET if the flag is set, FLAG_RESET otherwise
 *
 * @Note       - This function reads the status register SR1 to check the specified flag.
 */


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	// 2. confirm that start generation  is completed by checking the SB flag in the SR1
	// NOTE: until SB is cleared SCL will be stretched (pulled LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. clear the ADDR flag according to its software sequence
	// Note: until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. send the data until Len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// 7. when Len becomes zero wait for TXE=1 and BTF=1 before senerating the stop condition
	// Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));



	// 8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);




}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generetion is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//procedure to read only 1 byte from slava
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}


	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			if(i == 2) // if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}
			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}
	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}


}


/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */


void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}























