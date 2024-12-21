/*
 * usart.c
 *
 *  Created on: Dec 20, 2024
 *      Author: sleman
 */

#include "usart.h"

/*********************************************************************
 * @fn      		  - void USART_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              -

 */

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pUSARTx == USART1)
			{
				USART1_PCLK_EN();
			}else if(pUSARTx == USART2)
			{
				USART2_PCLK_EN();
			}else if(pUSARTx == USART6)
			{
				USART6_PCLK_EN();
			}
		}else
		{
			if(pUSARTx == USART1)
			{
				USART1_PCLK_DI();
			}else if(pUSARTx == USART2)
			{
				USART2_PCLK_DI();
			}else if(pUSARTx == USART6)
			{
				USART6_PCLK_DI();
			}
		}
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              -

 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// temporary variable
	uint32_t tempreg = 0;

	// *** Configuration of CR1 **************************************
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable USART Tx and Rx engines
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempreg |= (1 << USART_CR1_PCE);
	}else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		tempreg |= (1 << USART_CR1_PCE);
		tempreg |= (1 << USART_CR1_PS);
	}

	// program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	// *** Configuration of CR2 **************************************
	tempreg = 0;

	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	// program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	// *** Configuration of CR3 **************************************
	tempreg=0;

		//Configuration of USART hardware flow control
		if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
		{
			//Implement the code to enable CTS flow control
			tempreg |= ( 1 << USART_CR3_CTSE);


		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
		{
			//Implement the code to enable RTS flow control
			tempreg |= ( 1 << USART_CR3_RTSE);

		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
		{
			//Implement the code to enable both CTS and RTS Flow control
			tempreg |= ((1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE));
		}

		pUSARTHandle->pUSARTx->CR3 = tempreg;

		// *** Configuration of BRR (Baudrate) ***********************
}

/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              -

 */



uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{
    if(pUSARTx->SR & FlagName)
    {
    	return SET;
    }

   return RESET;
}


/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName);
}

/*********************************************************************
 * @fn      		  - USART_SendData
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
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;

   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		// wait until TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				// Increment pTxBuffer twice (16 bits)
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	//wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
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

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				// increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              -

 */

void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}
