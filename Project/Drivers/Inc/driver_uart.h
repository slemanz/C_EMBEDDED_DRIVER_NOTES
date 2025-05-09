#ifndef DRIVER_UART_H_
#define DRIVER_UART_H_

#include "stm32f401xx.h"

// Define the callback function type
typedef void (*uart_callback_t)(uint8_t data);

/*
 * Configuration structure for USARTx peripheral
 */

typedef struct
{
	UART_RegDef_t *pUSARTx;
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_ParityControl;
}UART_Config_t;

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define UART_MODE_ONLY_TX		0
#define UART_MODE_ONLY_RX		1
#define UART_MODE_TXRX			2


/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_9600			9600
#define USART_STD_BAUD_19200		19200
#define USART_STD_BAUD_115200		115200


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define UART_PARITY_EN_ODD          2
#define UART_PARITY_EN_EVEN         1
#define UART_PARITY_DISABLE         0


/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1			0
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_2			2
#define USART_STOPBITS_1_5

/*
 *  Bit position definions Uart
 */

#define UART_CR1_UE                 13
#define UART_CR1_M                  12
#define UART_CR1_PCE                10
#define UART_CR1_PS                 9
#define UART_CR1_RXNEIE             5
#define UART_CR1_TE                 3
#define UART_CR1_RE                 2

#define UART_CR2_STOP               12

#define UART_SR_CTS                 9
#define UART_SR_TXE                 7
#define UART_SR_TC                  6
#define UART_SR_RXNE                5
#define UART_SR_ORE                 3




#define UART_CR1_UE_MASK             (1 << UART_CR1_UE)
#define UART_CR1_M_MASK              (1 << UART_CR1_M)
#define UART_CR1_PCE_MASK            (1 << UART_CR1_PCE)
#define UART_CR1_PS_MASK             (1 << UART_CR1_PS)
#define UART_CR1_RXNEIE_MASK         (1 << UART_CR1_RXNEIE)
#define UART_CR1_TE_MASK             (1 << UART_CR1_TE)
#define UART_CR1_RE_MASK             (1 << UART_CR1_RE)

#define UART_CR2_STOP_MASK           (1 << UART_CR2_STOP)

#define UART_SR_CTS_MASK             (1 << UART_SR_CTS)
#define UART_SR_TXE_MASK             (1 << UART_SR_TXE)
#define UART_SR_TC_MASK              (1 << UART_SR_TC)
#define UART_SR_RXNE_MASK            (1 << UART_SR_RXNE)
#define UART_SR_ORE_MASK             (1 << UART_SR_ORE)

/*
 *  UART related status flags definitions
 */


#define UART_FLAG_CTS	                (UART_SR_TXE_CTS_MASK)
#define UART_FLAG_TXE	                (UART_SR_TXE_MASK)
#define UART_FLAG_TC                 	(UART_SR_TC_MASK)
#define UART_FLAG_RXNE              	(UART_SR_RXNE_MASK)
#define UART_FLAG_ORE                 	(UART_SR_ORE_MASK)

/********************************************************************************************
 * 								APIs supported by this driver
 * 					for more information check the function definitions
 ********************************************************************************************/

/*
 * Peripheral Clock setup
 */
void UART_PeriClockControl(UART_RegDef_t *pUARTx, uint8_t EnorDi);


/*
 * Init and De-init
 */

void UART_Init(UART_Config_t *pUARTConfig);
void UART_DeInit(UART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */

void UART_write_byte(UART_RegDef_t *pUARTx, uint8_t data);
uint8_t UART_read_byte(UART_RegDef_t *pUARTx);

void UART_Write(UART_RegDef_t *pUARTx, uint8_t *pTxBuffer, uint32_t Len);
void UART_Read(UART_RegDef_t *pUARTx, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

void UART_InterruptConfig(uint8_t interrupt, uint8_t EnorDi);
void UART_CallbackRegister(UART_RegDef_t *pUARTx, uart_callback_t rx_callback, uart_callback_t tx_callback);


/*
 * Other Peripheral Control APIs
 */
void UART_PeripheralControl(UART_RegDef_t *pUARTx, uint8_t EnorDi);
uint8_t UART_GetFlagStatus(UART_RegDef_t *pUARTx , uint8_t FlagName);
void UART_ClearFlag(UART_RegDef_t *pUARTx, uint16_t StatusFlagName);


// REFACT
void uart2_interrupt_enable(void);

int uart2_write(int ch);
int uart2_read(void);


#endif