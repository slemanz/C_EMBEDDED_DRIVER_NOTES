/*
 * i2c.h
 *
 *  Created on: Jun 6, 2024
 *      Author: sleman
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f401xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint32_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2C;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

 /*
  * @I2C_SCLSpeed
  */

#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

/*
 *	@I2C_ACKControl
 */

#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 *	@I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


/********************************************************************************************
 * 								APIs supported by this driver
 * 					for more information check the function definitions
 ********************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * Init and De-init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */

/*
 * IRQ Configuration and ISR Handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);



/*
 * Other peripheral control API
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Application Callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
uint32_t RCC_GetPCLK1Value(void);





#endif /* INC_I2C_H_ */
