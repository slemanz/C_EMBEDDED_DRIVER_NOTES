#ifndef INC_DRIVER_INTERRUPT_H_
#define INC_DRIVER_INTERRUPT_H_

#include "stm32f401xx.h"

#define IRQ_NO_UART2            38

void interrupt_config(uint8_t IRQNumber, uint8_t EnorDi);
void interrupt_priorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

#endif /* INC_DRIVER_INTERRUPT_H_ */

