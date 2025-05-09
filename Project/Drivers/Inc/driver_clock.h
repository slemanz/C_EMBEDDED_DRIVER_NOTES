#ifndef INC_DRIVER_CLOCK_H_
#define INC_DRIVER_CLOCK_H_

#include "stm32f401xx.h"

#define CLOCK_INTERNAL_RC_VALUE 16000000

typedef enum
{
    CLOCK_OK,
    CLOCK_NOT_OK
}clock_status_e;

clock_status_e clock_init(void);

uint32_t clock_getValue(void);

#endif /*INC_DRIVER_CLOCK_H_ */