#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

typedef enum {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTERNATE,
    GPIO_MODE_ANALOG
} GPIO_Mode;

typedef enum {
    GPIO_NOPULL,
    GPIO_PULLUP,
    GPIO_PULLDOWN
} GPIO_Pull;

typedef struct {
    uint32_t MODER;     // GPIO port mode register
    uint32_t OTYPER;    // GPIO port output type register
    uint32_t OSPEEDR;   // GPIO port output speed register
    uint32_t PUPDR;     // GPIO port pull-up/pull-down register
    uint32_t IDR;       // GPIO port input data register
    uint32_t ODR;       // GPIO port output data register
    uint32_t BSRR;      // GPIO port bit set/reset register
    uint32_t LCKR;      // GPIO port configuration lock register
    uint32_t AFR[2];    // GPIO alternate function registers
} GPIO_TypeDef;

// Function pointer types for dependency injection
typedef void (*GPIO_Init_Func)(GPIO_TypeDef*, uint16_t, GPIO_Mode, GPIO_Pull);
typedef void (*GPIO_Write_Func)(GPIO_TypeDef*, uint16_t, GPIO_PinState);
typedef GPIO_PinState (*GPIO_Read_Func)(GPIO_TypeDef*, uint16_t);

// Function prototypes
void GPIO_Init(GPIO_TypeDef *GPIOx, uint16_t pin, GPIO_Mode mode, GPIO_Pull pull);
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t pin, GPIO_PinState state);
GPIO_PinState GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif // GPIO_H