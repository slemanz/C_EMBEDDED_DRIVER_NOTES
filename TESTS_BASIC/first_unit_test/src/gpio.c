#include "gpio.h"

void GPIO_Init(GPIO_TypeDef *GPIOx, uint16_t pin, GPIO_Mode mode, GPIO_Pull pull) {
    // Configure mode
    GPIOx->MODER &= ~(0x3 << (2 * pin));
    GPIOx->MODER |= (mode << (2 * pin));
    
    // Configure pull-up/pull-down
    GPIOx->PUPDR &= ~(0x3 << (2 * pin));
    GPIOx->PUPDR |= (pull << (2 * pin));
}

void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t pin, GPIO_PinState state) {
    if (state == GPIO_PIN_SET) {
        GPIOx->BSRR = (1 << pin);
    } else {
        GPIOx->BSRR = (1 << (pin + 16));
    }
}

GPIO_PinState GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t pin) {
    return (GPIO_PinState)((GPIOx->IDR >> pin) & 0x1);
}