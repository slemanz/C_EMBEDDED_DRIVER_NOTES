#include "config.h"
#include "stm32f401xx.h"

static const GPIO_Config_t gpioConfigs[] = {
    {GPIOB, GPIO_PIN_NO_2, GPIO_MODE_OUT,   GPIO_SPEED_LOW,     GPIO_OP_TYPE_PP, GPIO_NO_PUPD, GPIO_NO_ALTFN},
    {GPIOA, GPIO_PIN_NO_5, GPIO_MODE_OUT,   GPIO_SPEED_LOW,     GPIO_OP_TYPE_PP, GPIO_NO_PUPD, GPIO_NO_ALTFN},
    {GPIOA, GPIO_PIN_NO_2, GPIO_MODE_ALTFN, GPIO_SPEED_FAST,    GPIO_OP_TYPE_PP, GPIO_NO_PUPD, PA2_ALTFN_UART2_TX},
    {GPIOA, GPIO_PIN_NO_3, GPIO_MODE_ALTFN, GPIO_SPEED_FAST,    GPIO_OP_TYPE_PP, GPIO_NO_PUPD, PA3_ALTFN_UART2_RX}
    // Add more configurations as needed
};

static const UART_Config_t UartConfigs[] = {
    {UART2,     UART_MODE_ONLY_TX,      USART_STD_BAUD_115200,  USART_STOPBITS_1,    UART_PARITY_DISABLE}
    // Add more configurations as needed
};

static void config_gpio(void)
{
    for (uint32_t i = 0; i < sizeof(gpioConfigs) / sizeof(gpioConfigs[0]); i++)
    {
        GPIO_Init((GPIO_Config_t *)&gpioConfigs[i]);
    }
}

static void config_uart(void)
{
    for (uint32_t i = 0; i < sizeof(UartConfigs) / sizeof(UartConfigs[0]); i++)
    {
        UART_Init((UART_Config_t *)&UartConfigs[0]);
    }
}


void config_drivers(void)
{
    while(clock_init());

    config_gpio();
    config_uart();
    systick_init(1000, clock_getValue());
}


// printf retarget
extern int __io_putchar(int ch)
{
    uint8_t ch_send = (uint8_t)ch;
    UART_Write(UART2, &ch_send, 1);
    return ch;
}