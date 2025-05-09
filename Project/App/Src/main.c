#include "config.h"
#include "common-defines.h" 

#include "driver_gpio.h"
#include "driver_systick.h"



int main(void)
{
    config_drivers();

    uint64_t start_time = systick_get();


    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_2, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);

    while (1)
    {
        if((systick_get() - start_time) >= 100)
        {
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
            GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_2);
            start_time = systick_get();
        }
    }
}
