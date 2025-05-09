#include "driver_systick.h"

void systick_init(uint32_t freq, uint32_t cpu_freq)
{
    systick_set_frequency(freq, cpu_freq);
    systick_counter_enable();
    systick_interrupt_enable();
}

void systick_set_frequency(uint32_t freq, uint32_t cpu_freq)
{
    // reload with number of clocks persecond
    SYSTICK->LOAD   = (cpu_freq/freq) - 1;

    // clear systick current value register
    SYSTICK->VAL = 0;
}

void systick_counter_enable(void)
{
    // Enable systick and select internal clk src
    SYSTICK->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
}

void systick_interrupt_enable(void)
{
    // enable systick interrupt
    SYSTICK->CTRL |= CTRL_TICKINT;
}


static uint64_t ticks = 0;

uint64_t systick_get()
{
    return ticks;
}

void SysTick_Handler(void)
{
    ticks++;
}