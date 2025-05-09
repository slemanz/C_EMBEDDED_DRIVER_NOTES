#ifndef DRIVER_SYSTICK_H_
#define DRIVER_SYSTICK_H_

#include "stm32f401xx.h"

#define CTRL_ENABLE                 (1U << 0)
#define CTRL_CLKSRC                 (1U << 2)
#define CTRL_COUNTFLAG              (1U << 16)
#define CTRL_TICKINT                (1U << 1)

void systick_init(uint32_t freq, uint32_t cpu_freq);
void systick_set_frequency(uint32_t freq, uint32_t cpu_freq);
void systick_counter_enable(void);
void systick_interrupt_enable(void);
uint64_t systick_get();

#endif