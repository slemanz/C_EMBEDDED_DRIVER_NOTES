#include "driver_clock.h"

static uint32_t clock_value;

clock_status_e clock_init(void)
{
    clock_value = CLOCK_INTERNAL_RC_VALUE;
    return CLOCK_OK;

}

uint32_t clock_getValue(void)
{
    return clock_value;
}