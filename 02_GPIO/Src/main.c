
#include <stdint.h>


#define RCC_BASE_ADDR			0x40023800UL
#define RCC_CFGR_REG_OFFSET		0x08 // clock configuration register
#define RCC_CFGR_REG_ADDR		(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)

#define GPIOA_BASE_ADDR            0x40020000UL

int main(void)
{

	for(;;);
}
