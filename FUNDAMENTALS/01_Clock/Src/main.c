#include <stdint.h>

#define RCC_BASE_ADDR			0x40023800UL
#define RCC_CFGR_REG_OFFSET		0x08 // clock configuration register
#define RCC_CFGR_REG_ADDR		(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)

#define GPIOA_BASE_ADDR            0x40020000UL

int main(void)
{
	uint32_t *pRccCfgrReg = (uint32_t*)RCC_CFGR_REG_ADDR;

	//Clock-out capability example:
	//1. Configure the RCC_CFGR MCO1 bit fields to select HSI as clock source
	*pRccCfgrReg &= ~(0x3 << 21); //clear 21 and 22 bit positions
	// could be:
	// 00: HSI clock selected
	// 01: LSE oscillator selected
	// 10: HSE oscillator clock selected
	// 11: PLL clock selected

	//Configure MCO1 prescaler to 4 just as example ->
	*pRccCfgrReg |= ( 1 << 25);
	*pRccCfgrReg |= ( 1 << 26);

	//2. Configure PA8 to AF0 mode to behave as MCO1 signal
	// but I wont do that, is just to know that it can be done

    /* Loop forever */
	for(;;);
}
