#include "stm32f401xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}


int main(void)
{
	pinout_init();



	while(1)
	{
		LED1 = 0;
		LED2 = 1;
		delay();
		LED1 = 1;
		LED2 = 0;
		delay();
	}
}
