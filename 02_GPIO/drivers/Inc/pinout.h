#ifndef INC_PINOUT_H_
#define INC_PINOUT_H_

#include "stm32f401xx.h"

/*
 *  PIN DEFINITIONS
 *
 *	PB2 -> LED1 (EXAMPLE)
 */


/* Port abstract by bitfield */
typedef struct{
	__vo uint32_t pin0  		: 1;
	__vo uint32_t pin1  		: 1;
	__vo uint32_t pin2  		: 1;
	__vo uint32_t pin3  		: 1;
	__vo uint32_t pin4  		: 1;
	__vo uint32_t pin5  		: 1;
	__vo uint32_t pin6  		: 1;
	__vo uint32_t pin7  		: 1;
	__vo uint32_t pin8  		: 1;
	__vo uint32_t pin9 	 		: 1;
	__vo uint32_t pin10  		: 1;
	__vo uint32_t pin11  		: 1;
	__vo uint32_t pin12  		: 1;
	__vo uint32_t pin13  		: 1;
	__vo uint32_t pin14  		: 1;
	__vo uint32_t pin15  		: 1;
	__vo uint32_t reserved  	: 16;
}PORTx_pin_t;



/*
 * Ports definitions
 */

#define PORT_LED1	GPIOA



/*
 * Pins definitions
 */

#define PIN_LED1	GPIO_PIN_NO_2




/*
 *  Pins interface
 */

#define PORTA_OUT ((PORTx_pin_t*)&GPIOA->ODR)
#define PORTB_OUT ((PORTx_pin_t*)&GPIOB->ODR)
#define PORTC_OUT ((PORTx_pin_t*)&GPIOC->ODR)

#define PORTA_IN ((PORTx_pin_t*)&GPIOA->IDR)



/*
 *  Pins abstraction
 */

#define LED1 (PORTC_OUT->pin12)



#endif /* INC_PINOUT_H_ */
