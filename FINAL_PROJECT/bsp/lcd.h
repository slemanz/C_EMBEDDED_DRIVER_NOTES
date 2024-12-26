/*
 * lcd.h
 *
 *  Created on: Dec 26, 2024
 *      Author: sleman
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f401xx.h"

/* bsp exposed apis */

/* Application configurable items */
#define LCD_GPIO_PORT  		GPIOA
#define LCD_GPIO_RS	   		GPIO_PIN_NO_6
#define LCD_GPIO_RW	   		GPIO_PIN_NO_7
#define LCD_GPIO_EN	   		GPIO_PIN_NO_8
#define LCD_GPIO_D4	   		GPIO_PIN_NO_9
#define LCD_GPIO_D5	   		GPIO_PIN_NO_10
#define LCD_GPIO_D6	   		GPIO_PIN_NO_11
#define LCD_GPIO_D7	   		GPIO_PIN_NO_12

void lcd_init(void);


#endif /* LCD_H_ */
