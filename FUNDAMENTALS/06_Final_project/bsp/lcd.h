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

/* LCD commands */
#define LCD_CMD_4DL_2N_5X8F			0x28
#define LCD_CMD_DON_CURON			0x0E
#define LCD_CMD_INCAD				0x06
#define LCD_CMD_DIS_CLEAR			0x01
#define LCD_CMD_DIS_RETURN_HOME		0x02



void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_send_char(uint8_t data);
void lcd_display_clear(void);
void lcd_print_string(char *message);
void lcd_display_return_home(void);
void lcd_set_cursor(uint8_t row, uint8_t column);

#endif /* LCD_H_ */
