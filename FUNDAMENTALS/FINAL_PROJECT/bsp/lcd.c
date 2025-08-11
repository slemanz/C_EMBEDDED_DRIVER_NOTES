/*
 * lcd.c
 *
 *  Created on: Dec 26, 2024
 *      Author: sleman
 */

#include "lcd.h"


static void write_4_bits(uint8_t value);
static void lcd_enable(void);
static void udelay(uint32_t cnt);



void lcd_send_command(uint8_t cmd)
{
	/* RS = 0 for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* R/nW = 0 for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);


	write_4_bits(cmd >> 4);
	write_4_bits(cmd & 0x0F);

}

void lcd_send_char(uint8_t data)
{
	/* RS = 1 for LCD user data */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	/* R/nW = 0 for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(data >> 4);   /* Higher nibble */
	write_4_bits(data & 0x0F); /* Lower nibble */
}

void lcd_display_clear(void)
{
	lcd_send_command(LCD_CMD_DIS_CLEAR);
	mdelay(2);
}

void lcd_print_string(char *message)
{
	do
	{
		lcd_send_char((uint8_t)*message++);
	}while(*message != '\0');
}

void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	mdelay(2);
}

void lcd_set_cursor(uint8_t row, uint8_t column)
{
	column--;
	switch (row)
	{
		case 1:
			lcd_send_command((column |= 0x80));
			break;
		case 2:
			lcd_send_command((column |= 0xC0));
			break;
		default:
			break;
	}
}

void lcd_init(void)
{
	// 1. configure the gpio pins which are used for lcd connections

	GPIO_Handle_t lcd_signal;

	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	// 2. Do the lcd initialization

	mdelay(40);

	/* RS = 0, for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* RnW = 0, writing to LCD */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(0x3);

	mdelay(5);

	write_4_bits(0x3);

	udelay(150);

	write_4_bits(0x3);
	write_4_bits(0x2);

	// function set command
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	// display ON and cursor ON
	lcd_send_command(LCD_CMD_DON_CURON);

	// display clear
	lcd_display_clear();

	// entry mode set
	lcd_send_command(LCD_CMD_INCAD);

}


/* write 4 bits of data/command on to D4, D5, D6, D7 lines */
static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1) );

	lcd_enable();
}

static void lcd_enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100);
}

void mdelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < (cnt * 1000); i++)
	{
		__asm("NOP");
	}
}

static void udelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < (cnt * 1); i++)
	{
		__asm("NOP");
	}
}
