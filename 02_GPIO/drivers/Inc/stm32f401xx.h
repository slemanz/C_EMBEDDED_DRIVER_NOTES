/*
 * stm32f401xx.h
 *
 *      Author: sleman
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

// this header file describes the microcontroller

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define ROM_BASEADDR						0x1FFF 0000 	/* system memory */
#define SRAM1 								SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE							0x40000000U
#define APB1PERIPH_BASE						PERIPH_BASE
#define APB2PERIPH_BASE						0x40010000U
#define AHB1PERIPH_BASE						0x40020000U
#define AHB2PERIPH_BASE						0x50000000U


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASE + 0x1000U)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASE + 0x1C00U)


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR						(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR						(APB1PERIPH_BASE + 0x5C00U)

#define SPI2_BASEADDR						(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR						(APB1PERIPH_BASE + 0x3C00U)

#define USART2_BASEADDR						(APB1PERIPH_BASE + 0x4400U)


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR						(APB2PERIPH_BASE + 0x3C00U)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASE + 0x3800U)

#define SPI1_BASEADDR						(APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASEADDR						(APB2PERIPH_BASE + 0x3400U)

#define USART1_BASEADDR						(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR						(APB2PERIPH_BASE + 0x1400U)
























#endif /* INC_STM32F401XX_H_ */
