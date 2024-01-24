/*
 * stm32f401xx.h
 *
 *      Author: sleman
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

// this header file describes the microcontroller
#include <stdint.h>
#define __vo volatile

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

#define RCC_BASEADDR						(AHB1PERIPH_BASE + 0x3800U)


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



/*******************peripheral register definition structures*******************/

typedef struct
{
	__vo uint32_t MODER; /* configure the mode of pin 			address offset: 0x00 */
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t reserved1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t reserved2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t reserved3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t reserved4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t reserved5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * 	peripheral definitions (Peripheral base address typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC				((GPIO_RegDef_t*)RCC_BASEADDR)


/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))


/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))


/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))


/*
 * Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))


/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))


/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock disable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))


/*
 * some generic macros
 */

#define ENABLE 						1
#define	DISABLE 					0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET















#endif /* INC_STM32F401XX_H_ */