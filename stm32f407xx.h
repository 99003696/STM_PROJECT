//MCU specific Header File
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stdint.h>

// Defining Memory Base Addresses

#define SRAM1_BASEADDR  0x20000000U
#define SRAM2_BASEADDR  0x2001C000U
#define FLASH_BASEADDR  0x08000000U
#define ROM_BASEADDR    0x1FFF0000U

//Defining Peripheral Base Addresses

#define APB1_BASEADDR  0x40000000U
#define APB2_BASEADDR  0x40010000U
#define AHB1_BASEADDR  0x40020000U

//Defining peripherals Base Address which is hanging on to AHB1 Bus

#define GPIOA_BASEADDR (AHB1_BASEADDR + 0x0000U)//0x40020000
#define GPIOB_BASEADDR (AHB1_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR (AHB1_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR (AHB1_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR (AHB1_BASEADDR + 0x1000U)
#define RCC_BASEADDR  (AHB1_BASEADDR + 0x3800UL)


//Defining peripherals Base Address which is hanging on to APB1 Bus

#define TIM2_BASEADDR  (APB1_BASEADDR + 0x00U)
#define TIM3_BASEADDR  (APB1_BASEADDR + 0x04U)
#define TIM4_BASEADDR  (APB1_BASEADDR + 0x08U)
#define TIM5_BASEADDR  (APB1_BASEADDR + 0x0CU)
#define TIM6_BASEADDR  (APB1_BASEADDR + 0x10U)
#define TIM7_BASEADDR  (APB1_BASEADDR + 0x14U)
#define TIM12_BASEADDR (APB1_BASEADDR + 0x18U)
#define TIM13_BASEADDR (APB1_BASEADDR + 0x1CU)
#define TIM14_BASEADDR (APB1_BASEADDR + 0x20U)
#define SPI2_BASEADDR  (APB1_BASEADDR + 0x38U)
#define SPI3_BASEADDR  (APB1_BASEADDR + 0x3CU)
#define USART2_BASEADDR (APB1_BASEADDR+ 0x44U)
#define USART3_BASEADDR (APB1_BASEADDR+ 0x48U)
#define UART4_BASEADDR (APB1_BASEADDR+ 0x4CU)
#define UART5_BASEADDR (APB1_BASEADDR+ 0x50U)
#define I2C1_BASEADDR (APB1_BASEADDR+ 0x54U)
#define I2C2_BASEADDR (APB1_BASEADDR+ 0x58U)
#define I2C3_BASEADDR (APB1_BASEADDR+ 0x5CU)
#define UART7_BASEADDR (APB1_BASEADDR+ 0x78U)
#define UART8_BASEADDR (APB1_BASEADDR+ 0x7CU)

//Defining peripherals Base Address which is hanging on to APB2 Bus

#define TIM1_BASEADDR  (APB2_BASEADDR + 0x00U)
#define TIM8_BASEADDR  (APB2_BASEADDR + 0x04U)
#define TIM9_BASEADDR  (APB2_BASEADDR + 0x40U)
#define TIM10_BASEADDR  (APB2_BASEADDR + 0x44U)
#define TIM11_BASEADDR  (APB2_BASEADDR + 0x48U)
#define USART1_BASEADDR (APB2_BASEADDR+ 0x10U)
#define USART6_BASEADDR (APB2_BASEADDR+ 0x14U)
#define SPI1_BASEADDR  (APB2_BASEADDR + 0x30U)
#define SPI4_BASEADDR  (APB2_BASEADDR + 0x34U)
#define SPI5_BASEADDR  (APB2_BASEADDR + 0x50U)
#define SPI6_BASEADDR  (APB2_BASEADDR + 0x54U)

#define __vo volatile

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSSR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}Gpio_RegDef_t;

#define GPIOA ((Gpio_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((Gpio_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((Gpio_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((Gpio_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((Gpio_RegDef_t*)GPIOE_BASEADDR)

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	 uint32_t Reserved_0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	 uint32_t Reserved_1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	 uint32_t Reserved_2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	 uint32_t Reserved_3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	 uint32_t Reserved_4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	 uint32_t Reserved_5[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
} Rcc_RegDEF_t;

#define RCC ((Rcc_RegDEF_t*)RCC_BASEADDR)

//Peripheral Clock Enable

#define GPIOA_PCLK_EN() (RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC -> AHB1ENR |= (1 << 4))

//Peripheral Clock Disable

#define GPIOA_PCLK_DIS() (RCC -> AHB1ENR&= ~(1 << 0))
#define GPIOB_PCLK_DIS() (RCC -> AHB1RSTR&= ~(1 << 1))
#define GPIOC_PCLK_DIS() (RCC -> AHB1RSTR&= ~(1 << 2))
#define GPIOD_PCLK_DIS() (RCC -> AHB1RSTR&= ~(1 << 3))
#define GPIOE_PCLK_DIS() (RCC -> AHB1RSTR&= ~(1 << 4))

#define GPIOA_REG_RESET()                   do{(RCC ->AHB1RSTR |=(1 << 0));(RCC ->AHB1RSTR &=~(1 << 0));}while(0)
#define GPIOB_REG_RESET()                   do{(RCC ->AHB1RSTR |=(1 << 1));(RCC ->AHB1RSTR &=~(1 << 1));}while(0)
#define GPIOC_REG_RESET()                   do{(RCC ->AHB1RSTR |=(1 << 2));(RCC ->AHB1RSTR &=~(1 << 2));}while(0)
#define GPIOD_REG_RESET()                   do{(RCC ->AHB1RSTR |=(1 << 3));(RCC ->AHB1RSTR &=~(1 << 3));}while(0)
#define GPIOE_REG_RESET()                   do{(RCC ->AHB1RSTR |=(1 << 4));(RCC ->AHB1RSTR &=~(1 << 4));}while(0)

// Some Generic macros

#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

#include "stm32f407xx_gpio_driver.h"

#endif

