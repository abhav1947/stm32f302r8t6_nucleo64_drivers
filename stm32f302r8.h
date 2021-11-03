/*
 * stm32f302r8.h Macros File
 *
 *  Created on: 06-Aug-2021
 *      Author: Abhav S Velidi
 */

#ifndef INC_STM32F302R8_H_
#define INC_STM32F302R8_H_

#include<stdint.h>

/*ARM Cortex M4 processor NVIC ISERx Register Addresses*/
#define NVIC_ISER0		((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*)0xE000E10C)

/*ARM Cortex M4 processor NVIC ICERx Register Addresses*/
#define NVIC_ICER0		((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1		((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3		((volatile uint32_t*)0xE000E18C)

/*ARM Cortex M4 processor priority base Register Addresses*/
#define NVIC_PR_BASE_ADDR	((volatile uint32_t*)0xE000E400)

//No of priority bits implemented
#define NO_PR_BITS_IMPLEMENTED		4

//IRQ Priority
#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO15		15


/*C macros for flash and SRAM memory addresses */
#define FLASH_BASEADDR		0x80000000U
#define SRAM_BASEADDR		0x20000000U
#define ROM_BASEADDR		0x1FFFD800U

/*C macros for BUS domain base addresses */
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		0x40000000U
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x48000000U
#define AHB3PERIPH_BASEADDR		0x50000000U

/*C Macros for each and every peripheral on BUS domains*/

#define RCC_BASEADDR 		(AHB1PERIPH_BASEADDR + 0X1000)

#define GPIOA_BASEADDR		(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB2PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB2PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB2PERIPH_BASEADDR + 0X1C00)

#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x7800)

#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)
#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)

#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x0000)
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x0400)


/*****************************peripheral register definition structure*****************************************/
 typedef struct
 {
	 volatile uint32_t MODER;		/*GPIO port mode register*/
	 volatile uint32_t OTYPER;		/*GPIO port output type register*/
	 volatile uint32_t OSPEEDR;		/*GPIO port output speed register*/
	 volatile uint32_t PUPDR;		/*GPIO port pull-up/pull-down register*/
	 volatile uint32_t IDR;			/*GPIO port input data register*/
	 volatile uint32_t ODR;			/*GPIO port output data register*/
	 volatile uint32_t BSRR;		/*GPIO port bit set/reset register*/
	 volatile uint32_t LCKR;		/*GPIO port configuration lock register*/
	 volatile uint32_t AFR[2];		/*AFR[0] low ; AFR[1] High ; GPIO alternate function register*/

 }GPIO_RegDef_t;

 typedef struct
  {
 	 volatile uint32_t CR;			/*Clock control register*/
 	 volatile uint32_t CFGR;		/*Clock configuration register*/
 	 volatile uint32_t CIR;			/*Clock interrupt register*/
 	 volatile uint32_t APB2RSTR;	/*APB2 peripheral reset register*/
 	 volatile uint32_t APB1RSTR;	/*APB1 peripheral reset register*/
 	 volatile uint32_t AHBENR;		/*AHB peripheral clock enable register*/
 	 volatile uint32_t APB2ENR;		/*APB2 peripheral clock enable register*/
 	 volatile uint32_t APB1ENR;		/*APB1 peripheral clock enable register*/
 	 volatile uint32_t BDCR;		/*RTC domain control register*/
 	 volatile uint32_t CSR;			/*Control/status register*/
 	 volatile uint32_t AHBRSTR;		/*AHB peripheral reset register*/
 	 volatile uint32_t CFGR2;		/*Clock configuration register 2*/
 	 volatile uint32_t CFGR3;		/*Clock configuration register 3*/

  }RCC_RegDef_t;


/*Peripheral register definition structure for EXTI*/
  typedef struct
  {
	volatile uint32_t IMR;  //Interrupt mask register
	volatile uint32_t EMR;  //Event mask register
	volatile uint32_t RTSR;  //Rising trigger selection register
	volatile uint32_t FTSR;  //Falling Trigger selection register
	volatile uint32_t SWEIR;  //Software event interrupt mask register
	volatile uint32_t PR;  //Pending register
  }EXTI_RegDef_t;

/*Peripheral register definition structure for SYSCONFIG*/
  typedef struct
  {
	  volatile uint32_t CFGR; // configuration register
	  volatile uint32_t Reserved;
	  volatile uint32_t EXTICR[4]; // EXTI Control Register
	  volatile uint32_t CFGR2;
  }SYSCFG_RegDef_t;

/*peripheral definition*/
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI  ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*C macros for clock enable of GPIO*/
#define GPIOA_PCLK_EN()  (RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN()  (RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN()  (RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN()  (RCC->AHBENR |= (1<<20))
#define GPIOE_PCLK_EN()  (RCC->AHBENR |= (1<<21))
#define GPIOF_PCLK_EN()  (RCC->AHBENR |= (1<<22))
#define GPIOG_PCLK_EN()  (RCC->AHBENR |= (1<<23))
#define GPIOH_PCLK_EN()  (RCC->AHBENR |= (1<<16))

/*C macros for clock enable of I2c peripheral*/
#define I2C1_PCLK_EN()  (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()  (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()  (RCC->APB1ENR |= (1<<30))

/*C macros for clock enable of SPI peripheral*/
#define SPI1_PCLK_EN()  (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()  (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()  (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()  (RCC->APB2ENR |= (1<<15))

/*C macros for clock enable of SPI peripheral*/
#define USART1_PCLK_EN()  (RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN()  (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()  (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()  (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()  (RCC->APB1ENR |= (1<<20))

/*C macros for clock enable of SYSCFG peripheral*/
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1<<0));

/*C macros for clock disable of GPIO*/
#define GPIOA_PCLK_DI()  (RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI()  (RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI()  (RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI()  (RCC->AHBENR &= ~(1<<20))
#define GPIOE_PCLK_DI()  (RCC->AHBENR &= ~(1<<21))
#define GPIOF_PCLK_DI()  (RCC->AHBENR &= ~(1<<22))
#define GPIOG_PCLK_DI()  (RCC->AHBENR &= ~(1<<23))
#define GPIOH_PCLK_DI()  (RCC->AHBENR &= ~(1<<16))

/*C macros for clock enable of I2C peripheral*/
#define I2C1_PCLK_DI()  (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()  (RCC->APB1ENR &= ~(1<<30))

/*C macros for clock enable of SPI peripheral*/
#define SPI1_PCLK_DI()  (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()  (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()  (RCC->APB2ENR &= ~(1<<15))

/*C macros for clock enable of SPI peripheral*/
#define USART1_PCLK_DI()  (RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()  (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()  (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()  (RCC->APB1ENR &= ~(1<<20))

/*C macros for clock disable of SYSCFG peripheral*/
#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &= ~(1<<0));

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
								    (x == GPIOD) ? 3 :\
								    (x == GPIOE) ? 4 :\
								    (x == GPIOF) ? 5 :0)

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	0

/*C macros for reseting the peripherals*/
#define GPIOA_REG_RESET()  do{(RCC->AHBRSTR |= (1<<17)); (RCC->AHBRSTR &= ~(1<<17));}while(0)
#define GPIOB_REG_RESET()  do{(RCC->AHBRSTR |= (1<<18)); (RCC->AHBRSTR &= ~(1<<18));}while(0)
#define GPIOC_REG_RESET()  do{(RCC->AHBRSTR |= (1<<19)); (RCC->AHBRSTR &= ~(1<<19));}while(0)
#define GPIOD_REG_RESET()  do{(RCC->AHBRSTR |= (1<<20)); (RCC->AHBRSTR &= ~(1<<20));}while(0)
#define GPIOE_REG_RESET()  do{(RCC->AHBRSTR |= (1<<21)); (RCC->AHBRSTR &= ~(1<<21));}while(0)
#define GPIOF_REG_RESET()  do{(RCC->AHBRSTR |= (1<<22)); (RCC->AHBRSTR &= ~(1<<22));}while(0)
#define GPIOG_REG_RESET()  do{(RCC->AHBRSTR |= (1<<23)); (RCC->AHBRSTR &= ~(1<<23));}while(0)
#define GPIOH_REG_RESET()  do{(RCC->AHBRSTR |= (1<<16)); (RCC->AHBRSTR &= ~(1<<16));}while(0)

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

#include "stm32f302r8x_gpio_driver.h"

#endif /* INC_STM32F302R8_H_ */


