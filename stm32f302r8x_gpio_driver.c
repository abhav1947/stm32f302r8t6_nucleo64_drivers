/*
 * stm32f302r8x_gpio_driver.c
 *
 *  Created on: 07-Aug-2021
 *      Author: Abhav S Velidi
 */


#include "stm32f302r8x_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * *pGPIOx         - base address of the gpio peripheral
 * EnorDi         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if(pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}else if(pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}else if(pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}

	}
}



/*********************************************************************
 * @fn      		  - GPIO_Initialization
 *
 * @brief             - i.e all the register of the GPIO peripheral must be set as required using the current function
 *
 * *pGPIOHandle       - It is a structure which contains Base address and the refisterg configyartion of the particula selected peripheral
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_Init(GPIO_Handel_t *pGPIOHandle)
{
	uint32_t temp = 0;
	GPIO_PeriClockControl(pGPIOHandle->pGPIOX, ENABLE);
	// 1 . configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle -> pGPIOX -> MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle -> pGPIOX -> MODER |= temp;
		temp = 0;
	}else
	{

	}

	temp = 0;

	// 2 . configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOX -> OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOX -> OSPEEDR |= temp;
	temp = 0;

	// 3 . configure the pupd settings
	temp = pGPIOHandle ->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOX -> PUPDR &= ~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOX -> PUPDR |= temp;
	temp = 0;

	// 4 . Configure the output type
	temp = pGPIOHandle-> GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOX -> OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle -> pGPIOX -> OTYPER |= temp;
	temp = 0;

	// 5 . configure the alternate function
	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOX->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOX->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}

	//6. IRQ handeling
	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
	{
		//1. configure FTSR
		EXTI -> FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//clear corresponding bit
		EXTI -> FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
	{
		//2. configure RTSR
		EXTI -> RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//clear corresponding bit
		EXTI -> RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
	{
		//2. configure both RTSR and FTSR
		EXTI -> RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//clear corresponding bit
		EXTI -> FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// Configure the GPIO port selection in SYSCFG_EXTICR
	uint8_t temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
	uint8_t temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle -> pGPIOX);
	SYSCFG_PCLK_EN();
	SYSCFG -> EXTICR[temp1] = (portcode << temp2*4);// Stores the portcode for particular GPIO Port
	//Enable the EXTI interrupt delivery using IMR
	EXTI -> IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
}



/*********************************************************************
 * @fn      		  - GPIO_DeInitialization
 *
 * @brief             - Used to reset the registers
 *
 * *pGPIOx            - base address of the gpio peripheral
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
					{
						GPIOA_REG_RESET();
					}else if(pGPIOx == GPIOB)
					{
						GPIOB_REG_RESET();
					}else if(pGPIOx == GPIOC)
					{
						GPIOC_REG_RESET();
					}else if(pGPIOx == GPIOD)
					{
						GPIOD_REG_RESET();
					}else if(pGPIOx == GPIOE)
					{
						GPIOE_REG_RESET();
					}else if(pGPIOx == GPIOF)
					{
						GPIOF_REG_RESET();
					}else if(pGPIOx == GPIOG)
					{
						GPIOG_REG_RESET();
					}else if(pGPIOx == GPIOH)
					{
						GPIOH_REG_RESET();
					}
}



/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Used to read data from the peripheral pins
 *
 * *pGPIOx            - base address of the gpio peripheral
 * PinNumber          - Gives the pin number of the peripheral
 *
 * @return            - uint8_t
 * @Note              -  none
 */
uint8_t GPIO_ReadFromInputpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx-> IDR >> PinNumber) & (0x00000001));
	return value;
}



/*********************************************************************
 * @fn      		  - GPIO_ReadFromPort
 *
 * @brief             - Used to read data from the peripheral port
 *
 * *pGPIOx            - base address of the gpio peripheral
 *
 * @return            - uint16_t
 * @Note              -  none
 */
uint16_t GPIO_ReadFromInputport(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
		value = (uint16_t) (pGPIOx-> IDR );
		return value;
}



/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Used to write data out through the peripheral pins
 *
 * *pGPIOx            - base address of the gpio peripheral
 * PinNumber          - Gives the pin number of the peripheral
 * value              - Value to be written out
 * @return            -  none
 * @Note              -  none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx -> ODR |= (1 << PinNumber);//write 1 to output data register
	}else
	{
		pGPIOx -> ODR &= ~(1 << PinNumber);//write 0 to output data register
	}
}



/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -  Used to write data out through the peripheral  port
 *
 * *pGPIOx            - base address of the gpio peripheral
 * value              - Value to be written out
 *
 * @return            -  none
 * @Note              -  none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx -> ODR = Value;
}



/*********************************************************************
 * @fn      		  -  GPIO_ToggleOutputPin
 *
 * @brief             -  Used to toggle peripheral pin
 *
 * *pGPIOx            - base address of the gpio peripheral
 * PinNumber          - Gives the pin number of the peripheral
 *
 * @return            -  none
 * @Note              -  none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx -> ODR ^= (1 << PinNumber);
}



/*********************************************************************
 * @fn      		  -  GPIO_IRQConfig
 *
 * @brief             -  Used to configure the interrupts
 *
 * IRQNumber            - Interupt number
 * IRPPriority          - Interput priority
 * EnorDi               - Enable or Disable Interupt
 *
 * PinNumber          - Gives the pin number of the peripheral
 * @return            -  none
 * @Note              -  none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31) // to program interrupt no 1 to 31
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if (IRQNumber > 31 && IRQNumber < 64)  // to program interrupt no 32 to 63
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if (IRQNumber >= 64 && IRQNumber < 96)  // to program interrupt no 64 to 95
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if (IRQNumber > 31 && IRQNumber < 96)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}

	}
}

/* Priority enable API*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	//lets find the ipr register first
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= (IRQPriority << shift_amount );
}

/*********************************************************************
 * @fn      		  -  GPIO_IRQConfig
 *
 * @brief             -  Used to configure the interrupt Handling
 *
 * @return            -  none
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear exti pr register
	if(EXTI->PR & ( 1 << PinNumber))
			{
				EXTI->PR |= (1 << PinNumber);
			}
}


































