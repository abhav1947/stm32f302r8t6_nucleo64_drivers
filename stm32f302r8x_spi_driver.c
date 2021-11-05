/*
 * stm32f302r8x_spi_driver.c
 *
 *  Created on: 05-Nov-2021
 *      Author: Abhav S Velidi
 */

#include "stm32f302r8x_spi_driver.h"

/***********************************************************Needs to be completed************************************************************/
//Peripheral clock enable for particular SPI chosen
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}

}


//Initialization of SPI
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Lest configure the CR1 register
	uint32_t tempreg = 0;

	//1. Configure Device Mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configure the BUS configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_CONFIG_FD)
	{
		//bidirectional Mode must be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_CONFIG_HD)
	{
		//bidirectional mode must be set
		tempreg |= 1 << SPI_CR1_BIDIMODE;

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI Mode must be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//Receiving only bit must be set
		tempreg |= 1 << SPI_CR1_RXONLY;
	}

	//3. DFF length
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_CRCL;

	//4.Clock Speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//5.CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6.CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7.SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}


//De-Initialization of SPI
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

//Flag status of TXE
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//Data Send
//This is Blocking call
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE flag is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2.Check the DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_CRCL))
		{
			//16 bit DFF
			//1.Load the data into the Data Register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			/*    pTxBuffer is a 8 bit address but DR is set to 16 bit address so
			 * 	  we do type casting from 8bit to 16bit data i.e (uint16_t*)pTxBuffer
			 * 	  Now after type casting we need to dereference the pointer to get the value stored in the address
			 * 	  For that we do dereferencing by *((uint16_t*)pTxBuffer)              */
			//2.Decrease the length twice
			Len--;
			Len--;
			//3.Increment the Tx buffer address by 2 as we have transfered two bytes of data
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			//1.Load the data into data register
			pSPIx->DR = *(pTxBuffer);
			//2.Decrement the length once
			Len--;
			//3.Increment the Tx buffer once as we have transfered only 1 byte of data or otherwise known as 8bits
			pTxBuffer++;
		}
	}
}


//Data Receive
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}


//Interrupt Configuration
void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
{

}


//Interrupt Priority
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{

}


//Interrupt handling
void SPI_IRQHandling(SPI_Handle_t *pSPIx)
{

}
