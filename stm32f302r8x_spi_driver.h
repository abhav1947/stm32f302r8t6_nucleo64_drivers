/*
 * stm32f302r8x_spi_driver.h
 *
 *  Created on: 05-Nov-2021
 *      Author: Abhav S Velidi
 */

#ifndef INC_STM32F302R8X_SPI_DRIVER_H_
#define INC_STM32F302R8X_SPI_DRIVER_H_

#include"stm32f302r8.h"

//Structure created to handle the SPI configurations
typedef struct
{

	volatile uint8_t SPI_DeviceMode;	// Used to set the device in SLAVE or MASTER Mode
	volatile uint8_t SPI_BusConfig; 	//Used to set the MOSI and MISO in DUPLEX, HALF DUPLEX, SIMPLEX Mode
	volatile uint8_t SPI_SclkSpeed; 	// Baud Rate of transmission or clock to be fed to the slave
	volatile uint8_t SPI_DFF;			//Used to set the data line as 16bit or 8bit
	volatile uint8_t SPI_CPOL;			//Used to set the polarity of the clock. By default the Polarity is set to "0"
	volatile uint8_t SPI_CPHA;			//Used to set the phase of clock either 1st edge or 2nd edge of the clock
	volatile uint8_t SPI_SSM;			//Used to set software slave management

}SPI_Config_t;

//handle structure used to select which SPI to be used and set the particular SPI Configuration
typedef struct
{

	SPI_RegDef_t *pSPIx;	//Used to select the SPI base address
	SPI_Config_t SPIConfig; //Used to set the configuration of the SPI chosen

}SPI_Handle_t;


//SPI Device Mode
#define SPI_DEVICE_MODE_MASTER	1	//to select master
#define SPI_DEVICE_MODE_SLAVE	0	//to select slave


//SPI Bus configuration Modes
#define SPI_CONFIG_FD				1	//Full Duplex mode
#define	SPI_CONFIG_HD				2	//Half Duplex Mode
#define SPI_CONFIG_SIMPLEX_RXONLY	3	//Simple Mode Reception Only


//SPI SCLK Speed Modes
#define SPI_SCLK_SPEED_DIV2		0	//Speed is Freq divided by 2
#define SPI_SCLK_SPEED_DIV4		1	//Speed is Freq divided by 4
#define SPI_SCLK_SPEED_DIV8		2	//Speed is Freq divided by 8
#define SPI_SCLK_SPEED_DIV16	3	//Speed is Freq divided by 16
#define SPI_SCLK_SPEED_DIV32	4	//Speed is Freq divided by 32
#define SPI_SCLK_SPEED_DIV64	5	//Speed is Freq divided by 64
#define SPI_SCLK_SPEED_DIV128	6	//Speed is Freq divided by 128
#define SPI_SCLK_SPEED_DIV256	7	//Speed is Freq divided by 256


//SPI DFF Modes
#define SPI_DFF_8BITS	0	//8bit data
#define SPI_DFF_16BITS	1	//16bit data


//SPI CPOl Modes
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0


//SPI CPHA Modes
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0


//SPI SSM
#define SPI_SSM_EN	1	//Hardware Slave management
#define SPI_SSM_DI	0	//Software Slave management

//Flag Macros
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)



/***************************************************************************************************************************
 *                                                      API supported by this driver
 **************************************************************************************************************************/

//Peripheral Clock Setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//Initialization and De-initialization
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//Data Send And Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);		//pTxBuffer is a pointer that points to the address where data must be read from
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);	//pRxBuffer is a pointer that point to the address where data must me stored

//SPI Interrupt handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIx);

//SPI Peripheral Control
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

#endif /* INC_STM32F302R8X_SPI_DRIVER_H_ */
