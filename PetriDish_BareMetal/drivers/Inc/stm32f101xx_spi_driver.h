/*
 * stm32f101xx_spi_driver.h
 *
 *  Created on: Nov 15, 2024
 *      Author: arron
 *
 *      Note - This has only been written to support full duplex mode
 */

#ifndef INC_STM32F101XX_SPI_DRIVER_H_
#define INC_STM32F101XX_SPI_DRIVER_H_

#include "stm32f101xx.h"

typedef struct {
	uint8_t	LSBFirst;				/* @LSBFirst */
	uint8_t	DataWidth;				/* @DataWidth */
}SPI_BusConfig_t;

typedef struct {
	uint8_t	SPI_DeviceMode;			/* @SPI_DeviceMode */
	SPI_BusConfig_t	SPI_BusConfig;	/* @SPI_BusConfig_t */
	uint8_t	SPI_ClockPhase;			/* @SPI_ClockPhase */
	uint8_t	SPI_ClockPolarity;		/* @SPI_ClockPolarity */
	uint8_t	SPI_SSM;				/* @SPI_SSM */
	uint8_t	SPI_SclkSpeed;			/* @SPI_SclkSpeed */
}SPI_Config_t;

/*
 * This is a handle structure for a SPI peripheral
 */
typedef struct {
	SPI_RegDef_t	*pSPI;			/* Holds the base address of SPIx(x:0,1) */
	SPI_Config_t	SPIConfig;		/* Holds configuration settings for SPI peripheral */
}SPI_Handle_t;

/*
 * @LSBFirst
 */
#define SPI_TX_MSBFIRST	0
#define SPI_TX_LSBFIRST	1

/*
 * @DataWidth
 */
#define SPI_DATAFORMAT8BITS		0
#define SPI_DATAFORMAT16BITS	1
/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE 0
#define SPI_DEVICE_MODE_MASTER 1

/*
 * @SPI_ClockPhase
 */
#define SPI_CPHA_FIRST	0
#define SPI_CPHA_SECOND	1

/*
 * @SPI_ClockPolarity
 */
#define SPI_CPOL_IDLELOW		0
#define SPI_CPOL_IDLEHIGH	1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI	0
#define SPI_SSM_EN	1

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_DIV2	0	// Either APB1 (SPI2) or APB2 (SPI1) clock speed divided by 2
#define SPI_SCLK_DIV4	1	// Either APB1 (SPI2) or APB2 (SPI1) clock speed divided by 4
#define SPI_SCLK_DIV8	2	// Either APB1 (SPI2) or APB2 (SPI1) clock speed divided by 8
#define SPI_SCLK_DIV16	3	// Either APB1 (SPI2) or APB2 (SPI1) clock speed divided by 16
#define SPI_SCLK_DIV32	4	// Either APB1 (SPI2) or APB2 (SPI1) clock speed divided by 32
#define SPI_SCLK_DIV64	5	// Either APB1 (SPI2) or APB2 (SPI1) clock speed divided by 64
#define SPI_SCLK_DIV128	6	// Either APB1 (SPI2) or APB2 (SPI1) clock speed divided by 128
#define SPI_SCLK_DIV256	7	// Either APB1 (SPI2) or APB2 (SPI1) clock speed divided by 256


/*
 * SPI related status flag definitions
 */
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)

/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_Init(SPI_Handle_t *pSPIxHandle);
void SPI_DeInit(SPI_Handle_t *pSPIxHandle);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_SendReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t *pRxBuffer, uint32_t Len);
//void SPI_IT_Config(void);		/* no interrupts required so not written */
//void SPI_IT_Handler(void);	/* no interrupts required so not written */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_EnorDI(SPI_RegDef_t *pSPIx, uint8_t EnorDI);


#endif /* INC_STM32F101XX_SPI_DRIVER_H_ */
