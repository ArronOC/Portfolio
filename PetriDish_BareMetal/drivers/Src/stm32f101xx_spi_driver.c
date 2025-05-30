/*
 * stm32f101xx_spi_driver.c
 *
 *  Created on: Nov 15, 2024
 *      Author: arron
 */

#include "stm32f101xx_spi_driver.h"


/*****************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]	- base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
	} else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
	}
}

/*****************************************************************
 * @fn			- SPI_Init
 *
 * @brief		- This function configures the given SPI peripheral
 *
 * @param[in]	- SPI handle
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_Init(SPI_Handle_t *pSPIxHandle)
{
	uint16_t tempreg = 0;

	// Enable the SPI clock source
	SPI_PeriClockControl(pSPIxHandle->pSPI, ENABLE);

	/* Procedure for configuring in SPI master mode */
	if (pSPIxHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
	{
		// 1. Select the BR[2:0] bits to define the serial clock baud rate (see SPI_CR1 register).
		tempreg |= (pSPIxHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

		/*
		 * 2. Select the CPOL and CPHA bits to define one of the four relationships between the
		 * data transfer and the serial clock (see Figure 240).
		 */
		tempreg |= (pSPIxHandle->SPIConfig.SPI_ClockPhase << SPI_CR1_CPHA);
		tempreg |= (pSPIxHandle->SPIConfig.SPI_ClockPolarity << SPI_CR1_CPOL);

		// 3. Set the DFF bit to define 8- or 16-bit data frame format
		tempreg |= (pSPIxHandle->SPIConfig.SPI_BusConfig.DataWidth << SPI_CR1_DFF);

		// 4. Configure the LSBFIRST bit in the SPI_CR1 register to define the frame format.
		tempreg |= (pSPIxHandle->SPIConfig.SPI_BusConfig.LSBFirst << SPI_CR1_LSBFIRST);

		/*
		 * For master mode
		 */
		tempreg |= (pSPIxHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
		if (pSPIxHandle->SPIConfig.SPI_SSM == SPI_SSM_EN)
		{
			/*
			 * When SSM is enabled in master mode, you need to set a 1 to the SSI bit
			 * if the SPE is enabled whilst SSI is 0 a MODF error occurs.
			 * Because the device recognised another source driving the NSS line resulting
			 * in it being reconfigured as a slave
			 */
			tempreg |= (0x1 << SPI_CR1_SSI);
		}
		else
		{
			// would add configuration of SSOE bit here, but not used on this board
		}

		/*
		 * 6. The MSTR and SPE bits must be set (they remain set only if the NSS pin is connected
		 * to a high-level signal).
		 */
		tempreg |= (0x1 << SPI_CR1_MSTR);	/* Set the MTSR (master selection) */
		//tempreg |= (0x1 << SPI_CR1_SPE);	/* Enables the SPI peripheral (SPE) */
	} else if (pSPIxHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE)
	{
		// Not written as the PetriDish board doesn't support this functionality
	}

	pSPIxHandle->pSPI->CR[0] = tempreg;
}

/*****************************************************************
 * @fn			- SPI_DeInit
 *
 * @brief		- This function disables the given SPI peripheral
 *
 * @param[in]	- SPI handle
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_DeInit(SPI_Handle_t *pSPIxHandle)
{
	pSPIxHandle->pSPI->CR[0] &= ~(0x1 << 6);	/* Disables the SPI peripheral (SPE) */
}

/*****************************************************************
 * @fn			- SPI_SendData
 *
 * @brief		- This function sends data over SPI using blocking mode
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- pointer of the data to send
 * @param[in]	- number of data transmissions to take place
 *
 * @return		- none
 *
 * @Note		- this is blocking/polling based implementation, meaning the processor cannot execute other steps
 * 				  whilst executing it
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	if (Len != 0)
	{
		SPI_EnorDI(pSPIx, ENABLE);
		while (Len > 0)
		{
			//1. Check Tx buffer is empty from the SPI_SR TXE bit
			while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

			//2. Check the DFF bit in CR1 to determine data format
			if ((pSPIx->CR[0] & (1 << SPI_CR1_DFF)))
			{
				// 16-bit mode
				pSPIx->DR = *((uint16_t*)pTxBuffer);	// Convert to uint16_t pointer, then deference the pointer
				Len--;
				Len--;
				(uint16_t*)pTxBuffer++;		// increment the pointer by 2 bytes
			} else
			{
				// 8-bit mode
				pSPIx->DR = *pTxBuffer;
				Len--;
				pTxBuffer++;				// increment the pointer by 1 bytes
			}

			// Wait for TX transmit to complete
			while(SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG) == FLAG_SET);

			/*
			 * Couldn't find any explanation as to why the below is required
			 * Without it, later SPI read data arrives in the wrong order
			 */
			if (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_SET)
			{
				pSPIx->DR;
				pSPIx->DR;
				pSPIx->DR;
				pSPIx->DR;
			}

		}
		SPI_EnorDI(pSPIx, DISABLE);
	}
}

/*****************************************************************
 * @fn			- SPI_ReceiveData
 *
 * @brief		- This function receives data over SPI using blocking mode
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- pointer of where to store the data
 * @param[in]	- number of data receptions to take place
 *
 * @return		- none
 *
 * @Note		- this is blocking/polling based implementation, meaning the processor cannot execute other steps
 * 				  whilst executing it
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	if (Len != 0)
	{
		SPI_EnorDI(pSPIx, ENABLE);
		while (Len > 0)
		{
			// have to transmit a 0 otherwise there will be no clock pulse for data to be shifted out of target
			pSPIx->DR = 0;

			//1. Check Rx buffer is not empty from the SPI_SR RXNE bit
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			//2. Check the DFF bit in CR1 to determine data format
			if ((pSPIx->CR[0] & (1 << SPI_CR1_DFF)))
			{
				// 16-bit mode
				// Load the data from DR into Rx buffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;		// increment the pointer by 2 bytes
			} else
			{
				// 8-bit mode
				// Load the data from DR into Rx buffer address
				*pRxBuffer = pSPIx->DR;
				Len--;
				pRxBuffer++;				// increment the pointer by 1 bytes
			}
		}
		SPI_EnorDI(pSPIx, DISABLE);
	}
}

/*****************************************************************
 * @fn			- SPI_SendReceiveData
 *
 * @brief		- This function sends data then receives data over SPI using blocking mode
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- pointer of the data to send
 * @param[in]	- pointer of where to store the data
 * @param[in]	- number of data receptions to take place
 *
 * @return		- none
 *
 * @Note		- this is blocking/polling based implementation, meaning the processor cannot execute other steps
 * 				  whilst executing it
 */
void SPI_SendReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t *pRxBuffer, uint32_t Len)
{
	if (Len != 0)
	{
		SPI_EnorDI(pSPIx, ENABLE);
		while (Len > 0)
		{
			//1. Check Tx buffer is empty from the SPI_SR TXE bit
			while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

			//2. Check the DFF bit in CR1 to determine data format
			if ((pSPIx->CR[0] & (1 << SPI_CR1_DFF)))
			{
				// 16-bit mode
				pSPIx->DR = *((uint16_t*)pTxBuffer);	// Convert to uint16_t pointer, then deference the pointer
				(uint16_t*)pTxBuffer++;		// increment the pointer by 2 bytes
			} else
			{
				// 8-bit mode
				pSPIx->DR = *pTxBuffer;
				pTxBuffer++;				// increment the pointer by 1 bytes
			}

			//3. Check Rx buffer is not empty from the SPI_SR RXNE bit
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			//4. Check the DFF bit in CR1 to determine data format
			if ((pSPIx->CR[0] & (1 << SPI_CR1_DFF)))
			{
				// 16-bit mode
				// Load the data from DR into Rx buffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;		// increment the pointer by 2 bytes
			} else
			{
				// 8-bit mode
				// Load the data from DR into Rx buffer address
				*pRxBuffer = pSPIx->DR;
				Len--;
				pRxBuffer++;				// increment the pointer by 1 bytes
			}
		}
		SPI_EnorDI(pSPIx, DISABLE);
	}
}

/*****************************************************************
 * @fn			- SPI_GetFlagStatus
 *
 * @brief		- This function checks if a SPI flag is set
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- flag in the SR to check
 *
 * @return		- 1 for flag set, 0 for flag not set
 *
 * @Note		-
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	} else
	{
		return FLAG_RESET;
	}
}

/*****************************************************************
 * @fn			- SPI_EnorDI
 *
 * @brief		- This function Enables/Disables the SPI peripheral
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- enable or disable
 *
 * @return		-
 *
 * @Note		-
 */
void SPI_EnorDI(SPI_RegDef_t *pSPIx, uint8_t EnorDI)
{
	if (EnorDI)
	{
		pSPIx->CR[0] |= (0x1 << SPI_CR1_SPE);	/* Enables the SPI peripheral (SPE) */
	} else
	{
		//1. Wait for TX buffer to be empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		//2. Wait for SPI Peripheral to not be busy
		while(SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG) == FLAG_SET);
		pSPIx->CR[0] &= ~(0x1 << SPI_CR1_SPE);	/* Disable the SPI peripheral (SPE) */
	}

}
