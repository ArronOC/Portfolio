/*
 * CAV25M01.c
 *
 *  Created on: Jul 14, 2024
 *      Author: arron
 */
#include "CAV25M01.h"

/*****************************************************************
 * @fn			- CAV25M01_Init
 *
 * @brief		- This function checks the device can be communicated with and configures Block protect mode
 *
 * @param[in]	- Handle structure for CAV25M01
 *  *
 * @return		- 1 for success, 0 for fail
 *
 * @Note		-
 */
uint8_t CAV25M01_Init(CAV25M01_Handle_t *pdev)
{
	uint8_t txData[5], rxData[5], statusReg, origVal;
	//memset(txData,0,5);
	//memset(rxData,0,5);

	/*
	 * 1. Enable WPEN bit to enable nWP pin
	 * 	  Enable BP bits to enable block protect
	 */
	statusReg = CAV25M01_ReadStatusReg(pdev);
	statusReg |= WPENmask;
	statusReg |= BPmask;
	//statusReg = 0;
	CAV25M01_WriteStatusReg(pdev, &statusReg);

	//2. read data stored in @U6_TestAddr
	txData[0] = OPCODE_READ;
	txData[1] = (uint8_t)(U6_TestAddr >> 16);
	txData[2] = (uint8_t)(U6_TestAddr >> 8);
	txData[3] = (uint8_t)(U6_TestAddr);
	CAV25M01_SendReceive(pdev, txData, rxData, 5);
	origVal = rxData[4];

	//3. Send the inverse of the read data
	txData[0] = OPCODE_WRITE;
	txData[4] = ~origVal;
	CAV25M01_SendData(pdev, txData, 5);

	//4. Read the data location again
	txData[0] = OPCODE_READ;
	txData[4] = 0;
	CAV25M01_SendReceive(pdev, txData, rxData, 5);

	//5. Check write operation was successful
	if (rxData[4] != origVal)
	{
		// Init passed
		return 1;
	} else
	{
		// Init failed
		return 0;
	}

}

/*****************************************************************
 * @fn			- CAV25M01_SendData
 *
 * @brief		- This function sends data over SPI using blocking mode
 *
 * @param[in]	- Handle structure for CAV25M01
 * @param[in]	- pointer of data to transmit
 * @param[in]	- number of data transmissions to take place
 *
 * @return		- none
 *
 * @Note		- this is blocking/polling based implementation, meaning the processor cannot execute other steps
 * 				  whilst executing it
 */
void CAV25M01_SendData(CAV25M01_Handle_t *pdev, uint8_t *txData, uint8_t bytes)
{
	CAV25M01_enableWriteMode(pdev);
	CAV25M01_DevReady(pdev);		// Wait for device to finish last write cycle

	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable WP
	GPIO_WriteToOutputPin(pdev->pnHOLD->pGPIOx, pdev->pnHOLD->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);	// Disable HOLD
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);	// Enable Chip select
	SPI_SendData(pdev->pSPIhandle->pSPI, txData, bytes);
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET); 	// Disable Chip select
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET); 	// Enable WP

	CAV25M01_disableWriteMode(pdev);
}



/*****************************************************************
 * @fn			- CAV25M01_SendReceive
 *
 * @brief		- This function sends & receives data over SPI using blocking mode
 *
 * @param[in]	- Handle structure for CAV25M01
 * @param[in]	- pointer of data to transmit
 * @param[in]	- pointer for where to store received data
 * @param[in]	- number of SPI clock cycles to take place (writes + reads)
 *
 * @return		- none
 *
 * @Note		- this is blocking/polling based implementation, meaning the processor cannot execute other steps
 * 				  whilst executing it
 */
void CAV25M01_SendReceive(CAV25M01_Handle_t *pdev, uint8_t *txData, uint8_t *rxData, uint8_t ClockCycles)
{
	CAV25M01_enableWriteMode(pdev);
	CAV25M01_DevReady(pdev);		// Wait for device to finish last write cycle

	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable WP
	GPIO_WriteToOutputPin(pdev->pnHOLD->pGPIOx, pdev->pnHOLD->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);	// Disable HOLD
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);	// Enable Chip select
	SPI_SendReceiveData(pdev->pSPIhandle->pSPI, txData, rxData, ClockCycles);
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET); 	// Disable Chip select
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET); 	// Enable WP

	CAV25M01_disableWriteMode(pdev);
}


/*****************************************************************
 * @fn			- CAV25M01_DevReady
 *
 * @brief		- This function queries the status register and waits until the nRDY bit is reset
 *
 * @param[in]	- Handle structure for CAV25M01
 *
 * @return		- none
 *
 * @Note		- Currently no timeout feature so infinite loops are possible
 */
void CAV25M01_DevReady(CAV25M01_Handle_t *pdev)
{
	uint8_t nReady, loop = 0, maxLoops = 5;
	do {
		nReady = CAV25M01_ReadStatusReg(pdev) & nRDYmask;														// Check nRDY bit in status register
		loop++;
		DELAY_MS(5);
		//for (uint32_t i = 0; i < 100000; i++);
	} while ((nReady == nRDYmask) && (loop < maxLoops));

	// timeout occurred CAV25M01 is stuck in a internal write cycle, power cycle to exit this state
	if (loop == maxLoops) {
		//status = HAL_ERROR;
	}
}


/*****************************************************************
 * @fn			- CAV25M01_ReadStatusReg
 *
 * @brief		- This function reads the status register
 *
 * @param[in]	- Handle structure for CAV25M01
 *
 * @return		- contents of the status register
 *
 * @Note		-
 */
uint8_t CAV25M01_ReadStatusReg(CAV25M01_Handle_t *pdev)
{
	uint8_t rxData[2] = {0,0};
	uint8_t txData[2] = {OPCODE_RDSR,0};
	GPIO_WriteToOutputPin(pdev->pnHOLD->pGPIOx, pdev->pnHOLD->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable HOLD
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);		// Enable Chip select
	SPI_SendReceiveData(pdev->pSPIhandle->pSPI, txData, rxData, 2);
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable Chip select
	return rxData[1];
}

/*****************************************************************
 * @fn			- CAV25M01_WriteStatusReg
 *
 * @brief		- This function writes data to the status register
 *
 * @param[in]	- Handle structure for CAV25M01
 * @param[in]	- Data to write to status register
 *
 * @return		- contents of the status register
 *
 * @Note		- the IPL and LIP bits cannot be set to 1 using the same WRSR instruction (refer to CAV25M01 datasheet)
 */
void CAV25M01_WriteStatusReg(CAV25M01_Handle_t *pdev, uint8_t *data)
{
	CAV25M01_DevReady(pdev);		// Wait for device to finish last write cycle
	uint8_t txData[2] = {OPCODE_WREN, 0};
	// Send write enable command (can't use defined function otherwise infinite loop possible)
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable WP
	GPIO_WriteToOutputPin(pdev->pnHOLD->pGPIOx, pdev->pnHOLD->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);	// Disable HOLD
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);	// Enable Chip select
	SPI_SendData(pdev->pSPIhandle->pSPI, txData, 1);
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET); 	// Disable Chip select
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET); 	// Enable WP

	CAV25M01_DevReady(pdev);		// Wait for device to finish last write cycle
	// Write to the status register
	txData[0] = OPCODE_WRSR;
	txData[1] = *data;
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);			// Disable WP
	GPIO_WriteToOutputPin(pdev->pnHOLD->pGPIOx, pdev->pnHOLD->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable HOLD
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);		// Enable Chip select
	SPI_SendData(pdev->pSPIhandle->pSPI, txData, 2);
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable Chip select
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET); 		// Enable WP
}

/*****************************************************************
 * @fn			- CAV25M01_SerNum
 *
 * @brief		- This function either stores or reads the serial number to/from EEPROM
 *
 * @param[in]	- Handle structure for CAV25M01
 * @param[in]	- Pointer to where serial number is stored for storing or where to save for recalling
 * @param[in]	- 1 for storing, 0 for recalling
 *
 * @return		-
 *
 * @Note		- serial number is of size @SerNumSizeBytes
 */
void CAV25M01_SerNum(CAV25M01_Handle_t *pdev, char *SerNum, uint8_t STOREorRECALL)
{
	uint8_t txData[SerNumSizeBytes+4];
	uint8_t rxData[SerNumSizeBytes+4];
	//memset(txData,0,(SerNumSizeBytes+4));	// Could also set size by (sizeof(txData)/sizeof(txData[0]))
	//memset(rxData,0,(SerNumSizeBytes+4));

	CAV25M01_IdentPageSwitch(pdev, DISABLE);	/* Switch from Ident page to main EEPROM */

	if (STOREorRECALL == STORE)
	{
		txData[0] = OPCODE_WRITE;
		txData[1] = (uint8_t)(U6_SERIALADDR >> 16);
		txData[2] = (uint8_t)(U6_SERIALADDR >> 8);
		txData[3] = (uint8_t)(U6_SERIALADDR);
		for (uint8_t i = 0; i < SerNumSizeBytes; i++)
		{
			txData[i+4] = (uint8_t)SerNum[i];
		}
		CAV25M01_SendData(pdev, txData, (SerNumSizeBytes + 4));

	} else if (STOREorRECALL == RECALL)
	{
		txData[0] = OPCODE_READ;
		txData[1] = (uint8_t)(U6_SERIALADDR >> 16);
		txData[2] = (uint8_t)(U6_SERIALADDR >> 8);
		txData[3] = (uint8_t)(U6_SERIALADDR);
		CAV25M01_SendReceive(pdev, txData, rxData, (SerNumSizeBytes+4));
		for (uint8_t i = 0; i < (SerNumSizeBytes+4); i++)
		{
			SerNum[i] = (char)rxData[i+4];
		}
	}
}

/*****************************************************************
 * @fn			- CAV25M01_AccData
 *
 * @brief		- This function either stores or reads the accelerometer offset data to/from EEPROM
 *
 * @param[in]	- Handle structure for CAV25M01
 * @param[in]	- Pointer to where Xoffset is stored for storing or where to save for recalling
 * @param[in]	- Pointer to where Yoffset is stored for storing or where to save for recalling
 * @param[in]	- Pointer to where Zoffset is stored for storing or where to save for recalling
 * @param[in]	- 1 for storing, 0 for recalling
 *
 * @return		-
 *
 * @Note		- serial number is of size @SerNumSizeBytes
 */
void CAV25M01_AccData(CAV25M01_Handle_t *pdev, uint8_t *Xoff, uint8_t *Yoff, uint8_t *Zoff, uint8_t STOREorRECALL)
{
	uint8_t txData[5], rxData[5];

	CAV25M01_IdentPageSwitch(pdev, DISABLE);	/* Switch from Ident page to main EEPROM */

	if (STOREorRECALL == STORE)
	{
		txData[0] = OPCODE_WRITE;
		/* Three independent writes in case addresses aren't consecutive in the future */
		/* Save X offset data to EEPROM */
		txData[1] = (uint8_t)(U6_XoffAddr >> 16);
		txData[2] = (uint8_t)(U6_XoffAddr >> 8);
		txData[3] = (uint8_t)(U6_XoffAddr);
		txData[4] = *Xoff;
		CAV25M01_SendData(pdev, txData, 5);

		/* Save Y offset data to EEPROM */
		txData[1] = (uint8_t)(U6_YoffAddr >> 16);
		txData[2] = (uint8_t)(U6_YoffAddr >> 8);
		txData[3] = (uint8_t)(U6_YoffAddr);
		txData[4] = *Yoff;
		CAV25M01_SendData(pdev, txData, 5);

		/* Save Z offset data to EEPROM */
		txData[1] = (uint8_t)(U6_ZoffAddr >> 16);
		txData[2] = (uint8_t)(U6_ZoffAddr >> 8);
		txData[3] = (uint8_t)(U6_ZoffAddr);
		txData[4] = *Zoff;
		CAV25M01_SendData(pdev, txData, 5);

	} else if (STOREorRECALL == RECALL)
	{
		txData[0] = OPCODE_READ;
		txData[4] = 0;
		/* Three independent reads in case addresses aren't consecutive in the future */
		/* Read X offset data from EEPROM */
		txData[1] = (uint8_t)(U6_XoffAddr >> 16);
		txData[2] = (uint8_t)(U6_XoffAddr >> 8);
		txData[3] = (uint8_t)(U6_XoffAddr);
		CAV25M01_SendReceive(pdev, txData, rxData, 5);
		*Xoff = rxData[4];
		/* Read Y offset data from EEPROM */
		txData[1] = (uint8_t)(U6_YoffAddr >> 16);
		txData[2] = (uint8_t)(U6_YoffAddr >> 8);
		txData[3] = (uint8_t)(U6_YoffAddr);
		CAV25M01_SendReceive(pdev, txData, rxData, 5);
		*Yoff = rxData[4];
		/* Read Z offset data from EEPROM */
		txData[1] = (uint8_t)(U6_ZoffAddr >> 16);
		txData[2] = (uint8_t)(U6_ZoffAddr >> 8);
		txData[3] = (uint8_t)(U6_ZoffAddr);
		CAV25M01_SendReceive(pdev, txData, rxData, 5);
		*Zoff = rxData[4];
	}
}

/*****************************************************************
 * @fn			- CAV25M01_enableWriteMode
 *
 * @brief		- This function enables writing data to memory, disabling any block protection in the process
 *
 * @param[in]	- Handle structure for CAV25M01
 *
 * @return		-
 *
 * @Note		- Block protection would have to be re-enabled after the necessary write function
 */
void CAV25M01_enableWriteMode(CAV25M01_Handle_t *pdev)
{
	uint8_t txData = OPCODE_WREN;
	uint8_t statusReg = 0;


	CAV25M01_DevReady(pdev);	// wait for device to be ready

	// 1. Check if any block protect is enabled
	statusReg = CAV25M01_ReadStatusReg(pdev);
	if (statusReg && BPmask)
	{
		// 2. If BP bits are set, set BP bits to 0
		statusReg &= ~BPmask;
		CAV25M01_WriteStatusReg(pdev,&statusReg);
		CAV25M01_DevReady(pdev);		// Wait for device to finish last write cycle
	}

	// 3.Send write enable command
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable WP
	GPIO_WriteToOutputPin(pdev->pnHOLD->pGPIOx, pdev->pnHOLD->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);	// Disable HOLD
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);	// Enable Chip select
	SPI_SendData(pdev->pSPIhandle->pSPI, &txData, 1);
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET); 	// Disable Chip select
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET); 	// Enable WP

	// 4.Check the WEL bit in the Status Register
	statusReg = CAV25M01_ReadStatusReg(pdev);
	// WEL bit was not set, something went wrong
	if ((statusReg & WELmask) != 0x2){
		//status = HAL_ERROR;
	}
}


/*****************************************************************
 * @fn			- CAV25M01_disableWriteMode
 *
 * @brief		- This function disables the WEL bit in the status register
 *
 * @param[in]	- Handle structure for CAV25M01
 *
 * @return		-
 *
 * @Note		- Could add block protection setting to this in the future
 */
void CAV25M01_disableWriteMode(CAV25M01_Handle_t *pdev)
{
	uint8_t txData = OPCODE_WRDI;
	uint8_t statusReg;

	CAV25M01_DevReady(pdev);			 			 // wait for device to be ready

	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);		// Disable WP
	GPIO_WriteToOutputPin(pdev->pnHOLD->pGPIOx, pdev->pnHOLD->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);	// Disable HOLD
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);	// Enable Chip select
	SPI_SendData(pdev->pSPIhandle->pSPI, &txData, 1);
	GPIO_WriteToOutputPin(pdev->pnCS0->pGPIOx, pdev->pnCS0->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET); 	// Disable Chip select
	GPIO_WriteToOutputPin(pdev->pnWP->pGPIOx, pdev->pnWP->GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET); 	// Enable WP

	// Re-enable block protect bits
	statusReg = CAV25M01_ReadStatusReg(pdev);
	statusReg |= BPmask;
	CAV25M01_WriteStatusReg(pdev, &statusReg);

	CAV25M01_DevReady(pdev);		// Wait for device to finish last write cycle

	// Check the WEL bit in the Status Register
	statusReg = CAV25M01_ReadStatusReg(pdev);
	// WEL bit still set, something went wrong
	if ((statusReg & WELmask) != 0){
		//status = HAL_ERROR;
	}
}

/*****************************************************************
 * @fn			- CAV25M01_IdentPageSwitch
 *
 * @brief		- This function switches the device between the Identification page and main memory
 *
 * @param[in]	- Handle structure for CAV25M01
 * @param[in]	- Enable or Disable identification page
 *
 * @return		-
 *
 * @Note		-
 */
void CAV25M01_IdentPageSwitch(CAV25M01_Handle_t *pdev, uint8_t ENorDI)
{
	uint8_t statusReg;

	statusReg = CAV25M01_ReadStatusReg(pdev);

	uint8_t IdentActive = statusReg & IPLmask;

	if (ENorDI == ENABLE)
	{
		if (!IdentActive)
		{
			statusReg |= IPLmask;
			CAV25M01_WriteStatusReg(pdev, &statusReg);
		}
	} else if (ENorDI == DISABLE)
	{
		// If EEPROM is set to access Identification page we need to swap it to main memory
		if (IdentActive)
		{
			statusReg &= ~IPLmask;
			CAV25M01_WriteStatusReg(pdev, &statusReg);
		}
	}
}
