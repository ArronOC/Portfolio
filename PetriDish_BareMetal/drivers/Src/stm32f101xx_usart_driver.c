/*
 * stm32f101xx_usart_driver.c
 *
 *  Created on: Jan 18, 2025
 *      Author: arron
 */

#include "stm32f101xx_usart_driver.h"


/*****************************************************************
 * @fn			- USART_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given USART peripheral
 *
 * @param[in]	- base address of the USART peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
	} else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
	}
}

/*****************************************************************
 * @fn			- USART_EnorDI
 *
 * @brief		- This function enables or disables the given USART peripheral
 *
 * @pI2Cx		- base address of the USART peripheral
 * @EnorDI		- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_ENorDI(USART_RegDef_t *pUSARTx, uint8_t EnorDI)
{
	if (EnorDI)
	{
		pUSARTx->CR1 |= (0x1 << USART_CR1_POS_UE);
	} else
	{
		pUSARTx->CR1 &= ~(0x1 << USART_CR1_POS_UE);
	}
}

/*****************************************************************
 * @fn			- USART_GetFlagStatus
 *
 * @brief		- This function checks if the given flag is set
 *
 * @param[in]	- pointer to the Handle of the USART peripheral
 * @param[in]	- FLAG to check
 *
 * @return		- whether flag is set or not
 *
 * @Note		-
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	} else
	{
		return FLAG_RESET;
	}
}

/*****************************************************************
 * @fn			- USART_ClearFlag
 *
 * @brief		- This function clears the given flag in the SR
 *
 * @pUSARTx		- base address of the USART peripheral
 * @StatusFlagName		- FLAG to clear
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	uint32_t tempReg;
	if (StatusFlagName == USART_FLAG_CTS)
	{
		pUSARTx->SR &= ~(1 << USART_SR_POS_CTS);	// CTS Cleared by writing 0 to flag

	} else if (StatusFlagName == USART_FLAG_LBD)
	{
		pUSARTx->SR &= ~(1 << USART_SR_POS_LBD);	// LBD Cleared by writing 0 to flag

	} else if (StatusFlagName == USART_FLAG_TC)
	{
		pUSARTx->SR &= ~(1 << USART_SR_POS_TC);		// TC Cleared by writing 0 to flag

	} else if (StatusFlagName == USART_FLAG_RXNE)
	{
		pUSARTx->SR &= ~(1 << USART_SR_POS_RXNE);	// RXNE Cleared by writing 0 to flag

	} else if (StatusFlagName == USART_FLAG_TXE)
	{
		pUSARTx->DR = 0x0;							// TXE Cleared by writing to the DR

	} else if (StatusFlagName == USART_FLAG_IDLE)
	{
		tempReg = pUSARTx->SR;						// IDLE Cleared by reading SR then reading DR
		tempReg = pUSARTx->DR;

	} else if (StatusFlagName == USART_FLAG_ORE)
	{
		tempReg = pUSARTx->SR;						// ORE Cleared by reading SR then reading DR
		tempReg = pUSARTx->DR;

	} else if (StatusFlagName == USART_FLAG_NE)
	{
		tempReg = pUSARTx->SR;						// NE Cleared by reading SR then reading DR
		tempReg = pUSARTx->DR;

	} else if (StatusFlagName == USART_FLAG_FE)
	{
		tempReg = pUSARTx->SR;						// FE Cleared by reading SR then reading DR
		tempReg = pUSARTx->DR;

	} else if (StatusFlagName == USART_FLAG_PE)
	{
		tempReg = pUSARTx->SR;						// PE Cleared by reading SR then reading DR
		tempReg = pUSARTx->DR;

	}
	(void) tempReg;
}


/*****************************************************************
 * @fn			- USART_Init
 *
 * @brief		- This function initialises the UART interface with the configuration settings stored in the handle
 *
 * @param[in]	- pointer to the Handle of the USART peripheral
 *
 * @return		-
 *
 * @Note		-
 */
void USART_Init(USART_Handle_t *pUSARTxHandle)
{
	//Temporary variable
	uint32_t tempreg = 0;

	//Enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTxHandle->pUSARTx, ENABLE);

	/******************************** Configuration of CR1******************************************/
	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTxHandle->USART_Config.Mode == USART_MODE_ONLY_RX)
	{
		//Enable the Receiver bit field
		tempreg |= (1 << USART_CR1_POS_RE);
	}else if (pUSARTxHandle->USART_Config.Mode == USART_MODE_ONLY_TX)
	{
		//Enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_POS_TE );

	}else if (pUSARTxHandle->USART_Config.Mode == USART_MODE_TXRX)
	{
		//Enable both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_POS_TE) | ( 1 << USART_CR1_POS_RE) );
	}

	//Configure the Word length configuration item
	tempreg |= ( pUSARTxHandle->USART_Config.WordLength << USART_CR1_POS_M );


	//Configuration of parity control bit fields
	if ( pUSARTxHandle->USART_Config.ParityControl == USART_PARITY_EVEN)
	{
		//Enable the parity control
		tempreg |= ( 1 << USART_CR1_POS_PCE);

		//Even parity is selected by default

	}else if (pUSARTxHandle->USART_Config.ParityControl == USART_PARITY_ODD )
	{
		//Enable the parity control
		tempreg |= ( 1 << USART_CR1_POS_PCE);

		//Enable ODD parity
		tempreg |= ( 1 << USART_CR1_POS_PS);

	}

	//Program the CR1 register
	pUSARTxHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg = 0;

	//Configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTxHandle->USART_Config.NumStopBits << USART_CR2_POS_STOP;

	//Program the CR2 register
	pUSARTxHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg = 0;

	//Configuration of USART hardware flow control
	if ( pUSARTxHandle->USART_Config.HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_POS_CTSE);


	}else if (pUSARTxHandle->USART_Config.HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_POS_RTSE);

	}else if (pUSARTxHandle->USART_Config.HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( (1 << USART_CR3_POS_CTSE) | (1 << USART_CR3_POS_RTSE) );
	}


	pUSARTxHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Set the BRR register
	USART_SetBaudRate(pUSARTxHandle);

	// Enable the USART peripheral
	USART_ENorDI(pUSARTxHandle->pUSARTx, ENABLE);
}

/*********************************************************************
 * @fn      		  - USART_DeInit
 *
 * @brief             - This function disables the USART peripheral
 *
 * @param[in]         - Pointer to the base address of the USART peripheral
 *
 * @return            -
 *
 * @Note              -

 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{

	USART_ENorDI(pUSARTx, DISABLE);
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - This function sets the BRR register for the requested baud rate
 *
 * @param[in]         - Pointer to the Handle of the USART peripheral
 *
 * @return            -
 *
 * @Note              -

 */
void USART_SetBaudRate(USART_Handle_t *pUSARTxHandle)
{
	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	usartdiv = ( 25 * pUSARTxHandle->USART_Config.APBClock / (4 * pUSARTxHandle->USART_Config.BaudRate) );

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << USART_BRR_POS_DIVMantissa;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part << USART_BRR_POS_DIVfraction;

	//copy the value of tempreg in to BRR register
	pUSARTxHandle->pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - This function sends data over USART from the STM32
 *
 * @param[in]         - Pointer to the Handle of the USART peripheral
 * @param[in]         - Pointer to the data to send
 * @param[in]         - Number of bytes to send
 *
 * @return            -
 *
 * @Note              - This function sends the data in blocking mode

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_NONE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Increment the buffer address
			pTxBuffer++;
		}
	}

	//Wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - This function receives data over USART to the STM32
 *
 * @param[in]         - Pointer to the Handle of the USART peripheral
 * @param[in]         - Pointer to where to store the read data
 * @param[in]         - Number of bytes to read
 *
 * @return            -
 *
 * @Note              - This function reads the data in blocking mode

 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Wait until RXNE flag is set in the SR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_NONE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_NONE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             - This function sends data over USART using interrupt mode
 *
 * @param[in]         - Pointer to the Handle of the USART peripheral
 * @param[in]         - Pointer of data to send
 * @param[in]         - Number of bytes to send
 *
 * @return            -
 *
 * @Note              - This function sends the data in interrupt mode

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_POS_TXEIE);


		//Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_POS_TCIE);


	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		pUSARTHandle->RxLFRcvd = NOT_RECEIVED;
		pUSARTHandle->RxCRRcvd = NOT_RECEIVED;
		(void)pUSARTHandle->pUSARTx->DR;

		//Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_POS_RXNEIE);

	}

	return rxstate;

}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

	/*************************Check for TC flag ********************************************/

	//Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_POS_TC);

	//Check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_POS_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_POS_TC);

				//Clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_POS_TCIE);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*************************Check for TXE flag ********************************************/

	//Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_POS_TXE);

	//Check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_POS_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_NONE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_POS_TXEIE);
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_POS_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_POS_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of RXNE
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_NONE)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						// Check if new line character received (\r = 13, \n = 10)
						if (*pUSARTHandle->pRxBuffer == 13)
						{
							pUSARTHandle->RxCRRcvd = RECEIVED;
						} else if (*pUSARTHandle->pRxBuffer == 10)
						{
							pUSARTHandle->RxLFRcvd = RECEIVED;
						}


						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						// Check if new line character received (\r = 13, \n = 10)
						// Check if new line character received (\r = 13, \n = 10)
						if (*pUSARTHandle->pRxBuffer == 13)
						{
							pUSARTHandle->RxCRRcvd = RECEIVED;
						} else if (*pUSARTHandle->pRxBuffer == 10)
						{
							pUSARTHandle->RxLFRcvd = RECEIVED;
						}
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_NONE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);


					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					// Check if new line character received (\r = 13, \n = 10)
					// Check if new line character received (\r = 13, \n = 10)
					if (*pUSARTHandle->pRxBuffer == 13)
					{
						pUSARTHandle->RxCRRcvd = RECEIVED;
					} else if (*pUSARTHandle->pRxBuffer == 10)
					{
						pUSARTHandle->RxLFRcvd = RECEIVED;
					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen-=1;
				}


			}
			// if RxBuffer is full set rx state to complete & disable RXNE interrupt
			if(! pUSARTHandle->RxLen)
			{
				//disable the RXNE
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_POS_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			} else
			{
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_RCVD);
			}
		}
	}


	/*************************Check for CTS flag ********************************************/

	//Check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_POS_CTS);

	//Check the state of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_POS_CTSE);

	//Check the state of CTSIE bit in CR3
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_POS_CTSIE);


	if(temp1  && temp2  && temp3)
	{
		//Clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_POS_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

	/*************************Check for IDLE detection flag ********************************************/

	//Check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_POS_IDLE);

	//Check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_POS_IDLEIE);


	if(temp1 && temp2)
	{
		//Clear the IDLE flag.
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_POS_IDLE);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

	/*************************Check for Overrun detection flag ********************************************/

	//Check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_POS_ORE;

	//Check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_POS_RXNEIE;


	if(temp1  && temp2 )
	{
		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



	/*************************Check for Error Flag ********************************************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_POS_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_POS_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_POS_NE) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_POS_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}


}
