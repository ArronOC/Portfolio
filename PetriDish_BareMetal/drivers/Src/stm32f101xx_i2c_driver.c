/*
 * stm32f101xx_i2c_driver.c
 *
 *  Created on: Dec 8, 2024
 *      Author: arron
 */


#include "stm32f101xx_i2c_driver.h"
#include "main.h"

/* =========Private functions========= */
static void I2C_GenerateSTARTCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t READorWRITE);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

/*****************************************************************
 * @fn			- I2C_GenerateSTARTCondition
 *
 * @brief		- This function generates a I2C START condition on the given I2C peripheral
 *
 * @param[in]	- I2C peripheral to generate START condition on
 *
 * @return		- none
 *
 * @Note		-
 */
static void I2C_GenerateSTARTCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/*****************************************************************
 * @fn			- I2C_GenerateSTOPCondition
 *
 * @brief		- This function generates a I2C STOP condition on the given I2C peripheral
 *
 * @param[in]	- I2C peripheral to generate STOP condition on
 *
 * @return		- none
 *
 * @Note		-
 */
void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/*****************************************************************
 * @fn			- I2C_ExecuteAddressPhase
 *
 * @brief		- This function executes the address phase on the given I2C peripheral
 *
 * @param[in]	- I2C peripheral to generate address phase on
 * @param[in]	- Address of slave device
 * @param[in]	- Write or Read operation
 *
 * @return		- none
 *
 * @Note		-
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t READorWRITE)
{
	SlaveAddr = SlaveAddr << 1;

	if (READorWRITE == READ)
	{
		SlaveAddr |= 1;			// Slave address + the R bit
	} else
	{
		SlaveAddr &= ~(1);		// Slave address + the W bit
	}

	pI2Cx->DR = SlaveAddr;
}

/*****************************************************************
 * @fn			- I2C_ClearADDRFlag
 *
 * @brief		- This function clears the ADDR flag in the status register for the given I2C peripheral
 *
 * @param[in]	- I2C peripheral to clear ADDR flag on
 *
 * @return		- none
 *
 * @Note		-
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR[0];
	dummyRead = pI2Cx->SR[1];

	// Noticed that it sometimes isn't cleared on the first attempt, so read it again
	if ( I2C_GetFlagStatus(pI2Cx, I2C_FLAG_ADDR) )
	{
		dummyRead = pI2Cx->SR[0];
		dummyRead = pI2Cx->SR[1];
	}

	(void) dummyRead;	// Suppress compiler warning message
}

/* ================================ */

/*****************************************************************
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given I2C peripheral
 *
 * @param[in]	- base address of the I2C peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
	} else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
	}
}

/*****************************************************************
 * @fn			- I2C_EnorDI
 *
 * @brief		- This function enables or disables the given I2C peripheral
 *
 * @pI2Cx		- base address of the I2C peripheral
 * @EnorDI		- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_EnorDI(I2C_RegDef_t *pI2Cx, uint8_t EnorDI)
{
	if (EnorDI)
	{
		pI2Cx->CR1 |= (0x1 << I2C_CR1_PE);	/* Enables the I2C peripheral (PE) */
	} else
	{
		/*
		 * Don't need to wait for communication to end as it will disable
		 * the peripheral automatically at the end of communication
		 */
		pI2Cx->CR1 &= ~(0x1 << I2C_CR1_PE);	/* Disable the I2C peripheral (PE) */
	}

}


/*****************************************************************
 * @fn			- I2C_Init
 *
 * @brief		- This function configures the I2C peripheral
 *
 * @pI2CxHandle	- handle of the I2C peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_Init(I2C_Handle_t *pI2CxHandle)
{
	uint32_t CCR = 0;
	uint8_t trise;

	// Enable the I2C clock source
	I2C_PeriClockControl(pI2CxHandle->pI2Cx, ENABLE);

	// Disable the I2C Peripheral so that all registers can be altered
	I2C_EnorDI(pI2CxHandle->pI2Cx, DISABLE);

	// Calculate value of PCLK1
	uint32_t PCLK1 = RCC_GetPCLK1_Value();

	// 1.Configure the mode to fast or standard
	if (pI2CxHandle->I2C_Config.I2C_SCLSpeed > I2C_SCL_SPEED_SM1K)	// Check if configuring for standard or fast mode
	{
		//Configure the mode to fast
		pI2CxHandle->pI2Cx->CCR |= (0x1 << I2C_CCR_FS);

		// Set the duty cycle (not needed in standard mode)
		if (pI2CxHandle->I2C_Config.I2C_DutyCycle == I2C_FM_DUTY_2)
		{
			pI2CxHandle->pI2Cx->CCR &= ~(0x1 << I2C_CCR_DUTY);	// Duty cycle = /2

			/*
			 * Th = CCR*Tpclk1
			 * Tl = 2*CCR*Tpclk1
			 * Th + Tl = 3*CCR*Tpclk1
			 * Th + Tl = 3*CCR/APB1freq
			 * CCR = (Th+Tl)APB1freq/3
			 * CCR = APB1freq/(3*SCLfreq)
			 */
			CCR = PCLK1 / (3 * pI2CxHandle->I2C_Config.I2C_SCLSpeed);
		} else
		{
			pI2CxHandle->pI2Cx->CCR |= (0x1 << I2C_CCR_DUTY);	// Duty cycle = 16/9

			/*
			 * Th = 9*CCR*Tpclk1
			 * Tl = 16*CCR*Tpclk1
			 * Th + Tl = 25*CCR*Tpclk1
			 * (1/SCLfreq) = 25*CCR/APB1freq
			 * SCLfreq = APB1freq/25*CCR
			 * CCR = APB1freq/(25*SCLfreq)
			 */
			CCR = PCLK1 / (25 * pI2CxHandle->I2C_Config.I2C_SCLSpeed);
		}
	} else
	{
		//Configure the mode to standard
		pI2CxHandle->pI2Cx->CCR &= ~(0x1 << I2C_CCR_FS);

		// CCR = APB1freq/(2*SCLfreq)
		CCR = PCLK1 / (2 * pI2CxHandle->I2C_Config.I2C_SCLSpeed);

	}
	// Configure the Clock Control register CCR value from previously calculated value
	CCR &= 0xFFF; // ensure only 12 bits are used
	pI2CxHandle->pI2Cx->CCR |= (CCR << I2C_CCR_CCR);


	// 2.Configure the speed of SCL
	pI2CxHandle->pI2Cx->CR2 &= ~(0x3F << I2C_CR2_FREQ);			 	// Reset CR2 FREQ[5:0]
	pI2CxHandle->pI2Cx->CR2 |= ((PCLK1 / 1000000U) & 0x3F) << I2C_CR2_FREQ;	// Set CR2 FREQ[5:0]

	//3.Configure device address (only used when slave)
	pI2CxHandle->pI2Cx->OAR[0] &= ~(0x3FF << I2C_OAR1_ADDR10BIT);	// Reset Own Address 1

	if (pI2CxHandle->I2C_Config.I2C_AddrMode == I2C_10BIT_SLAVE)
	{
		pI2CxHandle->pI2Cx->OAR[0] |= (0x1 << I2C_OAR1_ADDMODE); 									// Set to 10 bit address mode
		pI2CxHandle->pI2Cx->OAR[0] |= (pI2CxHandle->I2C_Config.I2C_OwnAddr1 << I2C_OAR1_ADDR10BIT);	// Set the 10 bit address
	} else
	{
		pI2CxHandle->pI2Cx->OAR[0] &= ~(0x1 << I2C_OAR1_ADDMODE); 									// Set to 7 bit address mode
		pI2CxHandle->pI2Cx->OAR[0] |= (pI2CxHandle->I2C_Config.I2C_OwnAddr1 << I2C_OAR1_ADDR7BIT);	// Set the 7 bit address

		// Configure the second device address
		if (pI2CxHandle->I2C_Config.I2C_DualAddrEn == I2C_DUAL_ADDRESS_EN)
		{
			pI2CxHandle->pI2Cx->OAR[1] |= (0x1 << I2C_OAR2_ENDUAL);		// enable dual 7 bit addressing
			pI2CxHandle->pI2Cx->OAR[1] &= ~(0x7F << I2C_OAR2_ADDR2);	// Reset Own Address 2
			pI2CxHandle->pI2Cx->OAR[1] |= (pI2CxHandle->I2C_Config.I2C_OwnAddr2 << I2C_OAR2_ADDR2); // Set the second 7 bit address
		} else
		{
			pI2CxHandle->pI2Cx->OAR[1] &= ~(0x1 << I2C_OAR2_ENDUAL);	// disable dual 7 bit addressing
		}
	}

	//4.Configure the rise time for I2C pins
	/*
	 * TRISE = (Fpclk1 * Trise(max)) + 1
	 * TRISE = FPCLK1 *
	 * Sm Trise(max) = 1000ns
	 * Fm Trise(max) = 300ns
	 */

	if (pI2CxHandle->I2C_Config.I2C_SCLSpeed > I2C_SCL_SPEED_SM1K)	// Check if configuring for standard or fast mode
	{
		// Fast mode
		trise = ((PCLK1 * 30U) / 100000000U) + 1;
	} else
	{
		// Standard mode
		trise = (PCLK1 / 1000000U) + 1;
	}
	pI2CxHandle->pI2Cx->TRISE = (trise & 0x3F);	// only 5 bits


	//5. Enable the peripheral
	I2C_EnorDI(pI2CxHandle->pI2Cx, ENABLE);

	//6. Configure the Acking mode (can only be done once peripheral enabled)
	if (pI2CxHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLED)
	{
		pI2CxHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);	// Enable Ack
	} else
	{
		pI2CxHandle->pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);	// Disable Ack
	}

	//6. Enable Error interrupts
	pI2CxHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
}

/*****************************************************************
 * @fn			- I2C_DeInit
 *
 * @brief		- This function resets the I2C peripheral
 *
 * @pI2Cx		- base address of the I2C peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_RST();
	} else
	{
		I2C2_RST();
	}
}

/*****************************************************************
 * @fn			- RCC_GetPCLK1_Value
 *
 * @brief		- This function calculates the APB1 clock frequency in Hz
 * 				  Which is used for configuring the I2C peripheral
 *
 *
 * @return		- APB1/PCLK1 clock frequency in Hz
 *
 * @Note		- none
 */
uint32_t RCC_GetPCLK1_Value(void)
{
	uint32_t pclk1, sysclk, hclk, clksrc, pllsrc, prediv1, pllmul,
	ahbpre, apb1pre;

	uint32_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
	uint32_t APB1_PreScaler[4] = {2,4,8,16};

	//1. Determine SYSCLK source
	clksrc = (RCC->CFGR & RCC_CFGR_SWS_MASK) >> RCC_CFGR_SWS;

	if (clksrc == 0) 	// HSI used as system clock
	{
		sysclk = HSI_CLK_FREQ; //8 MHz
	} else if (clksrc == 1) // HSE used as system clock
	{
		sysclk = HSE_OSC_FREQ; //defined in main.h
	} else	//PLL used as system clock
	{
		//Determine PLL source
		pllsrc = (RCC->CFGR >> RCC_CFGR_PLLSRC) & 0x1;
		if (pllsrc == 0) //HSI/2 used as PLL clock
		{
			sysclk = HSI_CLK_FREQ / 2;
		} else	// Clock from PREDIV1 selected as PLL clock
		{
			// this is HSE divided by either 1 or 2
			prediv1 = (RCC->CFGR >> RCC_CFGR_PLLXTPRE) & 0x1;
			if (prediv1 == 0)
			{
				// PREDIV1 does not divide HSE clock
				sysclk = HSE_OSC_FREQ;
			} else
			{
				// PREDIV1 divides HSE clock by 2
				sysclk = HSE_OSC_FREQ / 2;
			}

			// Take PLLMUL into account
			pllmul = (RCC->CFGR & RCC_CFGR_PLLMUL_MASK) >> RCC_CFGR_PLLMUL;
			if (pllmul == 2)
			{
				sysclk = sysclk * 4;	//PLL input clock x4
			} else if (pllmul == 3)
			{
				sysclk = sysclk * 5;	//PLL input clock x5
			} else if (pllmul == 4)
			{
				sysclk = sysclk * 6;	//PLL input clock x6
			} else if (pllmul == 5)
			{
				sysclk = sysclk * 7;	//PLL input clock x7
			} else if (pllmul == 6)
			{
				sysclk = sysclk * 8;	//PLL input clock x8
			} else if (pllmul == 7)
			{
				sysclk = sysclk * 9;	//PLL input clock x9
			} else if (pllmul == 13)
			{
				sysclk = (sysclk * 6) + (sysclk / 2);	//PLL input clock x6.5
			}
		}
	}

	//2. Take AHB Prescaler into account
	ahbpre = (RCC->CFGR & RCC_CFGR_HPRE_MASK) >> RCC_CFGR_HPRE;
	if (ahbpre < 8)
	{
		hclk = sysclk;
	} else
	{
		hclk = sysclk / AHB_PreScaler[ahbpre - 8];
	}

	//3. Take APB1 Prescaler into account
	apb1pre = (RCC->CFGR & RCC_CFGR_PPRE1_MASK) >> RCC_CFGR_PPRE1;
	if (apb1pre < 4)
	{
		pclk1 = hclk;
	} else
	{
		pclk1 = hclk / APB1_PreScaler[apb1pre - 4];
	}

	return pclk1;
}

/*****************************************************************
 * @fn			- I2C_MasterSendData
 *
 * @brief		- This function sends data to a slave device
 *
 * @param[in]	- pointer to the Handle of the I2C peripheral
 * @param[in]	- pointer to the data to transmit
 * @param[in]	- number of bytes to transmit
 * @param[in]	- address of the slave device
 * @param[in]	- maximum time to wait for the I2C operation in milliseconds
 *
 * @return		- I2C error code
 *
 * @Note		-
 */
I2C_ERROR_CODE  I2C_MasterSendData(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint32_t TIMEOUT_MS)
{
	uint32_t TIMEDOUT = 0;
	SYST_STARTorSTOP(ENABLE);

	//1. Generate the START condition
	I2C_GenerateSTARTCondition(pI2CxHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//	 Note: Until SB is cleared SCL will be stretched (pulled low)
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_SB)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//3. Send the address of the slave with R/W bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CxHandle->pI2Cx, SlaveAddr, WRITE);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1 register
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_ADDR)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//5. Clear the ADDR flag according to its software sequence
	//	 Note: Until ADDR is cleared SCL will be stretched (pulled low)
	I2C_ClearADDRFlag(pI2CxHandle->pI2Cx);

	//6. Send the data until Len becomes 0
	while (Len > 0)
	{
		while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_TXE)) && (!TIMEDOUT) )	// wait until TXE is set
		{
			TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
		}
		if ( TIMEDOUT )	return USER_TIMEOUT;

		pI2CxHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len becomes zero wait for TXE=1 & BTF=1 before generating the STOP condition
	//	 Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//		  when BTF=1 SCL will be stretched (pulled low)
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_TXE)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_BTF)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//8. Generate STOP condition and master need not to wait for the completion of STOP condition.
	//   Note: generating STOP, automatically clears the BTF
	I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);

	return I2C_NO_ERROR;
}


/*****************************************************************
 * @fn			- I2C_MasterReceiveData
 *
 * @brief		- This function reads data from a slave device
 *
 * @param[in]	- pointer to the Handle of the I2C peripheral
 * @param[in]	- pointer to store the read data too
 * @param[in]	- number of bytes to read
 * @param[in]	- address of the slave device
 * @param[in]	- maximum time to wait for the I2C operation in milliseconds
 *
 * @return		- I2C error code
 *
 * @Note		-
 */
I2C_ERROR_CODE I2C_MasterReceiveData(I2C_Handle_t *pI2CxHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint32_t TIMEOUT_MS)
{
	uint32_t TIMEDOUT = 0;
	SYST_STARTorSTOP(ENABLE);

	//1. Generate the START condition
	I2C_GenerateSTARTCondition(pI2CxHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//	 Note: Until SB is cleared SCL will be stretched (pulled low)
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_SB)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//3. Send the address of the slave with R/W bit set to r(1) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CxHandle->pI2Cx, SlaveAddr, READ);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1 register
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_ADDR)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//5. Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CxHandle->pI2Cx);

	//6. For single byte reception disable ACK and issue STOP byte
	if (Len == 1)
	{
		pI2CxHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);		// Disable ACK
		I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);		// Generate STOP condition
	} else
	{
		pI2CxHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);		// Enable ACK
	}

	//7. Read the data until Len becomes 0
	//   when the second last byte is in the DR, Disable ACK & issue STOP
	//   so it automatically happens after final byte transfer
	while (Len > 0)
	{
		while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_RXNE)) && (!TIMEDOUT) )	// wait until RXNE is set
		{
			TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
		}
		if ( TIMEDOUT )	return USER_TIMEOUT;

		if (Len == 2)
		{
			pI2CxHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);	// Disable ACK
			I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);	// Generate STOP condition
		}
		*pRxBuffer = pI2CxHandle->pI2Cx->DR;
		pRxBuffer++;
		Len--;
	}

	//8. Re-enable ACKing if it was enabled in Init
	if (pI2CxHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLED)
	{
		pI2CxHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}

	return I2C_NO_ERROR;
}

/*****************************************************************
 * @fn			- I2C_Mem_MasterSendData
 *
 * @brief		- This function sends data to a slave device's given internal address
 *
 * @param[in]	- pointer to the Handle of the I2C peripheral
 * @param[in]	- pointer to the data to transmit
 * @param[in]	- number of bytes to transmit
 * @param[in]	- address of the slave device
 * @param[in]	- slave devices internal address to write to
 * @param[in]	- maximum time to wait for the I2C operation in milliseconds
 *
 * @return		- I2C error code
 *
 * @Note		-
 */
I2C_ERROR_CODE  I2C_Mem_MasterSendData(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SlaveRegister, uint32_t TIMEOUT_MS)
{
	uint32_t TIMEDOUT = 0;
	SYST_STARTorSTOP(ENABLE);

	//1. Generate the START condition
	I2C_GenerateSTARTCondition(pI2CxHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//	 Note: Until SB is cleared SCL will be stretched (pulled low)
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_SB)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//3. Send the address of the slave with R/W bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CxHandle->pI2Cx, SlaveAddr, WRITE);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1 register
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_ADDR)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//5. Clear the ADDR flag according to its software sequence
	//	 Note: Until ADDR is cleared SCL will be stretched (pulled low)
	I2C_ClearADDRFlag(pI2CxHandle->pI2Cx);

	//6. Send the Slave devices register address to which data shall be written to
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_TXE)) )	// wait until TXE is set
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	pI2CxHandle->pI2Cx->DR = SlaveRegister;

	//7. Send the data until Len becomes 0
	while (Len > 0)
	{
		while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_TXE)) && (!TIMEDOUT) )	// wait until TXE is set
		{
			TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
		}
		if ( TIMEDOUT )	return USER_TIMEOUT;

		pI2CxHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//8. When Len becomes zero wait for TXE=1 & BTF=1 before generating the STOP condition
	//	 Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//		  when BTF=1 SCL will be stretched (pulled low)
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_TXE)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_BTF)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//9. Generate STOP condition and master need not to wait for the completion of STOP condition.
	//   Note: generating STOP, automatically clears the BTF
	I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);

	SYST_STARTorSTOP(DISABLE);

	return I2C_NO_ERROR;
}

/*****************************************************************
 * @fn			- I2C_Mem_MasterReceiveData
 *
 * @brief		- This function reads data from a slave device's given register
 *
 * @param[in]	- pointer to the Handle of the I2C peripheral
 * @param[in]	- pointer to store the read data too
 * @param[in]	- number of bytes to read
 * @param[in]	- address of the slave device
 * @param[in]	- slave devices internal address to read from
 * @param[in]	- maximum time to wait for the I2C operation in milliseconds
 *
 * @return		- I2C error code
 *
 * @Note		-
 */
I2C_ERROR_CODE I2C_Mem_MasterReceiveData(I2C_Handle_t *pI2CxHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SlaveRegister, uint32_t TIMEOUT_MS)
{
	uint32_t TIMEDOUT = 0;
	SYST_STARTorSTOP(ENABLE);

	// This line somehow makes it work every time?
	//uint32_t temp = ELAPSED_MS();

	//1. Generate the START condition
	I2C_GenerateSTARTCondition(pI2CxHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//	 Note: Until SB is cleared SCL will be stretched (pulled low)
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_SB)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//3. Send the address of the slave with R/W bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CxHandle->pI2Cx, SlaveAddr, WRITE);

	//4. Confirm that address phase is completed by checking the ADDR flag 	while( ! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_ADDR) );
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_ADDR)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//5. Clear the ADDR flag according to its software sequence
	//	 Note: Until ADDR is cleared SCL will be stretched (pulled low)
	I2C_ClearADDRFlag(pI2CxHandle->pI2Cx);

	//6. Send the Slave devices register address to which data shall be written to
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_TXE)) && (!TIMEDOUT) )	// wait until TXE is set
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	pI2CxHandle->pI2Cx->DR = SlaveRegister;

	//7. Generate the RE-START condition
	I2C_GenerateSTARTCondition(pI2CxHandle->pI2Cx);

	//8. Confirm that RE-START generation is completed by checking the SB flag in the SR1
	//	 Note: Until SB is cleared SCL will be stretched (pulled low)
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_SB)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//9. Send the address of the slave with R/W bit set to r(1) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CxHandle->pI2Cx, SlaveAddr, READ);

	//10. Confirm that address phase is completed by checking the ADDR flag in SR1 register
	while( (! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_ADDR)) && (!TIMEDOUT) )
	{
		TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
	}
	if ( TIMEDOUT )	return USER_TIMEOUT;

	//11. Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CxHandle->pI2Cx);

	//12. For single byte reception disable ACK and issue STOP byte
	if (Len == 1)
	{
		pI2CxHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);		// Disable ACK
		I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);		// Generate STOP condition
	} else
	{
		pI2CxHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);		// Enable ACK
	}

	//13. Read the data until Len becomes 0
	//   when the second last byte is in the DR, Disable ACK & issue STOP
	//   so it automatically happens after final byte transfer
	while (Len > 0)
	{
		while( ! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_FLAG_RXNE) && (!TIMEDOUT) )	// wait until RXNE is set
		{
			TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);
		}
		if ( TIMEDOUT )	return USER_TIMEOUT;

		if (Len == 2)
		{
			pI2CxHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);	// Disable ACK
			I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);	// Generate STOP condition
		}

		*pRxBuffer = pI2CxHandle->pI2Cx->DR;
		pRxBuffer++;
		Len--;
	}

	//14. Re-enable ACKing if it was enabled in Init
	if (pI2CxHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLED)
	{
		pI2CxHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}

	SYST_STARTorSTOP(DISABLE);
	return I2C_NO_ERROR;
}

/*****************************************************************
 * @fn			- I2C_GetFlagStatus
 *
 * @brief		- This function checks if the given flag is set
 *
 * @param[in]	- pointer to the Handle of the I2C peripheral
 * @param[in]	- FLAG to check
 *
 * @return		- whether flag is set or not
 *
 * @Note		- Doesn't check flags in SR2
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR[0] & FlagName)
	{
		return FLAG_SET;
	} else
	{
		return FLAG_RESET;
	}
}


/*****************************************************************
 * @fn			- I2C_ERR_IRQHandling
 *
 * @brief		-
 *
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CxHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CxHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CxHandle->pI2Cx->SR[0]) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the bus error flag
		pI2CxHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CxHandle,I2C_BUS_ERROR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CxHandle->pI2Cx->SR[0]) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CxHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CxHandle,I2C_ARLO_ERROR);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CxHandle->pI2Cx->SR[0]) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CxHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CxHandle,I2C_ACK_ERROR);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CxHandle->pI2Cx->SR[0]) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CxHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CxHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CxHandle->pI2Cx->SR[0]) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CxHandle->pI2Cx->SR[0] &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CxHandle,I2C_TIMEOUT_ERROR);
	}
}

///*****************************************************************
// * @fn			- I2C_MasterSendDataIT
// *
// * @brief		-
// *
// * @param[in]	-
// *
// * @return		-
// *
// * @Note		-
// */
//uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
//{
//
//	uint8_t busystate = pI2CxHandle->TxRxState;
//
//	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
//	{
//		pI2CxHandle->pTxBuffer = pTxBuffer;
//		pI2CxHandle->TxLen = Len;
//		pI2CxHandle->TxRxState = I2C_BUSY_IN_TX;
//		pI2CxHandle->DevAddr = SlaveAddr;
//		pI2CxHandle->Sr = Sr;
//
//		//Implement code to Generate START Condition
//		I2C_GenerateSTARTCondition(pI2CxHandle->pI2Cx);
//
//		//Implement the code to enable ITBUFEN Control Bit
//		pI2CxHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
//
//		//Implement the code to enable ITEVFEN Control Bit
//		pI2CxHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
//
//		//Implement the code to enable ITERREN Control Bit
//		pI2CxHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
//	}
//
//	return busystate;
//}
//
///*****************************************************************
// * @fn			- I2C_MasterReceiveDataIT
// *
// * @brief		-
// *
// * @param[in]	-
// *
// * @return		-
// *
// * @Note		-
// */
//uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CxHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
//{
//	uint8_t busystate = pI2CxHandle->TxRxState;
//
//	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
//	{
//		pI2CxHandle->pRxBuffer = pRxBuffer;
//		pI2CxHandle->RxLen = Len;
//		pI2CxHandle->TxRxState = I2C_BUSY_IN_RX;
//		pI2CxHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
//		pI2CxHandle->DevAddr = SlaveAddr;
//		pI2CxHandle->Sr = Sr;
//
//		//Implement code to Generate START Condition
//		I2C_GenerateSTARTCondition(pI2CxHandle->pI2Cx);
//
//		//Implement the code to enable ITBUFEN Control Bit
//		pI2CxHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
//
//		//Implement the code to enable ITEVFEN Control Bit
//		pI2CxHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
//
//		//Implement the code to enable ITERREN Control Bit
//		pI2CxHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
//	}
//
//	return busystate;
//}
//
//
///*****************************************************************
// * @fn			- I2C_EV_IRQHandling
// *
// * @brief		-
// *
// * @param[in]	-
// *
// * @return		-
// *
// * @Note		-
// */
//void I2C_EV_IRQHandling(I2C_Handle_t *pI2CxHandle)
//{
//	//Interrupt handling for both master and slave mode of a device
//	uint32_t ITEVTEN, ITBUFEN, StatusReg;
//
//	ITEVTEN = pI2CxHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
//	ITBUFEN = pI2CxHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
//	StatusReg = pI2CxHandle->pI2Cx->SR[0];
//
//	//1. Handle For interrupt generated by SB event
//	//	Note : SB flag is only applicable in Master mode
//	if (ITEVTEN && (StatusReg & I2C_FLAG_SB))
//	{
//		//The interrupt is generated because of SB event
//		//This block will not be executed in slave mode because for slave SB is always zero
//		//In this block the address phase needs to be executed
//		if ( pI2CxHandle->TxRxState == I2C_BUSY_IN_TX )
//		{
//			I2C_ExecuteAddressPhase(pI2CxHandle->pI2Cx, pI2CxHandle->DevAddr, WRITE);
//		} else if ( pI2CxHandle->TxRxState == I2C_BUSY_IN_RX )
//		{
//			I2C_ExecuteAddressPhase(pI2CxHandle->pI2Cx, pI2CxHandle->DevAddr, READ);
//		}
//
//	}
//
//	//2. Handle For interrupt generated by ADDR event
//	//Note : When master mode : Address is sent
//	//		 When Slave mode  : Address matched with own address
//	if (ITEVTEN && (StatusReg & I2C_FLAG_ADDR))
//	{
//		//Interrupt is generated because of ADDR event
//		I2C_ClearADDRFlag(pI2CxHandle->pI2Cx);
//	}
//
//	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
//	if (ITEVTEN && (StatusReg & I2C_FLAG_BTF))
//	{
//		//Interrupt is generated because of BTF event
//		if (pI2CxHandle->TxRxState == I2C_BUSY_IN_TX)
//		{
//			//make sure that TXE is also set.
//			if ( StatusReg & I2C_FLAG_TXE )
//			{
//				//BTF, TXE = 1, close transmission
//				if (pI2CxHandle->TxLen == 0)
//				{
//					//1. Generate the STOP condition if no repeated start.
//					if (pI2CxHandle->Sr == I2C_SR_DISABLE)
//					{
//						I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);
//					}
//
//					//2. Reset all member elements of the handle structure.
//					I2C_CloseSendData();
//
//					//3. Notify the application about transmission complete.
//					I2C_ApplicationEventCallback(pI2CxHandle, I2C_EV_TX_CMPLT);
//				}
//
//			}
//
//		} else if (pI2CxHandle->TxRxState == I2C_BUSY_IN_RX)
//		{
//			// Nothing to do
//		}
//
//	}
//
//	//4. Handle For interrupt generated by STOPF event
//	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
//	if (ITEVTEN && (StatusReg & I2C_FLAG_STOPF))
//	{
//		//Interrupt is generated because of STOPF event
//		//Clear the STOPF (i.e 1) read SR1 2) Write to CR1 )
//		//reading SR1 is already done to get to this stage
//		pI2CxHandle->pI2Cx->CR1 |= 0x0000;
//
//		//Notify the application that STOP is detected
//		I2C_ApplicationEventCallback(pI2CxHandle, I2C_EV_STOP);
//	}
//
//	//5. Handle For interrupt generated by TXE event
//	if (ITEVTEN && ITBUFEN && (StatusReg & I2C_FLAG_TXE))
//	{
//		//Interrupt is generated because of TXE event
//		//We have to do the data transmission
//		if ( pI2CxHandle->TxRxState == I2C_BUSY_IN_TX )
//		{
//			//Check for device is in Master mode
//			if ( pI2CxHandle->pI2Cx->SR[1] & (1 << I2C_SR2_MSL) )
//			{
//				if ( pI2CxHandle->TxLen > 0 )
//				{
//					//1. Load the data in to DR
//					pI2CxHandle->pI2Cx->DR = *(pI2CxHandle->pTxBuffer);
//
//					//2. Decrement the TxLen
//					pI2CxHandle->TxLen--;
//
//					//3. Increment the buffer address
//					pI2CxHandle->pTxBuffer++;
//				}
//			}
//		}
//	}
//
//	//6. Handle For interrupt generated by RXNE event
//	if (ITEVTEN && ITBUFEN && (StatusReg & I2C_FLAG_RXNE))
//	{
//		//Interrupt is generated because of RXNE event
//		if ( pI2CxHandle->TxRxState == I2C_BUSY_IN_RX )
//		{
//			//We have to do the data reception
//			if ( pI2CxHandle->RxSize == 1 )
//			{
//
//			}
//
//			if ( pI2CxHandle->RxSize > 1 )
//			{
//
//			}
//		}
//	}
//}
