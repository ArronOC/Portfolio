/*
 * LM75B_TempSensor.c
 *
 *  Created on: Jun 22, 2024
 *      Author: arron
 */

#include "LM75B_TempSensor.h"

/*****************************************************************
 * @fn			- LM75B_POR
 *
 * @brief		- This function configures the Conf, TOS & Thyst registers on the LM75B
 *
 * @param[in]	- Handle structure for LM75B
 * @param[in]	- Maximum time to wait for an I2C operation
 *  *
 * @return		- I2C_ERROR_CODE
 *
 * @Note		-
 */
I2C_ERROR_CODE LM75B_POR(LM75B_Handle_t *dev)
{
	I2C_ERROR_CODE status = I2C_NO_ERROR;

	uint8_t RxBuffer[2],
			TxBuffer[2];

	/*
	 * 1. Check the Configuration register is the expected value
	 * Set it to the expected value if it isn't
	 */
	TxBuffer[0] = LM75B_RegConfig;
	//I2C_MasterSendData(dev->hi2c,TxBuffer,1,dev->Addr);
	//I2C_MasterReceiveData(dev->hi2c,RxBuffer,1,dev->Addr);
	status = I2C_Mem_MasterReceiveData(dev->i2cHandle, RxBuffer, 1, dev->Addr, LM75B_RegConfig, I2C_MAX_TIMEOUT_MS);
	if (RxBuffer[0] != LM75B_ConfigByte)
	{
		TxBuffer[0] = LM75B_ConfigByte;
		I2C_Mem_MasterSendData(dev->i2cHandle, TxBuffer, 1, dev->Addr, LM75B_RegConfig, I2C_MAX_TIMEOUT_MS);
	}

	/*
	 * 2. Check the TOS register is the expected value
	 * Set it to the expected value if it isn't
	 */
	status = I2C_Mem_MasterReceiveData(dev->i2cHandle, RxBuffer, 2, dev->Addr, LM75B_RegTOS, I2C_MAX_TIMEOUT_MS);
	if ((RxBuffer[0] != LM75B_tosByteMSB) || (RxBuffer[1] != LM75B_tosByteLSB))
	{
		TxBuffer[0] = LM75B_tosByteMSB;
		TxBuffer[1] = LM75B_tosByteLSB;
		I2C_Mem_MasterSendData(dev->i2cHandle, TxBuffer, 2, dev->Addr, LM75B_RegTOS, I2C_MAX_TIMEOUT_MS);
	}

	/*
	 * 3. Check the THyst register is the expected value
	 * Set it to the expected value if it isn't
	 */
	status = I2C_Mem_MasterReceiveData(dev->i2cHandle, RxBuffer, 2, dev->Addr, LM75B_RegThyst, I2C_MAX_TIMEOUT_MS);
	if ((RxBuffer[0] != LM75B_thystByteMSB) || (RxBuffer[1] != LM75B_thystByteLSB))
	{
		TxBuffer[0] = LM75B_thystByteMSB;
		TxBuffer[1] = LM75B_thystByteLSB;
		I2C_Mem_MasterSendData(dev->i2cHandle, TxBuffer, 2, dev->Addr, LM75B_RegThyst, I2C_MAX_TIMEOUT_MS);
	}

	return status;
}

/*****************************************************************
 * @fn			- LM75B_ReadTemp
 *
 * @brief		- This function returns a temperature reading from the LM75B
 *
 * @param[in]	- Handle structure for LM75B
 * @param[in]	- Maximum time to wait for an I2C operation
 *  *
 * @return		- temperature in celsius
 *
 * @Note		-
 */
float LM75B_ReadTemp(LM75B_Handle_t *dev)
{
	uint8_t RxBuffer[2] = {0,0};
	uint16_t TempHex;
	I2C_ERROR_CODE status;
	float TempC = 0.0f;

	status = I2C_Mem_MasterReceiveData(dev->i2cHandle, RxBuffer, 2, dev->Addr, LM75B_RegTemp, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) I2C_ApplicationEventCallback(dev->i2cHandle, status);
	/*
	 * Process for temperature calculation
	 * right bit shift of 4 (removes bits 4:0 of LSB)
	 * if D10 = 0, temperature is positive
	 * 		temperature = decimal value * 0.125 C
	 * if D10 = 1, temperature is negative
	 * 		temperature = -(two's complement of data) * 0.125 C
	 */
	TempHex = ((RxBuffer[0] << 8) + RxBuffer[1]);

	if (!(TempHex & 0x8000))// if positive temperature
	{
		TempC = (TempHex >> 5) * 0.125f;
	}
	else // if negative temperature take two's complement first
	{
		TempHex--; // subtract 1
		TempHex = !TempHex; // take the complement
		TempC = (TempHex >> 5) * -0.125f;
	}
	return TempC;
}
