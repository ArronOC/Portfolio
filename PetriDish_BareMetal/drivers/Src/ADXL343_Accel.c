/*
 * ADXL343_Accel.c
 *
 *  Created on: Jun 23, 2024
 *      Author: arron
 */

#include "ADXL343_Accel.h"

/*****************************************************************
 * @fn			- ADXL343_Init
 *
 * @brief		- This function configures ADXL343 accelerometer
 * 				  to perform readings at a rate of 6.25Hz and generate
 * 				  and interrupt on INT1 when new data is ready
 *
 * @param[in]	- Handle structure for ADXL343
 *
 *  *
 * @return		- I2C_ERROR_CODE
 *
 * @Note		-
 */
I2C_ERROR_CODE ADXL343_Init(ADXL343_Handle_t *dev)
{
	I2C_ERROR_CODE status;
	uint8_t RxBuffer, TxBuffer;

	/*
	 * 1. Confirm we can communicate with the ADXL343 by reading the DEVICE ID
	 */
	status = I2C_Mem_MasterReceiveData(dev->i2cHandle, &RxBuffer, 1, dev->SlaveAddr, ADXL343_Reg_DevID, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error
	if (RxBuffer != ADXL343_DevID) return USER_INVALID_DEVICE;

	/*
	 * 2. Set the ADXL343 BW_Rate to 6.25Hz & LOW_POWER Disabled
	 */
	TxBuffer = 0x6;
	status = I2C_Mem_MasterSendData(dev->i2cHandle, &TxBuffer, 1, dev->SlaveAddr, ADXL343_Reg_BW_RATE, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	/*
	 * 3. Configure the DATA FORMAT register
	 * 	+/- 2g range
	 * 	right justified MSB mode with sign extension
	 * 	10 bit mode
	 * 	interrupts are active high
	 */
	TxBuffer = 0x0;
	status = I2C_Mem_MasterSendData(dev->i2cHandle, &TxBuffer, 1, dev->SlaveAddr, ADXL343_Reg_DATA_FORMAT, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	/*
	 * 4. Configure the INT MAP register
	 * 	DATA READY interrupt set to INT1
	 * 	all other interrupts to INT2
	 */
	TxBuffer = 0x7F;
	status = I2C_Mem_MasterSendData(dev->i2cHandle, &TxBuffer, 1, dev->SlaveAddr, ADXL343_Reg_INT_MAP, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	/*
	 * 5. Configure the INT ENABLE register
	 * 	Disable all interrupts (so that resets occur from a known state)
	 */
	status = ADXL343_DisableInterrupts(dev);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	/*
	 * 6. Configure the POWER CTL register
	 * 	Set the device into measure mode
	 */
	TxBuffer = 0x8;
	status = I2C_Mem_MasterSendData(dev->i2cHandle, &TxBuffer, 1, dev->SlaveAddr, ADXL343_Reg_POWER_CTL, I2C_MAX_TIMEOUT_MS);
	return status;
}

/*****************************************************************
 * @fn			- ADXL343_EnDataReadyInt
 *
 * @brief		- This function enables the DATA_READY interrupt
 *
 * @param[in]	- Handle structure for ADXL343
 *
 * @return		- I2C_ERROR_CODE
 *
 * @Note		-
 */
I2C_ERROR_CODE ADXL343_EnDataReadyInt(ADXL343_Handle_t *dev)
{
	uint8_t TxBuffer = 0x80;
	return I2C_Mem_MasterSendData(dev->i2cHandle, &TxBuffer, 1, dev->SlaveAddr, ADXL343_Reg_INT_ENABLE, I2C_MAX_TIMEOUT_MS);
}


/*****************************************************************
 * @fn			- ADXL343_DisableInterrupts
 *
 * @brief		- This function disables all interrupts
 *
 * @param[in]	- Handle structure for ADXL343
 *
 * @return		- I2C_ERROR_CODE
 *
 * @Note		-
 */
I2C_ERROR_CODE ADXL343_DisableInterrupts(ADXL343_Handle_t *dev)
{
	uint8_t TxBuffer = 0x0;
	return I2C_Mem_MasterSendData(dev->i2cHandle, &TxBuffer, 1, dev->SlaveAddr, ADXL343_Reg_INT_ENABLE, I2C_MAX_TIMEOUT_MS);
}


/*****************************************************************
 * @fn			- ADXL343_CalibrateFlat
 *
 * @brief		- This function calibrates the ADXL343 to give X=0g,Y=0g,Z=1g
 * 				  readings in the position it is in when it is calibrated
 * 				  these values are stored in the X/Y/Z Offset registers
 * 				  values are calculated over an average of 10 readings
 *
 * @param[in]	- Handle structure for ADXL343
 * @param[in]	- timeout in ms (used incase interrupts not triggered and prevents infinite loop)
 *
 *  *
 * @return		- I2C_ERROR_CODE
 *
 * @Note		- It is important that the PCB be laying flat, facing upwards and does not tilt
 * 				  whilst calibration is taking place. Ideally this function would have a timeout.
 */
I2C_ERROR_CODE ADXL343_CalibrateFlat(ADXL343_Handle_t *dev, GPIO_Handle_t *pInt1)
{
	I2C_ERROR_CODE status;
	uint8_t readings = 10, index = 0, reset = 0, TIMEDOUT = 0, RawData[6],  Xoff, Yoff, Zoff;
	float   Xo=0.0f, Yo=0.0f, Zo=0.0f;

	// 1. Reset offset registers to ensure valid calibration
	status = I2C_Mem_MasterSendData(dev->i2cHandle, &reset, 1, dev->SlaveAddr, ADXL343_Reg_OFSX, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	status = I2C_Mem_MasterSendData(dev->i2cHandle, &reset, 1, dev->SlaveAddr, ADXL343_Reg_OFSY, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	status = I2C_Mem_MasterSendData(dev->i2cHandle, &reset, 1, dev->SlaveAddr, ADXL343_Reg_OFSZ, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	// 2. Wait for number of specified readings to be taken.
	//SYST_STARTorSTOP(ENABLE);
	while ((index < readings) && !TIMEDOUT)
	{
		//TIMEDOUT = (ELAPSED_MS() > TIMEOUT_MS);

		if (dev->NewData == READY) // Check if Data ready
		{
			status = ADXL343_ReadData(dev, RawData, pInt1);
			if (status != I2C_NO_ERROR) return status; // break out of function if error
			ADXL343_CalcGForce(RawData, &dev->Xg, &dev->Yg, &dev->Zg);
			Xo = Xo + dev->Xg;
			Yo = Yo + dev->Yg;
			Zo = Zo + dev->Zg;
			index++;
		}
	}

//	if ( TIMEDOUT )
//	{
//		SYST_STARTorSTOP(DISABLE);
//		return I2C_TIMEOUT_ERROR;
//	}

	// 3. Average results
	Xo = Xo/readings;
	Yo = Yo/readings;
	Zo = 1 - (Zo/readings);

	// 4. Convert to uint8_t twos complement
	Xoff = -ADXL343_OffsetComplementCalc(Xo);
	Yoff = -ADXL343_OffsetComplementCalc(Yo);
	Zoff = ADXL343_OffsetComplementCalc(Zo);

	// 5. Write offset values to the accelerometer offset registers
	status = I2C_Mem_MasterSendData(dev->i2cHandle, &Xoff, 1, dev->SlaveAddr, ADXL343_Reg_OFSX, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	status = I2C_Mem_MasterSendData(dev->i2cHandle, &Yoff, 1, dev->SlaveAddr, ADXL343_Reg_OFSY, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	status = I2C_Mem_MasterSendData(dev->i2cHandle, &Zoff, 1, dev->SlaveAddr, ADXL343_Reg_OFSZ, I2C_MAX_TIMEOUT_MS);
	if (status != I2C_NO_ERROR) return status; // break out of function if error

	dev->Xoff = Xoff;
	dev->Yoff = Yoff;
	dev->Zoff = Zoff;

	return status;
}

/*****************************************************************
 * @fn			- ADXL343_ReadData
 *
 * @brief		- This functions reads the DATA X/Y/Z registers
 *
 * @param[in]	- Handle structure for ADXL343
 * @param[in]	- Pointer to where to store the read data
 * 				  (pointer should be at least 6 bytes long as there are 6 reads)
 *
 *
 * @return		- I2C_ERROR_CODE
 *
 * @Note		-
 */
I2C_ERROR_CODE ADXL343_ReadData(ADXL343_Handle_t *dev, uint8_t *pRawData, GPIO_Handle_t *pInt1)
{
	I2C_ERROR_CODE status;
	dev->NewData = nREADY;	// Clear interrupt flag
	// Keep reading data until ACC_INT1 goes low
	while ( GPIO_ReadFromInputPin(pInt1->pGPIOx, pInt1->GPIO_PinConfig.GPIO_PinNumber) )
	{
		status = I2C_Mem_MasterReceiveData(dev->i2cHandle, pRawData, 6, dev->SlaveAddr, ADXL343_Reg_DATAX0, I2C_MAX_TIMEOUT_MS);
		if (status != I2C_NO_ERROR) return status; // break out of function if error
	}
	return status;
}

/*****************************************************************
 * @fn			- ADXL343_CalcGForce
 *
 * @brief		- This functions converts the RAW data read from the ADXL343
 * 				  into a g force
 *
 * @param[in]	- RAW data for X/Y/Z (6 bytes long)
 * @param[in]	- Pointer to where to store the read data
 * 				  (pointer should be at least 6 bytes long as there are 6 reads)
 *
 *
 * @return		- I2C_ERROR_CODE
 *
 * @Note		-
 */
void ADXL343_CalcGForce(uint8_t *pRawData, float *pXg, float *pYg, float *pZg)
{
	int16_t dataX, dataY, dataZ;

	// 1. combine the RAW data MSB/LSB into 16 bit
	dataX = (((int16_t)pRawData[1] << 8) | pRawData[0]);
	dataY = (((int16_t)pRawData[3] << 8) | pRawData[2]);
	dataZ = (((int16_t)pRawData[5] << 8) | pRawData[4]);

	//2. Calculate g force
	if (dataX & 0x200)	// negative x acceleration, perform twos complement
	{
		*pXg = (((~dataX + 1) & 0x1FF) / ADXL343_scale_factor) * -1;
	}
	else				// positive x acceleration
	{
		*pXg = (dataX & 0x1FF) / ADXL343_scale_factor;
	}

	if (dataY & 0x200)	// negative y acceleration, perform twos complement
	{
		*pYg = (((~dataY + 1) & 0x1FF) / ADXL343_scale_factor) * -1;
	}
	else				// positive y acceleration
	{
		*pYg = (dataY & 0x1FF) / ADXL343_scale_factor;
	}

	if (dataZ & 0x200) // negative z acceleration, perform twos complement
	{
		*pZg = (((~dataZ + 1) & 0x1FF) / ADXL343_scale_factor) * -1;
	}
	else				// positive z acceleration
	{
		*pZg = (dataZ & 0x1FF) / ADXL343_scale_factor;
	}

	return;
}

/*****************************************************************
 * @fn			- ADXL343_PitchRollCalc
 *
 * @brief		- This functions converts the RAW data read from the ADXL343
 * 				  into a Pitch & Roll value in degrees
 *
 * @param[in]	- Handle structure for the ADXL343
 * @param[in]	- RAW data for X/Y/Z (6 bytes long)
 *
 *
 * @return		- I2C_ERROR_CODE
 *
 * @Note		-  Pitch; front of PCB points down results in negative pitch
 * 				   Roll;  left side of PCB points down results in negative roll
 *                 if Zg is negative, all LEDs turn on
 */
void ADXL343_PitchRollCalc(ADXL343_Handle_t *dev, uint8_t rawData[6])
{
	float Xg, Yg, Zg;
	ADXL343_CalcGForce(rawData, &Xg, &Yg, &Zg);
	dev->Pitch = atan(Yg/sqrt(Xg * Xg + Zg * Zg)) * 57.3; // pitch = atan((-Accx)/sqrt(Accy^2+Accz^2)), *57.3 converts from radians to degrees
	dev->Roll = atan(-Xg/sqrt(Yg * Yg + Zg * Zg)) * 57.3; // roll = atan(AccY/sqrt(Accx^2 + Accz^2)), *57.3 converts from radians to degrees
	dev->Zg = Zg;


	// add thing here to get +/- 180 degree range
	if (dev->Zg < 0)
	{

	}

	return;
}








// Might not be used
uint8_t ADXL343_OffsetComplementCalc(float val)
{
	uint8_t Offset;

	int gOff = (int)(round(val/ADXL343_OffsetReg_ScaleFactor));

	/*
	uint8_t *array;
	array = (uint8_t*)(&val);
	*/

	if (gOff < 0)
	{
		Offset = ~(uint8_t)(gOff*-1) + 1;
	}
	else
	{
		Offset = (uint8_t)gOff;
	}

	return Offset;
}
