/*
 * ADXL343_Accel.h
 *
 *  Created on: Jun 23, 2024
 *      Author: arron
 */

#ifndef INC_ADXL343_ACCEL_H_
#define INC_ADXL343_ACCEL_H_

#include "stm32f101xx_i2c_driver.h"
#include <math.h>

typedef struct {
	I2C_Handle_t *i2cHandle;	/* I2C Handle */
	uint8_t	SlaveAddr;			/* Slave address of the ADXL343 */
	float Pitch, Roll;			/* Location to store positional data */
	float Xg, Yg, Zg;			/* Location to store acceleration data */
	uint8_t Xoff, Yoff, Zoff;	/* Location to store offset values */
	uint8_t NewData;			/* @NewData */
} ADXL343_Handle_t;

/*
 * @NewData
 */
#define nREADY	0
#define READY	1

#define ADXL343_Addr_High	0x1D
#define ADXL343_Addr_Low	0x53

// Register Map
#define ADXL343_Reg_DevID 			0x0		// Device ID register
#define ADXL343_Reg_THRESH_TAP 		0x1D	// Tap threshold register
#define ADXL343_Reg_OFSX 			0x1E	// X-axis offset register
#define ADXL343_Reg_OFSY 			0x1F	// Y-axis offset register
#define ADXL343_Reg_OFSZ 			0x20	// Z-axis offset register
#define ADXL343_Reg_DUR 			0x21	// Tap duration register
#define ADXL343_Reg_Latent 			0x22	// Tap latency register
#define ADXL343_Reg_Window 			0x23	// Tap window register
#define ADXL343_Reg_THRESH_ACT 		0x24	// Activity threshold register
#define ADXL343_Reg_THRESH_INACT 	0x25	// Inactivity threshold register
#define ADXL343_Reg_TIME_INACT 		0x26	// Inactivity time register
#define ADXL343_Reg_ACT_INACT_CTL 	0x27	// Axis enable control for activity and inactivity detection register
#define ADXL343_Reg_THRESH_FF 		0x28	// Free-fall threshold register
#define ADXL343_Reg_TIME_FF 		0x29	// Free-fall time register
#define ADXL343_Reg_TAP_AXES 		0x2A	// Axis control for singal tap/double tap register
#define ADXL343_Reg_ACT_TAP_STATUS 	0x2B	// Source of single tap/double tap register
#define ADXL343_Reg_BW_RATE 		0x2C	// Data rate and power mode control register
#define ADXL343_Reg_POWER_CTL 		0x2D	// Power-saving features control register
#define ADXL343_Reg_INT_ENABLE 		0x2E	// Interrupt enable control register
#define ADXL343_Reg_INT_MAP 		0x2F	// Interrupt mapping control register
#define ADXL343_Reg_INT_SOURCE 		0x30	// Source of interrupts register
#define ADXL343_Reg_DATA_FORMAT 	0x31	// Data format control register
#define ADXL343_Reg_DATAX0 			0x32	// X-Axis Data 0 register
#define ADXL343_Reg_DATAX1 			0x33	// X-Axis Data 1 register
#define ADXL343_Reg_DATAY0 			0x34	// Y-Axis Data 0 register
#define ADXL343_Reg_DATAY1 			0x35	// Y-Axis Data 1 register
#define ADXL343_Reg_DATAZ0 			0x36	// Z-Axis Data 0 register
#define ADXL343_Reg_DATAZ1 			0x37	// Z-Axis Data 1 register
#define ADXL343_Reg_FIFO_CTL 		0x38	// FIFO control register
#define ADXL343_Reg_FIFO_STATUS 	0x39	// FIFO status register


#define ADXL343_DevID					0xE5	// Expected value in Device ID Register
#define ADXL343_scale_factor			256.0f	// LSB/g scale factor in full res mode
#define ADXL343_OffsetReg_ScaleFactor	0.0156f // 15.6mg/LSB


/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
I2C_ERROR_CODE ADXL343_Init(ADXL343_Handle_t *dev);
I2C_ERROR_CODE ADXL343_EnDataReadyInt(ADXL343_Handle_t *dev);
I2C_ERROR_CODE ADXL343_DisableInterrupts(ADXL343_Handle_t *dev);
I2C_ERROR_CODE ADXL343_ReadData(ADXL343_Handle_t *dev, uint8_t *pRawData, GPIO_Handle_t *pInt1);
I2C_ERROR_CODE ADXL343_CalibrateFlat(ADXL343_Handle_t *dev, GPIO_Handle_t *pInt1);
void ADXL343_PitchRollCalc(ADXL343_Handle_t *dev, uint8_t Data[6]);
void ADXL343_CalcGForce(uint8_t *pRawData, float *pXg, float *pYg, float *pZg);

// might not be used
uint8_t ADXL343_OffsetComplementCalc(float val);

#endif /* INC_ADXL343_ACCEL_H_ */
