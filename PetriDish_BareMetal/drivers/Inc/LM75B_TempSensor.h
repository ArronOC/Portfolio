/*
 * LM75B_TempSensor.h
 *
 *  Created on: Jun 22, 2024
 *      Author: arron
 */

#ifndef INC_LM75B_TEMPSENSOR_H_
#define INC_LM75B_TEMPSENSOR_H_

#include "stm32f101xx_i2c_driver.h"


typedef struct {
	I2C_Handle_t *i2cHandle;	/* Handle for I2C */
	uint8_t Addr;				/* @SlaveAddress */
} LM75B_Handle_t;

/*
 *  @SlaveAddress
 */
#define LM75B_Addr_000	0x48	// A0/A1/A2 = 0
#define LM75B_Addr_001	0x49	// A0 = 1, A1/A2 = 0
#define LM75B_Addr_010	0x4A	// A1 = 1, A0/A2 = 0
#define LM75B_Addr_011	0x4B	// A0/A1 = 1, A2 = 0
#define LM75B_Addr_100	0x4C	// A2 = 1, A0/A1 = 0
#define LM75B_Addr_101	0x4D	// A0/A2 = 1, A1 = 0
#define LM75B_Addr_110	0x4E	// A1/A2 = 1, A0 = 0
#define LM75B_Addr_111	0x4F	// A0/A1/A2 = 1



#define tempRes 		0.125 // resolution of 0.125C

#define LM75B_RegConfig		0x1
#define LM75B_ConfigByte	0x0

#define LM75B_RegTemp 		0x0		// Temperature register containing two 8-bit data bytes.
									// bits 4:0 of LSB are not used

#define LM75B_RegTOS		0x3		// POR state 0x5000 (80C) Overtemperature shutdown threshold register containing two 8-bit data bytes
#define LM75B_tosByteMSB	0x50
#define LM75B_tosByteLSB	0x0

#define LM75B_RegThyst		0x02	// POR state 0x4B00 (75C) Hysteresis register contains two 8-bit data bytes
#define LM75B_thystByteMSB	0x4B
#define LM75B_thystByteLSB	0x0


/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
I2C_ERROR_CODE LM75B_POR(LM75B_Handle_t *dev);
float LM75B_ReadTemp(LM75B_Handle_t *dev);

#endif /* INC_LM75B_TEMPSENSOR_H_ */
