/*
 * CAV25M01.h
 *	Supports SPI mode (0,0) & (1,1)
 *  Created on: Jul 14, 2024
 *      Author: arron
 */





#ifndef INC_CAV25M01_H_
#define INC_CAV25M01_H_

#include <string.h>
#include "stm32f101xx_spi_driver.h"
#include "stm32f101xx_gpio_driver.h"
//#include "main.h"

#define OPCODE_WREN	0x6	// Enable write operations
#define OPCODE_WRDI	0x4	// Disable write operations
#define OPCODE_RDSR	0x5	// Read Status Register
#define OPCODE_WRSR	0x1	// Write Status Register
#define OPCODE_READ	0x3	// Read data from memory
#define OPCODE_WRITE 	0x2	// Write data to memory

#define nRDYmask 0x1
#define WELmask 0x2
#define BPmask 0xC
#define LIPmask 0x10
#define IPLmask 0x40
#define WPENmask 0x80

#define pageSize 256

#define timeout 50 // 50ms timeout

#define STORE	1
#define RECALL	0

/* ==================== U6 SPI EEPROM Memory Map ==================== */
/*
 * MEMORY MAP
 * 0x0 -> 0x27 = Serial Number
 * 0x28 = accelerometer X offset value
 * 0x29 = accelerometer Y offset value
 * 0x30 = accelerometer Z offset value
 */
#define U6_SERIALADDR 	0x0UL		// memory location of serial number
#define U6_XoffAddr		0x28UL		// Memory location to store accelerometer X offset value
#define U6_YoffAddr		0x29UL		// Memory location to store accelerometer Y offset value
#define U6_ZoffAddr		0x2AUL		// Memory location to store accelerometer Z offset value
#define U6_TestAddr		0x500UL		// Memory location which is altered within @CAV25M01_Init

/*
 * serial number format
 * written as ascii
 * BoardName Revision DesignDate #UniqueIdenfitier
 * STM32_COMMS_BOARD REV2 20/07/2024 #1
 * 36 bytes
 */
#define SerNumSizeBytes 36
/* ================================================================== */

typedef struct{
	GPIO_Handle_t *pnCS0;
	GPIO_Handle_t *pnWP;
	GPIO_Handle_t *pnHOLD;
	SPI_Handle_t *pSPIhandle;
}CAV25M01_Handle_t;

/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
uint8_t CAV25M01_Init(CAV25M01_Handle_t *pdev);
void CAV25M01_SendData(CAV25M01_Handle_t *pdev, uint8_t *txData, uint8_t bytes);
void CAV25M01_SendReceive(CAV25M01_Handle_t *pdev, uint8_t *txData, uint8_t *rxData, uint8_t Bytes);
void CAV25M01_DevReady(CAV25M01_Handle_t *pdev);
uint8_t CAV25M01_ReadStatusReg(CAV25M01_Handle_t *pdev);
void CAV25M01_WriteStatusReg(CAV25M01_Handle_t *pdev, uint8_t *data);
void CAV25M01_enableWriteMode(CAV25M01_Handle_t *pdev);
void CAV25M01_disableWriteMode(CAV25M01_Handle_t *pdev);
void CAV25M01_AccData(CAV25M01_Handle_t *pdev, uint8_t *Xoff, uint8_t *Yoff, uint8_t *Zoff, uint8_t STOREorRECALL);
void CAV25M01_SerNum(CAV25M01_Handle_t *pdev, char *SerNum, uint8_t STOREorRECALL);
void CAV25M01_IdentPageSwitch(CAV25M01_Handle_t *pdev, uint8_t ENorDI);

#endif /* INC_CAV25M01_H_ */
