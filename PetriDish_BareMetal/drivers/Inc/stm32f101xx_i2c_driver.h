/*
 * stm32f101xx_i2c_driver.h
 *
 *  Created on: Dec 8, 2024
 *      Author: arron
 */

#ifndef INC_STM32F101XX_I2C_DRIVER_H_
#define INC_STM32F101XX_I2C_DRIVER_H_

#include "stm32f101xx.h"

typedef enum I2C_Error_Code {
	I2C_NO_ERROR 		= 0,	// No error occured
	I2C_BUS_ERROR		= 1,	// Misplaced start or stop condition
	I2C_ACK_ERROR		= 2,	// Acknowledge failure
	I2C_ARLO_ERROR		= 3,	// Arbitration lost detected
	I2C_TIMEOUT_ERROR	= 4,	// Timeout occurred
	I2C_ERROR_OVR		= 5,	// Overrun/Underrun occurred
	USER_TIMEOUT		= 6,	// No flag set in I2C peripheral, but used to state the blocking API timed out
	USER_INVALID_DEVICE	= 7		// No flag set in I2C peripher, used for when I2C operation reads invalid device ID
} I2C_ERROR_CODE;

typedef struct {
	uint32_t	I2C_SCLSpeed;			/* @I2C_SCLSpeed */
	uint8_t		I2C_DutyCycle;			/* @I2C_DutyCycle */
	uint8_t		I2C_ACKControl;			/* @I2C_ACKControl */
	uint8_t		I2C_AddrMode;			/* @I2C_AddrMode */
	uint8_t		I2C_DualAddrEn;			/* @I2C_DualAddrEn */
	uint16_t	I2C_OwnAddr1;			/* @I2C_OwnAddress1 */
	uint16_t	I2C_OwnAddr2;			/* @I2C_OwnAddress2 */
}I2C_Config_t;

typedef struct {
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;
	uint8_t 		*pTxBuffer;	/* To store the app. Tx buffer address */
	uint8_t 		*pRxBuffer; /* To store the app. Rx buffer address */
	uint32_t 		TxLen;		/* To store Tx len */
	uint32_t 		RxLen;		/* To store Rx len */
	uint8_t 		TxRxState;	/* To store communication state @I2C_Application_States*/
	uint8_t			DevAddr;	/* To store slave/device address */
	uint32_t		RxSize;		/* To store Rx size */
	uint8_t			Sr;			/* To store repeated start value @I2C_Repeated_Start */
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 * Up to 400000Hz (this stm32 device does not support fast mode plus)
 */
#define I2C_SCL_SPEED_SM1K	100000	//Maximum speed supported in standard mode
#define I2C_SCL_SPEED_FM4K	400000	//Maximum speed supported in fast mode

/*
 * @I2C_DutyCycle
 */
#define I2C_FM_DUTY_2		0 /* Tlow/Thigh = 2 */
#define I2C_FM_DUTY_16_9	1 /* Tlow/Thigh = 16/9 */

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_DISABLED	0
#define I2C_ACK_ENABLED		1

/*
 * @I2C_AddrMode
 */
#define I2C_7BIT_SLAVE	0	/* Slave address is 7 bits wide */
#define	I2C_10BIT_SLAVE 1	/* Slave address is 10 bits wide */

/*
 * @I2C_DualAddrEn
 * Dual address mode can only be enabled in 7 bit address mode
 */
#define I2C_DUAL_ADDRESS_DI 0
#define I2C_DUAL_ADDRESS_EN 1

/*
 * @I2C_Application_States
 */
#define I2C_READY		0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2

/*
 * @I2C_Repeated_Start
 */
#define I2C_SR_DISABLE	0
#define I2C_SR_ENABLE	1

/*
 * I2C related status flag definitions
 */
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RxNE)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TxE)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR		(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_MSL		(1 << I2C_SR2_MSL)
#define I2C_FLAG_BUSY		(1 << I2C_SR2_BUSY)
#define I2C_FLAG_TRA		(1 << I2C_SR2_TRA)
#define I2C_FLAG_GENCALL	(1 << I2C_SR2_GENCALL)
#define I2C_FLAG_DUALF		(1 << I2C_SR2_DUALF)
#define I2C_FLAG_PEC		(1 << I2C_SR2_PEC)


#define READ	1
#define WRITE	0
#define I2C_MAX_TIMEOUT_MS	1000	//1000ms timeout for I2C functions

/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_EnorDI(I2C_RegDef_t *pI2Cx, uint8_t EnorDI);
void I2C_Init(I2C_Handle_t *pI2CxHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
I2C_ERROR_CODE I2C_MasterSendData(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint32_t TIMEOUT_MS);
I2C_ERROR_CODE I2C_MasterReceiveData(I2C_Handle_t *pI2CxHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint32_t TIMEOUT_MS);
I2C_ERROR_CODE I2C_Mem_MasterSendData(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SlaveRegister, uint32_t TIMEOUT_MS);
I2C_ERROR_CODE I2C_Mem_MasterReceiveData(I2C_Handle_t *pI2CxHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SlaveRegister, uint32_t TIMEOUT_MS);
void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
uint32_t RCC_GetPCLK1_Value(void);

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CxHandle);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CxHandle, uint8_t AppEv);
/*
 * Following APIs were not fully implemented
 */
//uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
//uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CxHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
//void I2C_EV_IRQHandling(I2C_Handle_t *pI2CxHandle);

#endif /* INC_STM32F101XX_I2C_DRIVER_H_ */
