/*
 * stm32f101xx_usart_driver.h
 *
 *  Created on: Jan 18, 2025
 *      Author: arron
 */

#ifndef INC_STM32F101XX_USART_DRIVER_H_
#define INC_STM32F101XX_USART_DRIVER_H_

#include "stm32f101xx.h"

typedef struct {
	uint8_t		Mode;			/* @Mode */
	uint32_t	BaudRate;		/* @BaudRate */
	uint32_t	APBClock;		/* Clock speed in Hz of the APB bus */
	uint8_t		NumStopBits;	/* @NumStopBits */
	uint8_t		WordLength;		/* @WordLength */
	uint8_t		ParityControl;	/* @ParityControl */
	uint8_t		HWFlowControl;	/* @HWFlowControl */

}USART_Config_t;

typedef struct {
	USART_RegDef_t	*pUSARTx;
	USART_Config_t	USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
	uint8_t RxCRRcvd;
	uint8_t RxLFRcvd;
}USART_Handle_t;

/*
 * @Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 * @BaudRate
 */
#define USART_BaudRate_BPS_1200		1200
#define USART_BaudRate_BPS_2400		2400
#define USART_BaudRate_BPS_4800		4800
#define USART_BaudRate_BPS_9600		9600
#define USART_BaudRate_BPS_14400	14400
#define USART_BaudRate_BPS_19200	19200
#define USART_BaudRate_BPS_38400	38400
#define USART_BaudRate_BPS_57600	57600
#define USART_BaudRate_BPS_115200	115200
#define USART_BaudRate_BPS_230400	230400
#define USART_BaudRate_BPS_460800	460800
#define USART_BaudRate_BPS_921600	921600

/*
 * @NumStopBits
 */
#define USART_STOPBITS_1	0	// 1 Stop bit
#define USART_STOPBITS_0_5	1	// 0.5 Stop bits
#define USART_STOPBITS_2	2	// 2 Stop bits
#define USART_STOPBITS_1_5	3	// 1.5 Stop bits

/*
 * @WordLength
 */
#define USART_WORDLEN_8BITS	0	// 8 bit word length
#define USART_WORDLEN_9BITS	1	// 9 bit word length

/*
 * @ParityControl
 */
#define USART_PARITY_NONE	0	// no parity
#define USART_PARITY_EVEN	1	// even parity
#define USART_PARITY_ODD	2	// odd parity

/*
 * @HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART related status flag definitions
 */
#define USART_FLAG_CTS		(1 << USART_SR_POS_CTS)
#define USART_FLAG_LBD		(1 << USART_SR_POS_LBD)
#define USART_FLAG_TXE		(1 << USART_SR_POS_TXE)
#define USART_FLAG_TC		(1 << USART_SR_POS_TC)
#define USART_FLAG_RXNE		(1 << USART_SR_POS_RXNE)
#define USART_FLAG_IDLE		(1 << USART_SR_POS_IDLE)
#define USART_FLAG_ORE		(1 << USART_SR_POS_ORE)
#define USART_FLAG_NE		(1 << USART_SR_POS_NE)
#define USART_FLAG_FE		(1 << USART_SR_POS_FE)
#define USART_FLAG_PE		(1 << USART_SR_POS_PE)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 		1
#define USART_BUSY_IN_TX 		2
#define USART_READY				0


#define USART_EVENT_TX_CMPLT	0
#define	USART_EVENT_RX_CMPLT	1
#define	USART_EVENT_IDLE		2
#define	USART_EVENT_CTS			3
#define	USART_EVENT_PE			4
#define	USART_ERR_FE			5
#define	USART_ERR_NE			6
#define	USART_ERR_ORE			7
#define	USART_EVENT_RX_RCVD		8

#define NOT_RECEIVED	0
#define RECEIVED		1

#include <stddef.h>
/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Peripheral Enable/Disable
 */
void USART_ENorDI(USART_RegDef_t *pUSARTx, uint8_t EnorDI);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTxHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Other Peripheral Control APIs
 */
void USART_SetBaudRate(USART_Handle_t *pUSARTxHandle);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * ISR handling
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv);

#endif /* INC_STM32F101XX_USART_DRIVER_H_ */
