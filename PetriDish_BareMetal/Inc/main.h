/*
 * main.h
 *
 *  Created on: Aug 31, 2024
 *      Author: arron
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f101xx.h"
#include "CAV25M01.h"
#include "LM75B_TempSensor.h"
#include "ADXL343_Accel.h"
#include <math.h>

#define HSE_OSC_FREQ	16000000	// Frequency of crystal connected to HSE

#define invertedZg 	0 	//Zg values less than 0 are considered to be board is inverted (turns all LEDs on)
#define angleThres 	5 	//5 degrees rotation is required before LEDs will illuminate


void GPIO_INIT_ALL(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void LED_PetriDish(float Roll, float Pitch, float Zg);
void LED_ErrorState(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void Command_Handling(USART_Handle_t *pUSARTHandle, char *txBuff);
void UserMessages(USART_Handle_t *pUSARTHandle, char *txBuff, uint8_t BlockingMode);
void SetupDevice(CAV25M01_Handle_t *pdev);

#endif /* MAIN_H_ */
