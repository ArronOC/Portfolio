/*
 * stm32f101xx_afio_driver.h
 *
 *  Created on: Nov 9, 2024
 *      Author: arron
 */

#ifndef INC_STM32F101XX_AFIO_DRIVER_H_
#define INC_STM32F101XX_AFIO_DRIVER_H_

#include "stm32f101xx.h"

/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
void AFIO_PeriEn(uint8_t EnorDi);
void AFIO_REMAP(void);

#endif /* INC_STM32F101XX_AFIO_DRIVER_H_ */
