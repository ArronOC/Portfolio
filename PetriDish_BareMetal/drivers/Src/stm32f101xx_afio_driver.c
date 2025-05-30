/*
 * stm32f101xx_afio_driver.c
 *
 *  Created on: Nov 9, 2024
 *      Author: arron
 */

#include "stm32f101xx_afio_driver.h"


/*****************************************************************
 * @fn			- AFIO_PeriEn
 *
 * @brief		- This function enables the AFIO peripheral
 *
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		-
 */
void AFIO_PeriEn(uint8_t EnorDi)
{
	/*
	 * APB2 also need to be enabled
	 * AFIO clock needs to be enabled RCC_APB2ENR
	 */
	if(EnorDi == ENABLE)
	{
		AFIO_PCLK_EN();
	} else
	{
		AFIO_PCLK_DI();
	}
}

/*****************************************************************
 * @fn			- AFIO_REMAP
 *
 * @brief		- This function remaps pins to alternate functions
 *
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- this function is purpose written for STM32_petridish board
 */
void AFIO_REMAP(void)
{
	AFIO->MAPR |= (0x2 << 24);	// Release PA15 from JTDI so it can be used as AFIO
	AFIO->MAPR |= (0x3 << 8); 	// Remap TIM2 as full remap
	AFIO->MAPR |= (0x1 << 1);	// Remap I2C1 to set to PB8/PB9
}
