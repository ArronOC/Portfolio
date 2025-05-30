/*
 * stm32f101xx_tim_driver.c
 *
 *  Created on: Nov 9, 2024
 *      Author: arron
 */


#include "stm32f101xx_tim_driver.h"

/*****************************************************************
 * @fn			- TIM_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given TIMER peripheral
 *
 * @param[in]	- base address of the TIMER peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pTIMx == TIM1)
		{
			TIM1_PCLK_EN();
		}else if (pTIMx == TIM2)
		{
			TIM2_PCLK_EN();
		}else if (pTIMx == TIM3)
		{
			TIM3_PCLK_EN();
		}else if (pTIMx == TIM4)
		{
			TIM4_PCLK_EN();
		}
	} else
	{
		if (pTIMx == TIM1)
		{
			TIM1_PCLK_DI();
		}else if (pTIMx == TIM2)
		{
			TIM2_PCLK_DI();
		}else if (pTIMx == TIM3)
		{
			TIM3_PCLK_DI();
		}else if (pTIMx == TIM4)
		{
			TIM4_PCLK_DI();
		}
	}
}


/*****************************************************************
 * @fn			- TIM_init
 *
 * @brief		- This functions configures the specified TIMER as a PWM output
 *
 * @param[in]	- Config structure for the TIMER to configure
 *
 * @return		- none
 *
 * @Note		- This function has been purpose writen for initialising the STM32_Petridish board
 */
void TIM_init(TIM_Handle_t *pTIMHandle)
{
	// Enable the peripheral so settings can be configured
	TIM_PeriClockControl(pTIMHandle->pTIMx, ENABLE);

	// Configure the prescaler so that the counter clock frequency is 1MHz
	// Note that this has been hard coded for a 48MHz PLL clock, if alternative clock is used this needs updating
	pTIMHandle->pTIMx->PSC = 47;

	pTIMHandle->pTIMx->CR[0] |= (TIM_CLOCKDIVISION_DIV1 << 8); 			// Set the clock division
	pTIMHandle->pTIMx->CR[0] |= (TIM_AUTORELOAD_PRELOAD_ENABLE << 7);	// Set the auto reload preload setting
	pTIMHandle->pTIMx->CR[0] |= (TIM_COUNTERMODE_UP << 4);				// set the counter direction

	pTIMHandle->pTIMx->ARR = pTIMHandle->init.Period;	// Maximum value the counter counts to

	if (pTIMHandle->init.EnableCH1 == ENABLE)
	{
		pTIMHandle->pTIMx->CCMR[0] &= ~(0x3);		// Configure TIM1_CH1 as an output
		pTIMHandle->pTIMx->CCMR[0] |= (0x6 << 4);	// Configure TIM1_CH1 to PWM Mode 1
		pTIMHandle->pTIMx->CCMR[0] |= (0x1 << 3);	// Enable preload TIM1_CH1
		pTIMHandle->pTIMx->CCER &= ~(0x1 << 1);		// Configure output polarity as active high
		pTIMHandle->pTIMx->CCER |= 0x1;				// Enable the capture/compare
	}

	if (pTIMHandle->init.EnableCH2 == ENABLE)
	{
		pTIMHandle->pTIMx->CCMR[0] &= ~(0x3 << 8);	// Configure TIM1_CH2 as an output
		pTIMHandle->pTIMx->CCMR[0] |= (0x6 << 12);	// Configure TIM1_CH2 to PWM Mode 1
		pTIMHandle->pTIMx->CCMR[0] |= (0x1 << 11);	// Enable preload TIM1_CH2
		pTIMHandle->pTIMx->CCER &= ~(0x1 << 5);		// Configure output polarity as active high
		pTIMHandle->pTIMx->CCER |= (0x1 << 4);				// Enable the capture/compare
	}

	if (pTIMHandle->init.EnableCH3 == ENABLE)
	{
		pTIMHandle->pTIMx->CCMR[1] &= ~(0x3);		// Configure TIM1_CH3 as an output
		pTIMHandle->pTIMx->CCMR[1] |= (0x6 << 4);	// Configure TIM1_CH3 to PWM Mode 1
		pTIMHandle->pTIMx->CCMR[1] |= (0x1 << 3);	// Enable preload TIM1_CH3
		pTIMHandle->pTIMx->CCER &= ~(0x1 << 9);		// Configure output polarity as active high
		pTIMHandle->pTIMx->CCER |= (0x1 << 8);		// Enable the capture/compare
	}

	if (pTIMHandle->init.EnableCH4 == ENABLE)
	{
		pTIMHandle->pTIMx->CCMR[1] &= ~(0x3 << 8);	// Configure TIM1_CH2 as an output
		pTIMHandle->pTIMx->CCMR[1] |= (0x6 << 12);	// Configure TIM1_CH2 to PWM Mode 1
		pTIMHandle->pTIMx->CCMR[1] |= (0x1 << 11);	// Enable preload TIM1_CH2
		pTIMHandle->pTIMx->CCER &= ~(0x1 << 13);	// Configure output polarity as active high
		pTIMHandle->pTIMx->CCER |= (0x1 << 12);		// Enable the capture/compare
	}

	pTIMHandle->pTIMx->BDTR |= (0x1 << 14); // Enable the Automatic Output Enable, which will trigger the Master Output Enable when an event generation occurs

	pTIMHandle->pTIMx->CR[0] |= 0x1;	// Enable the counter
}
