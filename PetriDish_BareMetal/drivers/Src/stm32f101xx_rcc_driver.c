/*
 * stm32f101xx_rcc_driver.c
 *
 *  Created on: Oct 6, 2024
 *      Author: arron
 */

#include "stm32f101xx_rcc_driver.h"

/*****************************************************************
 * @fn			- RCC_Init_SysClk
 *
 * @brief		- This function sets which clock source to use as system clock and configures any HSI/HSE/PLL settings required
 *
 * @param[in]	- Config structure for the system clock
 *
 * @return		- none
 *
 * @Note		- max system clock speed = 48/72MHz
 */
void RCC_Init_SysClk(RCC_SysConfig_t *pSysConfig)
{
	uint8_t PllRdy = 0;
	if (pSysConfig->SysClkSrc == RCC_SW_HSI)
	{

		RCC->CR |= 0x1 << 0; 		// Set HSI to ON
		RCC->CFGR &= ~(0x3 << 0); 	// Set HSI to system clock

	} else if (pSysConfig->SysClkSrc == RCC_SW_HSE)
	{
		RCC->CR |= 0x1 << 16; 		// Set HSE to ON
		RCC->CFGR &= ~(0x3 << 0);	// Reset system clock mux
		RCC->CFGR |= RCC_SWS_HSE; 	// Set HSE to system clock

	} else if (pSysConfig->SysClkSrc == RCC_SW_PLL)
	{

		RCC->CFGR &= ~(0x3 << 0);	// Set HSI to system clock so that PLL settings can be configured
		RCC->CR &= ~(0x1 << 24);	// Turn off PLL Clock so that PLL settings can be configured

		if (pSysConfig->PllConfig.PLLsrc == RCC_PLL_SRC_HSI)
		{
			RCC->CFGR &= ~(0x1 << 16); // Set PLL source to HSI
		} else
		{
			RCC->CR |= 0x1 << 16; 							// Enable HSE Clock
			RCC->CFGR |= (0x1 << 16); 						// Set PLL source to HSE
			RCC->CFGR &= ~(0x1 << 17); 						// Reset value in PLLXTPRE
			RCC->CFGR |= (pSysConfig->PllConfig.PLLHSEDiv << 17);	// Set PLL HSE source division factor (PREDIV1/PLLXTPRE)
		}

		RCC->CFGR &= ~(0xF << 18);							//Reset the PLL Multiplier to default value
		RCC->CFGR |= pSysConfig->PllConfig.PLLMul << 18;	//Set the PLL Multiplier

		RCC->CR |= (0x1 << 24);	// Enable the PLL Clock
		// wait for PLL Clock to be locked in before activating
		do
		{
			PllRdy = (uint8_t) ((RCC->CR >> 25) & 0x1);
		} while (PllRdy == 0);

		RCC->CFGR |= (RCC_SW_PLL << 0);	// Set PLL as the system clock
	}
}

/*****************************************************************
 * @fn			- RCC_Init_AHBClk
 *
 * @brief		- This function configures the AHB prescaler to get the desired HCLK
 *
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- Max AHB clock speed = 48/72MHz
 */
void RCC_Init_AHBClk(RCC_Prescalers_t *RccPrescalers)
{
	RCC->CFGR &= ~(0xF << 4); 							// Reset the AHB prescaler
	RCC->CFGR |= (RccPrescalers->AHBprescaler << 4);	// Set the AHB prescaler
}

/*****************************************************************
 * @fn			- RCC_Init_PCLK1
 *
 * @brief		- This function configures the APB1 prescaler to get the desired APB1/PCLK1
 *
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- Maximum frequency for APB1 = 36MHz
 */
void RCC_Init_PCLK1(RCC_Prescalers_t *RccPrescalers)
{
	RCC->CFGR &= ~(0x7 << 8); 							// Reset the APB1 prescaler
	RCC->CFGR |= (RccPrescalers->APB1prescaler << 8);	// Set the APB1 prescaler
}

/*****************************************************************
 * @fn			- RCC_Init_PCLK2
 *
 * @brief		- This function configures the APB2 prescaler to get the desired APB2/PCLK2
 *
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- Maximum frequency for APB2 = 48/72MHz
 */
void RCC_Init_PCLK2(RCC_Prescalers_t *RccPrescalers)
{
	RCC->CFGR &= ~(0x7 << 11); 							// Reset the APB2 prescaler
	RCC->CFGR |= (RccPrescalers->APB2prescaler << 11);	// Set the APB2 prescaler
}


/*****************************************************************
 * @fn			- RCC_Init_ADCCLK
 *
 * @brief		- This function configures the ADC prescaler
 *
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		-
 */
void RCC_Init_ADCCLK(RCC_Prescalers_t *RccPrescalers)
{
	RCC->CFGR &= ~(0x3 << 14); 							// Reset the ADC prescaler
	RCC->CFGR |= (RccPrescalers->ADCprescaler << 14);	// Set the ADC prescaler
}

