/*
 * stm32f101xx.c
 *
 *  Created on: Dec 22, 2024
 *      Author: arron
 */

#include "stm32f101xx.h"


static __vo uint32_t msticks = 0;

/*****************************************************************
 * @fn			- SYST_Config
 *
 * @brief		- This function configures the SysTick to provide exceptions on 1ms intervals
 *
 * @param[in]	- Clock speed of the processor in Hz
 *
 * @return		- none
 *
 * @Note		-
 */
void SYST_Config(uint32_t SystemClockHz)
{
	//1. Reset control register
	Systick->CTRL = 0x0;

	//2. Set reload value
	Systick->RVR = ( (SystemClockHz / 1000) - 1)& 0xFFFFFF;	//only 24 bits

	//3. Set priority systick exception
	SCB->SHP[11] = 0x10;

	//4. Reset Systick value
	Systick->CVR = 0;

	//5. Enable systick
	Systick->CTRL |= (0x1 << SYST_CSR_CLKSOURCE);	//Set to internal clock source
	Systick->CTRL |= (0x1 << SYST_CSR_TICKINT);		//Enable Systick interrupt
	//Systick->CTRL |= (0x1 << SYST_CSR_ENABLE);		//Enable the counter
}

/*****************************************************************
 * @fn			- DELAY_MS
 *
 * @brief		- This functions waits for the number of milliseconds requested
 *
 * @param[in]	- milliseconds to wait
 *
 * @return		- none
 *
 * @Note		-
 */
void DELAY_MS(uint32_t ms)
{
	msticks = 0;
	uint32_t startticks = 0;
	Systick->CVR = 0;								//Reset Systick value
	Systick->CTRL |= (0x1 << SYST_CSR_ENABLE);		//Enable the counter
	while ( (msticks - startticks) < ms );
	Systick->CTRL &= ~(0x1 << SYST_CSR_ENABLE);		//Disable the counter
}

/*****************************************************************
 * @fn			- ELAPSED_MS
 *
 * @brief		- This functions returns the number of milliseconds passed since the Systick timer started
 *
 * @param[in]	- none
 *
 * @return		- milliseconds passed since Systick enabled
 *
 * @Note		-
 */
uint32_t ELAPSED_MS(void)
{
	/*
	 * Needed to store value of msticks in separate variable before returning it.
	 * despite being "volatile" something would be misinterpreted by the compiler
	 * resulting in I2C operations timing out
	 */
	uint32_t ms = msticks;
	return ms;
}

/*****************************************************************
 * @fn			- SYST_STARTorSTOP
 *
 * @brief		- This functions starts or stops the Systick timer
 *
 * @param[in]	- ENABLE or DISABLE
 *
 * @return		-
 *
 * @Note		-
 */
void SYST_STARTorSTOP(uint8_t ENorDI)
{
	if (ENorDI)
	{
		msticks = 0;
		Systick->CVR = 0;								//Reset Systick value
		Systick->CTRL |= (0x1 << SYST_CSR_ENABLE);		//Enable the counter
	} else
	{
		Systick->CTRL &= ~(0x1 << SYST_CSR_ENABLE);		//Disable the counter
	}
}

/*****************************************************************
 * @fn			- SysTick_Handler
 *
 * @brief		- This function is called when a Systick exception occurs and increments the millisecond counter
 *
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		-
 */
void SysTick_Handler(void)
{
	//Systick is automatically cleared upon entering this handler
	//increment count of 1ms intervals
	msticks++;
}


/*****************************************************************
 * @fn			- NVIC_IRQITConfig
 *
 * @brief		- This function enables/disables a given interrupt on the NVIC
 *
 * @param[in]	- IRQNumber - refer to Table 63 in the reference manual (position column)
 * @param[in]	- EnorDi - 1 = enable, 0 = disable
 *
 * @return		-
 *
 * @Note		- none
 */
void NVIC_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber < 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
	} else
	{
		if (IRQNumber < 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
	}
}

/*****************************************************************
 * @fn			- NVIC_IRQPriorityConfig
 *
 * @brief		- This function is used to reorder the NVIC priority list
 *
 * @param[in]	- IRQNumber - refer to Table 63 in the reference manual (position column)
 * @param[in]	- IRQPriority desired interrupt priority number
 *
 * @return		-
 *
 * @Note		- none
 */
void NVIC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
