/*
 * stm32f101xx_gpio_driver.c
 *
 *  Created on: Oct 5, 2024
 *      Author: arron
 */

#include "stm32f101xx_gpio_driver.h"

/*****************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
	} else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
	}
}


/*****************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function configures the given GPIO pin
 *
 * @param[in]	- handle structure for the GPIO pin
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t CRreg, BitGroup;

	/*
	 * CRreg - Determines if we are writing to CRL (0->7) or CRH (8->15)
	 * BitGroup - Determines which group of bits to write to in the register
	 */
	CRreg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
	BitGroup = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;


	/*
	 * Configure the mode and speed of GPIO pin
	 * Pin 0->7 is written to register CRL
	 * Pin 8->15 is written to register CRH
	 */
	pGPIOHandle->pGPIOx->CR[CRreg] &= ~(0x3 << (4 * BitGroup)); // Clear the bits first
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode >= GPIO_MODE_IT_FT)
	{
		pGPIOHandle->pGPIOx->CR[CRreg] |= (GPIO_MODE_INP << (4 * BitGroup));	// Configure interrupt pins as input mode
	} else
	{
		pGPIOHandle->pGPIOx->CR[CRreg] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * BitGroup));
	}

	/*
	 * 2. Set the Pin configuration
	 * Pin 0->7 is written to register CRL
	 * Pin 8->15 is written to register CRH
	 */
	pGPIOHandle->pGPIOx->CR[CRreg] &= ~(0x3 << (2 + (4 * BitGroup))); // Clear the bits first

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinCNF == GPIO_CNF_INP_PD)			// Pull down input mode
	{
		// Configure the GPIO pin as input PUPD
		pGPIOHandle->pGPIOx->CR[CRreg] |= (0x2 << (2 + (4 * BitGroup)));
		// Configure the PxODR register as pull down (write 0 to ODR bit)
		pGPIOHandle->pGPIOx->ODR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinCNF == GPIO_CNF_INP_PU)	// Pull up input mode
	{
		// Configure the GPIO pin as input PUPD
		pGPIOHandle->pGPIOx->CR[CRreg] |= (0x2 << (2 + (4 * BitGroup)));
		// Configure the PxODR register as pull up (write 1 to ODR bit)
		pGPIOHandle->pGPIOx->ODR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	} else																	// All other non interrupt modes
	{
		// Configure the GPIO pin as stated in GPIO_Pin_CNF
		pGPIOHandle->pGPIOx->CR[CRreg] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinCNF << (2 + (4 * BitGroup)));
	}

	// Interrupt Configuration
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode >= GPIO_MODE_IT_FT)
	{
		// 1. Configure FTSR and/or RTSR depending on interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)			// Falling edge trigger
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// Enable the FTSR register
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// Disable the RTSR register to ensure both aren't set

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)		// Rising edge trigger
		{
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// Disable the FTSR register to ensure both aren't set
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// Enable the RTSR register

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)	// Rising or Falling edge trigger
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// Enable the FTSR register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// Enable the RTSR register
		}

		// 2. Configure the GPIO Port selection in AFIO_EXTICR
		AFIO_PCLK_EN();																// Ensure AFIO clock is enabled, so EXTICR can be set
		uint8_t EXTICRreg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;			// Determine which EXTICR reg we need to change
		uint8_t EXTICRBitGroup = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4; 	// Determine which bit group needs changing (Px0 = bits 0:3, Px1 = bits 4:7)
		AFIO->EXTICR[EXTICRreg] &= ~(0xF << (4 * EXTICRBitGroup));					// Clear the bits first

		uint8_t EXTIPortcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);			// Retrieve EXTI code for given GPIO port
		AFIO->EXTICR[EXTICRreg] |= (EXTIPortcode << (4 * EXTICRBitGroup));			// Set EXTI port code

		// 3. Enable the EXTI interrupt delivery using IMR (interrupt mask register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
}


/*****************************************************************
 * @fn			- GPIO_DeInit
 *
 * @brief		- This function returns the given GPIO port to its reset state
 *
 * @param[in]	- base address of the GPIO peripheral
 *
 * @return		- none
 *
 * @Note		- This needs to be run for each GPIO pin
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
}

/*****************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function reads the given GPIO Pin
 *
 * @param[in]	- Base address for the GPIO port to read
 * @param[in]	- Pin number to be read
 *
 * @return		- either a 0 for low, or 1 for high
 *
 * @Note		- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value = 0;

	Value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x1);

	return Value;
}

/*****************************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- This function reads the given GPIO port
 *
 * @param[in]	- Base address for the GPIO port to read
 *
 * @return		- uint16_t containing the input state of each GPIO pin
 *
 * @Note		- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value = 0;

	Value = (uint16_t) pGPIOx->IDR;

	return Value;
}

/*****************************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- This function sets a GPIO pin to the desired state
 *
 * @param[in]	- Base address of GPIO port
 * @param[in]	- Pin number to set
 * @param[in]	- macro GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return		-
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (GPIO_PIN_SET << PinNumber);	// Set value 1 to output pin, leaving others the same
	} else
	{
		pGPIOx->ODR &= ~(GPIO_PIN_SET << PinNumber);	// Set value 0 to output pin, leaving others the same
	}


}

/*****************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- This function is used to set the entire GPIO port at once
 *
 * @param[in]	- Base address of GPIO port to set
 * @param[in]	- Value to set each pin too
 *
 * @return		-
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*****************************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- This function changes the given GPIO pin to the opposite state
 *
 * @param[in]	- Base address of GPIO port
 * @param[in]	- Pin number to change
 *
 * @return		-
 *
 * @Note		- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber); // XOR
}

/*****************************************************************
 * @fn			- GPIO_IRQITConfig
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
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn			- GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*****************************************************************
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- This function
 *
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI PR Register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		// Clear
		EXTI->PR |= (1 << PinNumber);
	}
}
