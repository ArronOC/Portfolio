/*
 * stm32f101xx_gpio_driver.h
 *
 *  Created on: Oct 5, 2024
 *      Author: arron
 */

#ifndef INC_STM32F101XX_GPIO_DRIVER_H_
#define INC_STM32F101XX_GPIO_DRIVER_H_

#include "stm32f101xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;		/* possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;		/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinCNF;		/* possible values from @GPIO_PIN_CNF */
}GPIO_PinConfig_t;

/*
 * This is a handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;				// This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	// This variable holds GPIO pin configuration settings
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_INP 			0		// input mode (also reset state with combined @GPIO_PIN_CONFIG GPIO_CNF_INP_RESET)
#define GPIO_MODE_10MHZ_OUT		1		// output mode, max speed 10MHz
#define GPIO_MODE_2MHZ_OUT		2		// output mode, max speed 2MHz
#define GPIO_MODE_50MHZ_OUT 	3		// output mode, max speed 50MHz
#define GPIO_MODE_IT_FT 		4		// for use in interrupt input mode (falling edge)
#define GPIO_MODE_IT_RT 		5		// for use in interrupt input mode (rising edge)
#define GPIO_MODE_IT_RFT 		6		// for use in interrupt input mode (rising/falling edge)

/*
 * @GPIO_PIN_CNF
 * GPIO pin possible configurations
 */
#define GPIO_CNF_INP_ANALOG		0		// input mode analog
#define GPIO_CNF_INP_RESET		1		// input mode floating input
#define GPIO_CNF_INP_PD			4		// input with pull down
#define GPIO_CNF_INP_PU			5		// input with pull up
#define GPIO_CNF_OUT_GEN_PP		0		// general purpose output with push pull
#define GPIO_CNF_OUT_GEN_OD		1		// general purpose output with open drain
#define GPIO_CNF_OUT_ALT_PP		2		// alternate function output with push pull
#define GPIO_CNF_OUT_ALT_OD		3		// alternate function output with open drain

/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F101XX_GPIO_DRIVER_H_ */
