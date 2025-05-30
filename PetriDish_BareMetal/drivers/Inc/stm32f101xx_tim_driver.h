/*
 * stm32f101xx_tim_driver.h
 *
 *  Created on: Nov 9, 2024
 *      Author: arron
 */

#ifndef INC_STM32F101XX_TIM_DRIVER_H_
#define INC_STM32F101XX_TIM_DRIVER_H_

#include "stm32f101xx.h"


//htim1.Init.Prescaler = 0;
//	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim1.Init.Period = 65535;
//	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	htim1.Init.RepetitionCounter = 0;
//	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

typedef struct
{
	uint8_t		EnableCH1;		/* Set to 1, to configure TIMx_CH1 */
	uint8_t		EnableCH2;		/* Set to 1, to configure TIMx_CH2 */
	uint8_t		EnableCH3;		/* Set to 1, to configure TIMx_CH3 */
	uint8_t		EnableCH4;		/* Set to 1, to configure TIMx_CH4 */
	uint32_t	Period;			/* 0->65535	*/
}TIM_Init_t;

typedef struct
{
	TIM_RegDef_t *pTIMx;	// This holds the base address of TIMx registers
	TIM_Init_t init;		// holds the settings used to configure the Timer upon boot
}TIM_Handle_t;


/*
 * @TIM_COUNT_MODE
 * Direction of counter
 */
#define TIM_COUNTERMODE_UP		0
#define TIM_COUNTERMODE_DOWN	1

/*
 * @TIM_CLOCK_DIV
 * Division ratio between timer clock (CK_INT) and dead time and sampling clock (Tdts)
 */
#define TIM_CLOCKDIVISION_DIV1	0	/*!< Clock division: tDTS=tCK_INT   */
#define TIM_CLOCKDIVISION_DIV2	1	/*!< Clock division: tDTS=2*tCK_INT */
#define TIM_CLOCKDIVISION_DIV4	2	/*!< Clock division: tDTS=4*tCK_INT */

/*
 * @TIM_AUTO_RELOAD_PRELOAD
 * Enable or disable auto reload preload
 */
#define TIM_AUTORELOAD_PRELOAD_DISABLE	0
#define TIM_AUTORELOAD_PRELOAD_ENABLE	1

/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi);
void TIM_init(TIM_Handle_t *pTIMHandle);

#endif /* INC_STM32F101XX_TIM_DRIVER_H_ */
