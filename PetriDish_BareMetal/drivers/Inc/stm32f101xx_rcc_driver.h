/*
 * stm32f101xx_rcc_driver.h
 *
 *  Created on: Oct 6, 2024
 *      Author: arron
 */

#ifndef INC_STM32F101XX_RCC_DRIVER_H_
#define INC_STM32F101XX_RCC_DRIVER_H_

#include "stm32f101xx.h"

typedef struct
{
	uint8_t	PLLsrc;		/* possible values from @PLL_SRC */
	uint8_t PLLHSEDiv;	/* possible values from @PLL_HSE_DIV */
	uint8_t	PLLMul;		/* possible values from @PLL_MULTIPLIER */

}RCC_PllConfig_t;

typedef struct
{
	uint8_t SysClkSrc;			/* possible values from @RCC_CFGR_SW */
	RCC_PllConfig_t PllConfig;	/* This variables holds PLL configuration settings */
}RCC_SysConfig_t;

typedef struct
{
	uint8_t AHBprescaler;		/* possible values from @RCC_CFGR_HPRE */
	uint8_t APB1prescaler;		/* possible values from @RCC_CFGR_PPRE1 */
	uint8_t APB2prescaler;		/* possible values from @RCC_CFGR_PPRE2 */
	uint8_t ADCprescaler;		/* possible values from @RCC_CFGR_ADCPRE */
}RCC_Prescalers_t;

/*
 * @PLL_SRC
 */
#define RCC_PLL_SRC_HSI 0	// Use HSI as the PLL Source clock
#define RCC_PLL_SRC_HSE 1	// Use HSE as the PLL Source clock

/*
 * @PLL_HSE_DIV
 * for setting value of PREDIV1[3:0] in RCC_CFGR2 register
 */
#define RCC_PLL_HSE_DIV_0		0	//PREDIV1 input clock not divided
#define RCC_PLL_HSE_DIV_2		1	//PREDIV1 input clock divided by 2
//#define RCC_PLL_HSE_DIV_3		2	//PREDIV1 input clock divided by 3
//#define RCC_PLL_HSE_DIV_4		3	//PREDIV1 input clock divided by 4
//#define RCC_PLL_HSE_DIV_5		4	//PREDIV1 input clock divided by 5
//#define RCC_PLL_HSE_DIV_6		5	//PREDIV1 input clock divided by 6
//#define RCC_PLL_HSE_DIV_7		6	//PREDIV1 input clock divided by 7
//#define RCC_PLL_HSE_DIV_8		7	//PREDIV1 input clock divided by 8
//#define RCC_PLL_HSE_DIV_9		8	//PREDIV1 input clock divided by 9
//#define RCC_PLL_HSE_DIV_10		9	//PREDIV1 input clock divided by 10
//#define RCC_PLL_HSE_DIV_11		10	//PREDIV1 input clock divided by 11
//#define RCC_PLL_HSE_DIV_12		11	//PREDIV1 input clock divided by 12
//#define RCC_PLL_HSE_DIV_13		12	//PREDIV1 input clock divided by 13
//#define RCC_PLL_HSE_DIV_14		13	//PREDIV1 input clock divided by 14
//#define RCC_PLL_HSE_DIV_15		14	//PREDIV1 input clock divided by 15
//#define RCC_PLL_HSE_DIV_16		15	//PREDIV1 input clock divided by 16

/*
 * @PLL_MULTIPLIER
 */
#define RCC_PLL_MUL_4		2	// PLL input clock x4
#define RCC_PLL_MUL_5		3	// PLL input clock x5
#define RCC_PLL_MUL_6		4	// PLL input clock x6
#define RCC_PLL_MUL_7		5	// PLL input clock x7
#define RCC_PLL_MUL_8		6	// PLL input clock x8
#define RCC_PLL_MUL_9		7	// PLL input clock x9
#define RCC_PLL_MUL_6p5		13	// PLL input clock x6.5

/*
 * @RCC_CFGR_SW
 * Possible states for SW in RCC_CFGR register
 */
#define RCC_SW_HSI	0	// HSI selected as system clock
#define RCC_SW_HSE	1	// HSE selected as system clock
#define RCC_SW_PLL	2	// PLL selected as system clock
#define RCC_SW_NA	3	// Not allowed input


/*
 * @RCC_CFGR_SWS
 * Possible states for SWS in RCC_CFGR register
 */
#define RCC_SWS_HSI	0	// HSI used as system clock
#define RCC_SWS_HSE	1	// HSE used as system clock
#define RCC_SWS_PLL	2	// PLL used as system clock
#define RCC_SWS_NA	3	// Not allowed input

/*
 * @RCC_CFGR_HPRE
 * Possible states for HPRE (AHB prescaler) in RCC_CFGR register
 */
#define RCC_HPRE_1		0	// AHB = SYSCLK
#define RCC_HPRE_2		8	// AHB = SYSCLK/2
#define RCC_HPRE_4		9	// AHB = SYSCLK/4
#define RCC_HPRE_8		10	// AHB = SYSCLK/8
#define RCC_HPRE_16		11	// AHB = SYSCLK/16
#define RCC_HPRE_64		12	// AHB = SYSCLK/64
#define RCC_HPRE_128	13	// AHB = SYSCLK/128
#define RCC_HPRE_256	14	// AHB = SYSCLK/256
#define RCC_HPRE_512	15	// AHB = SYSCLK/512

/*
 * @RCC_CFGR_PPRE1
 * Possible states for PPRE1 (APB1 prescaler) in RCC_CFGR register
 */
#define RCC_PPRE1_1		0	// APB1 = AHB
#define RCC_PPRE1_2		4	// APB1 = AHB/2
#define RCC_PPRE1_4		5	// APB1 = AHB/4
#define RCC_PPRE1_8		6	// APB1 = AHB/8
#define RCC_PPRE1_16	7	// APB1 = AHB/16

/*
 * @RCC_CFGR_PPRE2
 * Possible states for PPRE2 (APB2 prescaler) in RCC_CFGR register
 */
#define RCC_PPRE2_1		0	// APB2 = AHB
#define RCC_PPRE2_2		4	// APB2 = AHB/2
#define RCC_PPRE2_4		5	// APB2 = AHB/4
#define RCC_PPRE2_8		6	// APB2 = AHB/8
#define RCC_PPRE2_16	7	// APB2 = AHB/16

/*
 * @RCC_CFGR_ADCPRE
 * Possible states for ADCPRE (ADC prescaler) in RCC_CFGR register
 */
#define RCC_ADCPRE1_2		0	// APB2/2
#define RCC_ADCPRE1_4		1	// APB2/4
#define RCC_ADCPRE1_6		2	// APB2/6
#define RCC_ADCPRE1_8		3	// APB2/8

/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
void RCC_Init_SysClk(RCC_SysConfig_t *pSysConfig);
void RCC_Init_AHBClk(RCC_Prescalers_t *RccPrescalers);
void RCC_Init_PCLK1(RCC_Prescalers_t *RccPrescalers);
void RCC_Init_PCLK2(RCC_Prescalers_t *RccPrescalers);
void RCC_Init_ADCCLK(RCC_Prescalers_t *RccPrescalers);
#endif /* INC_STM32F101XX_RCC_DRIVER_H_ */
