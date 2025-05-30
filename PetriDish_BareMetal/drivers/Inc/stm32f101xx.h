/*
 * STM32F101xx.h
 *
 *	Contains the following
 *	Base addresses of various memories present in MCU (i.e. Flash, SRAM, ROM)
 *	Base addresses of various bus domains such as (AHBx, APBx)
 *	Base addresses of various peripherals present in different bus domains
 *	Clock management macros
 *	IRQ definitions
 *	Peripheral Register definition structures
 *	Peripheral register bit definitions
 *	Other useful MCU configuration macros
 *
 *  Created on: Oct 5, 2024
 *      Author: arron
 */

#ifndef STM32F101XX_H_
#define STM32F101XX_H_

#include <stdint.h>

#define __vo volatile

/***************************START:Processor Specific Detail***************************/
/*
 * ARM Cortex M3 Processor NVIC ISERx Register Address
 */
#define NVIC_ISER0	((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1	((__vo uint32_t*) 0xE000E104)

/*
 * ARM Cortex M3 Processor NVIC ICERx Register Address
 */
#define NVIC_ICER0	((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1	((__vo uint32_t*) 0xE000E184)

/*
 * ARM Cortex M3 Processor NVIC IPRx Register Address
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*) 0xE000E400)

/*
 * ARM Cortex M3 Processor number of priority bits implemented in priority register
 */
#define NO_PR_BITS_IMPLEMENTED 4

/*
 * ARM Cortex M3 Processor System timer registers
 */
#define SYST_BASE_ADDR	((__vo uint32_t*) 0xE000E010)
typedef struct {
	__vo uint32_t CTRL;
	__vo uint32_t RVR;
	__vo uint32_t CVR;
	__vo uint32_t CALIB;
}SysTick_RegDef_t;
#define Systick	((SysTick_RegDef_t*) SYST_BASE_ADDR)
/*
 * ARM Cortex M3 Processor System timer bit positions
 */
#define SYST_CSR_ENABLE		0
#define SYST_CSR_TICKINT	1
#define SYST_CSR_CLKSOURCE	2
#define SYST_CSR_COUNTFLAG	16

#define SYST_RVR_RELOAD		0

#define SYST_CVR_CURRENT	0

#define SYST_CALIB_TENMS	0
#define SYST_CALIB_SKEW		30
#define SYST_CALIB_NOREF	31

/*
 * ARM Cortex M3 Processor System Control Block Register Address
 */
#define SCB_BASE_ADDR	((__vo uint32_t*) 0xE000ED00)	// Actually starts at 0xE000E008 but we don't want to touch that register...
typedef struct {
	__vo  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
	__vo uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
	__vo uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
	__vo uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
	__vo uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
	__vo uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
	__vo uint8_t SHP[12];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-15) */
	__vo uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
	__vo uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
	__vo uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
	__vo uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
	__vo uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
	__vo uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
	__vo uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
	__vo  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
	__vo  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
	__vo  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
	__vo  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
	__vo  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
	uint32_t RESERVED0[5U];
	__vo uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;
#define SCB	((SCB_Type*) SCB_BASE_ADDR)

/*!< Interrupt Number Definition */
typedef enum
{
	/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
	NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
	HardFault_IRQn              = -13,    /*!< 3 Cortex-M3 Hard Fault Interrupt                     */
	MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt              */
	BusFault_IRQn               = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
	UsageFault_IRQn             = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
	SVCall_IRQn                 = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                       */
	DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
	PendSV_IRQn                 = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                       */
	SysTick_IRQn                = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                   */

	/******  STM32 specific Interrupt Numbers *********************************************************/
	WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                            */
	PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt            */
	TAMPER_IRQn                 = 2,      /*!< Tamper Interrupt                                     */
	RTC_IRQn                    = 3,      /*!< RTC global Interrupt                                 */
	FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                               */
	RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                 */
	EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                 */
	EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                 */
	EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                 */
	EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                 */
	EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                 */
	DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                      */
	DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                      */
	DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                      */
	DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                      */
	DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                      */
	DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                      */
	DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                      */
	ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
	USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
	USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
	CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
	CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
	EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
	TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
	TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
	TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
	TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
	TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
	TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
	TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
	I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
	I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
	I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
	I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
	SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
	SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
	USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
	USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
	USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
	EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
	RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
	USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
} IRQn_Type;



/***************************END:Processor Specific Detail***************************/

#define HSI_CLK_FREQ	8000000	//8MHz

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U		// only one SRAM for STM32F101xx
#define ROM_BASEADDR				0x1FFFF000U		// referred to as system memory in datasheet
#define SRAM 						SRAM1_BASEADDR	// main SRAM is SRAM1

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR				0x40000000U	//Base address of peripheral registers
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR	//Base address of APB1 peripheral registers
#define APB2PERIPH_BASEADDR			0x40010000U	//Base address of APB2 peripheral registers
#define AHB1PERIPH_BASEADDR			0x40018000U	//Base address of AHB peripheral register (note that STM32F101xx only has one AHB, other devices could have multiple)

/*
 * Base addresses of peripherals which are hanging off AHB1 bus
 */
#define SDIO_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define DMA1_BASEADDR				(AHB1PERIPH_BASEADDR + 0x8000)
#define DMA2_BASEADDR				(AHB1PERIPH_BASEADDR + 0x8400)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x9000)
#define FLASHINT_BASEADDR			(AHB1PERIPH_BASEADDR + 0xA000)
#define CRC_BASEADDR				(AHB1PERIPH_BASEADDR + 0xB000)
#define ETHERNET_BASEADDR			(AHB1PERIPH_BASEADDR + 0x10000)
#define USBOTGFS_BASEADDR			(AHB1PERIPH_BASEADDR + 0xFFE8000)
#define FSMC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x5FFE8000)

/*
 * Base addresses of peripherals which are hanging off APB1 bus
 */
#define TIM2_BASEADDR				(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR				(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR				(APB1PERIPH_BASEADDR + 0x0800)
// TIM5 -> TIM14 not routed on STM32F101xx so not included
#define RTC_BASEADDR				(APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x2C00)
#define IWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x3000)
#define	SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define USBFS_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)
#define USBCAN_BASEADDR				(APB1PERIPH_BASEADDR + 0x6000)
#define bxCAN2_BASEADDR				(APB1PERIPH_BASEADDR + 0x6800)
#define bxCAN1_BASEADDR				(APB1PERIPH_BASEADDR + 0x6400)
#define BKP_BASEADDR				(APB1PERIPH_BASEADDR + 0x6C00)
#define PWR_BASEADDR				(APB1PERIPH_BASEADDR + 0x7000)
#define DAC_BASEADDR				(APB1PERIPH_BASEADDR + 0x7400)

/*
 * Base addresses of peripherals which are hanging off APB2 bus
 */
#define AFIO_BASEADDR				(APB2PERIPH_BASEADDR + 0x0000)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x0400)
#define GPIOA_BASEADDR				(APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR				(APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR				(APB2PERIPH_BASEADDR + 0x1800)
// STM32F101xx does not have GPIOC-G routed so skipped
#define ADC1_BASEADDR				(APB2PERIPH_BASEADDR + 0x2400)
#define ADC2_BASEADDR				(APB2PERIPH_BASEADDR + 0x2800)
#define TIM1_BASEADDR				(APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)
#define ADC3_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
// STM32F101xx does not have TIM8, TIM9, TIM10, TIM11 so skipped
#define DBGMCU_BASEADDR						0xE0042000U

/*************************peripheral register definition structures*************************/
/*
 * Note:
 * These register structures should be defined as volatile because the contents of the registers can change on each clock cycle
 * instead of writing 'volatile' the shorthand version '__vo' can be used as we defined it earlier in this header file
 */

typedef struct {
	__vo uint32_t CR;			// Clock control register
	__vo uint32_t CFGR;			// Clock configuration register
	__vo uint32_t CIR;			// Clock interrupt register
	__vo uint32_t APB2RSTR;		// APB2 peripheral reset register
	__vo uint32_t APB1RSTR;		// APB1 peripheral reset register
	__vo uint32_t AHBENR;		// AHB Peripheral Clock enable register
	__vo uint32_t APB2ENR;		// APB2 peripheral clock enable register
	__vo uint32_t APB1ENR;		// APB1 peripheral clock enable register
	__vo uint32_t BDCR;			// Backup domain control register
	__vo uint32_t CSR;			// Control/status register
	__vo uint32_t AHBSTR;		// AHB peripheral clock reset register
	__vo uint32_t CFGR2;		// Clock configuration register2
}RCC_RegDef_t;

typedef struct {
	__vo uint32_t CR[2];		// CR[0] Port configuration register low, CR[1] Port configuration register high					Address offset: 0x00
	__vo uint32_t IDR;			// Port input data register																			Address offset: 0x08
	__vo uint32_t ODR;			// Port output data register																		Address offset: 0x0C
	__vo uint32_t BSRR;			// Port bit set/reset register																		Address offset: 0x10
	__vo uint32_t BRR;			// Port bit reset register																			Address offset: 0x14
	__vo uint32_t LCKR;			// Port configuration lock register																	Address offset: 0x18
}GPIO_RegDef_t;

typedef struct {
	__vo uint32_t EVCR;			// Give a short description			Address offset: 0x00
	__vo uint32_t MAPR;			// Give a short description			Address offset: 0x04
	__vo uint32_t EXTICR[4];		// EXTICR1[0]...EXTICR4[3]			Address offset: 0x08
	__vo uint32_t MAPR2;			// Give a short description			Address offset: 0x1C
}AFIO_RegDef_t;


typedef struct {
	__vo uint32_t CR[2];		// Control Register 1 & 2		Address offset: 0x00
	__vo uint32_t SMCR;			// Slave mode control register	Address offset: 0x08
	__vo uint32_t DIER;			// DMA/Interrupt enable reg		Address offset: 0x0C
	__vo uint32_t SR;			// Status register				Address offset: 0x10
	__vo uint32_t EGR;			// Event Generation Register	Address offset: 0x14
	__vo uint32_t CCMR[2];		// Capture/Compare mode reg		Address offset: 0x18
	__vo uint32_t CCER;			// Capture/Compare enable reg	Address offset: 0x20
	__vo uint32_t CNT;			// Counter						Address offset: 0x24
	__vo uint32_t PSC;			// Prescaler					Address offset: 0x28
	__vo uint32_t ARR;			// Auto-reload register			Address offset: 0x2C
	__vo uint32_t RCR;			// Repetition Counter Register	Address offset: 0x30
	__vo uint32_t CCR[4];		// Capture/compare register		Address offset: 0x34
	__vo uint32_t BDTR;			// break and dead time reg		Address offset: 0x44
	__vo uint32_t DCR;			// DMA control register			Address offset: 0x48
	__vo uint32_t DMAR;			// DMA address for full trnsfr	Address offset: 0x4C
}TIM_RegDef_t;

typedef struct {
	__vo uint32_t IMR;			// Interrupt Mask Register
	__vo uint32_t EMR;			// Event Mask Register
	__vo uint32_t RTSR;			// Rising Trigger Selection Register
	__vo uint32_t FTSR;			// Falling Trigger Selection Register
	__vo uint32_t SWIER;		// Software Interrupt Event Register
	__vo uint32_t PR;			// Pending Register
}EXTI_RegDef_t;

typedef struct {
	__vo uint32_t CR[2];		//SPI Control Register 1 & 2
	__vo uint32_t SR;			//SPI Status Register
	__vo uint32_t DR;			//SPI Data register
	__vo uint32_t CRCPR;		//SPI CRC Polynomial register
	__vo uint32_t RXCRCR;		//SPI RX CRC Register
	__vo uint32_t TXCRCR;		//SPI TX CRC Register
	__vo uint32_t I2SCFGR;		//SPI_I2S Configuration register
	__vo uint32_t I2SPR;		//SPI_I2S Prescaler register
}SPI_RegDef_t;

typedef struct {
	__vo uint32_t CR1;			//I2C Control Register 1
	__vo uint32_t CR2;			//I2C Control Register 2
	__vo uint32_t OAR[2];		//I2C Own Address Register 1 & 2
	__vo uint32_t DR;			//I2C Data Register
	__vo uint32_t SR[2];		//I2C Status Register 1 & 2
	__vo uint32_t CCR;			//I2C Clock Control Register
	__vo uint32_t TRISE;		//I2C TRISE Register (maximum rise time in Fm/Sm mode
}I2C_RegDef_t;


typedef struct {
	__vo uint32_t SR;			//USART Status Register
	__vo uint32_t DR;			//USART Data Register
	__vo uint32_t BRR;			//USART Baud Rate Register
	__vo uint32_t CR1;			//USART Control Register 1
	__vo uint32_t CR2;			//USART Control Register 2
	__vo uint32_t CR3;			//USART Control Register 3
	__vo uint32_t GTPR;			//USART Guard Time and prescaler register
}USART_RegDef_t;

typedef struct {
	__vo uint32_t IDCODE;
	__vo uint32_t CR;
}DBGMCU_RegDef_t;

/*
 * peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define RCC		((RCC_RegDef_t*)	 RCC_BASEADDR)

#define GPIOA	((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define AFIO	((AFIO_RegDef_t*) AFIO_BASEADDR)

#define TIM1	((TIM_RegDef_t*) TIM1_BASEADDR)
#define TIM2	((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM3	((TIM_RegDef_t*) TIM3_BASEADDR)
#define TIM4	((TIM_RegDef_t*) TIM4_BASEADDR)

#define EXTI	((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SPI1	((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*) SPI2_BASEADDR)

#define I2C1	((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t*) I2C2_BASEADDR)

#define USART1	((USART_RegDef_t*) USART1_BASEADDR)
#define USART2	((USART_RegDef_t*) USART2_BASEADDR)
#define USART3	((USART_RegDef_t*) USART3_BASEADDR)

#define DBGMCU	((DBGMCU_RegDef_t*) DBGMCU_BASEADDR)
/*
 * Clock Enable/Disable/Reset Macros for GPIOx peripheral
 */
#define GPIOA_PCLK_EN()	(RCC->APB2ENR |= (0x1 << 2))	// Enable GPIOA
#define GPIOB_PCLK_EN()	(RCC->APB2ENR |= (0x1 << 3))	// Enable GPIOB
#define GPIOA_PCLK_DI()	(RCC->APB2ENR &= ~(0x1 << 2))	// Disable GPIOA
#define GPIOB_PCLK_DI()	(RCC->APB2ENR &= ~(0x1 << 3))	// Disable GPIOB
#define GPIOA_REG_RESET() do{ (RCC->APB2RSTR |= (0x1 << 2)); (RCC->APB2RSTR &= ~(0x1 << 2)); }while(0)	// Issue command to reset GPIOA, then issue command to take out of reset
#define GPIOB_REG_RESET() do{ (RCC->APB2RSTR |= (0x1 << 3)); (RCC->APB2RSTR &= ~(0x1 << 3)); }while(0)	// Issue command to reset GPIOB, then issue command to take out of reset

/*
 * Returns port code for given GPIOx base address
 * written in Conditional Or Ternary notation ":\" = else if
 */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :0)

/*
 * Clock enable/Disable macro for AFIO peripheral
 */
#define AFIO_PCLK_EN()	(RCC->APB2ENR |= (0x1 << 0))	// Enable AFIO
#define AFIO_PCLK_DI()	(RCC->APB2ENR &= ~(0x1 << 0))	// Disable AFIO

/*
 * Clock enable/Disable macros for TIMER peripherals
 */
#define TIM1_PCLK_EN()	(RCC->APB2ENR |= (0x1 << 11))	// Enable TIMER1
#define TIM2_PCLK_EN()	(RCC->APB1ENR |= (0x1 << 0))	// Enable TIMER2
#define TIM3_PCLK_EN()	(RCC->APB1ENR |= (0x1 << 1))	// Enable TIMER3
#define TIM4_PCLK_EN()	(RCC->APB1ENR |= (0x1 << 2))	// Enable TIMER4
#define TIM1_PCLK_DI()	(RCC->APB2ENR &= ~(0x1 << 11))	// Disable TIMER1
#define TIM2_PCLK_DI()	(RCC->APB1ENR &= ~(0x1 << 0))	// Disable TIMER2
#define TIM3_PCLK_DI()	(RCC->APB1ENR &= ~(0x1 << 1))	// Disable TIMER3
#define TIM4_PCLK_DI()	(RCC->APB1ENR &= ~(0x1 << 2))	// Disable TIMER4

/*
 * Clock enable/Disable macros for SPI peripheral
 */
#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (0x1 << 12))	// Enable SPI1
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (0x1 << 14))	// Enable SPI2
#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(0x1 << 12))	// Disable SPI1
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(0x1 << 14))	// Disable SPI2

/*
 * Clock enable/Disable/reset macros for I2C peripheral
 */
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (0x1 << 21))	// Enable I2C1
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (0x1 << 22))	// Enable I2C2
#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(0x1 << 21))	// Disable I2C1
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(0x1 << 22))	// Disable I2C2
#define I2C1_RST()		(RCC->APB1RSTR |= (0x1 << 21))	// Reset I2C1
#define I2C2_RST()		(RCC->APB1RSTR |= (0x1 << 22))	// Reset I2C1

/*
 * Clock enable/Disable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (0x1 << 14))	// Enable USART1
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (0x1 << 17))	// Enable USART2
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (0x1 << 18))	// Enable USART3
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(0x1 << 14))	// Disable USART1
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(0x1 << 17))	// Disable USART2
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(0x1 << 18))	// Disable USART3

/*
 * Macro for checking if debugger connected (for SWV outputs)
 */
#define DEBUGGER_CONNECTED()	(DBGMCU->CR & 0x7)

// Generic Macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET	RESET
#define FLAG_SET	SET

/* =========RCC Bit macros========= */
// bit positions
#define RCC_CFGR_SWS			2
#define RCC_CFGR_HPRE			4
#define RCC_CFGR_PPRE1			8
#define RCC_CFGR_PLLSRC			16
#define RCC_CFGR_PLLXTPRE		17
#define RCC_CFGR_PLLMUL			18

// masks
#define RCC_CFGR_SWS_MASK		0xC
#define RCC_CFGR_HPRE_MASK		0xF0
#define RCC_CFGR_PPRE1_MASK		0x700
#define RCC_CFGR_PLLMUL_MASK	0x3C0000
/* ================================ */

/* =========SPI Bit macros========= */
#define	SPI_CR1_BIDIMODE	15
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_CRCEN		13
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_DFF			11
#define SPI_CR1_RXONLY		10
#define SPI_CR1_SSM			9
#define SPI_CR1_SSI			8
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SPE			6
#define SPI_CR1_BR			3
#define SPI_CR1_MSTR		2
#define SPI_CR1_CPOL		1
#define SPI_CR1_CPHA		0

#define SPI_CR2_TXEIE		7
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_ERRIE		5
#define SPI_CR2_SSOE		2
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_RXDMAEN		0

#define SPI_SR_BSY			7
#define SPI_SR_OVR			6
#define SPI_SR_MODF			5
#define SPI_SR_CRCERR		4
#define SPI_SR_UDR			3
#define SPI_SR_CHSIDE		2
#define SPI_SR_TXE			1
#define SPI_SR_RXNE			0
/* ================================ */

/* =========I2C Bit macros========= */
#define I2C_CR1_SWRST		15
#define I2C_CR1_ALERT		13
#define I2C_CR1_PEC			12
#define I2C_CR1_POS			11
#define I2C_CR1_ACK			10
#define I2C_CR1_STOP		9
#define I2C_CR1_START		8
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_ENGC		6
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENARP		4
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_SMBUS		1
#define I2C_CR1_PE			0

#define I2C_CR2_LAST		12
#define I2C_CR2_DMAEN		11
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITERREN		8
#define I2C_CR2_FREQ		0

#define I2C_OAR1_ADDMODE	15
#define I2C_OAR1_ADDR7BIT	1
#define I2C_OAR1_ADDR10BIT	0

#define I2C_OAR2_ADDR2		1
#define I2C_OAR2_ENDUAL		0

#define I2C_SR1_SMBALERT	15
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_PECERR		12
#define I2C_SR1_OVR			11
#define I2C_SR1_AF			10
#define I2C_SR1_ARLO		9
#define I2C_SR1_BERR		8
#define I2C_SR1_TxE			7
#define I2C_SR1_RxNE		6
#define I2C_SR1_STOPF		4
#define I2C_SR1_ADD10		3
#define I2C_SR1_BTF			2
#define I2C_SR1_ADDR		1
#define I2C_SR1_SB			0

#define I2C_SR2_PEC			8
#define I2C_SR2_DUALF		7
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_GENCALL		4
#define I2C_SR2_TRA			2
#define I2C_SR2_BUSY		1
#define I2C_SR2_MSL			0

#define I2C_CCR_FS			15
#define I2C_CCR_DUTY		14
#define I2C_CCR_CCR			0

#define I2C_TRISE_TRISE		0
/* ================================ */

/* =========USART Bit macros========= */
// bit positions
#define USART_SR_POS_PE		0
#define USART_SR_POS_FE		1
#define USART_SR_POS_NE		2
#define USART_SR_POS_ORE	3
#define USART_SR_POS_IDLE	4
#define USART_SR_POS_RXNE	5
#define USART_SR_POS_TC		6
#define USART_SR_POS_TXE	7
#define USART_SR_POS_LBD	8
#define USART_SR_POS_CTS	9

#define USART_BRR_POS_DIVfraction	0
#define USART_BRR_POS_DIVMantissa	4

#define USART_CR1_POS_SBK		0
#define USART_CR1_POS_RWU		1
#define USART_CR1_POS_RE		2
#define USART_CR1_POS_TE		3
#define USART_CR1_POS_IDLEIE	4
#define USART_CR1_POS_RXNEIE	5
#define USART_CR1_POS_TCIE		6
#define USART_CR1_POS_TXEIE		7
#define USART_CR1_POS_PEIE		8
#define USART_CR1_POS_PS		9
#define USART_CR1_POS_PCE		10
#define USART_CR1_POS_WAKE		11
#define USART_CR1_POS_M			12
#define USART_CR1_POS_UE		13

#define USART_CR2_POS_ADD		0
#define USART_CR2_POS_LBDL		5
#define USART_CR2_POS_LBDIE		6
#define USART_CR2_POS_LBCL		8
#define USART_CR2_POS_CPHA		9
#define USART_CR2_POS_CPOL		10
#define USART_CR2_POS_CLKEN		11
#define USART_CR2_POS_STOP		12
#define USART_CR2_POS_LINEN		14

#define USART_CR3_POS_EIE		0
#define USART_CR3_POS_IREN		1
#define USART_CR3_POS_IRLP		2
#define USART_CR3_POS_HDSEL		3
#define USART_CR3_POS_NACK		4
#define USART_CR3_POS_SCEN		5
#define USART_CR3_POS_DMAR		6
#define USART_CR3_POS_DMAT		7
#define USART_CR3_POS_RTSE		8
#define USART_CR3_POS_CTSE		9
#define USART_CR3_POS_CTSIE		10

#define USART_GTPR_POS_PSC		0
#define USART_GTPR_POS_GT		8
/* ================================ */

/* ================================ */
#include "stm32f101xx_gpio_driver.h"
#include "stm32f101xx_rcc_driver.h"
#include "stm32f101xx_afio_driver.h"
#include "stm32f101xx_tim_driver.h"
#include "stm32f101xx_spi_driver.h"
#include "stm32f101xx_i2c_driver.h"
#include "stm32f101xx_usart_driver.h"

/**************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************/
void SYST_Config(uint32_t SystemClockHz);
void DELAY_MS(uint32_t ms);
void SysTick_Handler(void);
uint32_t ELAPSED_MS(void);
void SYST_STARTorSTOP(uint8_t ENorDI);
void NVIC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void NVIC_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);

#endif /* STM32F101XX_H_ */
