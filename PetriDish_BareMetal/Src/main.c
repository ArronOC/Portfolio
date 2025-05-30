/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Arron O'Connor
 * @brief          : Main program body
 ******************************************************************************
 * This application was developed for the STM32_Comms_Board Rev #2
 * It utilises self written bare metal drivers for
 * 		AFIO
 * 		GPIO
 * 		I2C
 * 		RCC
 * 		SPI
 * 		TIMER
 * 		USART
 *
 * A console to control this board can be establish by connecting to the UART header with the following settings
 * 	baud rate 				= 115200
 * 	parity 					= none
 * 	data bits 				= 8
 * 	stop bits 				= 1
 * 	hardware flow control	= none
 * the device expects to see a \r\n termination on commands sent
 *
 * The main function of the application is to update the LEDs positioned on the
 * edge of the board based on the angle of tilt the accelerometer is experiencing.
 * The intensity of the LEDs increases as the angle increases. To picture this
 * think of a ball rolling in a petri dish, the intensity shows how close to the
 * edge the ball is, and which LED is on determines which direction the ball is rolling
 *
 * An I2C Temperature Sensor can be triggered for a readout using the UART command TEMP
 *
 * Serial information and accelerometer offsets are stored in a SPI EEPROM
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>

#include "main.h"
#include "stm32f101xx.h"

GPIO_Handle_t 		BootLED, nCS0, nHOLD, nWP, ACC_INT1;
ADXL343_Handle_t	Accelerometer;
TIM_Handle_t 		htim1, htim2, htim3, htim4;
I2C_Handle_t 		hi2c1;
USART_Handle_t		husart2;
uint32_t 			SysClk, APB1Clk, APB2Clk, ADCClk;
uint32_t 			PERIOD = 1000;	// Maximum value timer is set to for peak brightness of LEDs
char msgTxBuff[150];
char uartRxBuff[20];
char command[20];

int main(void)
{
	/* =================Initialise variables/Handles========================= */
	RCC_SysConfig_t		SystemClockConfiguration;
	RCC_Prescalers_t 	ClockPrescalers;
	I2C_ERROR_CODE 		I2C_Status = I2C_NO_ERROR;
	SPI_Handle_t		hspi1;
	CAV25M01_Handle_t 	EEPROM;
	LM75B_Handle_t 		TempSensor;
	char ClkSrc[4];
	char rdSerialNumber[SerNumSizeBytes];
	char BlankSerial[SerNumSizeBytes];
	uint8_t PetriMode = ENABLE;			// Enables petri dish mode by default
	uint8_t AccelMsgs = DISABLE;		// suppress accelerometer readouts over UART by default
	uint8_t Xoff = 0, Yoff = 0, Zoff = 0, AccelData[6];
	float tempC = 0.0f;

	memset(command, '\0', sizeof(command));

	htim1.pTIMx = TIM1;							// Set handle1 to use TIMER1
	htim1.init.Period = PERIOD;					// Set PERIOD
	htim1.init.EnableCH1 = ENABLE;				// SE LED
	htim1.init.EnableCH2 = ENABLE;				// E LED
	htim1.init.EnableCH3 = ENABLE;				// NE LED
	htim1.init.EnableCH4 = DISABLE;				// Not used

	htim2.pTIMx = TIM2;							// Set handle2 to use TIMER2
	htim2.init.Period = PERIOD;					// Set PERIOD
	htim2.init.EnableCH1 = ENABLE;				// N LED
	htim2.init.EnableCH2 = DISABLE;				// Not used
	htim2.init.EnableCH3 = ENABLE;				// SW LED
	htim2.init.EnableCH4 = ENABLE;				// S LED

	htim3.pTIMx = TIM3;							// Set handle3 to use TIMER3
	htim3.init.Period = PERIOD;					// Set PERIOD
	htim3.init.EnableCH1 = DISABLE;				// Not used
	htim3.init.EnableCH2 = DISABLE;				// Not used
	htim3.init.EnableCH3 = DISABLE;				// Not used
	htim3.init.EnableCH4 = ENABLE;				// W LED

	htim4.pTIMx = TIM4;							// Set handle4 to use TIMER4
	htim4.init.Period = PERIOD;				 	// Set PERIOD
	htim4.init.EnableCH1 = ENABLE;				// NW LED
	htim4.init.EnableCH2 = DISABLE;				// Not used
	htim4.init.EnableCH3 = DISABLE;				// Not used
	htim4.init.EnableCH4 = DISABLE;				// Not used

	hspi1.pSPI = SPI1;
	hspi1.SPIConfig.SPI_BusConfig.DataWidth = SPI_DATAFORMAT8BITS;
	hspi1.SPIConfig.SPI_BusConfig.LSBFirst = SPI_TX_MSBFIRST;
	hspi1.SPIConfig.SPI_ClockPhase = SPI_CPHA_FIRST;
	hspi1.SPIConfig.SPI_ClockPolarity = SPI_CPOL_IDLELOW;
	hspi1.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	hspi1.SPIConfig.SPI_SSM = SPI_SSM_EN;
	// SPI Clock speed is configured later depending on which clock source is used for sysclk

	hi2c1.pI2Cx = I2C1;
	hi2c1.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_FM4K;		// Fast mode 400kHz
	hi2c1.I2C_Config.I2C_DutyCycle = I2C_FM_DUTY_2;
	hi2c1.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLED;
	hi2c1.I2C_Config.I2C_AddrMode = I2C_7BIT_SLAVE;
	hi2c1.I2C_Config.I2C_DualAddrEn = I2C_DUAL_ADDRESS_DI;
	hi2c1.I2C_Config.I2C_OwnAddr1 = 0;						// Unit is I2C master, no address

	husart2.pUSARTx = USART2;
	husart2.USART_Config.BaudRate = USART_BaudRate_BPS_115200;
	husart2.USART_Config.HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	husart2.USART_Config.Mode = USART_MODE_TXRX;
	husart2.USART_Config.NumStopBits = USART_STOPBITS_1;
	husart2.USART_Config.ParityControl = USART_PARITY_NONE;
	husart2.USART_Config.WordLength = USART_WORDLEN_8BITS;
	husart2.TxBusyState = USART_READY;
	husart2.RxBusyState = USART_READY;

	EEPROM.pSPIhandle = &hspi1;
	EEPROM.pnCS0 = &nCS0;
	EEPROM.pnHOLD = &nHOLD;
	EEPROM.pnWP = &nWP;

	TempSensor.i2cHandle = &hi2c1;
	TempSensor.Addr = LM75B_Addr_000;

	Accelerometer.i2cHandle = &hi2c1;
	Accelerometer.SlaveAddr = ADXL343_Addr_High;
	Accelerometer.NewData = nREADY;
	/* ================================================================ */

	/* =====================Clock Configuration======================== */
	/*
	 * Configures all system buses
	 * PLL @ 48MHz is the default, but HSE @ 16MHz can also be used
	 */
	if (0)	// Use 16MHz HSE as system clock (was used during debug)
	{
		SystemClockConfiguration.SysClkSrc = RCC_SW_HSE;	// Use HSE as sysclock source
		ClockPrescalers.APB1prescaler = RCC_PPRE1_1;		// Set APB1 prescaler /1 (16MHz)
		ClockPrescalers.ADCprescaler = RCC_ADCPRE1_4;		// Set ADC prescaler to /4
		ClockPrescalers.AHBprescaler = RCC_HPRE_1;			// Set AHB prescaler to /1
		ClockPrescalers.APB2prescaler = RCC_PPRE2_1;		// Set APB2 prescaler to /1 (16MHz)
		SysClk = HSE_OSC_FREQ;
		APB1Clk = HSE_OSC_FREQ;
		APB2Clk = HSE_OSC_FREQ;
		ADCClk = APB2Clk / 4;
		hspi1.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV4;		// 4MHz operation when sysclk = 16MHz
		husart2.USART_Config.APBClock = APB1Clk;
		strcpy(ClkSrc, "HSE");
	}
	if (1)	// Use 48MHz PLL as system clock
	{
		SystemClockConfiguration.SysClkSrc = RCC_SW_PLL;					// Set PLL to be the system clock source
		SystemClockConfiguration.PllConfig.PLLsrc = RCC_PLL_SRC_HSE;		// Set HSE to the be clock source for PLL (16MHz)
		SystemClockConfiguration.PllConfig.PLLHSEDiv = RCC_PLL_HSE_DIV_2;	// Set HSE divider to /2 (16/2=8MHz)
		SystemClockConfiguration.PllConfig.PLLMul= RCC_PLL_MUL_6;			// Set PLL multiplier to x6 (8*6 = 48MHz)
		ClockPrescalers.APB1prescaler = RCC_PPRE1_2;						// Set APB1 prescaler /2 (24MHz)
		ClockPrescalers.ADCprescaler = RCC_ADCPRE1_4;						// Set ADC prescaler to /4
		ClockPrescalers.AHBprescaler = RCC_HPRE_1;							// Set AHB prescaler to /1
		ClockPrescalers.APB2prescaler = RCC_PPRE2_1;						// Set APB2 prescaler to /1 (48MHz)
		SysClk = 48000000;
		APB1Clk = 24000000;
		APB2Clk = SysClk;
		ADCClk = APB2Clk / 4;
		hspi1.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV8;	// 6MHz operation when sysclk = 48MHz
		husart2.USART_Config.APBClock = APB1Clk;
		strcpy(ClkSrc, "PLL");
	}
	RCC_Init_SysClk(&SystemClockConfiguration);	// Configure system clock
	RCC_Init_AHBClk(&ClockPrescalers);			// Configure AHB clock
	RCC_Init_PCLK1(&ClockPrescalers);			// Configure PCLK1/APB1
	RCC_Init_PCLK2(&ClockPrescalers);			// Configure PCLK2/APB2
	RCC_Init_ADCCLK(&ClockPrescalers);			// Configure ADCCLK (not actually used in this application)
	SYST_Config(SysClk);													// setup SysTick to provide exceptions on 1ms intervals
	/* ================================================================ */

	/* =====================GPIO Configuration========================= */
	GPIO_PeriClockControl(GPIOA, ENABLE);		// Enables GPIOA clock
	GPIO_PeriClockControl(GPIOB, ENABLE);		// Enables GPIOB clock
	AFIO_PeriEn(ENABLE);						// Enables AFIO clock
	AFIO_REMAP();								// Re-maps certain pins to their alternate functions
	GPIO_INIT_ALL();							// Configures all GPIO pins to their required GPIO/AFIO configuration
	/* ================================================================= */

	/* =====================USART Configuration========================= */
	/*
	 * Configure USART2 peripheral
	 * Console message to indicate device is booting
	 */
	USART_Init(&husart2);									// Configure USART2
	NVIC_IRQITConfig(USART2_IRQn, ENABLE);					// Enables USART2 interrupts
	UserMessages(&husart2, " Device Booting...", ENABLE);	// Console messages over SWV (debugger connected) and/or UART interface
	/* ================================================================= */

	/* ================System Clock Console Messages==================== */
	/*
	 * Console messages stating clock speeds of various busses
	 * Note that this can't happen until UART is active hence why it is later in application
	 */
	sprintf(msgTxBuff, "\r\nClock Configuration:\r\n"
			"System Clock Source\t%s", ClkSrc);
	UserMessages(&husart2, msgTxBuff, ENABLE);

	sprintf(msgTxBuff, "System Clock Frequency\t%luMHz", (SysClk/1000000));
	UserMessages(&husart2, msgTxBuff, ENABLE);

	sprintf(msgTxBuff, "AHB Clock Frequency\t%luMHz",(SysClk/1000000));
	UserMessages(&husart2, msgTxBuff, ENABLE);

	sprintf(msgTxBuff, "APB1 Clock Frequency\t%luMHz", (APB1Clk/1000000));
	UserMessages(&husart2, msgTxBuff, ENABLE);

	sprintf(msgTxBuff, "APB2 Clock Frequency\t%luMHz",(APB2Clk/1000000));
	UserMessages(&husart2, msgTxBuff, ENABLE);

	sprintf(msgTxBuff, "ADC Clock Frequency\t%luMHz",(ADCClk/1000000));
	UserMessages(&husart2, msgTxBuff, ENABLE);
	/* ================================================================= */

	/* ===================GPIO/AFIO Console Messages==================== */
	/*
	 * Console messages stating GPIO/AFIO configuration
	 * Note that this can't happen until UART is active hence why it is later in application
	 */
	sprintf(msgTxBuff, "\r\nGPIO Configuration:\r\n"
			"GPIOA Enabled");
	UserMessages(&husart2, msgTxBuff, ENABLE);

	UserMessages(&husart2, "GPIOB Enabled", ENABLE);

	UserMessages(&husart2, "AFIO Enabled", ENABLE);

	sprintf(msgTxBuff,	"Pin Re-mapping\r\n"
			"PA15 Released from JTDI\r\n"
			"PB4 Released from nJTRST\r\n"
			"TIM2 Pins fully re-mapped\r\n"
			"I2C1 Re-mapped to PB8/PB9");
	UserMessages(&husart2, msgTxBuff, ENABLE);

	sprintf(msgTxBuff, "GPIO Pins initialised\r\n\r\n"
			"Initialising Peripherals...");
	UserMessages(&husart2, msgTxBuff, ENABLE);
	/* ================================================================= */

	/* =====================TIMER Configuration========================= */
	/*
	 * Configure TIMER1 -> 4 peripherals
	 * Console messages to give status updates
	 */
	TIM_init(&htim1);
	UserMessages(&husart2, "TIMER1 Initialised", ENABLE);

	TIM_init(&htim2);
	UserMessages(&husart2, "TIMER2 Initialised", ENABLE);

	TIM_init(&htim3);
	UserMessages(&husart2, "TIMER3 Initialised", ENABLE);

	TIM_init(&htim4);
	UserMessages(&husart2, "TIMER4 Initialised", ENABLE);
	/* ================================================================= */


	/* ======================SPI Configuration========================== */
	/*
	 * Configure SPI1 peripheral
	 * Console messages to give status updates
	 */
	SPI_Init(&hspi1);
	UserMessages(&husart2, "SPI1 Initialised", ENABLE);
	/* ================================================================= */


	/* ======================I2C Configuration========================== */
	/*
	 * Configure I2C1 peripheral
	 * Enables the I2C1 error interrupt in the NVIC
	 * Console messages to give status updates
	 */
	I2C_Init(&hi2c1);
	NVIC_IRQITConfig(I2C1_ER_IRQn, ENABLE);
	UserMessages(&husart2, "I2C1 Initialised", ENABLE);
	/* ================================================================= */


	/* ===================== Initialise IC handles ===================== */
	/*
	 * Configures U4, U5 & U6 providing updates over the console
	 * application will hang here if configuration fails
	 */
	UserMessages(&husart2, "\r\nInitialising Devices...", ENABLE);

	// U6 SPI EEPROM (CAV25M01VE-GT3)
	if (CAV25M01_Init(&EEPROM))
	{
		// Configuration PASSED
		UserMessages(&husart2, "Communication to SPI EPPROM U6 PASSED", ENABLE);
	} else
	{
		// Configuration FAILED
		UserMessages(&husart2, "Communication to SPI EPPROM U6 FAILED", ENABLE);
		while(1); //Hang here due to failure
	}

	// U4 I2C Temperature Sensor (LM75BD,118)
	I2C_Status = LM75B_POR(&TempSensor);
	if ( I2C_Status == I2C_NO_ERROR)
	{
		// Configuration PASSED
		UserMessages(&husart2, "Temperature Sensor U4 Configured", ENABLE);
	} else
	{
		// Configuration FAILED
		UserMessages(&husart2, "Temperature Sensor U4 FAILED To Configure", ENABLE);
		I2C_ApplicationEventCallback(TempSensor.i2cHandle, I2C_Status);
	}

	// U5 I2C Accelerometer (ADXL343BCCZ-RL7)
	I2C_Status = ADXL343_Init(&Accelerometer);
	if ( I2C_Status == I2C_NO_ERROR )
	{
		// Configuration PASSED
		UserMessages(&husart2, "Accelerometer U5 Configured", ENABLE);

		// Setup the NVIC to accept the Data Ready interrupt from the accelerometer
		NVIC_IRQPriorityConfig(EXTI4_IRQn, 15); 				// Set EXTI4 interrupt priority
		NVIC_IRQITConfig(EXTI4_IRQn, ENABLE);					// Enable the ACC_INT1 interrupt
		I2C_Status = ADXL343_EnDataReadyInt(&Accelerometer); 	// Enable INT1 on ADXL343
		if ( I2C_Status == I2C_NO_ERROR )
		{
			// Interrupt configuration PASSED
			UserMessages(&husart2, "U5 DATA_READY (ACC_INT1) I2C Interrupt Enabled", ENABLE);
		} else
		{
			// Interrupt configuration FAILED
			UserMessages(&husart2, "Failed to enable U5 DATA_READY (ACC_INT1) I2C interrupt", ENABLE);
			I2C_ApplicationEventCallback(Accelerometer.i2cHandle, I2C_Status);
		}
	} else
	{
		// Configuration FAILED
		UserMessages(&husart2, "Accelerometer U5 FAILED to Configure", ENABLE);
		I2C_ApplicationEventCallback(Accelerometer.i2cHandle, I2C_Status);
	}
	/* ================================================================= */

	/* ========================== Application ========================== */
	/*
	 * Check if device has been initialised
	 * i.e. serial data written to EEPROM
	 */
	CAV25M01_SerNum(&EEPROM, rdSerialNumber, RECALL);
	if ( strncmp(rdSerialNumber, "STM32", 5) != 0 )		// check first 5 bytes for data
	{
		// Device not initialised, run SETUP procedure
		UserMessages(&husart2, "\r\nNo serial number stored on device, performing first time setup", ENABLE);
		SetupDevice(&EEPROM);

		// Retrieve serial number from EEPROM
		CAV25M01_SerNum(&EEPROM, rdSerialNumber, RECALL);
		UserMessages(&husart2, rdSerialNumber, ENABLE);		// reports serial number over console
		UserMessages(&husart2, "First time setup complete", ENABLE);
	}
	else
	{
		// Device already initialised, report serial number over console
		UserMessages(&husart2, rdSerialNumber, ENABLE);
	}

	// Begin UART Rx interrupt so commands can be received
	USART_ReceiveDataIT(&husart2, (uint8_t *)uartRxBuff, sizeof(uartRxBuff));

	sprintf(msgTxBuff, "\r\nDevice successfully booted\r\n"
			"Ready to receive commands");
	UserMessages(&husart2, msgTxBuff, ENABLE);

	/* Main loop body */
	while(1)
	{
		GPIO_ToggleOutputPin(BootLED.pGPIOx, BootLED.GPIO_PinConfig.GPIO_PinNumber);		// Toggles the green boot LED

		// Read accelerometer data and update LEDs if enabled & new data ready
		if (PetriMode == ENABLE)
		{
			if (Accelerometer.NewData == READY)
			{
				I2C_Status = ADXL343_ReadData(&Accelerometer, AccelData, &ACC_INT1);
				if (I2C_Status != I2C_NO_ERROR) I2C_ApplicationEventCallback(Accelerometer.i2cHandle, I2C_Status);
				ADXL343_PitchRollCalc(&Accelerometer, AccelData);
				LED_PetriDish(Accelerometer.Roll, Accelerometer.Pitch, Accelerometer.Zg);

				// report accelerometer data over console if enabled
				if (AccelMsgs == ENABLE)
				{
					sprintf(msgTxBuff, "roll: %.2f, pitch: %.2f, Zg: %.2f", Accelerometer.Roll, Accelerometer.Pitch, Accelerometer.Zg);
					UserMessages(&husart2, msgTxBuff, ENABLE);
				}
			}
		}
		else
		{
			// Petri mode disabled, turn LEDs off
			htim1.pTIMx->CCR[0] = 0;	// SE LED
			htim1.pTIMx->CCR[1] = 0;	// E LED
			htim1.pTIMx->CCR[2] = 0;	// NE LED
			htim2.pTIMx->CCR[2] = 0;	// SW LED
			htim2.pTIMx->CCR[3] = 0;	// S LED
			htim3.pTIMx->CCR[3] = 0;	// W LED
			htim4.pTIMx->CCR[0] = 0;	// NW LED
			htim2.pTIMx->CCR[0] = 0;	// N LED
		}

		if (strcasecmp(command,"") == 0)
		{
			// no command received, perform no action
		}
		else if (strcasecmp(command,"SETUP") == 0)
		{
			// Setup device serial number
			UserMessages(&husart2, "Entering setup mode", ENABLE);

			SetupDevice(&EEPROM);

			// Retrieve serial number from EEPROM & report over console
			CAV25M01_SerNum(&EEPROM, rdSerialNumber, RECALL);
			UserMessages(&husart2, rdSerialNumber, ENABLE);

			Command_Handling(&husart2, "Exiting setup mode");
		}
		else if (strcasecmp(command,"CALIBRATE") == 0)
		{
			// Calibrate accelerometer offsets
			sprintf(msgTxBuff, "Ensure device is flat\r\n"
					"Starting calibration procedure in");
			UserMessages(&husart2, msgTxBuff, ENABLE);

			// 5 second countdown
			for (uint8_t i = 5; i > 0; i--)
			{
				sprintf(msgTxBuff, "%u", i);
				UserMessages(&husart2, msgTxBuff, ENABLE);
				DELAY_MS(1000);
			}

			// calibrates X,Y,Z offsets
			I2C_Status = ADXL343_CalibrateFlat(&Accelerometer, &ACC_INT1);
			if (I2C_Status == I2C_NO_ERROR)
			{
				strcpy(msgTxBuff, "Accelerometer Calibrated");
				Command_Handling(&husart2, msgTxBuff);
			} else
			{
				strcpy(msgTxBuff, "Failed to Calibrate Accelerometer");
				Command_Handling(&husart2, msgTxBuff);
				I2C_ApplicationEventCallback(Accelerometer.i2cHandle, I2C_Status);
			}
		}
		else if (strcasecmp(command,"FEEDBACK OFF") == 0)
		{
			// Turn off leds updating based on accelerometer data
			PetriMode = DISABLE;
			Command_Handling(&husart2, "Accelerometer LED feedback disabled");
		}
		else if (strcasecmp(command,"FEEDBACK ON") == 0)
		{
			// Turn on leds updating based on accelerometer data
			PetriMode = ENABLE;
			Command_Handling(&husart2, "Accelerometer LED feedback enabled");
		}
		else if (strcasecmp(command,"DATA ON") == 0)
		{
			// Turn on accelerometer reporting over UART
			AccelMsgs = ENABLE;
			Command_Handling(&husart2, " ");
		}
		else if (strcasecmp(command,"DATA OFF") == 0)
		{
			// Turn off accelerometer reporting over UART
			AccelMsgs = DISABLE;
			Command_Handling(&husart2, "Disabled accelerometer readings reporting over UART");
		}
		else if (strcasecmp(command,"TEMP") == 0)
		{
			// take a temperature reading
			tempC = LM75B_ReadTemp(&TempSensor);
			sprintf(msgTxBuff,"Temperature: %.3fC", tempC);

			Command_Handling(&husart2, msgTxBuff);
		}
		else if (strcasecmp(command,"FORMAT") == 0)
		{
			// Delete serial information
			CAV25M01_SerNum(&EEPROM, BlankSerial, STORE);

			// Delete accelerometer offset data
			Xoff = 0;
			Yoff = 0;
			Zoff = 0;
			CAV25M01_AccData(&EEPROM, &Xoff, &Yoff, &Zoff, STORE);

			Command_Handling(&husart2, "Device formatted");
		}
		else
		{
			// unknown command entered, print list of available commands to console
			sprintf(msgTxBuff, "Unknown command entered\r\nList of available commands (case insensitive, termination = \r\n):");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			sprintf(msgTxBuff, "SETUP          Waits for entry of serial number and stores this in EEPROM (blank device enters this mode on power up)");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			sprintf(msgTxBuff, "CALIBRATE      Configures accelerometer flat level by configuring the X/Y/Z offsets");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			sprintf(msgTxBuff, "FEEDBACK OFF   Stops updating the LEDs based on accelerometer data, LEDs turn off");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			sprintf(msgTxBuff, "FEEDBACK ON    Starts updating the LEDs based on accelerometer data (on by default upon power up)");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			sprintf(msgTxBuff, "DATA ON        Accelerometer data reported over UART port (off by default upon power up)");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			sprintf(msgTxBuff, "DATA OFF       Stops accelerometer data from being reported over UART port");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			sprintf(msgTxBuff, "TEMP           Takes a temperature reading");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			sprintf(msgTxBuff, "FORMAT         Clears device of serial number & accelerometer offsets");
			UserMessages(&husart2, msgTxBuff,ENABLE);

			Command_Handling(&husart2, "Enter Command:");
		}


		DELAY_MS(200);
	}
}

/*****************************************************************
 * @fn			- GPIO_INIT_ALL
 *
 * @brief		- This functions configures all GPIO pins the required state for the STM32_Petridish board
 *
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		-
 */
void GPIO_INIT_ALL(void)
{
	GPIO_Handle_t GPIO;	//Generic GPIO handle to initialise pins

	// Configure PA0 (BootLED) as output push pull 2MHz
	BootLED.pGPIOx = GPIOA;
	BootLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	BootLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_2MHZ_OUT;
	BootLED.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_GEN_PP;
	GPIO_Init(&BootLED);

	// Configure PB4 (ACC_INT1) as input interrupt on rising edge
	ACC_INT1.pGPIOx = GPIOB;
	ACC_INT1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	ACC_INT1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	ACC_INT1.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_INP_RESET;
	GPIO_Init(&ACC_INT1);

	// Configure PB5 (ACC_INT2) as input interrupt on rising edge
	GPIO.pGPIOx = GPIOB;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIO.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_INP_RESET;
	GPIO_Init(&GPIO);

	/* ==================== TIM Definitions ==================== */
	// Configure PA8 (LEDse) for AFIO Output
	GPIO.pGPIOx = GPIOA;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_50MHZ_OUT;
	GPIO.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_ALT_PP;
	GPIO_Init(&GPIO);

	// Configure PA9 (LEDe) for AFIO Output
	GPIO.pGPIOx = GPIOA;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&GPIO);

	// Configure PA10 (LEDne) for AFIO Output
	GPIO.pGPIOx = GPIOA;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&GPIO);

	// Configure PA15 (LEDn) for AFIO Output
	GPIO.pGPIOx = GPIOA;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GPIO);

	// Configure PB6 (LEDnw) for AFIO Output
	GPIO.pGPIOx = GPIOB;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&GPIO);

	// Configure PB1 (LEDw) for AFIO Output
	GPIO.pGPIOx = GPIOB;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&GPIO);

	// Configure PB10 (LEDsw) for AFIO Output
	GPIO.pGPIOx = GPIOB;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&GPIO);

	// Configure PB11 (LEDs) for AFIO Output
	GPIO.pGPIOx = GPIOB;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&GPIO);

	/*==================================================================*/
	/* ======================== SPI Definitions ========================*/
	// Configure PA5 (SCK) for AFIO Output (SPI SCK)
	GPIO.pGPIOx = GPIOA;
	GPIO.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_ALT_PP;
	GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_50MHZ_OUT;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&GPIO);

	// Configure PA7 (MOSI)for AFIO Output (SPI MOSI)
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&GPIO);

	// Configure PA6 (MISO) as floating Input (AFIO Input) (SPI MISO)
	GPIO.pGPIOx = GPIOA;
	GPIO.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_INP_RESET;
	GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INP;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&GPIO);

	// Configure PB0 (nCS0) for GPIO Output
	nCS0.pGPIOx = GPIOB;
	nCS0.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_GEN_PP;
	nCS0.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_50MHZ_OUT;
	nCS0.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Init(&nCS0);
	GPIO_WriteToOutputPin(nCS0.pGPIOx, nCS0.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);

	// Configure PB12 (nWP) for GPIO Output
	nWP.pGPIOx = GPIOB;
	nWP.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_GEN_PP;
	nWP.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_50MHZ_OUT;
	nWP.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&nWP);
	GPIO_WriteToOutputPin(nWP.pGPIOx, nWP.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);

	// Configure PB13 (nHOLD) for GPIO Output
	nHOLD.pGPIOx = GPIOB;
	nHOLD.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_GEN_PP;
	nHOLD.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_50MHZ_OUT;
	nHOLD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&nHOLD);
	GPIO_WriteToOutputPin(nHOLD.pGPIOx, nHOLD.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
	/*==================================================================*/

	/*========================= I2C Definitions ========================*/
	// Configure PB8 (SCL) for AFIO Output (I2C SCL)
	GPIO.pGPIOx = GPIOB;
	GPIO.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_ALT_OD;
	GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_50MHZ_OUT;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&GPIO);

	// Configure PB9 (SCK) for AFIO bi-directional (I2C SCK)
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&GPIO);
	/*==================================================================*/

	/*======================== UART Definitions ========================*/
	// Configure PA2 (UART_TX) for AFIO Output
	GPIO.pGPIOx = GPIOA;
	GPIO.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_ALT_PP;
	GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_50MHZ_OUT;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&GPIO);
	// Configure PA3 (UART_RX) for Input
	GPIO.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_INP_RESET;
	GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INP;
	GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&GPIO);
	/*==================================================================*/
}

/*****************************************************************
 * @fn			- EXTI4_IRQHandler
 *
 * @brief		- Interrupt handler for EXTI4 (ACC_INT1)
 *
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- overrides weak definition in startup file
 */
void EXTI4_IRQHandler(void)
{
	// 1. Set DataReady Flag
	Accelerometer.NewData = READY;

	// 2. Clear the interrupt in EXTI
	GPIO_IRQHandling(GPIO_PIN_NO_4);
}


/*****************************************************************
 * @fn			- EXTI9_5_IRQHandler
 *
 * @brief		- Interrupt handler for EXTI9_5
 *
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- overrides weak definition in startup file
 */
void EXTI9_5_IRQHandler(void)
{
	// May need to add delay so that multiple triggers don't occur
	// Determine which EXTI line was triggered
	if ((EXTI->PR & 0x20) == 0x20)			// Check if EXTI5 was triggered (ACC_INT2, Not used in this application)
	{
		// handle the interrupt
		GPIO_IRQHandling(GPIO_PIN_NO_5);
		// perform function

	} else if ((EXTI->PR & 0x40) == 0x40)	// Check if EXTI6 was triggered
	{
		// handle the interrupt
		GPIO_IRQHandling(GPIO_PIN_NO_6);
		// perform function

	} else if ((EXTI->PR & 0x80) == 0x80)	// Check if EXTI7 was triggered
	{
		// handle the interrupt
		GPIO_IRQHandling(GPIO_PIN_NO_7);
		// perform function

	} else if ((EXTI->PR & 0x100) == 0x100)	// Check if EXTI8 was triggered
	{
		// handle the interrupt
		GPIO_IRQHandling(GPIO_PIN_NO_8);
		// perform function

	} else if ((EXTI->PR & 0x200) == 0x200)	// Check if EXTI9 was triggered
	{
		// handle the interrupt
		GPIO_IRQHandling(GPIO_PIN_NO_9);
		// perform function

	}
}

/*****************************************************************
 * @fn			- LED_ErrorState
 *
 * @brief		- This functions sets all LEDs to max brightness to show an error has occurred
 *
 * @param[in]	- none
 *
 * @return		- none
 *
 * @Note		- additional error states could be added with different LED combinations
 */
void LED_ErrorState(void)
{
	htim1.pTIMx->CCR[0] = PERIOD;	// SE LED
	htim1.pTIMx->CCR[1] = PERIOD;	// E LED
	htim1.pTIMx->CCR[2] = PERIOD;	// NE LED
	htim2.pTIMx->CCR[2] = PERIOD;	// SW LED
	htim2.pTIMx->CCR[3] = PERIOD;	// S LED
	htim3.pTIMx->CCR[3] = PERIOD;	// W LED
	htim4.pTIMx->CCR[0] = PERIOD;	// NW LED
	htim2.pTIMx->CCR[0] = PERIOD;	// N LED
}

/*****************************************************************
 * @fn			- LED_PetriDish
 *
 * @brief		- This functions sets the LED brightness based off the given accelerometer data
 *
 * @param[in]	- roll position of accelerometer
 * @param[in]	- pitch position of accelerometer
 * @param[in]	- Zg position of accelerometer
 *
 * @return		- none
 *
 * @Note		-
 */
void LED_PetriDish(float Roll, float Pitch, float Zg)
{
	int32_t maxBrightness = PERIOD,			// LED is in this state above 85 degrees tilt
			minBrightness = 0,				// LED is in this state if not tilting in this direction
			degBrightness = PERIOD / 90,	// how much LEDBrightness increases per degree of tilt
			botLED, midLED, topLED,
			rollBrightness, pitchBrightness;
	uint8_t posRoll, posPitch;
	float 	diagonalWeighting;

	// init LED values
	int32_t nLED = minBrightness;
	int32_t neLED = minBrightness;
	int32_t swLED = minBrightness;
	int32_t wLED = minBrightness;
	int32_t nwLED = minBrightness;
	int32_t sLED = minBrightness;
	int32_t eLED = minBrightness;
	int32_t seLED = minBrightness;


	if (Zg < invertedZg)
	{
		// board inverted, set all LEDs to maximum
		htim1.pTIMx->CCR[0] = maxBrightness;	// SE LED
		htim1.pTIMx->CCR[1] = maxBrightness;	// E LED
		htim1.pTIMx->CCR[2] = maxBrightness;	// NE LED
		htim2.pTIMx->CCR[2] = maxBrightness;	// SW LED
		htim2.pTIMx->CCR[3] = maxBrightness;	// S LED
		htim3.pTIMx->CCR[3] = maxBrightness;	// W LED
		htim4.pTIMx->CCR[0] = maxBrightness;	// NW LED
		htim2.pTIMx->CCR[0] = maxBrightness;	// N LED
		return;
	}
	else
	{
		// Determine direction of roll for active LEDs
		if (Roll > 0) posRoll = 1;
		else
		{
			posRoll = 0;
			Roll = Roll * -1;	// remove negative values to make maths easier
		}

		// Determine direction of pitch for active LEDs
		if (Pitch > 0) posPitch = 1;
		else
		{
			posPitch = 0;
			Pitch = Pitch * -1;	// remove negative values to make maths easier
		}

		rollBrightness = (int32_t)Roll * degBrightness;
		pitchBrightness = (int32_t)Pitch * degBrightness;
		diagonalWeighting = atan(Pitch/Roll) * 57.3; // *57.3 converts to degrees


		if (diagonalWeighting < 22.5f)		// i.e. tilting between SE & E with more E weighting
		{
			topLED = rollBrightness;
			midLED = pitchBrightness;
			botLED = minBrightness;
		}
		else if (diagonalWeighting < 45) 	// i.e. tilting between SE & E with more SE weighting
		{
			topLED = rollBrightness;
			midLED = rollBrightness + pitchBrightness;
			botLED = minBrightness;
		}
		else if (diagonalWeighting < 67.5f) // i.e. tilting between SE & S with more SE weighting
		{
			botLED = pitchBrightness;
			midLED = pitchBrightness + rollBrightness;
			topLED = minBrightness;
		}
		else 								// i.e. tilting between SE & S with more S weighting
		{
			botLED = pitchBrightness;
			midLED = rollBrightness;
			topLED = minBrightness;
		}


		if (posRoll && posPitch)		// SE Quadrant
		{
			eLED = topLED;
			sLED = botLED;
			seLED = midLED;
		}
		else if (posRoll && !posPitch)	// NE Quadrant
		{
			nLED = botLED;
			eLED = topLED;
			neLED = midLED;
		}
		else if (!posRoll && posPitch)	// SW Quadrant
		{
			wLED = topLED;
			sLED = botLED;
			swLED = midLED;
		}
		else if (!posRoll && !posPitch)	// NW Quadrant
		{
			nLED = botLED;
			wLED = topLED;
			nwLED = midLED;
		}

		// Set all LEDs
		htim1.pTIMx->CCR[0] = seLED;	// SE LED
		htim1.pTIMx->CCR[1] = eLED;		// E LED
		htim1.pTIMx->CCR[2] = neLED;	// NE LED
		htim2.pTIMx->CCR[2] = swLED;	// SW LED
		htim2.pTIMx->CCR[3] = sLED;		// S LED
		htim3.pTIMx->CCR[3] = wLED;		// W LED
		htim4.pTIMx->CCR[0] = nwLED;	// NW LED
		htim2.pTIMx->CCR[0] = nLED;		// N LED
	}
	return;
}

/*****************************************************************
 * @fn			- I2C1_ER_IRQHandler
 *
 * @brief		- Interrupt handler for I2C1 error events
 *
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		- overwrites weak definition in startup file
 */
void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&hi2c1);
}

/*****************************************************************
 * @fn			- I2C_ApplicationEventCallback
 *
 * @brief		- I2C error processor
 *
 * @param[in]	- I2C Handle of the given I2C peripheral
 * @param[in]	- Type of I2C error which occurred
 *
 * @return		-
 *
 * @Note		-
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CxHandle, uint8_t AppEv)
{
	if ( AppEv == I2C_BUS_ERROR)
	{
		UserMessages(&husart2, "I2C Bus error occured", DISABLE);
		// Generate STOP condition
		I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);

		LED_ErrorState();
		// Hang in infinite loop
		while(1);

	} else if ( AppEv == I2C_ACK_ERROR)
	{
		UserMessages(&husart2, "I2C Ack error occured", DISABLE);
		// Generate STOP condition
		I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);

		LED_ErrorState();
		// Hang in infinite loop
		while(1);

	} else if ( AppEv == I2C_ARLO_ERROR)
	{
		UserMessages(&husart2, "I2C ARLO error occured", DISABLE);
		// Generate STOP condition
		I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);

		LED_ErrorState();
		// Hang in infinite loop
		while(1);

	} else if ( AppEv == I2C_TIMEOUT_ERROR)
	{
		UserMessages(&husart2, "I2C Timeout error occured", DISABLE);
		// Generate STOP condition
		I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);

		LED_ErrorState();
		// Hang in infinite loop
		while(1);

	} else if ( AppEv == I2C_ERROR_OVR)
	{
		UserMessages(&husart2, "I2C OVR error occured", DISABLE);
		// Generate STOP condition
		I2C_GenerateSTOPCondition(pI2CxHandle->pI2Cx);

		LED_ErrorState();
		// Hang in infinite loop
		while(1);

	} else if ( AppEv == USER_INVALID_DEVICE)
	{
		UserMessages(&husart2, "I2C DEVICE ID is invalid", DISABLE);
		LED_ErrorState();
		// Hang in infinite loop
		while(1);

	} else if ( AppEv == USER_TIMEOUT)
	{
		UserMessages(&husart2, "I2C Blocking API timed out", DISABLE);
		LED_ErrorState();
		// Hang in infinite loop
		while(1);
	}
}

/*****************************************************************
 * @fn			- USART2_IRQHandler
 *
 * @brief		- Interrupt handler for USART2
 *
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		- overwrites weak definition in startup file
 */
void USART2_IRQHandler(void)
{
	USART_IRQHandling(&husart2);
}

/*****************************************************************
 * @fn			- USART_ApplicationEventCallback
 *
 * @brief		- This functions handles the type of interrupt which triggered
 *
 * @param[in]	- Handle for the given USART peripheral
 * @param[in]	- type of event which occurred
 *
 * @return		- none
 *
 * @Note		-
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
	if (ApEv == USART_EVENT_TX_CMPLT)
	{
		/*
		 * Transmission of data complete
		 * Set Tx State back to ready for next transmission
		 */
		pUSARTHandle->TxBusyState = USART_READY;
	} else if (ApEv == USART_EVENT_RX_CMPLT)
	{
		// Check if a \r\n (decimal 13 & 10) character has been received in last two bytes
		if ((pUSARTHandle->RxCRRcvd == RECEIVED) && (pUSARTHandle->RxLFRcvd == RECEIVED))
		{
			// \r\n received, treated as end of command

			// Copy contents of uartRxBuff to command, so that main loop can select correct condition
			memcpy(command,uartRxBuff,strlen(uartRxBuff) - 2 );
			// Remove \r\n from command
			command[strcspn(command, "\n")] = 0;
			command[strcspn(command, "\r")] = 0;

			//Reset uart Rx buffer
			memset(uartRxBuff, '\0', sizeof(uartRxBuff));
		} else
		{
			// unknown command entered, notify main loop
			strcpy(command, "Unknown");
			//Reset uart Rx buffer
			memset(uartRxBuff, '\0', sizeof(uartRxBuff));
		}
	} else if (ApEv == USART_EVENT_IDLE)
	{
		// IDLE error occurred, clear flag
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_IDLE);
	} else if (ApEv == USART_EVENT_CTS)
	{
		// CTS error occurred
		LED_ErrorState();
		while(1);
	} else if (ApEv == USART_EVENT_PE)
	{
		// Parity error occurred
		LED_ErrorState();
		while(1);
	} else if (ApEv == USART_ERR_FE)
	{
		// Frame error occurred
		LED_ErrorState();
		while(1);
	} else if (ApEv == USART_ERR_NE)
	{
		// Noise error occurred
		LED_ErrorState();
		while(1);
	} else if (ApEv == USART_ERR_ORE)
	{
		// Overrun error occurred
		LED_ErrorState();
		while(1);
	} else if (ApEv == USART_EVENT_RX_RCVD)
	{
		// Received a character
		if ((pUSARTHandle->RxCRRcvd == RECEIVED) && (pUSARTHandle->RxLFRcvd == RECEIVED))
		{
			//disable the RXNE
			pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_POS_RXNEIE );
			pUSARTHandle->RxBusyState = USART_READY;

			// \r\n received, treated as end of command
			// Copy contents of uartRxBuff to command, so that main loop can select correct condition
			memcpy(command,uartRxBuff,strlen(uartRxBuff));
			// Remove \r\n from command
			command[strcspn(command, "\n")] = 0;
			command[strcspn(command, "\r")] = 0;

			//Reset uart Rx buffer
			memset(uartRxBuff, '\0', sizeof(uartRxBuff));

			// Check if \r\n were the only characters received
			if (strcasecmp(command, "") == 0)
			{
				// Restart the UART receive interrupt
				USART_ReceiveDataIT(&husart2, (uint8_t *)uartRxBuff, sizeof(uartRxBuff));
			}

		}
	}
}

/*****************************************************************
 * @fn			- Command_Handling
 *
 * @brief		- This function deals with the cleanup and reporting over console when a command has been completed
 *
 * @param[in]	- Handle for the given USART peripheral
 * @param[in]	- end of command message to report over console
 *
 * @return		- none
 *
 * @Note		-
 */
void Command_Handling(USART_Handle_t *pUSARTHandle, char *txBuff)
{
	//Command complete, reset buffer
	memset(command, '\0', sizeof(command));

	// Send user message in interrupt mode
	UserMessages(pUSARTHandle, txBuff, DISABLE);

	// Restart the UART receive interrupt
	USART_ReceiveDataIT(pUSARTHandle, (uint8_t *)uartRxBuff, sizeof(uartRxBuff));
}

/*****************************************************************
 * @fn			- UserMessages
 *
 * @brief		- This function send messages via SWV (debugger connected) & USART
 *
 * @param[in]	- Handle for the given USART peripheral
 * @param[in]	- message to report over console
 * @param[in]	- UART transmission uses interrupt or blocking mode
 *
 * @return		- none
 *
 * @Note		- if the USART peripheral is currently busy transmitting, it will wait until that is completed
 */
void UserMessages(USART_Handle_t *pUSARTHandle, char *txBuff, uint8_t BlockingMode)
{
	char uartTxBuff[150];

	// Wait for any existing interrupt based transfers to complete before sending in blocking mode
	while(pUSARTHandle->TxBusyState != USART_READY);

	// This happens here otherwise the txbuff can be overwritten mid transmit
	//memset(uartTxBuff, 0, sizeof(uartTxBuff));
	strcpy(uartTxBuff, txBuff);

	// Add \r\n to end of string for formatting in console
	strcat(uartTxBuff, "\r\n");
	if (BlockingMode == DISABLE)
	{
		USART_SendDataIT(pUSARTHandle, (uint8_t *)uartTxBuff, strlen(uartTxBuff));
	} else
	{
		USART_SendData(pUSARTHandle, (uint8_t *)uartTxBuff, strlen(uartTxBuff));
	}

	// Only send via SWV if debugger is connected
	if (DEBUGGER_CONNECTED())
	{
		printf("%s",uartTxBuff);		// Ensure SWV debugger speed set to clock source (16MHz HSE mode or 48MHz PLL)
	}
}

/*****************************************************************
 * @fn			- SetupDevice
 *
 * @brief		- This function configures device setup of creating a serial number
 *
 * @param[in]	- Handle for the EEPROM
 *
 * @return		- none
 *
 * @Note		-
 */
void SetupDevice(CAV25M01_Handle_t *pdev)
{
	uint8_t TimeStamp[12];
	uint8_t SerNum[3];
	char SerialNumber[SerNumSizeBytes];

	sprintf(msgTxBuff, "Enter date (DD/MM/YYYY)");
	UserMessages(&husart2, msgTxBuff, ENABLE);

	// also waits for a \r\n to be received
	USART_ReceiveData(&husart2,TimeStamp,12);

	sprintf(msgTxBuff, "Enter serial number (single digit number)");
	UserMessages(&husart2, msgTxBuff, ENABLE);

	// also waits for a \r\n to be received
	USART_ReceiveData(&husart2,SerNum,3);

	// Build serial string
	strcpy(SerialNumber, "STM32_COMMS_BOARD_REV2 ");
	strncat(SerialNumber, (char *)TimeStamp, 10);
	strcat(SerialNumber, " #");
	strncat(SerialNumber, (char *)SerNum, 1);

	// Store the serial
	CAV25M01_SerNum(pdev, SerialNumber, STORE);
}
