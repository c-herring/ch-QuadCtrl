/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32l4xx.h"
#include "stm32l4xx_nucleo.h"
#include "ch-QuadCtrl.h"
#include "string.h"
// Define the global encoder lookup table
int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

int main(void)
{
	// Init functions
	HAL_Init();
	SystemClock_Config();
	GPIO_Init();
	// Init UART2
	USART2_UART_Init();

	// Initialise Timer 4
	TIM4_Init();
	// Start both PWM Channels
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	// Initialise the stopwatches
	uint32_t LEDStopwatch = HAL_GetTick();
	uint32_t PIDStopwatch = HAL_GetTick();

	for(;;)
	{
		if (HAL_GetTick() - LEDStopwatch > 100)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			LEDStopwatch = HAL_GetTick();
			// Print some debug stuff
			//sprintf(txbuff, "Motor0 Enc = %d\tMotor1 Enc = %d\n\r", Motor0.encPos, Motor1.encPos);
			//HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, strlen(txbuff), 0xff);
		}

		if (HAL_GetTick() - PIDStopwatch > 1000)
		{
			PIDStopwatch = HAL_GetTick();
		}
	}
}

void PIDMotors_Init(void)
{
	// Create the PID params struct. This needs to become populated from memory
	PIDParams_TypeDef motorA_params;
	motorA_params.Kp = 0.15f;
	motorA_params.Ki = 0.001f;
	motorA_params.Kd = 0.0f;

	Motor_Init(&Motor0, motorA_params, PID_TD, PWM_PERIOD, 5000.0);
	Motor_Init(&Motor1, motorA_params, PID_TD, PWM_PERIOD, 5000.0);
}

//
// Initialise GPIO
//
void GPIO_Init(void)
{
	// Enable the GPIO Clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef gpioinitstruct = {0};

	// Set the indicator LED on the Nucleo as an output
	gpioinitstruct.Pin = GPIO_PIN_5;
	gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull   = GPIO_NOPULL;
	gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpioinitstruct);

	// Set encoder 1 A signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder0_A_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder0_A_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Set encoder 1 B signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder0_B_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder0_B_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Set encoder 2 A signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder1_A_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder1_A_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Set encoder 2 B signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder1_B_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder1_B_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

//
// Initialise UART2
//
void USART2_UART_Init(void)
{

	// This is all pretty self explaining.
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_Init(&huart2);

	// Initialise UART2 buffers and start the circular RX buffer
	cmdbuff[0] = '\0';
	cmdBuffIndex = 0;
	HAL_UART_Receive_DMA(&huart2, &rxB, 1);
}

/*
 * This function is called from inside HAL_UART_Init
 * Initialize the UART gpio
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	// Create init struct to pass to the GPIO init function
	GPIO_InitTypeDef GPIO_InitStruct;

	// This function is called for all UART initialization.
	if(huart->Instance==USART2)
	{
		// Enable the peripheral clock for PORTA GPIO
		__GPIOA_CLK_ENABLE();
		// Enable the clock associated with the UART2 peripheral
		//__USART2_CLK_ENABLE();
		__HAL_RCC_USART2_CLK_ENABLE();

		// GPIO pins to be configured are 2 and 3. We want pullup pulldown mode
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		// Low speed is more than sufficient for our serial comms (<5MHz)
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		// Set the alternate function to AF7, USART2
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

		// Init
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);


		// Get the DMA2 clock chooching
		__HAL_RCC_DMA1_CLK_ENABLE();

		// Set up DMA
		hdma_usart2_rx.Instance = DMA1_Channel6;
		hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
		hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;// The direction id from peripheral to memory.
		hdma_usart2_rx.Init.PeriphInc = DMA_PINC_ENABLE; // Do not increment the peripheral memory.
		hdma_usart2_rx.Init.MemInc = DMA_MINC_DISABLE; // Do not increment the memory address.
		hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // DMA peripheral data size only needs to be one byte
		hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE; // As does the memory
		hdma_usart2_rx.Init.Mode = 	DMA_CIRCULAR; // Circular will reload DMA_SxNDTR register after each transfer (DMA_SxNDTR is number of remaining data to be transfered)
		hdma_usart2_rx.Init.Priority = 	DMA_PRIORITY_LOW; // Low priority
		HAL_DMA_Init(&hdma_usart2_rx);

		__HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

		// Enable DMA interrupt
		HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, UART_PRIORITY, UART_RX_SUBPRIORITY);
		HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	}
}

/*
 * Initialize timer4. Put PWM mode on pin PB6
 */
void TIM4_Init(void)
{
	__HAL_RCC_TIM4_CLK_ENABLE();
	// Create the configuration structures
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	// First init the timer. The RCC cloxk for timer 4 is done inside the HAL_TIM_PWM_MspInit callback. Why can't we just do that here?
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0; //TODO timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	// PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
	// TIM_Period = (timer_tick_frequency / PWM_frequency) - 1
	// Assuming APB1 clock is running at 84MHZ, we have no prescaler. If this is true then we are targeting 10khz
	// TIM_Period = 84000000 / 10000 - 1 = 8399
	htim4.Init.Period = PWM_PERIOD;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	// Timer base is set in here:
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		//Error_Handler();
	}



	// TODO: What does all this do!?
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		//Error_Handler();
	}



	// TODO: What does all this do?!
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		//Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		//Error_Handler();
	}

	// Now set the pin to use the timer
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = Motor0_PWM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(Motor0_PWM_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = Motor1_PWM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(Motor1_PWM_Port, &GPIO_InitStruct);

	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 4000);
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    //Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    //Error_Handler();
  }

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    //Error_Handler();
  }


}

void parseCommand()
{
	int motorIndex = -1;
	PIDParams_TypeDef PID_Params;
	float tempfloat1;
	float tempfloat2;
	float tempfloat3;

	switch (cmdbuff[0])
	{
		// CMD is targeted towards motor A
		case '0':
			motorIndex = 0;
			break;
		// CMD is targeted towards motor B
		case '1':
			motorIndex = 1;
			break;
		// No Motor is specified, CMD is invalid
		default:
			motorIndex = -1;
			break;
	}

	if (motorIndex != -1)
	{
		switch (cmdbuff[1])
		{
			case 'V':
				// Set the velocity
				if (sscanf(cmdbuff+2, "%f", &tempfloat1) != EOF)
				{
					sprintf(txbuff, "I saw mot = %d\tV = %f\n\r", motorIndex, tempfloat1);
					HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
				}
				else
				{
					// If a velocity
					cannot be read, it is invalid.
					motorIndex = -1;
				}
				break;
			case 'v':
				// Return the velocity
				if (motorIndex == 0) sprintf(txbuff, "0v%f", Motor0.vel);
				if (motorIndex == 1) sprintf(txbuff, "1v%f", Motor1.vel);
				HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
				break;
			case 'p':
				// Return current encoder position
				if (motorIndex == 0) sprintf(txbuff, "0p%d", Motor0.encPos);
				if (motorIndex == 1) sprintf(txbuff, "1p%d", Motor1.encPos);
				HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
				break;
			case 'K':
				if (sscanf(cmdbuff+2, "p%fi%fd%f", &PID_Params.Kp, &PID_Params.Ki, &PID_Params.Kd) != 3)
				{
					// If exactly three values were not read, it was invalid command
					motorIndex = -1;
				} else
				{
					if (motorIndex == 0) PID_Init(&Motor0.pid, PID_Params);
					if (motorIndex == 1) PID_Init(&Motor1.pid, PID_Params);
				}
				break;
			case 'k':
				if (motorIndex == 0) sprintf(txbuff, "0kp%fi%fd%f", Motor0.pid.params.Kp, Motor0.pid.params.Ki, Motor0.pid.params.Kd);
				if (motorIndex == 1) sprintf(txbuff, "1kp%fi%fd%f", Motor1.pid.params.Kp, Motor1.pid.params.Ki, Motor1.pid.params.Kd);
				HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
				break;

			// COmmand is not recognised
			default:
				motorIndex = -1;
		}
	}

	if (motorIndex == -1)
	{
		sprintf(txbuff, "?%s\n\r", cmdbuff);
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
	}

}
