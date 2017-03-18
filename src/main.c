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

// Function Prototypes
void SystemClock_Config(void);
void USART2_UART_Init(void);
void GPIO_Init(void);

int main(void)
{
	// Init functions
	HAL_Init();
	SystemClock_Config();
	GPIO_Init();
	// Init UART2
	USART2_UART_Init();


	uint32_t LEDStopwatch = HAL_GetTick();
	uint32_t PIDStopwatch = HAL_GetTick();

	for(;;)
	{
		if (HAL_GetTick() - LEDStopwatch > 1000)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			LEDStopwatch = HAL_GetTick();
			sprintf(txbuff, "LED Stopwatch triggered\n\r");
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
		}

		if (HAL_GetTick() - PIDStopwatch > 1000)
		{
			sprintf(txbuff, "PID Stopwatch triggered\n\r");
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)txbuff, strlen(txbuff));
			PIDStopwatch = HAL_GetTick();
		}
	}
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
	gpioinitstruct.Pin = Encoder1_A_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder1_A_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Set encoder 1 B signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder1_B_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder1_B_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Set encoder 2 A signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder2_A_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder2_A_Port, &gpioinitstruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Set encoder 2 B signal as an interrupt for both rising and falling edge
	gpioinitstruct.Pin = Encoder2_B_Pin;
	gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpioinitstruct.Pull   = GPIO_PULLDOWN;
	HAL_GPIO_Init(Encoder2_B_Port, &gpioinitstruct);
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
