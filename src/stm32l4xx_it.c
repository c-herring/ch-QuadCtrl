/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32l4xx_it.h"
#include "ch-QuadCtrl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

//
// USART2 Interrupt handler
//
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

// Handle the callback for UART2 DMA RX
void DMA1_Channel6_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel6_IRQn);
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

// DMA RX callback for UART. Half complete callback is triggered every time we get a char.
// Complete callback will never trigger as it is a circular buffer.
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	__HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun

	// Return character is seen, or we have exceeded the buffer length (-1 to leave room for null)
	if (rxB == '\r' | cmdBuffIndex > MAX_CMD_BUFFER_LEN-2)
	{
		// Null terminate the string so sscanf can do it's job
		cmdbuff[cmdBuffIndex] = '\0';
		// Reset the index back to zero
		cmdBuffIndex = 0;
		// Parse the command
		parseCommand();
	} else
	{
		// Add this command to the buffer
		cmdbuff[cmdBuffIndex++] = rxB;
	}

}

void EXTI15_10_IRQHandler(void)
{
	// Check which interrupt was triggered, clear the interrupt before we handle it
	if (__HAL_GPIO_EXTI_GET_IT(Encoder0_A_Pin) != RESET) __HAL_GPIO_EXTI_CLEAR_IT(Encoder0_A_Pin);
	if (__HAL_GPIO_EXTI_GET_IT(Encoder0_B_Pin) != RESET) __HAL_GPIO_EXTI_CLEAR_IT(Encoder0_B_Pin);
	if (__HAL_GPIO_EXTI_GET_IT(Encoder1_A_Pin) != RESET) __HAL_GPIO_EXTI_CLEAR_IT(Encoder1_A_Pin);
	if (__HAL_GPIO_EXTI_GET_IT(Encoder1_B_Pin) != RESET) __HAL_GPIO_EXTI_CLEAR_IT(Encoder1_B_Pin);

	// Shift old encoder state left two bits
	Motor0.encState = Motor0.encState << 2;
	// OR in the two encoder channels
	Motor0.encState |= HAL_GPIO_ReadPin(Encoder0_A_Port, Encoder0_A_Pin) | HAL_GPIO_ReadPin(Encoder0_B_Port, Encoder0_B_Pin) << 1;
	// Update encoder position
	Motor0.encPos += lookup_table[Motor0.encState & 0b1111];

	// Do this all again for motor 2
	Motor1.encState = Motor1.encState << 2;
	Motor1.encState |= HAL_GPIO_ReadPin(Encoder1_A_Port, Encoder1_A_Pin) | HAL_GPIO_ReadPin(Encoder1_B_Port, Encoder1_B_Pin) << 1;
	Motor1.encPos += lookup_table[Motor1.encState & 0b1111];

}
