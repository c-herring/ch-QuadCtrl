/*
 * ch-QuadCtrl.h
 *
 *  Created on: Mar 18, 2017
 *      Author: HeZ
 */

#ifndef CH_QUADCTRL_H_
#define CH_QUADCTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

// ------- Hardware Definitions -------
// Pin and port for encoder signals. Note these are handled in the 10-15 ISR, so must be pins 10-15
#define Encoder1_A_Port	GPIOA
#define Encoder1_A_Pin  GPIO_PIN_11
#define Encoder1_B_Port  GPIOA
#define Encoder1_B_Pin  GPIO_PIN_12
// ----
#define Encoder2_A_Port	GPIOB
#define Encoder2_A_Pin  GPIO_PIN_13
#define Encoder2_B_Port  GPIOB
#define Encoder2_B_Pin  GPIO_PIN_14

// ------- UART Stuff --------
// Create a global UART2 Handle
UART_HandleTypeDef huart2;
// Create a global DMA handle that will be connected to UART2 recieve
DMA_HandleTypeDef hdma_usart2_rx;

// UART NVIC Priorities
#define UART_PRIORITY         6
#define UART_RX_SUBPRIORITY   0

// Buffer lengths
#define MAX_TX_BUFFER_LEN 500
#define MAX_CMD_BUFFER_LEN 50

// Allocate buffers
char txbuff[MAX_TX_BUFFER_LEN]; 	// Transmit buffer
char cmdbuff[MAX_CMD_BUFFER_LEN];	// Receive buffer
int32_t cmdBuffIndex;				// Current position in the RX buffer
uint8_t rxB;						// Single RX byte from the circular DMA buffer

// ------- Timer and PWM stuff -------
// PWM timer handle
TIM_HandleTypeDef htim4;



#ifdef __cplusplus
}
#endif

#endif /* CH_QUADCTRL_H_ */
