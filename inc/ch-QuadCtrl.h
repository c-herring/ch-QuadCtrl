/*
 * ch-QuadCtrl.h
 *
 *  Created on: Mar 18, 2017
 *      Author: HeZ
 */
#include "PIDMotor.h"

#ifndef CH_QUADCTRL_H_
#define CH_QUADCTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

// ------- Hardware Definitions -------
// Pin and port for encoder signals. Note these are handled in the 10-15 ISR, so must be pins 10-15
#define Encoder0_A_Port	GPIOA
#define Encoder0_A_Pin  GPIO_PIN_11
#define Encoder0_B_Port  GPIOA
#define Encoder0_B_Pin  GPIO_PIN_12
// ----
#define Encoder1_A_Port	GPIOB
#define Encoder1_A_Pin  GPIO_PIN_13
#define Encoder1_B_Port  GPIOB
#define Encoder1_B_Pin  GPIO_PIN_14

// ------- UART Stuff --------
// Create a global UART2 Handle
UART_HandleTypeDef huart2;
// Create a global DMA handle that will be connected to UART2 receive
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
#define Motor0_DIR_Port		GPIOC
#define Motor0_DIR_Pin		GPIO_PIN_8
#define Motor1_DIR_Port		GPIOC
#define Motor1_DIR_Pin		GPIO_PIN_6
#define Motor0_PWM_Port 	GPIOB
#define Motor0_PWM_Pin		GPIO_PIN_6
#define Motor1_PWM_Port		GPIOB
#define Motor1_PWM_Pin		GPIO_PIN_8
#define Motor0_TIM_Channel	TIM_CHANNEL_1
#define Motor1_TIM_Channel	TIM_CHANNEL_3
#define Motor_FORWARD 		GPIO_PIN_SET
#define Motor_REVERSE 		GPIO_PIN_RESET

// -------- Motor PID Stuff --------
PIDMotor_TypeDef Motor0;
PIDMotor_TypeDef Motor1;
#define PID_TD 25 // ms PID loop time
#define PWM_PERIOD 3999

// -------- Encoder Stuff --------
// Quadrature encoder lookup table
extern int8_t lookup_table[];

// -------- Function Definitions --------
extern void GPIO_Init(void);
void SystemClock_Config(void);
void USART2_UART_Init(void);
void GPIO_Init(void);
void TIM4_Init(void);
void PIDMotors_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* CH_QUADCTRL_H_ */
