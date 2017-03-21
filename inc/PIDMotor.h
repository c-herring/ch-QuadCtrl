/*
 * PIDMotor.h
 *
 *  Created on: Mar 10, 2017
 *      Author: HeZ
 */

#ifndef PIDMOTOR_H_
#define PIDMOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32l4xx.h"

#define PID_OUTPUT_DEADZONE_PERCENT 1.0f // If the output is less than this percent of full scale, just turn the motor off.

typedef enum {
	Forward,
	Reverse
} PIDMotor_State;

typedef struct {
	GPIO_TypeDef 	encA_PORT;
	uint16_t		encA_PIN;
	GPIO_TypeDef 	encB_PORT;
	uint16_t		encB_PIN;
	GPIO_TypeDef 	DIR_PORT;
	uint16_t		DIR_PIN;
	GPIO_TypeDef 	PWM_PORT;
	uint16_t		PWM_PIN;
	uint32_t 		TIM_CHANNEL;
} PIDHardware_TypeDef;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} PIDParams_TypeDef;

typedef struct {
	PIDParams_TypeDef params;

	float error;
	float prev_error;
	float Ierror;
	float Ierror_limit;
	uint32_t pidRate; //
	float outLim;
	int32_t outLimLower;
	uint32_t out;
	float rawOut;
	float processParam;
	float pid_error;

} PIDControl_TypeDef;

typedef struct {
	PIDControl_TypeDef pid;
	PIDMotor_State state;
	PIDHardware_TypeDef hw;

	uint32_t encState;
	int32_t encPos; // TODO: Add functionality to wrap-around if we go past +2^31-1 or -2^31
	int32_t lastEncPos;

	float vel; // Current velocity
	float velSet; // Velocity set point

	float debug1;
	float debug2;
	float debug3;

}PIDMotor_TypeDef;


// Initialise the motor
extern void Motor_Init(PIDMotor_TypeDef *motor, PIDParams_TypeDef _pid_params, uint32_t _pidRate,  uint32_t pwmPeriod, float _Ierror_lim);

// Initialise a pid struct
extern void PID_Init(PIDControl_TypeDef *pid, PIDParams_TypeDef _pid_params);

// Computer this PID loop
extern void PID_Compute(PIDMotor_TypeDef *motor);

// Set the velocity (in encoder ticks per second)
extern void Motor_Vel_Set(PIDMotor_TypeDef *motor, float vel);

#ifdef __cplusplus
}
#endif

#endif /* PIDMOTOR_H_ */
