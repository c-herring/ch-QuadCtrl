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
	uint32_t out;
	float rawOut;
	float processParam;

} PIDControl_TypeDef;

typedef struct {
	PIDControl_TypeDef pid;

	int32_t encPos; // TODO: Add functionality to wrap-around if we go past +2^31-1 or -2^31
	int32_t lastEncPos;

	float vel; // Current velocity
	float velSet; // Velocity set point



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
