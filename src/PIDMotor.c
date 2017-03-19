/*
 * PIDMotor.c
 *
 *  Created on: Mar 10, 2017
 *      Author: HeZ
 */

#include "stm32l4xx.h"
#include "PIDMotor.h"





// Initialise the motor and PID structure. This must be called before running the PID
void Motor_Init(PIDMotor_TypeDef *motor, PIDParams_TypeDef _pid_params, uint32_t _pidRate, uint32_t _pwmPeriod, float _Ierror_lim)
{
	// Copy the PID parameters into the structure
	PID_Init(&motor->pid, _pid_params);
	motor->pid.pidRate = _pidRate;
	motor->pid.outLim = _pwmPeriod;

	motor->pid.error = 0.0f;
	motor->pid.prev_error = 0.0f;
	motor->pid.Ierror = 0.0f;
	motor->pid.Ierror_limit = _Ierror_lim;

	// Set intial values
	motor->encPos = 0;
	motor->encState = 0;
	motor->lastEncPos = 0;
	motor->vel = 0.0f;
	motor->velSet = 0.0f;
	motor->pid.processParam = 0.0f;
	motor->pid.rawOut = 0.0f;

	//motor->hw = hwDesc;
}

// Initialise the PID strucutre
void PID_Init(PIDControl_TypeDef *pid, PIDParams_TypeDef _pid_params)
{
	pid->params.Kp = _pid_params.Kp;
	pid->params.Ki = _pid_params.Ki;
	pid->params.Kd = _pid_params.Kd;

}

// Compute PID
void PID_Compute(PIDMotor_TypeDef *motor)
{
	// Current velocity
	motor->vel = (float)(motor->encPos - motor->lastEncPos)/motor->pid.pidRate*1000;
	motor->lastEncPos = motor->encPos;
	// Calc the error
	motor->pid.error = motor->velSet - motor->vel;

	motor->pid.pid_error = motor->pid.params.Kp*motor->pid.error + motor->pid.params.Kd*(motor->pid.error - motor->pid.prev_error) + motor->pid.params.Ki*motor->pid.Ierror;

	motor->debug1 = motor->pid.params.Kp*motor->pid.error;
	motor->debug2 = motor->pid.params.Kd*(motor->pid.error - motor->pid.prev_error);
	motor->debug3 = motor->pid.params.Ki*motor->pid.Ierror;

	// Save last error
	motor->pid.prev_error =  motor->pid.error;

	motor->pid.processParam += motor->pid.pid_error;
	//if (motor->pid.processParam > motor->pid.outLim) motor->pid.processParam = motor->pid.outLim;
	//if (motor->pid.processParam < 0) motor->pid.processParam = 0;

	// Limit the range of the process parameter
	if (motor->pid.processParam > motor->pid.outLim) motor->pid.processParam = motor->pid.outLim;
	if (motor->pid.processParam < -motor->pid.outLim) motor->pid.processParam = -motor->pid.outLim;


	if (motor->velSet >= 0)
	{
		motor->pid.rawOut = (motor->pid.processParam > 0) ? motor->pid.processParam : 0;
	}
	else
	{
		motor->pid.rawOut = (motor->pid.processParam < 0) ? -motor->pid.processParam : 0;
	}

	//if (motor->velSet >= 0)
	//if (motor->velSet >= 0)
	//{
	//	motor->pid.processParam > motor->pid.outLim ? motor->pid.processParam = motor->pid.outLim : 0;
	//} else {
	//	motor->pid.processParam > motor->pid.outLim ? motor->pid.processParam = motor->pid.outLim : 0;
	//}


	motor->pid.Ierror += motor->pid.error;
	if (motor->pid.Ierror > motor->pid.Ierror_limit) motor->pid.Ierror = motor->pid.Ierror_limit;
	if (motor->pid.Ierror < -motor->pid.Ierror_limit) motor->pid.Ierror = -motor->pid.Ierror_limit;
	if (motor->velSet == 0) motor->pid.Ierror = 0;



}

void PID_VelSet(PIDMotor_TypeDef *motor, float vel)
{
	motor->velSet = vel;
}







