//
// Created by Linus on 18.04.21.
//
#include "pid.h"

void set(struct PID_dev * pid, float target){
    pid->target = target;
}

float compute(struct PID_dev * pid, float temperature){
	pid->temperature = temperature;

    pid->currError = pid->target - temperature;
    pid->p = pid->kp * pid->currError;

    pid->currTime = HAL_GetTick()/1000.0;
    float elapsed_time = pid->currTime - pid->prevTime;

    pid->i += pid->ki * pid->currError;

    pid->d = pid->kd * (pid->currError - pid->error) / elapsed_time;

    pid->error = pid->currError;
    pid->prevTime = pid->currTime;
    float PIDvalue = pid->p + pid->i + pid->d;
    pid->pwr = (uint16_t)PIDvalue;
    return PIDvalue;
}

