#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->out_min    = out_min;
    pid->out_max    = out_max;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measured, float dt)
{
    float error      = setpoint - measured;
    float derivative = (error - pid->prev_error) / dt;

    pid->integral  += error * dt;

    /* Anti-windup: clamp integral contribution */
    float i_term = pid->ki * pid->integral;
    if      (i_term > pid->out_max) { pid->integral = pid->out_max / pid->ki; i_term = pid->out_max; }
    else if (i_term < pid->out_min) { pid->integral = pid->out_min / pid->ki; i_term = pid->out_min; }

    float output = pid->kp * error + i_term + pid->kd * derivative;

    if      (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    pid->prev_error = error;
    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
