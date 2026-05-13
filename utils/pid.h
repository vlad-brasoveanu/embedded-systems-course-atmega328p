#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float out_min;   /* output clamp lower bound */
    float out_max;   /* output clamp upper bound */
} PID_t;

/**
 * @brief Initialise a PID controller.
 * @param pid      Controller instance.
 * @param kp       Proportional gain.
 * @param ki       Integral gain.
 * @param kd       Derivative gain.
 * @param out_min  Minimum output value (anti-windup + clamp).
 * @param out_max  Maximum output value.
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);

/**
 * @brief Compute one PID step.
 * @param pid       Controller instance.
 * @param setpoint  Desired value.
 * @param measured  Current measured value.
 * @param dt        Time step in seconds.
 * @return Control output, clamped to [out_min, out_max].
 */
float PID_Update(PID_t *pid, float setpoint, float measured, float dt);

/**
 * @brief Reset integrator and derivative state (call after large setpoint jumps).
 */
void PID_Reset(PID_t *pid);

#endif /* PID_H */
