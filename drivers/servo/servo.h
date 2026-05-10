#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/*
 * Servo pin assignments:
 *   SERVO_X → D9  / OC1A  (Timer1, 50 Hz, 0.5 µs resolution) – pitch / X axis
 *   SERVO_Y → D10 / OC1B  (Timer1, 50 Hz, 0.5 µs resolution) – roll  / Y axis
 *   SERVO_Z → D3  / OC2B  (Timer2, ~61 Hz, ~64 µs resolution) – yaw   / Z axis
 *
 * SG90 pulse range: 1000 µs (0°) … 2000 µs (180°), centre 1500 µs (90°).
 *
 * Note: Timer2 is 8-bit so Z-axis angular resolution is ~11°/step.
 * For better Z precision an external PWM driver (e.g. PCA9685) is
 * recommended in production.
 */

typedef enum {
    SERVO_X = 0,   /* D9  – pitch / X axis */
    SERVO_Y = 1,   /* D10 – roll  / Y axis */
    SERVO_Z = 2,   /* D3  – yaw   / Z axis */
} ServoAxis_t;

/* Pulse-width limits in microseconds (hardware range: 0° … 180°) */
#define SERVO_MIN_US  1000U
#define SERVO_MAX_US  2000U
#define SERVO_MID_US  1500U

/* Safe angle limits – keeps the SG90 away from its mechanical endstops
 * to prevent stall current and avoid burning the motor.
 * Servo_SetAngle() clamps to this range automatically.              */
#define SERVO_SAFE_MIN_DEG   10.0f
#define SERVO_SAFE_MAX_DEG  170.0f

/**
 * @brief Initialise Timer1 (pitch/roll) and Timer2 (yaw) for 50/61 Hz PWM.
 *        All axes are centred at 90° on return.
 */
void Servo_Init(void);

/**
 * @brief Set servo position by angle.
 * @param axis  SERVO_PITCH, SERVO_ROLL, or SERVO_YAW.
 * @param angle Angle in degrees [0, 180].
 */
void Servo_SetAngle(ServoAxis_t axis, float angle);

/**
 * @brief Set servo position by raw pulse width.
 * @param axis     Target axis.
 * @param pulse_us Pulse width in µs [SERVO_MIN_US, SERVO_MAX_US].
 */
void Servo_SetPulse_us(ServoAxis_t axis, uint16_t pulse_us);

/**
 * @brief Move all axes to 90° (level/centre position).
 */
void Servo_CenterAll(void);

#endif /* SERVO_H */
