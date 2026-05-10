#include "servo.h"
#include "timer1.h"
#include "timer2.h"
#include <stdint.h>

/*
 * Timer1 – prescaler 8, ICR1 = 39999
 *   f  = 16 000 000 / (8 * 40 000) = 50 Hz  (period = 20 ms)
 *   resolution: 1 count = 0.5 µs
 *   OCR1x = pulse_us * 2
 */
#define T1_PRESCALER  8U
#define T1_TOP        39999U
#define T1_US_TO_CNT(us)  ((uint16_t)((us) * 2U))

/*
 * Timer2 – prescaler 1024, TOP = 255 (Fast PWM mode 3)
 *   f  = 16 000 000 / (1024 * 256) = 61.04 Hz  (period ≈ 16 384 µs)
 *   OCR2B = round(pulse_us / 16384 * 255)
 *   Simplified: OCR2B = (pulse_us * 255 + 8192) / 16384
 */
#define T2_PRESCALER  1024U
#define T2_PERIOD_US  16384U
#define T2_US_TO_CNT(us)  ((uint8_t)(((uint32_t)(us) * 255U + T2_PERIOD_US / 2U) / T2_PERIOD_US))

/* Clamp pulse to valid SG90 range */
static uint16_t clamp_pulse(uint16_t us)
{
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

/* Linear mapping from angle [SAFE_MIN, SAFE_MAX] to pulse [MIN_US, MAX_US].
 * Input is clamped to the safe range to protect the SG90 from stall. */
static uint16_t angle_to_us(float angle)
{
    if (angle < SERVO_SAFE_MIN_DEG) angle = SERVO_SAFE_MIN_DEG;
    if (angle > SERVO_SAFE_MAX_DEG) angle = SERVO_SAFE_MAX_DEG;
    return (uint16_t)(SERVO_MIN_US + (uint16_t)(angle * (float)(SERVO_MAX_US - SERVO_MIN_US) / 180.0f));
}

void Servo_Init(void)
{
    Timer1_FastPWM_Init(T1_PRESCALER, T1_TOP);
    Timer2_FastPWM_Init(T2_PRESCALER);
    Servo_CenterAll();
}

void Servo_SetPulse_us(ServoAxis_t axis, uint16_t pulse_us)
{
    pulse_us = clamp_pulse(pulse_us);

    switch (axis) {
        case SERVO_X:
            Timer1_SetDutyCycleA(T1_US_TO_CNT(pulse_us));
            break;
        case SERVO_Y:
            Timer1_SetDutyCycleB(T1_US_TO_CNT(pulse_us));
            break;
        case SERVO_Z:
            Timer2_SetDutyCycleB(T2_US_TO_CNT(pulse_us));
            break;
    }
}

void Servo_SetAngle(ServoAxis_t axis, float angle)
{
    Servo_SetPulse_us(axis, angle_to_us(angle));
}

void Servo_CenterAll(void)
{
    Servo_SetPulse_us(SERVO_X, SERVO_MID_US);
    Servo_SetPulse_us(SERVO_Y, SERVO_MID_US);
    Servo_SetPulse_us(SERVO_Z, SERVO_MID_US);
}
