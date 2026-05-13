#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdbool.h>

/* ---- Tuning parameters ---------------------------------------------------
 *
 * Start with Ki = 0, Kd = 0. Raise Kp until the platform corrects quickly
 * but just starts to oscillate, then back off ~20 %.
 * Add Kd (try 0.1) only if it oscillates. Add a small Ki only if it settles
 * a few degrees off level and never fully corrects.
 * -------------------------------------------------------------------------- */
#define GIMBAL_KP      2.0f    /* proportional – how hard to correct an error   */
#define GIMBAL_KI      0.01f   /* integral     – correct a persistent offset     */
#define GIMBAL_KD      0.1f    /* derivative   – damp overshooting               */

/* Yaw (Z axis) must have Ki = 0.
 * Pitch and roll are anchored by gravity so their integral clears itself.
 * Yaw is only anchored by the magnetometer (weak reference). Any Ki on
 * yaw accumulates every time the gimbal is moved and never fully clears,
 * causing the Z servo to drift further left/right the more you use it.   */
#define GIMBAL_KI_YAW  0.0f

/* Maximum servo correction from 90° centre.
 * With 90°: servo travels full range 0°–180° (clamped to 10°–170° in driver) */
#define GIMBAL_MAX_DEG   90.0f

/* Madgwick filter convergence speed.
 * Higher = snappier but noisier. 0.04 is a good starting point.          */
#define GIMBAL_MADGWICK_BETA   0.04f   /* raised from 0.02 – lets mag correct yaw drift faster */

/* Time (ms) the filter runs before the home position is captured.
 * Gives the filter time to converge so the home angles are accurate.      */
#define GIMBAL_WARMUP_MS   1000U

/* ---- Current gimbal state ------------------------------------------------
 * Updated every control cycle. Can be read from anywhere in the program.  */
typedef struct {
    float pitch_deg;   /* X axis angle (deg) */
    float roll_deg;    /* Y axis angle (deg) */
    float yaw_deg;     /* Z axis angle (deg) */
} Gimbal_State_t;

extern volatile Gimbal_State_t g_gimbal_state;

/* Initialise all hardware and capture the home position.
 * Returns false if a sensor is not detected on the I2C bus.               */
bool Gimbal_Init(void);

/* Run one control cycle. Call this in the main loop as fast as possible.  */
void Gimbal_Update(void);

#endif /* GIMBAL_H */
