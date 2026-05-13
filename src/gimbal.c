#include "gimbal.h"
#include "lsm6dso.h"    /* accel + gyro  */
#include "lis3mdl.h"    /* magnetometer  */
#include "servo.h"
#include "madgwick.h"
#include "pid.h"
#include "i2c.h"
#include "timer0.h"

/* ---- Module variables ---------------------------------------------------- */

static Madgwick_t s_filter;   /* attitude filter (gives pitch/roll/yaw)  */
static PID_t      s_pid_x;    /* PID for X axis (pitch)                  */
static PID_t      s_pid_y;    /* PID for Y axis (roll)                   */
static PID_t      s_pid_z;    /* PID for Z axis (yaw)                    */
static uint32_t   s_last_ms;  /* time of the previous update (ms)        */

/* Home angles: the orientation captured at power-on.
 * The PIDs try to hold these angles forever.                              */
static float s_x_home;
static float s_y_home;
static float s_z_home;

/* Current gimbal state – updated every cycle, readable from anywhere */
volatile Gimbal_State_t g_gimbal_state;

/* ---- Helper -------------------------------------------------------------- */

/* Wraps a yaw error to [-180, 180] so the controller always takes the
 * short path around the 0/360 boundary.
 * Example: yaw = 350°, home = 10°  →  error = -20°, not +340°.           */
static float wrap_180(float e)
{
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

/* ---- Gimbal_Init --------------------------------------------------------- */

bool Gimbal_Init(void)
{
    /* 1. Start the millisecond timer (used to measure dt each cycle) */
    Timer0_Init();

    /* 2. Start the I2C bus at 400 kHz */
    I2C_Init(400000UL);

    /* 3. Check each sensor is connected and configure it.
     *    Returns false immediately if a sensor is missing.            */
    if (!LSM6DSO_Init()) return false;   /* accel + gyro  */
    if (!LIS3MDL_Init()) return false;   /* magnetometer  */

    /* 4. Initialise servos and move them all to 90° (centre) */
    Servo_Init();

    /* 5. Initialise the Madgwick filter with the chosen beta value */
    Madgwick_Init(&s_filter, GIMBAL_MADGWICK_BETA);

    /* 6. Initialise one PID per axis.
     *    X and Y use the full gains. Z (yaw) uses Ki=0 to prevent drift
     *    – see GIMBAL_KI_YAW comment in gimbal.h for the reason.        */
    PID_Init(&s_pid_x, GIMBAL_KP, GIMBAL_KI,     GIMBAL_KD,
             -GIMBAL_MAX_DEG, GIMBAL_MAX_DEG);
    PID_Init(&s_pid_y, GIMBAL_KP, GIMBAL_KI,     GIMBAL_KD,
             -GIMBAL_MAX_DEG, GIMBAL_MAX_DEG);
    PID_Init(&s_pid_z, GIMBAL_KP, GIMBAL_KI_YAW, GIMBAL_KD,
             -GIMBAL_MAX_DEG, GIMBAL_MAX_DEG);

    /* 7. Warm-up: run the filter for GIMBAL_WARMUP_MS milliseconds.
     *    The filter starts from a zero quaternion and needs a few hundred
     *    samples to converge to the real orientation. Without this, the
     *    home angles captured below would all read 0°.                 */
    uint32_t end = Millis() + GIMBAL_WARMUP_MS;
    while (Millis() < end) {
        LSM6DSO_Data_t imu;
        LIS3MDL_Data_t mag;
        if (LSM6DSO_Read(&imu)) {
            bool have_mag = LIS3MDL_Read(&mag);
            float dt_nom  = 1.0f / 208.0f;   /* nominal 208 Hz sample period */
            if (have_mag)
                Madgwick_Update9DOF(&s_filter,
                                    imu.gx, imu.gy, imu.gz,
                                    imu.ax, imu.ay, imu.az,
                                    mag.mx, mag.my, mag.mz, dt_nom);
            else
                Madgwick_Update6DOF(&s_filter,
                                    imu.gx, imu.gy, imu.gz,
                                    imu.ax, imu.ay, imu.az, dt_nom);
        }
    }

    /* 8. Capture home: the current orientation becomes the hold target.
     *    Place the rig in the desired resting position before powering on. */
    s_x_home = Madgwick_GetPitch(&s_filter);
    s_y_home = Madgwick_GetRoll (&s_filter);
    s_z_home = Madgwick_GetYaw  (&s_filter);

    s_last_ms = Millis();
    return true;
}

/* ---- Gimbal_Update ------------------------------------------------------- */

void Gimbal_Update(void)
{
    /* Step 1 – Read the IMU.
     * Returns false when no new sample is ready yet (sensor runs at 208 Hz).
     * In that case we exit early and call this function again next loop.   */
    LSM6DSO_Data_t imu;
    if (!LSM6DSO_Read(&imu))
        return;

    /* Step 2 – Read the magnetometer (for drift-free yaw).
     * have_mag = false is fine; the filter falls back to 6-DOF mode.      */
    LIS3MDL_Data_t mag;
    bool have_mag = LIS3MDL_Read(&mag);

    /* Step 3 – Compute dt (time since last update, in seconds).
     * This is passed to the filter and PID so they work in real time.     */
    uint32_t now = Millis();
    float dt = (float)(now - s_last_ms) * 0.001f;
    if (dt <= 0.0f) dt = 0.001f;   /* safety: never pass zero dt */
    s_last_ms = now;

    /* Step 4 – Run Madgwick filter to get pitch, roll, yaw.
     * 9-DOF (with mag) gives drift-free yaw.
     * 6-DOF (without mag) is used as fallback when mag has no new data.   */
    if (have_mag)
        Madgwick_Update9DOF(&s_filter,
                            imu.gx, imu.gy, imu.gz,
                            imu.ax, imu.ay, imu.az,
                            mag.mx, mag.my, mag.mz, dt);
    else
        Madgwick_Update6DOF(&s_filter,
                            imu.gx, imu.gy, imu.gz,
                            imu.ax, imu.ay, imu.az, dt);

    float pitch = Madgwick_GetPitch(&s_filter);
    float roll  = Madgwick_GetRoll (&s_filter);
    float yaw   = Madgwick_GetYaw  (&s_filter);

    /* Save the angles so they can be read from anywhere */
    g_gimbal_state.pitch_deg = pitch;
    g_gimbal_state.roll_deg  = roll;
    g_gimbal_state.yaw_deg   = yaw;

    /* Step 5 – PID controllers.
     * Each PID computes: correction = Kp * error + Ki * integral + Kd * derivative
     * error = setpoint (home) - measured (current angle)
     * Output is the number of degrees to move the servo from its centre.  */
    float x_corr = PID_Update(&s_pid_x, s_x_home, pitch, dt);
    float y_corr = PID_Update(&s_pid_y, s_y_home, roll,  dt);

    /* Yaw error is wrapped so the controller takes the short path */
    float z_err  = wrap_180(yaw - s_z_home);
    float z_corr = PID_Update(&s_pid_z, 0.0f, z_err, dt);

    /* Step 6 – Move the servos.
     * 90° is the centre position. Adding the correction tilts the servo
     * in the direction needed to level the platform.
     * If any axis corrects the wrong way, change + to - for that axis.   */
    Servo_SetAngle(SERVO_X, 90.0f + x_corr);
    Servo_SetAngle(SERVO_Y, 90.0f + y_corr);
    Servo_SetAngle(SERVO_Z, 90.0f + z_corr);
}
