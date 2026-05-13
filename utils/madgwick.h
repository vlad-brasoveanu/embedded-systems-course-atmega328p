#ifndef MADGWICK_H
#define MADGWICK_H

/*
 * Madgwick AHRS filter – Sebastian O.H. Madgwick, 2010.
 *
 * Fuses gyroscope, accelerometer, and (optionally) magnetometer to produce
 * a drift-free quaternion attitude estimate.
 *
 * Two update functions are provided:
 *   Madgwick_Update9DOF  – full fusion: gyro + accel + magnetometer
 *                          Gives accurate pitch, roll AND yaw.
 *   Madgwick_Update6DOF  – gyro + accel only
 *                          Gives accurate pitch and roll; yaw drifts slowly.
 *
 * Gyroscope input: degrees/s  (converted to rad/s internally)
 * Accelerometer:  any consistent unit (normalised internally)
 * Magnetometer:   any consistent unit (normalised internally)
 *
 * Beta gain: controls convergence speed vs noise.
 *   Typical range: 0.033 (slow/smooth) – 0.2 (fast/noisy)
 *   Default: 0.1
 */

#define MADGWICK_BETA_DEFAULT  0.1f

typedef struct {
    float beta;             /* gradient-descent step size */
    float q0, q1, q2, q3;  /* unit quaternion (w, x, y, z) */
} Madgwick_t;

/**
 * @brief Initialise the filter (identity quaternion, horizontal).
 * @param f    Filter instance.
 * @param beta Step gain (use MADGWICK_BETA_DEFAULT).
 */
void Madgwick_Init(Madgwick_t *f, float beta);

/**
 * @brief Full 9-DOF update: accel + gyro + magnetometer.
 *        Use this whenever the magnetometer has valid data.
 *
 * @param f         Filter instance.
 * @param gx,gy,gz  Gyroscope rates (degrees/s).
 * @param ax,ay,az  Accelerometer (any units, normalised internally).
 * @param mx,my,mz  Magnetometer  (any units, normalised internally).
 * @param dt        Time step (seconds).
 */
void Madgwick_Update9DOF(Madgwick_t *f,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt);

/**
 * @brief Reduced 6-DOF update: accel + gyro only.
 *        Yaw will integrate from gyro and drift over time.
 */
void Madgwick_Update6DOF(Madgwick_t *f,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt);

/** @brief Extract pitch angle in degrees from the current quaternion. */
float Madgwick_GetPitch(const Madgwick_t *f);

/** @brief Extract roll angle in degrees from the current quaternion. */
float Madgwick_GetRoll(const Madgwick_t *f);

/**
 * @brief Extract yaw (heading) angle in degrees [-180, 180] from quaternion.
 *        Only meaningful after Madgwick_Update9DOF with valid mag data.
 */
float Madgwick_GetYaw(const Madgwick_t *f);

#endif /* MADGWICK_H */
