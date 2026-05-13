#ifndef FILTER_H
#define FILTER_H

/*
 * Complementary filter for pitch/roll estimation.
 *
 * Fuses accelerometer angles (slow but accurate over time) with gyroscope
 * integration (fast but drifts).  Alpha controls the trust split:
 *   higher alpha  → trust gyro more  (smoother, more lag)
 *   lower  alpha  → trust accel more (noisier, less drift)
 *
 * Typical starting value: FILTER_ALPHA = 0.96f
 */

#define FILTER_ALPHA_DEFAULT  0.96f

typedef struct {
    float alpha;        /* complementary coefficient [0, 1) */
    float pitch;        /* current pitch estimate (degrees) */
    float roll;         /* current roll  estimate (degrees) */
} CompFilter_t;

/**
 * @brief Initialise the filter state.
 * @param f     Pointer to filter instance.
 * @param alpha Complementary coefficient (use FILTER_ALPHA_DEFAULT).
 */
void Filter_Init(CompFilter_t *f, float alpha);

/**
 * @brief Update pitch and roll estimates.
 *
 * @param f    Filter instance.
 * @param ax   Accelerometer X  (g).
 * @param ay   Accelerometer Y  (g).
 * @param az   Accelerometer Z  (g).
 * @param gx   Gyroscope X rate (degrees/s) – contributes to pitch.
 * @param gy   Gyroscope Y rate (degrees/s) – contributes to roll.
 * @param dt   Time elapsed since last call (seconds).
 */
void Filter_Update(CompFilter_t *f,
                   float ax, float ay, float az,
                   float gx, float gy,
                   float dt);

#endif /* FILTER_H */
