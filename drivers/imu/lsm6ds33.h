#ifndef LSM6DS33_H
#define LSM6DS33_H

#include <stdint.h>
#include <stdbool.h>

/* I2C address: SA0 pin high (default on AltIMU-10 v5) */
#define LSM6DS33_ADDR       0x6B

/* Key register addresses */
#define LSM6DS33_WHO_AM_I   0x0F   /* expected 0x69 */
#define LSM6DS33_CTRL1_XL   0x10   /* accelerometer control */
#define LSM6DS33_CTRL2_G    0x11   /* gyroscope control */
#define LSM6DS33_CTRL3_C    0x12   /* common control */
#define LSM6DS33_STATUS_REG 0x1E
#define LSM6DS33_OUTX_L_G   0x22   /* gyro X low byte  */
#define LSM6DS33_OUTX_L_XL  0x28   /* accel X low byte */

/* Sensitivity constants (SI units) */
#define LSM6DS33_ACCEL_SENS_G    0.000061f  /* g per LSB  (±2 g FS)     */
#define LSM6DS33_GYRO_SENS_DPS   0.00875f   /* dps per LSB (±250 dps FS) */

typedef struct {
    float ax, ay, az;   /* acceleration in g */
    float gx, gy, gz;   /* angular rate in degrees/s */
} LSM6DS33_Data_t;

/**
 * @brief Initialise the sensor: verify WHO_AM_I, configure ODR and full-scale.
 * @return true on success.
 */
bool LSM6DS33_Init(void);

/**
 * @brief Read raw accelerometer and gyroscope values and convert to SI.
 * @param out Pointer to result structure.
 * @return true if new data was available and read successfully.
 */
bool LSM6DS33_Read(LSM6DS33_Data_t *out);

#endif /* LSM6DS33_H */
