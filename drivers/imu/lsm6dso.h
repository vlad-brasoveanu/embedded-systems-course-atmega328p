#ifndef LSM6DSO_H
#define LSM6DSO_H

#include <stdint.h>
#include <stdbool.h>

/* LSM6DSO – 6-axis IMU (accel + gyro) on Pololu AltIMU-10 v6
 * I2C address: 0x6B  (SA0 pin tied high on the AltIMU board)
 * WHO_AM_I:    0x6C
 * ODR:         208 Hz for both accel and gyro
 * Full scale:  ±2 g (accel),  ±250 dps (gyro)                      */

#define LSM6DSO_ADDR        0x6B

/* Register addresses */
#define LSM6DSO_WHO_AM_I    0x0F   /* identity register       */
#define LSM6DSO_CTRL1_XL    0x10   /* accel config            */
#define LSM6DSO_CTRL2_G     0x11   /* gyro config             */
#define LSM6DSO_CTRL3_C     0x12   /* general config          */
#define LSM6DSO_STATUS_REG  0x1E   /* data-ready flags        */
#define LSM6DSO_OUTX_L_G    0x22   /* first gyro output byte  */

/* Convert raw 16-bit values to real units */
#define LSM6DSO_ACCEL_SENS  0.000061f   /* g   per LSB  (±2 g   scale) */
#define LSM6DSO_GYRO_SENS   0.00875f    /* dps per LSB  (±250 dps scale) */

/* Sensor data in real units */
typedef struct {
    float ax, ay, az;   /* acceleration  (g)       */
    float gx, gy, gz;   /* angular rate  (deg/s)   */
} LSM6DSO_Data_t;

/* Verify WHO_AM_I and configure ODR. Returns false if sensor not found. */
bool LSM6DSO_Init(void);

/* Read one sample directly from the output registers.
 * Returns false if no new data is ready yet.                         */
bool LSM6DSO_Read(LSM6DSO_Data_t *out);

#endif /* LSM6DSO_H */
