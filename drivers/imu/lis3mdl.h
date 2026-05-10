#ifndef LIS3MDL_H
#define LIS3MDL_H

#include <stdint.h>
#include <stdbool.h>

/*
 * LIS3MDL – 3-axis magnetometer on Pololu AltIMU-10 v6
 *
 * I2C address: SA1 pin high (default on AltIMU-10 board) → 0x1E
 * Used for tilt-compensated heading (yaw reference) fed into the
 * Madgwick 9-DOF AHRS filter.
 */
#define LIS3MDL_ADDR         0x1E

/* Register addresses */
#define LIS3MDL_WHO_AM_I     0x0F   /* expected: 0x3D                    */
#define LIS3MDL_CTRL_REG1    0x20   /* ODR, XY operating mode, TEMP_EN   */
#define LIS3MDL_CTRL_REG2    0x21   /* full-scale selection               */
#define LIS3MDL_CTRL_REG3    0x22   /* operating mode                     */
#define LIS3MDL_CTRL_REG4    0x23   /* Z operating mode                   */
#define LIS3MDL_CTRL_REG5    0x24   /* BDU                                */
#define LIS3MDL_STATUS_REG   0x27   /* bit 3: ZYXDA – all axes ready      */
#define LIS3MDL_OUT_X_L      0x28   /* X low byte (6 bytes follow)        */

/*
 * Sensitivity at ±4 gauss full-scale: 6842 LSB / gauss
 * → 1 / 6842 ≈ 0.0001461 gauss / LSB
 */
#define LIS3MDL_SENS_GAUSS   0.0001461f

typedef struct {
    float mx, my, mz;   /* magnetic field in gauss */
} LIS3MDL_Data_t;

/**
 * @brief Initialise the LIS3MDL: verify WHO_AM_I (0x3D), set 80 Hz ODR,
 *        ±4 G full-scale, ultra-high-performance XYZ, continuous mode.
 * @return true on success.
 */
bool LIS3MDL_Init(void);

/**
 * @brief Read all three magnetic axes and convert to gauss.
 * @param out  Destination struct.
 * @return true if new data was available and read without I2C error.
 */
bool LIS3MDL_Read(LIS3MDL_Data_t *out);

#endif /* LIS3MDL_H */
