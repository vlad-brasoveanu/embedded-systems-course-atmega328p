#include "lsm6dso.h"
#include "i2c.h"

#define LSM6DSO_WHO_AM_I_VAL  0x6C

/* Config values written to registers during Init:
 *   CTRL3_C  = 0x04  – enable register auto-increment (makes burst reads easy)
 *   CTRL2_G  = 0x60  – gyro  ODR 208 Hz, ±250 dps full scale
 *   CTRL1_XL = 0x60  – accel ODR 208 Hz, ±2 g   full scale              */
#define CTRL3_C_VAL   0x04
#define CTRL2_G_VAL   0x60
#define CTRL1_XL_VAL  0x60

/* Pack two bytes (little-endian) into a signed 16-bit integer */
static inline int16_t to_int16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

bool LSM6DSO_Init(void)
{
    /* Check that the sensor is present and is the right type */
    uint8_t who = 0;
    if (I2C_Read(LSM6DSO_ADDR, LSM6DSO_WHO_AM_I, &who, 1) != I2C_OK)
        return false;
    if (who != LSM6DSO_WHO_AM_I_VAL)
        return false;

    uint8_t val;

    /* Enable register auto-increment so we can read multiple bytes at once */
    val = CTRL3_C_VAL;
    if (I2C_Write(LSM6DSO_ADDR, LSM6DSO_CTRL3_C, &val, 1) != I2C_OK)
        return false;

    /* Set gyro to 208 Hz output rate, ±250 dps range */
    val = CTRL2_G_VAL;
    if (I2C_Write(LSM6DSO_ADDR, LSM6DSO_CTRL2_G, &val, 1) != I2C_OK)
        return false;

    /* Set accel to 208 Hz output rate, ±2 g range */
    val = CTRL1_XL_VAL;
    if (I2C_Write(LSM6DSO_ADDR, LSM6DSO_CTRL1_XL, &val, 1) != I2C_OK)
        return false;

    return true;
}

bool LSM6DSO_Read(LSM6DSO_Data_t *out)
{
    /* STATUS_REG bit0 = accel ready, bit1 = gyro ready.
     * Wait until both are available.                                    */
    uint8_t status = 0;
    if (I2C_Read(LSM6DSO_ADDR, LSM6DSO_STATUS_REG, &status, 1) != I2C_OK)
        return false;
    if (!(status & 0x03))
        return false;   /* data not ready yet */

    /* Read 12 bytes starting at OUTX_L_G:
     * bytes 0-5  = gyro  X/Y/Z (low byte first)
     * bytes 6-11 = accel X/Y/Z (low byte first)                        */
    uint8_t raw[12];
    if (I2C_Read(LSM6DSO_ADDR, LSM6DSO_OUTX_L_G, raw, 12) != I2C_OK)
        return false;

    out->gx = to_int16(raw[0],  raw[1])  * LSM6DSO_GYRO_SENS;
    out->gy = to_int16(raw[2],  raw[3])  * LSM6DSO_GYRO_SENS;
    out->gz = to_int16(raw[4],  raw[5])  * LSM6DSO_GYRO_SENS;
    out->ax = to_int16(raw[6],  raw[7])  * LSM6DSO_ACCEL_SENS;
    out->ay = to_int16(raw[8],  raw[9])  * LSM6DSO_ACCEL_SENS;
    out->az = to_int16(raw[10], raw[11]) * LSM6DSO_ACCEL_SENS;

    return true;
}
