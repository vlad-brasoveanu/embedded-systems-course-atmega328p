#include "lsm6ds33.h"
#include "i2c.h"

/* ODR 208 Hz, ±2 g full-scale */
#define CTRL1_XL_VAL  0x60
/* ODR 208 Hz, ±250 dps full-scale */
#define CTRL2_G_VAL   0x60
/* Auto-increment register address on multi-byte reads */
#define CTRL3_C_VAL   0x04

static inline int16_t to_int16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

bool LSM6DS33_Init(void)
{
    uint8_t who = 0;
    if (I2C_Read(LSM6DS33_ADDR, LSM6DS33_WHO_AM_I, &who, 1) != I2C_OK)
        return false;
    if (who != 0x69)
        return false;

    uint8_t val;

    val = CTRL3_C_VAL;
    if (I2C_Write(LSM6DS33_ADDR, LSM6DS33_CTRL3_C, &val, 1) != I2C_OK)
        return false;

    val = CTRL2_G_VAL;
    if (I2C_Write(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, &val, 1) != I2C_OK)
        return false;

    val = CTRL1_XL_VAL;
    if (I2C_Write(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, &val, 1) != I2C_OK)
        return false;

    return true;
}

bool LSM6DS33_Read(LSM6DS33_Data_t *out)
{
    uint8_t status = 0;
    if (I2C_Read(LSM6DS33_ADDR, LSM6DS33_STATUS_REG, &status, 1) != I2C_OK)
        return false;

    /* Bit 0: XLDA (accel data ready), Bit 1: GDA (gyro data ready) */
    if (!(status & 0x03))
        return false;

    /* Read 6 gyro bytes then 6 accel bytes (registers 0x22-0x2D, auto-inc) */
    uint8_t raw[12];
    if (I2C_Read(LSM6DS33_ADDR, LSM6DS33_OUTX_L_G, raw, 12) != I2C_OK)
        return false;

    out->gx = to_int16(raw[0],  raw[1])  * LSM6DS33_GYRO_SENS_DPS;
    out->gy = to_int16(raw[2],  raw[3])  * LSM6DS33_GYRO_SENS_DPS;
    out->gz = to_int16(raw[4],  raw[5])  * LSM6DS33_GYRO_SENS_DPS;

    out->ax = to_int16(raw[6],  raw[7])  * LSM6DS33_ACCEL_SENS_G;
    out->ay = to_int16(raw[8],  raw[9])  * LSM6DS33_ACCEL_SENS_G;
    out->az = to_int16(raw[10], raw[11]) * LSM6DS33_ACCEL_SENS_G;

    return true;
}
