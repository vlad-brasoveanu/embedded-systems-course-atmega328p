#include "lis3mdl.h"
#include "i2c.h"

/*
 * CTRL_REG1: TEMP_EN=0, OM=11 (ultra-high XY), DO=111 (80 Hz), FAST_ODR=0
 *   bit7=0, bits6:5=11, bits4:2=111, bit1=0, bit0=0 → 0b01111100 = 0x7C
 *
 * CTRL_REG2: FS=00 (±4 gauss, best sensitivity) → 0x00
 *
 * CTRL_REG3: MD=00 (continuous-conversion mode) → 0x00
 *
 * CTRL_REG4: OMZ=11 (ultra-high Z performance)
 *   bits3:2=11 → 0b00001100 = 0x0C
 *
 * CTRL_REG5: BDU=1 (block data update – prevents reading mixed old/new)
 *   bit6=1 → 0x40
 */
#define CTRL_REG1_VAL  0x7C
#define CTRL_REG2_VAL  0x00
#define CTRL_REG3_VAL  0x00
#define CTRL_REG4_VAL  0x0C
#define CTRL_REG5_VAL  0x40

#define LIS3MDL_WHO_AM_I_VAL  0x3D

static inline int16_t to_int16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

bool LIS3MDL_Init(void)
{
    uint8_t who = 0;
    if (I2C_Read(LIS3MDL_ADDR, LIS3MDL_WHO_AM_I, &who, 1) != I2C_OK)
        return false;
    if (who != LIS3MDL_WHO_AM_I_VAL)
        return false;

    uint8_t val;

    val = CTRL_REG1_VAL;
    if (I2C_Write(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, &val, 1) != I2C_OK)
        return false;

    val = CTRL_REG2_VAL;
    if (I2C_Write(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, &val, 1) != I2C_OK)
        return false;

    val = CTRL_REG4_VAL;
    if (I2C_Write(LIS3MDL_ADDR, LIS3MDL_CTRL_REG4, &val, 1) != I2C_OK)
        return false;

    val = CTRL_REG5_VAL;
    if (I2C_Write(LIS3MDL_ADDR, LIS3MDL_CTRL_REG5, &val, 1) != I2C_OK)
        return false;

    /* Set continuous mode last to start measurements immediately */
    val = CTRL_REG3_VAL;
    if (I2C_Write(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, &val, 1) != I2C_OK)
        return false;

    return true;
}

bool LIS3MDL_Read(LIS3MDL_Data_t *out)
{
    uint8_t status = 0;
    if (I2C_Read(LIS3MDL_ADDR, LIS3MDL_STATUS_REG, &status, 1) != I2C_OK)
        return false;

    /* Bit 3: ZYXDA – all axes have new data */
    if (!(status & 0x08))
        return false;

    /*
     * Burst-read 6 bytes: OUT_X_L (0x28) through OUT_Z_H (0x2D).
     * Register auto-increment is always active on LIS3MDL.
     */
    uint8_t raw[6];
    if (I2C_Read(LIS3MDL_ADDR, LIS3MDL_OUT_X_L, raw, 6) != I2C_OK)
        return false;

    out->mx = to_int16(raw[0], raw[1]) * LIS3MDL_SENS_GAUSS;
    out->my = to_int16(raw[2], raw[3]) * LIS3MDL_SENS_GAUSS;
    out->mz = to_int16(raw[4], raw[5]) * LIS3MDL_SENS_GAUSS;

    return true;
}
