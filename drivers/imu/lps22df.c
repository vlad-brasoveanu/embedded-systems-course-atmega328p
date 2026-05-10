#include "lps22df.h"
#include "i2c.h"
#include <math.h>

/*
 * CTRL_REG1:
 *   ODR[3:0] at bits [6:3] → 0100 = 25 Hz, 0101 = 50 Hz
 *   EN_LPFP at bit [2]     → 0 (disabled)
 *   LPFP_CFG at bit [1]    → 0
 *   BDU at bit [0]         → 1 (block data update)
 *
 *   50 Hz + BDU: bits = 0_0101_00_1 → 0b00101001 = 0x29
 */
#define CTRL_REG1_VAL  0x29

#define LPS22DF_WHO_AM_I_VAL   0xB4

/* Pressure raw → hPa: divide by 4096 (2^12) */
#define LPS22DF_PRESS_SENS     (1.0f / 4096.0f)

/* Temperature raw → °C: divide by 100 */
#define LPS22DF_TEMP_SENS      (1.0f / 100.0f)

bool LPS22DF_Init(void)
{
    uint8_t who = 0;
    if (I2C_Read(LPS22DF_ADDR, LPS22DF_WHO_AM_I, &who, 1) != I2C_OK)
        return false;
    if (who != LPS22DF_WHO_AM_I_VAL)
        return false;

    uint8_t val = CTRL_REG1_VAL;
    if (I2C_Write(LPS22DF_ADDR, LPS22DF_CTRL_REG1, &val, 1) != I2C_OK)
        return false;

    return true;
}

bool LPS22DF_Read(LPS22DF_Data_t *out)
{
    uint8_t status = 0;
    if (I2C_Read(LPS22DF_ADDR, LPS22DF_STATUS, &status, 1) != I2C_OK)
        return false;

    /* Bit 0: P_DA (pressure ready), Bit 1: T_DA (temperature ready) */
    if (!(status & 0x01))
        return false;

    /*
     * Burst-read 5 bytes starting at PRESS_OUT_XL (0x28):
     *   raw[0..2] → pressure  (24-bit signed, little-endian)
     *   raw[3..4] → temperature (16-bit signed, little-endian)
     */
    uint8_t raw[5];
    if (I2C_Read(LPS22DF_ADDR, LPS22DF_PRESS_OUT_XL, raw, 5) != I2C_OK)
        return false;

    /* Reconstruct signed 24-bit pressure: sign-extend from bit 23 */
    int32_t p_raw = ((int32_t)raw[2] << 16) | ((int32_t)raw[1] << 8) | raw[0];
    if (p_raw & 0x00800000) p_raw |= 0xFF000000;  /* sign-extend */

    int16_t t_raw = (int16_t)((uint16_t)raw[4] << 8 | raw[3]);

    out->pressure_hPa   = (float)p_raw  * LPS22DF_PRESS_SENS;
    out->temperature_C  = (float)t_raw  * LPS22DF_TEMP_SENS;

    /*
     * Barometric altitude formula (hypsometric):
     *   h = 44330 * (1 - (P / P0)^(1/5.255))
     * Valid within the troposphere (~11 km). Accurate to ~±1 m in still air.
     */
    out->altitude_m = 44330.0f *
        (1.0f - powf(out->pressure_hPa / LPS22DF_SEA_LEVEL_HPA, 0.190295f));

    return true;
}
