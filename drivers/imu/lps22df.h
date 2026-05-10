#ifndef LPS22DF_H
#define LPS22DF_H

#include <stdint.h>
#include <stdbool.h>

/*
 * LPS22DF – barometric pressure / temperature sensor on Pololu AltIMU-10 v6
 *
 * I2C address: SA0 pin high (default on AltIMU-10 board) → 0x5D
 * Pressure range: 260 – 1260 hPa
 * Resolution:     1/4096 hPa per LSB
 * Temperature:    1/100 °C per LSB
 *
 * Not used for gimbal servo control but provides:
 *   - Altitude reference (barometric formula, accurate to ~±1 m in still air)
 *   - Temperature compensation reference
 */
#define LPS22DF_ADDR          0x5D

/* Register addresses */
#define LPS22DF_WHO_AM_I      0x0F   /* expected: 0xB4          */
#define LPS22DF_CTRL_REG1     0x10   /* ODR, filter, BDU        */
#define LPS22DF_CTRL_REG2     0x11   /* one-shot, reset         */
#define LPS22DF_STATUS        0x27   /* bit0: P_DA, bit1: T_DA  */
#define LPS22DF_PRESS_OUT_XL  0x28   /* pressure LSB (3 bytes)  */
#define LPS22DF_TEMP_OUT_L    0x2B   /* temperature LSB (2 bytes)*/

/* Standard atmosphere reference for altitude calculation */
#define LPS22DF_SEA_LEVEL_HPA  1013.25f

typedef struct {
    float pressure_hPa;
    float temperature_C;
    float altitude_m;       /* barometric altitude above sea level */
} LPS22DF_Data_t;

/**
 * @brief Initialise LPS22DF: verify WHO_AM_I (0xB4), set 50 Hz ODR, BDU=1.
 * @return true on success.
 */
bool LPS22DF_Init(void);

/**
 * @brief Read pressure and temperature; compute barometric altitude.
 * @param out  Destination struct.
 * @return true if new data was ready and read without I2C error.
 */
bool LPS22DF_Read(LPS22DF_Data_t *out);

#endif /* LPS22DF_H */
