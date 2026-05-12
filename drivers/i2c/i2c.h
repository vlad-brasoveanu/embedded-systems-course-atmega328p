#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

/* TWI status codes */
#define I2C_STATUS_START        0x08
#define I2C_STATUS_REP_START    0x10
#define I2C_STATUS_MT_SLA_ACK   0x18
#define I2C_STATUS_MT_SLA_NACK  0x20
#define I2C_STATUS_MT_DATA_ACK  0x28
#define I2C_STATUS_MT_DATA_NACK 0x30
#define I2C_STATUS_MR_SLA_ACK   0x40
#define I2C_STATUS_MR_SLA_NACK  0x48
#define I2C_STATUS_MR_DATA_ACK  0x50
#define I2C_STATUS_MR_DATA_NACK 0x58

/* Return codes */
typedef enum {
    I2C_OK   = 0,
    I2C_ERR  = 1,
} I2C_Status_t;

/**
 * @brief Initialize the TWI peripheral.
 * @param clock_hz Desired SCL frequency in Hz (e.g. 100000 or 400000).
 */
void I2C_Init(uint32_t clock_hz);

/**
 * @brief Write one or more bytes to a device register.
 * @param addr  7-bit device address.
 * @param reg   Register address to write to.
 * @param data  Pointer to data buffer.
 * @param len   Number of bytes to write.
 * @return I2C_OK on success, I2C_ERR on failure.
 */
I2C_Status_t I2C_Write(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len);

/**
 * @brief Read one or more bytes from a device register.
 * @param addr  7-bit device address.
 * @param reg   Register address to read from.
 * @param data  Pointer to receive buffer.
 * @param len   Number of bytes to read.
 * @return I2C_OK on success, I2C_ERR on failure.
 */
I2C_Status_t I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

#endif /* I2C_H */
