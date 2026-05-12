#include "i2c.h"
#include <avr/io.h>

/* --- low-level helpers -------------------------------------------------- */

static void twi_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    /* TWSTO clears itself; no TWINT to poll */
}

static I2C_Status_t twi_write_byte(uint8_t byte)
{
    TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    uint8_t status = TWSR & 0xF8;
    if (status != I2C_STATUS_MT_SLA_ACK  &&
        status != I2C_STATUS_MT_DATA_ACK &&
        status != I2C_STATUS_MR_SLA_ACK)
        return I2C_ERR;
    return I2C_OK;
}

static uint8_t twi_read_byte(bool send_ack)
{
    if (send_ack)
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
    else
        TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

/* --- public API ---------------------------------------------------------- */

void I2C_Init(uint32_t clock_hz)
{
    /* Prescaler = 1 (TWPS1:0 = 00) */
    TWSR &= ~((1 << TWPS1) | (1 << TWPS0));

    /* TWBR = (F_CPU / clock_hz - 16) / 2  (prescaler = 1) */
    TWBR = (uint8_t)((F_CPU / clock_hz - 16UL) / 2UL);

    TWCR = (1 << TWEN);
}

I2C_Status_t I2C_Write(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    twi_start();
    if ((TWSR & 0xF8) != I2C_STATUS_START) { twi_stop(); return I2C_ERR; }

    /* SLA+W */
    if (twi_write_byte((uint8_t)(addr << 1)) != I2C_OK) { twi_stop(); return I2C_ERR; }

    /* Register address */
    TWDR = reg;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != I2C_STATUS_MT_DATA_ACK) { twi_stop(); return I2C_ERR; }

    /* Data bytes */
    for (uint8_t i = 0; i < len; i++) {
        TWDR = data[i];
        TWCR = (1 << TWINT) | (1 << TWEN);
        while (!(TWCR & (1 << TWINT)));
        if ((TWSR & 0xF8) != I2C_STATUS_MT_DATA_ACK) { twi_stop(); return I2C_ERR; }
    }

    twi_stop();
    return I2C_OK;
}

I2C_Status_t I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    /* Write register address first (no STOP between write and read) */
    twi_start();
    if ((TWSR & 0xF8) != I2C_STATUS_START) { twi_stop(); return I2C_ERR; }

    if (twi_write_byte((uint8_t)(addr << 1)) != I2C_OK) { twi_stop(); return I2C_ERR; }

    TWDR = reg;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != I2C_STATUS_MT_DATA_ACK) { twi_stop(); return I2C_ERR; }

    /* Repeated START then SLA+R */
    twi_start();
    if ((TWSR & 0xF8) != I2C_STATUS_REP_START) { twi_stop(); return I2C_ERR; }

    if (twi_write_byte((uint8_t)((addr << 1) | 0x01)) != I2C_OK) { twi_stop(); return I2C_ERR; }

    /* Read bytes: ACK all except last */
    for (uint8_t i = 0; i < len; i++)
        data[i] = twi_read_byte(i < (len - 1));

    twi_stop();
    return I2C_OK;
}
