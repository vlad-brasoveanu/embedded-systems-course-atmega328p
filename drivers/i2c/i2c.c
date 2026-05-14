#include "i2c.h"
#include <avr/io.h>

/*
 * TWI status codes (TWSR & 0xF8)
 * These are hardware values defined by the I2C spec and the ATmega datasheet.
 * MT = Master Transmit, MR = Master Receive, SLA = Slave Address.
 */
#define STATUS_START        0x08   /* START condition sent              */
#define STATUS_REP_START    0x10   /* repeated START sent               */
#define STATUS_MT_SLA_ACK   0x18   /* SLA+W sent, ACK received          */
#define STATUS_MT_DATA_ACK  0x28   /* data byte sent, ACK received      */
#define STATUS_MR_SLA_ACK   0x40   /* SLA+R sent, ACK received          */
#define STATUS_MR_DATA_ACK  0x50   /* data byte received, ACK sent      */
#define STATUS_MR_DATA_NACK 0x58   /* data byte received, NACK sent     */

/* ---- Low-level helpers --------------------------------------------------- */

/* Send a START (or repeated START) condition on the bus.
 * Returns I2C_ERR if the expected status code is not seen within timeout. */
static I2C_Status_t twi_start(uint8_t expected_status)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    uint16_t t = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT))) {
        if (--t == 0) return I2C_ERR;
    }
    return ((TWSR & 0xF8) == expected_status) ? I2C_OK : I2C_ERR;
}

/* Release the bus with a STOP condition. */
static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    /* TWSTO clears itself automatically; no TWINT to wait for */
}

/* Put one byte on the bus and wait for the hardware to finish.
 * Returns I2C_ERR on timeout or unexpected status code.        */
static I2C_Status_t twi_write_byte(uint8_t byte, uint8_t expected_status)
{
    TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);

    uint16_t t = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT))) {
        if (--t == 0) return I2C_ERR;
    }
    return ((TWSR & 0xF8) == expected_status) ? I2C_OK : I2C_ERR;
}

/* Read one byte from the bus. send_ack=true for every byte except the last. */
static I2C_Status_t twi_read_byte(uint8_t *out, bool send_ack)
{
    TWCR = send_ack
        ? (1 << TWINT) | (1 << TWEA) | (1 << TWEN)   /* ACK  */
        : (1 << TWINT) |               (1 << TWEN);   /* NACK */

    uint16_t t = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT))) {
        if (--t == 0) return I2C_ERR;
    }

    *out = TWDR;
    return I2C_OK;
}

/* ---- Public API ---------------------------------------------------------- */

void I2C_Init(uint32_t clock_hz)
{
    /* Set prescaler to 1 (TWPS1:0 = 00) */
    TWSR &= ~((1 << TWPS1) | (1 << TWPS0));

    /* Bit-rate register: TWBR = (F_CPU / SCL - 16) / 2  (with prescaler = 1) */
    TWBR = (uint8_t)((F_CPU / clock_hz - 16UL) / 2UL);

    /* Enable TWI peripheral */
    TWCR = (1 << TWEN);
}

I2C_Status_t I2C_Write(uint8_t addr, uint8_t reg,
                        const uint8_t *data, uint8_t len)
{
    /* START */
    if (twi_start(STATUS_START) != I2C_OK)
        goto fail;

    /* Send device address + write bit (addr << 1 | 0) */
    if (twi_write_byte((uint8_t)(addr << 1), STATUS_MT_SLA_ACK) != I2C_OK)
        goto fail;

    /* Send register address */
    if (twi_write_byte(reg, STATUS_MT_DATA_ACK) != I2C_OK)
        goto fail;

    /* Send data bytes */
    for (uint8_t i = 0; i < len; i++) {
        if (twi_write_byte(data[i], STATUS_MT_DATA_ACK) != I2C_OK)
            goto fail;
    }

    twi_stop();
    return I2C_OK;

fail:
    twi_stop();
    return I2C_ERR;
}

I2C_Status_t I2C_Read(uint8_t addr, uint8_t reg,
                       uint8_t *data, uint8_t len)
{
    /* START + write device address to set the register pointer */
    if (twi_start(STATUS_START) != I2C_OK)
        goto fail;

    if (twi_write_byte((uint8_t)(addr << 1), STATUS_MT_SLA_ACK) != I2C_OK)
        goto fail;

    if (twi_write_byte(reg, STATUS_MT_DATA_ACK) != I2C_OK)
        goto fail;

    /* Repeated START + send address again with read bit (addr << 1 | 1) */
    if (twi_start(STATUS_REP_START) != I2C_OK)
        goto fail;

    if (twi_write_byte((uint8_t)((addr << 1) | 0x01), STATUS_MR_SLA_ACK) != I2C_OK)
        goto fail;

    /* Read bytes: send ACK after every byte except the last */
    for (uint8_t i = 0; i < len; i++) {
        if (twi_read_byte(&data[i], i < (len - 1)) != I2C_OK)
            goto fail;
    }

    twi_stop();
    return I2C_OK;

fail:
    twi_stop();
    return I2C_ERR;
}

bool I2C_IsDevicePresent(uint8_t addr)
{
    /* Send START + address with write bit, check for ACK */
    if (twi_start(STATUS_START) != I2C_OK) {
        twi_stop();
        return false;
    }

    uint8_t sla_w = (uint8_t)(addr << 1);
    TWDR = sla_w;
    TWCR = (1 << TWINT) | (1 << TWEN);

    uint16_t t = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT))) {
        if (--t == 0) { twi_stop(); return false; }
    }

    bool present = ((TWSR & 0xF8) == STATUS_MT_SLA_ACK);
    twi_stop();
    return present;
}

uint8_t I2C_Scan(uint8_t *found, uint8_t max_found)
{
    uint8_t count = 0;

    for (uint8_t addr = 0x01; addr <= 0x7E && count < max_found; addr++) {
        if (I2C_IsDevicePresent(addr))
            found[count++] = addr;
    }

    return count;
}
