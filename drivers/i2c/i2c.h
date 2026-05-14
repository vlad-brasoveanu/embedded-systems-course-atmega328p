#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

/*
 * I2C (TWI) master driver for ATmega328P
 * ----------------------------------------
 * I2C uses two wires shared by all devices on the bus:
 *   SDA – data line   (A4 on Arduino Nano/Uno)
 *   SCL – clock line  (A5 on Arduino Nano/Uno)
 *
 * Every transaction follows this pattern:
 *   START → device address + R/W bit → register address → data → STOP
 *
 * Device address: 7 bits (0x00–0x7F). The driver shifts it left by 1 and
 * appends the R/W bit internally – you never need to do this yourself.
 *
 * Common clock speeds:
 *   100 000 Hz – standard mode  (most devices, more reliable over long wires)
 *   400 000 Hz – fast mode      (used in this project)
 */

/* Return code for all I2C operations */
typedef enum {
    I2C_OK  = 0,   /* transaction completed successfully */
    I2C_ERR = 1,   /* bus error, NACK, or timeout        */
} I2C_Status_t;

/* How many loop iterations to wait for TWINT before giving up.
 * At 16 MHz this is roughly 2-3 ms. Raise if using very slow devices. */
#define I2C_TIMEOUT  10000U

/* ---- Initialisation ------------------------------------------------------ */

/**
 * @brief Configure the TWI peripheral and enable it.
 * @param clock_hz SCL frequency in Hz. Typical values: 100000 or 400000.
 *
 * Call once at startup before using any other I2C function.
 * Example: I2C_Init(400000UL);
 */
void I2C_Init(uint32_t clock_hz);

/* ---- Core read / write --------------------------------------------------- */

/**
 * @brief Write one or more bytes to a device register.
 *
 * @param addr  7-bit device address (e.g. 0x6B for LSM6DSO).
 * @param reg   Register address inside the device to write to.
 * @param data  Pointer to the bytes to send.
 * @param len   Number of bytes to send.
 * @return I2C_OK on success, I2C_ERR if the device did not respond or
 *         the bus timed out.
 *
 * Example – write 0x04 to register 0x12 on device 0x6B:
 *   uint8_t val = 0x04;
 *   I2C_Write(0x6B, 0x12, &val, 1);
 */
I2C_Status_t I2C_Write(uint8_t addr, uint8_t reg,
                        const uint8_t *data, uint8_t len);

/**
 * @brief Read one or more bytes from a device register.
 *
 * Performs a combined write (register address) + repeated-START + read,
 * which is the standard way to read registers on I2C sensors.
 *
 * @param addr  7-bit device address.
 * @param reg   Register address to read from.
 * @param data  Buffer to store received bytes.
 * @param len   Number of bytes to read.
 * @return I2C_OK on success, I2C_ERR if the device did not respond or
 *         the bus timed out.
 *
 * Example – read 6 bytes starting at register 0x22:
 *   uint8_t buf[6];
 *   I2C_Read(0x6B, 0x22, buf, 6);
 */
I2C_Status_t I2C_Read(uint8_t addr, uint8_t reg,
                       uint8_t *data, uint8_t len);

/* ---- Utility ------------------------------------------------------------- */

/**
 * @brief Check whether a device is present on the bus.
 *
 * Sends a START + address byte and checks for an ACK. Does not transfer
 * any data. Useful for verifying wiring before use.
 *
 * @param addr  7-bit device address to probe.
 * @return true if the device responded with ACK, false otherwise.
 *
 * Example:
 *   if (!I2C_IsDevicePresent(0x6B)) { // LSM6DSO not found }
 */
bool I2C_IsDevicePresent(uint8_t addr);

/**
 * @brief Scan the entire 7-bit address space and record responding devices.
 *
 * Probes every address from 0x01 to 0x7E and writes the addresses of
 * responding devices into @p found. Stops early if @p max_found is reached.
 *
 * @param found      Buffer to receive the found addresses.
 * @param max_found  Size of the @p found buffer.
 * @return Number of devices found (0 if none).
 *
 * Example – find all devices, print their addresses:
 *   uint8_t addrs[16];
 *   uint8_t n = I2C_Scan(addrs, 16);
 *   for (uint8_t i = 0; i < n; i++) { ... use addrs[i] ... }
 */
uint8_t I2C_Scan(uint8_t *found, uint8_t max_found);

#endif /* I2C_H */
