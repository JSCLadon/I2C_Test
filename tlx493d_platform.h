#ifndef TLX493D_PLATFORM_H
#define TLX493D_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

#define TLX493D_I2C_PORT i2c0
#define TLX493D_I2C_BAUDRATE (400000u)
#define TLX493D_I2C_SDA 8
#define TLX493D_I2C_SCL 9

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read bytes from the TLx493D sensor via the configured I2C peripheral.
 *
 * @param addr 8-bit sensor address (write address as used by the generic library).
 * @param data Destination buffer that receives @p count bytes.
 * @param count Number of bytes to read from the sensor.
 * @return 0 on success, positive error code on failure as expected by the generic library.
 */
int32_t tlx_port_i2c_read(uint8_t addr, uint8_t *data, uint8_t count);

/**
 * @brief Write bytes to the TLx493D sensor via the configured I2C peripheral.
 *
 * @param addr 8-bit sensor address (write address as used by the generic library).
 * @param data Source buffer containing @p count bytes to transmit.
 * @param count Number of bytes to send to the sensor.
 * @return 0 on success, positive error code on failure.
 */
int32_t tlx_port_i2c_write(uint8_t addr, const uint8_t *data, uint8_t count);

/**
 * @brief Issue an I2C recovery pattern as required by the Infineon generic library.
 */
void tlx_port_i2c_recover(void);

/**
 * @brief Issue an I2C reset pattern as required by the Infineon generic library.
 */
void tlx_port_i2c_reset(void);

/**
 * @brief Drive the ADDR line to the desired level and wait the mandated setup time.
 *
 * The Pico hardware ties ADDR, so the implementation simply enforces the delay.
 */
void tlx_port_set_addr_and_wait(bool high);

/**
 * @brief Enable sensor power or provide any required stabilization delay.
 */
void tlx_port_power_enable(void);

/**
 * @brief Disable sensor power or provide any required stabilization delay.
 */
void tlx_port_power_disable(void);

/**
 * @brief Forward log messages emitted by the generic library.
 *
 * @param data Pointer to the string buffer provided by the library.
 * @param length Number of bytes within @p data to print.
 */
void tlx_port_log(const void *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif // TLX493D_PLATFORM_H
