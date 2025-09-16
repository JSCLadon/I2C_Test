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

int32_t tlx_port_i2c_read(uint8_t addr, uint8_t *data, uint8_t count);
int32_t tlx_port_i2c_write(uint8_t addr, const uint8_t *data, uint8_t count);
void tlx_port_i2c_recover(void);
void tlx_port_i2c_reset(void);
void tlx_port_set_addr_and_wait(bool high);
void tlx_port_power_enable(void);
void tlx_port_power_disable(void);
void tlx_port_log(const void *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif // TLX493D_PLATFORM_H
