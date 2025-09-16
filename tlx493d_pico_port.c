#include "tlx493d_platform.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

static inline int32_t map_i2c_result(int result, uint8_t expected)
{
    if (result == (int)expected) {
        return 0;
    }

    if (result < 0) {
        return (int32_t)(-result);
    }

    /* Return a small positive error code when fewer bytes were transferred */
    return (int32_t)(expected - (uint8_t)result + 1);
}

int32_t tlx_port_i2c_read(uint8_t addr, uint8_t *data, uint8_t count)
{
    if ((data == NULL) || (count == 0)) {
        return 0;
    }

    int result = i2c_read_blocking(TLX493D_I2C_PORT, addr >> 1, data, count, false);
    return map_i2c_result(result, count);
}

int32_t tlx_port_i2c_write(uint8_t addr, const uint8_t *data, uint8_t count)
{
    if ((data == NULL) || (count == 0)) {
        return 0;
    }

    int result = i2c_write_blocking(TLX493D_I2C_PORT, addr >> 1, data, count, false);
    return map_i2c_result(result, count);
}

void tlx_port_i2c_recover(void)
{
    (void)i2c_write_blocking(TLX493D_I2C_PORT, 0x7F, NULL, 0, false);
    sleep_us(200);
}

void tlx_port_i2c_reset(void)
{
    (void)i2c_write_blocking(TLX493D_I2C_PORT, 0x00, NULL, 0, false);
    sleep_us(200);
}

void tlx_port_set_addr_and_wait(bool high)
{
    (void)high;
    /* Hardware ties ADDR pin; nothing to drive here. Ensure the required delay. */
    sleep_us(300);
}

void tlx_port_power_enable(void)
{
    /* Sensor is permanently powered from the Pico board. Provide a short delay */
    sleep_ms(2);
}

void tlx_port_power_disable(void)
{
    /* No dedicated power switch; nothing to do. */
    sleep_ms(2);
}

void tlx_port_log(const void *data, uint32_t length)
{
    if ((data == NULL) || (length == 0)) {
        return;
    }

    fwrite(data, 1, length, stdout);
    fflush(stdout);
}
