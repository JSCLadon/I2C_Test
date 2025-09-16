// I2C_Test.cpp
// TLV493D Hall sensor readout example for Raspberry Pi Pico
// 2024-06-10 : Igor Petrovic
// 2025-09-16 : Updated to talk to Infineon TLV493D via I2C

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

#define TLV493D_I2C_ADDR 0x5EU
#define TLV493D_FRAME_REG 0x00

static const float TLV493D_SENSITIVITY_MT_PER_LSB = 0.098f;   // ~98 uT/LSB per datasheet
static const float TLV493D_TEMP_SLOPE_C_PER_LSB = 0.1f;       // datasheet value ~0.1 degC/LSB
static const float TLV493D_TEMP_OFFSET_C = 25.0f;             // output is relative to 25 degC reference

struct tlv493d_sample {
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int16_t temperature_raw;
};

static inline int16_t sign_extend12(uint16_t value)
{
    value &= 0x0FFF;
    if (value & 0x0800) {
        value |= 0xF000;
    }
    return (int16_t)value;
}

static bool tlv493d_init()
{
    sleep_ms(10);

    uint8_t config[] = {
        0x06,
        0x00,
        0x00,
        0x02
    };

    int written = i2c_write_blocking(I2C_PORT, TLV493D_I2C_ADDR, config, sizeof(config), false);
    return written == sizeof(config);
}

static bool tlv493d_read_raw(struct tlv493d_sample *sample)
{
    uint8_t reg = TLV493D_FRAME_REG;
    int res = i2c_write_blocking(I2C_PORT, TLV493D_I2C_ADDR, &reg, 1, true);
    if (res != 1) {
        return false;
    }

    uint8_t buffer[6] = {0};
    res = i2c_read_blocking(I2C_PORT, TLV493D_I2C_ADDR, buffer, sizeof(buffer), false);
    if (res != sizeof(buffer)) {
        return false;
    }

    uint16_t x_raw = ((buffer[1] & 0xF0) << 4) | buffer[0];
    uint16_t y_raw = ((buffer[1] & 0x0F) << 8) | buffer[2];
    uint16_t z_raw = ((uint16_t)buffer[3] << 4) | (buffer[4] >> 4);
    uint16_t t_raw = ((buffer[4] & 0x0F) << 8) | buffer[5];

    sample->x_raw = sign_extend12(x_raw);
    sample->y_raw = sign_extend12(y_raw);
    sample->z_raw = sign_extend12(z_raw);
    sample->temperature_raw = sign_extend12(t_raw);

    return true;
}

static float tlv493d_to_millitesla(int16_t raw)
{
    return (float)raw * TLV493D_SENSITIVITY_MT_PER_LSB;
}

static float tlv493d_to_celsius(int16_t raw)
{
    return (float)raw * TLV493D_TEMP_SLOPE_C_PER_LSB + TLV493D_TEMP_OFFSET_C;
}

int main()
{
    stdio_init_all();

    i2c_init(I2C_PORT, 400U * 1000U);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));

    printf("System Clock Frequency is %lu Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %lu Hz\n", clock_get_hz(clk_usb));

    if (!tlv493d_init()) {
        printf("Failed to configure TLV493D sensor\n");
        while (true) {
            tight_loop_contents();
        }
    }

    printf("TLV493D ready on I2C0 (%u kHz)\n", 400);

    struct tlv493d_sample sample;

    while (true) {
        if (tlv493d_read_raw(&sample)) {
            float x_mT = tlv493d_to_millitesla(sample.x_raw);
            float y_mT = tlv493d_to_millitesla(sample.y_raw);
            float z_mT = tlv493d_to_millitesla(sample.z_raw);
            float temperature_C = tlv493d_to_celsius(sample.temperature_raw);

            printf("Bx: %.3f mT  By: %.3f mT  Bz: %.3f mT  Temp: %.2f C\n",
                   x_mT,
                   y_mT,
                   z_mT,
                   temperature_C);
        } else {
            printf("I2C read error\n");
        }

        sleep_ms(100);
    }
}
