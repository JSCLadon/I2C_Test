// TLV493D Generic Library Example using Raspberry Pi Pico
// Demonstrates how to talk to a TLV493D-A1B6 Hall sensor via Infineon''s
// generic TLx493D driver library.

#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

extern "C" {
#include "vendor/src/TLx493D/TLx493D.h"
}

#include "tlx493d_platform.h"

static void init_i2c_bus()
{
    i2c_init(TLX493D_I2C_PORT, TLX493D_I2C_BAUDRATE);
    gpio_set_function(TLX493D_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TLX493D_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TLX493D_I2C_SDA);
    gpio_pull_up(TLX493D_I2C_SCL);
}

static float convert_field_to_mT(int16_t raw)
{
    // Scaling factor from the TLV493D-A1B6 datasheet
    return 0.098f * static_cast<float>(raw);
}

static float convert_temperature_to_c(float raw)
{
    // Conversion formula recommended by Infineon: ((raw - 340) * 1.1) + 25
    return ((raw - 340.0f) * 1.10f) + 25.0f;
}

int main()
{
    stdio_init_all();
    sleep_ms(200);

    init_i2c_bus();
    sleep_ms(5);

    int32_t status = TLx493D_init();
    if (status != TLx493D_OK) {
        printf("TLx493D_init failed (err=%ld)\n", status);
        while (true) {
            sleep_ms(1000);
        }
    }

    TLV493D_sensor_type_t sensor = TLx493D_get_sensor_type();
    if (sensor != TLx493D_TYPE_TLV_A1B6) {
        printf("Unexpected sensor type (%d)\n", sensor);
        while (true) {
            sleep_ms(1000);
        }
    }

    status = TLx493D_set_operation_mode(TLx493D_OP_MODE_MCM);
    if (status != TLx493D_OK) {
        printf("Failed to set measurement mode (err=%ld)\n", status);
    }

    printf("TLV493D-A1B6 ready\n");

    TLx493D_data_frame_t frame = {};

    while (true) {
        status = TLx493D_read_frame(&frame);
        if (status == TLx493D_OK) {
            float bx = convert_field_to_mT(frame.x);
            float by = convert_field_to_mT(frame.y);
            float bz = convert_field_to_mT(frame.z);
            float temp_c = convert_temperature_to_c(static_cast<float>(frame.temp));

            printf("Bx %.3f mT, By %.3f mT, Bz %.3f mT, Temp %.2f C\n",
                   bx, by, bz, temp_c);
        } else if (status == TLx493D_INVALID_FRAME) {
            printf("Invalid frame received, retrying...\n");
        } else {
            printf("Frame read failed (err=%ld)\n", status);
        }

        sleep_ms(200);
    }
}
