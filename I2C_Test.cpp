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

namespace tlv493d_demo {
namespace {

constexpr float kMilliteslaPerRaw = 0.098f;              ///< Datasheet scaling from LSB to mT.
constexpr float kTemperatureSlope = 1.10f;               ///< Datasheet temperature slope (?C / LSB).
constexpr float kTemperatureOffsetRaw = 340.0f;          ///< Raw offset for TLV493D temperature conversion.
constexpr float kTemperatureReferenceC = 25.0f;          ///< Reference temperature for conversion output.
constexpr uint32_t kLoopDelayMs = 50;                    ///< Delay between frame requests in milliseconds.

/**
 * @brief Container for a converted TLV493D measurement frame.
 */
struct Measurement {
    float bx_mT;   ///< Magnetic field along X in millitesla.
    float by_mT;   ///< Magnetic field along Y in millitesla.
    float bz_mT;   ///< Magnetic field along Z in millitesla.
    float temp_C;  ///< Temperature in degrees Celsius.
};

/**
 * @brief Configure the I2C hardware for communicating with the TLV493D sensor.
 */
void init_i2c_bus()
{
    i2c_init(TLX493D_I2C_PORT, TLX493D_I2C_BAUDRATE);
    gpio_set_function(TLX493D_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TLX493D_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TLX493D_I2C_SDA);
    gpio_pull_up(TLX493D_I2C_SCL);
}

/**
 * @brief Convert a raw TLx493D data frame into engineering units.
 */
Measurement convert_frame(const TLx493D_data_frame_t &frame)
{
    Measurement out{};
    out.bx_mT = kMilliteslaPerRaw * static_cast<float>(frame.x);
    out.by_mT = kMilliteslaPerRaw * static_cast<float>(frame.y);
    out.bz_mT = kMilliteslaPerRaw * static_cast<float>(frame.z);
    out.temp_C = ((static_cast<float>(frame.temp) - kTemperatureOffsetRaw) * kTemperatureSlope) +
                 kTemperatureReferenceC;
    return out;
}

/**
 * @brief Initialise the TLV493D generic driver and select an appropriate operating mode.
 *
 * @return true when the sensor has been detected and configured, false otherwise.
 */
bool initialize_sensor()
{
    const int32_t init_result = TLx493D_init();
    if (init_result != TLx493D_OK) {
        printf("TLx493D_init failed (err=%ld)\n", init_result);
        return false;
    }

    TLV493D_sensor_type_t sensor = TLx493D_get_sensor_type();
    if (sensor != TLx493D_TYPE_TLV_A1B6) {
        printf("Unexpected sensor type (%d)\n", sensor);
        return false;
    }

    const int32_t mode_result = TLx493D_set_operation_mode(TLx493D_OP_MODE_MCM);
    if (mode_result != TLx493D_OK) {
        printf("Failed to set measurement mode (err=%ld)\n", mode_result);
    }

    return true;
}

/**
 * @brief Read and convert a measurement frame from the sensor.
 *
 * @param[out] measurement Populated with the converted values on success.
 * @return true when a fresh frame has been obtained; false on error or invalid frame.
 */
bool read_measurement(Measurement &measurement)
{
    TLx493D_data_frame_t frame{};
    const int32_t status = TLx493D_read_frame(&frame);

    if (status == TLx493D_OK) {
        measurement = convert_frame(frame);
        return true;
    }

    if (status == TLx493D_INVALID_FRAME) {
        printf("Invalid frame received, retrying...\n");
    } else {
        printf("Frame read failed (err=%ld)\n", status);
    }

    return false;
}

} // namespace
} // namespace tlv493d_demo

int main()
{
    using namespace tlv493d_demo;

    stdio_init_all();
    sleep_ms(200);

    init_i2c_bus();
    sleep_ms(5);

    if (!initialize_sensor()) {
        while (true) {
            sleep_ms(1000);
        }
    }

    printf("TLV493D-A1B6 ready\n");

    Measurement measurement{};
    while (true) {
        if (read_measurement(measurement)) {
            printf("Bx %.3f mT, By %.3f mT, Bz %.3f mT, Temp %.2f C\n",
                   measurement.bx_mT,
                   measurement.by_mT,
                   measurement.bz_mT,
                   measurement.temp_C);
        }

        sleep_ms(kLoopDelayMs);
    }
}
