// TLV493D Generic Library Example using Raspberry Pi Pico
// Demonstrates how to talk to a TLV493D-A1B6 Hall sensor via Infineon's
// generic TLx493D driver library.
// This is the latest version.
// This is the very latest version


#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdio_usb.h"

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
constexpr uint32_t kLoopDelayUs = 303;                   ///< Delay between frame requests (~3.3 kHz) in microseconds.
constexpr size_t kMeasurementBufferCapacity = 256;       ///< USB transmit queue depth.
constexpr uint32_t kMeasurementPacketMagic = 0x544C564D; ///< "TLVM" marker for measurement packets.
constexpr size_t kMeasurementPacketSize = 36;            ///< Measurement packet size in bytes.
constexpr uint32_t kStatusPacketMagic = 0x544C5653;      ///< "TLVS" marker for status packets.
constexpr size_t kStatusPacketSize = 12;                 ///< Status packet size in bytes.

/**
 * @brief Container for a converted TLV493D measurement frame.
 */
struct Measurement {
    float bx_mT;   ///< Magnetic field along X in millitesla.
    float by_mT;   ///< Magnetic field along Y in millitesla.
    float bz_mT;   ///< Magnetic field along Z in millitesla.
    float temp_C;  ///< Temperature in degrees Celsius.
};

struct BufferedMeasurement {
    Measurement measurement;
    uint64_t timestamp_us;
    uint32_t sequence;
};

class MeasurementBuffer {
public:
    bool push(const BufferedMeasurement &entry)
    {
        bool clean_insert = true;
        if (count_ == kMeasurementBufferCapacity) {
            // Drop the oldest entry to make room for the newest measurement.
            tail_ = advance_index(tail_);
            --count_;
            clean_insert = false;
        }

        storage_[head_] = entry;
        head_ = advance_index(head_);
        ++count_;
        return clean_insert;
    }

    bool peek(BufferedMeasurement &entry) const
    {
        if (count_ == 0) {
            return false;
        }
        entry = storage_[tail_];
        return true;
    }

    void pop()
    {
        if (count_ == 0) {
            return;
        }
        tail_ = advance_index(tail_);
        --count_;
    }

    bool empty() const
    {
        return count_ == 0;
    }

private:
    static size_t advance_index(size_t index)
    {
        ++index;
        if (index == kMeasurementBufferCapacity) {
            index = 0;
        }
        return index;
    }

    BufferedMeasurement storage_[kMeasurementBufferCapacity]{};
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
};

MeasurementBuffer g_measurement_buffer;
uint32_t g_measurement_sequence = 0;
uint32_t g_measurement_overflows = 0;

void write_u32_le(uint8_t *dest, uint32_t value)
{
    dest[0] = static_cast<uint8_t>(value & 0xFFu);
    dest[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    dest[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    dest[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

void write_u64_le(uint8_t *dest, uint64_t value)
{
    for (int i = 0; i < 8; ++i) {
        dest[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFFu);
    }
}

void write_float_le(uint8_t *dest, float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    write_u32_le(dest, bits);
}

uint32_t compute_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

void encode_measurement_packet(const BufferedMeasurement &entry, uint8_t *buffer)
{
    write_u32_le(buffer + 0, kMeasurementPacketMagic);
    write_u32_le(buffer + 4, entry.sequence);
    write_u64_le(buffer + 8, entry.timestamp_us);
    write_float_le(buffer + 16, entry.measurement.bx_mT);
    write_float_le(buffer + 20, entry.measurement.by_mT);
    write_float_le(buffer + 24, entry.measurement.bz_mT);
    write_float_le(buffer + 28, entry.measurement.temp_C);
    const uint32_t checksum = compute_crc32(buffer, kMeasurementPacketSize - sizeof(uint32_t));
    write_u32_le(buffer + (kMeasurementPacketSize - sizeof(uint32_t)), checksum);
}

void encode_status_packet(uint32_t overflow_count, uint8_t *buffer)
{
    write_u32_le(buffer + 0, kStatusPacketMagic);
    write_u32_le(buffer + 4, overflow_count);
    const uint32_t checksum = compute_crc32(buffer, kStatusPacketSize - sizeof(uint32_t));
    write_u32_le(buffer + (kStatusPacketSize - sizeof(uint32_t)), checksum);
}

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

void queue_measurement_for_usb(const Measurement &measurement)
{
    const BufferedMeasurement entry{
        measurement,
        static_cast<uint64_t>(to_us_since_boot(get_absolute_time())),
        g_measurement_sequence++,
    };

    if (!g_measurement_buffer.push(entry)) {
        ++g_measurement_overflows;
    }
}

bool flush_overflow_status()
{
    if (g_measurement_overflows == 0) {
        return true;
    }

    uint8_t status_packet[kStatusPacketSize];
    encode_status_packet(g_measurement_overflows, status_packet);

    const size_t written = fwrite(status_packet, 1, kStatusPacketSize, stdout);
    if (written != kStatusPacketSize) {
        return false;
    }

    g_measurement_overflows = 0;
    return true;
}

void flush_measurement_buffer_to_usb()
{
    if (!stdio_usb_connected()) {
        return;
    }

    BufferedMeasurement entry{};
    while (g_measurement_buffer.peek(entry)) {
        uint8_t packet[kMeasurementPacketSize];
        encode_measurement_packet(entry, packet);

        const size_t written = fwrite(packet, 1, kMeasurementPacketSize, stdout);
        if (written != kMeasurementPacketSize) {
            break;
        }

        g_measurement_buffer.pop();
    }

    flush_overflow_status();
    fflush(stdout);
}

} // namespace
} // namespace tlv493d_demo

int main()
{
    using namespace tlv493d_demo;

    stdio_init_all();
    sleep_ms(200);

    // Give the USB host a moment to enumerate before continuing.
    for (int i = 0; i < 200 && !stdio_usb_connected(); ++i) {
        sleep_ms(5);
    }

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
            queue_measurement_for_usb(measurement);
        }

        flush_measurement_buffer_to_usb();
        sleep_us(kLoopDelayUs);
    }
}
