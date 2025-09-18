#include <array>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <termios.h>

namespace {

constexpr uint32_t kMeasurementPacketMagic = 0x544C564D; // "TLVM"
constexpr uint32_t kStatusPacketMagic = 0x544C5653;      // "TLVS"
constexpr std::size_t kMeasurementPacketSize = 36;
constexpr std::size_t kStatusPacketSize = 12;

uint32_t compute_crc32(const uint8_t *data, std::size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

uint32_t read_u32_le(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

uint64_t read_u64_le(const uint8_t *data)
{
    uint64_t value = 0;
    for (int i = 7; i >= 0; --i) {
        value = (value << 8) | data[i];
    }
    return value;
}

float read_float_le(const uint8_t *data)
{
    uint32_t raw = read_u32_le(data);
    float value;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

struct MeasurementPacket {
    uint32_t sequence = 0;
    uint64_t timestamp_us = 0;
    float bx_mT = 0.0f;
    float by_mT = 0.0f;
    float bz_mT = 0.0f;
    float temperature_C = 0.0f;
};

struct StatusPacket {
    uint32_t overflow_count = 0;
};

enum class PacketType {
    Measurement,
    Status,
};

struct Packet {
    PacketType type;
    MeasurementPacket measurement{};
    StatusPacket status{};
};

std::optional<Packet> try_decode_packet(std::vector<uint8_t> &buffer)
{
    // Ensure we have at least a magic field to inspect.
    while (buffer.size() >= sizeof(uint32_t)) {
        uint32_t magic = read_u32_le(buffer.data());

        if (magic == kMeasurementPacketMagic) {
            if (buffer.size() < kMeasurementPacketSize) {
                return std::nullopt;
            }

            const uint32_t expected_crc = read_u32_le(buffer.data() + kMeasurementPacketSize - sizeof(uint32_t));
            const uint32_t computed_crc = compute_crc32(buffer.data(), kMeasurementPacketSize - sizeof(uint32_t));
            if (expected_crc != computed_crc) {
                buffer.erase(buffer.begin());
                continue;
            }

            MeasurementPacket measurement;
            measurement.sequence = read_u32_le(buffer.data() + 4);
            measurement.timestamp_us = read_u64_le(buffer.data() + 8);
            measurement.bx_mT = read_float_le(buffer.data() + 16);
            measurement.by_mT = read_float_le(buffer.data() + 20);
            measurement.bz_mT = read_float_le(buffer.data() + 24);
            measurement.temperature_C = read_float_le(buffer.data() + 28);

            buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(kMeasurementPacketSize));

            Packet packet{PacketType::Measurement};
            packet.measurement = measurement;
            return packet;
        }

        if (magic == kStatusPacketMagic) {
            if (buffer.size() < kStatusPacketSize) {
                return std::nullopt;
            }

            const uint32_t expected_crc = read_u32_le(buffer.data() + kStatusPacketSize - sizeof(uint32_t));
            const uint32_t computed_crc = compute_crc32(buffer.data(), kStatusPacketSize - sizeof(uint32_t));
            if (expected_crc != computed_crc) {
                buffer.erase(buffer.begin());
                continue;
            }

            StatusPacket status;
            status.overflow_count = read_u32_le(buffer.data() + 4);

            buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(kStatusPacketSize));

            Packet packet{PacketType::Status};
            packet.status = status;
            return packet;
        }

        // Unknown magic: drop one byte and continue searching.
        buffer.erase(buffer.begin());
    }

    return std::nullopt;
}

class SerialStream {
public:
    SerialStream(std::string device_path, speed_t baud)
    {
        fd_ = ::open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            throw std::system_error(errno, std::generic_category(), "Failed to open serial device");
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            throw std::system_error(errno, std::generic_category(), "tcgetattr failed");
        }

        cfmakeraw(&tty);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;

        if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0) {
            throw std::system_error(errno, std::generic_category(), "Failed to set baud rate");
        }

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            throw std::system_error(errno, std::generic_category(), "tcsetattr failed");
        }
    }

    ~SerialStream()
    {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    SerialStream(const SerialStream &) = delete;
    SerialStream &operator=(const SerialStream &) = delete;

    ssize_t read(uint8_t *buffer, std::size_t length)
    {
        while (true) {
            ssize_t result = ::read(fd_, buffer, length);
            if (result >= 0) {
                return result;
            }

            if (errno == EINTR) {
                continue;
            }

            throw std::system_error(errno, std::generic_category(), "Serial read failed");
        }
    }

private:
    int fd_ = -1;
};

std::atomic<bool> g_keep_running{true};

void handle_signal(int signal)
{
    if (signal == SIGINT || signal == SIGTERM) {
        g_keep_running.store(false);
    }
}

void install_signal_handlers()
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
}

void print_measurement(const MeasurementPacket &m)
{
    const double timestamp_seconds = static_cast<double>(m.timestamp_us) / 1'000'000.0;

    std::cout << std::fixed << std::setprecision(3)
              << "seq=" << m.sequence
              << " t=" << timestamp_seconds << "s"
              << " Bx=" << m.bx_mT << "mT"
              << " By=" << m.by_mT << "mT"
              << " Bz=" << m.bz_mT << "mT"
              << " Temp=" << m.temperature_C << "C"
              << std::endl;
}

void print_status(const StatusPacket &status)
{
    std::cerr << "[INFO] Dropped measurements reported by Pico: " << status.overflow_count << std::endl;
}

} // namespace

int main(int argc, char *argv[])
{
    try {
        std::string device = "/dev/ttyACM0";
        if (argc > 1) {
            device = argv[1];
        }

        install_signal_handlers();

        SerialStream serial(device, B115200);
        std::vector<uint8_t> rx_buffer;
        rx_buffer.reserve(4096);
        std::array<uint8_t, 1024> scratch{};

        while (g_keep_running.load()) {
            ssize_t bytes_read = serial.read(scratch.data(), scratch.size());
            if (bytes_read == 0) {
                continue;
            }

            rx_buffer.insert(rx_buffer.end(), scratch.begin(),
                             scratch.begin() + static_cast<std::size_t>(bytes_read));

            while (auto packet = try_decode_packet(rx_buffer)) {
                if (packet->type == PacketType::Measurement) {
                    print_measurement(packet->measurement);
                } else {
                    print_status(packet->status);
                }
            }
        }

        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}

