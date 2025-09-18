# Introduction

This repository contains two closely related pieces of software for working with an
Infineon TLV493D-A1B6 3D Hall sensor:

* **Firmware** for the Raspberry Pi Pico that communicates with the sensor via I²C and
  streams converted magnetic-field and temperature readings over the USB CDC interface.
* **Host utilities** that run on a Linux-based Raspberry Pi and decode the binary frames
  produced by the Pico firmware.

The Pico firmware is based on Infineon's generic TLx493D driver library. Measurements are
converted to millitesla (Bx, By, Bz) and degrees Celsius before transmission.

# Getting Started

## Firmware prerequisites

* Raspberry Pi Pico SDK 2.0 or newer
* CMake 3.13+
* ARM GCC toolchain compatible with the Pico SDK

To build the firmware target:

```bash
mkdir build
cd build
cmake ..
ninja  # or: make
```

The resulting UF2 image can be found in the build directory and flashed onto the Pico in
the usual manner (BOOTSEL + drag-and-drop).

## Host-side reader prerequisites

* GNU g++ with C++17 support
* A Raspberry Pi (or any Linux host) connected to the Pico via USB

Compile the reader utility directly from the repository root:

```bash
g++ -std=c++17 -Wall -Wextra -O2 -o pico_usb_reader pico_usb_reader.cpp
```

The resulting executable listens to the USB CDC device exposed by the Pico (default:
`/dev/ttyACM0`) and prints decoded measurements to `stdout`.

# Usage

1. Flash the Pico firmware and connect the board via USB.
2. Build the host utility as described above.
3. Run the reader and optionally specify the serial device path:

   ```bash
   ./pico_usb_reader               # Uses /dev/ttyACM0 by default
   ./pico_usb_reader /dev/ttyACM1  # Explicit device selection
   ```

The tool validates the CRC-32 checksum embedded in each measurement frame and reports
sensor readings in human-readable form. Any overflow status packets emitted by the Pico
are logged to `stderr`.

# Contribute

Issues and pull requests that improve the firmware, reader utility, or documentation are
always welcome. Please describe your use case and test setup when proposing changes so
that others can reproduce the results.
