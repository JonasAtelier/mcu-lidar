# mcu-lidar

Read a 360° scan from an LDROBOT LD19 or LD06 over UART. Each point has a distance, angle and intensity.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL. Hardware testing is pending. The host tests use simulated devices.

## Contents

- [Quick start](#quick-start)
- [Scanners and examples](#scanners-and-examples)
- [Using the library in your project](#using-the-library-in-your-project)
- [License](#license)

## Quick start

Connect the LD19 TX to the configured RX pin. Use a 5 V supply rated for at least 300 mA, share ground and ground the unused PWM pin so the module controls its motor.

In a terminal with ESP-IDF activated, get the example and select the target for an ESP32. Change the target if your board uses another ESP32 chip.

```sh
git clone https://github.com/JonasAtelier/mcu-lidar.git lidar
cd lidar/examples/esp/ld19
idf.py set-target esp32
```

Check the [wiring guide](docs/hardware.md) and edit the settings in [main.c](examples/esp/ld19/main/main.c) to match your board. Connect the board by USB, then build, upload and open the serial monitor.

```sh
idf.py build flash monitor
```

The monitor reports how many points arrived in a turn, the nearest return in millimetres and the rotor speed.

## Scanners and examples

The LD19 and LD06 share a UART packet format and a driver. Read a packet for individual returns or call `lidar_read_scan()` for a full turn. The LD06 example also enables the near filter. [Packet and scan API](include/lidar.h)

| Chip | Driver header | ESP-IDF project | STM32 HAL example |
|---|---|---|---|
| [LD19](docs/chips.md#ld06-and-ld19) | [ld19.h](include/ld19.h) | [Scan and rotor speed](examples/esp/ld19/) | [Feed packets from UART DMA](examples/stm/ld19/main.c) |
| [LD06](docs/chips.md#ld06-and-ld19) | [ld19.h](include/ld19.h) | [Filtered scan](examples/esp/ld06/) | — |

A dash marks a hardware and platform combination without a dedicated example.

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Open the ESP-IDF project for your hardware, or copy the STM32 example into a project with board setup already in place. [All ESP-IDF examples](examples/esp/) · [All STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test` from the library root. [Test requirements and coverage](docs/development.md)

## License

[MIT](LICENSE). `src/lidar_filter.c` ports LDROBOT's near filter from its
SDK (ldlidar_stl_ros), which is MIT too, © 2020 SHENZHEN LDROBOT: its notice
is kept in [LICENSE-LDROBOT](LICENSE-LDROBOT).

[Browse the workshop catalog · ESP32 and STM32 drivers](https://github.com/JonasAtelier/jonas-atelier-libraries#esp32-and-stm32-drivers)
