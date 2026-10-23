# mcu-lidar

<p align="center">
  <img alt="platform: ESP-IDF | STM32 HAL" src="https://img.shields.io/badge/platform-ESP--IDF%20%7C%20STM32%20HAL-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

A small C library for 2D scanning lidars on **ESP-IDF 5.2+** and the **STM32
HAL**. The LDROBOT LD19 and LD06 share one driver and one handle API, and
hand out a `struct lidar_dev`. It offers CRC-checked packets of points
(angle, distance, intensity), whole revolutions assembled from them with the
rotor speed and timestamps, and LDROBOT's near filter as an opt-in. This
library replaces `esp-ld19-lidar`; the LD06, the STM32 port and the category
view are new.

The repository is named `mcu-lidar`; the C API, `CONFIG_LIDAR_*` options and
forge package key are `lidar`. The clone command below names the ESP-IDF
component directory `lidar` as well.

> **Hardware unverified.** Host tests use simulated scanners and platform
> stubs. Nothing here has run on a physical lidar.

[Supported hardware](#supported-hardware) · [Drivers](#drivers) ·
[Install](#install) ·
[Quick start](#quick-start) · [Examples](#examples) ·
[Documentation](#documentation) · [Tests](#tests) · [License](#license)

## Supported hardware

| Chip | Interface | Notes |
|---|---|---|
| **LD19** | UART | LDROBOT 360° lidar |
| **LD06** | UART | LDROBOT 360° lidar |

One Kconfig option, `LIDAR_LD19`. Wiring is in the
[wiring guide](docs/hardware.md).

## Drivers

<details><summary><b>LD19</b> — UART</summary>

4500 points/s at 10 Hz, its own speed control; **PWM pin grounded when
unused**.

- Interface: UART TX only, 230400 8N1
- Examples: ESP-IDF [ld19](examples/esp/ld19/) (esp-ld19-lidar's) · STM32 HAL
  [ld19](examples/stm/ld19/main.c), DMA to `ld19_rx_feed()`
- Chip reference: [LD06 and LD19](docs/chips.md#ld06-and-ld19)

</details>

<details><summary><b>LD06</b> — UART</summary>

Parsed by the same code as the LD19 in LDROBOT's SDK; `cfg.model =
LD19_MODEL_LD06`.

- Interface: the same as the LD19: UART TX only, 230400 8N1
- Examples: ESP-IDF [ld06](examples/esp/ld06/), the category view and the
  filter · STM32 HAL none
- Chip reference: [LD06 and LD19](docs/chips.md#ld06-and-ld19)

</details>

## Install

### ESP-IDF

```sh
git clone https://github.com/JonasAtelier/mcu-lidar components/lidar
```

```ini
CONFIG_LIDAR=y
CONFIG_LIDAR_LD19=y
```

Other components declare `REQUIRES lidar driver`.

### esp-forge / stm-forge

```make
lidar_GIT=https://github.com/JonasAtelier/mcu-lidar@stable
```

```make
PKG_lidar           = y
LIDAR_LD19          = y
```

stm-forge must be at `ee921c3` or later. See the
[installation guide](docs/installation.md). **No chip driver is enabled by
default.**

## Quick start

On ESP-IDF, exactly as esp-ld19-lidar; the driver installs the UART:

```c
#include "ld19.h"

ld19_config_t cfg = { .uart = UART_NUM_1, .rx_gpio = 16 };
ld19_handle_t lidar;
static ld19_point_t points[LD19_SCAN_MAX_POINTS];
size_t count;

ESP_ERROR_CHECK(ld19_init(&cfg, &lidar));
ld19_read_scan(lidar, points, LD19_SCAN_MAX_POINTS, &count, 300);
```

The same through the category, with the filter:

```c
struct lidar_scan scan = { .points = pts, .capacity = 600 };

if (lidar_read_scan(ld19_lidar(lidar), &scan, 300) == 0)
    lidar_filter_near(&scan);           /* opt-in */
```

- **A distance of 0 means no return**, not zero millimetres.
- On ESP-IDF the chip calls return `esp_err_t`, as esp-ld19-lidar did; on
  the HAL, **0 or a negative errno**. `lidar_*` is errno on both.
- On the STM32, feed the UART's DMA chunks to `ld19_rx_feed()` (`cfg.fed`);
  a blocking receive loses bytes at 230400 bps. See
  [API usage](docs/usage.md).

## Examples

Each driver's examples are listed under [Drivers](#drivers).

ESP-IDF examples locate this library through `EXTRA_COMPONENT_DIRS`, which
assumes the repository is checked out as `lidar`:

```sh
cd examples/esp/ld19
idf.py set-target esp32 # Replace with your board's target.
idf.py build
```

STM32 examples are snippets to integrate into your application.

## Scope and limitations

- **No motor control.** The LD19 holds its own 10 Hz. External speed control
  through its PWM pin is the app's; [hardware](docs/hardware.md) gives the
  manual's conditions.
- **The SDK's packet span check is opt-in** (`cfg.span_check`, off): the
  manual's own example packet fails it; see
  [chip reference](docs/chips.md#the-span-check).
- **No locks.** One thread per device, or your own mutex.

## Documentation

| Guide | Contents |
|---|---|
| [Installation](docs/installation.md) | Forge, ESP-IDF, STM32 HAL, build options and logging |
| [Hardware](docs/hardware.md) | Pins and wiring, the PWM pin, supply, the STM32's UART and DMA |
| [API usage](docs/usage.md) | Handles and the category view, packets and scans, the filter, the STM32 feed, errors, changes from esp-ld19-lidar |
| [Chip reference](docs/chips.md) | The packet, the CRC, angles, LD06 vs LD19, the span check and the scan rule, each cited |
| [Development](docs/development.md) | Layout, ports, bus abstraction, adding a driver and test coverage |

## Tests

```sh
make -C test test
```

Host-only; no toolchain or lidar needed. See the
[test reference](docs/development.md#tests). Error logs during the suite are
expected: tests exercise failure paths.

## License

[MIT](LICENSE). `src/lidar_filter.c` ports LDROBOT's near filter from its
SDK (ldlidar_stl_ros), which is MIT too, © 2020 SHENZHEN LDROBOT: its notice
is kept in [LICENSE-LDROBOT](LICENSE-LDROBOT).
