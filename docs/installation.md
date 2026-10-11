# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-lidar`; the forge package key, C API and configuration
options are `lidar`. The clone command below names the ESP-IDF component
directory `lidar`. It replaces `esp-ld19-lidar` (forge key `ld19`): a
project that `REQUIRES ld19` now `REQUIRES lidar`; `ld19.h` and its calls
are unchanged.

## With forge

stm-forge must be at `ee921c3` or later ("build: libraries may keep sources
in src/ subfolders"); older ones only compile `src/*.c`, so the chip drivers
in `src/ld19/` are never built and the link fails with
undefined references to the chip functions. esp-forge is unaffected: the
component's CMakeLists lists its sources.

With [esp-forge](https://github.com/JonasAtelier/esp-forge) or
[stm-forge](https://github.com/JonasAtelier/stm-forge). The catalog line in
`profiles/pkgversion` is the same for both:

```make
lidar_GIT=https://github.com/JonasAtelier/mcu-lidar@stable
```

and a target turns the library and its chips on in
`configs/<target>_defconfig`:

```make
PKG_lidar           = y
LIDAR_LD19          = y
# LIDAR_DEBUG is not set
```

or with `make menu`. esp-forge turns each line into `CONFIG_LIDAR_<CHIP>=y` in
`sdkconfig.defaults`; stm-forge passes it as `-DCONFIG_LIDAR_<CHIP>=1`.
`LIDAR_DEBUG = y` turns on `lidar_dbg()` lines on the HAL; on ESP-IDF the log
level decides.

## ESP-IDF

Clone into your project's `components/` directory. A local component is
named after its directory, and that name is what you `REQUIRES`:

```sh
git clone https://github.com/JonasAtelier/mcu-lidar components/lidar
```

```cmake
idf_component_register(SRCS "main.c" REQUIRES lidar driver)
```

Enable the chips under `idf.py menuconfig` → **Component config →
2D scanning lidar subsystem**, or put `CONFIG_LIDAR_<CHIP>=y` in your
`sdkconfig.defaults` as each example in `examples/esp/` does. With the
subsystem on and no chip picked, the build warns. ESP-IDF 5.2 or newer is
required for the `i2c_master` API.

## STM32 HAL

Copy `include/` and `src/` into an STM32 project, compile every `.c` under
`src/`, chip folders included, add `include/` and `src/` to the include path
(`src/` only needs to reach this library's own sources), and define what the
forge would: `CMSIS_device_header` as your family header (`"stm32f4xx.h"`,
`"stm32f7xx.h"`, `"stm32g0xx.h"`), `USE_HAL_DRIVER`, and
`CONFIG_LIDAR_<CHIP>=1`.

**No chip driver is built by default.** The core is always built and each
driver is opt-in: a chip whose option is absent or 0 compiles to nothing.
With the library on and no chip picked, the ESP-IDF CMakeLists warns and
`src/lidar_port_stm.c` emits a `#warning`.

**The HAL modules are yours to enable:**

| Part | Needs in hal_conf | HAL source | Gives |
|---|---|---|---|
| LD19 / LD06 | `HAL_UART_MODULE_ENABLED` | `stm32<family>xx_hal_uart.c` | `ld19_init()`, `ld19_rx_feed()` |

Feeding the driver from DMA (`cfg.fed`, recommended) also needs the app's
DMA set up for that UART (`HAL_DMA_MODULE_ENABLED`); the driver itself calls
no DMA function. Without the UART module the build stops with an `#error` in
`src/lidar_priv.h`. The bus layer's I2C and SPI code sits under its own
module switches and builds to nothing without them.

**The clock.** Packets are stamped with `lidar_time_us()`, by default
`HAL_GetTick() * 1000`: millisecond steps. It is weak; define your own for
real microseconds (the DWT cycle counter, or a free-running timer).

**printf() is the log.** Lines go out through `printf()`, so retarget
newlib's `_write()` to a UART.
