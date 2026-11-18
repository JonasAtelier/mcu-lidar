# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-lidar`; the forge package key, C API and configuration
options are `lidar`. The clone command below names the ESP-IDF component
directory `lidar`. It replaces `esp-ld19-lidar` (forge key `ld19`): a
project that `REQUIRES ld19` now `REQUIRES lidar`; `ld19.h` and its calls
are unchanged.

Three step-by-step guides come first: [ESP-IDF](#esp-idf-step-by-step),
[STM32 with CMake](#with-cmake-a-cubemx-cmake-project) and
[STM32 without CMake](#without-cmake-stm32cubeide-a-makefile). None
needs a forge; if you build with esp-forge or stm-forge, see
[With forge](#with-forge). The steps use the LD19; the same option and
driver read the LD06 (`cfg.model`). The reference follows the guides.

**Arduino is not supported yet.** TODO: make the library usable from
Arduino (Arduino IDE and PlatformIO).

## ESP-IDF, step by step

**You need** ESP-IDF 5.2 or newer, with `idf.py` working in your terminal, and
an ESP-IDF project: a folder with a `CMakeLists.txt` and a `main/` folder.
`idf.py create-project my-project` makes one.

ESP-IDF calls a library a **component**: a folder with its own
`CMakeLists.txt` that the build finds and compiles. This library is the
component `lidar`.

1. **Get the library.** Pick one way.

   - **Component manager**, the simplest: the first build downloads the
     library into `managed_components/lidar`. Create
     `main/idf_component.yml`:

     ```yaml
     dependencies:
       lidar:
         git: https://github.com/JonasAtelier/mcu-lidar.git
         version: stable
     ```

   - **Clone**, if you want to edit the library or build offline. In the
     project folder:

     ```sh
     git clone https://github.com/JonasAtelier/mcu-lidar components/lidar
     ```

     Keep the folder name `lidar`: ESP-IDF names a component after its
     folder.

   ```
   my-project/
   ├── CMakeLists.txt
   ├── sdkconfig.defaults       step 2
   ├── main/
   │   ├── CMakeLists.txt       step 3
   │   ├── idf_component.yml    step 1, component manager
   │   └── my-project.c         step 4
   └── components/
       └── lidar/               step 1, clone
   ```

2. **Turn your chip on.** No chip driver is built until you do. Create
   `sdkconfig.defaults` in the project folder, a text file of settings you
   choose, with this line:

   ```ini
   CONFIG_LIDAR_LD19=y
   ```

   It builds the driver for both the LD19 and the LD06. ESP-IDF copies
   these lines into `sdkconfig`, the project's full settings file, only when
   it creates that file: **if `sdkconfig` already exists, delete it** so
   your line is read. `idf.py menuconfig` → **Component config → 2D
   scanning lidar subsystem** does the same from a menu.

3. **Let your code see the library.** A new project's `main/CMakeLists.txt`
   has no `REQUIRES` and already sees every component. If yours has a
   `REQUIRES` list, add `lidar` and `driver` to it:

   ```cmake
   idf_component_register(SRCS "main.c" REQUIRES lidar driver)
   ```

4. **Write the smallest test.** Copy
   [examples/esp/ld19/main/main.c](../examples/esp/ld19/main/main.c) over
   your main source file and set `LIDAR_UART` and `LIDAR_RX_GPIO` to your
   wiring. [API usage](usage.md) explains the calls.

5. **Build.**

   ```sh
   idf.py set-target esp32    # your chip: esp32s3, esp32c3, ...
   idf.py build
   ```

   A good build ends with `Project build complete.` Flash it as any ESP-IDF
   app and open the serial monitor: the first line ends in
   `ld19-example: rotor at 3600 deg/s`, give or take, the lidar's nominal
   10 turns a second; then the example logs a scan about once a second.

**If it fails**

| The build prints | Why | Fix |
|---|---|---|
| `Failed to resolve component 'lidar' required by component 'main': unknown name.` | The clone is not in a folder named `lidar` | Rename it to `components/lidar` |
| `fatal error: ld19.h: No such file or directory` | `main/CMakeLists.txt` has a `REQUIRES` list without `lidar` | Step 3 |
| `lidar: CONFIG_LIDAR is on but no chip driver is selected` (a CMake warning) | No chip turned on | Step 2 |
| ``undefined reference to `ld19_init'`` | That chip is not turned on, often because `sdkconfig` existed before you edited `sdkconfig.defaults` | Delete `sdkconfig`, build again |

## STM32, step by step

**You need** a project for an STM32F4, F7 or G0 that uses ST's **HAL**, the
vendor library behind `HAL_Init()`, `HAL_UART_Init()` and friends, with the
UART your lidar is wired to already set up. STM32CubeMX makes such a
project; with **Toolchain / IDE: CMake** it makes a CMake one, the main case
below.

Two terms. A **HAL module** is the part of the HAL for one peripheral: I2C,
SPI, UART. It is compiled only when it is switched on in **hal_conf**, the
file `Core/Inc/stm32f4xx_hal_conf.h` (`stm32f7xx_hal_conf.h` on an F7,
`stm32g0xx_hal_conf.h` on a G0). CubeMX switches a module on when you enable
that peripheral.

### With CMake (a CubeMX CMake project)

1. **Get the library.** In the project folder, next to the top-level
   `CMakeLists.txt`:

   ```sh
   git clone https://github.com/JonasAtelier/mcu-lidar
   ```

   ```
   my-board/
   ├── CMakeLists.txt                  step 2
   ├── cmake/stm32cubemx/              made by CubeMX
   ├── Core/
   │   ├── Inc/stm32f4xx_hal_conf.h    hal_conf, step 3
   │   └── Src/main.c                  step 4
   ├── Drivers/                        the HAL, copied in by CubeMX
   └── mcu-lidar/                      step 1
   ```

2. **Add it to the build.** In the top `CMakeLists.txt`, after the line
   `add_subdirectory(cmake/stm32cubemx)`, add:

   ```cmake
   set(CONFIG_LIDAR_LD19 ON)           # LD19 and LD06
   set(MCU_HAL_TARGET stm32cubemx)
   set(MCU_DEVICE_HEADER stm32f4xx.h)  # stm32f7xx.h on an F7, stm32g0xx.h on a G0
   add_subdirectory(mcu-lidar)
   ```

   and add `lidar` to the link line CubeMX already wrote, so it reads:

   ```cmake
   target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx lidar)
   ```

   Do not add `PRIVATE`: CubeMX's line has no such keyword, and CMake refuses
   to mix the two forms on one target.

   `stm32cubemx` is a CMake *target*, a named bundle of settings, that CubeMX
   makes to carry the HAL's include folders and defines; `MCU_HAL_TARGET`
   lends those to the library, which works with any CubeMX version.
   `MCU_DEVICE_HEADER` is your chip family's main header.

3. **Check the HAL module.** The LD19 talks UART, so hal_conf must have
   `#define HAL_UART_MODULE_ENABLED`; enabling USART2 (or any U(S)ART) in
   asynchronous mode in CubeMX does that. Set it to 230400 baud, 8N1, and
   give its RX a DMA channel in circular mode (that switches on
   `HAL_DMA_MODULE_ENABLED`), with the USART's global interrupt enabled. The
   lidar sends without a pause, and DMA is what keeps every byte.

4. **Write the smallest test.** It is
   [examples/stm/ld19/main.c](../examples/stm/ld19/main.c) fitted into
   CubeMX's `Core/Src/main.c`. Paste each part between the
   `USER CODE BEGIN` and `END` comments named, so a regenerate keeps it.

   After `/* USER CODE BEGIN Includes */`:

   ```c
   #include "ld19.h"
   #include "lidar.h"
   #include "lidar_log.h"
   ```

   After `/* USER CODE BEGIN PV */`:

   ```c
   static ld19_handle_t lidar;
   static uint8_t dma_buf[256];
   static uint16_t dma_old;  /* where the last event left off */
   ```

   After `/* USER CODE BEGIN 2 */`, which comes after
   `MX_USART2_UART_Init()`:

   ```c
     static struct lidar_point points[LD19_SCAN_MAX_POINTS];
     struct lidar_scan scan = {
       .points = points,
       .capacity = LD19_SCAN_MAX_POINTS,
     };
     ld19_config_t cfg = { .uart = &huart2, .fed = true };

     if (ld19_init(&cfg, &lidar))
       Error_Handler();
     HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_buf, sizeof(dma_buf));
   ```

   After `/* USER CODE BEGIN 3 */`, inside the `while (1)` loop:

   ```c
       /* -ETIMEDOUT: no turn in 300 ms -- supply, wire, rotor. */
       if (lidar_read_scan(ld19_lidar(lidar), &scan, 300) == 0)
         lidar_log_scan(ld19_lidar(lidar), &scan);
   ```

   After `/* USER CODE BEGIN 4 */`, below `main()`:

   ```c
   /*
    * ReceiveToIdle in circular mode: size is the position the DMA has
    * written up to; hand over everything since the last call, wrapping at
    * the end.
    */
   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
   {
     if (huart != &huart2)
       return;
     if (size > dma_old) {
       ld19_rx_feed(lidar, dma_buf + dma_old, size - dma_old);
     } else if (size < dma_old) {
       ld19_rx_feed(lidar, dma_buf + dma_old, sizeof(dma_buf) - dma_old);
       ld19_rx_feed(lidar, dma_buf, size);
     }
     dma_old = size == sizeof(dma_buf) ? 0 : size;
   }
   ```

   The lines go out through `printf()`: send that to another UART by
   writing newlib's `_write()`, and add this to `CMakeLists.txt` so numbers
   are printed:

   ```cmake
   target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
   ```

   [API usage](usage.md) explains the calls.

5. **Build** as before, for example `cmake --build` with your build folder.
   A good build shows a `Linking C executable` line and no error. On the
   UART, one line per turn, such as
   `lidar: ld19: t 2000000 us  450 points  3600 deg/s  100 ms  nearest 800 mm at 30.0 deg`.

**If it fails**

| You see | Why | Fix |
|---|---|---|
| `lidar: no chip driver is selected; set(CONFIG_LIDAR_<CHIP> ON) before add_subdirectory()` (a CMake warning), then ``undefined reference to `ld19_init'`` | No chip turned on, or turned on after `add_subdirectory(mcu-lidar)` | Step 2 |
| `#error "LIDAR_LD19 needs the HAL UART module: ...` | The HAL module is off | Enable the peripheral in CubeMX and regenerate |
| `#error "lidar: CMSIS_device_header is not defined; ...` | `MCU_DEVICE_HEADER` is missing | Step 2 |
| `fatal error: stm32f4xx.h: No such file or directory` | `MCU_HAL_TARGET` is missing | Step 2 |
| `Target "stm32cubmx" not found.` (a CMake error) | `MCU_HAL_TARGET` is misspelt | Step 2 |
| `The plain signature for target_link_libraries has already been used with the target` | `PRIVATE` added to the link line | Step 2 |
| No error, but the numbers print as blanks | The C library (newlib-nano) leaves out `%f` | The `-u _printf_float` line of step 4 |

### Without CMake (STM32CubeIDE, a Makefile)

1. Copy `include/` and `src/` of this repository into your project, for
   example to `Middlewares/mcu-lidar/`, keeping the chip folder inside
   `src/`.
2. Add every `.c` file under `src/` to the build, chip folder included.
3. Add two include paths, `Middlewares/mcu-lidar/include` and
   `Middlewares/mcu-lidar/src`, and two defines:
   `CMSIS_device_header="stm32f4xx.h"`, quotes included, and
   `CONFIG_LIDAR_LD19=1`. CubeMX already defines `USE_HAL_DRIVER` and your
   device, such as `STM32F405xx`.
4. Steps 3 to 5 of the CMake guide apply as they are; `-u _printf_float`
   goes in the linker flags.

Where these settings usually live: in STM32CubeIDE, **Project → Properties →
C/C++ Build → Settings → Tool Settings → MCU GCC Compiler** (Include paths,
Preprocessor); in a CubeMX Makefile, `C_SOURCES`, `C_INCLUDES` and `C_DEFS`.
The build errors are the ones in the table above; the fix is the matching
define or include path instead of a CMake line. One more:
`#include expects "FILENAME" or <FILENAME>` means `CMSIS_device_header`
lost its quotes.

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

Setup: [ESP-IDF, step by step](#esp-idf-step-by-step). For reference:

- In `main/idf_component.yml`, `version` is a git ref; `stable` is the
  released one.
- A clone in `components/` is a local component, named after its folder;
  that name is what other components `REQUIRES`. The examples in
  `examples/esp/` need no name: their `main` lists no `REQUIRES`, and ESP-IDF
  gives `main` every component, so they build from any checkout name.
- With the subsystem on and no chip picked, the build warns.
- ESP-IDF 5.2 or newer is required for the `i2c_master` API.

## STM32 HAL

Setup: [With CMake](#with-cmake-a-cubemx-cmake-project) or
[Without CMake](#without-cmake-stm32cubeide-a-makefile). The
`add_subdirectory()` route builds a static library `lidar`. For reference:

- Each `CONFIG_LIDAR_<CHIP>` (and `CONFIG_LIDAR_DEBUG`) that is true reaches
  the compiler as `CONFIG_LIDAR_<CHIP>=1`, for the library and your app.
- `MCU_HAL_TARGET` is a target that carries your `USE_HAL_DRIVER`, device
  define (`STM32F405xx`, ...) and the include dirs of the HAL, CMSIS and your
  `stm32<family>xx_hal_conf.h`. The library copies only those include dirs
  and defines, never the target's sources, so this works with any CubeMX
  version. `MCU_DEVICE_HEADER` becomes `CMSIS_device_header`, which CubeMX
  does not define. Every `mcu-*` library reads these two, so set them once.
  Leave them unset if your project already provides both.
- The CPU flags (`-mcpu`, `-mfloat-abi`, ...) must reach the library too;
  CubeMX's toolchain file sets them for every target.
- Without CMake, `src/` only needs to be on the include path of this
  library's own sources; the defines are what stm-forge passes.

**No chip driver is built by default.** The core is always built and each
driver is opt-in: a chip whose option is absent or 0 compiles to nothing.
With the library on and no chip picked, the CMakeLists warns (ESP-IDF and
`add_subdirectory()`) and `src/lidar_port_stm.c` emits a `#warning`.

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
newlib's `_write()` to a UART. stm-forge links newlib-nano
(`--specs=nano.specs`), whose `printf()` drops `%f` unless the app links with
`-u _printf_float`: add `target_link_options(firmware PRIVATE -u _printf_float)`
to your app's `CMakeLists.txt` (in a CubeMX project the target is
`${CMAKE_PROJECT_NAME}`), or `lidar_log_scan()` prints blanks where the
speed and the angle go.
