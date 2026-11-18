# Architecture, driver development and tests

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Repository layout

```
include/                    public API, the only directory apps include
  lidar.h                   struct lidar_dev, points, packets, scans, the near filter
  lidar_log.h               lidar_err/warn/info/dbg, lidar_log_scan()
  ld19.h                    esp-ld19-lidar's handle API, the HAL's, cfg.model, ld19_rx_feed(), ld19_lidar()
src/                        private to the library
  lidar.c                   the core: packets, revolution assembly, the deadline, the byte ring
  lidar_filter.c            LDROBOT's near filter, ported (MIT, LICENSE-LDROBOT)
  lidar_log.c               lidar_log_scan()
  lidar_priv.h              bus ops, attach and probe, errno maps, waits, the ring, the turn collector
  lidar_port_esp.c          all of that on ESP-IDF; empty elsewhere
  lidar_port_stm.c          all of that on the STM32 HAL; empty under ESP-IDF
  ld19/                     ld19.c  ld19_proto.h
test/                       host tests, `make -C test test`
  test_ld19_proto.c         the packet, the CRC against the manual's table, the manual's example
  test_filter.c             the near filter's rules, written out
  core_cases.h              core cases, shared by esp/ and stm/
  ld19_cases.h              a fake LD19 on its line and its cases, shared
  esp/                      suites on idf_stubs/, through lidar_port_esp.c
  stm/                      suites on hal_stubs/, through lidar_port_stm.c, plus the fed mode
  idf_stubs/  hal_stubs/    the slice of ESP-IDF and of the HAL the code calls
examples/
  esp/<chip>/               ESP-IDF projects: ld19 (esp-ld19-lidar's), ld06
  stm/<chip>/               STM32 snippet: ld19, DMA to ld19_rx_feed()
docs/                       the five guides; the manuals here are gitignored
CMakeLists.txt              ESP-IDF component; elsewhere a static library for add_subdirectory(); stm-forge globs src/**/*.c instead
                            the static library's options are Kconfig's bool entries, found by the type line directly under each `config`
Kconfig                     LIDAR and LIDAR_<CHIP>, for ESP-IDF and both forges' menus
idf_component.yml           ESP-IDF component manifest
LICENSE, LICENSE-LDROBOT    MIT; LDROBOT's MIT notice for the ported filter
```

Naming inside `src/<chip>/`, the same in every category library:

| File | Holds |
|---|---|
| `<chip>.c` | the driver: init, del, the ops, the chip's calls |
| `<chip>_regs.h` | register map, each value with its source citation |
| `<chip>_proto.h` | serial protocol, for parts without registers |
| `<chip>_math.h` | pure conversions, compiled and asserted on the host |
| `<chip>_priv.h` | driver state, only if needed |

The LD19 has no registers, only a packet, so `ld19_proto.h` holds its whole
wire format; it was `ld19_parse.h` in esp-ld19-lidar. The chip driver keeps
the original's four-space style; the library's own files use tabs.

## Handles first

As in the other category libraries: `struct ld19_t` embeds a
`struct lidar_dev`, init fills it and points `priv` back at the handle, and
`ld19_lidar()` hands it out. The LD06 is the same handle, named by
`cfg.model`.

The category is what every spinning scanner shares: a `read_packet` op, and
everything above it in the core. That covers the timestamp, and the
revolution assembly with its carried split packet and its too-many-points
checks. `ld19_read_scan()` uses the same assembly through
`lidar_collect_turn()`, which puts each point through a callback, so the
handle API can keep its own point type without the two aliasing.

## Platform ports

The drivers in `src/` are the same C on both platforms; what differs is in
`src/lidar_port_esp.c` and `src/lidar_port_stm.c`: the bus ops, attach,
`lidar_i2c_probe()`, `lidar_sleep_ms()`, `lidar_yield()` and
`lidar_time_us()`.

What only one platform has stays in the driver behind `#ifdef ESP_PLATFORM`:

- **ESP-IDF:** the driver installing, configuring and deleting its own UART,
  as esp-ld19-lidar did.
- **HAL:** the app's UART handle, and the fed mode.

The handle calls return `esp_err_t` on ESP-IDF and errno on the HAL, through
`LIDAR_HANDLE_RET()`.

**The fed mode.** At 230400 bps the stream does not pause, and blocking HAL
receive loses what arrives between calls. So on the HAL the driver can read
from a `struct lidar_ring` instead, which the app fills from its DMA or
interrupt callback through `ld19_rx_feed()`. It is one writer and one
reader, each owning one free-running index, with acquire and release on the
indices.

## Bus abstraction

The bus layer is mcu-tof's without its 16-bit I2C index: `struct
lidar_bus_ops` with `transfer()`, `receive(bus, rx, len, drop_stale)` for a
part that streams, and `detach()`. Every category library offers I2C, SPI
and UART, used or not, and `test/{esp,stm}/test_port_*` check all three.
The LD19 reads with `receive()`, never dropping stale input -- a scan needs
the stream unbroken -- with the bus timeout set to what is left of the
caller's deadline.

## Adding a chip

1. `include/foo.h`: the handle API, platform types behind
   `#ifdef ESP_PLATFORM`, and `struct lidar_dev *foo_lidar(foo_handle_t)`.
2. `src/foo/foo.c`: include `lidar_priv.h` first, then wrap in
   `#if CONFIG_LIDAR_FOO`.
   - Embed a `struct lidar_dev`.
   - Give it a `read_packet` that returns the next valid packet within the
     timeout, every field but `t_us`.
   - Set `rate_hz` to its measurements per second, or 0.
3. In init: refuse a bad config, allocate, attach, check the scanner speaks,
   and only then set `lidar.ops` and `lidar.priv`. Wait with deadlines on
   `lidar_elapsed_ms()`. Work in errno; return `LIDAR_HANDLE_RET(err)`.
4. Add `CONFIG_LIDAR_FOO` to:
   - the no-chip checks in `CMakeLists.txt` (its ESP-IDF part; the
     `add_subdirectory()` part reads `Kconfig`) and `src/lidar_port_stm.c`;
   - a HAL module `#error` in `src/lidar_priv.h`;
   - its sources in `CMakeLists.txt`;
   - `Kconfig`;
   - both forges' defconfigs;
   - `test/Makefile`.

## Tests

```sh
make -C test test
```

Host-only; no toolchain, HAL or ESP-IDF needed. Nothing is checked against a
real lidar. esp-ld19-lidar's tests checked the parser and the driver on
ESP-IDF stubs; their cases are covered here, and more.

The fake scanner is a byte line. It holds what the lidar already sent, plus
an optional late chunk that arrives at a set time, so a wait that lasts can
be told from one that does not. Packets are built to the manual's layout. The
CRC they are sealed with is held to the manual's table separately, so
sealing and checking cannot agree on a wrong CRC.

Each change is checked by a test that fails without it, on both platforms:
the deadline on the clock, the SDK's crossing rule against a backward step,
the span check on and off, the carried tail, truncation at capacity, the too-long and
runaway turns, the ring's wrap and overflow, and the filter's every rule.

| Binary | Checks |
|---|---|
| `test_ld19_proto` | the CRC against all 256 entries of the manual's table; the manual's example packet: CRC 0x50, every field, **refused with the span check on and decoded with it off**; decoding; the truncated step; the wrap across 360; header, VerLen and CRC refusals; the span limit at and past its edge, and nothing checked with it off |
| `test_filter` | each NearFilter rule at its threshold; breaks by angle and by distance; a far point inside a group; the join across 360, and no join when too far; one group alone; the sort |
| `esp/test_core`, `stm/test_core` | the packet stamp; **the first scan from the first crossing, the second from the carried tail with one turn of packets**; a backward step and a low angle after a high one not counted as crossings; truncation; the too-long turn flagged, not with `rate_hz` 0; the runaway; **one deadline on the clock for the turn**; `-EINVAL`s; the ring; the log line; HAL `lidar_dbg()` |
| `esp/test_ld19`, `stm/test_ld19` | init past garbage, a lone header and a bad packet; the model's name; silence `-ENXIO` and garbage `-ENODEV`, each after a second, nothing leaked; resync; the fields; **the manual's example read as a packet by default, and with `cfg.span_check` dropped whole and the next packet read**; **a packet 3 ms away caught by a 5 ms timeout**, and one 80 ms away missed only after 50 ms; turns through the handle with capacity; the category view and the filter on a real turn. ESP-IDF: the UART installed at 230400 8N1 with RX only, 4096 RX bytes and no TX; deleted at del and after a failed init; the install's own error; the `esp_err_t` codes. HAL: **the fed mode**: no HAL call ever, chunks across packets, the ring full; a stream of two turns fed in 64-byte chunks, all 80 packets |
| `esp/test_port_esp`, `stm/test_port_stm` | the bus ops (I2C, SPI, UART transfer and `receive()` with and without `drop_stale`), the presence probe, the waits and the default clock |

All build under `-std=gnu11 -Wall -Wextra -Werror -pedantic` with UBSan, and
`-Wl,--wrap` (GNU ld) to count allocations and capture `printf()`.
