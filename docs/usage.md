# API usage

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## The handle and the category view

| Call | Does |
|---|---|
| `ld19_init(&cfg, &h)` | ESP-IDF: installs and configures `cfg.uart` itself (230400 8N1, RX on `cfg.rx_gpio`), as esp-ld19-lidar did, then waits up to 1 s for a valid packet. HAL: takes the app's `cfg.uart`; see [the STM32 feed](#the-stm32-feed) |
| `ld19_read_packet(h, &p, ms)` | one CRC-checked packet of 12 points |
| `ld19_read_scan(h, points, capacity, &count, ms)` | one revolution into the caller's array |
| `ld19_del(h)` | ESP-IDF: deletes the UART driver too |
| `ld19_lidar(h)` | the handle as a `struct lidar_dev` |
| `lidar_read_packet(l, &pkt, ms)` | a `struct lidar_packet`, stamped with `lidar_time_us()` |
| `lidar_read_scan(l, &scan, ms)` | a `struct lidar_scan`: points, `truncated`, speed, first and last packet's time |
| `lidar_filter_near(&scan)` | LDROBOT's near filter, opt-in |
| `lidar_log_scan(l, &scan)` | one info line |

`cfg.model` is `LD19_MODEL_LD19` (0) or `LD19_MODEL_LD06`; it names the
device "ld19" or "ld06" and changes nothing else -- LDROBOT's SDK reads both
the same way. `cfg.span_check`, off by default, turns on the SDK's packet
span check. On ESP-IDF the chip calls return `esp_err_t`; on the HAL, 0 or
a negative errno; every `lidar_*` call errno on both.

## Points

`angle_deg` is 0 to 360, 0 the front of the lidar, clockwise seen from
above (MAN 4, p.12). `distance_mm` **0 means no return**: glass, a mirror, a
dark or angled surface, or open space -- skip those points. `intensity` is
0-255; a white target within 6 m reads about 200 (MAN 3.2, p.10).

## Scans

A scan runs from one crossing of 0 degrees to the next. The crossing is
where an angle under 20 degrees follows one over 340, as in LDROBOT's SDK.
The packet it falls inside is split, and its tail opens the next scan, so
only the first call spends a turn finding a crossing. One deadline covers
the whole turn.

`truncated` (category view) means the scan is not one clean turn:

- points past `capacity` were dropped;
- the turn had more points than one turn at its speed can, more than
  4500 x 1.4 per rotor Hz;
- or no crossing came while the points passed twice a turn's worth. The scan
  ends there rather than growing without end.

LDROBOT's SDK drops the last two; here they are returned, flagged.
`ld19_read_scan()` has no flag; it drops points past capacity silently, as
esp-ld19-lidar did.

A packet taken with `ld19_read_packet()` or `lidar_read_packet()` between
two scans is missing from the next one.

## The near filter

`lidar_filter_near(&scan)` is LDROBOT's NearFilter for the LD06 and LD19,
ported from its SDK and off unless called. Points within 5 m are grouped by
angle (steps up to `speed / 4500 x 2` degrees) and distance (steps up to
3 %), the last group joined to the first across 360. A group is kept if:

- it has more than 15 points; or
- it has 3 or more points and their mean intensity is over 15; or
- it has fewer than 3 points and their mean intensity is 220 or more.

Any other group is noise: its points get distance 0 and intensity 0. The scan
comes back sorted by angle. Details in [chips](chips.md#the-near-filter).

## Waits

Every wait is a deadline on `lidar_time_us()`: a 5 ms timeout waits 5 ms at
a 100 Hz FreeRTOS tick too. esp-ld19-lidar's `pdMS_TO_TICKS()` made anything
under 10 ms no wait at all.

## The STM32 feed

With `cfg.fed = true`, `ld19_init()` returns at once; the app starts its DMA
or interrupt reception and passes each chunk to `ld19_rx_feed(h, data, len)`,
which is ISR-safe and never blocks. The driver reads from a 1 KiB ring and
never calls the HAL. With `cfg.fed = false`, the driver reads with blocking
`HAL_UART_Receive()`, which loses bytes at this rate (see
[hardware](hardware.md#stm32-the-uart-and-dma)). In that mode init waits for
the first packet, like the ESP-IDF one.

## Errors

| errno | ESP-IDF | Means |
|---|---|---|
| `-EINVAL` | `ESP_ERR_INVALID_ARG` | NULL, zero capacity, an unknown model |
| `-ENXIO` | `ESP_ERR_NOT_FOUND` | init: nothing arrived in a second |
| `-ENODEV` | `ESP_ERR_NOT_FOUND` | init: bytes, but no valid packet -- the baud rate |
| `-ETIMEDOUT` | `ESP_ERR_TIMEOUT` | nothing in time |
| `-ENOMEM` | `ESP_ERR_NO_MEM` | out of heap |
| others | as IDF gave them | the UART driver's own errors, from install or delete |

## Changes from esp-ld19-lidar

- `ld19.h` keeps every declaration; under ESP-IDF its token stream gains
  additions only. `cfg.model` and `cfg.span_check` are new last fields,
  0 the LD19 and the span check off.
- **Waits are deadlines on the clock** (see above).
- **The scan rule is the SDK's**: an angle under 20 after one over 340, not
  any step back, so a backward jitter cannot cut a scan in two.
- **The SDK's span check is available, off by default**: `cfg.span_check`
  drops a packet whose angles span more than `speed x 12 / 4500 x 1.5`
  degrees. See [chips](chips.md#the-span-check) for why it is off: the
  manual's own example fails it.
- **A runaway scan ends**: with no crossing in twice a turn's points, the
  scan returns there instead of running to the timeout.
- **The PWM pin is to be grounded**, not left open (both manuals).
- The packet's angle step, the CRC and the field layout are unchanged, and
  now cited.
