# Chip reference

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root. Where
every value comes from, and what was checked.

## Sources

| Short | Document |
|---|---|
| MAN | LDROBOT "LD19 Development Manual" V2.3 (via Elecrow), `docs/ld19_v2.3.pdf`, 30 pages; PDF page numbers |
| MAN25 | the same, V2.5 (via DFRobot, Chinese), `docs/ld19_v2.5.pdf`, 27 pages; cross-check |
| SDK | LDROBOT's ldlidar_stl_ros, commit 964afad8, MIT; paths under `ldlidar_driver/` unless named |

No LD06 manual could be fetched (the datasheet link returned HTML). The SDK
covers both models in one parser.

## The packet

MAN 3.1-3.2, p.7-8 and p.10; MAN25 p.5-6 and p.8; SDK
`include/dataprocess/lipkg.h:37-57`. All three agree on every byte.

| Bytes | Field | Value |
|---|---|---|
| 0 | header | 0x54 |
| 1 | VerLen | 0x2C: upper 3 bits the type, 1; lower 5 the points, 12 |
| 2-3 | speed | deg/s |
| 4-5 | start angle | 0.01 deg |
| 6-41 | 12 x distance (mm, 2 bytes), intensity (1 byte) | |
| 42-43 | end angle | 0.01 deg |
| 44-45 | timestamp | ms, 0-29999, then over again |
| 46 | CRC | of bytes 0-45 |

Two-byte fields are low byte first. The link is 230400 8N1 (MAN 2, p.6).
The rate is 4500 measurements a second (MAN 1, p.3; SDK
`src/dataprocess/lipkg.cpp:81-84`).

**The CRC.** MAN p.8-9, MAN25 p.6-7 and SDK `lipkg.cpp:26-48` print the
same 256-entry table, checked entry by entry by script. It is CRC-8 with
polynomial 0x4D, initial 0, MSB first, no reflection and no final xor,
which is how `ld19_crc8()` computes it; `test_ld19_proto.c` checks all 256
entries against the manual's. The manual's example packet (3.3, p.11)
carries CRC 0x50, and so does the computation.

**Angles.** The first point is at the start angle, the last at the end
angle, the rest evenly between (MAN 3.2, p.10). The manual's formula,
`(end - start) / (len - 1)`, has no wrap and no types. The SDK's does, and
is followed (`lipkg.cpp:165-166`): the step is `((end - start) mod 36000) /
11` in whole hundredths, truncated. esp-ld19-lidar did the same.

**Manual errata, not conflicts.** The example's field table (3.3, p.11)
does not match its own hex:

- point 2's distance is given as "00DCH = 200mm"; 0x00DC is 220;
- point 12 is listed as B0 00 EA, but the packet has C0 00 E5;
- V2.3 also gives point 2's intensity as "00B0H = 176mm"; V2.5 fixes that
  one to 226.

The tests use the hex.

## LD06 and LD19

In the SDK the two differ in nothing that reaches the wire:

| What | LD06 | LD19 | Where |
|---|---|---|---|
| type | `LD_06` | `LD_19` | `include/core/ldlidar_datatype.h:34-41` |
| parser, CRC | the same code, no model branch | | `lipkg.cpp:98-186` |
| measurement rate | 4500 | 4500 | `lipkg.cpp:81-84` |
| filter | NEAR_FILTER, 15, 220, 4500 | the same | `src/filter/tofbf.cpp:35-40` |
| baud | 230400 | 230400 | `launch/ld06.launch:6`, `launch/ld19.launch:6` |

So one driver reads both; `cfg.model` only names the device.

## The span check

The SDK takes a packet only if its angles span no more than its speed
allows (`lipkg.cpp:151-152`). In whole degrees, `end / 100 - start / 100`
(mod 360) must be at most `speed x 12 / 4500 x 1.5`: half as much again as
twelve measurements turn at that speed.

Here the check is **opt-in, `cfg.span_check`, off by default**. With it on,
the driver does the same in integers, exactly, and a packet that fails is
dropped whole, as the SDK drops it. The angles are taken mod 36000 first,
which the SDK does not do; that only matters for angles no lidar sends.

**Why off: the manual's own example packet fails it.**

- The packet: MAN 3.3, p.11; MAN25 p.9; 2152 deg/s, 324.27 to 334.70
  degrees, CRC 0x50, valid.
- The check: it spans 10 whole degrees, and the speed allows 8.6.
- Physically, at 2152 deg/s twelve measurements at 4500 a second span
  5.3 degrees; the example spans twice that.
- Either the example was made up, or the rate at that speed is not 4500. No
  source says which.

With the check on, a real lidar sending packets like that would lose every
one. At the nominal 10 Hz (3600 deg/s) the limit is 14.4 degrees and a
packet spans about 9, so it passes.

## The scan rule

The SDK ends a turn where an angle under 20 degrees follows one over 340
(`lipkg.cpp:196`). It drops a turn whose points times the rotor's Hz exceed
4500 x 1.4 (`:197`), and drops what it has once that passes 4500 x 2 with no
crossing (`:235`).

Here the crossing rule is the same, and the other two return the scan with
`truncated` set instead of dropping it. esp-ld19-lidar took any backward
step as the crossing.

The SDK also discards the first complete turn it assembles (`:218-219`).
This library does not: its first scan is already a whole turn, starting at
the first crossing.

## The near filter

`src/lidar_filter.c` ports `Tofbf::NearFilter()` (`src/filter/tofbf.cpp:83-182`)
with the LD06/LD19 settings (`:35-40`). The SDK applies it by default
(`lipkg.cpp:65, 208-210`); here it is opt-in, as esp-ld19-lidar had no
filter.

| Rule | SDK | Value |
|---|---|---|
| only points closer than | `:90` | 5000 mm |
| group break: angle step over | `:99, :108` | `(int)speed / 4500 x 2` deg |
| group break: distance step over | `:109` | 3 % of the previous point |
| join the last group to the first | `:124-130` | the same two limits, strict |
| kept, any intensity | `:135` | more than 15 points |
| zeroed: under 3 points with int mean intensity under | `:141-147` | 220 |
| zeroed: otherwise, mean intensity not over | `:168` | 15 |

Where the port differs:

- **Order.** The SDK returns the far points first and then the groups, and
  its caller re-sorts by time (`lipkg.cpp:215`). Here the scan is sorted by
  angle in place and noise is zeroed where it lies. Within one turn starting
  at 0 degrees that is the same order.
- **One group.** With a single group the SDK joins it to itself
  (`vector::insert` of its own range, which the standard does not allow) and
  then erases it. Here a single group is never joined.
