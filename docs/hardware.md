# Hardware

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root. "MAN" is
LDROBOT's LD19 Development Manual V2.3, `docs/ld19_v2.3.pdf` (V2.5,
`docs/ld19_v2.5.pdf`, agrees), by PDF page; both are gitignored. No LD06
manual could be had; what is said of the LD06 comes from LDROBOT's SDK
(see [chip reference](chips.md#ld06-and-ld19)).

## LD19

ZH1.5T-4P, 1.5 mm pitch (MAN 2, p.5):

| Pin | Signal | Goes to | Note |
|---|---|---|---|
| 1 | Tx | the MCU's UART RX | 0-3.5 V, 3.3 V typical: straight to a 3.3 V GPIO |
| 2 | PWM | **ground**, or the MCU's PWM output | motor control input, 0-3.3 V |
| 3 | GND | ground | |
| 4 | P5V | 4.5-5.5 V, 5 V typical | not a 3.3 V rail |

**The PWM pin must be grounded when it is not driven** (MAN p.6; V2.5 p.4).
esp-ld19-lidar's README said to leave it unconnected; that was wrong.

The link is one way, the lidar talking: 230400 bps, 8 data bits, 1 stop bit,
no parity, no flow control (MAN p.6). Nothing is sent to it, and no TX wire
is needed.

**Supply.** The manual gives the voltage, not the current. esp-ld19-lidar's
README says "up to about 300 mA while the rotor spins up"; a brownout at
power-up looks exactly like a dead lidar (`-ENXIO`, `ESP_ERR_NOT_FOUND`).

### Speed control

The LD19 runs its own motor: grounded PWM pin, internal control, 10 ± 0.1 Hz
within 3 s of power-up (MAN 1, p.3; 2, p.5). External control is the app's,
if it wants it (MAN 2, p.5-6):

- a square wave on PWM at 20-50 kHz, 30 kHz recommended;
- a duty cycle strictly inside 45-55 %, for at least 100 ms, switches the
  lidar to external control, and from then the duty cycle sets the speed,
  start and stop;
- it stays under external control until power is cut;
- the same duty cycle turns different units at different speeds: close the
  loop on the speed in every packet (`speed_dps`; V2.5 p.4).

This library does not drive the pin.

## LD06

LDROBOT's SDK reads it with the LD19's code at the LD19's baud rate (its
launch files, `launch/ld06.launch:6` and `launch/ld19.launch:6`, both
230400). Its pinout and any speed control input are not in any source this
library has: check your unit's own documentation, and ground a PWM pin if
it has one, as for the LD19.

## STM32: the UART and DMA

230400 bps carries 23 KB/s, and the lidar uses 17.6 of it without a pause:
4500 points a second (MAN 1, p.3) in packets of 12, each 47 bytes. A byte
takes 43 µs. A blocking `HAL_UART_Receive()` only listens while it runs, so
whatever the CPU does between two receives loses bytes; the driver resyncs,
but packets go missing every turn.

So the recommended setup hands the stream over from DMA:

1. The UART at 230400 8N1, RX on a DMA stream in circular mode.
2. `ld19_config_t cfg = { .uart = &huart2, .fed = true };`
3. `HAL_UARTEx_ReceiveToIdle_DMA()` after `ld19_init()`, and
   `HAL_UARTEx_RxEventCallback()` passing every new chunk to
   `ld19_rx_feed()` -- see `examples/stm/ld19/main.c`.

The driver keeps a 1 KiB ring, about 58 ms of the stream: read at least that
often. `ld19_rx_feed()` returns fewer bytes than it was given when the ring
is full.
