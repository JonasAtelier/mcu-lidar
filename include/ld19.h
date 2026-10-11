/*
 * ld19 — ESP-IDF driver for the LDROBOT LD19 360° laser scanner.
 *
 * A little turret that spins at about 10 Hz and measures the distance to
 * whatever it sees, ~4500 times a second, out to 12 m. It is a direct
 * time-of-flight unit, so it does not lose accuracy with distance the way a
 * triangulation scanner does, and it does not care about ambient light. Sold
 * by Waveshare and others as the LD19 or LD06; both speak the same protocol.
 *
 * It is a talker, not a listener: power it up and it streams packets forever.
 * There is no command set, nothing to configure, and no reason to wire a TX
 * pin to it. This driver therefore only ever reads.
 *
 * Provenance: the packet layout, the CRC and the angle interpolation come
 * from the LD19 Development Manual, section "Data packet format". LDROBOT
 * does not publish it as an open datasheet and it is NOT in docs/ — the
 * constants in src/ld19_parse.h cite it by section title only, and the README
 * says so rather than pretending they were derived from silicon.
 */
/*
 * In mcu-lidar: the LD06 too, through cfg.model -- LDROBOT's SDK parses
 * both with the same code; the same calls on the STM32 HAL, where they
 * return 0 or a negative errno and the app hands over its UART, ideally
 * through ld19_rx_feed() from a DMA or interrupt callback; and ld19_lidar(),
 * the handle as a struct lidar_dev. The packet layout and CRC now cite the
 * LD19 Development Manual (in docs/) and the SDK by page and line, in
 * src/ld19/ld19_proto.h. Waits are on the clock, and a turn ends only where
 * an angle under 20 degrees follows one over 340, as in the SDK. cfg.span_check
 * turns on the SDK's packet span check.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#else
#include CMSIS_device_header
#endif

#include "lidar.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LD19_POINTS_PER_PACKET  12
#define LD19_SCAN_MAX_POINTS    600   /* about 450 per turn at 10 Hz; headroom
                                       * for a slow rotor */

typedef struct ld19_t *ld19_handle_t;

/* Which scanner; they speak one protocol. 0 is the LD19. */
typedef enum {
    LD19_MODEL_LD19 = 0,
    LD19_MODEL_LD06,
} ld19_model_t;

#ifdef ESP_PLATFORM
typedef struct {
    uart_port_t uart;      /* the driver configures this port itself: 230400 8N1 */
    gpio_num_t  rx_gpio;   /* the lidar's TX wire. The LD19 only ever talks, so
                            * there is no TX pin */

    ld19_model_t model;    /* 0 is the LD19; LD19_MODEL_LD06 names it "ld06" */
    /* Drop a packet whose angles span more than its speed allows, as
     * LDROBOT's SDK does (lipkg.cpp:151-152). Off by default: the manual's
     * own example packet fails it (docs/chips.md). */
    bool         span_check;
} ld19_config_t;
#else
typedef struct {
    /* HAL_UART_Init()ed by the app at 230400 8N1; only its RX is used. */
    UART_HandleTypeDef *uart;
    /* true: the app hands every received byte to ld19_rx_feed() from its
     * DMA or interrupt callback, and the driver never calls the HAL. false:
     * the driver reads with blocking HAL_UART_Receive(), which listens only
     * while it runs, so bytes that arrive between two reads are lost --
     * packets go missing every turn at 230400 bps. */
    bool fed;
    ld19_model_t model;
    bool span_check;    /* as on ESP-IDF; off by default */
} ld19_config_t;
#endif

typedef struct {
    float    angle_deg;     /* 0..360, clockwise seen from above, as the lidar
                             * reports it */
    uint16_t distance_mm;   /* 0 means no return */
    uint8_t  intensity;     /* 0..255; higher is a firmer reading */
} ld19_point_t;

typedef struct {
    ld19_point_t points[LD19_POINTS_PER_PACKET];
    float        speed_dps;      /* rotor speed, degrees per second; about 3600
                                  * at 10 Hz */
    uint16_t     timestamp_ms;   /* the lidar's own clock, wraps at 30000 */
} ld19_packet_t;

#ifdef ESP_PLATFORM
/* Open the UART and wait for the first valid packet, so a lidar that is not
 * spinning or not wired fails here with ESP_ERR_NOT_FOUND. */
esp_err_t ld19_init(const ld19_config_t *cfg, ld19_handle_t *out);

/* One CRC-checked packet of 12 points. ESP_ERR_TIMEOUT if none arrives. */
/* (In mcu-lidar: taken between two ld19_read_scan() calls, its points are
 * missing from the next scan. A wrap is an angle under 20 degrees after one
 * over 340.) */
esp_err_t ld19_read_packet(ld19_handle_t dev, ld19_packet_t *out,
                           uint32_t timeout_ms);

/* One full turn: every point from one angle wrap to the next. count is how
 * many were written; capacity is how many the caller's array holds (use
 * LD19_SCAN_MAX_POINTS). Points past capacity are dropped, not an error. */
esp_err_t ld19_read_scan(ld19_handle_t dev, ld19_point_t *points, size_t capacity,
                         size_t *count, uint32_t timeout_ms);

esp_err_t ld19_del(ld19_handle_t dev);
#else
/* With cfg->fed false, waits for the first valid packet: -ENXIO if nothing
 * arrives in a second, -ENODEV if bytes do but no packet. With it true,
 * nothing can arrive before the app feeds the handle, so init returns at
 * once and the first read tells. */
int ld19_init(const ld19_config_t *cfg, ld19_handle_t *out);
int ld19_read_packet(ld19_handle_t dev, ld19_packet_t *out,
                     uint32_t timeout_ms);
int ld19_read_scan(ld19_handle_t dev, ld19_point_t *points, size_t capacity,
                   size_t *count, uint32_t timeout_ms);
int ld19_del(ld19_handle_t dev);

/* The bytes the app's UART received, from its DMA or interrupt callback
 * (HAL_UARTEx_RxEventCallback(), HAL_UART_RxCpltCallback()); ISR-safe, never
 * blocks. Returns how many were taken: fewer when the 1 KiB ring is full,
 * about 58 ms of the stream unread. 0 for a handle not made with fed. */
size_t ld19_rx_feed(ld19_handle_t dev, const uint8_t *data, size_t len);
#endif

/* The handle as a struct lidar_dev; NULL for a NULL handle. Valid until
 * ld19_del(). */
struct lidar_dev *ld19_lidar(ld19_handle_t dev);

#ifdef __cplusplus
}
#endif
