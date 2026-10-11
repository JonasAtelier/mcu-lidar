/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "lidar_priv.h"

#if CONFIG_LIDAR_LD19

#include <stdlib.h>
#include <string.h>

#include "ld19.h"
#include "ld19_proto.h"
#include "lidar_log.h"

/*
 * The stream is 4500 points / 12 per packet x 47 bytes, about 17.6 KB/s, of
 * the 23 KB/s 230400 8N1 can carry; 4096 bytes of UART buffer is about
 * 0.23 s of slack. (esp-ld19-lidar said ~9 KB/s and half a second.)
 */
#define RX_BUF_LEN      4096
#define RING_LEN        1024    /* STM, fed: about 58 ms of the stream */
#define INIT_TIMEOUT_MS 1000    /* a packet every ~2.7 ms when it spins */

#define TRY(expr) do { int _e = (expr); if (_e) return _e; } while (0)

struct ld19_t {
    struct lidar_dev  lidar;    /* the category view; priv points back */
    struct lidar_bus  bus;      /* blocking reads; unused when fed */
    bool              fed;      /* bytes come through ld19_rx_feed() */
    bool              span_check;  /* cfg.span_check */
    struct lidar_ring ring;
    uint8_t           ring_buf[RING_LEN];
    uint8_t           buf[LD19_PACKET_LEN];  /* bytes held over between reads */
    size_t            have;                  /* how many of them are valid */
    size_t            received;              /* bytes that ever arrived */
#ifdef ESP_PLATFORM
    uart_port_t       uart;
#endif
};

/* ------------------------------------------------------------ bytes ---- */

/*
 * Fill buf up to a whole packet, or fail once timeout_ms has passed since
 * start. Fed: from the ring, keeping what came even when the rest does not.
 * Read: one bus receive for what is missing, which on a short count leaves
 * nothing usable, so those bytes are gone; the next packet resyncs.
 */
static int fill(struct ld19_t *d, int64_t start, uint32_t timeout_ms)
{
    for (;;) {
        uint32_t elapsed = lidar_elapsed_ms(start);
        size_t need = LD19_PACKET_LEN - d->have;
        size_t got;

        if (!need)
            return 0;
        if (d->fed) {
            got = lidar_ring_get(&d->ring, d->buf + d->have, need);
            d->have += got;
            d->received += got;
            if (got == need)
                return 0;
            if (elapsed >= timeout_ms)
                return -ETIMEDOUT;
            lidar_yield();
            continue;
        }
        if (elapsed >= timeout_ms)
            return -ETIMEDOUT;
        d->bus.timeout_ms = timeout_ms - elapsed;
        TRY(lidar_bus_receive(&d->bus, d->buf + d->have, need, false));
        d->have += need;
        d->received += need;
    }
}

/*
 * The next valid packet. The lidar talks continuously and never waits to be
 * asked, so a read almost always starts in the middle of a packet: drop one
 * byte and try the next alignment until header, VerLen and CRC agree. With
 * cfg.span_check, a packet that checks out but spans more than its speed
 * allows is dropped whole, as LDROBOT's SDK drops it.
 */
static int read_packet(struct ld19_t *d, struct lidar_packet *pkt,
                       uint32_t timeout_ms)
{
    int64_t start = lidar_time_us();
    int ret;

    for (;;) {
        TRY(fill(d, start, timeout_ms));
        ret = ld19_parse(d->buf, pkt, d->span_check);
        if (ret == 0 || ret == -ERANGE) {
            d->have = 0;
            if (ret == 0)
                return 0;
            lidar_dbg(&d->lidar, "packet spans more than its speed allows");
            continue;
        }
        memmove(d->buf, d->buf + 1, --d->have);
    }
}

static int op_read_packet(struct lidar_dev *lidar, struct lidar_packet *pkt,
                          uint32_t timeout_ms)
{
    return read_packet(lidar->priv, pkt, timeout_ms);
}

static const struct lidar_ops ld19_ops = {
    .read_packet = op_read_packet,
};

/* ------------------------------------------------------------- init ---- */

static int setup(struct ld19_t *d, ld19_model_t model, bool span_check)
{
    if (model != LD19_MODEL_LD19 && model != LD19_MODEL_LD06)
        return -EINVAL;
    d->span_check = span_check;
    d->lidar.name = model == LD19_MODEL_LD06 ? "ld06" : "ld19";
    d->lidar.ops = &ld19_ops;
    d->lidar.priv = d;
    d->lidar.rate_hz = LD19_RATE_HZ;
    return lidar_ring_init(&d->ring, d->ring_buf, RING_LEN);
}

/* Silence for a second means there is nothing on the other end worth
 * handing back a working handle for. */
static int first_packet(struct ld19_t *d)
{
    struct lidar_packet pkt;
    int ret = read_packet(d, &pkt, INIT_TIMEOUT_MS);

    if (!ret)
        return 0;
    if (ret == -ETIMEDOUT && !d->received) {
        lidar_err(&d->lidar, "nothing in %d ms: check the 5 V supply, the TX "
                  "wire and that the rotor is turning", INIT_TIMEOUT_MS);
        return -ENXIO;
    }
    if (ret == -ETIMEDOUT) {
        lidar_err(&d->lidar, "bytes but no valid packet in %d ms: check the "
                  "baud rate, 230400", INIT_TIMEOUT_MS);
        return -ENODEV;
    }
    return ret;
}

#ifdef ESP_PLATFORM
static int init(const ld19_config_t *cfg, ld19_handle_t *out)
{
    uart_config_t uart_cfg = {
        .baud_rate  = LD19_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    struct ld19_t *d;
    int err;

    if (cfg == NULL || out == NULL)
        return -EINVAL;

    d = calloc(1, sizeof *d);
    if (d == NULL)
        return -ENOMEM;
    d->uart = cfg->uart;
    err = setup(d, cfg->model, cfg->span_check);
    if (err) {
        free(d);
        return err;
    }

    /* No transmit buffer: this driver has nothing to say to the lidar. */
    err = lidar_errno(uart_driver_install(d->uart, RX_BUF_LEN, 0, 0, NULL, 0));
    if (err) {
        free(d);
        return err;
    }

    err = lidar_errno(uart_param_config(d->uart, &uart_cfg));
    /* Only the lidar's TX wire goes anywhere, so leave the other pins alone. */
    if (!err)
        err = lidar_errno(uart_set_pin(d->uart, UART_PIN_NO_CHANGE,
                                       cfg->rx_gpio, UART_PIN_NO_CHANGE,
                                       UART_PIN_NO_CHANGE));
    if (!err)
        err = lidar_uart_attach(&d->bus, d->uart, INIT_TIMEOUT_MS);
    if (!err)
        err = first_packet(d);
    if (err) {
        uart_driver_delete(d->uart);
        free(d);
        return err;
    }
    *out = d;
    return 0;
}
#else
static int init(const ld19_config_t *cfg, ld19_handle_t *out)
{
    struct ld19_t *d;
    int err;

    if (cfg == NULL || out == NULL || cfg->uart == NULL)
        return -EINVAL;

    d = calloc(1, sizeof *d);
    if (d == NULL)
        return -ENOMEM;
    d->fed = cfg->fed;
    err = setup(d, cfg->model, cfg->span_check);
    if (!err && !d->fed) {
        err = lidar_uart_attach(&d->bus, cfg->uart, INIT_TIMEOUT_MS);
        if (!err)
            err = first_packet(d);
    }
    if (err) {
        free(d);
        return err;
    }
    *out = d;
    return 0;
}
#endif

/* ---------------------------------------------------- the handle API -- */

static void to_point(ld19_point_t *out, const struct lidar_point *p)
{
    out->angle_deg = p->angle_deg;
    out->distance_mm = p->distance_mm;
    out->intensity = p->intensity;
}

static void put_point(void *ctx, size_t index, const struct lidar_point *p)
{
    to_point(&((ld19_point_t *)ctx)[index], p);
}

#ifdef ESP_PLATFORM
esp_err_t ld19_init(const ld19_config_t *cfg, ld19_handle_t *out)
{
    return LIDAR_HANDLE_RET(init(cfg, out));
}
#else
int ld19_init(const ld19_config_t *cfg, ld19_handle_t *out)
{
    return init(cfg, out);
}
#endif

lidar_handle_ret ld19_read_packet(ld19_handle_t dev, ld19_packet_t *out,
                                  uint32_t timeout_ms)
{
    struct lidar_packet pkt;
    int err;

    if (dev == NULL || out == NULL)
        return LIDAR_HANDLE_RET(-EINVAL);
    err = lidar_read_packet(&dev->lidar, &pkt, timeout_ms);
    if (!err) {
        for (int i = 0; i < LD19_POINTS; i++)
            to_point(&out->points[i], &pkt.points[i]);
        out->speed_dps = pkt.speed_dps;
        out->timestamp_ms = pkt.sensor_ms;
    }
    return LIDAR_HANDLE_RET(err);
}

lidar_handle_ret ld19_read_scan(ld19_handle_t dev, ld19_point_t *points,
                                size_t capacity, size_t *count,
                                uint32_t timeout_ms)
{
    struct lidar_turn turn;
    int err;

    if (dev == NULL || points == NULL || count == NULL || capacity == 0)
        return LIDAR_HANDLE_RET(-EINVAL);
    *count = 0;
    err = lidar_collect_turn(&dev->lidar, capacity, put_point, points, &turn,
                             timeout_ms);
    if (!err)
        *count = turn.count;
    return LIDAR_HANDLE_RET(err);
}

lidar_handle_ret ld19_del(ld19_handle_t dev)
{
    int err = 0;

    if (dev == NULL)
        return LIDAR_HANDLE_RET(-EINVAL);
    if (dev->bus.ops)
        lidar_bus_detach(&dev->bus);
#ifdef ESP_PLATFORM
    err = lidar_errno(uart_driver_delete(dev->uart));
#endif
    free(dev);
    return LIDAR_HANDLE_RET(err);
}

#ifndef ESP_PLATFORM
size_t ld19_rx_feed(ld19_handle_t dev, const uint8_t *data, size_t len)
{
    if (dev == NULL || data == NULL || !dev->fed)
        return 0;
    return lidar_ring_put(&dev->ring, data, len);
}
#endif

struct lidar_dev *ld19_lidar(ld19_handle_t dev)
{
    return dev ? &dev->lidar : NULL;
}

#endif /* CONFIG_LIDAR_LD19 */
