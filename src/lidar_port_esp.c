/*
 * lidar_priv.h on ESP-IDF. The CMakeLists lists this file; stm-forge compiles
 * every .c in src/, so outside ESP-IDF it is empty.
 */
#ifdef ESP_PLATFORM

#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lidar.h"
#include "lidar_priv.h"

/* 400 kHz when a caller says 0: fast mode. */
#define DEFAULT_SCL_HZ		400000

int64_t lidar_time_us(void)
{
	return esp_timer_get_time();
}

/*
 * Ticks for at least ms milliseconds. Two things make a plain
 * vTaskDelay(pdMS_TO_TICKS(ms)) wait less than asked:
 *
 *   - pdMS_TO_TICKS() truncates. At IDF's default 100 Hz tick, 2 ms is 0
 *     ticks and 15 ms is 1, so round up instead.
 *   - A wait of n ticks counts tick interrupts, and the first one may be
 *     about to fire, so n ticks can be as little as n - 1 full periods.
 *     Add one.
 *
 * So 2 ms at 100 Hz asks for 2 ticks and waits 10-20 ms: long, never short.
 */
static TickType_t lidar_ticks(uint32_t ms)
{
	return (TickType_t)(((uint64_t)ms * configTICK_RATE_HZ + 999) / 1000) +
	       1;
}

void lidar_sleep_ms(uint32_t ms)
{
	vTaskDelay(lidar_ticks(ms));
}

/* A yield between polls, not a timed wait. */
void lidar_yield(void)
{
	vTaskDelay(1);
}

/* --------------------------------------------------------------- I2C -- */

int lidar_i2c_probe(lidar_i2c_bus handle, uint8_t address, uint32_t timeout_ms)
{
	return lidar_errno(i2c_master_probe(handle, address, (int)timeout_ms));
}

static int lidar_i2c_transfer(struct lidar_bus *bus, uint8_t cmd,
			      const uint8_t *tx, size_t tx_len, uint8_t *rx,
			      size_t rx_len)
{
	uint8_t out[1 + LIDAR_BUS_MAX_LEN];

	if ((tx_len && rx_len) || tx_len > LIDAR_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (rx_len)
		return lidar_errno(i2c_master_transmit_receive(
			bus->i2c, out, 1, rx, rx_len, (int)bus->timeout_ms));
	if (tx_len)
		memcpy(out + 1, tx, tx_len);
	return lidar_errno(i2c_master_transmit(bus->i2c, out, 1 + tx_len,
					       (int)bus->timeout_ms));
}

static void lidar_i2c_detach(struct lidar_bus *bus)
{
	i2c_master_bus_rm_device(bus->i2c);
}

static const struct lidar_bus_ops lidar_i2c_ops = {
	.kind = LIDAR_BUS_I2C,
	.transfer = lidar_i2c_transfer,
	.detach = lidar_i2c_detach,
};

int lidar_i2c_attach(struct lidar_bus *bus, lidar_i2c_bus handle,
		     uint8_t address, uint32_t scl_hz, uint32_t timeout_ms)
{
	i2c_device_config_t devcfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = address,
		.scl_speed_hz = scl_hz ? scl_hz : DEFAULT_SCL_HZ,
	};

	int ret;

	ret = lidar_errno(i2c_master_bus_add_device(handle, &devcfg,
						    &bus->i2c));
	if (ret)
		return ret;
	/* Set only now: a bus with ops is one that has something to detach. */
	bus->ops = &lidar_i2c_ops;
	bus->timeout_ms = timeout_ms;
	return 0;
}

/* --------------------------------------------------------------- SPI -- */

/*
 * One full-duplex transaction, so CS stays low across cmd, tx and rx: clock
 * out cmd and tx followed by zeros, and keep what came back after them. The
 * buffers live on the stack, word-aligned: IDF bounces non-DMA-capable or
 * unaligned buffers through a copy of its own.
 */
static int lidar_spi_transfer(struct lidar_bus *bus, uint8_t cmd,
			      const uint8_t *tx, size_t tx_len, uint8_t *rx,
			      size_t rx_len)
{
	uint8_t out[1 + LIDAR_BUS_MAX_LEN] __attribute__((aligned(4))) = { 0 };
	uint8_t in[1 + LIDAR_BUS_MAX_LEN] __attribute__((aligned(4)));
	spi_transaction_t t = { 0 };
	int ret;

	if (tx_len + rx_len > LIDAR_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	t.length = (1 + tx_len + rx_len) * 8;
	t.tx_buffer = out;
	t.rx_buffer = in;
	ret = lidar_errno(spi_device_polling_transmit(bus->spi, &t));
	if (!ret && rx_len)
		memcpy(rx, in + 1 + tx_len, rx_len);
	return ret;
}

static void lidar_spi_detach(struct lidar_bus *bus)
{
	spi_bus_remove_device(bus->spi);
}

static const struct lidar_bus_ops lidar_spi_ops = {
	.kind = LIDAR_BUS_SPI,
	.transfer = lidar_spi_transfer,
	.detach = lidar_spi_detach,
};

int lidar_spi_attach(struct lidar_bus *bus, spi_host_device_t host, int cs_pin,
		     uint32_t clock_hz, uint8_t mode)
{
	spi_device_interface_config_t devcfg = {
		.mode = mode,
		.clock_speed_hz = (int)clock_hz,
		.spics_io_num = cs_pin,
		.queue_size = 1,
	};

	int ret;

	ret = lidar_errno(spi_bus_add_device(host, &devcfg, &bus->spi));
	if (ret)
		return ret;
	bus->ops = &lidar_spi_ops;
	bus->timeout_ms = 0;	/* spi_master transfers take none */
	return 0;
}

/* -------------------------------------------------------------- UART -- */

/*
 * Drop whatever is waiting -- a late reply to an earlier command would read
 * as the start of this one -- then send, then wait for the whole reply.
 * uart_read_bytes() returns early only with everything; a short count means
 * the timeout ran out.
 */
static int lidar_uart_transfer(struct lidar_bus *bus, uint8_t cmd,
			       const uint8_t *tx, size_t tx_len, uint8_t *rx,
			       size_t rx_len)
{
	uint8_t out[1 + LIDAR_BUS_MAX_LEN];
	int n, ret;

	if (tx_len > LIDAR_BUS_MAX_LEN || rx_len > INT32_MAX)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	ret = lidar_errno(uart_flush_input(bus->uart));
	if (ret)
		return ret;
	n = uart_write_bytes(bus->uart, out, 1 + tx_len);
	if (n != (int)(1 + tx_len))
		return -EIO;
	if (!rx_len)
		return 0;
	n = uart_read_bytes(bus->uart, rx, (uint32_t)rx_len,
			    lidar_ticks(bus->timeout_ms));
	if (n < 0)
		return -EIO;
	return (size_t)n < rx_len ? -ETIMEDOUT : 0;
}

/* Nothing sent: the part streams. A short count means the wait ran out. */
static int lidar_uart_receive(struct lidar_bus *bus, uint8_t *rx, size_t rx_len,
			      bool drop_stale)
{
	int n, ret;

	if (!rx_len || rx_len > INT32_MAX)
		return -EINVAL;
	if (drop_stale) {
		ret = lidar_errno(uart_flush_input(bus->uart));
		if (ret)
			return ret;
	}
	n = uart_read_bytes(bus->uart, rx, (uint32_t)rx_len,
			    lidar_ticks(bus->timeout_ms));
	if (n < 0)
		return -EIO;
	return (size_t)n < rx_len ? -ETIMEDOUT : 0;
}

/* The driver is the app's: nothing was added, nothing to remove. */
static void lidar_uart_detach(struct lidar_bus *bus)
{
	(void)bus;
}

static const struct lidar_bus_ops lidar_uart_ops = {
	.kind = LIDAR_BUS_UART,
	.transfer = lidar_uart_transfer,
	.receive = lidar_uart_receive,
	.detach = lidar_uart_detach,
};

int lidar_uart_attach(struct lidar_bus *bus, uart_port_t port,
		      uint32_t timeout_ms)
{
	if (!uart_is_driver_installed(port))
		return -EINVAL;
	bus->ops = &lidar_uart_ops;
	bus->timeout_ms = timeout_ms;
	bus->uart = port;
	return 0;
}

#endif /* ESP_PLATFORM */
