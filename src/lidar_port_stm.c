/*
 * lidar_priv.h on the STM32 HAL. stm-forge compiles every .c in src/, so
 * under ESP-IDF this file is empty; the ESP-IDF CMakeLists never lists it.
 */
#ifndef ESP_PLATFORM

#include <string.h>

#include "lidar.h"
#include "lidar_priv.h"

/*
 * The core is always built; each chip driver only under its own option. With
 * the library built and no chip picked, say so: this file is compiled only
 * when the library is, so this is the CMakeLists' "no chip" warning for every
 * build that never reads that file (stm-forge, a hand-made project).
 */
#if !(CONFIG_LIDAR_LD19)
#warning "lidar: no chip driver is selected; define CONFIG_LIDAR_<CHIP>=1 (stm-forge: LIDAR_<CHIP> = y)"
#endif

/* Milliseconds dressed as microseconds; see lidar.h for a real clock. */
__attribute__((weak)) int64_t lidar_time_us(void)
{
	return (int64_t)HAL_GetTick() * 1000;
}

/*
 * HAL_Delay() already adds one tick to what it is asked, because its first
 * tick may be about to fire, so ms is passed straight through: 10 ms waits
 * 10-11 ms at the HAL's default 1 kHz tick, and at a slower tick still never
 * less than 10. Long, never short -- as long as the app has not replaced the
 * weak HAL_Delay() with one that drops that tick. It busy-waits on SysTick,
 * so never call it from an interrupt at or above SysTick's priority.
 */
void lidar_sleep_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

/* There is no scheduler to yield to: wait a tick instead. */
void lidar_yield(void)
{
	lidar_sleep_ms(1);
}

#if defined(HAL_I2C_MODULE_ENABLED) || defined(HAL_SPI_MODULE_ENABLED) || \
    defined(HAL_UART_MODULE_ENABLED)
/* Nothing to undo: the handle is the app's, and nothing was added to it. */
static void lidar_hal_detach(struct lidar_bus *bus)
{
	(void)bus;
}
#endif

#ifdef HAL_I2C_MODULE_ENABLED
/*
 * HAL_I2C_IsDeviceReady() returns HAL_ERROR when no trial was ACKed, which is
 * -ENXIO. A busy or stuck bus keeps its own errno. On the F4, no pull-ups or
 * SDA held low sets the peripheral's BUSY flag, so the HAL gives up after
 * 25 ms with HAL_BUSY, -EAGAIN; HAL_TIMEOUT, -ETIMEDOUT, means a START never
 * completed.
 */
int lidar_i2c_probe(lidar_i2c_bus handle, uint8_t address, uint32_t timeout_ms)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(handle, (uint16_t)(address << 1), 1,
				       timeout_ms);
	return status == HAL_ERROR ? -ENXIO : lidar_hal_errno(status);
}

/* HAL_I2C_Mem_Read(): cmd, repeated start, burst. Mem_Write(): cmd, data. */
static int lidar_i2c_transfer(struct lidar_bus *bus, uint8_t cmd,
			      const uint8_t *tx, size_t tx_len, uint8_t *rx,
			      size_t rx_len)
{
	uint16_t address = (uint16_t)(bus->i2c.address << 1);

	if ((tx_len && rx_len) || tx_len > LIDAR_BUS_MAX_LEN ||
	    rx_len > UINT16_MAX)
		return -EINVAL;
	if (rx_len)
		return lidar_hal_errno(HAL_I2C_Mem_Read(bus->i2c.handle,
							address, cmd,
							I2C_MEMADD_SIZE_8BIT,
							rx, (uint16_t)rx_len,
							bus->timeout_ms));
	if (tx_len)
		return lidar_hal_errno(HAL_I2C_Mem_Write(bus->i2c.handle,
							 address, cmd,
							 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t *)tx,
							 (uint16_t)tx_len,
							 bus->timeout_ms));
	/* A bare command, for a part with no registers: its own START, STOP. */
	return lidar_hal_errno(HAL_I2C_Master_Transmit(bus->i2c.handle, address,
						       &cmd, 1,
						       bus->timeout_ms));
}

static const struct lidar_bus_ops lidar_i2c_ops = {
	.kind = LIDAR_BUS_I2C,
	.transfer = lidar_i2c_transfer,
	.detach = lidar_hal_detach,
};

int lidar_i2c_attach(struct lidar_bus *bus, lidar_i2c_bus handle,
		     uint8_t address, uint32_t scl_hz, uint32_t timeout_ms)
{
	(void)scl_hz;
	bus->ops = &lidar_i2c_ops;
	bus->timeout_ms = timeout_ms;
	bus->i2c.handle = handle;
	bus->i2c.address = address;
	return 0;
}
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
/*
 * One transaction under a software chip select: CS low, cmd and tx out --
 * full duplex with zeros after them when there is something to read -- CS
 * high. CS goes high even when the transfer failed; see lidar_bus_ops.
 */
static int lidar_spi_transfer(struct lidar_bus *bus, uint8_t cmd,
			      const uint8_t *tx, size_t tx_len, uint8_t *rx,
			      size_t rx_len)
{
	uint8_t out[1 + LIDAR_BUS_MAX_LEN] = { 0 }, in[1 + LIDAR_BUS_MAX_LEN];
	uint16_t n = (uint16_t)(1 + tx_len + rx_len);
	HAL_StatusTypeDef status;

	if (tx_len + rx_len > LIDAR_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	HAL_GPIO_WritePin(bus->spi.cs_port, bus->spi.cs_pin, GPIO_PIN_RESET);
	if (rx_len)
		status = HAL_SPI_TransmitReceive(bus->spi.handle, out, in, n,
						 bus->timeout_ms);
	else
		status = HAL_SPI_Transmit(bus->spi.handle, out, n,
					  bus->timeout_ms);
	HAL_GPIO_WritePin(bus->spi.cs_port, bus->spi.cs_pin, GPIO_PIN_SET);
	if (status == HAL_OK && rx_len)
		memcpy(rx, in + 1 + tx_len, rx_len);
	return lidar_hal_errno(status);
}

static const struct lidar_bus_ops lidar_spi_ops = {
	.kind = LIDAR_BUS_SPI,
	.transfer = lidar_spi_transfer,
	.detach = lidar_hal_detach,
};

int lidar_spi_attach(struct lidar_bus *bus, SPI_HandleTypeDef *handle,
		     GPIO_TypeDef *cs_port, uint16_t cs_pin,
		     uint32_t timeout_ms)
{
	bus->ops = &lidar_spi_ops;
	bus->timeout_ms = timeout_ms;
	bus->spi.handle = handle;
	bus->spi.cs_port = cs_port;
	bus->spi.cs_pin = cs_pin;
	/* Deselected first, so the first transfer starts on a falling edge. */
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	return 0;
}
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
/*
 * Blocking HAL_UART_Receive() hands back whatever already sits in the data
 * register, so a late byte from an earlier exchange would be read as the
 * start of this reply. Drop it first: clear an overrun, then read the data
 * register empty (on the F7 that macro is a flush request). Bounded by the
 * timeout, in case the line never goes quiet.
 */
static void lidar_uart_drop_input(struct lidar_bus *bus)
{
	UART_HandleTypeDef *handle = bus->uart;
	uint32_t start = HAL_GetTick();

	__HAL_UART_CLEAR_OREFLAG(handle);
	while (__HAL_UART_GET_FLAG(handle, UART_FLAG_RXNE) &&
	       (uint32_t)(HAL_GetTick() - start) <= bus->timeout_ms)
		__HAL_UART_FLUSH_DRREGISTER(handle);
}

static int lidar_uart_transfer(struct lidar_bus *bus, uint8_t cmd,
			       const uint8_t *tx, size_t tx_len, uint8_t *rx,
			       size_t rx_len)
{
	uint8_t out[1 + LIDAR_BUS_MAX_LEN];
	int ret;

	if (tx_len > LIDAR_BUS_MAX_LEN || rx_len > UINT16_MAX)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	lidar_uart_drop_input(bus);
	ret = lidar_hal_errno(HAL_UART_Transmit(bus->uart, out,
						(uint16_t)(1 + tx_len),
						bus->timeout_ms));
	if (ret || !rx_len)
		return ret;
	/* A reply that stops short runs into the timeout: HAL_TIMEOUT. */
	return lidar_hal_errno(HAL_UART_Receive(bus->uart, rx, (uint16_t)rx_len,
						bus->timeout_ms));
}

/*
 * Nothing sent: the part streams. A short read runs into HAL_TIMEOUT. The
 * blocking HAL_UART_Receive() takes the line only while it runs: bytes that
 * arrive between two calls overrun the one-byte data register and are lost,
 * so a stream reader resyncs on its own framing.
 */
static int lidar_uart_receive(struct lidar_bus *bus, uint8_t *rx, size_t rx_len,
			      bool drop_stale)
{
	if (!rx_len || rx_len > UINT16_MAX)
		return -EINVAL;
	if (drop_stale)
		lidar_uart_drop_input(bus);
	return lidar_hal_errno(HAL_UART_Receive(bus->uart, rx, (uint16_t)rx_len,
						bus->timeout_ms));
}

static const struct lidar_bus_ops lidar_uart_ops = {
	.kind = LIDAR_BUS_UART,
	.transfer = lidar_uart_transfer,
	.receive = lidar_uart_receive,
	.detach = lidar_hal_detach,
};

int lidar_uart_attach(struct lidar_bus *bus, UART_HandleTypeDef *handle,
		      uint32_t timeout_ms)
{
	bus->ops = &lidar_uart_ops;
	bus->timeout_ms = timeout_ms;
	bus->uart = handle;
	return 0;
}
#endif /* HAL_UART_MODULE_ENABLED */

#endif /* !ESP_PLATFORM */
