/*
 * Host-side check of the ld19 driver on ESP-IDF: ../ld19_cases.h is the fake
 * scanner and the shared cases; this file feeds it from uart_read_bytes()
 * and checks what only ESP-IDF has -- the driver installing and configuring
 * its own UART, as esp-ld19-lidar did, and deleting it again, and the
 * esp_err_t codes its callers saw.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ld19.h"
#include "lidar.h"
#include "lidar_priv.h"

#define RET(err)	lidar_esp_err(err)
#define PORT		UART_NUM_1
#define RX_GPIO		16

static int64_t now_us(void)
{
	return stub_time_us;
}

static void pass_us(int64_t us)
{
	stub_time_us += us;
}

static ld19_config_t cfg_of(ld19_model_t model)
{
	return (ld19_config_t){ .uart = PORT, .rx_gpio = RX_GPIO,
				.model = model };
}

static int installed;
static int uart_live(void)
{
	return installed;
}

#include "../ld19_cases.h"

/* ------------------------------------------------- the IDF side of it --- */

static int rx_size, tx_size;
static uart_config_t ucfg;
static int pin_tx, pin_rx;
static esp_err_t install_err, delete_err;

esp_err_t uart_driver_install(uart_port_t port, int rx_buffer_size,
			      int tx_buffer_size, int queue_size, void *queue,
			      int intr_alloc_flags)
{
	(void)queue_size;
	(void)queue;
	(void)intr_alloc_flags;
	assert(port == PORT && !installed);
	if (install_err)
		return install_err;
	rx_size = rx_buffer_size;
	tx_size = tx_buffer_size;
	installed = 1;
	return ESP_OK;
}

esp_err_t uart_driver_delete(uart_port_t port)
{
	assert(port == PORT && installed);
	installed = 0;
	return delete_err;
}

esp_err_t uart_param_config(uart_port_t port, const uart_config_t *config)
{
	assert(port == PORT && installed);
	ucfg = *config;
	return ESP_OK;
}

esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts)
{
	assert(port == PORT && rts == UART_PIN_NO_CHANGE &&
	       cts == UART_PIN_NO_CHANGE);
	pin_tx = tx;
	pin_rx = rx;
	return ESP_OK;
}

bool uart_is_driver_installed(uart_port_t port)
{
	return port == PORT && installed;
}

/* A short count is the wait running out, as uart_read_bytes() has it. */
int uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
		    TickType_t ticks_to_wait)
{
	assert(port == PORT && installed && ticks_to_wait > 0);
	return (int)fake_take(buf, length,
			      (int64_t)ticks_to_wait * 1000000 / stub_tick_hz);
}

/* ----------------------------------------------------------- ESP cases --- */

/* 230400 8N1, RX only, 4096 bytes of RX buffer and none for TX. */
static void test_uart_setup(void)
{
	ld19_handle_t h;

	line_reset();
	send_packet(0, 500);
	h = open_ld19(LD19_MODEL_LD19);
	assert(ucfg.baud_rate == 230400 && ucfg.data_bits == UART_DATA_8_BITS);
	assert(ucfg.parity == UART_PARITY_DISABLE &&
	       ucfg.stop_bits == UART_STOP_BITS_1);
	assert(ucfg.flow_ctrl == UART_HW_FLOWCTRL_DISABLE);
	assert(pin_tx == UART_PIN_NO_CHANGE && pin_rx == RX_GPIO);
	assert(rx_size == 4096 && tx_size == 0);
	delete_err = ESP_ERR_INVALID_STATE;
	assert(ld19_del(h) == ESP_FAIL && !installed);
	delete_err = ESP_OK;

	/* The install's own error, nothing leaked. */
	install_err = ESP_ERR_INVALID_ARG;
	h = NULL;
	{
		ld19_config_t cfg = cfg_of(LD19_MODEL_LD19);

		assert(ld19_init(&cfg, &h) == ESP_ERR_INVALID_ARG);
	}
	install_err = ESP_OK;
	assert(!h && allocs == 0);
}

/* The codes esp-ld19-lidar's callers saw. */
static void test_esp_errors(void)
{
	ld19_config_t cfg = cfg_of(LD19_MODEL_LD19);
	ld19_handle_t h = NULL;
	ld19_packet_t p;

	line_reset();
	assert(ld19_init(&cfg, &h) == ESP_ERR_NOT_FOUND);
	line_reset();
	memset(line, 0x22, 200);
	line_len = 200;
	assert(ld19_init(&cfg, &h) == ESP_ERR_NOT_FOUND);
	line_reset();
	send_packet(0, 1);
	h = open_ld19(LD19_MODEL_LD19);
	assert(ld19_read_packet(h, &p, 20) == ESP_ERR_TIMEOUT);
	assert(ld19_read_packet(h, NULL, 20) == ESP_ERR_INVALID_ARG);
	assert(ld19_del(h) == ESP_OK);
	assert(ld19_del(NULL) == ESP_ERR_INVALID_ARG);
}

int main(void)
{
	run_ld19_cases();
	test_uart_setup();
	test_esp_errors();
	assert(!installed && allocs == 0);
	printf("all ld19 checks passed (ESP-IDF)\n");
	return 0;
}
