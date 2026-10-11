/*
 * Host-side check of the ld19 driver on the STM32 HAL: ../ld19_cases.h is
 * the fake scanner and the shared cases, here through blocking
 * HAL_UART_Receive(); this file adds what only the HAL side has -- the
 * fed mode, where the app's callback hands bytes to ld19_rx_feed() and the
 * driver never calls the HAL.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "ld19.h"
#include "lidar.h"

#define RET(err)	(err)

static UART_HandleTypeDef huart;
static int hal_calls;

static int64_t now_us(void)
{
	return (int64_t)stub_tick * 1000;
}

/* Whole milliseconds, rounded up: the HAL's tick is the clock. */
static void pass_us(int64_t us)
{
	stub_tick += (uint32_t)((us + 999) / 1000);
}

static ld19_config_t cfg_of(ld19_model_t model)
{
	return (ld19_config_t){ .uart = &huart, .model = model };
}

static int uart_live(void)
{
	return 0;
}

#include "../ld19_cases.h"

/* ------------------------------------------------- the HAL side of it --- */

/* Short is HAL_TIMEOUT, as the blocking receive has it. */
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *h, uint8_t *data,
				   uint16_t size, uint32_t timeout)
{
	assert(h == &huart && timeout > 0);
	hal_calls++;
	return fake_take(data, size, (int64_t)timeout * 1000) == size ?
		       HAL_OK : HAL_TIMEOUT;
}

/* ----------------------------------------------------------- STM cases --- */

/*
 * Fed: init returns at once, nothing is asked of the HAL, ever; bytes come
 * from the callback in whatever chunks the DMA hands over, across packet
 * boundaries; a read with nothing fed waits its timeout on the clock.
 */
static void test_fed(void)
{
	ld19_config_t cfg = { .uart = &huart, .fed = true };
	uint8_t raw[3 * LD19_PACKET_LEN], big[2000];
	ld19_packet_t p;
	ld19_handle_t h = NULL;
	int64_t t0;

	hal_calls = 0;
	assert(ld19_init(&cfg, &h) == 0 && h);
	packet(raw, 3600, 0, 825, 100, 0);
	packet(raw + 47, 3600, 900, 1725, 200, 0);
	packet(raw + 94, 3600, 1800, 2625, 300, 0);
	assert(ld19_rx_feed(h, raw, 30) == 30);		/* half a packet */
	t0 = now_us();
	assert(ld19_read_packet(h, &p, 5) == -ETIMEDOUT);
	assert(now_us() - t0 >= 5000);
	assert(ld19_rx_feed(h, raw + 30, 70) == 70);	/* the rest, and more */
	assert(ld19_read_packet(h, &p, 5) == 0 && p.points[0].distance_mm == 100);
	assert(ld19_read_packet(h, &p, 5) == 0 && p.points[0].distance_mm == 200);
	assert(ld19_rx_feed(h, raw + 100, 41) == 41);
	assert(ld19_read_packet(h, &p, 5) == 0 && p.points[0].distance_mm == 300);
	assert(hal_calls == 0);

	/* The ring holds 1 KiB; what does not fit is refused, counted. */
	memset(big, 0x00, sizeof big);
	assert(ld19_rx_feed(h, big, sizeof big) == 1024);
	assert(ld19_rx_feed(h, big, 1) == 0);
	assert(ld19_del(h) == 0);

	/* A handle that reads for itself takes no feed. */
	line_reset();
	send_packet(0, 1);
	h = open_ld19(LD19_MODEL_LD19);
	assert(ld19_rx_feed(h, raw, 10) == 0);
	ld19_del(h);
	assert(ld19_rx_feed(NULL, raw, 10) == 0);

	cfg.uart = NULL;
	assert(ld19_init(&cfg, &h) == -EINVAL);
}

/* The callback an app would write: DMA to idle, everything to the feed. */
static ld19_handle_t app_lidar;

static void app_rx_event(const uint8_t *buf, uint16_t n)
{
	ld19_rx_feed(app_lidar, buf, n);
}

/* Two turns, more than the ring holds, fed in 64-byte DMA chunks as the
 * reader drains it: every one of the 80 packets arrives. */
static void test_fed_stream(void)
{
	ld19_config_t cfg = { .uart = &huart, .fed = true,
			      .model = LD19_MODEL_LD06 };
	ld19_packet_t p;
	int got = 0;

	assert(ld19_init(&cfg, &app_lidar) == 0);
	line_reset();
	send_turns(2);
	while (line_pos < line_len) {
		size_t n = line_len - line_pos < 64 ? line_len - line_pos : 64;

		app_rx_event(line + line_pos, (uint16_t)n);
		line_pos += n;
		while (ld19_read_packet(app_lidar, &p, 0) == 0)
			got++;
	}
	assert(got == 80);
	ld19_del(app_lidar);
}

int main(void)
{
	run_ld19_cases();
	test_fed();
	test_fed_stream();
	printf("all ld19 checks passed (STM32 HAL)\n");
	return 0;
}
