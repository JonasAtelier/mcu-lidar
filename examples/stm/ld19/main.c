/*
 * LD19 on an STM32: the UART's DMA hands every chunk to the driver, which
 * the main loop reads a turn at a time.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the
 * UART are the app's -- here huart2 as CubeMX generates it
 * (MX_USART2_UART_Init()) at 230400 8N1 (LD19 Development Manual V2.3,
 * section 2, p.6), RX only, with its RX on a DMA stream in circular mode.
 * The lidar's PWM pin goes to ground. Needs HAL_UART_MODULE_ENABLED,
 * HAL_DMA_MODULE_ENABLED and LIDAR_LD19 = y.
 *
 * Why DMA: the lidar sends about 17.6 KB/s without a pause. A blocking
 * HAL_UART_Receive() only listens while it runs, and a byte lasts 43 us at
 * 230400, so the time spent between two receives loses bytes, and packets,
 * every turn. Set cfg.fed false to read that way anyway.
 */
#include CMSIS_device_header

#include "ld19.h"
#include "lidar.h"
#include "lidar_log.h"

extern UART_HandleTypeDef huart2;	/* from MX_USART2_UART_Init() */

static ld19_handle_t lidar;
static uint8_t dma_buf[256];
static uint16_t dma_old;		/* where the last event left off */

/*
 * ReceiveToIdle in circular mode: size is the position the DMA has written
 * up to; hand over everything since the last call, wrapping at the end.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart != &huart2)
		return;
	if (size > dma_old) {
		ld19_rx_feed(lidar, dma_buf + dma_old, size - dma_old);
	} else if (size < dma_old) {
		ld19_rx_feed(lidar, dma_buf + dma_old, sizeof(dma_buf) - dma_old);
		ld19_rx_feed(lidar, dma_buf, size);
	}
	dma_old = size == sizeof(dma_buf) ? 0 : size;
}

void lidar_task(void)
{
	static struct lidar_point points[LD19_SCAN_MAX_POINTS];
	struct lidar_scan scan = {
		.points = points,
		.capacity = LD19_SCAN_MAX_POINTS,
	};
	ld19_config_t cfg = { .uart = &huart2, .fed = true };

	if (ld19_init(&cfg, &lidar))
		return;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_buf, sizeof(dma_buf));

	for (;;) {
		/* -ETIMEDOUT: no turn in 300 ms -- supply, wire, rotor. */
		if (lidar_read_scan(ld19_lidar(lidar), &scan, 300))
			continue;
		lidar_log_scan(ld19_lidar(lidar), &scan);
	}
}
