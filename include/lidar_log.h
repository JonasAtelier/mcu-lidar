/*
 * Kernel dev_err()-style logging for lidar drivers and their users.
 *
 * ESP-IDF: everything goes out under the one ESP-IDF tag "lidar", prefixed
 * with the device's name, so one esp_log_level_set("lidar", ...) controls
 * every chip and a line still says which chip it came from. There is
 * deliberately no level machinery of our own; lidar_dbg() is also subject
 * to CONFIG_LOG_MAXIMUM_LEVEL, and CONFIG_LIDAR_DEBUG is not used.
 *
 * STM32 HAL: every line goes out through printf() as "lidar: <name>: ...",
 * so a line still says which chip it came from. Where printf() ends up is
 * the app's business: with newlib, retarget _write() to a UART. There is
 * deliberately no level machinery; lidar_dbg() alone is compiled out
 * unless CONFIG_LIDAR_DEBUG is 1 (stm-forge: LIDAR_DEBUG = y).
 */
#ifndef LIDAR_LOG_H
#define LIDAR_LOG_H

#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include <stdio.h>
#endif

#include "lidar.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ESP_PLATFORM
#define LIDAR_LOG_TAG	"lidar"

#define lidar_err(lidar, fmt, ...) \
	ESP_LOGE(LIDAR_LOG_TAG, "%s: " fmt, (lidar)->name, ##__VA_ARGS__)
#define lidar_warn(lidar, fmt, ...) \
	ESP_LOGW(LIDAR_LOG_TAG, "%s: " fmt, (lidar)->name, ##__VA_ARGS__)
#define lidar_info(lidar, fmt, ...) \
	ESP_LOGI(LIDAR_LOG_TAG, "%s: " fmt, (lidar)->name, ##__VA_ARGS__)
#define lidar_dbg(lidar, fmt, ...) \
	ESP_LOGD(LIDAR_LOG_TAG, "%s: " fmt, (lidar)->name, ##__VA_ARGS__)

#else /* STM32 HAL */
#define lidar_err(lidar, fmt, ...) \
	printf("lidar: %s: " fmt "\n", (lidar)->name, ##__VA_ARGS__)
#define lidar_warn(lidar, fmt, ...) \
	printf("lidar: %s: " fmt "\n", (lidar)->name, ##__VA_ARGS__)
#define lidar_info(lidar, fmt, ...) \
	printf("lidar: %s: " fmt "\n", (lidar)->name, ##__VA_ARGS__)

#if CONFIG_LIDAR_DEBUG
#define lidar_dbg(lidar, fmt, ...) \
	printf("lidar: %s: " fmt "\n", (lidar)->name, ##__VA_ARGS__)
#else
/* Still type-checked, so a debug line cannot rot while it is off. */
#define lidar_dbg(lidar, fmt, ...) do { \
	if (0) \
		printf("lidar: %s: " fmt "\n", (lidar)->name, \
		       ##__VA_ARGS__); \
} while (0)
#endif
#endif /* ESP_PLATFORM */

/* One info line: points, rotor speed, duration and the nearest return. */
void lidar_log_scan(const struct lidar_dev *lidar,
		    const struct lidar_scan *scan);

#ifdef __cplusplus
}
#endif

#endif /* LIDAR_LOG_H */
