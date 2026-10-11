/*
 * Host-side check of the core on ESP-IDF: the cases are ../core_cases.h;
 * this file supplies the log prefix and the clock.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "lidar.h"
#include "lidar_log.h"
#include "lidar_priv.h"

/* The ESP-IDF stub log puts the level letter first. */
#define LOG_PREFIX(level)	level " "

/* vTaskDelay() moves it, a tick at a time: 10 ms at the stub's 100 Hz. */
static int64_t now_us(void)
{
	return stub_time_us;
}

static void set_now_us(int64_t us)
{
	stub_time_us = us;
}

static void pass_ms(uint32_t ms)
{
	stub_time_us += (int64_t)ms * 1000;
}

#include "../core_cases.h"

int main(void)
{
	run_core_cases();
	printf("all core checks passed\n");
	return 0;
}
