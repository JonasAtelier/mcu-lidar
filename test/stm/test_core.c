/*
 * Host-side check of the core on the STM32 HAL: the cases are
 * ../core_cases.h; this file supplies the log prefix and the clock.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "lidar.h"
#include "lidar_log.h"
#include "lidar_priv.h"

/* The HAL's lines are bare printf(), no level letter. */
#define LOG_PREFIX(level)	""

/* tof_time_us() is HAL_GetTick() * 1000 by default; HAL_Delay() moves it. */
static int64_t now_us(void)
{
	return (int64_t)stub_tick * 1000;
}

static void set_now_us(int64_t us)
{
	stub_tick = (uint32_t)(us / 1000);
}

static void pass_ms(uint32_t ms)
{
	stub_tick += ms;
}

#include "../core_cases.h"

/* With CONFIG_TOF_DEBUG on, as test/Makefile builds it, dbg prints. */
static void test_dbg(void)
{
	struct lidar_dev t = { .name = "fake" };

	lidar_dbg(&t, "d %d", 1);
	assert(!strcmp(stub_log_last, "lidar: fake: d 1"));
}

int main(void)
{
	run_core_cases();
	test_dbg();
	printf("all core checks passed\n");
	return 0;
}
