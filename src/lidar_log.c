#include <stdint.h>

#include "lidar_log.h"

/*
 * Decimal text of v, written backwards from end. newlib-nano's printf, the
 * one stm-forge links, has no %lld, so the timestamp cannot go through PRId64.
 */
static const char *lidar_format_i64(char *end, int64_t v)
{
	uint64_t u = v < 0 ? 0 - (uint64_t)v : (uint64_t)v;

	*--end = '\0';
	do {
		*--end = (char)('0' + u % 10);
		u /= 10;
	} while (u);
	if (v < 0)
		*--end = '-';
	return end;
}

void lidar_log_scan(const struct lidar_dev *lidar,
		    const struct lidar_scan *scan)
{
	char t[21];	/* "-9223372036854775808" and its NUL */
	const struct lidar_point *nearest = NULL;

	for (size_t i = 0; i < scan->count; i++) {
		const struct lidar_point *p = &scan->points[i];

		if (p->distance_mm &&
		    (!nearest || p->distance_mm < nearest->distance_mm))
			nearest = p;
	}
	lidar_info(lidar, "t %s us  %u points%s  %.0f deg/s  %u ms  "
		   "nearest %u mm at %.1f deg",
		   lidar_format_i64(t + sizeof(t), scan->t_us),
		   (unsigned)scan->count, scan->truncated ? " (truncated)" : "",
		   (double)scan->speed_dps,
		   (unsigned)((scan->t_end_us - scan->t_us) / 1000),
		   nearest ? (unsigned)nearest->distance_mm : 0u,
		   nearest ? (double)nearest->angle_deg : 0.0);
}
