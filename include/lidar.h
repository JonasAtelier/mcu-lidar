/*
 * lidar -- a small Linux-IIO-style 2D scanning lidar subsystem for ESP-IDF
 * and the STM32 HAL.
 *
 * One struct lidar_dev per scanner. It holds what every spinning scanner
 * shares: packets of points (angle, distance, intensity) as the scanner
 * sends them, and whole revolutions assembled from them, with the rotor
 * speed and a timestamp. Code above this header sees one shape of point
 * whichever scanner produced it.
 *
 * Each chip keeps its own handle API; the handle holds a struct lidar_dev,
 * and <chip>_lidar() hands it out. The handle's init fills it and its del
 * ends it: there is no probe or remove of its own.
 *
 * Every int-returning function returns 0 or a negative errno.
 */
#ifndef LIDAR_H
#define LIDAR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The most points one packet carries, of any chip here. */
#define LIDAR_PACKET_MAX	12

struct lidar_point {
	float angle_deg;	/* 0..360, as the scanner counts it */
	uint16_t distance_mm;	/* 0: no return */
	uint8_t intensity;	/* 0..255; higher is a firmer return */
};

struct lidar_packet {
	struct lidar_point points[LIDAR_PACKET_MAX];
	uint8_t count;
	float speed_dps;	/* rotor speed, degrees per second */
	uint16_t sensor_ms;	/* the scanner's own clock */
	int64_t t_us;		/* lidar_time_us() when it was complete */
};

/*
 * One revolution. points and capacity are the caller's; the rest is filled
 * in. truncated: not one clean turn -- points past capacity were dropped,
 * or the turn had more points than one turn at its speed can (see the
 * chip's docs).
 */
struct lidar_scan {
	struct lidar_point *points;
	size_t capacity;
	size_t count;
	bool truncated;
	float speed_dps;	/* of the turn's last packet */
	int64_t t_us;		/* its first packet's t_us */
	int64_t t_end_us;	/* its last packet's t_us */
};

struct lidar_dev;

struct lidar_ops {
	/*
	 * The next valid packet, waiting at most timeout_ms; -ETIMEDOUT
	 * otherwise. Fills everything but t_us.
	 */
	int (*read_packet)(struct lidar_dev *lidar, struct lidar_packet *pkt,
			   uint32_t timeout_ms);
};

struct lidar_dev {
	const char *name;		/* "ld19"; log prefix */
	const struct lidar_ops *ops;
	void *priv;			/* the chip's handle */
	/* Measurements per second, for the too-many-points check of
	 * lidar_read_scan(); 0 skips it. */
	uint32_t rate_hz;

	/* The revolution assembler's state; see lidar_read_scan(). */
	struct lidar_packet carry;	/* the packet the last turn ended in */
	uint8_t carry_next;		/* its first point of this turn */
	bool carrying;
	bool have_last;
	float last_angle;
};

/* One packet. Taken between lidar_read_scan() calls, its points are missing
 * from the next scan. */
int lidar_read_packet(struct lidar_dev *lidar, struct lidar_packet *pkt,
		      uint32_t timeout_ms);

/*
 * One revolution into scan->points: from one crossing of 0 degrees to the
 * next, the crossing being an angle under 20 degrees after one over 340
 * (as LDROBOT's SDK has it). The packet a crossing falls inside is split;
 * its tail opens the next scan, so only the first call spends a turn
 * finding a crossing. One deadline for the whole turn, timeout_ms on
 * lidar_time_us(). -EINVAL for no points array or no capacity.
 */
int lidar_read_scan(struct lidar_dev *lidar, struct lidar_scan *scan,
		    uint32_t timeout_ms);

/*
 * The clock packets are stamped with, in microseconds. On ESP-IDF it is
 * esp_timer_get_time(). On the STM32 HAL the default is weak and only
 * HAL_GetTick() * 1000: millisecond steps that wrap after 49.7 days. Define
 * your own lidar_time_us() for real microseconds.
 */
int64_t lidar_time_us(void);

#ifdef __cplusplus
}
#endif

#endif /* LIDAR_H */
