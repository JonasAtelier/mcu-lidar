#include <errno.h>
#include <string.h>

#include "lidar.h"
#include "lidar_priv.h"

/*
 * The 0-degree crossing and the too-many-points limits of LDROBOT's SDK,
 * ldlidar_stl_ros 964afad8, ldlidar_driver/src/dataprocess/lipkg.cpp: a
 * turn ends where an angle under 20 follows one over 340 (:196); a turn
 * whose points times the rotor's Hz exceed the measurement rate x 1.4 is
 * too long for one turn (:197), and twice the rate with no crossing at all
 * is a runaway (:235). The SDK drops such data; here the turn is returned,
 * flagged truncated.
 */
#define CROSS_BELOW_DEG		20.0f
#define CROSS_ABOVE_DEG		340.0f
#define TURN_LIMIT		1.4
#define RUNAWAY_LIMIT		2.0

static bool lidar_bound(const struct lidar_dev *lidar)
{
	return lidar && lidar->ops && lidar->ops->read_packet;
}

int lidar_read_packet(struct lidar_dev *lidar, struct lidar_packet *pkt,
		      uint32_t timeout_ms)
{
	struct lidar_packet tmp = { 0 };
	int ret;

	if (!lidar_bound(lidar) || !pkt)
		return -EINVAL;
	ret = lidar->ops->read_packet(lidar, &tmp, timeout_ms);
	if (ret)
		return ret;
	tmp.t_us = lidar_time_us();
	*pkt = tmp;
	return 0;
}

/* points x Hz against rate x limit; Hz is degrees per second / 360. */
static bool lidar_over(const struct lidar_dev *lidar, size_t points,
		       float speed_dps, double limit)
{
	return lidar->rate_hz &&
	       (double)points * speed_dps / 360.0 > lidar->rate_hz * limit;
}

int lidar_collect_turn(struct lidar_dev *lidar, size_t capacity,
		       void (*put)(void *ctx, size_t index,
				   const struct lidar_point *point),
		       void *ctx, struct lidar_turn *turn,
		       uint32_t timeout_ms)
{
	int64_t start = lidar_time_us();
	bool collecting = lidar->carrying;

	memset(turn, 0, sizeof(*turn));
	for (;;) {
		struct lidar_packet pkt;
		uint32_t elapsed;
		int i = 0, ret;

		if (lidar->carrying) {
			pkt = lidar->carry;
			i = lidar->carry_next;
			lidar->carrying = false;
		} else {
			elapsed = lidar_elapsed_ms(start);
			if (elapsed >= timeout_ms)
				return -ETIMEDOUT;
			ret = lidar_read_packet(lidar, &pkt,
						timeout_ms - elapsed);
			if (ret)
				return ret;
		}

		for (; i < pkt.count; i++) {
			const struct lidar_point *p = &pkt.points[i];
			bool crossed = lidar->have_last &&
				       p->angle_deg < CROSS_BELOW_DEG &&
				       lidar->last_angle > CROSS_ABOVE_DEG;

			lidar->last_angle = p->angle_deg;
			lidar->have_last = true;

			if (crossed) {
				if (collecting) {
					/* The tail opens the next turn. Its
					 * first point is last_angle already,
					 * so it is not a crossing again. */
					lidar->carry = pkt;
					lidar->carry_next = (uint8_t)i;
					lidar->carrying = true;
					if (lidar_over(lidar, turn->total,
						       turn->speed_dps,
						       TURN_LIMIT))
						turn->truncated = true;
					return 0;
				}
				collecting = true;
			}
			if (!collecting)
				continue;

			if (!turn->total)
				turn->t_us = pkt.t_us;
			turn->t_end_us = pkt.t_us;
			turn->speed_dps = pkt.speed_dps;
			if (turn->count < capacity)
				put(ctx, turn->count++, p);
			else
				turn->truncated = true;
			turn->total++;

			/* No crossing in two turns' worth: end it here. */
			if (lidar_over(lidar, turn->total, turn->speed_dps,
				       RUNAWAY_LIMIT)) {
				lidar->carry = pkt;
				lidar->carry_next = (uint8_t)(i + 1);
				lidar->carrying = true;
				turn->truncated = true;
				return 0;
			}
		}
	}
}

static void lidar_put_point(void *ctx, size_t index,
			    const struct lidar_point *point)
{
	struct lidar_scan *scan = ctx;

	scan->points[index] = *point;
}

int lidar_read_scan(struct lidar_dev *lidar, struct lidar_scan *scan,
		    uint32_t timeout_ms)
{
	struct lidar_turn turn;
	int ret;

	if (!lidar_bound(lidar) || !scan || !scan->points || !scan->capacity)
		return -EINVAL;

	scan->count = 0;
	ret = lidar_collect_turn(lidar, scan->capacity, lidar_put_point, scan,
				 &turn, timeout_ms);
	if (ret)
		return ret;
	scan->count = turn.count;
	scan->truncated = turn.truncated;
	scan->speed_dps = turn.speed_dps;
	scan->t_us = turn.t_us;
	scan->t_end_us = turn.t_end_us;
	return 0;
}

uint32_t lidar_elapsed_ms(int64_t start)
{
	int64_t us = lidar_time_us() - start;

	if (us <= 0)
		return 0;
	return us / 1000 > UINT32_MAX ? UINT32_MAX : (uint32_t)(us / 1000);
}

/* ------------------------------------------------------------- ring -- */

int lidar_ring_init(struct lidar_ring *ring, uint8_t *buf, uint32_t size)
{
	if (!ring || !buf || !size || (size & (size - 1)))
		return -EINVAL;
	memset(ring, 0, sizeof(*ring));
	ring->buf = buf;
	ring->size = size;
	return 0;
}

/*
 * One writer and one reader, each owning one index: the writer fills the
 * bytes before it publishes head, the reader takes them before it
 * publishes tail. Acquire and release order that on any core.
 */
size_t lidar_ring_put(struct lidar_ring *ring, const uint8_t *data,
		      size_t len)
{
	uint32_t head = ring->head;
	uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
	uint32_t room = ring->size - (head - tail);
	size_t n = len < room ? len : room;

	for (size_t i = 0; i < n; i++)
		ring->buf[(head + i) & (ring->size - 1)] = data[i];
	__atomic_store_n(&ring->head, head + (uint32_t)n, __ATOMIC_RELEASE);
	if (n < len)
		ring->dropped += (uint32_t)(len - n);
	return n;
}

size_t lidar_ring_get(struct lidar_ring *ring, uint8_t *out, size_t len)
{
	uint32_t tail = ring->tail;
	uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
	uint32_t have = head - tail;
	size_t n = len < have ? len : have;

	for (size_t i = 0; i < n; i++)
		out[i] = ring->buf[(tail + i) & (ring->size - 1)];
	__atomic_store_n(&ring->tail, tail + (uint32_t)n, __ATOMIC_RELEASE);
	return n;
}
