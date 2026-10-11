/*
 * LDROBOT's near filter for the LD06 and LD19, ported from its SDK:
 * ldlidar_stl_ros 964afad8, ldlidar_driver/src/filter/tofbf.cpp, Tofbf's
 * constructor (:31-52) and NearFilter() (:83-182). MIT, (c) 2020 SHENZHEN
 * LDROBOT CO., LTD; the notice is in LICENSE-LDROBOT.
 *
 * Same decisions, without the SDK's containers: the SDK splits the points
 * into those within 5 m and the rest, sorts the near ones by angle, groups
 * them, zeroes the groups it judges noise, and returns everything in a new
 * order that its caller then sorts by time. Here the scan is sorted by angle
 * in place, the groups are walked over the near points in that order --
 * a far point between two near ones does not break their group, as in the
 * SDK -- and a noise group's points are zeroed where they lie. The scan ends
 * sorted by angle.
 *
 * One difference: with a single group the SDK's 0/360 join inserts that
 * group into itself, which std::vector does not allow, and then erases it;
 * here a single group is never joined to itself.
 */
#include <math.h>
#include <stddef.h>

#include "lidar.h"

#define NEAR_MM			5000	/* :90, closer than this is filtered */
#define SCAN_FREQUENCY		4500	/* :39, measurements per second */
#define INTENSITY_LOW		15	/* :37, a group fainter is noise */
#define INTENSITY_SINGLE	220	/* :38, a group under 3 needs this */
#define DISTANCE_STEP		0.03	/* :109, 3 % of the last distance */
#define GROUP_KEPT		15	/* :135, more points than this stay */
#define GROUP_SMALL		3	/* :141, fewer is a near-single */

struct group {
	size_t first, last;	/* positions in the scan, near points only */
	size_t n;
	unsigned long intensity;	/* sum; the SDK's distance mean
					 * (:162-165) decides nothing */
};

static bool near_point(const struct lidar_point *p)
{
	return p->distance_mm < NEAR_MM;
}

/* Insertion sort by angle: a scan is in rotation order already, so this is
 * one pass but for the few points jitter moved. Stable. */
static void sort_by_angle(struct lidar_point *p, size_t n)
{
	for (size_t i = 1; i < n; i++) {
		struct lidar_point v = p[i];
		size_t j = i;

		while (j > 0 && p[j - 1].angle_deg > v.angle_deg) {
			p[j] = p[j - 1];
			j--;
		}
		p[j] = v;
	}
}

static void group_add(struct group *g, size_t at, const struct lidar_point *p)
{
	if (!g->n)
		g->first = at;
	g->last = at;
	g->n++;
	g->intensity += p->intensity;
}

/* :132-178: kept, or every near point in it zeroed. */
static bool group_is_noise(const struct group *g)
{
	if (g->n > GROUP_KEPT)
		return false;
	/* :142-147: an int average, as the SDK takes it */
	if (g->n < GROUP_SMALL &&
	    (int)(g->intensity / g->n) < INTENSITY_SINGLE)
		return true;
	return !((double)g->intensity / (double)g->n > INTENSITY_LOW);
}

static void zero_range(struct lidar_point *p, size_t first, size_t last)
{
	for (size_t i = first; i <= last; i++) {
		if (near_point(&p[i])) {
			p[i].distance_mm = 0;
			p[i].intensity = 0;
		}
	}
}

/* Positions first..last, near points only, wrapping past the end. */
static void zero_group(struct lidar_point *p, size_t count,
		       const struct group *g)
{
	if (g->first <= g->last) {
		zero_range(p, g->first, g->last);
	} else {
		zero_range(p, g->first, count - 1);
		zero_range(p, 0, g->last);
	}
}

void lidar_filter_near(struct lidar_scan *scan)
{
	struct group head = { 0 }, cur = { 0 };
	const struct lidar_point *last = NULL;
	struct lidar_point *p;
	double limit;
	size_t groups = 0, i;

	if (!scan || !scan->points || !scan->count)
		return;
	p = scan->points;
	sort_by_angle(p, scan->count);

	/* :32, :99: the speed is taken as an int, degrees per second. */
	limit = (double)(int)scan->speed_dps / SCAN_FREQUENCY * 2;

	for (i = 0; i < scan->count; i++) {
		if (!near_point(&p[i]))
			continue;
		/* :108-109; the SDK's first "last" is at -10 degrees, 0 mm */
		if (last && (fabs(p[i].angle_deg - last->angle_deg) > limit ||
			     fabs((double)p[i].distance_mm -
				  last->distance_mm) >
				     last->distance_mm * DISTANCE_STEP)) {
			/* The first group waits: it may join the last. */
			if (groups++ == 0)
				head = cur;
			else if (group_is_noise(&cur))
				zero_group(p, scan->count, &cur);
			cur = (struct group){ 0 };
		}
		group_add(&cur, i, &p[i]);
		last = &p[i];
	}
	if (!cur.n)
		return;
	if (groups++ == 0) {
		/* One group: nothing to join it to. */
		if (group_is_noise(&cur))
			zero_group(p, scan->count, &cur);
		return;
	}

	/* :124-130: 0 and 359 degrees joined, the last group to the first. */
	if (fabs(p[head.first].angle_deg + 360.0f - p[cur.last].angle_deg) <
		    limit &&
	    fabs((double)p[head.first].distance_mm - p[cur.last].distance_mm) <
		    p[cur.last].distance_mm * DISTANCE_STEP) {
		struct group joined = cur;

		joined.last = head.last;
		joined.n += head.n;
		joined.intensity += head.intensity;
		if (group_is_noise(&joined))
			zero_group(p, scan->count, &joined);
		return;
	}
	if (group_is_noise(&head))
		zero_group(p, scan->count, &head);
	if (group_is_noise(&cur))
		zero_group(p, scan->count, &cur);
}
