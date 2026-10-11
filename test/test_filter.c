/*
 * Host-side check of lidar_filter_near() against the rules of LDROBOT's
 * NearFilter (ldlidar_stl_ros 964afad8, ldlidar_driver/src/filter/
 * tofbf.cpp:83-182, with the LD06/LD19 settings of :35-40), each written out
 * here: within 5 m only; groups broken by an angle step over
 * speed / 4500 x 2 degrees or a distance step over 3 % of the last; more
 * than 15 points kept; under 3 points zeroed unless their int mean
 * intensity is 220 or more; otherwise zeroed unless the mean is over 15;
 * the last group joined to the first across 360.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "lidar.h"

#define SPEED	3600.0f		/* limit: 3600 / 4500 x 2 = 1.6 degrees */

static struct lidar_point pts[64];
static size_t n;

static void add(float angle, uint16_t mm, uint8_t intensity)
{
	pts[n++] = (struct lidar_point){ angle, mm, intensity };
}

/* count points from angle a0, step degrees apart */
static void run(float a0, float step, size_t count, uint16_t mm,
		uint8_t intensity)
{
	for (size_t i = 0; i < count; i++)
		add(a0 + step * (float)i, mm, intensity);
}

static struct lidar_scan scan_of(void)
{
	return (struct lidar_scan){ .points = pts, .capacity = 64,
				    .count = n, .speed_dps = SPEED };
}

static size_t zeroed(float from, float to)
{
	size_t z = 0;

	for (size_t i = 0; i < n; i++)
		if (pts[i].angle_deg >= from && pts[i].angle_deg <= to &&
		    !pts[i].distance_mm && !pts[i].intensity)
			z++;
	return z;
}

static void test_rules(void)
{
	struct lidar_scan s;

	n = 0;
	run(10, 1, 16, 1000, 5);	/* 16 faint points: kept, > 15 */
	run(50, 1, 2, 2000, 219);	/* 2 points, mean 219 < 220: zeroed */
	run(60, 1, 2, 2000, 220);	/* 2 points, mean 220: kept */
	run(70, 1, 5, 3000, 15);	/* 5 points, mean 15: zeroed */
	run(90, 1, 5, 3000, 16);	/* 5 points, mean 16: kept */
	run(120, 1, 3, 5000, 0);	/* at 5 m: not near, kept */
	run(130, 1, 2, 4999, 0);	/* just within: zeroed */
	run(200, 1, 20, 1000, 200);	/* closes the turn, kept */
	s = scan_of();
	lidar_filter_near(&s);
	assert(zeroed(10, 25) == 0);
	assert(zeroed(50, 51) == 2 && zeroed(60, 61) == 0);
	assert(zeroed(70, 74) == 5 && zeroed(90, 94) == 0);
	assert(zeroed(120, 122) == 0 && pts[32].distance_mm == 5000);
	assert(zeroed(130, 131) == 2);
	assert(zeroed(200, 219) == 0);
}

/* Breaks: an angle step over 1.6 degrees, or a distance over 3 %. */
static void test_breaks(void)
{
	struct lidar_scan s;

	/* 4 points, 1.5 apart: one group of 4, mean 16, kept. At 1.7 apart
	 * they are 4 singles of mean 16, each under 220: all zeroed. */
	n = 0;
	run(0.5f, 1.5f, 4, 1000, 16);
	run(100, 1.7f, 4, 1000, 16);
	run(300, 1, 20, 1000, 200);
	s = scan_of();
	lidar_filter_near(&s);
	assert(zeroed(0, 10) == 0 && zeroed(100, 110) == 4);

	/* 1000 -> 1030 is 3 %, not over: one group. 1000 -> 1031: two. */
	n = 0;
	add(10, 1000, 16); add(11, 1000, 16); add(12, 1030, 16);
	add(50, 1000, 16); add(51, 1000, 16); add(52, 1031, 16);
	run(300, 1, 20, 1000, 200);
	s = scan_of();
	lidar_filter_near(&s);
	assert(zeroed(10, 12) == 0);
	assert(zeroed(50, 51) == 2 && zeroed(52, 52) == 1);

	/* A far point between two near ones does not break their group. */
	n = 0;
	add(10, 1000, 16); add(10.5f, 8000, 0); add(11, 1000, 16);
	add(12, 1000, 16);
	run(300, 1, 20, 1000, 200);
	s = scan_of();
	lidar_filter_near(&s);
	assert(zeroed(10, 12) == 0 && pts[1].distance_mm == 8000);
}

/* The last group joins the first across 360: two pairs of mean 100 would
 * each be zeroed; joined, four points of mean 100 are kept. */
static void test_join(void)
{
	struct lidar_scan s;

	n = 0;
	add(0.2f, 1000, 100); add(1.0f, 1000, 100);
	run(100, 1, 20, 2000, 200);
	add(358.9f, 1000, 100); add(359.6f, 1000, 100);
	s = scan_of();
	lidar_filter_near(&s);
	assert(zeroed(0, 2) == 0 && zeroed(358, 360) == 0);

	/* Too far apart in distance: not joined, both pairs zeroed. */
	n = 0;
	add(0.2f, 1000, 100); add(1.0f, 1000, 100);
	run(100, 1, 20, 2000, 200);
	add(358.9f, 1200, 100); add(359.6f, 1200, 100);
	s = scan_of();
	lidar_filter_near(&s);
	assert(zeroed(0, 2) == 2 && zeroed(358, 360) == 2);
}

/* One group only: never joined to itself; decided on its own. Sorted. */
static void test_single_and_sort(void)
{
	struct lidar_scan s;

	n = 0;
	add(359.5f, 1000, 10); add(0.3f, 1000, 10);	/* out of order */
	s = scan_of();
	lidar_filter_near(&s);
	assert(pts[0].angle_deg < pts[1].angle_deg);
	/* 0.3 and 359.5 sorted are two groups, joined across 360: still
	 * two points of mean 10, zeroed. */
	assert(zeroed(0, 360) == 2);

	n = 0;
	run(10, 1, 4, 1000, 50);
	s = scan_of();
	lidar_filter_near(&s);
	assert(zeroed(0, 360) == 0);

	/* Nothing to do, and no crash. */
	s = (struct lidar_scan){ 0 };
	lidar_filter_near(&s);
	lidar_filter_near(NULL);
}

int main(void)
{
	test_rules();
	test_breaks();
	test_join();
	test_single_and_sort();
	printf("test_filter: all passed\n");
	return 0;
}
