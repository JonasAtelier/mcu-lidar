/*
 * The core's cases -- packets and their timestamp, revolution assembly with
 * the SDK's 340/20 crossing, the carried split packet, truncation, the
 * too-long and runaway turns, the deadline, the byte ring and the log --
 * shared by esp/test_core.c and stm/test_core.c. Each includes this after
 * providing:
 *
 *   LOG_PREFIX             what the platform's log puts before "lidar: "
 *   stub_log_last          the last log line
 *   now_us(), set_now_us() the platform's clock behind lidar_time_us()
 *   pass_ms(ms)            let time pass, as a wait for data would
 */

/* A queue of packets the fake chip hands out, one per read. */
static struct lidar_packet queue[128];
static int queued, served;
static int fake_ret;

static int fake_read(struct lidar_dev *l, struct lidar_packet *p,
		     uint32_t timeout_ms)
{
	(void)l;
	if (fake_ret)
		return fake_ret;
	if (served == queued) {
		pass_ms(timeout_ms);		/* nothing came */
		return -ETIMEDOUT;
	}
	pass_ms(2);				/* a packet every ~2.7 ms */
	*p = queue[served++];
	p->t_us = -1;				/* the core stamps it */
	return 0;
}

static const struct lidar_ops fake_ops = { .read_packet = fake_read };

static struct lidar_dev fake_dev(void)
{
	struct lidar_dev d = { .name = "fake", .ops = &fake_ops,
			       .rate_hz = 4500 };

	queued = served = 0;
	fake_ret = 0;
	set_now_us(1000000);
	return d;
}

/* 12 points from a0, step apart, in hundredths of a degree, wrapped as
 * the LD19's parser wraps them. */
static void push(int a0, int step, float speed)
{
	struct lidar_packet *p = &queue[queued++];

	assert(queued <= 128);
	memset(p, 0, sizeof(*p));
	p->count = 12;
	p->speed_dps = speed;
	for (int i = 0; i < 12; i++) {
		p->points[i].angle_deg = (float)((a0 + step * i) % 36000) / 100.0f;
		p->points[i].distance_mm = (uint16_t)(1000 + queued * 12 + i);
		p->points[i].intensity = 100;
	}
}

/*
 * Whole turns: 40 packets 9 degrees apart from 4.5, 12 points 0.75 apart.
 * The last packet of each turn starts at 355.5 and crosses 360 at its 7th
 * point, so every turn is 480 points, 0.00 to 359.25.
 */
static void push_turns(int turns, float speed)
{
	for (int t = 0; t < turns; t++)
		for (int k = 0; k < 40; k++)
			push(450 + 900 * k, 75, speed);
}

static void test_packet(void)
{
	struct lidar_dev d = fake_dev();
	struct lidar_packet p;

	push(0, 100, 3600);
	set_now_us(5000000);
	assert(lidar_read_packet(&d, &p, 100) == 0);
	assert(p.count == 12 && p.t_us == 5002000);	/* after it came */
	assert(lidar_read_packet(&d, &p, 100) == -ETIMEDOUT);
	fake_ret = -EIO;
	assert(lidar_read_packet(&d, &p, 100) == -EIO);
	assert(lidar_read_packet(NULL, &p, 100) == -EINVAL);
	assert(lidar_read_packet(&d, NULL, 100) == -EINVAL);
}

/*
 * The first scan starts at the first crossing; it ends at the next, inside
 * a packet; the tail opens the second scan, which needs no extra turn.
 */
static void test_scan_assembly(void)
{
	struct lidar_dev d = fake_dev();
	struct lidar_point pts[600];
	struct lidar_scan s = { .points = pts, .capacity = 600 };
	int before;

	push_turns(3, 3600);
	assert(lidar_read_scan(&d, &s, 1000) == 0);
	assert(served == 80);		/* one turn to find, one to read */
	assert(s.count == 480 && !s.truncated && s.speed_dps == 3600.0f);
	assert(pts[0].angle_deg == 0.0f && pts[5].angle_deg == 3.75f);
	assert(pts[6].angle_deg == 4.5f && pts[479].angle_deg == 359.25f);
	assert(s.t_end_us > s.t_us);

	/* The second starts on the carried tail: one turn of packets. */
	before = served;
	assert(lidar_read_scan(&d, &s, 1000) == 0);
	assert(served - before == 40 && s.count == 480);
	assert(pts[0].angle_deg == 0.0f && pts[479].angle_deg == 359.25f);
}

/* A backward step that is not a crossing does not end the scan; only an
 * angle under 20 after one over 340 does (lipkg.cpp:196). */
static void test_crossing_rule(void)
{
	struct lidar_dev d = fake_dev();
	struct lidar_point pts[400];
	struct lidar_scan s = { .points = pts, .capacity = 400 };

	push(35550, 75, 3600);		/* the first crossing, at 0.00 */
	push(10000, 75, 3600);		/* 100.00 .. 108.25 */
	push(9900, 75, 3600);		/* back to 99.00: not a crossing */
	push(1000, 75, 3600);		/* back to 10.00: under 20, but after
					 * 107.25, not over 340 */
	push(35550, 75, 3600);		/* the next crossing */
	assert(lidar_read_scan(&d, &s, 1000) == 0);
	assert(s.count == 6 + 12 + 12 + 12 + 6);
	assert(pts[6].angle_deg == 100.0f && pts[18].angle_deg == 99.0f);
	assert(pts[30].angle_deg == 10.0f);

	/* The bounds: 345 then 15 crosses; 335 then 15 does not; 358.25
	 * then 12 does, past any small step. */
	d = fake_dev();
	push(35550, 75, 3600);		/* first crossing */
	push(3450, 75, 3600);		/* 34.50 .. */
	push(33500, 75, 3600);		/* 335.00 .. 343.25 */
	push(1500, 75, 3600);		/* 15.00: after 343.25, a crossing */
	assert(lidar_read_scan(&d, &s, 1000) == 0);
	assert(s.count == 6 + 12 + 12);
	d = fake_dev();
	push(35550, 75, 3600);
	push(32500, 75, 3600);		/* 325.00 .. 333.25 */
	push(1500, 75, 3600);		/* after 333.25: not a crossing */
	push(35000, 75, 3600);		/* 350.00 .. 358.25 */
	push(1200, 75, 3600);		/* 12.00 after 358.25: a crossing */
	assert(lidar_read_scan(&d, &s, 1000) == 0);
	assert(s.count == 6 + 12 + 12 + 12);
}

/* Capacity: the rest of the turn dropped and flagged. */
static void test_truncated(void)
{
	struct lidar_dev d = fake_dev();
	struct lidar_point pts[100];
	struct lidar_scan s = { .points = pts, .capacity = 100 };

	push_turns(2, 3600);
	assert(lidar_read_scan(&d, &s, 1000) == 0);
	assert(s.count == 100 && s.truncated);
	assert(pts[99].angle_deg < 100);
}

/*
 * Too long: 480 points at 3600 deg/s is 480 x 10 = 4800 against
 * 4500 x 1.4 = 6300, fine; at 6000 deg/s it is 8000, flagged
 * (lipkg.cpp:197). Runaway: no crossing while points x Hz pass 4500 x 2
 * (lipkg.cpp:235): the scan ends there, flagged.
 */
static void test_too_long(void)
{
	struct lidar_dev d = fake_dev();
	struct lidar_point pts[1000];
	struct lidar_scan s = { .points = pts, .capacity = 1000 };

	push_turns(2, 6000);
	assert(lidar_read_scan(&d, &s, 1000) == 0);
	assert(s.count == 480 && s.truncated);

	/* rate_hz 0: no such check. */
	d = fake_dev();
	d.rate_hz = 0;
	push_turns(2, 6000);
	assert(lidar_read_scan(&d, &s, 1000) == 0 && !s.truncated);

	/* Runaway: never crossing again; 9000 / (3600 / 360) = 900 points,
	 * and the 901st is over. */
	d = fake_dev();
	push(35550, 75, 3600);
	for (int i = 0; i < 80; i++)
		push(10000, 0, 3600);
	assert(lidar_read_scan(&d, &s, 1000) == 0);
	assert(s.truncated && s.count == 901);
}

/* One deadline for the whole turn, on the clock. */
static void test_deadline(void)
{
	struct lidar_dev d = fake_dev();
	struct lidar_point pts[400];
	struct lidar_scan s = { .points = pts, .capacity = 400 };
	int64_t t0;

	push(35550, 75, 3600);
	for (int k = 0; k < 20; k++)
		push(450 + 900 * k, 75, 3600);	/* the turn never ends */
	t0 = now_us();
	assert(lidar_read_scan(&d, &s, 150) == -ETIMEDOUT);
	assert(now_us() - t0 >= 150000);
	assert(s.count == 0);

	assert(lidar_read_scan(&d, NULL, 10) == -EINVAL);
	s.capacity = 0;
	assert(lidar_read_scan(&d, &s, 10) == -EINVAL);
	s.capacity = 400;
	s.points = NULL;
	assert(lidar_read_scan(&d, &s, 10) == -EINVAL);

	/* Packets that keep coming do not stretch it: 100 of them 2 ms
	 * apart, no crossing, no runaway check -- the turn gives up at
	 * 150 ms, not after the last packet. */
	d = fake_dev();
	d.rate_hz = 0;
	s.points = pts;
	push(35550, 75, 3600);
	for (int k = 0; k < 100; k++)
		push(10000, 0, 3600);
	t0 = now_us();
	assert(lidar_read_scan(&d, &s, 150) == -ETIMEDOUT);
	assert(now_us() - t0 >= 150000 && now_us() - t0 < 160000);
	assert(served < 100);
}

static void test_ring(void)
{
	struct lidar_ring r;
	uint8_t buf[8], out[16];
	uint8_t in[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

	assert(lidar_ring_init(&r, buf, 6) == -EINVAL);
	assert(lidar_ring_init(&r, buf, 8) == 0);
	assert(lidar_ring_put(&r, in, 5) == 5);
	assert(lidar_ring_get(&r, out, 3) == 3 && out[2] == 3);
	/* Wraps: 2 waiting, 6 more fit, 4 dropped and counted. */
	assert(lidar_ring_put(&r, in, 10) == 6 && r.dropped == 4);
	assert(lidar_ring_get(&r, out, 16) == 8);
	assert(out[0] == 4 && out[1] == 5 && out[2] == 1 && out[7] == 6);
	assert(lidar_ring_get(&r, out, 16) == 0);
	/* The indices run free past 2^32 without a hitch. */
	r.head = r.tail = UINT32_MAX - 1;
	assert(lidar_ring_put(&r, in, 4) == 4);
	assert(lidar_ring_get(&r, out, 4) == 4 && out[3] == 4);
}

static void test_log(void)
{
	struct lidar_dev d = fake_dev();
	struct lidar_point pts[3] = {
		{ 10, 0, 0 }, { 20.5f, 1500, 9 }, { 30, 800, 9 },
	};
	struct lidar_scan s = { .points = pts, .capacity = 3, .count = 3,
				.speed_dps = 3600, .t_us = 2000000,
				.t_end_us = 2100000 };

	lidar_log_scan(&d, &s);
	assert(!strcmp(stub_log_last, LOG_PREFIX("I")
		       "lidar: fake: t 2000000 us  3 points  3600 deg/s  "
		       "100 ms  nearest 800 mm at 30.0 deg"));
	s.truncated = true;
	lidar_log_scan(&d, &s);
	assert(strstr(stub_log_last, "3 points (truncated)"));
}

static void run_core_cases(void)
{
	test_packet();
	test_scan_assembly();
	test_crossing_rule();
	test_truncated();
	test_too_long();
	test_deadline();
	test_ring();
	test_log();
}
