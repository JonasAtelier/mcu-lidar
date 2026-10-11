/*
 * A fake LD19 on its serial line, and this library's cases against it,
 * shared by esp/test_ld19.c and stm/test_ld19.c. The line is a byte queue
 * the scanner has already sent, plus an optional late chunk that arrives at
 * a set time, so a case can tell a wait that lasts from one that does not.
 * Packets are built to the manual's layout (MAN 3.1, p.7-8) and sealed with
 * ld19_crc8(), which test_ld19_proto.c holds to the manual's whole table.
 *
 * Each platform file feeds it from its own UART calls through fake_take()
 * and provides:
 *
 *   RET(err)        what the handle API returns for a negative errno
 *   cfg_of(model)   a config for that platform's UART, read mode
 *   now_us()        the platform's clock behind lidar_time_us()
 *   pass_us(us)     let time pass
 *   uart_live()     UART drivers installed and not deleted (0 on the HAL)
 */

#include "ld19_proto.h"

extern char stub_log_last[256];

/* ------------------------------------------------------- the fake line --- */

static uint8_t line[16384];
static size_t line_len, line_pos;
static uint8_t late[512];
static size_t late_len;
static int64_t late_at_us = -1;

static void line_reset(void)
{
	line_len = line_pos = late_len = 0;
	late_at_us = -1;
}

static void put_bytes(const uint8_t *b, size_t n)
{
	assert(line_len + n <= sizeof line);
	memcpy(line + line_len, b, n);
	line_len += n;
}

/* A packet, sealed: speed deg/s, angles in hundredths, distances from mm0. */
static void packet(uint8_t raw[LD19_PACKET_LEN], uint16_t speed,
		   uint16_t start, uint16_t end, uint16_t mm0, uint16_t ts)
{
	memset(raw, 0, LD19_PACKET_LEN);
	raw[0] = 0x54;
	raw[1] = 0x2C;
	raw[2] = (uint8_t)speed;
	raw[3] = (uint8_t)(speed >> 8);
	raw[4] = (uint8_t)start;
	raw[5] = (uint8_t)(start >> 8);
	for (int i = 0; i < 12; i++) {
		uint16_t d = (uint16_t)(mm0 + i);

		raw[6 + 3 * i] = (uint8_t)d;
		raw[7 + 3 * i] = (uint8_t)(d >> 8);
		raw[8 + 3 * i] = (uint8_t)(100 + i);
	}
	raw[42] = (uint8_t)end;
	raw[43] = (uint8_t)(end >> 8);
	raw[44] = (uint8_t)ts;
	raw[45] = (uint8_t)(ts >> 8);
	raw[46] = ld19_crc8(raw, 46);
}

/* One packet onto the line: 12 points 0.75 degrees apart from start. */
static void send_packet(uint16_t start, uint16_t mm0)
{
	uint8_t raw[LD19_PACKET_LEN];

	packet(raw, 3600, start, (uint16_t)((start + 825) % 36000), mm0, 7);
	put_bytes(raw, sizeof raw);
}

/* Whole turns: 40 packets 9 degrees apart from 4.5; the last crosses 360
 * at its 7th point, so each turn is 480 points, 0.00 to 359.25. */
static void send_turns(int turns)
{
	for (int t = 0; t < turns; t++)
		for (int k = 0; k < 40; k++)
			send_packet((uint16_t)(450 + 900 * k),
				    (uint16_t)(1000 + 12 * k));
}

/*
 * Up to n bytes for a reader that will wait wait_us for them: what is on
 * the line, plus the late chunk if it arrives within the wait. A short take
 * means the whole wait passed.
 */
static size_t fake_take(uint8_t *buf, size_t n, int64_t wait_us)
{
	size_t got;

	if (line_len - line_pos < n && late_len && late_at_us >= 0 &&
	    now_us() + wait_us >= late_at_us) {
		if (now_us() < late_at_us)
			pass_us(late_at_us - now_us());
		put_bytes(late, late_len);
		late_len = 0;
	}
	got = line_len - line_pos < n ? line_len - line_pos : n;
	memcpy(buf, line + line_pos, got);
	line_pos += got;
	if (got < n)
		pass_us(wait_us);
	return got;
}

/* --------------------------------------------------------- allocation --- */

static int allocs;
static bool calloc_fails;
void *__real_calloc(size_t n, size_t size);
void __real_free(void *p);

void *__wrap_calloc(size_t n, size_t size)
{
	void *p;

	if (calloc_fails)
		return NULL;
	p = __real_calloc(n, size);
	if (p)
		allocs++;
	return p;
}

void __wrap_free(void *p)
{
	if (p)
		allocs--;
	__real_free(p);
}

static ld19_handle_t open_ld19(ld19_model_t model)
{
	ld19_config_t cfg = cfg_of(model);
	ld19_handle_t h = NULL;

	assert(ld19_init(&cfg, &h) == RET(0) && h);
	return h;
}

/* ------------------------------------------------------------- cases --- */

/* Init finds the first packet past garbage, a lone header and a broken
 * packet; the name is the model's. */
static void test_init(void)
{
	static const uint8_t junk[] = { 0x00, 0x54, 0x13, 0x54, 0x2C, 0x99 };
	uint8_t raw[LD19_PACKET_LEN];
	ld19_handle_t h;

	line_reset();
	allocs = 0;
	put_bytes(junk, sizeof junk);
	packet(raw, 3600, 0, 825, 500, 1);
	raw[20] ^= 0x40;			/* CRC no longer matches */
	put_bytes(raw, sizeof raw);
	send_packet(0, 500);
	h = open_ld19(LD19_MODEL_LD19);
	assert(!strcmp(ld19_lidar(h)->name, "ld19") && allocs == 1);
	assert(ld19_lidar(h)->rate_hz == 4500);
	assert(ld19_del(h) == RET(0) && allocs == 0 && uart_live() == 0);

	line_reset();
	send_packet(0, 500);
	h = open_ld19(LD19_MODEL_LD06);
	assert(!strcmp(ld19_lidar(h)->name, "ld06"));
	ld19_del(h);
}

/* Silence is -ENXIO; bytes but never a packet, -ENODEV; each after a
 * second on the clock, nothing leaked. A bad model or NULLs, -EINVAL. */
static void test_init_errors(void)
{
	ld19_config_t cfg = cfg_of(LD19_MODEL_LD19);
	ld19_handle_t h = NULL;
	uint8_t junk[300];
	int64_t t0;

	line_reset();
	allocs = 0;
	t0 = now_us();
	assert(ld19_init(&cfg, &h) == RET(-ENXIO));
	assert(now_us() - t0 >= 1000000);
	assert(strstr(stub_log_last, "check the 5 V supply"));
	assert(allocs == 0 && uart_live() == 0 && !h);

	line_reset();
	memset(junk, 0x11, sizeof junk);
	put_bytes(junk, sizeof junk);
	assert(ld19_init(&cfg, &h) == RET(-ENODEV));
	assert(strstr(stub_log_last, "check the baud rate"));
	assert(allocs == 0 && uart_live() == 0);

	cfg.model = (ld19_model_t)2;
	assert(ld19_init(&cfg, &h) == RET(-EINVAL));
	assert(ld19_init(NULL, &h) == RET(-EINVAL));
	cfg = cfg_of(LD19_MODEL_LD19);
	assert(ld19_init(&cfg, NULL) == RET(-EINVAL));
	calloc_fails = true;
	assert(ld19_init(&cfg, &h) == RET(-ENOMEM));
	calloc_fails = false;
	assert(allocs == 0 && uart_live() == 0 && !h);
}

/* Packets: resync, CRC, the fields, and the manual's own example: a packet
 * with the span check off, as by default, and dropped whole with it on. */
static void test_read_packet(void)
{
	/* MAN 3.3, p.11: valid, but 10 degrees at 2152 deg/s is too wide for
	 * the SDK's span check. */
	static const uint8_t example[LD19_PACKET_LEN] = {
		0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E, 0xE0, 0x00, 0xE4, 0xDC,
		0x00, 0xE2, 0xD9, 0x00, 0xE5, 0xD5, 0x00, 0xE3, 0xD3, 0x00,
		0xE4, 0xD0, 0x00, 0xE9, 0xCD, 0x00, 0xE4, 0xCA, 0x00, 0xE2,
		0xC7, 0x00, 0xE9, 0xC5, 0x00, 0xE5, 0xC2, 0x00, 0xE5, 0xC0,
		0x00, 0xE5, 0xBE, 0x82, 0x3A, 0x1A, 0x50,
	};
	uint8_t raw[LD19_PACKET_LEN];
	ld19_packet_t p;
	ld19_handle_t h;

	line_reset();
	send_packet(0, 500);
	h = open_ld19(LD19_MODEL_LD19);

	/* A packet whose header sits inside garbage, half of it, then all. */
	put_bytes((const uint8_t *)"\x54\x2C\x00", 3);
	packet(raw, 3600, 9000, 9825, 2000, 4321);
	put_bytes(raw, sizeof raw);
	assert(ld19_read_packet(h, &p, 100) == RET(0));
	assert(p.points[0].distance_mm == 2000 && p.points[11].intensity == 111);
	assert(p.points[0].angle_deg == 90.0f && p.points[11].angle_deg == 98.25f);
	assert(p.speed_dps == 3600.0f && p.timestamp_ms == 4321);

	/* The span check is off by default: the example is a packet. */
	put_bytes(example, sizeof example);
	send_packet(18000, 3000);
	assert(ld19_read_packet(h, &p, 100) == RET(0));
	assert(p.points[0].distance_mm == 224 && p.speed_dps == 2152.0f);
	assert(ld19_read_packet(h, &p, 100) == RET(0));
	assert(p.points[0].distance_mm == 3000);

	/* Nothing more: a timeout. */
	assert(ld19_read_packet(h, &p, 50) == RET(-ETIMEDOUT));
	assert(ld19_read_packet(NULL, &p, 50) == RET(-EINVAL));
	ld19_del(h);

	/* With cfg.span_check the example is dropped whole -- none of its 47
	 * bytes starts a packet again -- and the next one read. */
	{
		ld19_config_t cfg = cfg_of(LD19_MODEL_LD19);

		cfg.span_check = true;
		line_reset();
		send_packet(0, 500);
		assert(ld19_init(&cfg, &h) == RET(0));
	}
	put_bytes(example, sizeof example);
	send_packet(18000, 3000);
	assert(ld19_read_packet(h, &p, 100) == RET(0));
	assert(p.points[0].distance_mm == 3000);
	ld19_del(h);
}

/*
 * The deadline is on the clock: a packet 3 ms away is caught by a 5 ms
 * timeout -- which at IDF's 100 Hz tick esp-ld19-lidar's pdMS_TO_TICKS(5)
 * made no wait at all -- and one 80 ms away is missed by a 50 ms timeout,
 * but only once 50 ms have passed.
 */
static void test_deadline(void)
{
	ld19_packet_t p;
	ld19_handle_t h;
	int64_t t0;

	line_reset();
	send_packet(0, 500);
	h = open_ld19(LD19_MODEL_LD19);

	packet(late, 3600, 0, 825, 700, 0);
	late_len = LD19_PACKET_LEN;
	late_at_us = now_us() + 3000;
	assert(ld19_read_packet(h, &p, 5) == RET(0));
	assert(p.points[0].distance_mm == 700);

	packet(late, 3600, 0, 825, 800, 0);
	late_len = LD19_PACKET_LEN;
	late_at_us = now_us() + 80000;
	t0 = now_us();
	assert(ld19_read_packet(h, &p, 50) == RET(-ETIMEDOUT));
	assert(now_us() - t0 >= 50000);
	ld19_del(h);
}

/* Turns through the handle: the first found, the second carried. */
static void test_read_scan(void)
{
	static ld19_point_t pts[LD19_SCAN_MAX_POINTS];
	ld19_handle_t h;
	size_t count = 99;

	line_reset();
	send_packet(0, 500);
	h = open_ld19(LD19_MODEL_LD19);
	send_turns(3);
	assert(ld19_read_scan(h, pts, LD19_SCAN_MAX_POINTS, &count, 1000) ==
	       RET(0));
	assert(count == 480 && pts[0].angle_deg == 0.0f &&
	       pts[479].angle_deg == 359.25f);
	assert(pts[6].distance_mm == 1000);	/* packet 0, point 0 */
	assert(ld19_read_scan(h, pts, LD19_SCAN_MAX_POINTS, &count, 1000) ==
	       RET(0));
	assert(count == 480 && pts[0].angle_deg == 0.0f);

	/* Capacity: the rest of the turn dropped, not an error. */
	send_turns(1);
	assert(ld19_read_scan(h, pts, 100, &count, 1000) == RET(0));
	assert(count == 100);

	assert(ld19_read_scan(h, pts, 0, &count, 10) == RET(-EINVAL));
	assert(ld19_read_scan(h, NULL, 10, &count, 10) == RET(-EINVAL));
	ld19_del(h);
}

/* The category view: the same turns, flagged, stamped and filtered. */
static void test_lidar_view(void)
{
	static struct lidar_point pts[600];
	struct lidar_scan s = { .points = pts, .capacity = 600 };
	struct lidar_packet pkt;
	ld19_handle_t h;
	struct lidar_dev *l;

	assert(ld19_lidar(NULL) == NULL);
	line_reset();
	send_packet(0, 500);
	h = open_ld19(LD19_MODEL_LD06);
	l = ld19_lidar(h);

	send_turns(2);
	assert(lidar_read_scan(l, &s, 1000) == 0);
	assert(s.count == 480 && !s.truncated && s.speed_dps == 3600.0f);

	send_packet(0, 900);
	assert(lidar_read_packet(l, &pkt, 100) == 0);
	assert(pkt.count == 12 && pkt.points[0].distance_mm == 900);
	assert(pkt.sensor_ms == 7 && pkt.t_us == now_us());

	/* Opt-in filter on a real turn: points 1000+ mm, intensity 100+,
	 * one group per packet of 12: kept. */
	lidar_filter_near(&s);
	for (size_t i = 0; i < s.count; i++)
		assert(pts[i].distance_mm);
	ld19_del(h);
}

static void run_ld19_cases(void)
{
	test_init();
	test_init_errors();
	test_read_packet();
	test_deadline();
	test_read_scan();
	test_lidar_view();
}
