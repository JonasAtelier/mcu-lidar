/*
 * Host-side check of src/ld19/ld19_proto.h against the LD19 Development
 * Manual V2.3 (docs/ld19_v2.3.pdf, PDF pages): the CRC against all 256
 * entries of the manual's table (p.8-9, the same in the SDK's
 * lipkg.cpp:26-48), the manual's example packet (3.3, p.11) byte for byte,
 * the angle interpolation and its wrap, and the SDK's span check. No bus,
 * no platform.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ld19_proto.h"

#define NEAR(a, b) (fabs((double)(a) - (double)(b)) < 1e-3)

/* MAN p.8-9, as printed. */
static const uint8_t manual_table[256] = {
	0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
	0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
	0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
	0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
	0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
	0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
	0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
	0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
	0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
	0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
	0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
	0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
	0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
	0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
	0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
	0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
	0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
	0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
	0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
	0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
	0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
	0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
	0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
	0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
	0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
	0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
	0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
	0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
	0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
	0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
	0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
	0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
};

/* MAN 3.3, p.11: the example packet, as printed. */
static const uint8_t example[LD19_PACKET_LEN] = {
	0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E, 0xE0, 0x00, 0xE4, 0xDC, 0x00, 0xE2,
	0xD9, 0x00, 0xE5, 0xD5, 0x00, 0xE3, 0xD3, 0x00, 0xE4, 0xD0, 0x00, 0xE9,
	0xCD, 0x00, 0xE4, 0xCA, 0x00, 0xE2, 0xC7, 0x00, 0xE9, 0xC5, 0x00, 0xE5,
	0xC2, 0x00, 0xE5, 0xC0, 0x00, 0xE5, 0xBE, 0x82, 0x3A, 0x1A, 0x50,
};

/* A packet with 12 points at i*100 mm and intensity 200+i, sealed. */
static void build(uint8_t raw[LD19_PACKET_LEN], uint16_t speed,
		  uint16_t start, uint16_t end, uint16_t ts)
{
	memset(raw, 0, LD19_PACKET_LEN);
	raw[0] = LD19_HEADER;
	raw[1] = LD19_VERLEN;
	raw[2] = (uint8_t)speed;
	raw[3] = (uint8_t)(speed >> 8);
	raw[4] = (uint8_t)start;
	raw[5] = (uint8_t)(start >> 8);
	for (int i = 0; i < LD19_POINTS; i++) {
		uint16_t d = (uint16_t)(i * 100);

		raw[6 + 3 * i] = (uint8_t)d;
		raw[7 + 3 * i] = (uint8_t)(d >> 8);
		raw[8 + 3 * i] = (uint8_t)(200 + i);
	}
	raw[42] = (uint8_t)end;
	raw[43] = (uint8_t)(end >> 8);
	raw[44] = (uint8_t)ts;
	raw[45] = (uint8_t)(ts >> 8);
	raw[46] = ld19_crc8(raw, 46);
}

/* A table CRC's entry i is the CRC of the one byte i. */
static void test_crc(void)
{
	for (unsigned i = 0; i < 256; i++) {
		uint8_t b = (uint8_t)i;

		assert(ld19_crc8(&b, 1) == manual_table[i]);
	}
	assert(ld19_crc8(NULL, 0) == 0);
	/* The manual's CalCRC8() over the example: 0x50, its last byte. */
	{
		uint8_t crc = 0;

		for (int i = 0; i < 46; i++)
			crc = manual_table[(crc ^ example[i]) & 0xff];
		assert(crc == 0x50 && ld19_crc8(example, 46) == 0x50);
	}
}

/* The fields as the manual's table reads them (p.11), where it agrees with
 * its own hex: its point-2 distance says 200 mm for 00DC (220), and its
 * point-12 row (B0 00 EA) is not in the packet (C0 00 E5). */
static void test_example_fields(void)
{
	assert(ld19_le16(example + LD19_SPEED) == 2152);
	assert(ld19_le16(example + LD19_START) == 32427);
	assert(ld19_le16(example + LD19_END) == 33470);
	assert(ld19_le16(example + LD19_TIMESTAMP) == 0x1A3A);
	assert(ld19_le16(example + LD19_DATA) == 224 && example[8] == 228);
	assert(ld19_le16(example + LD19_DATA + 3) == 220 && example[11] == 226);
	assert(ld19_le16(example + LD19_DATA + 33) == 192 && example[41] == 229);
}

/*
 * The SDK's span check (lipkg.cpp:151-152) drops the manual's own example:
 * it spans 10 whole degrees, and 2152 deg/s allows 2152 x 12 / 4500 x 1.5 =
 * 8.6. So the check is opt-in: off, the example is a packet like any other.
 */
static void test_example_span(void)
{
	struct lidar_packet pkt;

	assert(!ld19_span_ok(32427, 33470, 2152));
	assert(ld19_parse(example, &pkt, true) == -ERANGE);
	/* The same angles at 10 Hz pass: 14.4 degrees allowed. */
	assert(ld19_span_ok(32427, 33470, 3600));

	memset(&pkt, 0, sizeof(pkt));
	assert(ld19_parse(example, &pkt, false) == 0);
	assert(pkt.speed_dps == 2152.0f && pkt.sensor_ms == 0x1A3A);
	assert(pkt.points[0].distance_mm == 224 && pkt.points[0].intensity == 228);
	assert(NEAR(pkt.points[0].angle_deg, 324.27));
	assert(NEAR(pkt.points[11].angle_deg, 324.27 + 11 * 0.94));
}

static void test_decode(void)
{
	uint8_t raw[LD19_PACKET_LEN];
	struct lidar_packet pkt;

	/* 1100 hundredths over 11 gaps: one degree a point. */
	build(raw, 3600, 0, 1100, 1234);
	assert(ld19_parse(raw, &pkt, true) == 0 && pkt.count == 12);
	for (int i = 0; i < 12; i++) {
		assert(NEAR(pkt.points[i].angle_deg, i));
		assert(pkt.points[i].distance_mm == i * 100);
		assert(pkt.points[i].intensity == 200 + i);
	}
	assert(pkt.speed_dps == 3600.0f && pkt.sensor_ms == 1234);

	/* The step is whole hundredths, truncated (lipkg.cpp:165-166):
	 * 1043 / 11 = 94, so point 11 is 324.27 + 10.34. */
	build(raw, 3600, 32427, 33470, 0);
	assert(ld19_parse(raw, &pkt, true) == 0);
	assert(NEAR(pkt.points[1].angle_deg, 325.21));
	assert(NEAR(pkt.points[11].angle_deg, 334.61));

	/* Across 360: 359.90 to 1.00, wrapped, in range. */
	build(raw, 3600, 35990, 100, 0);
	assert(ld19_parse(raw, &pkt, true) == 0);
	assert(NEAR(pkt.points[0].angle_deg, 359.90));
	assert(NEAR(pkt.points[1].angle_deg, 0.0));
	assert(NEAR(pkt.points[11].angle_deg, 1.0));
	for (int i = 0; i < 12; i++)
		assert(pkt.points[i].angle_deg >= 0 &&
		       pkt.points[i].angle_deg < 360);
}

static void test_rejects(void)
{
	uint8_t raw[LD19_PACKET_LEN];
	struct lidar_packet pkt;

	build(raw, 3600, 0, 1100, 0);
	raw[0] = 0x55;
	raw[46] = ld19_crc8(raw, 46);
	assert(ld19_parse(raw, &pkt, true) == -EBADMSG);
	build(raw, 3600, 0, 1100, 0);
	raw[1] = 0x2D;
	raw[46] = ld19_crc8(raw, 46);
	assert(ld19_parse(raw, &pkt, true) == -EBADMSG);
	build(raw, 3600, 0, 1100, 0);
	raw[10] ^= 0x01;
	assert(ld19_parse(raw, &pkt, true) == -EBADMSG);

	/* Span: 14 whole degrees allowed at 3600 deg/s (14.4); 15 not. A
	 * rotor at rest allows none. */
	build(raw, 3600, 0, 1400, 0);
	assert(ld19_parse(raw, &pkt, true) == 0);
	build(raw, 3600, 0, 1500, 0);
	assert(ld19_parse(raw, &pkt, true) == -ERANGE);
	build(raw, 0, 0, 100, 0);
	assert(ld19_parse(raw, &pkt, true) == -ERANGE);
	build(raw, 0, 0, 99, 0);
	assert(ld19_parse(raw, &pkt, true) == 0);
	/* Off, none of that is checked. */
	build(raw, 0, 0, 1500, 0);
	assert(ld19_parse(raw, &pkt, false) == 0);
	/* Exactly at the limit passes: 3750 deg/s allows exactly 15. */
	assert(ld19_span_ok(0, 1500, 3750) && !ld19_span_ok(0, 1600, 3750));
}

int main(void)
{
	test_crc();
	test_example_fields();
	test_example_span();
	test_decode();
	test_rejects();
	printf("test_ld19_proto: all passed\n");
	return 0;
}
