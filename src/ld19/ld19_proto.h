/*
 * The LD19 and LD06 wire format: one 47-byte packet, its CRC, its angles and
 * its plausibility check. Pure, so it compiles and is asserted on the host.
 *
 * Sources, cited as below:
 *   MAN  LDROBOT "LD19 Development Manual" V2.3 (Elecrow), docs/ld19_v2.3.pdf;
 *        page numbers are the PDF's. V2.5 (DFRobot, docs/ld19_v2.5.pdf)
 *        agrees on every value here.
 *   SDK  LDROBOT's ldlidar_stl_ros, commit 964afad8, MIT, file:line under
 *        ldlidar_driver/ unless named. It parses the LD06 and the LD19 with
 *        the same code and no model branch: lipkg.cpp:78-186.
 */
#ifndef LD19_PROTO_H
#define LD19_PROTO_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lidar.h"

/*
 * 230400 8N1, no flow control, one way (MAN 2, p.6). The LD06 the same:
 * the SDK's launch files give port_baudrate 230400 for each
 * (launch/ld06.launch:6, launch/ld19.launch:6).
 */
#define LD19_BAUD		230400

/* 4500 measurements a second (MAN 1, p.3), for both (lipkg.cpp:81-84). */
#define LD19_RATE_HZ		4500

/*
 * MAN 3.1, p.7-8; SDK include/dataprocess/lipkg.h:37-57: header,
 * VerLen, speed, start angle, 12 x (distance, intensity), end angle,
 * timestamp, CRC. Every 2-byte field low byte first (MAN 3.2, p.10).
 */
#define LD19_PACKET_LEN		47
#define LD19_HEADER		0x54	/* MAN p.7; lipkg.h:38 */
/* Upper three bits the packet type, 1; lower five the points, 12: 0x2C
 * (MAN p.7; lipkg.h:39). */
#define LD19_VERLEN		0x2C
#define LD19_POINTS		12	/* lipkg.h:40 */
#define LD19_SPEED		2	/* deg/s */
#define LD19_START		4	/* 0.01 deg */
#define LD19_DATA		6	/* distance mm (2), intensity (1) each */
#define LD19_END		42	/* 0.01 deg */
#define LD19_TIMESTAMP		44	/* ms, 0..29999 (MAN p.7) */
#define LD19_CRC		46

#define LD19_TURN_CD		36000	/* hundredths of a degree */

static inline uint16_t ld19_le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

/*
 * CRC-8 over every byte before the CRC (MAN p.8; lipkg.cpp:129). MAN
 * p.8-9 and lipkg.cpp:26-48 give the same 256-entry table, and the table is
 * this: polynomial 0x4D, initial 0, MSB first, no reflection, no final
 * xor. test_ld19_proto.c holds it to all 256 entries of the manual's.
 */
static inline uint8_t ld19_crc8(const uint8_t *p, size_t n)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < n; i++) {
		crc ^= p[i];
		for (int bit = 0; bit < 8; bit++)
			crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x4Du)
					    : (uint8_t)(crc << 1);
	}
	return crc;
}

/*
 * The SDK's plausibility check (lipkg.cpp:151-152), opt-in here because
 * the manual's own example packet fails it (see docs/chips.md): the packet
 * spans, in
 * whole degrees, end / 100 - start / 100 (mod 360), and that must not be
 * more than speed x 12 / 4500 x 1.5 degrees -- half as much again as twelve
 * measurements can turn at that speed. Worked here in integers, exactly:
 * span x 4500 x 2 <= speed x 12 x 3. The SDK compares in double. The
 * angles come in already taken mod 36000.
 */
static inline bool ld19_span_ok(uint32_t start_cd, uint32_t end_cd,
				uint16_t speed_dps)
{
	uint32_t span = (end_cd / 100 + 360 - start_cd / 100) % 360;

	return span * LD19_RATE_HZ * 2 <= (uint32_t)speed_dps * LD19_POINTS * 3;
}

/*
 * Validate and decode one packet into pkt (not t_us). -EBADMSG: not a
 * packet -- header, VerLen or CRC wrong. -ERANGE, only with span_check: a
 * packet, CRC and all, whose angles span more than its speed allows; the
 * SDK drops those.
 *
 * Angles: the first point is at the start angle and the last at the end
 * angle, the rest evenly between (MAN 3.2, p.10). The step is
 * ((end - start) mod 36000) / 11 in whole hundredths, as the SDK takes it
 * (lipkg.cpp:165-166); the manual's own formula has no wrap and no types.
 */
static inline int ld19_parse(const uint8_t raw[LD19_PACKET_LEN],
			     struct lidar_packet *pkt, bool span_check)
{
	uint32_t start, end, step;

	if (raw[0] != LD19_HEADER || raw[1] != LD19_VERLEN ||
	    raw[LD19_CRC] != ld19_crc8(raw, LD19_PACKET_LEN - 1))
		return -EBADMSG;

	start = ld19_le16(raw + LD19_START) % LD19_TURN_CD;
	end = ld19_le16(raw + LD19_END) % LD19_TURN_CD;
	if (span_check &&
	    !ld19_span_ok(start, end, ld19_le16(raw + LD19_SPEED)))
		return -ERANGE;
	step = (end + LD19_TURN_CD - start) % LD19_TURN_CD / (LD19_POINTS - 1);

	for (int i = 0; i < LD19_POINTS; i++) {
		const uint8_t *m = raw + LD19_DATA + 3 * i;

		pkt->points[i].angle_deg =
			(float)((start + step * (uint32_t)i) % LD19_TURN_CD) /
			100.0f;
		pkt->points[i].distance_mm = ld19_le16(m);
		pkt->points[i].intensity = m[2];
	}
	pkt->count = LD19_POINTS;
	pkt->speed_dps = (float)ld19_le16(raw + LD19_SPEED);
	pkt->sensor_ms = ld19_le16(raw + LD19_TIMESTAMP);
	return 0;
}

#endif /* LD19_PROTO_H */
