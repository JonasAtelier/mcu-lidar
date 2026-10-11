/*
 * Print one line a second: how many points came back in the last turn, the
 * nearest thing the lidar can see, and how fast the rotor is going.
 *
 * Wire the lidar's TX to the GPIO below, its 5 V and GND to a supply that can
 * give it 300 mA, and tie its PWM pin to ground so the module runs its own
 * motor -- the manual says it must be grounded when not driven (LD19
 * Development Manual V2.3, section 2, p.6).
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ld19.h"

#define LIDAR_UART      UART_NUM_1
#define LIDAR_RX_GPIO   16
#define SCAN_TIMEOUT_MS 300     /* a turn takes 100 ms at the nominal 10 Hz */
#define SCANS_PER_LOG   10      /* ...so ten of them is about one second */

static const char *TAG = "ld19-example";

void app_main(void)
{
    /* 600 points is 4.8 KB. Static, because that is more than a task stack
     * wants to carry. */
    static ld19_point_t points[LD19_SCAN_MAX_POINTS];
    ld19_config_t cfg = {
        .uart    = LIDAR_UART,
        .rx_gpio = LIDAR_RX_GPIO,
    };
    ld19_handle_t lidar;
    ld19_packet_t pkt;
    int scans = 0;

    ESP_ERROR_CHECK(ld19_init(&cfg, &lidar));

    /* The rotor speed only lives in a packet, and taking one mid-scan would
     * steal points from that turn. Read it once, here, before the loop. */
    if (ld19_read_packet(lidar, &pkt, SCAN_TIMEOUT_MS) == ESP_OK)
        ESP_LOGI(TAG, "rotor at %.0f deg/s", (double)pkt.speed_dps);

    while (1) {
        ld19_point_t nearest = { 0 };
        size_t count = 0;
        esp_err_t err;

        err = ld19_read_scan(lidar, points, LD19_SCAN_MAX_POINTS, &count,
                             SCAN_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "no scan: %s", esp_err_to_name(err));
            continue;
        }

        if (++scans < SCANS_PER_LOG)
            continue;

        scans = 0;

        for (size_t i = 0; i < count; i++) {
            /* Zero means the beam brought nothing back: glass, a mirror, or
             * open space past 12 m. */
            if (points[i].distance_mm == 0)
                continue;

            if (nearest.distance_mm == 0 ||
                points[i].distance_mm < nearest.distance_mm)
                nearest = points[i];
        }

        ESP_LOGI(TAG, "%u points, nearest %u mm at %.1f deg",
                 (unsigned)count, nearest.distance_mm,
                 (double)nearest.angle_deg);
    }
}
