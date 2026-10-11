/*
 * An LD06 through the category view: one line a second with the turn's
 * points, the rotor speed and the nearest return, after LDROBOT's near
 * filter has zeroed the speckle.
 *
 * Wire the lidar's TX to the GPIO below, its 5 V and GND to a supply that
 * can give it the rotor's start-up current, and its PWM pin, if it has one,
 * to ground (see docs/hardware.md).
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ld19.h"
#include "lidar.h"
#include "lidar_log.h"

#define LIDAR_UART      UART_NUM_1
#define LIDAR_RX_GPIO   16
#define SCAN_TIMEOUT_MS 300     /* a turn takes 100 ms at the nominal 10 Hz */
#define SCANS_PER_LOG   10

void app_main(void)
{
    /* 600 points is 4.8 KB: static, not on the task's stack. */
    static struct lidar_point points[LD19_SCAN_MAX_POINTS];
    struct lidar_scan scan = {
        .points = points,
        .capacity = LD19_SCAN_MAX_POINTS,
    };
    ld19_config_t cfg = {
        .uart    = LIDAR_UART,
        .rx_gpio = LIDAR_RX_GPIO,
        .model   = LD19_MODEL_LD06,
    };
    ld19_handle_t handle;
    struct lidar_dev *lidar;
    int scans = 0;

    ESP_ERROR_CHECK(ld19_init(&cfg, &handle));
    lidar = ld19_lidar(handle);

    while (1) {
        int err = lidar_read_scan(lidar, &scan, SCAN_TIMEOUT_MS);

        if (err) {
            ESP_LOGW("ld06-example", "no scan (%d)", err);
            continue;
        }
        lidar_filter_near(&scan);   /* opt-in; sorts by angle */
        if (++scans % SCANS_PER_LOG == 0)
            lidar_log_scan(lidar, &scan);
    }
}
