# ESP-IDF only. stm-forge never reads this file: it compiles every .c under
# src/, chip folders included, puts src/ on the include path of this
# library's sources and adds include/ -- which is why each port file is empty
# on the other platform. Here the chip files are listed one by one, and
# PRIV_INCLUDE_DIRS "src" lets a chip folder reach lidar_priv.h by plain name.
#
# The core is always built, even with CONFIG_LIDAR off; each chip driver only
# when its Kconfig option is on. Kconfig values are available here during
# the build phase, and only SRCS depends on them -- never REQUIRES, which IDF
# resolves before sdkconfig. That is also why LIDAR off still builds the core:
# an empty SRCS would need a second, differently-shaped register call, and an
# unreferenced core costs nothing once --gc-sections has run.
#
# REQUIRES driver, not esp_driver_i2c/uart/gpio: the split components do not
# exist on IDF 5.2, and driver still pulls them in on later versions.
# esp_timer is lidar_time_us()'s clock.
set(srcs "src/lidar.c" "src/lidar_log.c" "src/lidar_filter.c"
         "src/lidar_port_esp.c")

if(CONFIG_LIDAR AND NOT (CONFIG_LIDAR_LD19))
    message(WARNING "lidar: CONFIG_LIDAR is on but no chip driver is selected; "
                    "set CONFIG_LIDAR_<CHIP>=y (esp-forge: LIDAR_<CHIP> = y)")
endif()

if(CONFIG_LIDAR_LD19)
    list(APPEND srcs "src/ld19/ld19.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES driver esp_timer)
