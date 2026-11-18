# Read by ESP-IDF (and so esp-forge) as a component, and by any other CMake
# project that add_subdirectory()s this repo, such as a CubeMX-generated STM32
# one. stm-forge never reads it: it compiles every .c under src/, chip folders
# included, puts src/ on the include path of this library's sources and adds
# include/ -- which is why each port file is empty on the other platform.
# ESP_PLATFORM tells the two apart; IDF sets it in its early requirements pass
# as well as in the build.

if(NOT ESP_PLATFORM)
    # A static library `lidar`. Every .c under src/, as stm-forge does: a
    # chip file compiles to nothing unless its CONFIG_LIDAR_<CHIP> is 1, so
    # there is no source list here to keep in step with the IDF one below.
    file(GLOB_RECURSE lidar_sources CONFIGURE_DEPENDS src/*.c)
    add_library(lidar STATIC ${lidar_sources})
    # include/ for the app; src/ only for this library's own sources, so a
    # chip folder reaches lidar_priv.h by plain name and no private header
    # is ever on the app's include path.
    target_include_directories(lidar PUBLIC include PRIVATE src)

    # The options are the Kconfig bools, read from Kconfig so a new chip needs
    # no line here: a true CMake variable CONFIG_LIDAR_<CHIP> (or
    # CONFIG_LIDAR_DEBUG) becomes CONFIG_LIDAR_<CHIP>=1, what stm-forge
    # passes. Only bools: an int option cannot be 1-or-absent. Each `config`
    # line in Kconfig is followed directly by its type line, which is what the
    # match relies on. PUBLIC, because lidar_log.h reads CONFIG_LIDAR_DEBUG in
    # the app's sources.
    file(READ Kconfig lidar_kconfig)
    string(REGEX MATCHALL "config LIDAR_[A-Z0-9_]+[ \t\r\n]+bool"
           lidar_options "${lidar_kconfig}")
    list(TRANSFORM lidar_options REPLACE
         "config (LIDAR_[A-Z0-9_]+)[ \t\r\n]+bool" "CONFIG_\\1")
    set(lidar_chips_on "")
    foreach(option IN LISTS lidar_options)
        if(${option})
            target_compile_definitions(lidar PUBLIC ${option}=1)
            if(NOT option STREQUAL "CONFIG_LIDAR_DEBUG")
                list(APPEND lidar_chips_on ${option})
            endif()
        endif()
    endforeach()
    if(NOT lidar_chips_on)
        message(WARNING "lidar: no chip driver is selected; "
                        "set(CONFIG_LIDAR_<CHIP> ON) before add_subdirectory()")
    endif()

    # The HAL is the app's. MCU_HAL_TARGET names a target carrying its include
    # dirs and defines (USE_HAL_DRIVER, the device, the hal_conf.h directory)
    # -- stm32cubemx in a CubeMX-generated project; MCU_DEVICE_HEADER is the
    # family header the public headers include as CMSIS_device_header, which
    # CubeMX does not define. Both are shared by every mcu-* library, so an
    # app sets them once. Left unset, the app provides them its own way.
    #
    # Only the target's include dirs and defines are copied, the target is
    # never linked: CubeMX up to 6.13 also hangs main.c, the HAL and the
    # startup file on stm32cubemx as INTERFACE sources, which linking would
    # compile into liblidar.a. A misspelt name fails at generate time.
    if(MCU_HAL_TARGET)
        target_include_directories(lidar PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_INCLUDE_DIRECTORIES>)
        target_compile_definitions(lidar PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_COMPILE_DEFINITIONS>)
    endif()
    if(MCU_DEVICE_HEADER)
        target_compile_definitions(lidar PUBLIC
            "CMSIS_device_header=\"${MCU_DEVICE_HEADER}\"")
    endif()
    return()
endif()

# ESP-IDF: the chip files are listed one by one, and PRIV_INCLUDE_DIRS "src"
# lets a chip folder reach lidar_priv.h by plain name.
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
