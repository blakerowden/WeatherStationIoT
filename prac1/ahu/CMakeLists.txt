# SPDX-License-Identifier: Apache-2.0

set(BOARD nrf52840dongle_nrf52840)
set(CONF_FILE prj.conf usb.conf shell.conf)
set(DTC_OVERLAY_FILE dtc_shell.overlay)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ahu)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

#Add Include Directories
include_directories(
                        ../../oslib/ahu_drivers/ahu_ble
                        ../../oslib/ahu_drivers/ahu_shell
                        ../../oslib/common
                        )

#Add Target Sources
target_sources(app PRIVATE
                        src/main.c
                        ../../oslib/ahu_drivers/ahu_ble/ahu_ble.c
                        ../../oslib/ahu_drivers/ahu_shell/shell_base.c
                        ../../oslib/ahu_drivers/ahu_shell/shell_led.c
                        ../../oslib/ahu_drivers/ahu_shell/shell_time.c
                        ../../oslib/common/led_driver.c
                        )
