# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(peripheral)

include_directories(
                        ../../oslib/scu_drivers/scu_sensors
                        ../../oslib/scu_drivers/scu_ble
                        ../../oslib/scu_drivers/scu_power_management
                        ../../oslib/common/
                        )

target_sources(app PRIVATE
  src/main.c
  ../../oslib/scu_drivers/scu_sensors/scu_sensors.c
  ../../oslib/scu_drivers/scu_ble/scu_ble.c
  ../../oslib/scu_drivers/scu_power_management/scu_power_management.c
  ../../oslib/common/hci_driver.c
)
