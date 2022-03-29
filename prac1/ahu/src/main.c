/**
 * @file main.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au) - s4427634
 * @brief Weather Station - Application Host Unit
 * @version 0.2
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "led_driver.h"
#include "shell_base.h"
#include "shell_time.h"
#include "shell_led.h"
#include "shell_scu.h"
#include "ble_base.h"
#include "pb_driver.h"
#include "ahu_data.h"

// Debug Settings ==============================================================
#define DEBUG_BLE_LED 0

// Logging Module ==============================================================
LOG_MODULE_REGISTER(log_main);

// Functions ===================================================================

/**
 * @brief Initialises the hardware and shell
 *
 */
void initialise(void)
{

        init_leds();
        setup_pb();
        begin_shell();
}

void main(void)
{

        initialise();
}

// Thread Defines ==============================================================

// START BLE BASE entry thread : Delayed Start (Wait for USB to be ready)
K_THREAD_DEFINE(ble_base, THREAD_BLE_BASE_STACK, thread_ble_base, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(rx_data, THREAD_BLE_BASE_STACK, process_rx_data, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(ahu_data_thread, THREAD_BLE_BASE_STACK, JSON_thread, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
// Start BLE LED Thread
#if DEBUG_BLE_LED
K_THREAD_DEFINE(ble_led, THREAD_BLE_LED_STACK, thread_ble_led, NULL, NULL, NULL, THREAD_PRIORITY_BLE_LED, 0, 0);
#endif

// Shell Commands ==============================================================

// Level 2 //
SHELL_STATIC_SUBCMD_SET_CREATE(hts221_read,
                               SHELL_CMD(t, NULL, "Read the temperature", cmd_hts221_read_temp),
                               SHELL_CMD(h, NULL, "Read the humidity", cmd_hts221_read_hum),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(lis2dh_read,
                               SHELL_CMD(x, NULL, "Read the X-Axis", cmd_lis2dh_read_x),
                               SHELL_CMD(y, NULL, "Read the Y-Axis", cmd_lis2dh_read_y),
                               SHELL_CMD(z, NULL, "Read the Z-Axis", cmd_lis2dh_read_z),
                               SHELL_SUBCMD_SET_END);

// Level 1 //
SHELL_STATIC_SUBCMD_SET_CREATE(time_read,
                               SHELL_CMD(f, NULL, "Show the time formatted as hours, minutes, seconds..", cmd_disp_format_time),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(hts221_readwrite,
                               SHELL_CMD(r, &hts221_read, "Read", NULL),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(lps22_readwrite,
                               SHELL_CMD(r, NULL, "Read Pressure", cmd_lps22_read_pres),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(ccs811_readwrite,
                               SHELL_CMD(r, NULL, "Read VOC", cmd_ccs811_read_voc),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(lis2dh_readwrite,
                               SHELL_CMD(r, &lis2dh_read, "Read", NULL),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(buzzer_readwrite,
                               SHELL_CMD(w, NULL, "Write Frequency <freq>", cmd_buzzer_write_freq),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(rgb_readwrite,
                               SHELL_CMD(w, NULL, "Write <R, G, B>", cmd_rgb_write_rgb),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(pb_readwrite,
                               SHELL_CMD(r, NULL, "Read", cmd_pb_read_state),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(dc_readwrite,
                               SHELL_CMD(w, NULL, "Write the duty cycle <percentage on>", cmd_dc_write_percentage),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sample_readwrite,
                               SHELL_CMD(w, NULL, "Write the sampling time <sec>", cmd_sample_write_sec),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(all_control,
                               SHELL_CMD(o, NULL, "Start reading all sensors", cmd_all_on),
                               SHELL_CMD(f, NULL, "Stop reading all sensors", cmd_all_off),
                               SHELL_SUBCMD_SET_END);

// Level 0 //
SHELL_CMD_REGISTER(time, &time_read, "Display elapsed time since power on", cmd_disp_time);
SHELL_CMD_REGISTER(led, NULL, "Control the AHUs LEDs", cmd_led_control);

SHELL_CMD_REGISTER(hts221, &hts221_readwrite, "Temp/Humidity", NULL);
SHELL_CMD_REGISTER(lps22, &lps22_readwrite, "Pressure", NULL);
SHELL_CMD_REGISTER(ccs811, &ccs811_readwrite, "VOC", NULL);
SHELL_CMD_REGISTER(lis2dh, &lis2dh_readwrite, "XYZ Acceleration", NULL);
SHELL_CMD_REGISTER(buzzer, &buzzer_readwrite, "buzzer frequency", NULL);
SHELL_CMD_REGISTER(rgb, &rgb_readwrite, "RGB LED", NULL);
SHELL_CMD_REGISTER(pb, &pb_readwrite, "Pushbutton state (0 or 1)", NULL);
SHELL_CMD_REGISTER(dc, &dc_readwrite, "Duty Cycle", NULL);
SHELL_CMD_REGISTER(sample, &sample_readwrite, "Sample Time", NULL);
SHELL_CMD_REGISTER(all, &all_control, "Toggle Reading All Devices", NULL);
