/**
 * @file main.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au) - s4427634
 * @brief Weather Station - Application Host Unit
 * @version 0.1
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
#include "ble_base.h"


// Debug Define Fkags ==========================================================
#define DEBUG_BLE 0x01
#define DEBUG_SHELL 0x02
#define DEBUG_LED 0x04
#define DEBUG_ALL 0xFF

// Set the flags for debugging
#define DEBUG_LEVEL (DEBUG_BLE | DEBUG_SHELL | DEBUG_LED)

// Logging Module
 LOG_MODULE_REGISTER(main);

/**
 * @brief Initialises the hardware and shell
 * 
 */
void main(void) {

	init_leds();
	begin_shell();
    
}

// Thread Defines ==============================================================

//START BLE BASE entry thread : Delayed Start (Wait for USB to be ready)
K_THREAD_DEFINE(ble_base, THREAD_BLE_BASE_STACK, thread_ble_base, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
#if (DEBUG_LEVEL & DEBUG_SHELL)
//Reads BLE Buffers and prints them to USB Console
K_THREAD_DEFINE(ble_read_out, THREAD_BLE_BASE_STACK, thread_ble_read_out, NULL, NULL, NULL, THREAD_PRIORITY_READ_BASE, 0, 0);
#endif
//Start BLE LED Thread
#if (DEBUG_LEVEL & DEBUG_BLE)
K_THREAD_DEFINE(ble_led, THREAD_BLE_LED_STACK, thread_ble_led, NULL, NULL, NULL, THREAD_PRIORITY_BLE_LED, 0, 0);
#endif
// Shell Commands ==============================================================

// Level 1 //
SHELL_STATIC_SUBCMD_SET_CREATE(time_read,
        SHELL_CMD(f, NULL, "Show the time formatted as hours, minutes, seconds..", cmd_disp_format_time),
        SHELL_SUBCMD_SET_END
);

// Level 0 //
SHELL_CMD_REGISTER(time, &time_read, "Display elapsed time since power on", cmd_disp_time);
SHELL_CMD_REGISTER(led, NULL, "Control the AHUs LEDs", cmd_led_control);



