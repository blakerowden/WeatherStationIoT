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


// Debug Define Flags ==========================================================
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
void initialise(void) {   

    init_leds();
    begin_shell();
    
}


void main(void) {

    initialise();
    
}

// Thread Defines ==============================================================

//START BLE BASE entry thread : Delayed Start (Wait for USB to be ready)
K_THREAD_DEFINE(ble_base, THREAD_BLE_BASE_STACK, thread_ble_base, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(rx_data, THREAD_BLE_BASE_STACK, process_rx_data, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
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

SHELL_CMD_REGISTER(hts221, NULL, "Temp/Humidity", cmd_led_control);
SHELL_CMD_REGISTER(lps22, NULL, "Pressure", cmd_led_control);



