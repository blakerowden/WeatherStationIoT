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

// Private Includes ============================================================

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "shell_base.h"
#include "led_driver.h"

// Private Defines =============================================================

#define OK  0
#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

// Logging Module ==============================================================

 LOG_MODULE_REGISTER(main);

// Private Function Prototypes =================================================

void hardware_setup(void);

// Private Shell Command Prototypes ============================================

static int cmd_disp_time(const struct shell *, size_t, char **);
static int cmd_disp_format_time(const struct shell *, size_t, char **);
static int cmd_led_control(const struct shell *, size_t, char **);

// Private Thread Prototypes ===================================================

void my_entry_point(void *, void *, void *);

// Insertion Point =============================================================

/**
 * @brief The main function for the application host unit
 * 
 */
void main(void) {

	hardware_setup();
    
    while (1)
    {
        k_sleep(K_MSEC(100));
    }
    
}

/**
 * @brief Sets up the hardware for the application host unit
 * 
 */
void hardware_setup(void) {

    begin_shell();
   
    led0_init();
    led1_init();
    led2_init();

}

// Thread Functions ============================================================

/**
 * @brief 
 * 
 * @param a 
 * @param b 
 * @param c 
 */
void my_entry_point(void *a, void *b, void *c) {   
    
    
    while(1) {
        k_sleep(K_MSEC(1000));
    }
}

// Shell Commands ==============================================================

/**
 * @brief Display the system uptime in seconds
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return Return 0 if the command was successful
 */
static int cmd_disp_time(const struct shell *shell, size_t argc,
                        char **argv) {

        ARG_UNUSED(argc);
        ARG_UNUSED(argv);

        int sec = k_uptime_get()/1000;

        shell_print(shell, "%us\n", sec);

        return OK;
}

/**
 * @brief Display the system uptime in a human readable format of hrs, min, sec
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_disp_format_time(const struct shell *shell, size_t argc,
                        char **argv)
{
        ARG_UNUSED(argc);
        ARG_UNUSED(argv);
        int sec = k_uptime_get()/1000;
        int min = sec/60;
        int hr = min/60;

        shell_print(shell, "%uhrs %umin %us\n", hr, (min%60), (sec%60));
        return OK;
}

static int cmd_led_control(const struct shell *shell, size_t argc,
                        char **argv) {

    switch (argv[1][0]) {
        case 'o':
            switch (argv[2][0]) {
                case 'r':
                    led1_on();
                    return OK;
                case 'g':
                    led2_on();
                    return OK;
                case 'b':
                    led0_on();
                    return OK;
                default:
                    no_command();
                    return OK;
            }
        case 'f':
            switch (argv[2][0]) {
                case 'r':
                    led1_off();
                    return OK;
                case 'g':
                    led2_off();
                    return OK;
                case 'b':
                    led0_off();
                    return OK;
                default:
                    no_command();
                    return OK;
            }
        case 't':
            switch (argv[2][0]) {
                case 'r':
                    led1_toggle();
                    return OK;
                case 'g':
                    led2_toggle();
                    return OK;
                case 'b':
                    led0_toggle();
                    return OK;
                default:
                    no_command();
                    return OK;
            }
            
        default:
            no_command();
            return OK;
    }

    return OK;

}

// Level 1 Subcommand Macros ===================================================

SHELL_STATIC_SUBCMD_SET_CREATE(time_read,
        SHELL_CMD(f,   NULL, "Show the time formatted as hours, minutes, seconds..", cmd_disp_format_time),
        SHELL_SUBCMD_SET_END
);

// Level 0 Root Subcommand Macros ==============================================

SHELL_CMD_REGISTER(time, &time_read, "Display elapsed time since power on", cmd_disp_time);
SHELL_CMD_REGISTER(led, NULL, "Control the AHUs LEDs", cmd_led_control);

// Thread Macros ===============================================================

K_THREAD_DEFINE(my_tid, MY_STACK_SIZE,
                my_entry_point, NULL, NULL, NULL,
                MY_PRIORITY, 0, 0);
