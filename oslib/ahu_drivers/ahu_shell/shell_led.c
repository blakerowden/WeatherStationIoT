/**
 * @file ahu_led.c
 * @author Blake Rowden (s4427634@uqconnect.edu.au)
 * @brief Shell Time library built for the CSSE4011 Practical 1
 * @version 0.1
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#define OK 0

#include "shell_time.h"
#include "led_driver.h"
#include "log_driver.h"

// Logging Module
LOG_MODULE_REGISTER(SHELL_LED, INITIAL_SHELL_LED_LOG_LEVEL);

int cmd_led_control(const struct shell *shell, size_t argc,
                    char **argv)
{

    switch (argv[1][0])
    {
    case 'o':
        switch (argv[2][0])
        {
        case 'r':
            led1_on();
            return OK;
        case 'g':
            led2_on();
            return OK;
        case 'b':
            led3_on();
            return OK;
        default:
            no_command();
            return OK;
        }
    case 'f':
        switch (argv[2][0])
        {
        case 'r':
            led1_off();
            return OK;
        case 'g':
            led2_off();
            return OK;
        case 'b':
            led3_off();
            return OK;
        default:
            no_command();
            return OK;
        }
    case 't':
        switch (argv[2][0])
        {
        case 'r':
            led1_toggle();
            return OK;
        case 'g':
            led2_toggle();
            return OK;
        case 'b':
            led3_toggle();
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