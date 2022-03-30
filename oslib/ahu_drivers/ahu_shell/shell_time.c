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

#include "shell_time.h"
#include "log_driver.h"

#define OK 0

// Logging Module
LOG_MODULE_REGISTER(SHELL_TIME, INITIAL_SHELL_TIME_LOG_LEVEL);

/**
 * @brief Display the system uptime in seconds
 *
 * @param shell
 * @param argc
 * @param argv
 * @return Return 0 if the command was successful
 */
int cmd_disp_time(const struct shell *shell, size_t argc,
                  char **argv)
{

        ARG_UNUSED(argc);
        ARG_UNUSED(argv);

        int sec = k_uptime_get() / 1000;

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
int cmd_disp_format_time(const struct shell *shell, size_t argc,
                         char **argv)
{
        ARG_UNUSED(argc);
        ARG_UNUSED(argv);
        int sec = k_uptime_get() / 1000;
        int min = sec / 60;
        int hr = min / 60;

        shell_print(shell, "%uhrs %umin %us\n", hr, (min % 60), (sec % 60));
        return OK;
}