/**
 * @file shell_time.h
 * @author Blake Rowden (s4427634@uqconnect.edu.au)
 * @brief Shell Time library built for the CSSE4011 Practical 1
 * @version 0.1
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>
#include <shell/shell.h>

#ifndef SHELL_TIME_H
#define SHELL_TIME_H

/**
 * @brief Display the time in seconds
 * 
 * @return int error code
 */
int cmd_disp_time(const struct shell *, size_t, char **);

/**
 * @brief Display the time in a human readable format of hrs, min, sec
 * 
 * @return int 
 */
int cmd_disp_format_time(const struct shell *, size_t, char **);

#endif // SHELL_TIME_H