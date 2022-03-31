/**
 * @file shell_led.h
 * @author Blake Rowden (s4427634@uqconnect.edu.au)
 * @brief Shell LED library built for the CSSE4011 Practical 1
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

#ifndef SHELL_LED_H
#define SHELL_LED_H

/**
 * @brief Interput handler for the shell_led command
 * 
 * @return int 
 */
int cmd_led_control(const struct shell *, size_t, char **);

#endif // SHELL_LED_H