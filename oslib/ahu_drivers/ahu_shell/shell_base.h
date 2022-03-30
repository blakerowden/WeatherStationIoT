/**
 * @file ahu_shell.h
 * @author Blake Rowden (s4427634@uqconnect.edu.au)
 * @brief Shell library built for the CSSE4011 Practical 1
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

#define OK 0

#ifndef AHU_SHELL_H
#define AHU_SHELL_H

/**
 * @brief Initialises the shell
 *
 * @return int 0 on success, negative errno code on failure.
 */
int begin_shell(void);

#endif // AHU_SHELL_H