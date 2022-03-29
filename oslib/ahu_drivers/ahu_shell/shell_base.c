/**
 * @file ahu_shell.c
 * @author Blake Rowden (s4427634@uqconnect.edu.au)
 * @brief Shell library built for the CSSE4011 Practical 1
 * @version 0.1
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "shell_base.h"
#include "log_driver.h"

// Logging Module
LOG_MODULE_REGISTER(SHELL_BASE, INITIAL_SHELL_BASE_LOG_LEVEL);

#define NO_USB -1

int begin_shell(void)
{
    const struct device *shell_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
    uint32_t dtr = 0;

    /* Enable the USB Driver */
    if (usb_enable(NULL))
        return NO_USB;

    /* Wait on DTR - 'Data Terminal Ready'
     * Will wait here until a terminal has been attached to the device
     * This is not necessary, however, can be useful for printing boot info etc..
     */
    while (!dtr)
    {
        uart_line_ctrl_get(shell_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    printk("\n=======================================\n");
    printk("Welcome to the Weather Station Terminal\n");
    printk("=======================================\n");
    printk("\n--Press TAB to see available commands--\n\n");
    return 0;
}
