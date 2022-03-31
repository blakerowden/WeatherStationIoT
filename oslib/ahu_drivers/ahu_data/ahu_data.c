/**
 * @file ahu_data.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <errno.h>
#include <stddef.h>
#include <sys/printk.h>
#include <usb/usb_device.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include <sys/byteorder.h>

#include "ahu_data.h"
#include "ble_base.h"
#include "hci_driver.h"
#include "led_driver.h"
#include "log_driver.h"

typedef struct
{
    uint8_t temp;
    uint8_t humidity;
    uint8_t air_pressure;
    uint8_t VOC;
    uint8_t X;
    uint8_t Y;
    uint8_t Z;
    uint8_t rgb[3];
    uint8_t buzzer;
    bool push_button;
} ahu_data_t;

// Logging Module
LOG_MODULE_REGISTER(DATA, INITIAL_DATA_LOG_LEVEL);

ahu_data_t ahu_data;

/**
 * @brief Process the RAW data into the data struct
 *
 */
void process_raw_data(void)
{

    switch (rx_buff[1])
    {
    case 0x01:
        ahu_data.temp = rx_buff[2];
        break;
    case 0x02:
        ahu_data.humidity = rx_buff[2];
        break;
    case 0x03:
        ahu_data.air_pressure = rx_buff[2];
        break;
    case 0x04:
        ahu_data.VOC = rx_buff[2];
        break;
    case 0x05:
        ahu_data.X = rx_buff[2];
        break;
    case 0x06:
        ahu_data.Y = rx_buff[2];
        break;
    case 0x07:
        ahu_data.Z = rx_buff[2];
        break;
    case 0x08:
        ahu_data.rgb[0] = rx_buff[2];
        ahu_data.rgb[1] = rx_buff[3];
        ahu_data.rgb[2] = rx_buff[4];
        break;
    case 0x09:
        ahu_data.buzzer = rx_buff[2];
        break;
    case 0x0A:
        ahu_data.push_button = rx_buff[2];
        break;
    default:
        break;
    }
}

/**
 * @brief Print the response from the SCU
 *
 */
void print_scu_response(void)
{

    switch (rx_buff[1])
    {
    case 0x01:
        printk("The temperature is %i Degrees\n", ahu_data.temp);
        break;
    case 0x02:
        printk("The humidity is %i Percent\n", ahu_data.humidity);
        break;
    case 0x03:
        printk("The air pressure is %i mmHg\n", ahu_data.air_pressure);
        break;
    case 0x04:
        printk("The Volatile Organic Compounds are %i ppm\n", ahu_data.VOC);
        break;
    case 0x05:
        printk("The X Acceleration is %i N\n", ahu_data.X);
        break;
    case 0x06:
        printk("The Y Acceleration is %i N\n", ahu_data.Y);
        break;
    case 0x07:
        printk("The Z Acceleration is %i N\n", ahu_data.Z);
        break;
    case 0x08:
        printk("The RGB values are %i, %i, %i\n", ahu_data.rgb[0], ahu_data.rgb[1],
               ahu_data.rgb[2]);
        break;
    case 0x09:
        printk("The buzzer is now set to %i Hz\n", ahu_data.buzzer);
        break;
    case 0x0A:
        if (ahu_data.push_button)
        {
            printk("The push button is pressed\n");
        }
        else
        {
            printk("The push button is not pressed\n");
        }
        break;
    default:
        break;
    }
}

/**
 * @brief Process incoming data from the SCU RX GATT attribute.
 *
 */
void process_rx_data(void)
{

    while (1)
    {

        if (!k_sem_take(&sem_data_arrived, K_FOREVER))
        {
            LOG_DBG("[RAW RX]: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n", rx_buff[0],
                    rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4], rx_buff[5]);
            process_raw_data();
            if (!all_active)
            {
                print_scu_response();
            }
        }
    }
}

/**
 * @brief Print JSON data to the Shell
 *
 */
void JSON_thread(void)
{
    stream_freq = 1;
    while (1)
    {
        if (all_active)
        {
            printk("{ \"1\" :%i, \"2\" :%i, \"3\" :%i, \"4\" :%i, \"5\" :%i, \"6\" "
                   ":%i, \"7\" :%i, \"8\" :%i, \"9\" :%i, \"10\" :%i }\n",
                   ahu_data.temp, ahu_data.humidity, ahu_data.air_pressure,
                   ahu_data.VOC, ahu_data.X, ahu_data.Y, ahu_data.Z, ahu_data.rgb[0],
                   ahu_data.buzzer, ahu_data.push_button);
        }

        k_sleep(K_MSEC(stream_freq * 1000));
    }
}