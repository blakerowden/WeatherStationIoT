/**
 * @file hci_driver.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief
 * @version 0.1
 * @date 2022-03-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "hci_driver.h"

// Logging Module
LOG_MODULE_REGISTER(HCI, LOG_LEVEL_ERR);

uint16_t tx_buff[] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
uint16_t rx_buff[] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

uint8_t stream_freq = 5;
uint8_t all_active = 0;

uint8_t get_data_length(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)
{

    if (data4 & 0xFF00)
    {
        return 8;
    }
    else if (data4)
    {
        return 7;
    }
    else if (data3 & 0xFF00)
    {
        return 6;
    }
    else if (data3)
    {
        return 5;
    }
    else if (data2 & 0xFF00)
    {
        return 4;
    }
    else if (data2)
    {
        return 3;
    }
    else if (data1 & 0xFF00)
    {
        return 2;
    }
    else if (data1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int package_hci_message(uint8_t type, uint16_t device_id, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)
{

    uint8_t data_length = get_data_length(data1, data2, data3, data4);
    LOG_DBG("data_length: %d", data_length);

    switch (type)
    {
    case REQUEST:
        tx_buff[0] = (PREAMBLE << 8) | (REQUEST << 4) | data_length;
        tx_buff[1] = device_id;
        tx_buff[2] = data1;
        if (data_length > 6)
        {
            tx_buff[5] = data4;
        }
        if (data_length > 4)
        {
            tx_buff[4] = data3;
        }
        if (data_length > 2)
        {
            tx_buff[3] = data2;
        }
        break;

    case RESPONSE:
        rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | data_length;
        rx_buff[1] = device_id;
        rx_buff[2] = data1;
        if (data_length > 6)
        {
            rx_buff[5] = data4;
        }
        if (data_length > 4)
        {
            rx_buff[4] = data3;
        }
        if (data_length > 2)
        {
            rx_buff[3] = data2;
        }
        break;
    default:
        break;
    }

    return 0;
}

void clear_tx(void)
{
    for (int i = 0; i < 5; i++)
    {
        tx_buff[i] = 0x0000;
    }
}

void clear_rx(void)
{
    for (int i = 0; i < 5; i++)
    {
        rx_buff[i] = 0x0000;
    }
}
