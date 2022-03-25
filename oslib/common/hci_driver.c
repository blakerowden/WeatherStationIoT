/**
 * @file hci_driver.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "hci_driver.h"

uint16_t tx_buff[] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
uint16_t rx_buff[] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

uint8_t get_data_length(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4, uint16_t data5) {
    
    if (!data1) {
        return 1;
    } else if (!data2) {
        if (!(data1 & 0xFF00)) {
            return 1;
        } else {
            return  2;
        }
    } else if (!data3) {
        if (!(data2 & 0xFF00)) {
            return  3;
        } else {
            return  4;
        }
    } else if (!data4) {
        if (!(data3 & 0xFF00)) {
            return  5;
        } else {
            return  6;
        }
    } else if (!data5) {
        if (!(data4 & 0xFF00)) {
            return  7;
        } else {
            return  8;
        }
    }else {
        if (!(data5 & 0xFF00)) {
            return  7;
        } else {
            return  8;
        }

    }
}

int package_hci_message(uint8_t type, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4, uint16_t data5) {
    
    uint8_t data_length = get_data_length(data1, data2, data3, data4, data5);

    switch (type) {
    case REQUEST:
        tx_buff[0] = (PREAMBLE << 8) | (REQUEST << 4) | data_length;
        tx_buff[1] = data1;
        if (data_length > 8) {
            tx_buff[5] = data5;
        } if (data_length > 6) {
            tx_buff[4] = data4;
        } if (data_length > 4) {
            tx_buff[3] = data3;
        } if (data_length > 2) {
            tx_buff[2] = data2;
        }
        break;

    case RESPONSE:
        rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | data_length;
        rx_buff[1] = data1;
        if (data_length > 8) {
            rx_buff[5] = data5;
        } if (data_length > 6) {
            rx_buff[4] = data4;
        } if (data_length > 4) {
            rx_buff[3] = data3;
        } if (data_length > 2) {
            rx_buff[2] = data2;
        }
        break;
    default:
        break;
    }
    
    return 0;
}

void clear_tx(void) {
    for (int i = 0; i < 5; i++) {
        tx_buff[i] = 0x0000;
    }
}

void clear_rx(void) {
    for (int i = 0; i < 5; i++) {
        rx_buff[i] = 0x0000;
    }
}
