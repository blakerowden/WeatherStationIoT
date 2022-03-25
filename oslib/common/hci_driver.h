/**
 * @file hci_driver.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef HCI_DRIVER_H
#define HCI_DRIVER_H

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#define PREAMBLE 0xAA
#define REQUEST 0x01
#define RESPONSE 0x02

typedef enum {
    NO_DEV,
    HTS221_T,
    HTS221_H,
    LPS22_AP,
    CCS811_VOC,
    LIS2DH_X_ACC,
    LIS2DH_Y_ACC,
    LIS2DH_Z_ACC,
    RGB_LED,
    BUZ,
    PB,
    DC,
    SAMPLE,
    ALL
} device_id;

extern uint16_t tx_buff[6];
extern uint16_t rx_buff[6];

uint8_t get_data_length(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4, uint16_t data5);
int package_hci_message(uint8_t type, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4, uint16_t data5);

void clear_tx(void);
void clear_rx(void);


#endif //HCI_DRIVER_H