/**
 * @file hci_driver.h
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
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

// Stores the DID's for the different devices
typedef enum
{
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

extern uint16_t tx_buff[6]; // Stores the data to be sent to the AHU
extern uint16_t rx_buff[6]; // Stores the data received to the AHU

extern uint8_t stream_freq; // Stores the frequency of the data stream
extern uint8_t all_active;  // Stores the state of the all_active flag

/**
 * @brief Get the data length object
 *
 * @param data1
 * @param data2
 * @param data3
 * @param data4
 * @return uint8_t The length of the data in bytes
 */
uint8_t get_data_length(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);

/**
 * @brief Pages up the data into the HCI format for tx and rx
 *
 * @param type
 * @param data1
 * @param data2
 * @param data3
 * @param data4
 * @param data5
 * @return int
 */
int package_hci_message(uint8_t type, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4, uint16_t data5);

/**
 * @brief Clears the TX buffer
 *
 */
void clear_tx(void);

/**
 * @brief Clears the RX buffer
 *
 */
void clear_rx(void);

#endif // HCI_DRIVER_H