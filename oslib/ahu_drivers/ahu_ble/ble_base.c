/**
 * @file ble_base.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief 
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#include "ble_base.h"
#include "led_driver.h"
#include "ble_uuid.h"

void thread_ble_base(void);
static void start_scan(void);

static struct bt_conn *default_conn;
bool ble_connected;

//RX BUFFER
int16_t rx_buff[] = {
    0x00, 0x00, 0x00, 0x00
};

//RX BUFFER
int16_t tx_buff[] = {
    0x00, 0x00, 0x00, 0x00
};

/**
 * @brief Callback for BLE scanning, checks weather the returned 
 *          UUID matches the custom UUID of the mobile device.
 *        If matched, attempt to connect to device.
 * 
 * @param data Callback data from scanning
 * @param user_data Device User data
 * @return true 
 * @return false 
 */
static bool parse_device(struct bt_data *data, void *user_data)
{
    bt_addr_le_t *addr = user_data;
    int i;
    int matchedCount = 0;

    printk("[AD]: %u data_len %u\n", data->type, data->data_len);

    if (data->type == BT_DATA_UUID128_ALL)
    {

        uint16_t temp = 0;
        for (i = 0; i < data->data_len; i++)
        {
            temp = data->data[i];
            if (temp == ble_uuid[i])
            {
                matchedCount++;
            }
        }

        if (matchedCount == UUID_BUFFER_SIZE)
        {
            //MOBILE UUID MATCHED
            printk("Mobile UUID Found, attempting to connect\n");

            int err = bt_le_scan_stop();
            k_msleep(10);

            if (err)
            {
                printk("Stop LE scan failed (err %d)\n", err);
                return true;
            }

            struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;

            err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                    param, &default_conn);
            if (err)
            {
                printk("Create conn failed (err %d)\n", err);
                start_scan();
            }

            return false;
        }
    }
    return true;
}

/**
 * @brief Callback function for when scan detects device, scanned devices
 *          are filtered by their connectibilty and scan data is parsed.
 * 
 * @param addr Device Address
 * @param rssi RSSI 
 * @param type Device Type
 * @param ad Adv Data
 */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{

    if (default_conn)
    {
        return;
    }

    /* We're only interested in connectable events */
    if (type == BT_GAP_ADV_TYPE_ADV_IND ||
        type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND)
    {
        bt_data_parse(ad, parse_device, (void *)addr);
    }
}

/**
 * @brief Starts passive BLE scanning for nearby
 *          devices.
 */
static void start_scan(void)
{
    int err;

    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err)
    {
        printk("Scanning failed to start (err %d)\n", err);
        return;
    }

    printk("Scanning successfully started\n");
}

/**
 * @brief Callback for when reading RSSI Gatt atrribute data
 *          from the mobile device. The data read is saved into    
 *          and internal rx buffer. 
 * 
 * @param conn ble connection handler
 * @param err  ble ATT error val
 * @param params Read params
 * @param data Data read from GATT attribute
 * @param length Len of Data
 * @return uint8_t retVal (custom)
 */
uint8_t read_from_scu(struct bt_conn *conn, uint8_t err,
                              struct bt_gatt_read_params *params,
                              const void *data, uint16_t length)
{
    memcpy(&rx_buff, data, sizeof(rx_buff));
    //printk("RSSI: N1:%d, N2:%d, N3:%d, N4:%d\n", rx_rssi[0], rx_rssi[1], rx_rssi[2], rx_rssi[3]);
    return 0;
}

/**
 * @brief BLE Device connected callback function. If an error is detected
 *          scan is restarted. Else, the app can establish that the 
 *          devices are now conneted using flag ble_connected;
 * 
 * @param conn Connection handler
 * @param err BLE ERR
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err)
    {
        printk("Failed to connect to %s (%u)\n", addr, err);

        bt_conn_unref(default_conn);
        default_conn = NULL;

        start_scan();
        return;
    }

    if (conn != default_conn)
    {
        return;
    }
    ble_connected = true;
    printk("Connected: %s\n", addr);
}

/**
 * @brief BLE Disconnected callback, when disconnected, restarts BLE scanning
 *          and the app can detect that BLE has been disconnected by referring to
 *          ble_connected flag.
 * 
 * @param conn Connection handler.
 * @param reason Disconnect reason (ERR VAL).
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (conn != default_conn)
    {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

    bt_conn_unref(default_conn);
    default_conn = NULL;
    ble_connected = false;
    start_scan();
}



/**
 * @brief Connection callback struct, required to set conn/disconn
 *          function pointers.
 */
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

void thread_ble_read_out(void)
{
    static struct bt_gatt_read_params read_params_rx = {
        .func = read_from_scu,
        .handle_count = 0,
        .by_uuid.uuid = &node_rx.uuid,
        .by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    };

    int timeStamp = 0;

    while (1)
    {

        if (ble_connected)
        {
            timeStamp = k_cyc_to_ms_floor64(k_cycle_get_32());
            //Read from the rx attribute
            bt_gatt_read(default_conn, &read_params_rx);
            
            printk("%x,%x,%x,%x,%x \n", rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4]);

        }

        k_usleep(100);
    }
}

/**
 * @brief BLE Base entry thread, starts initial ble scanning.
 *          When a valid mobile device is connected.
 */
void thread_ble_base(void)
{
    int err;

    err = bt_enable(NULL);
    default_conn = NULL;

    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    bt_conn_cb_register(&conn_callbacks);

    start_scan();

    //Should not reach here
    printk("Debug_1\n");
}

/**
 * @brief Super important thread that will ensure le
 * 			led is blinking. Like i said, this is super important.
 * 
 */
void thread_ble_led(void)
{

    ble_connected = false;
    bool led_is_on = true;

    while (1)
    {
        led_is_on = !led_is_on;

        if (ble_connected)
        {
            gpio_pin_set(device_get_binding(LED1), LED1_PIN, (int)led_is_on);
            gpio_pin_set(device_get_binding(LED0), LED0_PIN, (int)false);
            k_msleep(BLE_CONN_SLEEP_MS);
        }
        else
        {
            gpio_pin_set(device_get_binding(LED1), LED1_PIN, (int)led_is_on);
            gpio_pin_set(device_get_binding(LED0), LED0_PIN, (int)led_is_on);
            k_msleep(BLE_DISC_SLEEP_MS);
        }
    }
}