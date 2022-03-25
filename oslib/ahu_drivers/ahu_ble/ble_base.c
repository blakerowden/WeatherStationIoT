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
#include "shell_scu.h"
#include "hci_driver.h"

static void gatt_discovery(void);
static void start_scan(void);

static uint16_t tx_handle;
static struct bt_conn *default_conn;

//BLE Connection Flag
bool ble_connected;

K_SEM_DEFINE(sem_name, 0, 1);



/**
 * @brief Used to parse the advertisement data 
 *        in order to find the UUID of the service we are looking for.
 * 
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
    gatt_discovery();
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

static void gatt_write_cb(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_write_params *params)
{
	if (err != BT_ATT_ERR_SUCCESS) {
		printk("Write failed: 0x%02X\n", err);
	}

	(void)memset(params, 0, sizeof(*params));

}

/**
 * @brief Write data to the SCU TX GATT attribute.
 * 
 */
void scu_write(void)
{
	static struct bt_gatt_write_params write_params;
	int err;

	write_params.data = tx_buff;
	write_params.length = sizeof(tx_buff);
	write_params.func = gatt_write_cb;
	write_params.handle = tx_handle;

	err = bt_gatt_write(default_conn, &write_params);
	if (err != 0) {
		printk("bt_gatt_write failed: %d\n", err);
	}

}

/**
 * @brief BLE Base entry thread, starts initial ble scanning.
 *       Also, sets the connection callback struct.
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
 * @brief Used to parse the GATT service and characteristic data.
 */
static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params)
{
	int err;

	if (attr == NULL) {
		if (tx_handle == 0) {
			printk("Did not discover TX (%x)", tx_handle);
		}

		(void)memset(params, 0, sizeof(*params));

		return BT_GATT_ITER_STOP;
	}
    
	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (params->type == BT_GATT_DISCOVER_PRIMARY &&
	    bt_uuid_cmp(params->uuid, &node_scu.uuid) == 0) {
		printk("Found scu service\n");
		params->uuid = NULL;
		params->start_handle = attr->handle + 1;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, params);
		if (err != 0) {
			printk("Discover failed (err %d)\n", err);
		}

		return BT_GATT_ITER_STOP;
	} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

		if (bt_uuid_cmp(chrc->uuid, &node_tx.uuid) == 0) {
			printk("Found tx\n");
			tx_handle = chrc->value_handle;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief BLE GATT discovery function, used to discover the SCU TX GATT attribute.
 * 
 */
static void gatt_discovery(void)
{
	static struct bt_gatt_discover_params discover_params;
	int err;

	printk("Discovering services and characteristics\n");

	discover_params.uuid = &node_scu.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(default_conn, &discover_params);
	if (err != 0) {
		printk("Discover failed(err %d)\n", err);
	}
}

/**
 * @brief BLE LED blinker thread, blinks the LED when BLE is connected.
 * 			Used for debugging
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

/**
 * @brief CALLBACK function to process incoming data from the SCU RX GATT attribute. 
 */
static ssize_t write_rx(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset,
			uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(rx_buff)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
    k_sem_give(&sem_name);
    
	return len;
}

void process_rx_data(void) {
     
     while(1) {
        
        if (!k_sem_take(&sem_name, K_FOREVER)) {
            printk("[RX]: 0x%X 0x%x 0x%X 0x%x 0x%X\n", 
                rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4]);
        }
            
    }

}


/**
 * @brief Initialize the SCU GATT attributes.
 * 
 */
BT_GATT_SERVICE_DEFINE(ahu,
                    
        BT_GATT_PRIMARY_SERVICE(&node_ahu),

        BT_GATT_CHARACTERISTIC(&node_rx.uuid,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL, write_rx, &rx_buff),);