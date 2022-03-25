/**
 ************************************************************************
 * @file mobile_connect.c
 * @author Wilfred MK, Aaron Helmore
 * @date 20.04.2021
 * @brief Mobile node will adv on BLE and await connection. Sets up 
 *          GATT attritubes for the base to read required data.
 **********************************************************************
 **/
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#include "mobile_node_ble.h"
#include "mobile_connect.h"

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000
#define SHORT_SLEEP_MS 50

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xe3, 0x68, 0x4d, 0xe0, 0xa9, 0xcf, 0x11, 0xec,
                    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02),
};

//Keeps Track of BLE connection within APP
bool ble_conencted = false;
static void gatt_discover(void);
static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params);

static struct bt_uuid_128 mobile_uuid = BT_UUID_INIT_128(
    0xe3, 0x68, 0x4d, 0xe0, 0xa9, 0xcf, 0x11, 0xec,
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

static struct bt_uuid_128 node_rx = BT_UUID_INIT_128(
    0x41, 0xbf, 0xbd, 0xba, 0xa9, 0xd0, 0x11, 0xec, 
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

static struct bt_uuid_128 node_tx = BT_UUID_INIT_128( 
   0x2a, 0xae, 0xfc, 0xda, 0xa9, 0xd0, 0x11, 0xec, 
   0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

static struct bt_uuid_128 node_ahu = BT_UUID_INIT_128( 
   0xe3, 0x68, 0x4d, 0xe0, 0xa9, 0xcf, 0x11, 0xec,
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

//GATT CHARACTERISTIC VALUES
//RX BUFFER
uint16_t rx_buff[6] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
uint16_t tx_buff[6];
uint16_t tx_update;
static uint16_t rx_handle;
static struct bt_conn *default_conn;
static void ahu_write(void);

static ssize_t write_tx(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset,
			uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(tx_buff)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	
    tx_update = 1U;
    printk("TX: HEADER:%X Device Val:%X Data1:%X Data2:%X Data3:%X\n", tx_buff[0], tx_buff[1], tx_buff[2], tx_buff[3], tx_buff[4]);

	return len;
}



BT_GATT_SERVICE_DEFINE(mobile_scu,
                    
        BT_GATT_PRIMARY_SERVICE(&mobile_uuid),

        BT_GATT_CHARACTERISTIC(&node_tx.uuid,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL, write_tx, &tx_buff),);

/**
 * @brief Connection call back
 * 
 * @param conn conenction handler
 * @param err err val
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        printk("Connection failed (err 0x%02x)\n", err);
        ble_conencted = false;
    }
    else
    {   
        default_conn = conn;
        printk("BLE Connected to Device\n");
        ble_conencted = true;
        struct bt_le_conn_param *param = BT_LE_CONN_PARAM(6, 6, 0, 400);
        char addr[BT_ADDR_LE_STR_LEN];
        printk("Connected: %s\n", addr);
        gatt_discover();

        if (bt_conn_le_param_update(conn, param) < 0)
        {
            while (1)
            {
                printk("Connection Update Error\n");
                k_msleep(10);
            }
        }
    }
}

static void gatt_discover(void)
{
	static struct bt_gatt_discover_params discover_params;
	int err;

	printk("Discovering services and characteristics\n");

	discover_params.uuid = &node_ahu.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(default_conn, &discover_params);
	if (err != 0) {
		printk("Discover failed(err %d)\n", err);
	}
}

static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params)
{
	int err;

	if (attr == NULL) {
		if (rx_handle == 0) {
			printk("Did not discover RX (%x)", rx_handle);
		}

		(void)memset(params, 0, sizeof(*params));

		return BT_GATT_ITER_STOP;
	}
    
	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (params->type == BT_GATT_DISCOVER_PRIMARY &&
	    bt_uuid_cmp(params->uuid, &node_ahu.uuid) == 0) {
		printk("Found ahu service\n");
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

		if (bt_uuid_cmp(chrc->uuid, &node_rx.uuid) == 0) {
			printk("Found rx\n");
			rx_handle = chrc->value_handle;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief Disconnect Callback, used to keep track of connection status 
 *          in the application layer
 * 
 * @param conn connection handler
 * @param reason disconnect reason.
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);
    ble_conencted = false;
}

/**
 * @brief Passcode handler for accessing encrypted data
 * 
 * @param conn connection handler
 * @param passkey passkey
 */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}

/**
 * @brief Authorisation cancelled handler
 * 
 * @param conn conenction handler
 */
static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n", addr);
}

/**
 * @brief Conn callback data structs, holds
 *          function pointers.
 * 
 */
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};
/**
 * @brief Conn AUTH callback data structs, holds
 *          function pointers.
 * 
 */
static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
};

/**
 * @brief Initialises bluetooth, and begins advertising data
 *            on BLE.
 * 
 */
static void bt_ready(void)
{
    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        //settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    //bt_passkey_set(0xAA289);
    printk("Advertising successfully started\n");
}

/**
 * @brief Enabled bluetooth, and sets connection callback handler, awaits
 *          central to connect to peripheral (mobile)
 * 
 */
void thread_ble_connect(void)
{
    bt_ready();

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);

    while (1)
    {
        k_msleep(SHORT_SLEEP_MS);
    }
}

/**
 * @brief Enable GPIO led pins
 * 
 */
void led_gpio_enable(void)
{
    int ret = gpio_pin_configure(device_get_binding(LED0), PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    if (ret < 0)
    {
        return;
    }
}

/**
 * @brief Debug blink led thread, blinks LED based on conenction status.
 * 
 */
void thread_ble_led(void)
{
    bool led_is_on = true;
    led_gpio_enable();

    while (1)
    {
        led_is_on = !led_is_on;
        gpio_pin_set(device_get_binding(LED0), PIN, (int)led_is_on);

        if (ble_conencted)
        {   
            
            k_msleep(1000);
        }
        else
        {
            k_msleep(BLE_DISC_SLEEP_MS);
        }
    }
}

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
static void ahu_write(void)
{
	static struct bt_gatt_write_params write_params;
	int err;

	write_params.data = rx_buff;
	write_params.length = sizeof(rx_buff);
	write_params.func = gatt_write_cb;
	write_params.handle = rx_handle;

	err = bt_gatt_write(default_conn, &write_params);
	if (err != 0) {
		printk("bt_gatt_write failed: %d\n", err);
	}

	printk("success\n");
}