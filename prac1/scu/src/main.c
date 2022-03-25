/**
 * @file main.c
 * @author Jack Mason
 * @date 2022-03-23
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>

#include "scu_sensors.h"
#include "ble_uuid.h"
//#include "led_driver.h"

//#include "ble_base.h"
//#include "led_driver.h"
//#include "ble_uuid.h"
//#include "shell_scu.h"
#include "hci_driver.h"

int m_rec = 0;

//Keeps Track of BLE connection within APP
bool ble_conencted = false; // ?
static struct bt_conn *default_conn;
static uint16_t rx_handle;

/* Custom Service Variables */

/*
static struct bt_uuid_128 mobile_uuid = BT_UUID_INIT_128(
    0xe3, 0x68, 0x4d, 0xe0, 0xa9, 0xcf, 0x11, 0xec,
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);
*/
/*
static struct bt_uuid_128 node_tx = BT_UUID_INIT_128( 
    0x2a, 0xae, 0xfc, 0xda, 0xa9, 0xd0, 0x11, 0xec, 
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

static struct bt_uuid_128 node_rx = BT_UUID_INIT_128(
    0x41, 0xbf, 0xbd, 0xba, 0xa9, 0xd0, 0x11, 0xec, 
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);
*/

/*
static struct bt_uuid_128 vnd_enc_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));
*/

//static struct bt_uuid_128 vnd_auth_uuid = BT_UUID_INIT_128(
	//BT_UUID_128_ENCODE(0x2aaefcda, 0xa9d0, 0x11ec, 0xb909, 0x0242ac120002));

#define VND_MAX_LEN 20
/*
static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > VND_MAX_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	value[offset + len] = 0;

	return len;
}
*/

//static uint8_t simulate_vnd;

/*
static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}
*/

/**
 * @brief Used to parse the GATT service and characteristic data.
 */
static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params)
{
	int err;

	if (attr == NULL) {
		if (rx_handle == 0) {
			printk("Did not discover TX (%x)", rx_handle);
		}

		(void)memset(params, 0, sizeof(*params));

		return BT_GATT_ITER_STOP;
	}
    
	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (params->type == BT_GATT_DISCOVER_PRIMARY &&
	    bt_uuid_cmp(params->uuid, &node_ahu.uuid) == 0) {
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

		if (bt_uuid_cmp(chrc->uuid, &node_rx.uuid) == 0) {
			printk("Found tx\n");
			rx_handle = chrc->value_handle;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

#define VND_LONG_MAX_LEN 74

static void gatt_write_cb(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_write_params *params)
{
	if (err != BT_ATT_ERR_SUCCESS) {
		printk("Write failed: 0x%02X\n", err);
	}

	(void)memset(params, 0, sizeof(*params));

}


//static int signed_value;
//uint8_t rx_buff[] = {0xAA, 0x01, 0x16, 0xcc, 0xdd};
//uint8_t rx_buff[] = {0xAA, 0x01, 0x16, 0x00, 0x00};
//uint16_t rx_buff[] = {0xAA00, 0x0100, 0x1600, 0xBB00, 0xCC00};
//uint16_t tx_buff[] = {0x1122, 0x3344, 0x5566, 0x7788, 0x99AA};

/*
static ssize_t read_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 //sizeof(signed_value));
				 sizeof(tx_buff));
}
*/

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset,
			    uint8_t flags)
{
	uint8_t *value = attr->user_data;

	//if (offset + len > sizeof(signed_value)) {
	if (offset + len > sizeof(tx_buff)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static const struct bt_uuid_128 vnd_signed_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x41bfbdba, 0xa9d0, 0x11ec, 0xb909, 0x0242ac120002));

static ssize_t read_rx(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset)
{
    const int16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(rx_buff));
}

/* Vendor Primary Service Declaration */
/*
BT_GATT_SERVICE_DEFINE(vnd_svc, //may be able to change vnd_svc to give service name
	BT_GATT_PRIMARY_SERVICE(&mobile_uuid), //start primary service here
	
	BT_GATT_CHARACTERISTIC(&node_tx.uuid, BT_GATT_CHRC_READ |
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_AUTH,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       //read_signed, write_signed, &signed_value),
				   read_signed, write_signed, &tx_buff),
	
	BT_GATT_CHARACTERISTIC(&node_rx.uuid,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              read_rx, NULL, &rx_buff),

);
*/

static ssize_t write_tx(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset,
			uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(rx_buff)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
    //k_sem_give(&sem_name);
	//scu_sensors_toggle_led(); //!!!!
	m_rec = 1;
    
	return len;
}

BT_GATT_SERVICE_DEFINE(scu,
                    
        BT_GATT_PRIMARY_SERVICE(&node_scu),

        BT_GATT_CHARACTERISTIC(&node_tx.uuid,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL, write_tx, &tx_buff),);

static const struct bt_data ad[] = { 
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0xe3, 0x68, 0x4d, 0xe0, 0xa9, 0xcf, 0x11, 0xec,
                    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02),
					//BT_UUID_CUSTOM_SERVICE_VAL), //custom service gets added here
};

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

void ahu_write(void)
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

/*
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
		scu_sensors_toggle_led();
		gatt_discovery();
	}
}
*/

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
        gatt_discovery();

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

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	//cts_init(); cts disable?

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

/*
void handle_hci_packet() {
	if (tx_buff[0] = 0xAA12) {
		//valid
		int i;
		for (i = 0; i < 5; i++) {
			rx_buff[i] = 0xAAAA;
		}
	} else {
		int j;
		for (j = 0; j < 5; j++) {
			rx_buff[j] = 0xFFFF;
		}
	}
}
*/

void main(void)
{
	//struct bt_gatt_attr *vnd_ind_attr; //indication
	char str[BT_UUID_STR_LEN]; //gets used in abcdef1
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_set_name("JM CSERVUP");
	bt_set_name("JM CSERVUP");

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	//might need this for bluetooth to work
	/*
	vnd_ind_attr = bt_gatt_find_by_uuid(vnd_svc.attrs, vnd_svc.attr_count,
					    &vnd_enc_uuid.uuid);
	bt_uuid_to_str(&vnd_enc_uuid.uuid, str, sizeof(str));
	printk("Indicate VND attr %p (UUID %s)\n", vnd_ind_attr, str);
	*/

	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	//tx_buff[0] = 0;
	scu_sensors_io_init();
	//int test;
	while (1) {
		k_sleep(K_SECONDS(1));
		//package_hci_message(1, 2, 3, 4, 5);

		//test = scu_sensors_get_button_status();
		//scu_sensors_toggle_led();

		/*
		if (test >= 1) { //0
			//gpio_pin_set_dt(&led, val);
			scu_sensors_toggle_led();
		}
		*/
		
		//scu_sensors_get_temp();

		//signed_value = (signed_value + 1) % 20;
		//tx_buff[0] = (tx_buff[0] + 1) % 20;

		/*
		if (tx_buff[0] == 0) {
			rx_buff[0] = 0xAABB;
		} else {
			rx_buff[0] = 0xEEFF;
		}
		//handle_hci_packet();
		*/
		rx_buff[0] = (rx_buff[0] + 1) % 20;
		ahu_write();
		scu_sensors_toggle_led();
		if (m_rec) {
			m_rec = 0;
		}
	}
}
