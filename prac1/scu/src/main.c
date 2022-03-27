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
#include "hci_driver.h"

#include <device.h>
#include <drivers/pwm.h>

#include <pm/pm.h>
#include <pm/device.h>
#include <pm/device_runtime.h>
#include <pm/state.h>
//#include <include/pm/device.h>

/*
#define PWM_LED0_NODE	DT_ALIAS(pwm-audio)

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR	DT_PWMS_CTLR(PWM_LED0_NODE)
#define PWM_CHANNEL	DT_PWMS_CHANNEL(PWM_LED0_NODE)
#define PWM_FLAGS	DT_PWMS_FLAGS(PWM_LED0_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR	DT_INVALID_NODE
#define PWM_CHANNEL	0
#define PWM_FLAGS	0
#endif
*/


#if DT_NODE_HAS_STATUS(DT_ALIAS(pwmaudio), okay)
#define PWM_DRIVER  DT_PWMS_CTLR(DT_ALIAS(pwmaudio))
//#define PWM_CHANNEL DT_PWMS_CHANNEL(DT_ALIAS(pwmaudio))
//#define PWM_FLAGS	DT_PWMS_FLAGS(DT_ALIAS(pwmaudio))
#else
#error "Choose a supported PWM driver"
#endif

int m_rec = 0;

//Keeps Track of BLE connection within APP
bool ble_connected = false;
static struct bt_conn *default_conn;
static uint16_t rx_handle;
static void start_scan(void);

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

static void gatt_write_cb(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_write_params *params)
{
	if (err != BT_ATT_ERR_SUCCESS) {
		printk("Write failed: 0x%02X\n", err);
	}

	(void)memset(params, 0, sizeof(*params));

}

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
                    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02), //node ahu/scu
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
        ble_connected = false;
    }
    else
    {   
        default_conn = conn;
        printk("BLE Connected to Device\n");
        ble_connected = true;
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

/*
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}
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

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

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

void main(void)
{
	
	struct device *pwm_dev;
	//64_t cycles;
	/*
    pwm_dev = device_get_binding(PWM_DRIVER);
    if (!pwm_dev) {
       printk("Cannot find %s!\n", PWM_DRIVER);
       return;
    }  
	//pwm_get_cycles_per_sec(pwm_dev, PWM_CHANNEL, &cycles);
	*/

	/*
	pwm_dev = DEVICE_DT_GET(PWM_DRIVER);
	if (!device_is_ready(pwm_dev)) {
		printk("Error: PWM device %s is not ready\n", pwm_dev->name);
		return;
	}
	*/
	
	

	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_set_name("JM SCU");

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	const struct device *hts = scu_sensors_init(HTS211);
	const struct device *ccs = scu_sensors_init(CCS811);
	const struct device *lis = scu_sensors_init(LIS2DH);
	const struct device *lps = scu_sensors_init(LPS22HB);
	const struct device *rgb = get_rgb_led_device();

	scu_sensors_scan(hts, ccs, lis, lps);
	scu_sensors_io_init();

	while (1) {
		
		/*
		if (pwm_pin_set_usec(pwm_dev, PWM_CHANNEL, 1000*50, 1000*50 / 2U, PWM_FLAGS)) {
			printk("pwm pin set fails\n");
			//return;
		}

		k_sleep(K_SECONDS(1));
		
		if (pwm_pin_set_usec(pwm_dev, PWM_CHANNEL, 1000*50, 0, PWM_FLAGS)) {
			printk("pwm off fails\n");
			//return;
		}
		*/
		


		k_sleep(K_SECONDS(1));
		if (m_rec == 1) {

			pm_device_action_run(hts, PM_DEVICE_ACTION_RESUME);
			pm_device_action_run(ccs, PM_DEVICE_ACTION_RESUME);
			pm_device_action_run(lis, PM_DEVICE_ACTION_RESUME);
			pm_device_action_run(lps, PM_DEVICE_ACTION_RESUME);
			pm_device_action_run(rgb, PM_DEVICE_ACTION_RESUME);

			//printk("[RX]: 0x%X 0x%x 0x%X 0x%x 0x%X 0x%x\n", 
                //tx_buff[0], tx_buff[1], tx_buff[2], tx_buff[3], tx_buff[4], tx_buff[5]);

			if (tx_buff[1] == HTS221_T) {

				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 9;
				rx_buff[1] = HTS221_T;
				rx_buff[2] = temp_buff[0];
				rx_buff[3] = temp_buff[1];
				rx_buff[4] = temp_buff[2];
				rx_buff[5] = temp_buff[3];
			}
			if (tx_buff[1] == HTS221_H) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 9;
				rx_buff[1] = HTS221_H;
				rx_buff[2] = humid_buff[0];
				rx_buff[3] = humid_buff[1];
				rx_buff[4] = humid_buff[2];
				rx_buff[5] = humid_buff[3];
			}
			if (tx_buff[1] == LPS22_AP) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 9;
				rx_buff[1] = LPS22_AP;
				rx_buff[2] = press_buff[0];
				rx_buff[3] = press_buff[1];
				rx_buff[4] = press_buff[2];
				rx_buff[5] = press_buff[3];
			}
			if (tx_buff[1] == CCS811_VOC) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 9;
				rx_buff[1] = CCS811_VOC;
				rx_buff[2] = voc_buff[0];
				rx_buff[3] = voc_buff[1];
				rx_buff[4] = voc_buff[2];
				rx_buff[5] = voc_buff[3];
			}
			if (tx_buff[1] == LIS2DH_X_ACC) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 9;
				rx_buff[1] = LIS2DH_X_ACC;
				rx_buff[2] = acc_x_buff[0];
				rx_buff[3] = acc_x_buff[1];
				rx_buff[4] = acc_x_buff[2];
				rx_buff[5] = acc_x_buff[3];
			}
			if (tx_buff[1] == LIS2DH_Y_ACC) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 9;
				rx_buff[1] = LIS2DH_Y_ACC;
				rx_buff[2] = acc_y_buff[0];
				rx_buff[3] = acc_y_buff[1];
				rx_buff[4] = acc_y_buff[2];
				rx_buff[5] = acc_y_buff[3];
			}
			if (tx_buff[1] == LIS2DH_Z_ACC) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 9;
				rx_buff[1] = LIS2DH_Z_ACC;
				rx_buff[2] = acc_z_buff[0];
				rx_buff[3] = acc_z_buff[1];
				rx_buff[4] = acc_z_buff[2];
				rx_buff[5] = acc_z_buff[3];
			}
			if (tx_buff[1] == RGB_LED) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				set_rgb_led(rgb, tx_buff[2], tx_buff[3], tx_buff[4]);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 7;
				rx_buff[1] = RGB_LED;
				rx_buff[2] = tx_buff[2];
				rx_buff[3] = tx_buff[3];
				rx_buff[4] = tx_buff[4];
				rx_buff[5] = 0;
			}
			if (tx_buff[1] == BUZ) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 5;
				rx_buff[1] = BUZ;
				rx_buff[2] = tx_buff[2];
				rx_buff[3] = 0;
				rx_buff[4] = 0;
				rx_buff[5] = 0;
			}
			if (tx_buff[1] == PB) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 5;
				rx_buff[1] = PB;
				rx_buff[2] = scu_sensors_get_button_status();
				rx_buff[3] = 0;
				rx_buff[4] = 0;
				rx_buff[5] = 0;
			}
			if (tx_buff[1] == DC) { //duty cycle
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 5;
				rx_buff[1] = DC;
				rx_buff[2] = tx_buff[2];
				rx_buff[3] = 0;
				rx_buff[4] = 0;
				rx_buff[5] = 0;
			}
			if (tx_buff[1] == SAMPLE) {
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 5;
				rx_buff[1] = SAMPLE;
				rx_buff[2] = tx_buff[2];
				rx_buff[3] = 0;
				rx_buff[4] = 0;
				rx_buff[5] = 0;
				
			}
			if (tx_buff[1] == ALL) {
				
				scu_sensors_scan(hts, ccs, lis, lps);
				rx_buff[0] = (PREAMBLE << 8) | (RESPONSE << 4) | 5;
				rx_buff[1] = ALL;
				rx_buff[2] = tx_buff[2];
				rx_buff[3] = 0;
				rx_buff[4] = 0;
				rx_buff[5] = 0;
			}
			ahu_write();
			m_rec = 0;
			pm_device_action_run(hts, PM_DEVICE_ACTION_SUSPEND);
			pm_device_action_run(ccs, PM_DEVICE_ACTION_SUSPEND);
			pm_device_action_run(lis, PM_DEVICE_ACTION_SUSPEND);
			pm_device_action_run(lps, PM_DEVICE_ACTION_SUSPEND);
			pm_device_action_run(rgb, PM_DEVICE_ACTION_SUSPEND);
			/*
			hts = scu_sensors_init(HTS211);
	const struct device *ccs = scu_sensors_init(CCS811);
	const struct device *lis = scu_sensors_init(LIS2DH);
	const struct device *lps = scu_sensors_init(LPS22HB);
	const struct device *rgb = get_rgb_led_device();

*/
		}
	}
}
