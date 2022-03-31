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
#include "scu_ble.h"
#include "hci_driver.h"

#include <device.h>
//#include <drivers/pwm.h>

//#include <pm/pm.h>
//#include <pm/device.h>
//#include <pm/device_runtime.h>
//#include <pm/state.h>

//set(DTC_OVERLAY_FILE pwm.overlay) //Cmakelists
/*
#if DT_NODE_HAS_STATUS(DT_ALIAS(pwmaudio), okay)
#define PWM_DRIVER  DT_PWMS_CTLR(DT_ALIAS(pwmaudio))
//#define PWM_CHANNEL DT_PWMS_CHANNEL(DT_ALIAS(pwmaudio))
//#define PWM_FLAGS	DT_PWMS_FLAGS(DT_ALIAS(pwmaudio))
#else
#error "Choose a supported PWM driver"
#endif
*/

void main(void)
{
	
	//struct device *pwm_dev;
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

		//k_sleep(K_SECONDS(1));
		
	}
}

K_THREAD_DEFINE(ble_base, THREAD_BLE_BASE_STACK, thread_bluetooth, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(rx_data, THREAD_BLE_BASE_STACK, thread_data, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(continuous, THREAD_BLE_BASE_STACK, thread_continuous, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE+1, 0, 0);