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
#include <drivers/pwm.h>

#include <pm/pm.h>
#include <pm/device.h>
#include <pm/device_runtime.h>
#include <pm/state.h>

#define PWM_BUZZ_NODE	DT_ALIAS(pwmaudio) 

#if DT_NODE_HAS_STATUS(PWM_BUZZ_NODE, okay)
#define PWM_CTLR	DT_PWMS_CTLR(PWM_BUZZ_NODE)
#define PWM_CHANNEL	DT_PWMS_CHANNEL(PWM_BUZZ_NODE)
#define PWM_FLAGS	DT_PWMS_FLAGS(PWM_BUZZ_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR	DT_INVALID_NODE
#define PWM_CHANNEL	0
#define PWM_FLAGS	0
#endif

#define MIN_PERIOD_USEC	(USEC_PER_SEC / 128U)
#define MAX_PERIOD_USEC	USEC_PER_SEC

void main(void)
{	
	const struct device *pwm;
	uint32_t max_period;
	uint32_t period;
	uint8_t dir = 0U;
	int ret;

	pwm = DEVICE_DT_GET(PWM_CTLR);
	if (!device_is_ready(pwm)) {
		printk("Error: PWM device %s is not ready\n", pwm->name);
		return;
	}

	printk("Calibrating for channel %d...\n", PWM_CHANNEL);
	max_period = MAX_PERIOD_USEC;
	while (pwm_pin_set_usec(pwm, PWM_CHANNEL,
				max_period, max_period / 2U, PWM_FLAGS)) {
		max_period /= 2U;
		if (max_period < (4U * MIN_PERIOD_USEC)) {
			printk("Error: PWM device "
			       "does not support a period at least %u\n",
			       4U * MIN_PERIOD_USEC);
			return;
		}
	}

	printk("Done calibrating; maximum/minimum periods %u/%u usec\n",
	       max_period, MIN_PERIOD_USEC);

	period = max_period;
	

	while (1) {
		
		ret = pwm_pin_set_usec(pwm, PWM_CHANNEL,
				       period, period / 2U, PWM_FLAGS);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return;
		}

		period = dir ? (period * 2U) : (period / 2U);
		if (period > max_period) {
			period = max_period / 2U;
			dir = 0U;
		} else if (period < MIN_PERIOD_USEC) {
			period = MIN_PERIOD_USEC * 2U;
			dir = 1U;
		}

		k_sleep(K_SECONDS(4U));
		
	}
}

K_THREAD_DEFINE(ble_base, THREAD_BLE_BASE_STACK, thread_bluetooth, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(rx_data, THREAD_BLE_BASE_STACK, thread_data, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(continuous, THREAD_BLE_BASE_STACK, thread_continuous, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE+1, 0, 0);