/**
  ******************************************************************************
  * @file   : scu_sensors.c
  * @brief  : 
  ******************************************************************************
  * @author Jack Mason
  * ***************************************************************************
  */

#include <drivers/sensor.h>
#include <device.h>
#include "scu_sensors.h"

//button & led
#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>

//rgb led
#include <drivers/gpio/gpio_sx1509b.h>

#define NUMBER_OF_LEDS 3

//#define GREEN_LED DT_GPIO_PIN(DT_NODELABEL(led0), gpios)
//#define BLUE_LED DT_GPIO_PIN(DT_NODELABEL(led1), gpios)
//#define RED_LED DT_GPIO_PIN(DT_NODELABEL(led2), gpios)

#define BLUE_LED DT_GPIO_PIN(DT_NODELABEL(led2), gpios)
#define RED_LED DT_GPIO_PIN(DT_NODELABEL(led0), gpios)
#define GREEN_LED DT_GPIO_PIN(DT_NODELABEL(led1), gpios)





enum sx1509b_color { sx1509b_red = 0, sx1509b_green, sx1509b_blue };

//static struct k_work_delayable rgb_work;
//static uint8_t which_color = sx1509b_red;
//static uint8_t iterator;

static const gpio_pin_t rgb_pins[] = {
	RED_LED,
	GREEN_LED,
	BLUE_LED,
};


/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;


struct sensor_value temp, humidity, voc, acc_x, acc_y, acc_z, pressure; //press,
struct device *mydev1; //= NULL? 

uint16_t temp_buff[] = {0x0000, 0x0000, 0x0000, 0x0000};
uint16_t humid_buff[] = {0x0000, 0x0000, 0x0000, 0x0000};
uint16_t voc_buff[] = {0x0000, 0x0000, 0x0000, 0x0000};
uint16_t acc_x_buff[] = {0x0000, 0x0000, 0x0000, 0x0000};
uint16_t acc_y_buff[] = {0x0000, 0x0000, 0x0000, 0x0000};
uint16_t acc_z_buff[] = {0x0000, 0x0000, 0x0000, 0x0000};
uint16_t press_buff[] = {0x0000, 0x0000, 0x0000, 0x0000};

int button_val;
int led_val = 0;

const struct device *get_rgb_led_device(void) {
	const struct device *dev_sx1509 = device_get_binding(DT_PROP(DT_NODELABEL(sx1509b), label));
	int err;

	if (dev_sx1509 == NULL) {
		printk("Error binding SX1509B device\n");

		return NULL;
	}

	for (int i = 0; i < NUMBER_OF_LEDS; i++) {
		err = sx1509b_led_intensity_pin_configure(dev_sx1509,
							  rgb_pins[i]);

		if (err) {
			printk("Error configuring pin for LED intensity\n");
		}
	}

	return dev_sx1509;
}

void set_rgb_led(const struct device *dev_sx1509, int red, int green, int blue) {
	sx1509b_led_intensity_pin_set(dev_sx1509, RED_LED, red);
	sx1509b_led_intensity_pin_set(dev_sx1509, GREEN_LED, green);
	sx1509b_led_intensity_pin_set(dev_sx1509, BLUE_LED, blue);
}

static const struct device *get_hts221_device(void) { 

	const struct device *dev = DEVICE_DT_GET_ANY(st_hts221); //bosch_bme280


	if (dev == NULL) {

		return NULL;

	}


	if (!device_is_ready(dev)) {

		return NULL;

	}

	return dev;

}

static const struct device *get_ccs811_device(void) { 

	const struct device *dev = DEVICE_DT_GET_ANY(ams_ccs811); //bosch_bme280 //ams,ccs811


	if (dev == NULL) {

		return NULL;

	}


	if (!device_is_ready(dev)) {

		return NULL;

	}

	return dev;

}

static const struct device *get_lis2dh_device(void) { 

	const struct device *dev = DEVICE_DT_GET_ANY(st_lis2dh);


	if (dev == NULL) {

		return NULL;

	}


	if (!device_is_ready(dev)) {

		return NULL;

	}

	return dev;

}

static const struct device *get_lps22hb_device(void) { 

	const struct device *dev = DEVICE_DT_GET_ANY(st_lps22hb_press); //-press


	if (dev == NULL) {

		return NULL;

	}


	if (!device_is_ready(dev)) {

		return NULL;

	}

	return dev;

}

const struct device *scu_sensors_init(int sensor) {
  if (sensor == LPS22HB) {
    const struct device *dev = get_lps22hb_device();
    return dev;
  }
  if (sensor == LIS2DH) {
    const struct device *dev = get_lis2dh_device();
    return dev;
  }
  if (sensor == CCS811) {
    const struct device *dev = get_ccs811_device();
    return dev;
  }
  if (sensor == HTS211) {
    const struct device *dev = get_hts221_device(); 
    return dev;
  }

  const struct device *dev = get_hts221_device(); 
  return dev;

}

void scu_sensors_scan(const struct device *dev_hts211, 
const struct device *dev_ccs811,
const struct device *dev_lis2dh,
const struct device *dev_lps22hb) {
  
  sensor_sample_fetch(dev_hts211);
  sensor_sample_fetch(dev_ccs811); 
  sensor_sample_fetch(dev_lis2dh); 
  sensor_sample_fetch(dev_lps22hb);

  sensor_channel_get(dev_hts211, SENSOR_CHAN_AMBIENT_TEMP, &temp);
  sensor_channel_get(dev_hts211, SENSOR_CHAN_HUMIDITY, &humidity);
  sensor_channel_get(dev_ccs811, SENSOR_CHAN_VOC, &voc);
  sensor_channel_get(dev_lis2dh, SENSOR_CHAN_ACCEL_X, &acc_x);
  sensor_channel_get(dev_lis2dh, SENSOR_CHAN_ACCEL_Y, &acc_y);
  sensor_channel_get(dev_lis2dh, SENSOR_CHAN_ACCEL_Z, &acc_z);
  sensor_channel_get(dev_lps22hb, SENSOR_CHAN_PRESS, &pressure);

  //button_val = gpio_pin_get_dt(&button);

  temp_buff[0] = (temp.val1 >> 16) & 0xFFFF;
  temp_buff[1] = temp.val1 & 0xFFFF;
  temp_buff[2] = (temp.val2 >> 16) & 0xFFFF;
  temp_buff[3] = temp.val2 & 0xFFFF;

  humid_buff[0] = (humidity.val1 >> 16) & 0xFFFF;
  humid_buff[1] = humidity.val1 & 0xFFFF;
  humid_buff[2] = (humidity.val2 >> 16) & 0xFFFF;
  humid_buff[3] = humidity.val2 & 0xFFFF;

  voc_buff[0] = (voc.val1 >> 16) & 0xFFFF;
  voc_buff[1] = voc.val1 & 0xFFFF;
  voc_buff[2] = (voc.val2 >> 16) & 0xFFFF;
  voc_buff[3] = voc.val2 & 0xFFFF;

  acc_x_buff[0] = (acc_x.val1 >> 16) & 0xFFFF;
  acc_x_buff[1] = acc_x.val1 & 0xFFFF;
  acc_x_buff[2] = (acc_x.val2 >> 16) & 0xFFFF;
  acc_x_buff[3] = acc_x.val2 & 0xFFFF;

  acc_y_buff[0] = (acc_y.val1 >> 16) & 0xFFFF;
  acc_y_buff[1] = acc_y.val1 & 0xFFFF;
  acc_y_buff[2] = (acc_y.val2 >> 16) & 0xFFFF;
  acc_y_buff[3] = acc_y.val2 & 0xFFFF;

  acc_z_buff[0] = (acc_z.val1 >> 16) & 0xFFFF;
  acc_z_buff[1] = acc_z.val1 & 0xFFFF;
  acc_z_buff[2] = (acc_z.val2 >> 16) & 0xFFFF;
  acc_z_buff[3] = acc_z.val2 & 0xFFFF;

  press_buff[0] = (pressure.val1 >> 16) & 0xFFFF;
  press_buff[1] = pressure.val1 & 0xFFFF;
  press_buff[2] = (pressure.val2 >> 16) & 0xFFFF;
  press_buff[3] = pressure.val2 & 0xFFFF;

}

int scu_sensors_get_temp() {
  return temp.val1;
}

int scu_sensors_get_humidity() {
  return humidity.val1;
}

int scu_sensors_get_voc() {
  return voc.val2; //change to val1
}

int scu_sensors_get_acc_x() {
  return acc_x.val1;
}

int scu_sensors_get_acc_y() {
  return acc_y.val1;
}

int scu_sensors_get_acc_z() {
  return acc_z.val1;
}

int scu_sensors_get_pressure() {
  return pressure.val1;
}

int scu_sensors_get_button_status() {
  button_val = gpio_pin_get_dt(&button);
  return button_val;
}

#define SLEEP_TIME_MS	1

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */

 /*
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});
*/ 

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int scu_sensors_toggle_led() {
	/*
  if(led.port) {
    led_val = 1 - led_val;
    gpio_pin_set_dt(&led, led_val);
  }
  return led_val;
  */
  return 0;

}

void scu_sensors_io_init() {
  int ret;

	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	/*
	if (led.port && !device_is_ready(led.port)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led.port->name);
		led.port = NULL;
	}
	if (led.port) {
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led.port->name, led.pin);
			led.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}
	*/
}

/*
void main(void)
{
	

	printk("Press the button\n");
	if (led.port) {
		while (1) {
			//If we have an LED, match its state to the button's. 
			//int val = gpio_pin_get_dt(&button);

			if (val >= 0) {
				//gpio_pin_set_dt(&led, val);
			}
			k_msleep(SLEEP_TIME_MS);
		}
	}
}
*/