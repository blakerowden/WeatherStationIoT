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


struct sensor_value temp, humidity; //press,
struct device *mydev1; //= NULL? 

static const struct device *get_hts221_device(void) { 

	const struct device *dev = DEVICE_DT_GET_ANY(st_hts221); //bosch_bme280 


	if (dev == NULL) {

		//No such node, or the node does not have status "okay". 

		//printk("\nError: no device found.\n");

		return NULL;

	}


	if (!device_is_ready(dev)) {

		//printk("\nError: Device \"%s\" is not ready; "

		       //"check the driver initialization logs for errors.\n",

		       //dev->name);

		return NULL;

	}


	//printk("Found device \"%s\", getting sensor data\n", dev->name);

	return dev;

}

const struct device *scu_sensors_init() {

  const struct device *dev = get_hts221_device(); //const struct device 

  if (dev == NULL) {
    ;//return;
  }

  return dev;

}

void scu_sensors_scan(const struct device *dev) {
  
  sensor_sample_fetch(dev);
  sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
  sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);

}

int scu_sensors_get_temp() {
  return temp.val1;
}

int scu_sensors_get_humidity() {
  return humidity.val1;
}