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
#include "sens.h"


struct sensor_value temp, humidity, voc, acc_x, acc_y, acc_z, pressure; //press,
struct device *mydev1; //= NULL? 

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

	const struct device *dev = DEVICE_DT_GET_ANY(st_lps22hb-press);


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
  //sensor_sample_fetch(dev_lis2dh); 
  //sensor_sample_fetch(dev_lps22hb);

  sensor_channel_get(dev_hts211, SENSOR_CHAN_AMBIENT_TEMP, &temp);
  sensor_channel_get(dev_hts211, SENSOR_CHAN_HUMIDITY, &humidity);
  sensor_channel_get(dev_ccs811, SENSOR_CHAN_VOC, &voc);
  //sensor_channel_get(dev_lis2dh, SENSOR_CHAN_ACCEL_X, &acc_x);
  //sensor_channel_get(dev_lis2dh, SENSOR_CHAN_ACCEL_Y, &acc_y);
  //sensor_channel_get(dev_lis2dh, SENSOR_CHAN_ACCEL_Z, &acc_z);
  //sensor_channel_get(dev_lps22hb, SENSOR_CHAN_PRESS, &pressure);

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