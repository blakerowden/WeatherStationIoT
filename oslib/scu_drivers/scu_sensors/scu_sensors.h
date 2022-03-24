/**
  ******************************************************************************
  * @file   : scu_sensors.h
  * @brief  : 
  ******************************************************************************
  * @author Jack Mason
  * ***************************************************************************
  */

#ifndef SCU_SENS_H
#define SCU_SENS_H

#include <device.h>

#define HTS211 1 //temperature, humidity
#define CCS811 2 //air quality (VOC)
#define LIS2DH 3 //acceleration
#define LPS22HB 4 //air pressure

const struct device *scu_sensors_init(int sensor);
void scu_sensors_scan(const struct device *dev_hts211, 
const struct device *dev_ccs811,
const struct device *dev_lis2dh,
const struct device *dev_lps22hb);
int scu_sensors_get_temp(void);
int scu_sensors_get_humidity(void);
int scu_sensors_get_voc(void);
int scu_sensors_get_acc_x(void);
int scu_sensors_get_acc_y(void);
int scu_sensors_get_acc_z(void);
int scu_sensors_get_pressure(void);

#endif //SCU_SENS_H
