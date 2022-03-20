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

const struct device *scu_sensors_init();
void scu_sensors_scan(const struct device *dev);
int scu_sensors_get_temp(void);
int scu_sensors_get_humidity(void);

#endif //SCU_SENS_H
