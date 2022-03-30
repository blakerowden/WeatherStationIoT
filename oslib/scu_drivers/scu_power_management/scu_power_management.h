/**
  ******************************************************************************
  * @file   : scu_power_management.h
  * @brief  : 
  ******************************************************************************
  * @author Jack Mason
  * ***************************************************************************
  */

#ifndef SCU_PM_H
#define SCU_PM_H

#define LOW_POWER 1
#define FULL_POWER 2

void device_resume(const struct device *dev1, 
const struct device *dev2,
const struct device *dev3,
const struct device *dev4,
const struct device *dev5);

void device_low_power(const struct device *dev1, 
const struct device *dev2,
const struct device *dev3,
const struct device *dev4,
const struct device *dev5);

#endif //SCU_PM_H
