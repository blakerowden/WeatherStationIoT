/**
  ******************************************************************************
  * @file   : scu_power_management.c
  * @brief  : 
  ******************************************************************************
  * @author Jack Mason
  * ***************************************************************************
  */

#include <pm/pm.h>
#include <pm/device.h>
#include <pm/device_runtime.h>
#include <pm/state.h>

void device_low_power(const struct device *dev1, 
const struct device *dev2,
const struct device *dev3,
const struct device *dev4,
const struct device *dev5) {
  pm_device_action_run(dev1, PM_DEVICE_ACTION_SUSPEND);
  pm_device_action_run(dev2, PM_DEVICE_ACTION_SUSPEND);
  pm_device_action_run(dev3, PM_DEVICE_ACTION_SUSPEND);
  pm_device_action_run(dev4, PM_DEVICE_ACTION_SUSPEND);
  pm_device_action_run(dev5, PM_DEVICE_ACTION_SUSPEND);
}

void device_resume(const struct device *dev1, 
const struct device *dev2,
const struct device *dev3,
const struct device *dev4,
const struct device *dev5) {
  pm_device_action_run(dev1, PM_DEVICE_ACTION_RESUME);
  pm_device_action_run(dev2, PM_DEVICE_ACTION_RESUME);
  pm_device_action_run(dev3, PM_DEVICE_ACTION_RESUME);
  pm_device_action_run(dev4, PM_DEVICE_ACTION_RESUME);
  pm_device_action_run(dev5, PM_DEVICE_ACTION_RESUME);
}