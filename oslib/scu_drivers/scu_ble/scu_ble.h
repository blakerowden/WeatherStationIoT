/**
  ******************************************************************************
  * @file   : scu_ble.h
  * @brief  : 
  ******************************************************************************
  * @author Jack Mason
  * ***************************************************************************
  */

#ifndef SCU_BLE_H
#define SCU_BLE_H

/* Debug Thread Stack size */
#define THREAD_BLE_BASE_STACK 4094

/* Debug Thread Priority */
#define THREAD_PRIORITY_BLE_BASE -2

void thread_data(void);
void thread_bluetooth();
void thread_continuous();

#endif //SCU_BLE_H
