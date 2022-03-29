/**
  ******************************************************************************
  * @file   : ble_base.h
  * @brief  : 
  ******************************************************************************
  * @author Blake Rowden
  * ***************************************************************************
  */

#ifndef BLE_BASE_H
#define BLE_BASE_H

/* Debug Thread Stack size */
#define THREAD_BLE_LED_STACK 512
#define THREAD_BLE_BASE_STACK 4094
/* Debug Thread Priority */
#define THREAD_PRIORITY_BLE_LED 10
#define THREAD_PRIORITY_BLE_BASE -2
#define THREAD_PRIORITY_READ_BASE -10

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000

extern struct k_sem sem_data_arrived;

//GATT CHARACTERISTIC VALUES

void thread_ble_led(void);

void thread_ble_base(void);

/**
 * @brief Write data to the SCU TX GATT attribute.
 * 
 */
void scu_write(void);



#endif //BLE_BASE_H