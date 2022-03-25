/**
 ************************************************************************
 * @file mobile_connect.h
 * @author Wilfred MK, Aaron Helmore
 * @date 20.04.2021
 * @brief Contains required definitions by mobile_connect.c
 **********************************************************************
 **/

#ifndef MOBILE_CONNECT_H
#define MOBILE_CONNECT_H

/* Debug Thread Stack size */
#define THREAD_BLE_LED_THREAD_STACK 1024
#define THREAD_BLE_CONNECT_STACK 2048
/* Debug Thread Priority */
#define THREAD_PRIORITY_BLE_LED_THREAD 20
#define THREAD_PRIORITY_BLE_CONNECT_THREAD -3

//GATT CHARACTERISTIC VALUES
extern uint16_t rx_buff[];
extern uint16_t tx_buff[];

//Contains All scaled Sensor data for in the following format.
//accel_XYZ[0:2], Gyro_XYZ[3:5], Mag_XYZ[6:8]
extern double mpu9250_sensor[];

void thread_ble_connect(void);

void thread_ble_led(void);

void thread_ble_base(void);

#endif