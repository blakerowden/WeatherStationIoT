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

extern uint16_t temp_buff[4];
extern uint16_t humid_buff[4];
extern uint16_t voc_buff[4];
extern uint16_t acc_x_buff[4];
extern uint16_t acc_y_buff[4];
extern uint16_t acc_z_buff[4];
extern uint16_t press_buff[4];

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
int scu_sensors_get_button_status(void);
int scu_sensors_toggle_led(void);
void scu_sensors_io_init(void);
const struct device *get_rgb_led_device(void);
void set_rgb_led(const struct device *dev_hts211, int red, int green, int blue);

#endif //SCU_SENS_H
