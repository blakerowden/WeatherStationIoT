/**
 * @file shell_scu.h
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief
 * @version 0.1
 * @date 2022-03-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>
#include <shell/shell.h>

#ifndef SHELL_SCU_H
#define SHELL_SCU_H

/**
 * @brief Interput handler for the read temperature command
 *      Requests the temperature from the HTS221 sensor
 *
 * @return int error code
 */
int cmd_hts221_read_temp(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the read humidity command
 *      Requests the humidity in percent
 *
 * @return int error code
 */
int cmd_hts221_read_hum(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the read x acceleration command
 *    Requests the x acceleration from the LIS2DH sensor
 *
 * @return int error code
 */
int cmd_lis2dh_read_x(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the read y acceleration command
 *   Requests the y acceleration from the LIS2DH sensor
 *
 * @return int error code
 */
int cmd_lis2dh_read_y(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the read z acceleration command
 *   Requests the z acceleration from the LIS2DH sensor
 *
 * @return int error code
 */
int cmd_lis2dh_read_z(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the read pressure command
 *      Requests the pressure from the LPS22 sensor
 *
 * @return int error code
 */
int cmd_lps22_read_pres(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the read VOC command
 *      Requests the VOC from the CCS811 sensor
 *
 * @return int error code
 */
int cmd_ccs811_read_voc(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the buzzer write command
 *      Writes a frequency value to the buzzer on the SCU
 *
 * @return int error code
 */
int cmd_buzzer_write_freq(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the RGB write command
 *      Writes RGB values to the scu LED
 *
 * @return int error code
 */
int cmd_rgb_write_rgb(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the push button read command
 *      Requests the push button state from the scu
 *
 * @return int error code
 */
int cmd_pb_read_state(const struct shell *, size_t, char **);

/**
 * @brief Interput handler for the duty cycle command
 *     Writes a duty cycle value to the power unit on the SCU
 *
 * @return int error code
 */
int cmd_dc_write_percentage(const struct shell *, size_t, char **);

/**
 * @brief Interupt handler for the read sample time command
 *      Writes a sample time value to the SCU 
 * @return int 
 */
int cmd_sample_write_sec(const struct shell *, size_t, char **);

/**
 * @brief Interupt handler for the read sample time command
 *      Sets the continuous sampling mode on the SCU to ON
 * 
 * @return int error code 
 */
int cmd_all_on(const struct shell *, size_t, char **);

/**
 * @brief Interupt handler for the read sample time command
 *      Sets the continuous sampling mode on the SCU to OFF
 * 
 * @return int error code
 */
int cmd_all_off(const struct shell *, size_t, char **);

#endif // SHELL_SCU_H