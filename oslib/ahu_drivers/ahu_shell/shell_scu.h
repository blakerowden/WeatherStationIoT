/**
 * @file shell_scu.h
 * @author your name (you@domain.com)
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

int cmd_hts221_read_temp(const struct shell *, size_t, char **);
int cmd_hts221_read_hum(const struct shell *, size_t, char **);

int cmd_lis2dh_read_x(const struct shell *, size_t, char **);
int cmd_lis2dh_read_y(const struct shell *, size_t, char **);
int cmd_lis2dh_read_z(const struct shell *, size_t, char **);

int cmd_lps22_read_pres(const struct shell *, size_t, char **);

int cmd_ccs811_read_voc(const struct shell *, size_t, char **);

int cmd_buzzer_write_freq(const struct shell *, size_t, char **);

int cmd_rgb_write_rgb(const struct shell *, size_t, char **);

int cmd_pb_read_state(const struct shell *, size_t, char **);

int cmd_dc_write_percentage(const struct shell *, size_t, char **);

int cmd_sample_write_sec(const struct shell *, size_t, char **);

int cmd_all_on(const struct shell *, size_t, char **);
int cmd_all_off(const struct shell *, size_t, char **);

#endif // SHELL_SCU_H