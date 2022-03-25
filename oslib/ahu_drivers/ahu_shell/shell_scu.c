/**
 * @file shell_scu.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "shell_scu.h"
#include "ble_base.h"
#include "hci_driver.h"

int cmd_hts221_read_temp(const struct shell *shell, size_t argc,
                        char **argv) {
    
    package_hci_message(REQUEST, HTS221_T, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_hts221_read_hum(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_lis2dh_read_x(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_lis2dh_read_y(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_lis2dh_read_z(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_lps22_read_pres(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_ccs811_read_voc(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_buzzer_write_freq(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_rgb_write_rgb(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_pb_read_state(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_dc_write_percentage(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}

int cmd_sample_write_sec(const struct shell *shell, size_t argc,
                        char **argv) {

    return 0;
}