/**
 * @file shell_scu.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
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
#include "log_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Logging Module
LOG_MODULE_REGISTER(SCU_CMD, INITIAL_SHELL_SCU_LOG_LEVEL);

int cmd_hts221_read_temp(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, HTS221_T, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_hts221_read_hum(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, HTS221_H, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_lis2dh_read_x(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, LIS2DH_X_ACC, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_lis2dh_read_y(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, LIS2DH_Y_ACC, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_lis2dh_read_z(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, LIS2DH_Z_ACC, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_lps22_read_pres(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, LPS22_AP, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_ccs811_read_voc(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, CCS811_VOC, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_buzzer_write_freq(const struct shell *shell, size_t argc, char **argv)
{

    uint16_t freq = atoi(argv[1]);
    package_hci_message(REQUEST, BUZ, freq, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_rgb_write_rgb(const struct shell *shell, size_t argc, char **argv)
{

    uint16_t red = atoi(argv[1]);
    uint16_t green = atoi(argv[2]);
    uint16_t blue = atoi(argv[3]);

    package_hci_message(REQUEST, RGB_LED, red, green, blue, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_pb_read_state(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, PB, 0, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_dc_write_percentage(const struct shell *shell, size_t argc,
                            char **argv)
{

    uint16_t percentage = atoi(argv[1]);
    package_hci_message(REQUEST, DC, percentage, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_sample_write_sec(const struct shell *shell, size_t argc, char **argv)
{

    stream_freq = atoi(argv[1]);
    package_hci_message(REQUEST, SAMPLE, stream_freq, 0, 0, 0);
    scu_write();
    clear_tx();

    return 0;
}

int cmd_all_on(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, ALL, 1, 0, 0, 0);
    scu_write();
    clear_tx();
    all_active = true;

    return 0;
}

int cmd_all_off(const struct shell *shell, size_t argc, char **argv)
{

    package_hci_message(REQUEST, ALL, 0, 0, 0, 0);
    scu_write();
    clear_tx();
    all_active = false;

    return 0;
}