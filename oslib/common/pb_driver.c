/**
 * @file pb_driver.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief
 * @version 0.1
 * @date 2022-03-31
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <device.h>
#include <drivers/gpio.h>
#include <inttypes.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <zephyr.h>

#include "hci_driver.h"
#include "shell_scu.h"

#define SLEEP_TIME_MS 1

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins)
{
  if (all_active)
  {
    cmd_all_off(NULL, 0, NULL);
  }
  else
  {
    cmd_all_on(NULL, 0, NULL);
  }
}

void setup_pb(void)
{
  int ret;

  if (!device_is_ready(button.port))
  {
    printk("Error: button device %s is not ready\n", button.port->name);
    return;
  }

  ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
  if (ret != 0)
  {
    printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name,
           button.pin);
    return;
  }

  ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0)
  {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
           button.port->name, button.pin);
    return;
  }

  gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
  gpio_add_callback(button.port, &button_cb_data);
  printk("Set up button at %s pin %d\n", button.port->name, button.pin);
}