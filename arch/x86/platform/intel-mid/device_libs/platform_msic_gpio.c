/*
 * platform_msic_gpio.c: MSIC GPIO platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Sathyanarayanan Kuppuswamy <sathyanarayanan.kuppuswamy@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>

#include "platform_msic.h"
#include "platform_ipc.h"

static void __init *msic_gpio_platform_data(void *info)
{
	static struct intel_msic_gpio_pdata msic_gpio_pdata;

	int gpio = get_gpio_by_name("msic_gpio_base");

	if (gpio < 0)
		return NULL;

	/* Basincove PMIC GPIO has total 8 GPIO pins,
	 * GPIO[5:2,0] support 1.8V, GPIO[7:6,1] support 1.8V and 3.3V,
	 * We group GPIO[5:2] to low voltage and GPIO[7:6] to
	 * high voltage. Because the CTL registers are contiguous,
	 * this grouping method doesn't affect the driver usage but
	 * easy for the driver sharing among multiple platforms.
	 */
	msic_gpio_pdata.ngpio_lv = 6;
	msic_gpio_pdata.ngpio_hv = 2;
	msic_gpio_pdata.gpio0_lv_ctlo = 0x7E;
	msic_gpio_pdata.gpio0_lv_ctli = 0x8E;
	msic_gpio_pdata.gpio0_hv_ctlo = 0x84;
	msic_gpio_pdata.gpio0_hv_ctli = 0x94;

	msic_gpio_pdata.can_sleep = 1;
	msic_gpio_pdata.gpio_base = gpio;
	msic_pdata.gpio = &msic_gpio_pdata;

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_GPIO);
}

static const struct devs_id msic_gpio_dev_id __initconst = {
	.name = "msic_gpio",
	.type = SFI_DEV_TYPE_IPC,
	.delay = 1,
	.get_platform_data = &msic_gpio_platform_data,
	.device_handler = &ipc_device_handler,
};

sfi_device(msic_gpio_dev_id);
