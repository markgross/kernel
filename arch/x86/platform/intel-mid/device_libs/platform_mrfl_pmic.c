/*
 * platform_mrfl_pmic.c: Platform data for Merrifield PMIC driver
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <asm/intel-mid.h>
#include <asm/pmic_pdata.h>
#include <asm/intel_mid_remoteproc.h>
#ifdef CONFIG_BQ24261_CHARGER
#include <linux/power/bq24261_charger.h>
#endif
#include "platform_ipc.h"
#include "platform_mrfl_pmic.h"

void __init *mrfl_pmic_ccsm_platform_data(void *info)
{
	static struct pmic_platform_data pmic_pdata;

#ifdef CONFIG_BQ24261_CHARGER
	pmic_pdata.cc_to_reg = bq24261_cc_to_reg;
	pmic_pdata.cv_to_reg = bq24261_cv_to_reg;
#endif
	register_rpmsg_service("rpmsg_pmic_ccsm", RPROC_SCU,
				RP_PMIC_CCSM);
out:
	return &pmic_pdata;
}

static const struct devs_id pmic_ccsm_dev_id __initconst = {
	.name = "pmic_ccsm",
	.type = SFI_DEV_TYPE_IPC,
	.delay = 1,
	.get_platform_data = &mrfl_pmic_ccsm_platform_data,
};

sfi_device(pmic_ccsm_dev_id);
