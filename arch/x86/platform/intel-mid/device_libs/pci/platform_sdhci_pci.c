/*
 * platform_mmc_sdhci_pci.c: mmc sdhci pci platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/intel-mid.h>
#include <linux/mmc/sdhci-pci-data.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <asm/intel_scu_ipc.h>
#include <linux/intel_mid_pm.h>
#include <linux/hardirq.h>
#include <linux/mmc/sdhci.h>

#include "platform_sdhci_pci.h"

#ifdef CONFIG_ATOM_SOC_POWER
static int panic_mode_emmc0_power_up(void *data)
{
	int ret;
	bool atomic_context;
	/*
	 * Since pmu_set_emmc_to_d0i0_atomic function can
	 * only be used in atomic context, before call this
	 * function, do a check first and make sure this function
	 * is used in atomic context.
	 */
	atomic_context = (!preemptible() || in_atomic_preempt_off());

	if (!atomic_context) {
		pr_err("%s: not in atomic context!\n", __func__);
		return -EPERM;
	}

	ret = pmu_set_emmc_to_d0i0_atomic();
	if (ret) {
		pr_err("%s: power up host failed with err %d\n",
				__func__, ret);
	}

	return ret;
}
#else
static int panic_mode_emmc0_power_up(void *data)
{
	return 0;
}
#endif

static unsigned int sdhci_pdata_quirks;

int sdhci_pdata_set_quirks(unsigned int quirks)
{
	/*Should not be set more than once*/
	WARN_ON(sdhci_pdata_quirks);
	sdhci_pdata_quirks = quirks;
	return 0;
}

static int mrfl_sdio_setup(struct sdhci_pci_data *data);
static void mrfl_sdio_cleanup(struct sdhci_pci_data *data);

static void (*sdhci_embedded_control)(void *dev_id, void (*virtual_cd)
					(void *dev_id, int card_present));

static unsigned int sdhci_pdata_quirks;

/* MFLD platform data */
static struct sdhci_pci_data mfld_sdhci_pci_data[] = {
	[EMMC0_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.setup = 0,
			.cleanup = 0,
			.power_up = panic_mode_emmc0_power_up,
	},
	[EMMC1_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.setup = 0,
			.cleanup = 0,
	},
	[SD_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = 69,
			.setup = 0,
			.cleanup = 0,
	},
	[SDIO_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
	},
};

/* CLV platform data */
static struct sdhci_pci_data clv_sdhci_pci_data[] = {
	[EMMC0_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.setup = 0,
			.cleanup = 0,
			.power_up = panic_mode_emmc0_power_up,
	},
	[EMMC1_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.setup = 0,
			.cleanup = 0,
	},
	[SD_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = 69,
			.setup = 0,
			.cleanup = 0,
	},
	[SDIO_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
	},
};

/* Board specific cleanup related to SDIO goes here */
static void mrfl_sdio_cleanup(struct sdhci_pci_data *data)
{
}


/* Board specific setup related to SDIO goes here */
static int mrfl_sdio_setup(struct sdhci_pci_data *data)
{
	return 0;
}

/* MRFL platform data */
static struct sdhci_pci_data mrfl_sdhci_pci_data[] = {
	[EMMC0_INDEX] = {
			.pdev = NULL,
			.slotno = EMMC0_INDEX,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = panic_mode_emmc0_power_up,
	},
	[EMMC1_INDEX] = {
			.pdev = NULL,
			.slotno = EMMC1_INDEX,
			.rst_n_gpio = 97,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
	},
	[SD_INDEX] = {
			.pdev = NULL,
			.slotno = SD_INDEX,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = 77,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
	},
	[SDIO_INDEX] = {
			.pdev = NULL,
			.slotno = SDIO_INDEX,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = mrfl_sdio_setup,
			.cleanup = mrfl_sdio_cleanup,
	},
};

/* Board specific setup related to SDIO goes here */

static int byt_sdio_setup(struct sdhci_pci_data *data)
{
	return 0;
}


/* BYT platform data */
static struct sdhci_pci_data byt_sdhci_pci_data[] = {
	[SDIO_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = byt_sdio_setup,
			.cleanup = NULL,
	},
};

static struct sdhci_pci_data *get_sdhci_platform_data(struct pci_dev *pdev)
{
	struct sdhci_pci_data *pdata = NULL;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_MFD_EMMC0:
		pdata = &mfld_sdhci_pci_data[EMMC0_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_MFD_EMMC1:
		pdata = &mfld_sdhci_pci_data[EMMC1_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_MFD_SD:
		pdata = &mfld_sdhci_pci_data[SD_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_MFD_SDIO1:
		pdata = &mfld_sdhci_pci_data[SDIO_INDEX];
		pdata->quirks = sdhci_pdata_quirks;
		pdata->register_embedded_control = sdhci_embedded_control;
		break;
	case PCI_DEVICE_ID_INTEL_CLV_EMMC0:
		pdata = &clv_sdhci_pci_data[EMMC0_INDEX];
		pdata->rst_n_gpio = get_gpio_by_name("emmc0_rst");
		break;
	case PCI_DEVICE_ID_INTEL_CLV_EMMC1:
		pdata = &clv_sdhci_pci_data[EMMC1_INDEX];
		pdata->rst_n_gpio = get_gpio_by_name("emmc1_rst");
		break;
	case PCI_DEVICE_ID_INTEL_CLV_SDIO0:
		pdata = &clv_sdhci_pci_data[SD_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_CLV_SDIO1:
		pdata = &clv_sdhci_pci_data[SDIO_INDEX];
		pdata->quirks = sdhci_pdata_quirks;
		pdata->register_embedded_control = sdhci_embedded_control;
		break;
	case PCI_DEVICE_ID_INTEL_MRFL_MMC:
		switch (PCI_FUNC(pdev->devfn)) {
		case 0:
			pdata = &mrfl_sdhci_pci_data[EMMC0_INDEX];
			/*
			 * The current eMMC device simulation in Merrifield
			 * VP only implements boot partition 0, does not
			 * implements boot partition 1. And the VP will
			 * crash if eMMC boot partition 1 is accessed
			 * during kernel boot. So, we just disable boot
			 * partition support for Merrifield VP platform.
			 */
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_VP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_EMMC_BOOT_PART;
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_HVP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_HIGH_SPEED;
			break;
		case 1:
			pdata = &mrfl_sdhci_pci_data[EMMC1_INDEX];
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_VP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_EMMC_BOOT_PART;
			/*
			 * Merrifield HVP platform only implements
			 * eMMC0 host controller in its FPGA, and
			 * does not implements other 3 Merrifield
			 * SDHCI host controllers.
			 */
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_HVP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_HOST_CTRL_HW;
			break;
		case 2:
			pdata = &mrfl_sdhci_pci_data[SD_INDEX];
			/* REVERTME: disable SD card temporarily for bring up
			 * revert this change once GPIO driver is updated
			 * with Merrifield specific changes
			 * if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_HVP)
			 */
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_HOST_CTRL_HW;
			break;
		case 3:
			pdata = &mrfl_sdhci_pci_data[SDIO_INDEX];
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_HVP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_HOST_CTRL_HW;
				pdata->quirks = sdhci_pdata_quirks;
				pdata->register_embedded_control =
					sdhci_embedded_control;
			break;
		default:
			pr_err("%s func %s: Invalid PCI Dev func no. (%d)\n",
				__FILE__, __func__, PCI_FUNC(pdev->devfn));
			break;
		}
		break;
	case PCI_DEVICE_ID_INTEL_BYT_SDIO:
		pr_err("setting quirks/embedded controls on SDIO");
		pdata = &byt_sdhci_pci_data[SDIO_INDEX];
		pdata->quirks = sdhci_pdata_quirks;
		pdata->register_embedded_control = sdhci_embedded_control;
		break;
	default:
		break;
	}
	return pdata;
}

int sdhci_pdata_set_embedded_control(void (*fnp)
			(void *dev_id, void (*virtual_cd)
			(void *dev_id, int card_present)))
{
	WARN_ON(sdhci_embedded_control);
	sdhci_embedded_control = fnp;
	return 0;
}

struct sdhci_pci_data *mmc_sdhci_pci_get_data(struct pci_dev *pci_dev, int slotno)
{
	return get_sdhci_platform_data(pci_dev);
}

static int __init init_sdhci_get_data(void)
{
	sdhci_pci_get_data = mmc_sdhci_pci_get_data;

	return 0;
}

arch_initcall(init_sdhci_get_data);

