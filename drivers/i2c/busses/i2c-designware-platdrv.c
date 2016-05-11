/*
 * Synopsys DesignWare I2C adapter driver (master only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/pci.h>
#include <linux/platform_data/i2c-designware.h>
#include <asm/intel-mid.h>
#include "i2c-designware-core.h"

static int dw_i2c_plat_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct dw_i2c_dev *i2c = platform_get_drvdata(pdev);

	dev_dbg(dev, "suspend called\n");
	return i2c_dw_suspend(i2c, false);
}

#ifdef CONFIG_ACPI
/*
 * The HCNT/LCNT information coming from ACPI should be the most accurate
 * for given platform. However, some systems get it wrong. On such systems
 * we get better results by calculating those based on the input clock.
 */
static const struct dmi_system_id dw_i2c_no_acpi_params[] = {
	{
		.ident = "Dell Inspiron 7348",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Dell Inc."),
			DMI_MATCH(DMI_PRODUCT_NAME, "Inspiron 7348"),
		},
	},
	{ }
};

static void dw_i2c_acpi_params(struct platform_device *pdev, char method[],
			       u16 *hcnt, u16 *lcnt, u32 *sda_hold)
{
	struct acpi_buffer buf = { ACPI_ALLOCATE_BUFFER };
	acpi_handle handle = ACPI_HANDLE(&pdev->dev);
	union acpi_object *obj;

	if (dmi_check_system(dw_i2c_no_acpi_params))
		return;

	if (ACPI_FAILURE(acpi_evaluate_object(handle, method, NULL, &buf)))
		return;

	obj = (union acpi_object *)buf.pointer;
	if (obj->type == ACPI_TYPE_PACKAGE && obj->package.count == 3) {
		const union acpi_object *objs = obj->package.elements;

		*hcnt = (u16)objs[0].integer.value;
		*lcnt = (u16)objs[1].integer.value;
		if (sda_hold)
			*sda_hold = (u32)objs[2].integer.value;
	}

	kfree(buf.pointer);
}

static void dw_i2c_acpi_unconfigure(struct platform_device *pdev)
{
        struct dw_i2c_dev *dev = platform_get_drvdata(pdev);
        const struct acpi_device_id *id;

        id = acpi_match_device(pdev->dev.driver->acpi_match_table, &pdev->dev);
        if (id && id->driver_data)
                clk_unregister(dev->clk);
}

#else
static inline int dw_i2c_acpi_configure(struct platform_device *pdev)
{
	return -ENODEV;
}
static inline void dw_i2c_acpi_unconfigure(struct platform_device *pdev) { }
#endif

static int dw_i2c_plat_runtime_suspend(struct device *dev)
{
	const struct acpi_device_id *id;

	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct dw_i2c_dev *i2c = platform_get_drvdata(pdev);

	dev_dbg(dev, "runtime suspend called\n");
	i2c_dw_suspend(i2c, true);

	dev->adapter.nr = -1;
	dev->tx_fifo_depth = 32;
	dev->rx_fifo_depth = 32;

	/*
	 * Try to get SDA hold time and *CNT values from an ACPI method if
	 * it exists for both supported speed modes.
	 */
	dw_i2c_acpi_params(pdev, "SSCN", &dev->ss_hcnt, &dev->ss_lcnt, NULL);
	dw_i2c_acpi_params(pdev, "FMCN", &dev->fs_hcnt, &dev->fs_lcnt,
			   &dev->sda_hold_time);

	id = acpi_match_device(pdev->dev.driver->acpi_match_table, &pdev->dev);
	if (id && id->driver_data)
		dev->accessor_flags |= (u32)id->driver_data;

	return 0;
}

static int dw_i2c_plat_runtime_resume(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct dw_i2c_dev *i2c = platform_get_drvdata(pdev);

	dev_dbg(dev, "runtime resume called\n");
	i2c_dw_resume(i2c, true);

	return 0;
}

static int dw_i2c_plat_probe(struct platform_device *pdev)
{
	struct dw_i2c_dev *dev;
	struct resource *mem, *ioarea;
	unsigned long start, len;
	int bus_idx = 0;
	int irq;

#ifdef CONFIG_ACPI
	for (id = dw_i2c_acpi_ids; id->id[0]; id++)
		if (!strncmp(id->id, dev_name(&pdev->dev), strlen(id->id))) {
			bus_idx = id->driver_data + bus_num;
			bus_num++;
		}

#else
	bus_idx = pdev->id;
#endif

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -EINVAL;
        }
        start = mem->start;
        len = resource_size(mem);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
 	}

	dev = i2c_dw_setup(&pdev->dev, bus_idx, start, len, irq);
	if (IS_ERR(dev)) {
		release_mem_region(mem->start, resource_size(mem));
		dev_err(&pdev->dev, "failed to setup i2c\n");
		return -EINVAL;
 	}

	platform_set_drvdata(pdev, dev);
	acpi_i2c_register_devices(&dev->adapter);

 	pm_runtime_set_active(&pdev->dev);
 	pm_runtime_enable(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
 
 	return 0;
}


#ifdef CONFIG_PM_SLEEP
static int dw_i2c_plat_prepare(struct device *dev)
{
	return pm_runtime_suspended(dev);
}

static void dw_i2c_plat_complete(struct device *dev)
{
	if (dev->power.direct_complete)
		pm_request_resume(dev);
}
#else
#define dw_i2c_plat_prepare	NULL
#define dw_i2c_plat_complete	NULL
#endif

static int __exit dw_i2c_remove(struct platform_device *pdev)
{
	struct dw_i2c_dev *dev = platform_get_drvdata(pdev);
	struct resource *mem;
 
	pm_runtime_forbid(&pdev->dev);
	i2c_dw_free(&pdev->dev, dev);
	platform_set_drvdata(pdev, NULL);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem)
		release_mem_region(mem->start, resource_size(mem));
	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops dw_i2c_dev_pm_ops = {
	.prepare = dw_i2c_plat_prepare,
	.complete = dw_i2c_plat_complete,
	SET_SYSTEM_SLEEP_PM_OPS(dw_i2c_plat_suspend, dw_i2c_plat_resume)
	SET_RUNTIME_PM_OPS(dw_i2c_plat_suspend, dw_i2c_plat_resume, NULL)
};

#define DW_I2C_DEV_PMOPS (&dw_i2c_dev_pm_ops)
#else
#define DW_I2C_DEV_PMOPS NULL
#endif

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:i2c_designware");

static struct platform_driver dw_i2c_driver = {
	.probe = dw_i2c_plat_probe,
	.remove = dw_i2c_plat_remove,
	.driver		= {
		.name	= "i2c_designware",
		.pm	= DW_I2C_DEV_PMOPS,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(dw_i2c_acpi_match),
#endif
	},
};

static int __init dw_i2c_init_driver(void)
{
	struct pci_dev *dw_pci;

	/*
	 * Try to get pci device, if exist, then exit ACPI platform
	 * register, On BYT FDK, include two enum mode: PCI, ACPI,
	 * ignore ACPI enum mode.
	 */
	dw_pci = pci_get_device(PCI_VENDOR_ID_INTEL, 0x0F41, NULL);
	if (dw_pci) {
		pr_info("DW I2C: Find I2C controller in PCI device, "
			"exit ACPI platform register!\n");
		return 0;
	}

	return platform_driver_register(&dw_i2c_driver);
}
module_init(dw_i2c_init_driver);

static void __exit dw_i2c_exit_driver(void)
{
	platform_driver_unregister(&dw_i2c_driver);
}
module_exit(dw_i2c_exit_driver);

MODULE_AUTHOR("Baruch Siach <baruch@tkos.co.il>");
MODULE_DESCRIPTION("Synopsys DesignWare I2C bus adapter");
MODULE_LICENSE("GPL");
