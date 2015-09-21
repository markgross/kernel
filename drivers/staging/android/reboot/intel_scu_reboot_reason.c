/*
 * Reboot Reason driver for Edison, derived from SCURR driver for Medfield.
 *
 * Copyright (C) 2015,2011 Intel Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/reboot.h>
#include <linux/pagemap.h>
#include <linux/blkdev.h>
#include <linux/debugfs.h>
#include <linux/genhd.h>
#include <linux/seq_file.h>
#include <linux/rpmsg.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 1))
#include <linux/platform_data/intel_mid_remoteproc.h>
#else
#include <asm/intel_mid_remoteproc.h>
#endif
#include <linux/delay.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel-mid.h>

#include "reboot_target.h"

#include <asm/intel_scu_ipc.h>
#include <linux/power_supply.h>

static int scurr_reboot_target_call(const char *target, int id)
{
	int ret_ipc;

	pr_info("%s: notified [%s] target\n", __func__, target);

	pr_warn("[REBOOT] %s, rebooting into %s\n", __func__, target);

	ret_ipc = intel_scu_ipc_write_osnib_rr(id);
	if (ret_ipc < 0)
		pr_err("%s cannot write %s reboot reason in OSNIB\n",
			__func__, target);
	return NOTIFY_DONE;
}

static struct reboot_target scurr_reboot_target = {
	.set_reboot_target = scurr_reboot_target_call,
};

static int scurr_init(void)
{
	pr_info("%s: reboot_target registered\n", __func__);
	if (reboot_target_register(&scurr_reboot_target))
		pr_warning("scurr: unable to register reboot notifier");

	return 0;
}

static void scurr_exit(void)
{
	pr_info("%s: reboot_target unregistered\n", __func__);
	reboot_target_unregister(&scurr_reboot_target);
}

static int __init scurr_probe(struct platform_device *dev)
{
	return scurr_init();
}

static int scurr_remove(struct platform_device *dev)
{
	scurr_exit();
	return 0;
}

static struct platform_driver scurr_driver = {
	.remove         = scurr_remove,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = KBUILD_MODNAME,
	},
};

static int __init scurr_init_module(void)
{
	int err=0;

	pr_info("Intel SCURR Driver\n");

        platform_device_register_simple(KBUILD_MODNAME, -1, NULL, 0);
	err =  platform_driver_probe(&scurr_driver,scurr_probe);

	return err;
}

static void __exit scurr_cleanup_module(void)
{
	platform_driver_unregister(&scurr_driver);
	pr_info("SCURR Module Unloaded\n");
}
module_init(scurr_init_module);
module_exit(scurr_cleanup_module);

MODULE_AUTHOR("Pierre Tardy <pierre.tardy@intel.com>");
MODULE_AUTHOR("Xiaokang Qin <xiaokang.qin@intel.com>");
MODULE_DESCRIPTION("Intel SCU Reboot Reason Driver");
MODULE_LICENSE("GPL v2");
