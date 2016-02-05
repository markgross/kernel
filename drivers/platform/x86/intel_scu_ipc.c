/*
 * intel_scu_ipc.c: Driver for the Intel SCU IPC mechanism
 *
 * (C) Copyright 2008-2010,2015 Intel Corporation
 * Author: Sreedhara DS (sreedhara.ds@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * SCU running in ARC processor communicates with other entity running in IA
 * core through IPC mechanism which in turn messaging between IA core ad SCU.
 * SCU has two IPC mechanism IPC-1 and IPC-2. IPC-1 is used between IA32 and
 * SCU where IPC-2 is used between P-Unit and SCU. This driver delas with
 * IPC-1 Driver provides an API for power control unit registers (e.g. MSIC)
 * along with other APIs.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/pm.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/sfi.h>
#include <linux/module.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipc.h>
#include <linux/pm_qos.h>
#include <linux/intel_mid_pm.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/notifier.h>
#include <linux/suspend.h>

enum {
	SCU_IPC_LINCROFT,
	SCU_IPC_PENWELL,
	SCU_IPC_CLOVERVIEW,
	SCU_IPC_TANGIER,
};

static int  scu_ipc_pm_callback(struct notifier_block *nb,
					unsigned long action,
					void *ignored);

static struct notifier_block scu_ipc_pm_notifier = {
	.notifier_call = scu_ipc_pm_callback,
	.priority = 1,
};

/* intel scu ipc driver data */
struct intel_scu_ipc_pdata_t {
	u32 i2c_base;
	u32 i2c_len;
	u8 irq_mode;
};

static struct intel_scu_ipc_pdata_t intel_scu_ipc_lincroft_pdata = {
	.i2c_base = 0xff12b000,
	.i2c_len = 0x10,
	.irq_mode = 0,
};

/* Penwell and Cloverview */
static struct intel_scu_ipc_pdata_t intel_scu_ipc_penwell_pdata = {
	.i2c_base = 0xff12b000,
	.i2c_len = 0x10,
	.irq_mode = 1,
};

static struct intel_scu_ipc_pdata_t intel_scu_ipc_tangier_pdata = {
	.i2c_base  = 0xff00d000,
	.i2c_len = 0x10,
	.irq_mode = 0,
};

static int ipc_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void ipc_remove(struct pci_dev *pdev);

struct intel_scu_ipc_dev {
	struct pci_dev *pdev;
	void __iomem *ipc_base;
	void __iomem *i2c_base;
	int ioc;
	int cmd;
	struct completion cmd_complete;
	u8 irq_mode;
};

static struct intel_scu_ipc_dev ipcdev; /* Only one for now */

static DEFINE_MUTEX(ipclock); /* lock used to prevent multiple call to SCU */

/* PM Qos struct */
static struct pm_qos_request *qos;

/* Suspend status*/
static bool suspend_status;
static DEFINE_MUTEX(scu_suspend_lock);

/* Suspend status get */
bool suspend_in_progress(void)
{
	return suspend_status;
}

/* Suspend status set */
void set_suspend_status(bool status)
{
	mutex_lock(&scu_suspend_lock);
	suspend_status = status;
	mutex_unlock(&scu_suspend_lock);
}

/* IPC PM notifier callback */
static int scu_ipc_pm_callback(struct notifier_block *nb,
					unsigned long action,
					void *ignored)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
		set_suspend_status(true);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		set_suspend_status(false);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

/*
 * Command Register (Write Only):
 * A write to this register results in an interrupt to the SCU core processor
 * Format:
 * |rfu2(8) | size(8) | command id(4) | rfu1(3) | ioc(1) | command(8)|
 */
void ipc_command(u32 cmd) /* Send ipc command */
{
	if (ipcdev.irq_mode) {
		reinit_completion(&ipcdev.cmd_complete);
		writel(cmd | IPC_IOC, ipcdev.ipc_base);
	}
	writel(cmd, ipcdev.ipc_base);
}

/*
 * IPC Write Buffer (Write Only):
 * 16-byte buffer for sending data associated with IPC command to
 * SCU. Size of the data is specified in the IPC_COMMAND_REG register
 */
static inline void ipc_data_writel(u32 data, u32 offset) /* Write ipc data */
{
	writel(data, ipcdev.ipc_base + IPC_WRITE_BUFFER + offset);
}

/*
 * Status Register (Read Only):
 * Driver will read this register to get the ready/busy status of the IPC
 * block and error status of the IPC command that was just processed by SCU
 * Format:
 * |rfu3(8)|error code(8)|initiator id(8)|cmd id(4)|rfu1(2)|error(1)|busy(1)|
 */
static inline u8 ipc_read_status(void)
{
	return __raw_readl(ipcdev.ipc_base + IPC_STATUS_ADDR);
}

static inline u8 ipc_data_readb(u32 offset) /* Read ipc byte data */
{
	return readb(ipcdev.ipc_base + IPC_READ_BUFFER + offset);
}

static inline u32 ipc_data_readl(u32 offset) /* Read ipc u32 data */
{
	return readl(ipcdev.ipc_base + IPC_READ_BUFFER + offset);
}

/* Wait till scu status is busy */
static inline int busy_loop(void)
{
	u32 status = ipc_read_status();
	u32 loop_count = 100000;

	/* break if scu doesn't reset busy bit after huge retry */
	while ((status & BIT(0)) && --loop_count) {
		udelay(1); /* scu processing time is in few u secods */
		status = ipc_read_status();
	}

	if (status & BIT(0)) {
		dev_err(&ipcdev.pdev->dev, "IPC timed out");
		return -ETIMEDOUT;
	}

	if (status & BIT(1))
		return -EIO;

	return 0;
}

/* Wait till ipc ioc interrupt is received or timeout in 3 HZ */
static inline int ipc_wait_for_interrupt(void)
{
	int status;

	if (!wait_for_completion_timeout(&ipcdev.cmd_complete, 3 * HZ)) {
		struct device *dev = &ipcdev.pdev->dev;
		dev_err(dev, "IPC timed out\n");
		return -ETIMEDOUT;
	}

	status = ipc_read_status();
	if (status & BIT(1))
		return -EIO;

	return 0;
}

int intel_scu_ipc_check_status(void)
{
	return ipcdev.irq_mode ? ipc_wait_for_interrupt() : busy_loop();
}

/* Read/Write power control(PMIC in Langwell, MSIC in PenWell) registers */
static int pwr_reg_rdwr(u16 *addr, u8 *data, u32 count, u32 op, u32 id)
{
        int nc;
        u32 offset = 0;
        int err;
        u8 cbuf[IPC_WWBUF_SIZE] = { };
        u32 *wbuf = (u32 *)&cbuf;

        mutex_lock(&ipclock);

        memset(cbuf, 0, sizeof(cbuf));

        if (ipcdev.pdev == NULL) {
                mutex_unlock(&ipclock);
                return -ENODEV;
        }

        for (nc = 0; nc < count; nc++, offset += 2) {
                cbuf[offset] = addr[nc];
                cbuf[offset + 1] = addr[nc] >> 8;
        }

	if (id == IPC_CMD_PCNTRL_R) {
		for (nc = 0, offset = 0; nc < count; nc++, offset += 4)
			ipc_data_writel(wbuf[nc], offset);
		ipc_command((count * 2) << 16 | id << 12 | 0 << 8 | op);
	} else if (id == IPC_CMD_PCNTRL_W) {
		for (nc = 0; nc < count; nc++, offset += 1)
			cbuf[offset] = data[nc];
		for (nc = 0, offset = 0; nc < count; nc++, offset += 4)
			ipc_data_writel(wbuf[nc], offset);
		ipc_command((count * 3) << 16 | id << 12 | 0 << 8 | op);
	} else if (id == IPC_CMD_PCNTRL_M) {
		cbuf[offset] = data[0];
		cbuf[offset + 1] = data[1];
		ipc_data_writel(wbuf[0], 0); /* Write wbuff */
		ipc_command(4 << 16 | id << 12 | 0 << 8 | op);
	}

	err = intel_scu_ipc_check_status();
	if (!err && id == IPC_CMD_PCNTRL_R) { /* Read rbuf */
		/* Workaround: values are read as 0 without memcpy_fromio */
		memcpy_fromio(cbuf, ipcdev.ipc_base + 0x90, 16);
		for (nc = 0; nc < count; nc++)
			data[nc] = ipc_data_readb(nc);
	}
	mutex_unlock(&ipclock);
	return err;
}

void intel_scu_ipc_lock(void)
{
	/* Prevent C-states beyond C6 */
	pm_qos_update_request(qos, CSTATE_EXIT_LATENCY_S0i1 - 1);

	/* Prevent S3 */
	mutex_lock(&scu_suspend_lock);

}
EXPORT_SYMBOL_GPL(intel_scu_ipc_lock);

void intel_scu_ipc_unlock(void)
{
        /* Re-enable S3 */
        mutex_unlock(&scu_suspend_lock);

        /* Re-enable Deeper C-states beyond C6 */
        pm_qos_update_request(qos, PM_QOS_DEFAULT_VALUE);
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_unlock);

/**
 *	intel_scu_ipc_ioread8		-	read a word via the SCU
 *	@addr: register on SCU
 *	@data: return pointer for read byte
 *
 *	Read a single register. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_ioread8(u16 addr, u8 *data)
{
	return pwr_reg_rdwr(&addr, data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread8);

/**
 *	intel_scu_ipc_ioread16		-	read a word via the SCU
 *	@addr: register on SCU
 *	@data: return pointer for read word
 *
 *	Read a register pair. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_ioread16(u16 addr, u16 *data)
{
	u16 x[2] = {addr, addr + 1};
	return pwr_reg_rdwr(x, (u8 *)data, 2, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread16);

/**
 *	intel_scu_ipc_ioread32		-	read a dword via the SCU
 *	@addr: register on SCU
 *	@data: return pointer for read dword
 *
 *	Read four registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_ioread32(u16 addr, u32 *data)
{
	u16 x[4] = {addr, addr + 1, addr + 2, addr + 3};
	return pwr_reg_rdwr(x, (u8 *)data, 4, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread32);

/**
 *	intel_scu_ipc_iowrite8		-	write a byte via the SCU
 *	@addr: register on SCU
 *	@data: byte to write
 *
 *	Write a single register. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_iowrite8(u16 addr, u8 data)
{
	return pwr_reg_rdwr(&addr, &data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite8);

/**
 *	intel_scu_ipc_iowrite16		-	write a word via the SCU
 *	@addr: register on SCU
 *	@data: word to write
 *
 *	Write two registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_iowrite16(u16 addr, u16 data)
{
	u16 x[2] = {addr, addr + 1};
	return pwr_reg_rdwr(x, (u8 *)&data, 2, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite16);

/**
 *	intel_scu_ipc_iowrite32		-	write a dword via the SCU
 *	@addr: register on SCU
 *	@data: dword to write
 *
 *	Write four registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_iowrite32(u16 addr, u32 data)
{
	u16 x[4] = {addr, addr + 1, addr + 2, addr + 3};
	return pwr_reg_rdwr(x, (u8 *)&data, 4, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite32);

/**
 *	intel_scu_ipc_readvv		-	read a set of registers
 *	@addr: register list
 *	@data: bytes to return
 *	@len: length of array
 *
 *	Read registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	The largest array length permitted by the hardware is 5 items.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_readv(u16 *addr, u8 *data, int len)
{
	return pwr_reg_rdwr(addr, data, len, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_readv);

/**
 *	intel_scu_ipc_writev		-	write a set of registers
 *	@addr: register list
 *	@data: bytes to write
 *	@len: length of array
 *
 *	Write registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	The largest array length permitted by the hardware is 5 items.
 *
 *	This function may sleep.
 *
 */
int intel_scu_ipc_writev(u16 *addr, u8 *data, int len)
{
	return pwr_reg_rdwr(addr, data, len, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_writev);

/**
 *	intel_scu_ipc_update_register	-	r/m/w a register
 *	@addr: register address
 *	@bits: bits to update
 *	@mask: mask of bits to update
 *
 *	Read-modify-write power control unit register. The first data argument
 *	must be register value and second is mask value
 *	mask is a bitmap that indicates which bits to update.
 *	0 = masked. Don't modify this bit, 1 = modify this bit.
 *	returns 0 on success or an error code.
 *
 *	This function may sleep. Locking between SCU accesses is handled
 *	for the caller.
 */
int intel_scu_ipc_update_register(u16 addr, u8 bits, u8 mask)
{
	u8 data[2] = { bits, mask };
	return pwr_reg_rdwr(&addr, data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_M);
}
EXPORT_SYMBOL(intel_scu_ipc_update_register);

/**
 *	intel_scu_ipc_simple_command	-	send a simple command
 *	@cmd: command
 *	@sub: sub type
 *
 *	Issue a simple command to the SCU. Do not use this interface if
 *	you must then access data as any data values may be overwritten
 *	by another SCU access by the time this function returns.
 *
 *	This function may sleep. Locking for SCU accesses is handled for
 *	the caller.
 */
int intel_scu_ipc_simple_command(int cmd, int sub)
{
	int err;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}
	ipc_command(sub << 12 | cmd);
	err = intel_scu_ipc_check_status();
	mutex_unlock(&ipclock);
	return err;
}
EXPORT_SYMBOL(intel_scu_ipc_simple_command);

/**
 * intel_scu_ipc_raw_cmd - raw ipc command with data
 * @cmd: command
 * @sub: sub type
 * @in: input data
 * @inlen: input length in bytes
 * @out: output data
 * @outlen: output length in dwords
 * @sptr: data writing to SPTR register
 * @dptr: data writing to DPTR register
 *
 * Issue a command to the SCU which involves data transfers. Do the
 * data copies under the lock but leave it for the caller to interpret
 */
int intel_scu_ipc_raw_cmd(u32 cmd, u32 sub, u32 *in, u32 inlen, u32 *out,
		u32 outlen, u32 dptr, u32 sptr)
{
	int i, err;
	u32 wbuf[4] = { 0 };

	if (ipcdev.pdev == NULL)
		return -ENODEV;

	if (inlen > 16)
		return -EINVAL;

	mutex_lock(&ipclock);
	memcpy(wbuf, (u8 *)in, inlen);

	writel(dptr, ipcdev.ipc_base + IPC_DPTR_ADDR);
	writel(sptr, ipcdev.ipc_base + IPC_SPTR_ADDR);

	/**
	 * SRAM controller doesn't support 8bit write, it only supports
	 * 32bit write, so we have to write into the WBUF in 32bit,
	 * and SCU FW will use the inlen to determine the actual input
	 * data length in the WBUF.
	 */
	for (i = 0; i < ((inlen + 3) / 4); i++)
		ipc_data_writel(wbuf[i], 4 * i);

	/**
	 * Watchdog IPC command is an exception here using double word
	 * as the unit of input data size because of historical reasons
	 * and SCU FW is doing so.
	 */
	if ((cmd & 0xFF) == IPCMSG_WATCHDOG_TIMER)
		inlen = (inlen + 3) / 4;
	/*
	 *  In case of 3 pmic writes or read-modify-writes
	 *  there are holes in the middle of the buffer which are
	 *  ignored by SCU. These bytes should not be included into
	 *  size of the ipc msg. Holes are as follows:
	 *  write: wbuf[6 & 7]
	 *  read-modifu-write: wbuf[6 & 7 & 11]
	 */
	else if ((cmd & 0xFF) == IPCMSG_PCNTRL) {
		if (sub == IPC_CMD_PCNTRL_W && inlen == 11)
			inlen -= 2;
		else if (sub == IPC_CMD_PCNTRL_M && inlen == 15)
			inlen -= 3;
	}
	ipc_command((inlen << 16) | (sub << 12) | cmd);
	err = intel_scu_ipc_check_status();

	for (i = 0; i < outlen; i++)
		*out++ = ipc_data_readl(4 * i);

	mutex_unlock(&ipclock);
	return err;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_raw_cmd);

int intel_scu_ipc_command(int cmd, int sub, u32 *in, int inlen,
						u32 *out, int outlen)
{
	int i, err;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}

	for (i = 0; i < inlen; i++)
		ipc_data_writel(*in++, 4 * i);

	ipc_command((inlen << 16) | (sub << 12) | cmd);
	err = intel_scu_ipc_check_status();

	if (!err) {
		for (i = 0; i < outlen; i++)
			*out++ = ipc_data_readl(4 * i);
	}

	mutex_unlock(&ipclock);
	return err;

}
EXPORT_SYMBOL_GPL(intel_scu_ipc_command);
/* I2C commands */
#define IPC_I2C_WRITE 1 /* I2C Write command */
#define IPC_I2C_READ  2 /* I2C Read command */

/**
 *	intel_scu_ipc_i2c_cntrl		-	I2C read/write operations
 *	@addr: I2C address + command bits
 *	@data: data to read/write
 *
 *	Perform an an I2C read/write operation via the SCU. All locking is
 *	handled for the caller. This function may sleep.
 *
 *	Returns an error code or 0 on success.
 *
 *	This has to be in the IPC driver for the locking.
 */
int intel_scu_ipc_i2c_cntrl(u32 addr, u32 *data)
{
	u32 cmd = 0;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}
	cmd = (addr >> 24) & 0xFF;
	if (cmd == IPC_I2C_READ) {
		writel(addr, ipcdev.i2c_base + IPC_I2C_CNTRL_ADDR);
		/* Write not getting updated without delay */
		mdelay(1);
		*data = readl(ipcdev.i2c_base + I2C_DATA_ADDR);
	} else if (cmd == IPC_I2C_WRITE) {
		writel(*data, ipcdev.i2c_base + I2C_DATA_ADDR);
		mdelay(1);
		writel(addr, ipcdev.i2c_base + IPC_I2C_CNTRL_ADDR);
	} else {
		dev_err(&ipcdev.pdev->dev,
			"intel_scu_ipc: I2C INVALID_CMD = 0x%x\n", cmd);

		mutex_unlock(&ipclock);
		return -EIO;
	}
	mutex_unlock(&ipclock);
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_i2c_cntrl);

/*
 * Interrupt handler gets called when ioc bit of IPC_COMMAND_REG set to 1
 * When ioc bit is set to 1, caller api must wait for interrupt handler called
 * which in turn unlocks the caller api.
 *
 * This is edge triggered so we need take no action to clear anything
 */
static irqreturn_t ioc(int irq, void *dev_id)
{
	if (ipcdev.irq_mode)
		complete(&ipcdev.cmd_complete);

	return IRQ_HANDLED;
}

/**
 *	ipc_probe - probe an Intel SCU IPC
 *	@dev: the PCI device matching
 *	@id: entry in the match table
 *
 *	Enable and install an intel SCU IPC. This appears in the PCI space
 *	but uses some hard coded addresses as well.
 */
static int ipc_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int err;
	struct intel_scu_ipc_pdata_t *pdata;
	resource_size_t base;

	if (ipcdev.pdev)		/* We support only one SCU */
		return -EBUSY;

	pdata = (struct intel_scu_ipc_pdata_t *)id->driver_data;

	ipcdev.pdev = pci_dev_get(dev);
	ipcdev.irq_mode = pdata->irq_mode;

	err = pci_enable_device(dev);
	if (err)
		return err;

	err = pci_request_regions(dev, "intel_scu_ipc");
	if (err)
		return err;

	base = pci_resource_start(dev, 0);
	if (!base)
		return -ENOMEM;

	init_completion(&ipcdev.cmd_complete);

	if (request_irq(dev->irq, ioc, 0, "intel_scu_ipc", &ipcdev))
		return -EBUSY;

	ipcdev.ipc_base = ioremap_nocache(base, pci_resource_len(dev, 0));
	if (!ipcdev.ipc_base)
		return -ENOMEM;

	ipcdev.i2c_base = ioremap_nocache(pdata->i2c_base, pdata->i2c_len);
	if (!ipcdev.i2c_base) {
		iounmap(ipcdev.ipc_base);
		return -ENOMEM;
	}

	intel_scu_devices_create();

	return 0;
}

/**
 *	ipc_remove - remove a bound IPC device
 *	@pdev: PCI device
 *
 *	In practice the SCU is not removable but this function is also
 *	called for each device on a module unload or cleanup which is the
 *	path that will get used.
 *
 *	Free up the mappings and release the PCI resources
 */
static void ipc_remove(struct pci_dev *pdev)
{
	free_irq(pdev->irq, &ipcdev);
	pci_release_regions(pdev);
	pci_dev_put(ipcdev.pdev);
	iounmap(ipcdev.ipc_base);
	iounmap(ipcdev.i2c_base);
	ipcdev.pdev = NULL;
	intel_scu_devices_destroy();
}

static const struct pci_device_id pci_ids[] = {
	{
		PCI_VDEVICE(INTEL, PCI_DEVICE_ID_LINCROFT),
		(kernel_ulong_t)&intel_scu_ipc_lincroft_pdata,
	}, {
		PCI_VDEVICE(INTEL, PCI_DEVICE_ID_PENWELL),
		(kernel_ulong_t)&intel_scu_ipc_penwell_pdata,
	}, {
		PCI_VDEVICE(INTEL, PCI_DEVICE_ID_CLOVERVIEW),
		(kernel_ulong_t)&intel_scu_ipc_penwell_pdata,
	}, {
		PCI_VDEVICE(INTEL, PCI_DEVICE_ID_TANGIER),
		(kernel_ulong_t)&intel_scu_ipc_tangier_pdata,
	}, {
		0,
	}
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static struct pci_driver ipc_driver = {
	.name = "intel_scu_ipc",
	.id_table = pci_ids,
	.probe = ipc_probe,
	.remove = ipc_remove,
};

static int __init intel_scu_ipc_init(void)
{
	int platform;		/* Platform type */

	platform = intel_mid_identify_cpu();
	if (platform == 0)
		return -ENODEV;

	qos = kzalloc(sizeof(struct pm_qos_request), GFP_KERNEL);
	if (!qos)
		return -ENOMEM;

	pm_qos_add_request(qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	register_pm_notifier(&scu_ipc_pm_notifier);

	return  pci_register_driver(&ipc_driver);
}

static void __exit intel_scu_ipc_exit(void)
{
	pm_qos_remove_request(qos);

	pci_unregister_driver(&ipc_driver);
}

MODULE_AUTHOR("Sreedhara DS <sreedhara.ds@intel.com>");
MODULE_DESCRIPTION("Intel SCU IPC driver");
MODULE_LICENSE("GPL");

module_init(intel_scu_ipc_init);
module_exit(intel_scu_ipc_exit);
