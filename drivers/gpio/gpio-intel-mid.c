/*
 * Intel MID GPIO driver
 *
 * Copyright (c) 2008-2014 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* Supports:
 * Moorestown platform Langwell chip.
 * Medfield platform Penwell chip.
 * Clovertrail platform Cloverview chip.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/stddef.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gpio/driver.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_flis.h>
#include "gpiodebug.h"

#define INTEL_MID_IRQ_TYPE_EDGE		(1 << 0)
#define INTEL_MID_IRQ_TYPE_LEVEL	(1 << 1)

#define TANGIER_I2C_FLIS_START	0x1D00
#define TANGIER_I2C_FLIS_END	0x1D34

#define to_lnw_priv(chip)       container_of(chip, struct intel_mid_gpio, chip)

/*
 * Langwell chip has 64 pins and thus there are 2 32bit registers to control
 * each feature, while Penwell chip has 96 pins for each block, and need 3 32bit
 * registers to control them, so we only define the order here instead of a
 * structure, to get a bit offset for a pin (use GPDR as an example):
 *
 * nreg = ngpio / 32;
 * reg = offset / 32;
 * bit = offset % 32;
 * reg_addr = reg_base + GPDR * nreg * 4 + reg * 4;
 *
 * so the bit of reg_addr is to control pin offset's GPDR feature
*/

enum GPIO_REG {
	GPLR = 0,	/* pin level read-only */
	GPDR,		/* pin direction */
	GPSR,		/* pin set */
	GPCR,		/* pin clear */
	GRER,		/* rising edge detect */
	GFER,		/* falling edge detect */
	GEDR,		/* edge detect result */
	GAFR,		/* alt function */
       GFBR = 9,       /* glitch filter bypas */
       GPIT,           /* interrupt type */
       GPIP = GFER,    /* level interrupt polarity */
       GPIM = GRER,    /* level interrupt mask */

       /* the following registers only exist on MRFLD */
       GFBR_TNG = 6,
       GIMR,           /* interrupt mask */
       GISR,           /* interrupt source */
       GITR = 32,      /* interrupt type */
       GLPR = 33,      /* level-input polarity */
};

enum GPIO_CONTROLLERS {
       LINCROFT_GPIO,
       PENWELL_GPIO_AON,
       PENWELL_GPIO_CORE,
       CLOVERVIEW_GPIO_AON,
       CLOVERVIEW_GPIO_CORE,
       TANGIER_GPIO,
};

/* intel_mid gpio driver data */
struct intel_mid_gpio_ddata {
	u16 ngpio;		/* number of gpio pins */
	u32 chip_irq_type;	/* chip interrupt type */
};

struct intel_mid_gpio {
	struct gpio_chip		chip;
	void __iomem			*reg_base;
	void __iomem			*reg_gplr;
	spinlock_t			lock;
	struct pci_dev			*pdev;
	struct irq_domain		*domain;
	u32				(*get_flis_offset)(int gpio);
	u32				chip_irq_type;
	int				type;
	struct gpio_debug		*debug;
};

struct gpio_flis_pair {
       int gpio;       /* gpio number */
       int offset;     /* register offset from FLIS base */
};

/*
 * The following mapping table lists the pin and flis offset pair
 * of some key gpio pins, the offset of other gpios can be calculated
 * from the table.
 */
static struct gpio_flis_pair gpio_flis_mapping_table[] = {
       { 0,    0x2900 },
       { 12,   0x2544 },
       { 14,   0x0958 },
       { 16,   0x2D18 },
       { 17,   0x1D10 },
       { 19,   0x1D00 },
       { 23,   0x1D18 },
       { 31,   -EINVAL }, /* No GPIO 31 in pin list */
       { 32,   0x1508 },
       { 44,   0x3500 },
       { 64,   0x2534 },
       { 68,   0x2D1C },
       { 70,   0x1500 },
       { 72,   0x3D00 },
       { 77,   0x0D00 },
       { 97,   0x0954 },
       { 98,   -EINVAL }, /* No GPIO 98-101 in pin list */
       { 102,  0x1910 },
       { 120,  0x1900 },
       { 124,  0x2100 },
       { 136,  -EINVAL }, /* No GPIO 136 in pin list */
       { 137,  0x2D00 },
       { 143,  -EINVAL }, /* No GPIO 143-153 in pin list */
       { 154,  0x092C },
       { 164,  0x3900 },
       { 177,  0x2500 },
       { 190,  0x2D50 },
};

/*
 * In new version of FW for Merrifield, I2C FLIS register can not
 * be written directly but go though a IPC way which is sleepable,
 * so we shouldn't use spin_lock_irq to protect the access when
 * is_merr_i2c_flis() return true.
 */
static inline bool is_merr_i2c_flis(u32 offset)
{
	return ((offset >= TANGIER_I2C_FLIS_START)
		&& (offset <= TANGIER_I2C_FLIS_END));
}

static u32 get_flis_offset_by_gpio(int gpio)
{
       int i;
       int start;
       u32 offset = -EINVAL;

       for (i = 0; i < ARRAY_SIZE(gpio_flis_mapping_table) - 1; i++) {
               if (gpio >= gpio_flis_mapping_table[i].gpio
                       && gpio < gpio_flis_mapping_table[i + 1].gpio)
                       break;
       }

       start = gpio_flis_mapping_table[i].gpio;

       if (gpio_flis_mapping_table[i].offset != -EINVAL) {
               offset = gpio_flis_mapping_table[i].offset
                               + (gpio - start) * 4;
       }

       return offset;
}

static inline struct intel_mid_gpio *to_intel_gpio_priv(struct gpio_chip *gc)
{
	return container_of(gc, struct intel_mid_gpio, chip);
}

static void __iomem *gpio_reg(struct gpio_chip *chip, unsigned offset,
			      enum GPIO_REG reg_type)
{
	struct intel_mid_gpio *priv = to_intel_gpio_priv(chip);
	unsigned nreg = chip->ngpio / 32;
	u8 reg = offset / 32;

	return priv->reg_base + reg_type * nreg * 4 + reg * 4;
}

static int lnw_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
                                unsigned debounce)
{
       struct intel_mid_gpio *lnw = to_lnw_priv(chip);
       void __iomem *gfbr;
       unsigned long flags;
       u32 value;
       enum GPIO_REG reg_type;

       reg_type = (lnw->type == TANGIER_GPIO) ? GFBR_TNG : GFBR;
       gfbr = gpio_reg(chip, offset, reg_type);

       if (lnw->pdev)
               pm_runtime_get(&lnw->pdev->dev);

       spin_lock_irqsave(&lnw->lock, flags);
       value = readl(gfbr);
       if (debounce) {
               /* debounce bypass disable */
               value &= ~BIT(offset % 32);
       } else {
               /* debounce bypass enable */
               value |= BIT(offset % 32);
       }
       writel(value, gfbr);
       spin_unlock_irqrestore(&lnw->lock, flags);

       if (lnw->pdev)
               pm_runtime_put(&lnw->pdev->dev);

       return 0;
}

static void __iomem *gpio_reg_2bit(struct gpio_chip *chip, unsigned offset,
				   enum GPIO_REG reg_type)
{
	struct intel_mid_gpio *priv = to_intel_gpio_priv(chip);
	unsigned nreg = chip->ngpio / 32;
	u8 reg = offset / 16;

	return priv->reg_base + reg_type * nreg * 4 + reg * 4;
}

static int intel_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *gafr = gpio_reg_2bit(chip, offset, GAFR);
	u32 value = readl(gafr);
	int shift = (offset % 16) << 1, af = (value >> shift) & 3;

	if (af) {
		value &= ~(3 << shift);
		writel(value, gafr);
	}
	return 0;
}

static int intel_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *gplr = gpio_reg(chip, offset, GPLR);

	return readl(gplr) & BIT(offset % 32);
}

#define PULLUP_ENABLE   (1 << 8)
#define PULLDOWN_ENABLE (1 << 9)
#define PUPD_VAL_2K     (0 << 4)
#define PUPD_VAL_20K    (1 << 4)
#define PUPD_VAL_50K    (2 << 4)
#define PUPD_VAL_910    (3 << 4)

static int lnw_gpio_set_pull(struct gpio_chip *chip, unsigned gpio, int value)
{
	u32 flis_offset, flis_value;
	struct intel_mid_gpio *lnw = to_lnw_priv(chip);
	unsigned long flags;

	if (lnw->type != TANGIER_GPIO)
		return 0;

	flis_offset = lnw->get_flis_offset(gpio);
	if (WARN(flis_offset == -EINVAL, "invalid pin %d\n", gpio))
		return -EINVAL;
	if (is_merr_i2c_flis(flis_offset))
		return 0;
	spin_lock_irqsave(&lnw->lock, flags);
	flis_value = get_flis_value(flis_offset);
	if (value) {
		flis_value |= PULLUP_ENABLE;
		flis_value &= ~PULLDOWN_ENABLE;
	} else {
		flis_value |= PULLDOWN_ENABLE;
		flis_value &= ~PULLUP_ENABLE;
	}
	flis_value |= PUPD_VAL_50K;
	set_flis_value(flis_value, flis_offset);
	spin_unlock_irqrestore(&lnw->lock, flags);

	return 0;
}

static void intel_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	void __iomem *gpsr, *gpcr;

	lnw_gpio_set_pull(chip, offset, value);

	if (value) {
		gpsr = gpio_reg(chip, offset, GPSR);
		writel(BIT(offset % 32), gpsr);
	} else {
		gpcr = gpio_reg(chip, offset, GPCR);
		writel(BIT(offset % 32), gpcr);
	}
}

static int intel_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct intel_mid_gpio *priv = to_intel_gpio_priv(chip);
	void __iomem *gpdr = gpio_reg(chip, offset, GPDR);
	u32 value;
	unsigned long flags;

	if (priv->pdev)
		pm_runtime_get(&priv->pdev->dev);

	spin_lock_irqsave(&priv->lock, flags);
	value = readl(gpdr);
	value &= ~BIT(offset % 32);
	writel(value, gpdr);
	spin_unlock_irqrestore(&priv->lock, flags);

	if (priv->pdev)
		pm_runtime_put(&priv->pdev->dev);

	return 0;
}

static int intel_gpio_direction_output(struct gpio_chip *chip,
			unsigned offset, int value)
{
	struct intel_mid_gpio *priv = to_intel_gpio_priv(chip);
	void __iomem *gpdr = gpio_reg(chip, offset, GPDR);
	unsigned long flags;

	intel_gpio_set(chip, offset, value);

	if (priv->pdev)
		pm_runtime_get(&priv->pdev->dev);

	spin_lock_irqsave(&priv->lock, flags);
	value = readl(gpdr);
	value |= BIT(offset % 32);
	writel(value, gpdr);
	spin_unlock_irqrestore(&priv->lock, flags);

	if (priv->pdev)
		pm_runtime_put(&priv->pdev->dev);

	return 0;
}

static int intel_mid_irq_type(struct irq_data *d, unsigned type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct intel_mid_gpio *priv = to_intel_gpio_priv(gc);
	u32 gpio = irqd_to_hwirq(d);
	unsigned long flags;
	u32 value;
	int ret = 0;
	void __iomem *grer = gpio_reg(&priv->chip, gpio, GRER);
	void __iomem *gfer = gpio_reg(&priv->chip, gpio, GFER);
	void __iomem *gpit, *gpip;

	if (gpio >= priv->chip.ngpio)
		return -EINVAL;

	if (priv->pdev)
		pm_runtime_get(&priv->pdev->dev);

	/* Chip that supports level interrupt has extra GPIT registers */
	if (priv->chip_irq_type & INTEL_MID_IRQ_TYPE_LEVEL) {
		switch (priv->type) {
		case TANGIER_GPIO:
			gpit = gpio_reg(&priv->chip, gpio, GITR);
			gpip = gpio_reg(&priv->chip, gpio, GLPR);
			break;
		default:
			ret = -EINVAL;
			goto out;
		}
		spin_lock_irqsave(&priv->lock, flags);
		if (type & IRQ_TYPE_LEVEL_MASK) {
			/* To prevent glitches from triggering an unintended
			 * level interrupt, configure GLPR register first
			 * and then configure GITR.
			 */
			if (type & IRQ_TYPE_LEVEL_LOW)
				value = readl(gpip) | BIT(gpio % 32);
			else
				value = readl(gpip) & (~BIT(gpio % 32));
			writel(value, gpip);
 
			value = readl(gpit) | BIT(gpio % 32);
			writel(value, gpit);

			irq_set_handler_locked(d, handle_level_irq);
		} else if (type & IRQ_TYPE_EDGE_BOTH) {
			value = readl(gpit) & (~BIT(gpio % 32));
			writel(value, gpit);

			if (type & IRQ_TYPE_EDGE_RISING)
				value = readl(grer) | BIT(gpio % 32);
			else
				value = readl(grer) & (~BIT(gpio % 32));
			writel(value, grer);

			if (type & IRQ_TYPE_EDGE_FALLING)
				value = readl(gfer) | BIT(gpio % 32);
			else
				value = readl(gfer) & (~BIT(gpio % 32));
			writel(value, gfer);

			irq_set_handler_locked(d, handle_edge_irq);
		}
		spin_unlock_irqrestore(&priv->lock, flags);
	} else {
		if (type & IRQ_TYPE_LEVEL_MASK) {
			ret = -EINVAL;
		} else if (type & IRQ_TYPE_EDGE_BOTH) {
			spin_lock_irqsave(&priv->lock, flags);

			if (type & IRQ_TYPE_EDGE_RISING)
				value = readl(grer) | BIT(gpio % 32);
			else
				value = readl(grer) & (~BIT(gpio % 32));
			writel(value, grer);

			if (type & IRQ_TYPE_EDGE_FALLING)
				value = readl(gfer) | BIT(gpio % 32);
			else
				value = readl(gfer) & (~BIT(gpio % 32));
			writel(value, gfer);

			spin_unlock_irqrestore(&priv->lock, flags);
		}
	}

out:
	if (priv->pdev)
		pm_runtime_put(&priv->pdev->dev);

	return ret;
}

static int lnw_set_maskunmask(struct irq_data *d, enum GPIO_REG reg_type,
				unsigned unmask)
{
	struct intel_mid_gpio *priv = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	unsigned long flags;
	u32 value;
	void __iomem *gp_reg;

	gp_reg = gpio_reg(&priv->chip, gpio, reg_type);

	spin_lock_irqsave(&priv->lock, flags);

	if (unmask) {
		/* enable interrupt from GPIO input pin */
		value = readl(gp_reg) | BIT(gpio % 32);
	} else {
		/* disable interrupt from GPIO input pin */
		value = readl(gp_reg) & (~BIT(gpio % 32));
	}

	writel(value, gp_reg);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void intel_mid_irq_unmask(struct irq_data *d)
{
	struct intel_mid_gpio *priv = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);

	if (gpio >= priv->chip.ngpio)
		return;

	switch (priv->type) {
	case TANGIER_GPIO:
		lnw_set_maskunmask(d, GIMR, 1);
		break;
	default:
		break;
	}
}

static void intel_mid_irq_mask(struct irq_data *d)
{
	struct intel_mid_gpio *priv = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);

	if (gpio >= priv->chip.ngpio)
		return;

	switch (priv->type) {
	case TANGIER_GPIO:
		lnw_set_maskunmask(d, GIMR, 0);
		break;
	default:
		break;
	}
}

static int intel_irq_set_wake(struct irq_data *d, unsigned on)
{
	return 0;
}

static void intel_irq_ack(struct irq_data *d)
{
}

static void intel_irq_shutdown(struct irq_data *d)
{
	struct intel_mid_gpio *priv = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	unsigned long flags;
	u32 value;
	void __iomem *grer = gpio_reg(&priv->chip, gpio, GRER);
	void __iomem *gfer = gpio_reg(&priv->chip, gpio, GFER);

	spin_lock_irqsave(&priv->lock, flags);
	value = readl(grer) & (~BIT(gpio % 32));
	writel(value, grer);
	value = readl(gfer) & (~BIT(gpio % 32));
	writel(value, gfer);
	spin_unlock_irqrestore(&priv->lock, flags);
};


static struct irq_chip intel_mid_irqchip = {
	.name		= "INTEL_MID-GPIO",
	.flags		= IRQCHIP_SET_TYPE_MASKED,
	.irq_mask	= intel_mid_irq_mask,
	.irq_unmask	= intel_mid_irq_unmask,
	.irq_set_type	= intel_mid_irq_type,
	.irq_set_wake	= intel_irq_set_wake,
	.irq_ack	= intel_irq_ack,
	.irq_shutdown	= intel_irq_shutdown,
};

static const struct intel_mid_gpio_ddata gpio_lincroft = {
	.ngpio = 64,
};

static const struct intel_mid_gpio_ddata gpio_penwell_aon = {
	.ngpio = 96,
	.chip_irq_type = INTEL_MID_IRQ_TYPE_EDGE,
};

static const struct intel_mid_gpio_ddata gpio_penwell_core = {
	.ngpio = 96,
	.chip_irq_type = INTEL_MID_IRQ_TYPE_EDGE,
};

static const struct intel_mid_gpio_ddata gpio_cloverview_aon = {
	.ngpio = 96,
	.chip_irq_type = INTEL_MID_IRQ_TYPE_EDGE | INTEL_MID_IRQ_TYPE_LEVEL,
};

static const struct intel_mid_gpio_ddata gpio_cloverview_core = {
	.ngpio = 96,
	.chip_irq_type = INTEL_MID_IRQ_TYPE_EDGE,
};

static const struct pci_device_id intel_gpio_ids[] = {
	{
		/* Lincroft */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x080f),
		.driver_data = (kernel_ulong_t)&gpio_lincroft,
	},
	{
		/* Penwell AON */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x081f),
		.driver_data = (kernel_ulong_t)&gpio_penwell_aon,
	},
	{
		/* Penwell Core */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x081a),
		.driver_data = (kernel_ulong_t)&gpio_penwell_core,
	},
	{
		/* Cloverview Aon */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08eb),
		.driver_data = (kernel_ulong_t)&gpio_cloverview_aon,
	},
	{
		/* Cloverview Core */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08f7),
		.driver_data = (kernel_ulong_t)&gpio_cloverview_core,
	},
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, intel_gpio_ids);

static void intel_mid_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct intel_mid_gpio *priv = to_intel_gpio_priv(gc);
	struct irq_data *data = irq_desc_get_irq_data(desc);
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	struct gpio_debug *debug;
	u32 base, gpio, mask;
	unsigned long pending;
	void __iomem *gp_reg;
	enum GPIO_REG reg_type;
	struct irq_desc *priv_irq_desc;
	unsigned int priv_irq;

	debug = priv->debug;

	reg_type = (priv->type == TANGIER_GPIO) ? GISR : GEDR;

	/* check GPIO controller to check which pin triggered the interrupt */
	for (base = 0; base < priv->chip.ngpio; base += 32) {
		gp_reg = gpio_reg(&priv->chip, base, reg_type);
		while ((pending = (priv->type != TANGIER_GPIO) ?
			readl(gp_reg) :
			(readl(gp_reg) &
			readl(gpio_reg(&priv->chip, base, GIMR))))) {
			gpio = __ffs(pending);
			DEFINE_DEBUG_IRQ_CONUNT_INCREASE(priv->chip.base +
				base + gpio);
			/* Mask irq if not requested in kernel */
			priv_irq = irq_find_mapping(priv->domain, base + gpio);
			priv_irq_desc = irq_to_desc(priv_irq);
			if (priv_irq_desc && unlikely(!priv_irq_desc->action)) {
				intel_mid_irq_mask(&priv_irq_desc->irq_data);
				continue;
			}

			mask = BIT(gpio);
			/* Clear before handling so we can't lose an edge */
			writel(mask, gp_reg);
			generic_handle_irq(priv_irq);
		}
	}

	chip->irq_eoi(data);
}

static char conf_reg_msg[] =
       "\nGPIO configuration register:\n"
       "\t[ 2: 0]\tpinmux\n"
       "\t[ 6: 4]\tpull strength\n"
       "\t[ 8: 8]\tpullup enable\n"
       "\t[ 9: 9]\tpulldown enable\n"
       "\t[10:10]\tslew A, B setting\n"
       "\t[12:12]\toverride input enable\n"
       "\t[13:13]\toverride input enable enable\n"
       "\t[14:14]\toverride output enable\n"
       "\t[15:15]\toverride output enable enable\n"
       "\t[16:16]\toverride input value\n"
       "\t[17:17]\tenable input data override\n"
       "\t[18:18]\toverride output value\n"
       "\t[19:19]\tenable output data override\n"
       "\t[21:21]\topen drain enable\n"
       "\t[22:22]\tenable OVR_IOSTBY_VAL\n"
       "\t[23:23]\tOVR_IOSTBY_VAL\n"
       "\t[24:24]\tSBY_OUTDATAOV_EN\n"
       "\t[25:25]\tSBY_INDATAOV_EN\n"
       "\t[26:26]\tSBY_OVOUTEN_EN\n"
       "\t[27:27]\tSBY_OVINEN_EN\n"
       "\t[29:28]\tstandby pullmode\n"
       "\t[30:30]\tstandby open drain mode\n";

static char *pinvalue[] = {"low", "high"};
static char *pindirection[] = {"in", "out"};
static char *irqtype[] = {"irq_none", "edge_rising", "edge_falling",
                       "edge_both"};
static char *pinmux[] = {"mode0", "mode1", "mode2", "mode3", "mode4", "mode5",
                       "mode6", "mode7"};
static char *pullmode[] = {"nopull", "pullup", "pulldown"};
static char *pullstrength[] = {"2k", "20k", "50k", "910ohms"};
static char *enable[] = {"disable", "enable"};
static char *override_direction[] = {"no-override", "override-enable",
                       "override-disable"};
static char *override_value[] = {"no-override", "override-high",
                       "override-low"};
static char *standby_trigger[] = {"no-override", "override-trigger",
                       "override-notrigger"};
static char *standby_pupd_state[] = {"keep", "pulldown", "pullup", "nopull"};

static int gpio_get_pinvalue(struct gpio_control *control, void *private_data,
               unsigned gpio)
{
       struct intel_mid_gpio *lnw = private_data;
       u32 value;

       value = intel_gpio_get(&lnw->chip, gpio);

       return value ? 1 : 0;
}

static int gpio_set_pinvalue(struct gpio_control *control, void *private_data,
               unsigned gpio, unsigned int num)
{
       struct intel_mid_gpio *lnw = private_data;

       intel_gpio_set(&lnw->chip, gpio, num);

       return 0;
}

static int gpio_get_normal(struct gpio_control *control, void *private_data,
               unsigned gpio)
{
       struct intel_mid_gpio *lnw = private_data;
       u32 __iomem *mem;
       u32 value;

       mem = gpio_reg(&lnw->chip, gpio, control->reg);

       value = readl(mem);
       value &= BIT(gpio % 32);

       if (control->invert)
               return value ? 0 : 1;
       else
               return value ? 1 : 0;
}

static int gpio_set_normal(struct gpio_control *control, void *private_data,
               unsigned gpio, unsigned int num)
{
       struct intel_mid_gpio *lnw = private_data;
       u32 __iomem *mem;
       u32 value;
       unsigned long flags;

       mem = gpio_reg(&lnw->chip, gpio, control->reg);

       spin_lock_irqsave(&lnw->lock, flags);
       value = readl(mem);
       value &= ~BIT(gpio % 32);
       if (control->invert) {
               if (num)
                       value &= ~BIT(gpio % 32);
               else
                       value |= BIT(gpio % 32);
       } else {
               if (num)
                       value |= BIT(gpio % 32);
               else
                       value &= ~BIT(gpio % 32);
       }
       writel(value, mem);
       spin_unlock_irqrestore(&lnw->lock, flags);

       return 0;
}

static int gpio_get_irqtype(struct gpio_control *control, void *private_data,
               unsigned gpio)
{
       struct intel_mid_gpio *lnw = private_data;
       void __iomem *grer = gpio_reg(&lnw->chip, gpio, GRER);
       void __iomem *gfer = gpio_reg(&lnw->chip, gpio, GFER);
       u32 value;
       int num;

       value = readl(grer) & BIT(gpio % 32);
       num = value ? 1 : 0;
       value = readl(gfer) & BIT(gpio % 32);
       if (num)
               num = value ? 3 : 1;
       else
               num = value ? 2 : 0;

       return num;
}

static int flis_get_normal(struct gpio_control *control, void *private_data,
               unsigned gpio)
{
       struct intel_mid_gpio *lnw = private_data;
       u32 offset, value;
       int num;

       if (lnw->type == TANGIER_GPIO) {
               offset = lnw->get_flis_offset(gpio);
               if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
                       return -1;

               value = get_flis_value(offset);
               num = (value >> control->shift) & control->mask;
               if (num < control->num)
                       return num;
       }

       return -1;
}

static int flis_set_normal(struct gpio_control *control, void *private_data,
               unsigned gpio, unsigned int num)
{
       struct intel_mid_gpio *lnw = private_data;
       u32 shift = control->shift;
       u32 mask = control->mask;
       u32 offset, value;
       unsigned long flags;

       if (lnw->type == TANGIER_GPIO) {
		offset = lnw->get_flis_offset(gpio);
		if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
                       return -1;
		if (!is_merr_i2c_flis(offset))
	               spin_lock_irqsave(&lnw->lock, flags);

		value = get_flis_value(offset);
		value &= ~(mask << shift);
		value |= ((num & mask) << shift);
		set_flis_value(value, offset);

		if (!is_merr_i2c_flis(offset))
			spin_unlock_irqrestore(&lnw->lock, flags);

		return 0;
       }

       return -1;
}

static int flis_get_override(struct gpio_control *control, void *private_data,
               unsigned gpio)
{
       struct intel_mid_gpio *lnw = private_data;
       u32 offset, value;
       u32 val_bit, en_bit;
       int num;

       if (lnw->type == TANGIER_GPIO) {
               offset = lnw->get_flis_offset(gpio);
               if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
                       return -1;

               val_bit = 1 << control->shift;
               en_bit = 1 << control->rshift;

               value = get_flis_value(offset);

               if (value & en_bit)
                       if (value & val_bit)
                               num = 1;
                       else
                               num = 2;
               else
                       num = 0;

               return num;
       }

       return -1;
}

static int flis_set_override(struct gpio_control *control, void *private_data,
               unsigned gpio, unsigned int num)
{
	struct intel_mid_gpio *lnw = private_data;
	u32 offset, value;
	u32 val_bit, en_bit;
	unsigned long flags;

	if (lnw->type == TANGIER_GPIO) {
		offset = lnw->get_flis_offset(gpio);
		if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
			return -1;

		val_bit = 1 << control->shift;
		en_bit = 1 << control->rshift;
		if (!is_merr_i2c_flis(offset))
			spin_lock_irqsave(&lnw->lock, flags);
		value = get_flis_value(offset);
		switch (num) {
		case 0:
			value &= ~(en_bit | val_bit);
			break;
		case 1:
                       value |= (en_bit | val_bit);
                       break;
		case 2:
			value |= en_bit;
			value &= ~val_bit;
			break;
		default:
			break;
		}
		set_flis_value(value, offset);
		if (!is_merr_i2c_flis(offset))
			spin_unlock_irqrestore(&lnw->lock, flags);

		return 0;
	}

	return -1;
}

#define GPIO_VALUE_CONTROL(xtype, xinfo, xnum) \
{      .type = xtype, .pininfo = xinfo, .num = xnum, \
       .get = gpio_get_pinvalue, .set = gpio_set_pinvalue}
#define GPIO_NORMAL_CONTROL(xtype, xinfo, xnum, xreg, xinvert) \
{      .type = xtype, .pininfo = xinfo, .num = xnum, .reg = xreg, \
       .invert = xinvert, .get = gpio_get_normal, .set = gpio_set_normal}
#define GPIO_IRQTYPE_CONTROL(xtype, xinfo, xnum) \
{      .type = xtype, .pininfo = xinfo, .num = xnum, \
       .get = gpio_get_irqtype, .set = NULL}
#define FLIS_NORMAL_CONTROL(xtype, xinfo, xnum, xshift, xmask) \
{      .type = xtype, .pininfo = xinfo, .num = xnum, .shift = xshift, \
       .mask = xmask, .get = flis_get_normal, .set = flis_set_normal}
#define FLIS_OVERRIDE_CONTROL(xtype, xinfo, xnum, xshift, xrshift) \
{      .type = xtype, .pininfo = xinfo, .num = xnum, .shift = xshift, \
       .rshift = xrshift, .get = flis_get_override, .set = flis_set_override}

static struct gpio_control lnw_gpio_controls[] = {
GPIO_VALUE_CONTROL(TYPE_PIN_VALUE, pinvalue, 2),
GPIO_NORMAL_CONTROL(TYPE_DIRECTION, pindirection, 2, GPDR, 0),
GPIO_IRQTYPE_CONTROL(TYPE_IRQ_TYPE, irqtype, 4),
GPIO_NORMAL_CONTROL(TYPE_DEBOUNCE, enable, 2, GFBR_TNG, 1),
FLIS_NORMAL_CONTROL(TYPE_PINMUX, pinmux, 8, 0, 0x7),
FLIS_NORMAL_CONTROL(TYPE_PULLSTRENGTH, pullstrength, 4, 4, 0x7),
FLIS_NORMAL_CONTROL(TYPE_PULLMODE, pullmode, 3, 8, 0x3),
FLIS_NORMAL_CONTROL(TYPE_OPEN_DRAIN, enable, 2, 21, 0x1),
FLIS_OVERRIDE_CONTROL(TYPE_OVERRIDE_INDIR, override_direction, 3, 12, 13),
FLIS_OVERRIDE_CONTROL(TYPE_OVERRIDE_OUTDIR, override_direction, 3, 14, 15),
FLIS_OVERRIDE_CONTROL(TYPE_OVERRIDE_INVAL, override_value, 3, 16, 17),
FLIS_OVERRIDE_CONTROL(TYPE_OVERRIDE_OUTVAL, override_value, 3, 18, 19),
FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_IO, standby_trigger, 3, 23, 22),
FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_OUTVAL, override_value, 3, 18, 24),
FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_INVAL, override_value, 3, 16, 25),
FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_OUTDIR, override_direction, 3, 14, 26),
FLIS_OVERRIDE_CONTROL(TYPE_SBY_OVR_INDIR, override_direction, 3, 12, 27),
FLIS_NORMAL_CONTROL(TYPE_SBY_PUPD_STATE, standby_pupd_state, 4, 28, 0x3),
FLIS_NORMAL_CONTROL(TYPE_SBY_OD_DIS, enable, 2, 30, 0x1),
};

static unsigned int intel_get_conf_reg(struct gpio_debug *debug, unsigned gpio)
{
        struct intel_mid_gpio *lnw = debug->private_data;
        u32 offset, value = 0;

        if (lnw->type == TANGIER_GPIO) {
                offset = lnw->get_flis_offset(gpio);
                if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
                        return -EINVAL;

                value = get_flis_value(offset);
        }

        return value;
}

static void intel_set_conf_reg(struct gpio_debug *debug, unsigned gpio,
                unsigned int value)
{
        struct intel_mid_gpio *lnw = debug->private_data;
        u32 offset;

        if (lnw->type == TANGIER_GPIO) {
                offset = lnw->get_flis_offset(gpio);
                if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
                        return;

                set_flis_value(value, offset);
        }

        return;
}
static char **intel_get_avl_pininfo(struct gpio_debug *debug, unsigned gpio,
                unsigned int type, unsigned *num)
{
        struct gpio_control *control;

        control = find_gpio_control(lnw_gpio_controls,
                        ARRAY_SIZE(lnw_gpio_controls), type);
        if (control == NULL)
                return NULL;

        *num = control->num;

        return control->pininfo;
}

static char *intel_get_cul_pininfo(struct gpio_debug *debug, unsigned gpio,
                unsigned int type)
{
	struct intel_mid_gpio *lnw = debug->private_data;
	struct gpio_control *control;
        int num;

        control = find_gpio_control(lnw_gpio_controls,
                        ARRAY_SIZE(lnw_gpio_controls), type);
        if (control == NULL)
                return NULL;

        num = control->get(control, lnw, gpio);
        if (num == -1)
                return NULL;

        return *(control->pininfo + num);
}

static void intel_set_pininfo(struct gpio_debug *debug, unsigned gpio,
                unsigned int type, const char *info)
{
	struct intel_mid_gpio *lnw = debug->private_data;
        struct gpio_control *control;
        int num;

        control = find_gpio_control(lnw_gpio_controls,
                        ARRAY_SIZE(lnw_gpio_controls), type);
        if (control == NULL)
                return;

        num = find_pininfo_num(control, info);
        if (num == -1)
                return;

        if (control->set)
                control->set(control, lnw, gpio, num);
}

static int intel_get_register_msg(char **buf, unsigned long *size)
{
       *buf = conf_reg_msg;
       *size = strlen(conf_reg_msg);

       return 0;
}

static struct gpio_debug_ops intel_gpio_debug_ops = {
        .get_conf_reg = intel_get_conf_reg,
        .set_conf_reg = intel_set_conf_reg,
        .get_avl_pininfo = intel_get_avl_pininfo,
        .get_cul_pininfo = intel_get_cul_pininfo,
        .set_pininfo = intel_set_pininfo,
        .get_register_msg = intel_get_register_msg,
};

static void intel_mid_irq_init_hw(struct intel_mid_gpio *priv)
{
	void __iomem *reg;
	unsigned base;

	for (base = 0; base < priv->chip.ngpio; base += 32) {
		/* Clear the rising-edge detect register */
		reg = gpio_reg(&priv->chip, base, GRER);
		writel(0, reg);
		/* Clear the falling-edge detect register */
		reg = gpio_reg(&priv->chip, base, GFER);
		writel(0, reg);
		/* Clear the edge detect status register */
		reg = gpio_reg(&priv->chip, base, GEDR);
		writel(~0, reg);
	}
}

static int intel_gpio_irq_map(struct irq_domain *d, unsigned int virq,
                            irq_hw_number_t hw)
{
        struct intel_mid_gpio *priv = d->host_data;

        irq_set_chip_and_handler_name(virq, &intel_mid_irqchip, handle_simple_irq,
                                      "demux");
        irq_set_chip_data(virq, priv);
        irq_set_irq_type(virq, IRQ_TYPE_NONE);

        return 0;
}

static const struct irq_domain_ops intel_gpio_irq_ops = {
        .map = intel_gpio_irq_map,
        .xlate = irq_domain_xlate_twocell,
};

static int intel_gpio_runtime_resume(struct device *dev)
{
	return 0;
}

static int intel_gpio_runtime_suspend(struct device *dev)
{
	return 0;
}

static int intel_gpio_runtime_idle(struct device *dev)
{
	int err = pm_schedule_suspend(dev, 500);
	return err ?: -EBUSY;
}

static const struct dev_pm_ops intel_gpio_pm_ops = {
	SET_RUNTIME_PM_OPS(intel_gpio_runtime_suspend,
			intel_gpio_runtime_resume,
			intel_gpio_runtime_idle)
};

static int intel_gpio_probe(struct pci_dev *pdev,
			  const struct pci_device_id *id)
{
	void __iomem *base;
	struct intel_mid_gpio *priv;
	struct gpio_debug *debug;
	u32 gpio_base;
	u32 irq_base;
	int retval;
	struct intel_mid_gpio_ddata *ddata =
				(struct intel_mid_gpio_ddata *)id->driver_data;

	retval = pcim_enable_device(pdev);
	if (retval)
		return retval;

	retval = pcim_iomap_regions(pdev, 1 << 0 | 1 << 1, pci_name(pdev));
	if (retval) {
		dev_err(&pdev->dev, "I/O memory mapping error\n");
		return retval;
	}

	base = pcim_iomap_table(pdev)[1];

	irq_base = readl(base);
	gpio_base = readl(sizeof(u32) + base);

	/* release the IO mapping, since we already get the info from bar1 */
	pcim_iounmap_regions(pdev, 1 << 1);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "can't allocate chip data\n");
		return -ENOMEM;
	}

	priv->type = id->driver_data;
	priv->reg_base = pcim_iomap_table(pdev)[0];
	priv->chip_irq_type = ddata->chip_irq_type;
	priv->chip.label = dev_name(&pdev->dev);
	priv->chip.dev = &pdev->dev;
	priv->chip.request = intel_gpio_request;
	priv->chip.direction_input = intel_gpio_direction_input;
	priv->chip.direction_output = intel_gpio_direction_output;
	priv->chip.get = intel_gpio_get;
	priv->chip.set = intel_gpio_set;
	priv->chip.base = gpio_base;
	priv->chip.ngpio = ddata->ngpio;
	priv->chip.can_sleep = false;
	priv->chip.set_debounce = lnw_gpio_set_debounce;
	priv->chip.dev = &pdev->dev;
	priv->pdev = pdev;

	spin_lock_init(&priv->lock);

	pci_set_drvdata(pdev, priv);
	retval = gpiochip_add(&priv->chip);
	if (retval) {
		dev_err(&pdev->dev, "gpiochip_add error %d\n", retval);
		return retval;
	}

	retval = gpiochip_irqchip_add(&priv->chip,
				      &intel_mid_irqchip,
				      irq_base,
				      handle_simple_irq,
				      IRQ_TYPE_NONE);
	if (retval) {
		dev_err(&pdev->dev,
			"could not connect irqchip to gpiochip\n");
		return retval;
	}

	intel_mid_irq_init_hw(priv);

	gpiochip_set_chained_irqchip(&priv->chip,
				     &intel_mid_irqchip,
				     pdev->irq,
				     intel_mid_irq_handler);

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	/* add for gpiodebug */
	debug = gpio_debug_alloc();
	if (debug) {
		__set_bit(TYPE_OVERRIDE_OUTDIR, debug->typebit);
		__set_bit(TYPE_OVERRIDE_OUTVAL, debug->typebit);
		__set_bit(TYPE_OVERRIDE_INDIR, debug->typebit);
		__set_bit(TYPE_OVERRIDE_INVAL, debug->typebit);
		__set_bit(TYPE_SBY_OVR_IO, debug->typebit);
		__set_bit(TYPE_SBY_OVR_OUTVAL, debug->typebit);
		__set_bit(TYPE_SBY_OVR_INVAL, debug->typebit);
		__set_bit(TYPE_SBY_OVR_OUTDIR, debug->typebit);
		__set_bit(TYPE_SBY_OVR_INDIR, debug->typebit);
		__set_bit(TYPE_SBY_PUPD_STATE, debug->typebit);
		__set_bit(TYPE_SBY_OD_DIS, debug->typebit);

		debug->chip = &priv->chip;
		debug->ops = &intel_gpio_debug_ops;
		debug->private_data = priv;
		priv->debug = debug;

		retval = gpio_debug_register(debug);
		if (retval) {
			dev_err(&pdev->dev, "langwell gpio_debug_register failed %d\n",
				retval);
			gpio_debug_remove(debug);
		}
	}

	return 0;
}

static struct pci_driver intel_gpio_driver = {
	.name		= "intel_mid_gpio",
	.id_table	= intel_gpio_ids,
	.probe		= intel_gpio_probe,
	.driver		= {
		.pm	= &intel_gpio_pm_ops,
	},
};

static int __init intel_gpio_init(void)
{
	return pci_register_driver(&intel_gpio_driver);
}

device_initcall(intel_gpio_init);
