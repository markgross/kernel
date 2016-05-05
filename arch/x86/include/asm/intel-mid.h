/*
 * intel-mid.h: Intel MID specific setup code
 *
 * (C) Copyright 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_INTEL_MID_H
#define _ASM_X86_INTEL_MID_H

#include <linux/types.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <asm/intel_mid_pcihelpers.h>

#define INTEL_MID_SSN_SIZE     32

/*
 * Access to message bus through three registers
 * in CUNIT(0:0:0) PCI configuration space.
 * MSGBUS_CTRL_REG(0xD0):
 *   31:24	= message bus opcode
 *   23:16	= message bus port
 *   15:8	= message bus address, low 8 bits.
 *   7:4	= message bus byte enables
 * MSGBUS_CTRL_EXT_REG(0xD8):
 *   31:8	= message bus address, high 24 bits.
 * MSGBUS_DATA_REG(0xD4):
 *   hold the data for write or read
 */
#define PCI_ROOT_MSGBUS_CTRL_REG	0xD0
#define PCI_ROOT_MSGBUS_DATA_REG	0xD4
#define PCI_ROOT_MSGBUS_CTRL_EXT_REG	0xD8
#define PCI_ROOT_MSGBUS_READ		0x10
#define PCI_ROOT_MSGBUS_WRITE		0x11
#define PCI_ROOT_MSGBUS_DWORD_ENABLE	0xf0

/*
 * Access to message bus through three registers
 * in CUNIT(0:0:0) PCI configuration space.
 * MSGBUS_CTRL_REG(0xD0):
 *   31:24	= message bus opcode
 *   23:16	= message bus port
 *   15:8	= message bus address, low 8 bits.
 *   7:4	= message bus byte enables
 * MSGBUS_CTRL_EXT_REG(0xD8):
 *   31:8	= message bus address, high 24 bits.
 * MSGBUS_DATA_REG(0xD4):
 *   hold the data for write or read
 */
#define PCI_ROOT_MSGBUS_CTRL_REG	0xD0
#define PCI_ROOT_MSGBUS_DATA_REG	0xD4
#define PCI_ROOT_MSGBUS_CTRL_EXT_REG	0xD8
#define PCI_ROOT_MSGBUS_READ		0x10
#define PCI_ROOT_MSGBUS_WRITE		0x11
#define PCI_ROOT_MSGBUS_DWORD_ENABLE	0xf0

extern int intel_mid_pci_init(void);
extern int get_gpio_by_name(const char *name);
extern void intel_scu_device_register(struct platform_device *pdev);
extern int __init sfi_parse_mrtc(struct sfi_table_header *table);
extern int __init sfi_parse_mtmr(struct sfi_table_header *table);
extern int sfi_mrtc_num;
extern struct sfi_rtc_table_entry sfi_mrtc_array[];
extern void *get_oem0_table(void);
extern void register_rpmsg_service(char *name, int id, u32 addr);
extern int sdhci_pci_request_regulators(void);

/* Define soft platform ID to comply with the OEMB table format. But SPID is not supported */
#define INTEL_PLATFORM_SSN_SIZE 32
struct soft_platform_id {
        u16 customer_id; /*Defines the final customer for the product */
        u16 vendor_id; /* Defines who owns the final product delivery */
        u16 manufacturer_id; /* Defines who build the hardware. This can be
                              * different for the same product */
        u16 platform_family_id; /* Defined by vendor and defines the family of
                                 * the product with the same root components */
        u16 product_line_id; /* Defined by vendor and defines the name of the
                              * product. This can be used to differentiate the
                              * feature set for the same product family (low
                              * cost vs full feature). */
        u16 hardware_id; /* Defined by vendor and defines the physical hardware
                          * component set present on the PCB/FAB */
        u8  fru[SPID_FRU_SIZE]; /* Field Replaceabl Unit */
} __packed;

/* OEMB table */
struct sfi_table_oemb {
	struct sfi_table_header header;
	u32 board_id;
	u32 board_fab;
	u8 iafw_major_version;
	u8 iafw_main_version;
	u8 val_hooks_major_version;
	u8 val_hooks_minor_version;
	u8 ia_suppfw_major_version;
	u8 ia_suppfw_minor_version;
	u8 scu_runtime_major_version;
	u8 scu_runtime_minor_version;
	u8 ifwi_major_version;
	u8 ifwi_minor_version;
	struct soft_platform_id spid;
	u8 ssn[INTEL_MID_SSN_SIZE];
} __packed;
/*
 * Here defines the array of devices platform data that IAFW would export
 * through SFI "DEVS" table, we use name and type to match the device and
 * its platform data.
 */
struct devs_id {
	char name[SFI_NAME_LEN + 1];
	u8 type;
	u8 delay;
	void *(*get_platform_data)(void *info);
	/* Custom handler for devices */
	void (*device_handler)(struct sfi_device_table_entry *pentry,
				struct devs_id *dev);
};

#define SD_NAME_SIZE 16
/**
 * struct sd_board_info - template for device creation
 * @name: Initializes sdio_device.name; identifies the driver.
 * @bus_num: board-specific identifier for a given SDIO controller.
 * @board_ref_clock: Initializes sd_device.board_ref_clock;
 * @platform_data: Initializes sd_device.platform_data; the particular
 *      data stored there is driver-specific.
 *
 */
struct sd_board_info {
       char            name[SD_NAME_SIZE];
       int             bus_num;
       unsigned short  addr;
       u32             board_ref_clock;
       void            *platform_data;
};


#define sfi_device(i)   \
	static const struct devs_id *const __intel_mid_sfi_##i##_dev __used \
	__attribute__((__section__(".x86_intel_mid_dev.init"))) = &i

/*
 * Medfield is the follow-up of Moorestown, it combines two chip solution into
 * one. Other than that it also added always-on and constant tsc and lapic
 * timers. Medfield is the platform name, and the chip name is called Penwell
 * we treat Medfield/Penwell as a variant of Moorestown. Penwell can be
 * identified via MSRs.
 */
enum intel_mid_cpu_type {
	/* 1 was Moorestown */
	INTEL_MID_CPU_CHIP_PENWELL = 2,
	INTEL_MID_CPU_CHIP_CLOVERVIEW,
	INTEL_MID_CPU_CHIP_TANGIER,
};

extern enum intel_mid_cpu_type __intel_mid_cpu_chip;

/**
 * struct intel_mid_ops - Interface between intel-mid & sub archs
 * @arch_setup: arch_setup function to re-initialize platform
 *             structures (x86_init, x86_platform_init)
 *
 * This structure can be extended if any new interface is required
 * between intel-mid & its sub arch files.
 */
struct intel_mid_ops {
	void (*arch_setup)(void);
};

/* Helper API's for INTEL_MID_OPS_INIT */
#define DECLARE_INTEL_MID_OPS_INIT(cpuname, cpuid)	\
				[cpuid] = get_##cpuname##_ops

/* Maximum number of CPU ops */
#define MAX_CPU_OPS(a) (sizeof(a)/sizeof(void *))

/*
 * For every new cpu addition, a weak get_<cpuname>_ops() function needs be
 * declared in arch/x86/platform/intel_mid/intel_mid_weak_decls.h.
 */
#define INTEL_MID_OPS_INIT {\
	DECLARE_INTEL_MID_OPS_INIT(penwell, INTEL_MID_CPU_CHIP_PENWELL), \
	DECLARE_INTEL_MID_OPS_INIT(cloverview, INTEL_MID_CPU_CHIP_CLOVERVIEW), \
	DECLARE_INTEL_MID_OPS_INIT(tangier, INTEL_MID_CPU_CHIP_TANGIER) \
};

#ifdef CONFIG_X86_INTEL_MID

static inline enum intel_mid_cpu_type intel_mid_identify_cpu(void)
{
	return __intel_mid_cpu_chip;
}

static inline bool intel_mid_has_msic(void)
{
	return (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_PENWELL);
}

#else /* !CONFIG_X86_INTEL_MID */

#define intel_mid_identify_cpu()    (0)
#define intel_mid_has_msic()    (0)

#endif /* !CONFIG_X86_INTEL_MID */

enum intel_mid_timer_options {
	INTEL_MID_TIMER_DEFAULT,
	INTEL_MID_TIMER_APBT_ONLY,
	INTEL_MID_TIMER_LAPIC_APBT,
};

extern enum intel_mid_timer_options intel_mid_timer_options;

#define spid_attr(_name) \
static struct kobj_attribute _name##_attr = { \
	.attr = {                             \
		.name = __stringify(_name),   \
		.mode = 0444,                 \
	},                                    \
	.show   = _name##_show,               \
}

/*
 * Penwell uses spread spectrum clock, so the freq number is not exactly
 * the same as reported by MSR based on SDM.
 */
#define FSB_FREQ_83SKU	83200
#define FSB_FREQ_100SKU	99840
#define FSB_FREQ_133SKU	133000

#define FSB_FREQ_167SKU	167000
#define FSB_FREQ_200SKU	200000
#define FSB_FREQ_267SKU	267000
#define FSB_FREQ_333SKU	333000
#define FSB_FREQ_400SKU	400000

/* Bus Select SoC Fuse value */
#define BSEL_SOC_FUSE_MASK	0x7
#define BSEL_SOC_FUSE_001	0x1 /* FSB 133MHz */
#define BSEL_SOC_FUSE_101	0x5 /* FSB 100MHz */
#define BSEL_SOC_FUSE_111	0x7 /* FSB 83MHz */

#define SFI_MTMR_MAX_NUM 8
#define SFI_MRTC_MAX	8

extern void intel_scu_devices_create(void);
extern void intel_scu_devices_destroy(void);

/* VRTC timer */
#define MRST_VRTC_MAP_SZ	(1024)
/*#define MRST_VRTC_PGOFFSET	(0xc00) */

extern void intel_mid_rtc_init(void);

/* the offset for the mapping of global gpio pin to irq */
#define INTEL_MID_IRQ_OFFSET 0x100

#endif /* _ASM_X86_INTEL_MID_H */
