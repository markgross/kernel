# IPC Devices
obj-y += platform_mrfl_regulator.o
obj-y += platform_ipc.o
obj-$(subst m,y,$(CONFIG_MFD_INTEL_MSIC)) += platform_msic.o
obj-$(subst m,y,$(CONFIG_SND_MFLD_MACHINE)) += platform_msic_audio.o
obj-$(subst m,y,$(CONFIG_GPIO_MSIC)) += platform_msic_gpio.o
obj-$(subst m,y,$(CONFIG_MFD_INTEL_MSIC)) += platform_msic_ocd.o
obj-$(subst m,y,$(CONFIG_MFD_INTEL_MSIC)) += platform_msic_battery.o
obj-y += pci/
obj-$(subst m,y,$(CONFIG_INTEL_MID_POWER_BUTTON)) += platform_msic_power_btn.o
obj-$(subst m,y,$(CONFIG_GPIO_INTEL_PMIC)) += platform_pmic_gpio.o
obj-$(subst m,y,$(CONFIG_INTEL_MFLD_THERMAL)) += platform_msic_thermal.o
obj-$(subst m,y,$(CONFIG_SENSORS_MID_VDD)) += platform_msic_vdd.o
obj-$(subst m,y,$(CONFIG_SENSORS_MRFL_OCD)) += platform_mrfl_ocd.o
obj-$(subst m,y,$(CONFIG_INTEL_SCU_FLIS)) += platform_scu_flis.o
obj-$(subst m,y,$(CONFIG_PMIC_CCSM)) += platform_mrfl_pmic.o
obj-$(subst m,y,$(CONFIG_I2C_PMIC)) += platform_mrfl_pmic_i2c.o
# I2C Devices
obj-$(subst m,y,$(CONFIG_I2C_DESIGNWARE_CORE_FORK)) += platform_dw_i2c.o
obj-$(subst m,y,$(CONFIG_SENSORS_EMC1403)) += platform_emc1403.o
obj-$(subst m,y,$(CONFIG_SENSORS_LIS3LV02D)) += platform_lis331.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X)) += platform_max7315.o
obj-$(subst m,y,$(CONFIG_INPUT_MPU3050)) += platform_mpu3050.o
obj-$(subst m,y,$(CONFIG_INPUT_BMA150)) += platform_bma023.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X)) += platform_tca6416.o
obj-$(subst m,y,$(CONFIG_DRM_MEDFIELD)) += platform_tc35876x.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X)) += platform_pcal9555a.o
# SPI Devices
obj-$(subst m,y,$(CONFIG_SPI_SPIDEV)) += platform_spidev.o
obj-$(subst m,y,$(CONFIG_TI_ADS7955_ADC)) += platform_ads7955.o
# MISC Devices
obj-$(subst m,y,$(CONFIG_KEYBOARD_GPIO)) += platform_gpio_keys.o
obj-$(subst m,y,$(CONFIG_INTEL_MID_WATCHDOG)) += platform_wdt.o
# ADC
obj-$(subst m,y,$(CONFIG_MSIC_GPADC))	+= platform_msic_adc.o
obj-$(subst m,y,$(CONFIG_IIO_BASINCOVE_GPADC))	+= platform_bcove_adc.o
# SD Devices
obj-$(subst m,y,$(CONFIG_WILINK_PLATFORM_DATA)) += platform_wl12xx.o
ifndef CONFIG_ACPI
obj-$(subst m,y,$(CONFIG_BCM_BT_LPM)) += platform_btlpm.o
endif
#I2C Devices
# Panel Control Device
obj-$(subst m,y,$(CONFIG_DRM_MRFLD)) += platform_panel.o
# GPS
obj-$(subst m,y,$(CONFIG_INTEL_MID_GPS)) += platform_gps.o
# WIFI devices
obj-$(subst m,y,$(CONFIG_WIFI_PLATFORM_DATA)) += platform_wifi.o
obj-$(subst m,y,$(CONFIG_MMC_SDHCI_ACPI)) += platform_sdio_regulator.o
# SCU log
obj-$(subst m,y,$(CONFIG_SCU_LOGGING)) += platform_scu_log.o
