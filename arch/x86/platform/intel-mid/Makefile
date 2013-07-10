obj-$(CONFIG_X86_INTEL_MID) += intel-mid.o intel_mid_vrtc.o mfld.o mrfl.o

# SFI specific code
ifdef CONFIG_X86_INTEL_MID
obj-$(CONFIG_SFI) += sfi.o device_libs/
obj-$(CONFIG_X86_INTEL_MID) += intel_mid_scu.o
endif
