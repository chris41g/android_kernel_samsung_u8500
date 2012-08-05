obj-$(CONFIG_U8500_MMIO) := st_mmio.o
ifeq ($(CONFIG_REGULATOR_RN5T592),y)
obj-$(CONFIG_U8500_MMIO) += subpmic_rn5t592.o
else
obj-$(CONFIG_U8500_MMIO) += subpmic_ncp6914.o
endif	