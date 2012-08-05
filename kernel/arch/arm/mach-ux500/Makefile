#
# Makefile for the linux kernel, U8500 machine.
#

ifeq ($(CONFIG_CRYPTO_DEV_UX500_HASH), m)
	CFLAGS_devices.o += -DCONFIG_CRYPTO_DEV_UX500_HASH
	CFLAGS_board-mop500.o += -DCONFIG_CRYPTO_DEV_UX500_HASH
endif

obj-y					:= clock.o timer-mtu.o \
					   devices-common.o pins.o dcache.o reboot_reasons.o
obj-$(CONFIG_PM)			+= pm/
obj-$(CONFIG_REGULATOR)			+= regulator-ux500.o

ifeq ($(CONFIG_UX500_SOC_DB8500), y)
obj-$(CONFIG_REGULATOR)			+= regulator-u8500.o
endif

ifeq ($(CONFIG_UX500_SOC_DB5500), y)
obj-$(CONFIG_REGULATOR)			+= regulator-u5500.o
endif

obj-$(CONFIG_U8500_REGULATOR_DEBUG)	+= virt-regulator-u8500.o
obj-$(CONFIG_ARCH_U8500)		+= devices.o cpu.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
obj-$(CONFIG_MACH_B5500)		+= board-u5500.o board-u5500-mcde.o \
					   board-u5500-sdi.o \
					   board-u5500-regulators.o \
					   board-u5500-pins.o
obj-$(CONFIG_MACH_SVP5500)		+= board-svp5500.o
obj-$(CONFIG_U5500_MMIO)		+= board-u5500-mmio.o
obj-$(CONFIG_U8500_MMIO)		+= board-sec-mmio.o
obj-$(CONFIG_MACH_U8500_MOP)		+= board-mop500.o board-mop500-msp.o board-mop500-uib.o \
					   board-mop500-stuib.o board-mop500-u8500uib.o \
					   board-mop500-pins.o board-mop500-bm.o \
					   board-mop500-regulators.o board-pins-sleep-force.o \
					   cw1200-wlan.o board-mop500-vibra.o
obj-$(CONFIG_MACH_U8500_PDP)		+= board-mop500.o board-mop500-msp.o board-mop500-bm.o \
					   board-mop500-pins.o board-mop500-regulators.o
					   # until pdp-specific files are there
obj-$(CONFIG_MOP500_SDI)		+= board-mop500-sdi.o
obj-$(CONFIG_MACH_SAMSUNG_U8500)	+= board-pins-sleep-force.o board-mop500-msp.o
obj-$(CONFIG_MACH_GODIN)		+= board-godin-pins.o board-godin.o board-godin-mcde.o board-godin-regulators.o \
					    board-godin-bm.o board-godin-sdi.o 
obj-$(CONFIG_MACH_JANICE)		+= board-janice-pins.o board-janice.o board-janice-mcde.o board-janice-regulators.o \
					    board-janice-bm.o board-janice-sdi.o 
obj-$(CONFIG_MACH_GAVINI)		+= board-gavini-pins.o board-gavini.o board-gavini-mcde.o board-gavini-regulators.o \
					    board-gavini-bm.o board-gavini-sdi.o 
obj-$(CONFIG_MACH_CODINA)		+= board-codina-pins.o board-codina.o board-codina-mcde.o board-codina-regulators.o \
					    board-codina-bm.o board-codina-sdi.o 
obj-$(CONFIG_UX500_SOC_DB5500)		+= devices-db5500.o cpu-db5500.o \
					   dma-db5500.o timer-db8500.o \
					   clock-db5500.o clock-debug.o
obj-$(CONFIG_UX500_SOC_DB8500)		+= devices-db8500.o \
					   cpu-db8500.o dma-db8500.o\
					   uart-db8500.o clock-db8500.o
obj-$(CONFIG_UX500_SOC_DB8500)		+= timer-rtt.o timer-db8500.o timer-db8500-prcmu.o
obj-$(CONFIG_MACH_U8500_PDP)		+= board-pdp-mcde.o
obj-$(CONFIG_MACH_U8500_MOP)		+= board-mop500-mcde.o
obj-$(CONFIG_MACH_SVP8500V1)		+= board-svp8500.o
obj-$(CONFIG_MACH_SVP8500V2)		+= board-svp8500.o

obj-$(CONFIG_CONNECTOR_SAMSUNG_3PI)	+= sec_jack.o

obj-$(CONFIG_MACH_U5500_SIMULATOR)	+= board-u5500.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_U8500_PRCMU)		+= prcmu-db8500.o
obj-$(CONFIG_U5500_PRCMU)		+= prcmu-db5500.o
obj-$(CONFIG_UX500_PRCMU_QOS_POWER)	+= prcmu-qos-power.o
obj-$(CONFIG_UX500_PRCMU_DEBUG)		+= prcmu-debug.o
obj-$(CONFIG_SENSORS1P_MOP)            	+= sensors1p.o
obj-$(CONFIG_MACH_U5500_BB)		+= board-u5500.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
ifeq ($(CONFIG_UX500_SOC_DB5500),y)
obj-$(CONFIG_USB)            		+= musb_db5500.o
else
obj-$(CONFIG_USB)            		+= musb_db8500.o
endif
obj-$(CONFIG_U5500_MLOADER)		+= mloader-db5500.o
obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o
obj-$(CONFIG_U5500_MODEM_IRQ)		+= modem_irq.o
obj-$(CONFIG_U5500_PWM)			+= pwm.o
obj-$(CONFIG_MOP500_NUIB)		+= board-mop500-nuib.o
obj-$(CONFIG_DB8500_MLOADER) 	+= mloader-db8500.o

obj-$(CONFIG_TEE_UX500)			+= tee_ux500.o product.o
obj-$(CONFIG_TEE_SVP)			+= tee_service_svp.o
obj-$(CONFIG_TEE_SVP)			+= tee_ta_start_modem_svp.o


obj-$(CONFIG_UX500_DEBUG_HWREG) 	+= hwreg.o
obj-$(CONFIG_HWMEM)			+= hwmem-int.o
obj-$(CONFIG_UX500_L2X0_PREFETCH_CTRL)	+= l2x0-prefetch.o
obj-$(CONFIG_UX500_DB_DUMP)		+= dbx500_dump.o

obj-$(CONFIG_RFKILL)		+=sec_rfkill.o

obj-$(CONFIG_SAMSUNG_KERNEL_DEBUG)      += sec_common.o
obj-$(CONFIG_SAMSUNG_PANIC_DISPLAY_LSOF)      += sec_lsof.o
obj-$(CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES)      += sec_pmic.o
obj-$(CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES)      += sec_i2c.o

