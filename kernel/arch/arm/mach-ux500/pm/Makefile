#
# Power save related files
#
obj-y := pm.o runtime.o

obj-$(CONFIG_U8500_CPUIDLE) 		+= cpuidle.o timer.o
obj-$(CONFIG_U8500_CPUIDLE_DEBUG) 	+= cpuidle_dbg.o
obj-$(CONFIG_UX500_CONTEXT) 		+= context.o context_arm.o context-db8500.o context-db5500.o
obj-$(CONFIG_U8500_CPUFREQ) 		+= cpufreq.o
obj-$(CONFIG_UX500_SUSPEND)		+= suspend.o
obj-$(CONFIG_UX500_SUSPEND_DBG)		+= suspend_dbg.o
obj-$(CONFIG_UX500_USECASE_GOVERNOR)	+= usecase_gov.o

ifeq ($(CONFIG_UX500_SOC_DB8500), y)
obj-$(CONFIG_U8500_CPUFREQ) 		+= cpufreq-db8500.o
endif
