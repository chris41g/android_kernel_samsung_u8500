#Android makefile to build kernel as a part of Android Build
ifeq ($(TARGET_USE_ST_ERICSSON_KERNEL),true)

# Give other modules a nice, symbolic name to use as a dependent
# Yes, there are modules that cannot build unless the kernel has
# been built. Typical (only?) example: loadable kernel modules.
.phony: build-kernel clean-kernel

PRIVATE_KERNEL_ARGS := -C kernel ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) LOCALVERSION=$(LOCALVERSION)

PRIVATE_OUT := $(abspath $(PRODUCT_OUT)/root)

export STERICCSON_WLAN_BUILT_IN=y

# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
PRIVATE_KERNEL_ARGS += O=$(KERNEL_OUTPUT)
endif
else
KERNEL_OUTPUT := $(call my-dir)
endif

# Include kernel in the Android build system
include $(CLEAR_VARS)

KERNEL_LIBPATH := $(KERNEL_OUTPUT)/arch/arm/boot
LOCAL_PATH := $(KERNEL_LIBPATH)
LOCAL_SRC_FILES := zImage
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)

$(KERNEL_LIBPATH)/$(LOCAL_SRC_FILES): build-kernel

include $(BUILD_PREBUILT)

include $(CLEAR_VARS)

KERNEL_LIBPATH := $(KERNEL_OUTPUT)
LOCAL_PATH := $(KERNEL_LIBPATH)
LOCAL_SRC_FILES := vmlinux
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)

$(KERNEL_LIBPATH)/$(LOCAL_SRC_FILES): build-kernel

include $(BUILD_PREBUILT)

# Configures, builds and installs the kernel. KERNEL_DEFCONFIG usually
# comes from the BoardConfig.mk file, but can be overridden on the
# command line or by an environment variable.
# If KERNEL_DEFCONFIG is set to 'local', configuration is skipped.
# This is useful if you want to play with your own, custom configuration.

build-kernel:

# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
	@mkdir -p $(KERNEL_OUTPUT)
endif
endif

ifeq ($(KERNEL_DEFCONFIG),local)
	@echo Skipping kernel configuration, KERNEL_DEFCONFIG set to local
else
	$(MAKE) $(PRIVATE_KERNEL_ARGS) $(KERNEL_DEFCONFIG)
endif

ifeq ($(shell [ -f kernel/net/compat-wireless/Makefile ] && echo "OK"), OK)
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--enable CONFIG_COMPAT_WIRELESS \
		--enable CONFIG_CFG80211 \
		--enable CONFIG_MAC80211_RC_DEFAULT_MINSTREL \
		--enable CONFIG_COMPAT_RFKILL \
		--enable CONFIG_NL80211_TESTMODE \
		--enable CONFIG_CFG80211_DEFAULT_PS \
		--enable CONFIG_CFG80211_REG_DEBUG \
		--enable CONFIG_MAC80211_RC_PID \
		--enable CONFIG_MAC80211_RC_MINSTREL \
		--enable CONFIG_MAC80211_LEDS \
		--enable CONFIG_MAC80211_MESH \
		--set-str CONFIG_COMPAT_MAC80211_RC_DEFAULT minstrel
endif
ifeq ($(SEC_PRODUCT_SHIP),true) 
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--enable CONFIG_SAMSUNG_PRODUCT_SHIP
endif
	$(MAKE) $(PRIVATE_KERNEL_ARGS) zImage
ifeq ($(KERNEL_NO_MODULES),)
	$(MAKE) $(PRIVATE_KERNEL_ARGS) modules
	$(MAKE) $(PRIVATE_KERNEL_ARGS) INSTALL_MOD_PATH:=$(PRIVATE_OUT) modules_install
else
	@echo Skipping building of kernel modules, KERNEL_NO_MODULES set
endif

build-kernel2: ramdisk recoveryimage build-kernel
	kernel/scripts/config --file $(KERNEL_OUTPUT)/.config \
		--set-str CONFIG_INITRAMFS_SOURCE "$(abspath $(TOP)/$(TARGET_ROOT_OUT))" \
		--set-val CONFIG_INITRAMFS_ROOT_UID 0 \
		--set-val CONFIG_INITRAMFS_ROOT_GID 0 \
		--enable CONFIG_INITRAMFS_COMPRESSION_NONE \
		--disable CONFIG_INITRAMFS_COMPRESSION_GZIP
	$(MAKE) $(PRIVATE_KERNEL_ARGS) zImage
	cd $(KERNEL_OUTPUT)/arch/arm/boot; \
	cp zImage kernel.bin; \
	tar cvf $(abspath $(PRODUCT_OUT))/kernel.tar kernel.bin

# Configures and runs menuconfig on the kernel based on
# KERNEL_DEFCONFIG given on commandline or in BoardConfig.mk.
# The build after running menuconfig must be run with
# KERNEL_DEFCONFIG=local to not override the configuration modification done.

menuconfig-kernel:
# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
	@mkdir -p $(KERNEL_OUTPUT)
endif
endif

	$(MAKE) $(PRIVATE_KERNEL_ARGS) $(KERNEL_DEFCONFIG)
	$(MAKE) $(PRIVATE_KERNEL_ARGS) menuconfig

clean clobber : clean-kernel

clean-kernel:
	$(MAKE) $(PRIVATE_KERNEL_ARGS) clean


endif
