/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef __ASM_ARCH_DEVICES_H__
#define __ASM_ARCH_DEVICES_H__

struct platform_device;
struct amba_device;

extern struct platform_device u5500_gpio_devs[];
extern struct platform_device u8500_gpio_devs[];

extern struct platform_device ux500_mcde_device;
extern struct platform_device u8500_shrm_device;
extern struct platform_device ux500_b2r2_device;
extern struct platform_device u8500_trace_modem;
extern struct platform_device u8500_pmem_device;
extern struct platform_device u8500_pmem_mio_device;
extern struct platform_device u8500_pmem_hwb_device;
extern struct platform_device ux500_hwmem_device;
extern struct platform_device ux500_dma_device;
extern struct platform_device ux500_hash1_device;
extern struct platform_device ux500_cryp1_device;
extern struct platform_device ux500_musb_device;
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* soonyong.cho : Define samsung product id and config string.
 *                Sources such as 'android.c' and 'devs.c' refered below define
 */
#define SAMSUNG_VENDOR_ID		0x04e8

#ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE
	/* USE DEVGURU HOST DRIVER */
	/* 0x6860 : MTP(0) + MS Composite (UMS) */
	/* 0x685E : UMS(0) + MS Composite (ADB) */
#define SAMSUNG_KIES_PRODUCT_ID		0x6860	/* mtp + acm(1,2) */
#define SAMSUNG_DEBUG_PRODUCT_ID	0x685e	/* ums + acm(1,2) + adb */
#define SAMSUNG_UMS_PRODUCT_ID		0x685B  /* UMS Only */
#define SAMSUNG_MTP_PRODUCT_ID		0x685C  /* MTP Only */
#ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
#define SAMSUNG_RNDIS_PRODUCT_ID	0x6861  /* RNDIS(0,1) + UMS (2) + MS Composite */
//#elif defined(CONFIG_USB_ANDROID_PHONET)
//#define SAMSUNG_RNDIS_PRODUCT_ID	0x6864  /* RNDIS(0,1) + PHONET(2,3) */
#else
#define SAMSUNG_RNDIS_PRODUCT_ID	0x6863  /* RNDIS only */
#endif
#define SAMSUNG_PHONET_PRODUCT_ID 0x2323  /* PHONET Only */
#else /* USE MCCI HOST DRIVER */
#define SAMSUNG_KIES_PRODUCT_ID		0x6877	/* Shrewbury ACM+MTP*/
#define SAMSUNG_DEBUG_PRODUCT_ID	0x681C	/* Shrewbury ACM+UMS+ADB*/
#define SAMSUNG_UMS_PRODUCT_ID		0x681D
#define SAMSUNG_MTP_PRODUCT_ID		0x68A9
#define SAMSUNG_RNDIS_PRODUCT_ID	0x6881
#endif
#define  ANDROID_DEBUG_CONFIG_STRING	"ACM + MTP + ADB (Debugging mode)"
#define  ANDROID_KIES_CONFIG_STRING	 	"ACM + MTP (SAMSUNG KIES mode)"
#define  ANDROID_UMS_CONFIG_STRING		"UMS Only (Not debugging mode)"
#define  ANDROID_MTP_CONFIG_STRING		"MTP Only (Not debugging mode)"
#ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
#define ANDROID_RNDIS_CONFIG_STRING		"RNDIS + UMS (Not debugging mode)"
#else
#define ANDROID_RNDIS_CONFIG_STRING		"RNDIS Only (Not debugging mode)"
#endif
#define ANDROID_PHONET_CONFIG_STRING		"PHONET Only (Not debugging mode)"
	/* Refered from S1, P1 */
#define USBSTATUS_UMS					0x0
#define USBSTATUS_SAMSUNG_KIES 		0x1
#define USBSTATUS_MTPONLY			0x2
#define USBSTATUS_ASKON				0x4
#define USBSTATUS_VTP					0x8
#define USBSTATUS_ADB					0x10
#define USBSTATUS_DM					0x20
#define USBSTATUS_ACM					0x40
#define USBSTATUS_SAMSUNG_KIES_REAL		0x80
#define USBSTATUS_PHONET			0x100

/* soonyong.cho : This is for setting unique serial number */
#else
/* Original VID & PID */
#define ANDROID_VENDOR_ID			0x18d1
#define ANDROID_PRODUCT_ID		0x0001
#define ANDROID_ADB_PRODUCT_ID	0x0005

#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */

extern struct platform_device u5500_pwm0_device;
extern struct platform_device u5500_pwm1_device;
extern struct platform_device u5500_pwm2_device;
extern struct platform_device u5500_pwm3_device;
extern struct platform_device ux500_wdt_device;
extern struct platform_device mloader_fw_device;
extern struct platform_device u8500_thsens_device;
extern struct platform_device ux500_stm_device;
extern struct platform_device ux500_mmio_device;
extern struct platform_device u8500_hsi_device;
#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

/**
 * Touchpanel related macros declaration
 */
#define TOUCH_GPIO_PIN 	84

#define TOUCH_XMAX 384
#define TOUCH_YMAX 704

#define PRCMU_CLOCK_OCR 0x1CC
#define TSC_EXT_CLOCK_9_6MHZ 0x840000
#endif
