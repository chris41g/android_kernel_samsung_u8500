/*
 * Copyright (C) 2009 ST Ericsson.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

/* AB5500 USB macros
 */
#define AB5500_REV_10 0x10
#define AB5500_REV_11 0x11
#define AB5500_REV_20 0x20
#define AB5500_REV_30 0x30
#define AB5500_USB_HOST 0x68
#define AB5500_USB_LINK_STATUS_UPDT_DONE 135
#define AB5500_IT_MASK20_MASK 0xFB
#define AB5500_IT_MASK21_MASK 0xBE
#define AB5500_IT_MASK2_MASK 0x3F
#define AB5500_IT_MASK12_MASK 0x7F
#define AB5500_SRC_INT_USB_DEVICE 0x80
#define AB5500_SRC_INT_USB_HOST 0x04
#define AB5500_VBUS_ENABLE 0x1
#define AB5500_VBUS_DISABLE 0x0
#define AB5500_USB_HOST_DEVICE_DISABLE 0x0
#define AB5500_USB_HOST_ENABLE 0x1
#define AB5500_USB_HOST_DISABLE 0x0
#define AB5500_USB_DEVICE_ENABLE 0x2
#define AB5500_USB_DEVICE_DISABLE 0x0
#define AB5500_MAIN_WATCHDOG_ENABLE 0x1
#define AB5500_MAIN_WATCHDOG_KICK 0x2
#define AB5500_MAIN_WATCHDOG_DISABLE 0x0
#define AB5500_USB_ADP_ENABLE 0x1

/* USB Macros
 */
#define WATCHDOG_DELAY 10
#define WATCHDOG_DELAY_US 100
#define USB_ENABLE 1
#define USB_DISABLE 0

/* UsbLineStatus register bit masks */
#define AB5500_USB_LINK_STATUS		0x78

/* UsbLineStatus register - usb types */
enum musb_link_status {
	USB_LINK_NOT_CONFIGURED,
	USB_LINK_STD_HOST_NC,
	USB_LINK_STD_HOST_C_NS,
	USB_LINK_STD_HOST_C_S,
	USB_LINK_HOST_CHG_NM,
	USB_LINK_HOST_CHG_HS,
	USB_LINK_HOST_CHG_HS_CHIRP,
	USB_LINK_DEDICATED_CHG,
	USB_LINK_ACA_RID_A,
	USB_LINK_ACA_RID_B,
	USB_LINK_ACA_RID_C_NM,
	USB_LINK_ACA_RID_C_HS,
	USB_LINK_ACA_RID_C_HS_CHIRP,
	USB_LINK_HM_IDGND,
	USB_LINK_OTG_HOST_NO_CURRENT,
	USB_LINK_NOT_VALID_LINK,
};

/* Specific functions for USB Phy enable and disable
 */
int musb_phy_en(u8 mode);
int musb_force_detect(u8 mode);
int musb_get_abx500_rev(void);


