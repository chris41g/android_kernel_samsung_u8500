/*
 * Copyright (C) 2009 STMicroelectronics
 * Copyright (C) 2009 ST-Ericsson SA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MUSB_U8500_H__
#define __MUSB_U8500_H__

#include <mach/hardware.h>

/*
 * TODO: We don't want dependencies from a generic driver (power/ab8500_bm)
 * towards a enum in machine specific header files. This should be changed
 */
enum ab8500_usb_state {
	AB8500_BM_USB_STATE_RESET_HS,	/* HighSpeed Reset */
	AB8500_BM_USB_STATE_RESET_FS,	/* FullSpeed/LowSpeed Reset */
	AB8500_BM_USB_STATE_CONFIGURED,
	AB8500_BM_USB_STATE_SUSPEND,
	AB8500_BM_USB_STATE_RESUME,
	AB8500_BM_USB_STATE_MAX,
};

/*
 * U8500-specific definitions
 */

/*
 * Top-level Control and Status Registers
 */
#define OTG_DMASEL	0x200	/* OTG DMA Selector Register */
#define OTG_TOPCTRL	0x204	/* OTG Top Control Register  */

/*
 * ULPI-specific Registers
 */
#define OTG_UVBCTRL	0x70	/* OTG ULPI VBUS Control Register*/
#define OTG_UCKIT	0x71	/* OTG ULPI CarKit Control Register*/
#define OTG_UINTMASK	0x72	/* OTG ULPI INT Mask Register*/
#define OTG_UINTSRC	0x73	/* OTG ULPI INT Source Register*/
#define OTG_UREGDATA	0x74	/* OTG ULPI Reg Data Register*/
#define OTG_UREGADDR	0x75	/* OTG ULPI Reg Address Register*/
#define OTG_UREGCTRL	0x76	/* OTG ULPI Reg Control Register*/
#define OTG_URAWDATA	0x77	/* OTG ULPI Raw Data Register*/

/*
 * OTG Top Control Register Bits
 */
#define OTG_TOPCTRL_MODE_ULPI	(1 << 0)	/* Activate ULPI interface*/
#define OTG_TOPCTRL_UDDR	(1 << 1)	/* Activate ULPI double-data rate mode */
#define OTG_TOPCTRL_SRST	(1 << 2)	/* OTG core soft reset*/
#define OTG_TOPCTRL_XGATE	(1 << 3)	/* Activate transceiver clock*/
#define OTG_TOPCTRL_I2C_OFF	(1 << 4)	/* Switch off I2C controller*/
#define OTG_TOPCTRL_HDEV	(1 << 5)	/* Select host mode with FS interface*/
#define OTG_TOPCTRL_VBUSLO	(1 << 6)	/* Enable VBUS for FS transceivers*/

/*
 * OTG ULPI VBUS Control Register Bits
 */
#define OTG_UVBCTRL_EXTVB	(1 << 0)	/* Use External VBUS*/
#define OTG_UVBCTRL_EXTVI	(1 << 1)	/* Use External VBUS Indicator*/

/*
 * OTG ULPI Reg Control Register Bits
 */
#define OTG_UREGCTRL_REGREQ	(1 << 0)	/* Request ULPI register access */
#define OTG_UREGCTRL_REGCMP	(1 << 1)	/* ULPI register access completion flag */
#define OTG_UREGCTRL_URW	(1 << 2)	/* Request read from register access */

/*
 * STULPI01/STULPI05-specific definitions
 */

/*
 * Registers accessors (different offsets for read/write/set/clear functions)
 */
#define ULPI_REG_READ(x) (x + 0)
#define ULPI_REG_WRITE(x) (x + 0)
#define ULPI_REG_SET(x) (x + 1)
#define ULPI_REG_CLEAR(x) (x + 2)

/*
 * STULPI01/STULPI05 Registers
 */
#define ULPI_VIDLO	0x00	/* Vendor ID Low (Read Only)    */
#define ULPI_VIDHI	0x01	/* Vendor ID High (Read Only)   */
#define ULPI_PIDLO	0x02	/* Product ID Low (Read Only)   */
#define ULPI_PIDHI	0x03	/* Product ID High (Read Only)  */
#define ULPI_FCTRL	0x04	/* Function Control Register    */
#define ULPI_ICTRL	0x07	/* Interface Control Register   */
#define ULPI_OCTRL	0x0A	/* OTG Control Register         */
#define ULPI_INTENRIS	0x0D	/* Interrupt Enable Rising      */
#define ULPI_INTENFAL	0x10	/* Interrupt Enable Falling     */
#define ULPI_INTSTATU	0x13	/* Interrupt Status (Read Only) */
#define ULPI_INTLATCH	0x14	/* Interrupt Latch (Read Only)  */
#define ULPI_DEBUG	0x15	/* Debug Register (Read Only)   */
#define ULPI_SCRATCH	0x16	/* Scratch Register             */
#define ULPI_CCTRL	0x19	/* Carkit Control Register      */
#define ULPI_PCTRL	0x3D	/* Power Control Register       */
#define ULPI_ULINKSTAT	0x39	/* USB Link Status & Control Register */

/*
 * Vendor and Product IDs
 */
#define ULPI_VIDLO_VALUE	0x83	/*Vendor ID Low*/
#define ULPI_VIDHI_VALUE	0x04	/*Vendor ID High*/
#define ULPI_PIDLO_VALUE	0x4B	/*Product ID Low*/
#define ULPI_PIDHI_VALUE	0x4F	/*Product ID High*/

/**
 * STULPI Function Control Register Bits
 */
#define ULPI_FCTRL_HSEN		(0 << 0)
#define ULPI_FCTRL_FSEN		(1 << 0)
#define ULPI_FCTRL_LSEN		(2 << 0)
#define ULPI_FCTRL_FSLSEN	(3 << 0)
#define ULPI_FCTRL_TSELECT	(1 << 2)
#define ULPI_FCTRL_OM_NORM	(0 << 3)
#define ULPI_FCTRL_OM_NONDRIV	(1 << 3)
#define ULPI_FCTRL_OM_NRZIDIS	(2 << 3)
#define ULPI_FCTRL_OM_NOSYNC	(3 << 3)
#define ULPI_FCTRL_RESET	(1 << 5)
#define ULPI_FCTRL_NOSUSPEND	(1 << 6)

/*
 * STULPI Interface Control Register Bits
 */
#define ULPI_ICTRL_6PINSERIAL	(1 << 0)
#define ULPI_ICTRL_3PINSERIAL	(1 << 1)
#define ULPI_ICTRL_CARKITEN	(1 << 2)
#define ULPI_ICTRL_PSCLOCKEN	(1 << 3)
#define ULPI_ICTRL_VBUSINV	(1 << 5)
#define ULPI_ICTRL_IPASSTHRU	(1 << 6)
#define ULPI_ICTRL_IPROTDIS	(1 << 7)

/*
 * STULPI OTG Control Register Bits
 */
#define ULPI_OCTRL_IDPULLUP	(1 << 0)
#define ULPI_OCTRL_DPPULLDOWN	(1 << 1)
#define ULPI_OCTRL_DMPULLDOWN	(1 << 2)
#define ULPI_OCTRL_DISCHRGVBUS	(1 << 3)
#define ULPI_OCTRL_CHRGVBUS	(1 << 4)
#define ULPI_OCTRL_DRVVBUS	(1 << 5)
#define ULPI_OCTRL_DRVVBUSEXT	(1 << 6)
#define ULPI_OCTRL_EXTVBUSIND	(1 << 7)

/*
 * STULPI USB Link Status & Control Register
 */

/* function prototypes */
void ab8500_bm_usb_state_changed_wrapper(u8 bm_usb_state);

#endif/* __MUSB_U8500_H__ */
