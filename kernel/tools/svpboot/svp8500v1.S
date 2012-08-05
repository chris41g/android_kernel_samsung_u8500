/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/amba/serial.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#define MACH_TYPE	MACH_TYPE_SVP8500V1
#define CMDLINE				\
	"root=/dev/ram0 init=init rw "	\
	"console=ttyAMA2,115200n8 "	\
	"mem=256M "			\
	"initrd=0x800000,60M "		\
	"earlyprintk"			\

#define CPU1_WAKEMAGIC_ADDR	(U8500_BACKUPRAM0_BASE + 0x1ff0)

#define NMK_GPIO_AFSLA	0x20
#define NMK_GPIO_AFSLB	0x24

.macro init_console_uart
	/* Enable GPIOs for UART2 (29, 30 -> AltC) */
	ldr	r5, =U8500_GPIO0_BASE
	ldr	r6, =((1 << 29) | (1 << 30))
	str	r6, [r5, #NMK_GPIO_AFSLA]
	str	r6, [r5, #NMK_GPIO_AFSLB]

	/* Enable UART2 */
	ldr	r5, =U8500_UART2_BASE
	ldr	r6, =(UART011_CR_TXE | UART01x_CR_UARTEN)
	str	r6, [r5, #UART011_CR]
.endm
