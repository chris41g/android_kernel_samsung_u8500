/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/amba/serial.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#define MACH_TYPE	MACH_TYPE_SVP5500
#define CMDLINE						\
	"root=/dev/ram0 init=init rw "			\
	"console=ttyAMA0,115200n8 " 			\
	"mem=24MB@0 mem=208M@48M "			\
	"initrd=0x3000000,60M "				\
	"earlyprintk "					\
	"memmap=0x01800000$0x01800000 "

#define CPU1_WAKEMAGIC_ADDR	(U5500_BACKUPRAM0_BASE + 0x1ff0)

#define NMK_GPIO_AFSLA	0x20

.macro init_console_uart
	/* Enable GPIOs for UART0 (28, 29 -> AltA) */
	ldr	r5, =U5500_GPIO0_BASE
	ldr	r6, =((1 << 28) | (1 << 29))
	str	r6, [r5, #NMK_GPIO_AFSLA]

	/* Enable UART0 */
	ldr	r5, =U5500_UART0_BASE
	ldr	r6, =(UART011_CR_TXE | UART01x_CR_UARTEN)
	str	r6, [r5, #UART011_CR]
.endm
