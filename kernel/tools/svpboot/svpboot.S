/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

#ifdef SVP5500
#include "svp5500.S"
#elif defined(SVP8500V1)
#include "svp8500v1.S"
#elif defined(SVP8500V2)
#include "svp8500v2.S"
#endif

	.text

ENTRY(_start)
	/* Enable SWP */
	mrc	p15, 0, r1, c1, c0, 0
	orr	r1, r1, #1 << 10
	mcr	p15, 0, r1, c1, c0, 0

	mrc	p15, 0, r3, c0, c0, 5
	ands	r3, r3, #0xf
	bne	core2

	init_console_uart

	mov	r0, #0
	ldr	r1, =MACH_TYPE
	ldr	r2, =taglist

	b	kernel

core2:
	ldr	r0, =CPU1_WAKEMAGIC_ADDR
	ldr	r2, =0xA1FEED01

1:	ldr	r1, [r0]
	cmp	r1, r2
	bne	1b

	/* Jump Addr is at WakeMagic + 4 */
	ldr	r1, [r0, #4]
	bx	r1

	.data
#define ATAG_CORE	0x54410001
#define ATAG_CMDLINE	0x54410009
#define ATAG_NONE	0x00000000

taglist:
	.long 2
	.long ATAG_CORE
	.long 2 + (cmdlineend - cmdlinestart) / 4
	.long ATAG_CMDLINE
cmdlinestart:
	.ascii CMDLINE
	.align 2
cmdlineend:
	.long 0
	.long ATAG_NONE
