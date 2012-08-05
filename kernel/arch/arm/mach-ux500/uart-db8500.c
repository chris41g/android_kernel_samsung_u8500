/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Arun R Murthy <arun.murthy@stericsson.com>,
 *	   Jonas Aaberg <jonas.aberg@stericsson.com> for ST-Ericsson
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/amba/serial.h>
#include <mach/setup.h>
#include <mach/hardware.h>

#include "pm/context.h"

extern u8 ux500_debug_uart_config;

#ifdef CONFIG_UX500_CONTEXT

#define UX500_NR_UARTS 3

static struct {
	struct clk *uart_clk;
	void __iomem *base;
	/* dr */
	/* rsr_err */
	u32 dma_wm;
	u32 timeout;
	/* fr */
	u32 lcrh_rx;
	u32 ilpr;
	u32 ibrd;
	u32 fbrd;
	u32 lcrh_tx;
	u32 cr;
	u32 ifls;
	u32 imsc;
	/* ris */
	/* mis */
	/* icr */
	u32 dmacr;
	u32 xfcr;
	u32 xon1;
	u32 xon2;
	u32 xoff1;
	u32 xoff2;
	/* itcr */
	/* itip */
	/* itop */
	/* tdr */
	u32 abcr;
	/* absr */
	/* abfmt */
	/* abdr */
	/* abdfr */
	/* abmr */
	u32 abimsc;
	/* abris */
	/* abmis */
	/* abicr */
	/* id_product_h_xy */
	/* id_provider */
	/* periphid0 */
	/* periphid1 */
	/* periphid2 */
	/* periphid3 */
	/* pcellid0 */
	/* pcellid1 */
	/* pcellid2 */
	/* pcellid3 */
} context_uart[UX500_NR_UARTS];


static void save_uart(void)
{
	int i;
	void __iomem *membase;

	for (i = 0; i < UX500_NR_UARTS; i++) {
		if (i != ux500_debug_uart_config)
			continue;

		membase = context_uart[i].base;
		clk_enable(context_uart[i].uart_clk);

		context_uart[i].dma_wm = readl(membase + ST_UART011_DMAWM);
		context_uart[i].timeout = readl(membase + ST_UART011_TIMEOUT);
		context_uart[i].lcrh_rx = readl(membase + ST_UART011_LCRH_RX);
		context_uart[i].ilpr = readl(membase + UART01x_ILPR);
		context_uart[i].ibrd = readl(membase + UART011_IBRD);
		context_uart[i].fbrd = readl(membase + UART011_FBRD);
		context_uart[i].lcrh_tx = readl(membase + ST_UART011_LCRH_TX);
		context_uart[i].cr = readl(membase + UART011_CR);
		context_uart[i].ifls = readl(membase + UART011_IFLS);
		context_uart[i].imsc = readl(membase + UART011_IMSC);
		context_uart[i].dmacr = readl(membase + UART011_DMACR);
		context_uart[i].xfcr = readl(membase + ST_UART011_XFCR);
		context_uart[i].xon1 = readl(membase + ST_UART011_XON1);
		context_uart[i].xon2 = readl(membase + ST_UART011_XON2);
		context_uart[i].xoff1 = readl(membase + ST_UART011_XOFF1);
		context_uart[i].xoff2 = readl(membase + ST_UART011_XOFF2);
		context_uart[i].abcr = readl(membase + ST_UART011_ABCR);
		context_uart[i].abimsc = readl(membase + ST_UART011_ABIMSC);

		clk_disable(context_uart[i].uart_clk);
	}
}

static void restore_uart(void)
{
	int i, cnt;
	int retries = 100;
	unsigned int cr;
	void __iomem *membase;

	for (i = 0; i < UX500_NR_UARTS; i++) {
		if (i != ux500_debug_uart_config)
			continue;

		membase = context_uart[i].base;
		clk_enable(context_uart[i].uart_clk);

		writew(context_uart[i].ifls, membase + UART011_IFLS);
		cr = UART01x_CR_UARTEN | UART011_CR_TXE | UART011_CR_LBE;

		writew(cr, membase + UART011_CR);
		writew(0, membase + UART011_FBRD);
		writew(1, membase + UART011_IBRD);
		writew(0, membase + ST_UART011_LCRH_RX);
		if (context_uart[i].lcrh_tx != ST_UART011_LCRH_RX) {
			int i;
			/*
			 * Wait 10 PCLKs before writing LCRH_TX register,
			 * to get this delay write read only register 10 times
			 */
			for (i = 0; i < 10; ++i)
				writew(0xff, membase + UART011_MIS);
			writew(0, membase + ST_UART011_LCRH_TX);
		}
		writew(0, membase + UART01x_DR);
		do {
			if (!(readw(membase + UART01x_FR) & UART01x_FR_BUSY))
				break;
		} while (retries-- > 0);
		if (retries < 0)
			pr_warning("%s:uart tx busy\n", __func__);
		barrier();

		writel(context_uart[i].dma_wm, membase + ST_UART011_DMAWM);
		writel(context_uart[i].timeout, membase + ST_UART011_TIMEOUT);
		writel(context_uart[i].lcrh_rx, membase + ST_UART011_LCRH_RX);
		writel(context_uart[i].ilpr, membase + UART01x_ILPR);
		writel(context_uart[i].ibrd, membase + UART011_IBRD);
		writel(context_uart[i].fbrd, membase + UART011_FBRD);
		/*
		 * Wait 10 PCLKs before writing LCRH_TX register,
		 * to get this delay write read only register 10-3
		 * times, as already there are 3 writes after
		 * ST_UART011_LCRH_RX
		 */
		for (cnt = 0; cnt < 7; cnt++)
			writew(0xff, membase + UART011_MIS);

		writel(context_uart[i].lcrh_tx, membase + ST_UART011_LCRH_TX);
		writel(context_uart[i].ifls, membase + UART011_IFLS);
		writel(context_uart[i].dmacr, membase + UART011_DMACR);
		writel(context_uart[i].xfcr, membase + ST_UART011_XFCR);
		writel(context_uart[i].xon1, membase + ST_UART011_XON1);
		writel(context_uart[i].xon2, membase + ST_UART011_XON2);
		writel(context_uart[i].xoff1, membase + ST_UART011_XOFF1);
		writel(context_uart[i].xoff2, membase + ST_UART011_XOFF2);
		writel(context_uart[i].abcr, membase + ST_UART011_ABCR);
		writel(context_uart[i].abimsc, membase + ST_UART011_ABIMSC);
		writel(context_uart[i].cr, membase + UART011_CR);
		writel(context_uart[i].imsc, membase + UART011_IMSC);

		clk_disable(context_uart[i].uart_clk);
	}
}


static int uart_context_notifier_call(struct notifier_block *this,
				     unsigned long event, void *data)
{
	switch (event) {
	case CONTEXT_APE_SAVE:
		save_uart();
		break;

	case CONTEXT_APE_RESTORE:
		restore_uart();
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block uart_context_notifier = {
	.notifier_call = uart_context_notifier_call,
};

static void uart_context_notifier_init(void)
{
	context_uart[0].base = ioremap(U8500_UART0_BASE, SZ_4K);
	context_uart[1].base = ioremap(U8500_UART1_BASE, SZ_4K);
	context_uart[2].base = ioremap(U8500_UART2_BASE, SZ_4K);

	context_uart[0].uart_clk = clk_get_sys("uart0", NULL);
	if (IS_ERR(context_uart[0].uart_clk)) {
		printk(KERN_ERR"%s:unable to get clk-uart0\n", __func__);
		return;
	}
	context_uart[1].uart_clk = clk_get_sys("uart1", NULL);
	if (IS_ERR(context_uart[1].uart_clk)) {
		printk(KERN_ERR"%s:unable to get clk-uart1\n", __func__);
		return;
	}
	context_uart[2].uart_clk = clk_get_sys("uart2", NULL);
	if (IS_ERR(context_uart[2].uart_clk)) {
		printk(KERN_ERR"%s:unable to get clk-uart2\n", __func__);
		return;
	}

	WARN_ON(context_ape_notifier_register(&uart_context_notifier));
}
#else
static void uart_context_notifier_init(void)
{
}
#endif

void __init db8500_uart_init(void)
{
	uart_context_notifier_init();
}
