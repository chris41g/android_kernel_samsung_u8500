/*
 * linux/drivers/mmc/core/sdio_irq.c
 *
 * Author:      Nicolas Pitre
 * Created:     June 18, 2007
 * Copyright:   MontaVista Software Inc.
 *
 * Copyright 2008 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>

#include "sdio_ops.h"

#define SDIO_IDLE_POLL_PERIOD 10

static int process_sdio_pending_irqs(struct mmc_card *card)
{
	int i, ret, count;
	unsigned char pending;

	/*
	 * Optimization, if there is only 1 function registered
	 * and IRQ:s are supported and currently enabled, then
	 * we can assume that this actually is an IRQ and we can
	 * call the registered IRQ handler directly without
	 * checking the CCCR registers.
	 */
	if ((card->host->caps & MMC_CAP_SDIO_IRQ) &&
	    card->host->sdio_irqs && (card->sdio_funcs == 1)) {
		struct sdio_func *func = card->sdio_func[0];
		if (func && func->irq_handler) {
			func->irq_handler(func);
			return 1;
		}
	}

	ret = mmc_io_rw_direct(card, 0, 0, SDIO_CCCR_INTx, 0, &pending);
	if (ret) {
		printk(KERN_DEBUG "%s: error %d reading SDIO_CCCR_INTx\n",
		       mmc_card_id(card), ret);
		return ret;
	}

	count = 0;
	for (i = 1; i <= 7; i++) {
		if (pending & (1 << i)) {
			struct sdio_func *func = card->sdio_func[i - 1];
			if (!func) {
				printk(KERN_WARNING "%s: pending IRQ for "
					"non-existant function\n",
					mmc_card_id(card));
				ret = -EINVAL;
			} else if (func->irq_handler) {
				func->irq_handler(func);
				count++;
			} else {
				printk(KERN_WARNING "%s: pending IRQ with no handler\n",
				       sdio_func_id(func));
				ret = -EINVAL;
			}
		}
	}

	if (count)
		return count;

	return ret;
}

static void sdio_irq_work_func(struct work_struct *work)
{
	struct mmc_host *host =	(struct mmc_host *)
		container_of(work, struct mmc_host, sdio_irq_work.work);
	int ret;

	/*
	 * We claim the host here on drivers behalf for a couple
	 * reasons:
	 *
	 * 1) it is already needed to retrieve the CCCR_INTx;
	 * 2) we want the driver(s) to clear the IRQ condition ASAP;
	 * 3) we need to control the abort condition locally.
	 *
	 * Just like traditional hard IRQ handlers, we expect SDIO
	 * IRQ handlers to be quick and to the point, so that the
	 * holding of the host lock does not cover too much work
	 * that doesn't require that lock to be held.
	 */
	mmc_claim_host(host);

	ret = process_sdio_pending_irqs(host->card);

	mmc_release_host(host);

	if (host->caps & MMC_CAP_SDIO_IRQ)
		host->ops->enable_sdio_irq(host, true);
	else {
		/*
		 * Adaptive polling frequency based on the assumption
		 * that an interrupt will be closely followed by more.
		 * This has a substantial benefit for network devices.
		 */
		if (ret > 0)
			host->sdio_poll_period /= 2;
		else {
			host->sdio_poll_period++;
			if (host->sdio_poll_period > SDIO_IDLE_POLL_PERIOD)
				host->sdio_poll_period = SDIO_IDLE_POLL_PERIOD;
		}
		queue_delayed_work(host->sdio_irq_workqueue,
				   &host->sdio_irq_work,
				   msecs_to_jiffies(host->sdio_poll_period));
	}
}

static int sdio_card_irq_get(struct mmc_card *card)
{
	struct mmc_host *host = card->host;

	WARN_ON(!host->claimed);

	if (!host->sdio_irqs++) {
		host->sdio_irq_workqueue =
			create_singlethread_workqueue("sdio_irq_workqueue");
		if (host->sdio_irq_workqueue == NULL) {
			host->sdio_irqs--;
			return -ENOMEM;
		}

		INIT_DELAYED_WORK(&host->sdio_irq_work, sdio_irq_work_func);

		/*
		 * We want to allow for SDIO cards to work even on non SDIO
		 * aware hosts.  One thing that non SDIO host cannot do is
		 * asynchronous notification of pending SDIO card interrupts
		 * hence we poll for them in that case.
		 */
		if (host->caps & MMC_CAP_SDIO_IRQ)
			host->ops->enable_sdio_irq(host, true);
		else {
			host->sdio_poll_period = SDIO_IDLE_POLL_PERIOD;
			queue_delayed_work(host->sdio_irq_workqueue,
				&host->sdio_irq_work,
				msecs_to_jiffies(host->sdio_poll_period));
		}
	}

	return 0;
}

static int sdio_card_irq_put(struct mmc_card *card)
{
	struct mmc_host *host = card->host;

	WARN_ON(!host->claimed);
	BUG_ON(host->sdio_irqs < 1);

	if (!--host->sdio_irqs) {
		host->ops->enable_sdio_irq(host, false);
		cancel_delayed_work_sync(&host->sdio_irq_work);
		destroy_workqueue(host->sdio_irq_workqueue);
		host->sdio_irq_workqueue = NULL;
	}

	return 0;
}

/**
 *	sdio_claim_irq - claim the IRQ for a SDIO function
 *	@func: SDIO function
 *	@handler: IRQ handler callback
 *
 *	Claim and activate the IRQ for the given SDIO function. The provided
 *	handler will be called when that IRQ is asserted.  The host is always
 *	claimed already when the handler is called so the handler must not
 *	call sdio_claim_host() nor sdio_release_host().
 */
int sdio_claim_irq(struct sdio_func *func, sdio_irq_handler_t *handler)
{
	int ret;
	unsigned char reg;

	BUG_ON(!func);
	BUG_ON(!func->card);

	pr_debug("SDIO: Enabling IRQ for %s...\n", sdio_func_id(func));

	if (func->irq_handler) {
		pr_debug("SDIO: IRQ for %s already in use.\n", sdio_func_id(func));
		return -EBUSY;
	}

	ret = mmc_io_rw_direct(func->card, 0, 0, SDIO_CCCR_IENx, 0, &reg);
	if (ret)
		return ret;

	reg |= 1 << func->num;

	reg |= 1; /* Master interrupt enable */

	ret = mmc_io_rw_direct(func->card, 1, 0, SDIO_CCCR_IENx, reg, NULL);
	if (ret)
		return ret;

	func->irq_handler = handler;
	ret = sdio_card_irq_get(func->card);
	if (ret)
		func->irq_handler = NULL;

	return ret;
}
EXPORT_SYMBOL_GPL(sdio_claim_irq);

/**
 *	sdio_release_irq - release the IRQ for a SDIO function
 *	@func: SDIO function
 *
 *	Disable and release the IRQ for the given SDIO function.
 */
int sdio_release_irq(struct sdio_func *func)
{
	int ret;
	unsigned char reg;

	BUG_ON(!func);
	BUG_ON(!func->card);

	pr_debug("SDIO: Disabling IRQ for %s...\n", sdio_func_id(func));

	if (func->irq_handler) {
		func->irq_handler = NULL;
		sdio_card_irq_put(func->card);
	}

	ret = mmc_io_rw_direct(func->card, 0, 0, SDIO_CCCR_IENx, 0, &reg);
	if (ret)
		return ret;

	reg &= ~(1 << func->num);

	/* Disable master interrupt with the last function interrupt */
	if (!(reg & 0xFE))
		reg = 0;

	ret = mmc_io_rw_direct(func->card, 1, 0, SDIO_CCCR_IENx, reg, NULL);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(sdio_release_irq);

