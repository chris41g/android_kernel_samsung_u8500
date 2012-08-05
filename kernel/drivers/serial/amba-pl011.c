/*
 *  linux/drivers/char/amba.c
 *
 *  Driver for AMBA serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This is a generic driver for ARM AMBA-type serial ports.  They
 * have a lot of 16550-like features, but are not register compatible.
 * Note that although they do have CTS, DCD and DSR inputs, they do
 * not have an RI input, nor do they have DTR or RTS outputs.  If
 * required, these have to be supplied via some other means (eg, GPIO)
 * and hooked into this driver.
 */

#if defined(CONFIG_SERIAL_AMBA_PL011_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <mach/uart.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/sizes.h>

#include <plat/gpio.h>
#include <plat/pincfg.h>

#define UART_NR			14

#define SERIAL_AMBA_MAJOR	204
#define SERIAL_AMBA_MINOR	64
#define SERIAL_AMBA_NR		UART_NR

#define AMBA_ISR_PASS_LIMIT	256

#define UART_DR_ERROR		(UART011_DR_OE|UART011_DR_BE|UART011_DR_PE|UART011_DR_FE)
#define UART_DUMMY_DR_RX	(1 << 16)

/*
 * The console UART is handled differently for power management (it doesn't
 * take the regulator, in order to allow the system to go to sleep even if the
 * console is open).  This should be removed once cable detect is in place.
 */
#ifdef CONFIG_SERIAL_CORE_CONSOLE
/* ttyAMA2 won't take the regulator even if it's not a console */
#if 1
#define uart_console(port)      ((port)->cons && 2 == (port)->line)
#else
#define uart_console(port)	((port)->cons \
		&& (port)->cons->index == (port)->line)
#endif
#else
#define uart_console(port)	(0)
#endif

/* Available amba pl011 port clock states */
enum pl011_clk_states {
	PL011_CLK_OFF = 0,	/* clock disabled */
	PL011_CLK_REQUEST_OFF,	/* disable after TX flushed */
	PL011_CLK_ON,		/* clock enabled */
	PL011_PORT_OFF,		/* port disabled */
};

/*
 * Backup registers to be used during regulator startup/shutdown
 */
static const u32 backup_regs[] = {
	UART011_IBRD,
	UART011_FBRD,
	ST_UART011_LCRH_RX,
	ST_UART011_LCRH_TX,
	UART011_CR,
	UART011_IMSC,
};

#define UART_WA_SAVE_NR	14

static void pl011_lockup_wa(unsigned long data);
static const u32 uart_wa_reg[UART_WA_SAVE_NR] = {
	ST_UART011_DMAWM,
	ST_UART011_TIMEOUT,
	ST_UART011_LCRH_RX,
	UART011_IBRD,
	UART011_FBRD,
	ST_UART011_LCRH_TX,
	UART011_IFLS,
	ST_UART011_XFCR,
	ST_UART011_XON1,
	ST_UART011_XON2,
	ST_UART011_XOFF1,
	ST_UART011_XOFF2,
	UART011_CR,
	UART011_IMSC
};

static u32 uart_wa_regdata[UART_WA_SAVE_NR];
static unsigned int uart_wa_tlet_line;
static DECLARE_TASKLET(pl011_lockup_tlet, pl011_lockup_wa,
		(unsigned long) &uart_wa_tlet_line);

/*
 * We wrap our port structure around the generic uart_port.
 */
struct uart_amba_port {
	struct uart_port	port;
	struct clk		*clk;
	unsigned int		im;		/* interrupt mask */
	unsigned int		old_status;
	unsigned int		ifls;		/* vendor-specific */
	unsigned int		lcrh_tx;	/* vendor-specific */
	unsigned int		lcrh_rx;	/* vendor-specific */
	unsigned int		old_cr;		/* state during shutdown */
	bool			oversampling;   /* vendor-specific */
	bool			interrupt_may_hang;   /* vendor-specific */
	bool			autorts;
#ifdef CONFIG_SERIAL_AMBA_PL011_CLOCK_CONTROL
	enum pl011_clk_states	clk_state;	/* actual clock state */
	struct delayed_work	clk_off_work;	/* work used for clock off */
	unsigned int		clk_off_delay;	/* clock off delay */
#endif
	struct regulator        *regulator;
	u32 backup[ARRAY_SIZE(backup_regs)];
};

/* There is by now at least one vendor with differing details, so handle it */
struct vendor_data {
	unsigned int		ifls;
	unsigned int		fifosize;
	unsigned int		lcrh_tx;
	unsigned int		lcrh_rx;
	bool			oversampling;
	bool			interrupt_may_hang;
};

static struct vendor_data vendor_arm = {
	.ifls			= UART011_IFLS_RX4_8|UART011_IFLS_TX4_8,
	.fifosize		= 16,
	.lcrh_tx		= UART011_LCRH,
	.lcrh_rx		= UART011_LCRH,
	.oversampling		= false,
};

static struct vendor_data vendor_st = {
	.ifls			= UART011_IFLS_RX_HALF|UART011_IFLS_TX_HALF,
	.fifosize		= 64,
	.lcrh_tx		= ST_UART011_LCRH_TX,
	.lcrh_rx		= ST_UART011_LCRH_RX,
	.oversampling		= true,
	.interrupt_may_hang	= true,
};

static struct uart_amba_port *amba_ports[UART_NR];

static void __pl011_startup(struct uart_amba_port *uap)
{
	unsigned int cr;

	writew(uap->ifls, uap->port.membase + UART011_IFLS);

	/*
	 * Provoke TX FIFO interrupt into asserting.
	 */
	cr = UART01x_CR_UARTEN | UART011_CR_TXE | UART011_CR_LBE;
	writew(cr, uap->port.membase + UART011_CR);
	writew(0, uap->port.membase + UART011_FBRD);
	writew(1, uap->port.membase + UART011_IBRD);
	writew(0, uap->port.membase + uap->lcrh_rx);
	if (uap->lcrh_tx != uap->lcrh_rx) {
		int i;
		/*
		 * Wait 10 PCLKs before writing LCRH_TX register,
		 * to get this delay write read only register 10 times
		 */
		for (i = 0; i < 10; ++i)
			writew(0xff, uap->port.membase + UART011_MIS);
		writew(0, uap->port.membase + uap->lcrh_tx);
	}
	if (uap->port.line==0) nmk_config_pin(PIN_CFG(3,GPIO)|PIN_OUTPUT_HIGH,false);
	writew(0, uap->port.membase + UART01x_DR);
	while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_BUSY)
		barrier();
	if (uap->port.line==0) nmk_config_pin(PIN_CFG(3,ALT_A)|PIN_OUTPUT_HIGH,false);
}

/* Backup the registers during regulator startup/shutdown */
#ifdef CONFIG_SERIAL_AMBA_PL011_CLOCK_CONTROL
static int pl011_backup(struct uart_amba_port *uap, bool suspend)
{
	int i, cnt;

	if (!suspend) {
		__pl011_startup(uap);
		writew(0, uap->port.membase + UART011_CR);
	}

	for (i = 0; i < ARRAY_SIZE(backup_regs); i++) {
		if (suspend)
			uap->backup[i] = readw(uap->port.membase +
						    backup_regs[i]);
		else {
			if (backup_regs[i] == ST_UART011_LCRH_TX) {
				/*
				 * Wait 10 PCLKs before writing LCRH_TX
				 * register, to get this delay write read
				 * only register 10 times
				 */
				for (cnt = 0; cnt < 10; ++cnt)
					writew(0xff, uap->port.membase +
							UART011_MIS);
			}

			writew(uap->backup[i],
			       uap->port.membase + backup_regs[i]);
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_SERIAL_AMBA_PL011_CLOCK_CONTROL
/* Turn clock off if TX buffer is empty, otherwise reschedule */
static void pl011_clock_off(struct work_struct *work)
{
	struct uart_amba_port *uap = container_of(work, struct uart_amba_port,
							clk_off_work.work);
	struct uart_port *port = &uap->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long flags;
	bool disable_regulator = false;
	unsigned int busy, interrupt_status;

	spin_lock_irqsave(&port->lock, flags);

	interrupt_status = readw(uap->port.membase + UART011_MIS);
	busy = readw(uap->port.membase + UART01x_FR) & UART01x_FR_BUSY;

	if (uap->clk_state == PL011_CLK_REQUEST_OFF) {
		if (uart_circ_empty(xmit) && !interrupt_status && !busy) {
			if (!uart_console(&uap->port) && uap->regulator) {
				pl011_backup(uap, true);
				disable_regulator = true;
			}
			uap->clk_state = PL011_CLK_OFF;
			clk_disable(uap->clk);
		} else
			schedule_delayed_work(&uap->clk_off_work,
						uap->clk_off_delay);
	}

	spin_unlock_irqrestore(&port->lock, flags);

	if (disable_regulator)
		regulator_disable(uap->regulator);
}

/* Request to turn off uart clock once pending TX is flushed */
static void pl011_clock_request_off(struct uart_port *port)
{
	unsigned long flags;
	struct uart_amba_port *uap = (struct uart_amba_port *)(port);

	spin_lock_irqsave(&port->lock, flags);

	if (uap->clk_state == PL011_CLK_ON) {
		uap->clk_state = PL011_CLK_REQUEST_OFF;
		/* Turn off later */
		schedule_delayed_work(&uap->clk_off_work,
			uap->clk_off_delay);
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

/* Request to immediately turn on uart clock */
static void pl011_clock_on(struct uart_port *port)
{
	unsigned long flags;
	struct uart_amba_port *uap = (struct uart_amba_port *)(port);

	spin_lock_irqsave(&port->lock, flags);

	switch (uap->clk_state) {
	case PL011_CLK_OFF:
		clk_enable(uap->clk);
		if (!uart_console(&uap->port) && uap->regulator) {
			spin_unlock_irqrestore(&port->lock, flags);
			regulator_enable(uap->regulator);
			spin_lock_irqsave(&port->lock, flags);
			pl011_backup(uap, false);
		}
		/* fallthrough */
	case PL011_CLK_REQUEST_OFF:
		cancel_delayed_work(&uap->clk_off_work);
		uap->clk_state = PL011_CLK_ON;
		break;
	default:
		break;
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

static void pl011_clock_check(struct uart_amba_port *uap)
{
	/* Reshedule work during off request  */
	if (uap->clk_state == PL011_CLK_REQUEST_OFF)
		/* New TX - restart work */
		if (cancel_delayed_work(&uap->clk_off_work))
			schedule_delayed_work(&uap->clk_off_work,
						uap->clk_off_delay);
}

static int pl011_power_startup(struct uart_amba_port *uap)
{
	int retval = 0;

	if (uap->clk_state == PL011_PORT_OFF) {
		if (!uart_console(&uap->port) && uap->regulator)
			regulator_enable(uap->regulator);
		retval = clk_enable(uap->clk);
		if (!retval)
			uap->clk_state = PL011_CLK_ON;
		else
			uap->clk_state = PL011_PORT_OFF;
	}

	return retval;
}

static void pl011_power_shutdown(struct uart_amba_port *uap)
{
	bool disable_regulator = false;

	cancel_delayed_work_sync(&uap->clk_off_work);

	spin_lock_irq(&uap->port.lock);
	if (uap->clk_state == PL011_CLK_ON ||
	   uap->clk_state ==  PL011_CLK_REQUEST_OFF) {
		clk_disable(uap->clk);
		if (!uart_console(&uap->port) && uap->regulator)
			disable_regulator = true;
	}
	uap->clk_state = PL011_PORT_OFF;
	spin_unlock_irq(&uap->port.lock);

	if (disable_regulator)
		regulator_disable(uap->regulator);
}

static void
pl011_clock_control(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
	speed_t new_baud = tty_termios_baud_rate(termios);

	if (new_baud == 0)
		pl011_clock_request_off(port);
	else
		pl011_clock_on(port);
}

static void pl011_clock_control_init(struct uart_amba_port *uap)
{
	uap->clk_state = PL011_PORT_OFF;
	INIT_DELAYED_WORK(&uap->clk_off_work, pl011_clock_off);
	uap->clk_off_delay = HZ / 10; /* 100 ms */
}

#else
/* Blank functions for clock control */
static inline void pl011_clock_check(struct uart_amba_port *uap)
{
}

static inline int pl011_power_startup(struct uart_amba_port *uap)
{
	return clk_enable(uap->clk);
}

static inline void pl011_power_shutdown(struct uart_amba_port *uap)
{
	clk_disable(uap->clk);
}

static inline void
pl011_clock_control(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
}

static inline void pl011_clock_control_init(struct uart_amba_port *uap)
{
}
#endif

/*
 * pl011_lockup_wa
 * This workaround aims to break the deadlock situation
 * when after long transfer over uart in hardware flow
 * control, uart interrupt registers cannot be cleared.
 * Hence uart transfer gets blocked.
 *
 * It is seen that during such deadlock condition ICR
 * don't get cleared even on multiple write. This leads
 * pass_counter to decrease and finally reach zero. This
 * can be taken as trigger point to run this UART_BT_WA.
 *
 */
static void pl011_lockup_wa(unsigned long data)
{
	struct uart_amba_port *uap = amba_ports[*(unsigned int *)data];
	void __iomem *base = uap->port.membase;
	struct circ_buf *xmit = &uap->port.state->xmit;
	struct tty_struct *tty = uap->port.state->port.tty;
	int buf_empty_retries = 200;
	int loop;

	/* Stop HCI layer from submitting data for tx */
	tty->hw_stopped = 1;

	while (!uart_circ_empty(xmit)) {
		if (buf_empty_retries-- == 0)
			break;
		udelay(100);
	}

	/* Backup registers */
	for (loop = 0; loop < UART_WA_SAVE_NR; loop++)
		uart_wa_regdata[loop] = readl(base + uart_wa_reg[loop]);

	/* Disable UART so that FIFO data is flushed out */
	writew(0x00, uap->port.membase + UART011_CR);

	/* Soft reset UART module */
	if (uap->port.dev->platform_data) {
		struct uart_amba_plat_data *plat;

		plat = uap->port.dev->platform_data;
		if (plat->reset)
			plat->reset();
	}

	/* Restore registers */
	for (loop = 0; loop < UART_WA_SAVE_NR; loop++)
		writew(uart_wa_regdata[loop] ,
				uap->port.membase + uart_wa_reg[loop]);

	/* Initialise the old status of the modem signals */
	uap->old_status = readw(uap->port.membase + UART01x_FR) &
				UART01x_FR_MODEM_ANY;

	if (readl(base + UART011_MIS) & 0x2)
		printk(KERN_EMERG "UART_BT_WA: ***FAILED***\n");

	/* Start Tx/Rx */
	tty->hw_stopped = 0;
}

static void pl011_stop_tx(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	uap->im &= ~UART011_TXIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void pl011_start_tx(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	uap->im |= UART011_TXIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void pl011_stop_rx(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	uap->im &= ~(UART011_RXIM|UART011_RTIM|UART011_FEIM|
		     UART011_PEIM|UART011_BEIM|UART011_OEIM);
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void pl011_enable_ms(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	uap->im |= UART011_RIMIM|UART011_CTSMIM|UART011_DCDMIM|UART011_DSRMIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void pl011_rx_chars(struct uart_amba_port *uap)
{
	struct tty_struct *tty = uap->port.state->port.tty;
	unsigned int status, ch, flag, max_count = 256;

	status = readw(uap->port.membase + UART01x_FR);
	while ((status & UART01x_FR_RXFE) == 0 && max_count--) {
		ch = readw(uap->port.membase + UART01x_DR) | UART_DUMMY_DR_RX;
		flag = TTY_NORMAL;
		uap->port.icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		if (unlikely(ch & UART_DR_ERROR)) {
			if (ch & UART011_DR_BE) {
				ch &= ~(UART011_DR_FE | UART011_DR_PE);
				uap->port.icount.brk++;
				if (uart_handle_break(&uap->port))
					goto ignore_char;
			} else if (ch & UART011_DR_PE)
				uap->port.icount.parity++;
			else if (ch & UART011_DR_FE)
				uap->port.icount.frame++;
			if (ch & UART011_DR_OE)
				uap->port.icount.overrun++;

			ch &= uap->port.read_status_mask;

			if (ch & UART011_DR_BE)
				flag = TTY_BREAK;
			else if (ch & UART011_DR_PE)
				flag = TTY_PARITY;
			else if (ch & UART011_DR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&uap->port, ch & 255))
			goto ignore_char;

		uart_insert_char(&uap->port, ch, UART011_DR_OE, ch, flag);

	ignore_char:
		status = readw(uap->port.membase + UART01x_FR);
	}
	spin_unlock(&uap->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&uap->port.lock);
}

static void pl011_tx_chars(struct uart_amba_port *uap)
{
	struct circ_buf *xmit = &uap->port.state->xmit;
	int count;

	if (uap->port.x_char) {
		writew(uap->port.x_char, uap->port.membase + UART01x_DR);
		uap->port.icount.tx++;
		uap->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&uap->port)) {
		pl011_stop_tx(&uap->port);
		return;
	}

	count = uap->port.fifosize >> 1;
	do {
		writew(xmit->buf[xmit->tail], uap->port.membase + UART01x_DR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uap->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (count)
		pl011_clock_check(uap);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uap->port);

	if (uart_circ_empty(xmit))
		pl011_stop_tx(&uap->port);
}

static void pl011_modem_status(struct uart_amba_port *uap)
{
	unsigned int status, delta;

	status = readw(uap->port.membase + UART01x_FR) & UART01x_FR_MODEM_ANY;

	delta = status ^ uap->old_status;
	uap->old_status = status;

	if (!delta)
		return;

	if (delta & UART01x_FR_DCD)
		uart_handle_dcd_change(&uap->port, status & UART01x_FR_DCD);

	if (delta & UART01x_FR_DSR)
		uap->port.icount.dsr++;

	if (delta & UART01x_FR_CTS)
		uart_handle_cts_change(&uap->port, status & UART01x_FR_CTS);

	wake_up_interruptible(&uap->port.state->port.delta_msr_wait);
}

static irqreturn_t pl011_int(int irq, void *dev_id)
{
	struct uart_amba_port *uap = dev_id;
	unsigned int status, pass_counter = AMBA_ISR_PASS_LIMIT;
	int handled = 0;

	spin_lock(&uap->port.lock);

	status = readw(uap->port.membase + UART011_MIS);
	if (status) {
		do {
			writew(status & ~(UART011_TXIS|UART011_RTIS|
					  UART011_RXIS),
				uap->port.membase + UART011_ICR);

			if (status & (UART011_RTIS|UART011_RXIS))
				pl011_rx_chars(uap);
			if (status & (UART011_DSRMIS|UART011_DCDMIS|
				      UART011_CTSMIS|UART011_RIMIS))
				pl011_modem_status(uap);
			if (status & UART011_TXIS)
				pl011_tx_chars(uap);

			if (pass_counter-- == 0) {
				if (uap->interrupt_may_hang) {
					uart_wa_tlet_line = uap->port.line;
					tasklet_schedule(&pl011_lockup_tlet);
				}
				break;
			}

			status = readw(uap->port.membase + UART011_MIS);
		} while (status != 0);
		handled = 1;
	}

	spin_unlock(&uap->port.lock);

	return IRQ_RETVAL(handled);
}

static unsigned int pl01x_tx_empty(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int status = readw(uap->port.membase + UART01x_FR);
	return status & (UART01x_FR_BUSY|UART01x_FR_TXFF) ? 0 : TIOCSER_TEMT;
}

static unsigned int pl01x_get_mctrl(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int result = 0;
	unsigned int status = readw(uap->port.membase + UART01x_FR);

#define TIOCMBIT(uartbit, tiocmbit)	\
	if (status & uartbit)		\
		result |= tiocmbit

	TIOCMBIT(UART01x_FR_DCD, TIOCM_CAR);
	TIOCMBIT(UART01x_FR_DSR, TIOCM_DSR);
	TIOCMBIT(UART01x_FR_CTS, TIOCM_CTS);
	TIOCMBIT(UART011_FR_RI, TIOCM_RNG);
#undef TIOCMBIT
	return result;
}

static void pl011_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int cr;

	cr = readw(uap->port.membase + UART011_CR);

#define	TIOCMBIT(tiocmbit, uartbit)		\
	if (mctrl & tiocmbit)		\
		cr |= uartbit;		\
	else				\
		cr &= ~uartbit

	TIOCMBIT(TIOCM_RTS, UART011_CR_RTS);
	TIOCMBIT(TIOCM_DTR, UART011_CR_DTR);
	TIOCMBIT(TIOCM_OUT1, UART011_CR_OUT1);
	TIOCMBIT(TIOCM_OUT2, UART011_CR_OUT2);
	TIOCMBIT(TIOCM_LOOP, UART011_CR_LBE);

	if (uap->autorts) {
		/* We need to disable auto-RTS if we want to turn RTS off */
		TIOCMBIT(TIOCM_RTS, UART011_CR_RTSEN);
	}
#undef TIOCMBIT

	writew(cr, uap->port.membase + UART011_CR);
}

static void pl011_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned long flags;
	unsigned int lcr_h;

	spin_lock_irqsave(&uap->port.lock, flags);
	lcr_h = readw(uap->port.membase + uap->lcrh_tx);
	if (break_state == -1)
		lcr_h |= UART01x_LCRH_BRK;
	else
		lcr_h &= ~UART01x_LCRH_BRK;
	writew(lcr_h, uap->port.membase + uap->lcrh_tx);
	spin_unlock_irqrestore(&uap->port.lock, flags);
}

#ifdef CONFIG_CONSOLE_POLL
static int pl010_get_poll_char(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int status;

	status = readw(uap->port.membase + UART01x_FR);
	if (status & UART01x_FR_RXFE)
		return NO_POLL_CHAR;

	return readw(uap->port.membase + UART01x_DR);
}

static void pl010_put_poll_char(struct uart_port *port,
			 unsigned char ch)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_TXFF)
		barrier();

	writew(ch, uap->port.membase + UART01x_DR);
}

#endif /* CONFIG_CONSOLE_POLL */

static int pl011_startup(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int cr;
	int retval;

	/*
	 * disable all interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = 0;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	writew(0xffff, uap->port.membase + UART011_ICR);
	spin_unlock_irq(&uap->port.lock);

	/*
	 * Try to enable the clock producer and the regulator.
	 */
	retval = pl011_power_startup(uap);
	if (retval)
		goto out;

	uap->port.uartclk = clk_get_rate(uap->clk);

	/*
	 * Clear previous interrupts before installing interrupt handler
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = 0;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	writew(0xffff, uap->port.membase + UART011_ICR);
	spin_unlock_irq(&uap->port.lock);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(uap->port.irq, pl011_int, 0, "uart-pl011", uap);
	if (retval)
		goto clk_dis;

	__pl011_startup(uap);

	/* restore RTS and DTR */
	cr = uap->old_cr & (UART011_CR_RTS | UART011_CR_DTR);
	cr |= UART01x_CR_UARTEN | UART011_CR_RXE | UART011_CR_TXE;
	writew(cr, uap->port.membase + UART011_CR);
	/*
	 * initialise the old status of the modem signals
	 */
	uap->old_status = readw(uap->port.membase + UART01x_FR) & UART01x_FR_MODEM_ANY;

	/*
	 * Finally, enable interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = UART011_RXIM | UART011_RTIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	spin_unlock_irq(&uap->port.lock);

	if (uap->port.dev->platform_data) {
		struct uart_amba_plat_data *plat;

		plat = uap->port.dev->platform_data;
		if (plat->init)
			plat->init();
	}

	return 0;

 clk_dis:
	pl011_power_shutdown(uap);
 out:
	return retval;
}

static void pl011_shutdown_channel(struct uart_amba_port *uap,
					unsigned int lcrh)
{
      unsigned long val;

      val = readw(uap->port.membase + lcrh);
      val &= ~(UART01x_LCRH_BRK | UART01x_LCRH_FEN);
      writew(val, uap->port.membase + lcrh);
}

static void pl011_shutdown(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int cr;

	/*
	 * disable all interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = 0;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	writew(0xffff, uap->port.membase + UART011_ICR);
	spin_unlock_irq(&uap->port.lock);

	/*
	 * Free the interrupt
	 */
	free_irq(uap->port.irq, uap);

	/*
	 * disable the port. It should not disable RTS and DTR.
	 * Also RTS and DTR state should be preserved to restore
	 * it during startup().
	 */
	uap->autorts = false;
	cr = readw(uap->port.membase + UART011_CR);
	uap->old_cr = cr;
	cr &= UART011_CR_RTS | UART011_CR_DTR;
	cr |= UART01x_CR_UARTEN | UART011_CR_TXE;
	writew(cr, uap->port.membase + UART011_CR);

	/*
	 * disable break condition and fifos
	 */
	pl011_shutdown_channel(uap, uap->lcrh_rx);
	if (uap->lcrh_rx != uap->lcrh_tx)
		pl011_shutdown_channel(uap, uap->lcrh_tx);

	/*
	 * Shut down the clock producer and the regulator
	 */
	pl011_power_shutdown(uap);

	if (uap->port.dev->platform_data) {
		struct uart_amba_plat_data *plat;

		plat = uap->port.dev->platform_data;
		if (plat->exit)
			plat->exit();
	}
}

/* Power/Clock management. */
static void pl011_serial_pm(struct uart_port *port, unsigned int state,
unsigned int oldstate)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	switch (state) {
		case 0: /*fully on */
			/*
			 * Enable the peripheral clock for this serial port.
			 * This is called on uart_open() or a resume event.
			 */
			pl011_power_startup(uap);
			break;
		case 3: /* powered down */
			/*
			 * Disable the peripheral clock for this serial port.
			 * This is called on uart_close() or a suspend event.
			 */
			pl011_power_shutdown(uap);
			break;
		default:
			printk(KERN_ERR "pl011_serial: unknown pm %d\n", state);
	}
}

static void
pl011_set_termios(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int lcr_h, old_cr;
	unsigned long flags;
	unsigned int baud, quot;

	/*
	 * Must be before uart_get_baud_rate() call, because
	 * this function changes baudrate to default in case of 0
	 * B0 hangup !!!
	 */
	pl011_clock_control(port, termios, old);

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0,
				  port->uartclk/(uap->oversampling ? 8 : 16));

	if (baud > port->uartclk/16)
		quot = DIV_ROUND_CLOSEST(port->uartclk * 8, baud);
	else
		quot = DIV_ROUND_CLOSEST(port->uartclk * 4, baud);

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr_h = UART01x_LCRH_WLEN_5;
		break;
	case CS6:
		lcr_h = UART01x_LCRH_WLEN_6;
		break;
	case CS7:
		lcr_h = UART01x_LCRH_WLEN_7;
		break;
	default: // CS8
		lcr_h = UART01x_LCRH_WLEN_8;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr_h |= UART01x_LCRH_STP2;
	if (termios->c_cflag & PARENB) {
		lcr_h |= UART01x_LCRH_PEN;
		if (!(termios->c_cflag & PARODD))
			lcr_h |= UART01x_LCRH_EPS;
	}
	if (port->fifosize > 1)
		lcr_h |= UART01x_LCRH_FEN;

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART011_DR_OE | 255;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART011_DR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART011_DR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART011_DR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_DR_RX;

	if (UART_ENABLE_MS(port, termios->c_cflag))
		pl011_enable_ms(port);

	/* first, disable everything */
	old_cr = readw(port->membase + UART011_CR);
	writew(0, port->membase + UART011_CR);

	if (termios->c_cflag & CRTSCTS) {
		if (old_cr & UART011_CR_RTS)
			old_cr |= UART011_CR_RTSEN;

		old_cr |= UART011_CR_CTSEN;
		uap->autorts = true;
	} else {
		old_cr &= ~(UART011_CR_CTSEN | UART011_CR_RTSEN);
		uap->autorts = false;
	}

	if (uap->oversampling) {
		if (baud > port->uartclk/16)
			old_cr |= ST_UART011_CR_OVSFACT;
		else
			old_cr &= ~ST_UART011_CR_OVSFACT;
	}

	/* Set baud rate */
	writew(quot & 0x3f, port->membase + UART011_FBRD);
	writew(quot >> 6, port->membase + UART011_IBRD);

	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	writew(lcr_h, port->membase + uap->lcrh_rx);
	if (uap->lcrh_rx != uap->lcrh_tx) {
		int i;
		/*
		 * Wait 10 PCLKs before writing LCRH_TX register,
		 * to get this delay write read only register 10 times
		 */
		for (i = 0; i < 10; ++i)
			writew(0xff, uap->port.membase + UART011_MIS);
		writew(lcr_h, port->membase + uap->lcrh_tx);
	}
	writew(old_cr, port->membase + UART011_CR);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *pl011_type(struct uart_port *port)
{
	return port->type == PORT_AMBA ? "AMBA/PL011" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void pl010_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, SZ_4K);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int pl010_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, SZ_4K, "uart-pl011")
			!= NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void pl010_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_AMBA;
		pl010_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int pl010_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_AMBA)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= nr_irqs)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops amba_pl011_pops = {
	.tx_empty	= pl01x_tx_empty,
	.set_mctrl	= pl011_set_mctrl,
	.get_mctrl	= pl01x_get_mctrl,
	.stop_tx	= pl011_stop_tx,
	.start_tx	= pl011_start_tx,
	.stop_rx	= pl011_stop_rx,
	.enable_ms	= pl011_enable_ms,
	.break_ctl	= pl011_break_ctl,
	.startup	= pl011_startup,
	.shutdown	= pl011_shutdown,
	.set_termios	= pl011_set_termios,
	.type		= pl011_type,
	.release_port	= pl010_release_port,
	.request_port	= pl010_request_port,
	.config_port	= pl010_config_port,
	.verify_port	= pl010_verify_port,
	.pm		= pl011_serial_pm,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = pl010_get_poll_char,
	.poll_put_char = pl010_put_poll_char,
#endif
};

#ifdef CONFIG_SERIAL_AMBA_PL011_CONSOLE

static void pl011_console_putchar(struct uart_port *port, int ch)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_TXFF)
		barrier();
	writew(ch, uap->port.membase + UART01x_DR);
}

static void
pl011_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_amba_port *uap = amba_ports[co->index];
	unsigned int status, old_cr, new_cr;
	unsigned long flags;
	int locked = 1;

	clk_enable(uap->clk);

	local_irq_save(flags);
	if (uap->port.sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock(&uap->port.lock);
	else
		spin_lock(&uap->port.lock);

	/*
	 *	First save the CR then disable the interrupts
	 */
	old_cr = readw(uap->port.membase + UART011_CR);
	new_cr = old_cr & ~UART011_CR_CTSEN;
	new_cr |= UART01x_CR_UARTEN | UART011_CR_TXE;
	writew(new_cr, uap->port.membase + UART011_CR);

	uart_console_write(&uap->port, s, count, pl011_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the TCR
	 */
	do {
		status = readw(uap->port.membase + UART01x_FR);
	} while (status & UART01x_FR_BUSY);
	writew(old_cr, uap->port.membase + UART011_CR);

	if (locked)
		spin_unlock(&uap->port.lock);
	local_irq_restore(flags);

	clk_disable(uap->clk);
}

static void __init
pl011_console_get_options(struct uart_amba_port *uap, int *baud,
			     int *parity, int *bits)
{
	if (readw(uap->port.membase + UART011_CR) & UART01x_CR_UARTEN) {
		unsigned int lcr_h, ibrd, fbrd;

		lcr_h = readw(uap->port.membase + uap->lcrh_tx);

		*parity = 'n';
		if (lcr_h & UART01x_LCRH_PEN) {
			if (lcr_h & UART01x_LCRH_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		if ((lcr_h & 0x60) == UART01x_LCRH_WLEN_7)
			*bits = 7;
		else
			*bits = 8;

		ibrd = readw(uap->port.membase + UART011_IBRD);
		fbrd = readw(uap->port.membase + UART011_FBRD);

		*baud = uap->port.uartclk * 4 / (64 * ibrd + fbrd);

		if (uap->oversampling) {
			if (readw(uap->port.membase + UART011_CR)
				  & ST_UART011_CR_OVSFACT)
				*baud *= 2;
		}
	}
}

static int __init pl011_console_setup(struct console *co, char *options)
{
	struct uart_amba_port *uap;
	int baud = 38400;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	uap = amba_ports[co->index];
	if (!uap)
		return -ENODEV;

	if (uap->port.dev->platform_data) {
		struct uart_amba_plat_data *plat;

		plat = uap->port.dev->platform_data;
		if (plat->init)
			plat->init();
	}

	uap->port.uartclk = clk_get_rate(uap->clk);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		pl011_console_get_options(uap, &baud, &parity, &bits);

	return uart_set_options(&uap->port, co, baud, parity, bits, flow);
}

static struct uart_driver amba_reg;
static struct console amba_console = {
	.name		= "ttyAMA",
	.write		= pl011_console_write,
	.device		= uart_console_device,
	.setup		= pl011_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &amba_reg,
};

#define AMBA_CONSOLE	(&amba_console)
#else
#define AMBA_CONSOLE	NULL
#endif

static struct uart_driver amba_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "ttyAMA",
	.dev_name		= "ttyAMA",
	.major			= SERIAL_AMBA_MAJOR,
	.minor			= SERIAL_AMBA_MINOR,
	.nr			= UART_NR,
	.cons			= AMBA_CONSOLE,
};

static int pl011_probe(struct amba_device *dev, struct amba_id *id)
{
	struct uart_amba_port *uap;
	struct vendor_data *vendor = id->data;
	void __iomem *base;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(amba_ports); i++)
		if (amba_ports[i] == NULL)
			break;

	if (i == ARRAY_SIZE(amba_ports)) {
		ret = -EBUSY;
		goto out;
	}

	uap = kzalloc(sizeof(struct uart_amba_port), GFP_KERNEL);
	if (uap == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	base = ioremap(dev->res.start, resource_size(&dev->res));
	if (!base) {
		ret = -ENOMEM;
		goto free;
	}

	uap->regulator = regulator_get(&dev->dev, "v-uart");
	if (IS_ERR(uap->regulator)) {
		dev_warn(&dev->dev, "could not get uart regulator\n");
		uap->regulator = NULL;
	}

	uap->clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(uap->clk)) {
		ret = PTR_ERR(uap->clk);
		goto unmap;
	}

	uap->ifls = vendor->ifls;
	uap->lcrh_rx = vendor->lcrh_rx;
	uap->lcrh_tx = vendor->lcrh_tx;
	uap->old_cr = 0;
	uap->oversampling = vendor->oversampling;
	uap->interrupt_may_hang = vendor->interrupt_may_hang;
	uap->port.dev = &dev->dev;
	uap->port.mapbase = dev->res.start;
	uap->port.membase = base;
	uap->port.iotype = UPIO_MEM;
	uap->port.irq = dev->irq[0];
	uap->port.fifosize = vendor->fifosize;
	uap->port.ops = &amba_pl011_pops;
	uap->port.flags = UPF_BOOT_AUTOCONF;
	uap->port.line = i;

	amba_ports[i] = uap;

	amba_set_drvdata(dev, uap);

	pl011_clock_control_init(uap);

	ret = uart_add_one_port(&amba_reg, &uap->port);
	if (ret) {
		amba_set_drvdata(dev, NULL);
		amba_ports[i] = NULL;
		clk_put(uap->clk);
 unmap:
		if (uap->regulator)
			regulator_put(uap->regulator);
		iounmap(base);
 free:
		kfree(uap);
	}
 out:
	return ret;
}

static int pl011_remove(struct amba_device *dev)
{
	struct uart_amba_port *uap = amba_get_drvdata(dev);
	int i;

	amba_set_drvdata(dev, NULL);

	uart_remove_one_port(&amba_reg, &uap->port);

	for (i = 0; i < ARRAY_SIZE(amba_ports); i++)
		if (amba_ports[i] == uap)
			amba_ports[i] = NULL;

	iounmap(uap->port.membase);
	if (uap->regulator)
		regulator_put(uap->regulator);
	clk_put(uap->clk);
	kfree(uap);
	return 0;
}

#ifdef CONFIG_PM
static int pl011_suspend(struct amba_device *dev, pm_message_t state)
{
	struct uart_amba_port *uap = amba_get_drvdata(dev);

	if (!uap)
		return -EINVAL;

#ifdef CONFIG_SERIAL_AMBA_PL011_CLOCK_CONTROL
	cancel_delayed_work_sync(&uap->clk_off_work);

	if (uap->clk_state == PL011_CLK_OFF)
		return 0;
#endif

	return uart_suspend_port(&amba_reg, &uap->port);
}

static int pl011_resume(struct amba_device *dev)
{
	struct uart_amba_port *uap = amba_get_drvdata(dev);

	if (!uap)
		return -EINVAL;

#ifdef CONFIG_SERIAL_AMBA_PL011_CLOCK_CONTROL
	if (uap->clk_state == PL011_CLK_OFF)
		return 0;
#endif

	return uart_resume_port(&amba_reg, &uap->port);
}
#endif

static struct amba_id pl011_ids[] = {
	{
		.id	= 0x00041011,
		.mask	= 0x000fffff,
		.data	= &vendor_arm,
	},
	{
		.id	= 0x00380802,
		.mask	= 0x00ffffff,
		.data	= &vendor_st,
	},
	{ 0, 0 },
};

static struct amba_driver pl011_driver = {
	.drv = {
		.name	= "uart-pl011",
	},
	.id_table	= pl011_ids,
	.probe		= pl011_probe,
	.remove		= pl011_remove,
#ifdef CONFIG_PM
	.suspend	= pl011_suspend,
	.resume		= pl011_resume,
#endif
};

static int __init pl011_init(void)
{
	int ret;
	printk(KERN_INFO "Serial: AMBA PL011 UART driver\n");

	ret = uart_register_driver(&amba_reg);
	if (ret == 0) {
		ret = amba_driver_register(&pl011_driver);
		if (ret)
			uart_unregister_driver(&amba_reg);
	}
	return ret;
}

static void __exit pl011_exit(void)
{
	amba_driver_unregister(&pl011_driver);
	uart_unregister_driver(&amba_reg);
}

/*
 * While this can be a module, if builtin it's most likely the console
 * So let's leave module_exit but move module_init to an earlier place
 */
arch_initcall(pl011_init);
module_exit(pl011_exit);

MODULE_AUTHOR("ARM Ltd/Deep Blue Solutions Ltd");
MODULE_DESCRIPTION("ARM AMBA serial port driver");
MODULE_LICENSE("GPL");
