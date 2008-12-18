/*
 * based on linux/drivers/serial/s5pc100.c
 *
 * Driver for onboard UARTs on the Samsung S3C
 *
 * Based on drivers/char/serial.c and drivers/char/21285.c
 *
 * Ben Dooks, (c) 2003 Simtec Electronics
 *
 * Changelog:
 *
 * 22-Jul-2004  BJD  Finished off device rewrite
 *
 * 21-Jul-2004  BJD  Thanks to <herbet@13thfloor.at> for pointing out
 *                   problems with baud rate and loss of IR settings. Update
 *                   to add configuration via platform_device structure
 *
 * 28-Sep-2004  BJD  Re-write for the following items
 *		     - S3C serial support
 *		     - Power Management support
 *		     - Fix console via IrDA devices
 *		     - SysReq (Herbert Ptzl)
 *		     - Break character handling (Herbert Ptzl)
 *		     - spin-lock initialisation (Dimitry Andric)
 *		     - added clock control
 *		     - updated init code to use platform_device info
*/

/* Note on 24XX error handling
 *
 * The s3c2410 manual has a love/hate affair with the contents of the
 * UERSTAT register in the UART blocks, and keeps marking some of the
 * error bits as reserved. Having checked with the s3c2410x01,
 * it copes with BREAKs properly, so I am happy to ignore the RESERVED
 * feature from the latter versions of the manual.
 *
 * If it becomes aparrent that latter versions of the 2410 remove these
 * bits, then action will have to be taken to differentiate the versions
 * and change the policy on BREAK
 *
 * BJD, 04-Nov-2004
*/

#if defined(CONFIG_SERIAL_S5PC1XX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <asm/hardware.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/arch/gpio.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-s3c-clock.h>

/* structures */

struct s3c_uart_info {
	char			*name;
	unsigned int		type;
	unsigned int		fifosize;
	unsigned long		rx_fifomask;
	unsigned long		rx_fifoshift;
	unsigned long		rx_fifofull;
	unsigned long		tx_fifomask;
	unsigned long		tx_fifoshift;
	unsigned long		tx_fifofull;

	/* clock source control */

	int (*get_clksrc)(struct uart_port *, struct s3c24xx_uart_clksrc *clk);
	int (*set_clksrc)(struct uart_port *, struct s3c24xx_uart_clksrc *clk);

	/* uart controls */
	int (*reset_port)(struct uart_port *, struct s3c2410_uartcfg *);
};

struct s3c_uart_port {
	unsigned char			rx_claimed;
	unsigned char			tx_claimed;

	struct s3c_uart_info	*info;
	struct s3c24xx_uart_clksrc	*clksrc;
	struct clk			*clk;
	struct clk			*baudclk;
	struct uart_port		port;
};


/* configuration defines */

#if 1
#if 0
/* send debug to the low-level output routines */

extern void printascii(const char *);

static void
s3c_serial_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}

#define dbg(x...) s3c_serial_dbg(x)

#else
#define dbg(x...) printk(KERN_DEBUG "s3c: ");
#endif
#else /* no debug */
#define dbg(x...) do {} while(0)
#endif

/* UART name and device definitions */
#define S3C_SERIAL_NAME		"ttySAC"
//#define S3C_SERIAL_MAJOR	TTY_MAJOR
#define S3C_SERIAL_MAJOR	204
#define S3C_SERIAL_MINOR	64

/* conversion functions */
#define s3c_dev_to_port(__dev) (struct uart_port *)dev_get_drvdata(__dev)
#define s3c_dev_to_cfg(__dev) (struct s3c2410_uartcfg *)((__dev)->platform_data)

/* port irq numbers */
#define TX_IRQ(port)	((port)->irq + 1)
#define RX_IRQ(port)	((port)->irq)
#define UART_IRQ(port)	((port)->irq)

/* register access controls */
#define portaddr(port, reg) ((port)->membase + (reg))

#define rd_regb(port, reg) (__raw_readb(portaddr(port, reg)))
#define rd_regl(port, reg) (__raw_readl(portaddr(port, reg)))

#define wr_regb(port, reg, val) \
  do { __raw_writeb(val, portaddr(port, reg)); } while(0)

#define wr_regl(port, reg, val) \
  do { __raw_writel(val, portaddr(port, reg)); } while(0)

/* macros to change one thing to another */
#define tx_enabled(port) ((port)->unused[0])
#define rx_enabled(port) ((port)->unused[1])

const unsigned int nSlotTable[16] = {0x0000, 0x0080, 0x0808, 0x0888, 0x2222, 0x4924, 0x4a52, 0x54aa,
				     		0x5555, 0xd555, 0xd5d5, 0xddd5, 0xdddd, 0xdfdd, 0xdfdf, 0xffdf};

#ifdef UART_HAS_INTMSK
static inline void uart_enable_irq (struct uart_port *port, uint val)
{
	uint mask;

	mask = rd_regl(port, S3C_UINTMSK);
	mask &= ~val;
	wr_regl(port, S3C_UINTMSK, mask);
}

static inline void uart_disable_irq (struct uart_port *port, uint val)
{
	uint mask;

	mask = rd_regl(port, S3C_UINTMSK);
	mask |= val;
	wr_regl(port, S3C_UINTMSK, mask);

}
#endif

/* flag to ignore all characters comming in */
#define RXSTAT_DUMMY_READ (0x10000000)

/* now comes the code to initialise either the s3c serial
 * port information
 */
static int s3c_serial_getsource_hw(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C_UCON);

	switch (ucon & S3C_UCON_CLKMASK) {
	case S3C_UCON_UCLK:
		clk->divisor = 1;
		clk->name = "uclk";
		break;

	case S3C_UCON_PCLK:
	case S3C_UCON_PCLK2:
		clk->divisor = 1;
		clk->name = "pclkd1";
		break;

	case S3C_UCON_FCLK:

#if defined (CONFIG_SERIAL_S3C64XX_HS_UART)

		clk->divisor = 2;   /* For 4000000bps, epll_clk_192Mhz is needed. */
		clk->name = "epll_clk_uart_192m";

#else

		clk->divisor = 2;
		clk->name = "mpll_clk_uart";

#endif
		break;
	}

	return 0;
}


static int s3c_serial_setsource_hw(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C_UCON);

	ucon &= ~S3C_UCON_CLKMASK;

	if (strcmp(clk->name, "uclk") == 0)
		ucon |= S3C_UCON_UCLK;
	else if (strcmp(clk->name, "pclkd1") == 0)
		ucon |= S3C_UCON_PCLK;
	else if (strcmp(clk->name, "epll_clk_uart_192m") == 0) 
		ucon |= S3C_UCON_FCLK;
	else if (strcmp(clk->name, "mpll_clk_uart") == 0)
		ucon |= S3C_UCON_FCLK;
	else {
		printk(KERN_ERR "unknown clock source %s\n", clk->name);
		return -EINVAL;
	}

	wr_regl(port, S3C_UCON, ucon);
	return 0;
}

static inline struct s3c_uart_port *to_ourport(struct uart_port *port)
{
	return container_of(port, struct s3c_uart_port, port);
}

/* translate a port to the device name */

static inline const char *s3c_serial_portname(struct uart_port *port)
{
	return to_platform_device(port->dev)->name;
}

static int s3c_serial_txempty_nofifo(struct uart_port *port)
{
	return (rd_regl(port, S3C_UTRSTAT) & S3C_UTRSTAT_TXE);
}

static void s3c_serial_rx_enable(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ucon, ufcon;
	int count = 10000;

	spin_lock_irqsave(&port->lock, flags);

	while (--count && !s3c_serial_txempty_nofifo(port))
		udelay(100);

	ufcon = rd_regl(port, S3C_UFCON);
	ufcon |= S3C_UFCON_RESETRX;
	wr_regl(port, S3C_UFCON, ufcon);

	ucon = rd_regl(port, S3C_UCON);
	ucon |= S3C_UCON_RXIRQMODE;
	wr_regl(port, S3C_UCON, ucon);

	rx_enabled(port) = 1;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c_serial_rx_disable(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ucon;

	spin_lock_irqsave(&port->lock, flags);

	ucon = rd_regl(port, S3C_UCON);
	ucon &= ~S3C_UCON_RXIRQMODE;
	wr_regl(port, S3C_UCON, ucon);

	rx_enabled(port) = 0;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c_serial_stop_tx(struct uart_port *port)
{
	if (tx_enabled(port)) {
#ifdef UART_HAS_INTMSK
		uart_disable_irq(port, UART_TX_INT);
#else
		disable_irq(TX_IRQ(port));
#endif
		tx_enabled(port) = 0;
		if (port->flags & UPF_CONS_FLOW)
			s3c_serial_rx_enable(port);
	}
}

static void s3c_serial_start_tx(struct uart_port *port)
{
	if (!tx_enabled(port)) {
		if (port->flags & UPF_CONS_FLOW)
			s3c_serial_rx_disable(port);

#ifdef UART_HAS_INTMSK
		uart_enable_irq(port, UART_TX_INT);
#else
		enable_irq(TX_IRQ(port));
#endif
		tx_enabled(port) = 1;
	}
}


static void s3c_serial_stop_rx(struct uart_port *port)
{
	if (rx_enabled(port)) {
#ifdef UART_HAS_INTMSK
		uart_disable_irq(port, UART_RX_INT);
#else
		disable_irq(RX_IRQ(port));
#endif
		rx_enabled(port) = 0;
	}
}

static void s3c_serial_enable_ms(struct uart_port *port)
{
}

static inline struct s3c_uart_info *s3c_port_to_info(struct uart_port *port)
{
	return to_ourport(port)->info;
}

static inline struct s3c2410_uartcfg *s3c_port_to_cfg(struct uart_port *port)
{
	if (port->dev == NULL)
		return NULL;

	return (struct s3c2410_uartcfg *)port->dev->platform_data;
}

static int s3c_serial_rx_fifocnt(struct s3c_uart_port *ourport,
				     unsigned long ufstat)
{
	struct s3c_uart_info *info = ourport->info;

	if (ufstat & info->rx_fifofull)
		return info->fifosize;

	return (ufstat & info->rx_fifomask) >> info->rx_fifoshift;
}


/* ? - where has parity gone?? */
#define S3C_UERSTAT_PARITY (0x1000)

static irqreturn_t
s3c_serial_rx_chars(int irq, void *dev_id)
{
	struct s3c_uart_port *ourport = dev_id;
	struct uart_port *port = &ourport->port;
	struct tty_struct *tty = port->info->tty;
	unsigned int ufcon, ch, flag, ufstat, uerstat;
	int max_count = 64;

	while (max_count-- > 0) {
		ufcon = rd_regl(port, S3C_UFCON);
		ufstat = rd_regl(port, S3C_UFSTAT);

		if (s3c_serial_rx_fifocnt(ourport, ufstat) == 0)
			break;

		uerstat = rd_regl(port, S3C_UERSTAT);
		ch = rd_regb(port, S3C_URXH);

		if (port->flags & UPF_CONS_FLOW) {
			int txe = s3c_serial_txempty_nofifo(port);

			if (rx_enabled(port)) {
				if (!txe) {
					rx_enabled(port) = 0;
					continue;
				}
			} else {
				if (txe) {
					ufcon |= S3C_UFCON_RESETRX;
					wr_regl(port, S3C_UFCON, ufcon);
					rx_enabled(port) = 1;
					goto out;
				}
				continue;
			}
		}

		/* insert the character into the buffer */

		flag = TTY_NORMAL;
		port->icount.rx++;

		if (uerstat & S3C_UERSTAT_ANY) {
			dbg("rxerr: port ch=0x%02x, rxs=0x%08x\n", ch, uerstat);

			/* check for break */
			if (uerstat & S3C_UERSTAT_BREAK) {
				dbg("break!\n");
				port->icount.brk++;
				if (uart_handle_break(port))
				    goto ignore_char;
			}

			if (uerstat & S3C_UERSTAT_FRAME)
				port->icount.frame++;
			if (uerstat & S3C_UERSTAT_OVERRUN)
				port->icount.overrun++;

			uerstat &= port->read_status_mask;

			if (uerstat & S3C_UERSTAT_BREAK)
				flag = TTY_BREAK;
			else if (uerstat & S3C_UERSTAT_PARITY)
				flag = TTY_PARITY;
			else if (uerstat & ( S3C_UERSTAT_FRAME | S3C_UERSTAT_OVERRUN))
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		uart_insert_char(port, uerstat, S3C_UERSTAT_OVERRUN, ch, flag);

	ignore_char:
		continue;
	}
	tty_flip_buffer_push(tty);

 out:
	return IRQ_HANDLED;
}

static irqreturn_t s3c_serial_tx_chars(int irq, void *id)
{
	struct s3c_uart_port *ourport = id;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->info->xmit;
	int count = 256;

	if (port->x_char) {
		wr_regb(port, S3C_UTXH, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		goto out;
	}

	/* if there isnt anything more to transmit, or the uart is now
	 * stopped, disable the uart and exit
	*/

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		s3c_serial_stop_tx(port);
		goto out;
	}

	/* try and drain the buffer... */

	while (!uart_circ_empty(xmit) && count-- > 0) {
		if (rd_regl(port, S3C_UFSTAT) & ourport->info->tx_fifofull)
			break;

		wr_regb(port, S3C_UTXH, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		s3c_serial_stop_tx(port);

 out:
	return IRQ_HANDLED;
}

static unsigned int s3c_serial_tx_empty(struct uart_port *port)
{
	struct s3c_uart_info *info = s3c_port_to_info(port);
	unsigned long ufstat = rd_regl(port, S3C_UFSTAT);
	unsigned long ufcon = rd_regl(port, S3C_UFCON);

	if (ufcon & S3C_UFCON_FIFOMODE) {
		if ((ufstat & info->tx_fifomask) != 0 ||
		    (ufstat & info->tx_fifofull))
			return 0;

		return 1;
	}

	return s3c_serial_txempty_nofifo(port);
}

/* no modem control lines */
static unsigned int s3c_serial_get_mctrl(struct uart_port *port)
{
	unsigned int umstat = rd_regb(port,S3C_UMSTAT);

	if (umstat & S3C_UMSTAT_CTS)
		return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
	else
		return TIOCM_CAR | TIOCM_DSR;
}

static void s3c_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* todo - possibly remove AFC and do manual CTS */
}

static void s3c_serial_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	unsigned int ucon;

	spin_lock_irqsave(&port->lock, flags);

	ucon = rd_regl(port, S3C_UCON);

	if (break_state)
		ucon |= S3C_UCON_SBREAK;
	else
		ucon &= ~S3C_UCON_SBREAK;

	wr_regl(port, S3C_UCON, ucon);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c_serial_shutdown(struct uart_port *port)
{
	struct s3c_uart_port *ourport = to_ourport(port);

#ifdef UART_HAS_INTMSK
	if (ourport->tx_claimed || ourport->rx_claimed) {
		free_irq(UART_IRQ(port), ourport);

		ourport->tx_claimed = 0;
		ourport->rx_claimed = 0;
		tx_enabled(port) = 0;
		rx_enabled(port) = 0;
	}
#else
	if (ourport->tx_claimed) {
		free_irq(TX_IRQ(port), ourport);
		tx_enabled(port) = 0;
		ourport->tx_claimed = 0;
	}

	if (ourport->rx_claimed) {
		free_irq(RX_IRQ(port), ourport);
		ourport->rx_claimed = 0;
		rx_enabled(port) = 0;
	}
#endif
}

#ifdef UART_HAS_INTMSK
static irqreturn_t
s3c_serial_interrupt(int irq, void *dev_id)
{
        struct s3c_uart_port *ourport = dev_id;
        struct uart_port *port = &ourport->port;
        uint intpnd;

	intpnd = rd_regl(port, S3C_UINTPND);

        if (intpnd & UART_RX_INT) {
		wr_regl(port, S3C_UINTPND, UART_RX_INT);
		rd_regl(port, S3C_UINTPND); /*clearing the interrupt*/

		return s3c_serial_rx_chars( irq, dev_id);
        }
        else if (intpnd & UART_TX_INT) {
		wr_regl(port, S3C_UINTPND, UART_TX_INT);
		rd_regl(port, S3C_UINTPND); /*clearing the interrupt*/
		return s3c_serial_tx_chars(irq, dev_id);
        }
        else if (intpnd & UART_ERR_INT) {
		printk("UART err int.\n");
		wr_regl(port, S3C_UINTPND, UART_ERR_INT);
		rd_regl(port, S3C_UINTPND); /*clearing the interrupt*/
		return IRQ_HANDLED;
        }
        else if (intpnd & UART_MODEM_INT) {
		printk("In Modem ... other case\n");
		wr_regl(port, S3C_UINTPND, UART_MODEM_INT);
		rd_regl(port, S3C_UINTPND); /*clearing the interrupt*/
		return IRQ_HANDLED;
	}
	// work around h/w problem : interrupt status bit update delay
	//return IRQ_NONE;
	return IRQ_HANDLED;

}
#endif

static int s3c_serial_startup(struct uart_port *port)
{
	struct s3c_uart_port *ourport = to_ourport(port);
	unsigned long flags;
	int ret;

	dbg("s3c_serial_startup: port=%p (%08lx,%p)\n",
	    port->mapbase, port->membase);

	local_irq_save(flags);

	rx_enabled(port) = 1;
	tx_enabled(port) = 1;
	
#ifdef UART_HAS_INTMSK
	uart_enable_irq(port, UART_TX_INT | UART_RX_INT);
	ret = request_irq(UART_IRQ(port), s3c_serial_interrupt, 0,
			  s3c_serial_portname(port), ourport);
	if (ret != 0) {
		printk(KERN_ERR "cannot get irq %d\n", UART_IRQ(port));
		return ret;
	}
#else
	ret = request_irq(RX_IRQ(port),
			  s3c_serial_rx_chars, 0,
			  s3c_serial_portname(port), ourport);
	if (ret != 0) {
		printk(KERN_ERR "cannot get irq %d\n", RX_IRQ(port));
		return ret;
	}

	ret = request_irq(TX_IRQ(port),
			  s3c_serial_tx_chars, 0,
			  s3c_serial_portname(port), ourport);
	if (ret) {
		printk(KERN_ERR "cannot get irq %d\n", TX_IRQ(port));
		s3c_serial_shutdown(port);
		local_irq_restore(flags);
		return ret;
	}
#endif

	ourport->rx_claimed = 1;
	ourport->tx_claimed = 1;

	dbg("s3c_serial_startup ok\n");

	/* the port reset code should have done the correct
	 * register setup for the port controls */
	local_irq_restore(flags);
	return ret;
}

/* power power management control */
static void s3c_serial_pm(struct uart_port *port, unsigned int level,
			      unsigned int old)
{
	struct s3c_uart_port *ourport = to_ourport(port);

	switch (level) {
	case 3:
		if (!IS_ERR(ourport->baudclk) && ourport->baudclk != NULL)
			clk_disable(ourport->baudclk);

		clk_disable(ourport->clk);
		break;

	case 0:
		clk_enable(ourport->clk);

		if (!IS_ERR(ourport->baudclk) && ourport->baudclk != NULL)
			clk_enable(ourport->baudclk);

		break;
	default:
		printk(KERN_ERR "s3c_serial: unknown pm %d\n", level);
	}
}

/* baud rate calculation
 *
 * The UARTs on the S3C can take their clocks from a number
 * of different sources, including the peripheral clock ("pclk") and an
 * external clock ("uclk"). The S3C2413 also adds the core clock ("fclk")
 * with a programmable extra divisor.
 *
 * The following code goes through the clock sources, and calculates the
 * baud clocks (and the resultant actual baud rates) and then tries to
 * pick the closest one and select that.
 *
 * NOTES:
 *	1) there is no current code to properly select/deselect FCLK on
 *	   the s3c2413, so only specify FCLK or non-FCLK in the clock
 *	   sources for the UART
 *
*/


#define MAX_CLKS (8)

static struct s3c24xx_uart_clksrc tmp_clksrc = {
	.name		= "pclkd1",
	.min_baud	= 0,
	.max_baud	= 0,
	.divisor	= 1,
};

static inline int
s3c_serial_getsource(struct uart_port *port, struct s3c24xx_uart_clksrc *c)
{
	struct s3c_uart_info *info = s3c_port_to_info(port);

	return (info->get_clksrc)(port, c);
}

static inline int
s3c_serial_setsource(struct uart_port *port, struct s3c24xx_uart_clksrc *c)
{
	struct s3c_uart_info *info = s3c_port_to_info(port);

	return (info->set_clksrc)(port, c);
}


struct baud_calc {
	struct s3c24xx_uart_clksrc	*clksrc;
	unsigned int			 calc;
	unsigned int			 quot;
	unsigned int			 slot;
	struct clk			*src;
};

static int s3c_serial_calcbaud(struct baud_calc *calc,
				   struct uart_port *port,
				   struct s3c24xx_uart_clksrc *clksrc,
				   unsigned int baud)
{
	unsigned long rate,nslot;
	unsigned long tempdiv;

	calc->src = clk_get(port->dev, clksrc->name);
	if (calc->src == NULL || IS_ERR(calc->src))
		return 0;

	baud/=10;

	rate = clk_get_rate(calc->src);
	rate /= clksrc->divisor;

	calc->clksrc = clksrc;

	tempdiv = (rate*10/(baud*16))-100;
	nslot = (((tempdiv%100)*16)+50)/100; /* 50 means round operation by ryu */
	calc->quot = tempdiv/100;
	calc->calc = (rate / (calc->quot * 16));
	calc->slot = nSlotTable[nslot];
	printk(KERN_DEBUG  "FOUND quot %d, calc %d, slot 0x%x, nslot %ld\n",
			       calc->quot, calc->calc ,
			       calc->slot, nslot);
	return 1;
}

static unsigned int s3c_serial_getclk(struct uart_port *port,
					  struct s3c24xx_uart_clksrc **clksrc,
					  struct clk **clk,
					  unsigned int baud, unsigned int *slot)
{
	struct s3c2410_uartcfg *cfg = s3c_port_to_cfg(port);
	struct s3c24xx_uart_clksrc *clkp;
	struct baud_calc res[MAX_CLKS];
	struct baud_calc *resptr, *best, *sptr;
	int i;

	clkp = cfg->clocks;
	best = NULL;

	if (cfg->clocks_size < 2) {
		if (cfg->clocks_size == 0)
			clkp = &tmp_clksrc;

		/* check to see if we're sourcing fclk, and if so we're
		 * going to have to update the clock source
		 */

		if (strcmp(clkp->name, "fclk") == 0) {
			struct s3c24xx_uart_clksrc src;

			s3c_serial_getsource(port, &src);

			/* check that the port already using fclk, and if
			 * not, then re-select fclk
			 */

			if (strcmp(src.name, clkp->name) == 0) {
				s3c_serial_setsource(port, clkp);
				s3c_serial_getsource(port, &src);
			}

			clkp->divisor = src.divisor;
		}

		s3c_serial_calcbaud(res, port, clkp, baud);
		best = res;
		resptr = best + 1;
	} else {
		resptr = res;

		for (i = 0; i < cfg->clocks_size; i++, clkp++) {
			if (s3c_serial_calcbaud(resptr, port, clkp, baud))
				resptr++;
		}
	}

	/* ok, we now need to select the best clock we found */

	if (best==NULL) {
		unsigned int deviation = (1<<30)|((1<<30)-1);
		int calc_deviation;
		
		best = sptr = res;

		for (sptr = res; sptr < resptr; sptr++) {
			printk(KERN_DEBUG
			       "found clk %p (%s) quot %d, calc %d\n",
			       sptr->clksrc, sptr->clksrc->name,
			       sptr->quot, sptr->calc);

			calc_deviation = baud - sptr->calc;
			
			if (calc_deviation < 0)
				calc_deviation = -calc_deviation;

			if (calc_deviation < deviation) {
				best = sptr;
				deviation = calc_deviation;
			}
		}

		printk(KERN_DEBUG "The best clksrc among candidate clksrc: %p(%s) (deviation: %d bps)\n", best, best->clksrc->name, deviation);
	}

	printk(KERN_DEBUG "Selected clock is %p (%s) quot %d, calc %d\n",
	       best->clksrc, best->clksrc->name, best->quot, best->calc);

	/* store results to pass back */
	*clksrc = best->clksrc;
	*clk    = best->src;
	*slot = best->slot;
	
	return best->quot;
}

static void s3c_serial_set_termios(struct uart_port *port,
				       struct ktermios *termios,
				       struct ktermios *old)
{
	struct s3c2410_uartcfg *cfg = s3c_port_to_cfg(port);
	struct s3c_uart_port *ourport = to_ourport(port);
	struct s3c24xx_uart_clksrc *clksrc = NULL;
	struct clk *clk = NULL;
	unsigned long flags;
	unsigned int baud, quot, slot = 0;
	unsigned int ulcon;
	unsigned int umcon;

	/*
	 * We don't support modem control lines.
	 */
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * Ask the core to calculate the divisor for us.
	 */

	baud = uart_get_baud_rate(port, termios, old, 0, 4000000);

	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
		quot = port->custom_divisor;
	else
		quot = s3c_serial_getclk(port, &clksrc, &clk, baud, &slot);

	/* check to see if we need  to change clock source */

	if (ourport->clksrc != clksrc || ourport->baudclk != clk) {
		s3c_serial_setsource(port, clksrc);

		if (ourport->baudclk != NULL && !IS_ERR(ourport->baudclk)) {
			clk_disable(ourport->baudclk);
			ourport->baudclk  = NULL;
		}

		clk_enable(clk);

		ourport->clksrc = clksrc;
		ourport->baudclk = clk;
	}

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		dbg("config: 5bits/char\n");
		ulcon = S3C_LCON_CS5;
		break;
	case CS6:
		dbg("config: 6bits/char\n");
		ulcon = S3C_LCON_CS6;
		break;
	case CS7:
		dbg("config: 7bits/char\n");
		ulcon = S3C_LCON_CS7;
		break;
	case CS8:
	default:
		dbg("config: 8bits/char\n");
		ulcon = S3C_LCON_CS8;
		break;
	}

	/* preserve original lcon IR settings */
	ulcon |= (cfg->ulcon & S3C_LCON_IRM);

	if (termios->c_cflag & CSTOPB)
		ulcon |= S3C_LCON_STOPB;

	umcon = (termios->c_cflag & CRTSCTS) ? S3C_UMCOM_AFC : 0;

	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			ulcon |= S3C_LCON_PODD;
		else
			ulcon |= S3C_LCON_PEVEN;
	} else {
		ulcon |= S3C_LCON_PNONE;
	}

	spin_lock_irqsave(&port->lock, flags);

	dbg("setting ulcon to %08x, brddiv to %d\n", ulcon, quot);

	wr_regl(port, S3C_ULCON, ulcon);
	wr_regl(port, S3C_UBRDIV, quot);
	wr_regl(port, S3C_UDIVSLOT, slot);
	wr_regl(port, S3C_UMCON, umcon);

	dbg("uart: ulcon = 0x%08x, ucon = 0x%08x, ufcon = 0x%08x\n",
	    rd_regl(port, S3C_ULCON),
	    rd_regl(port, S3C_UCON),
	    rd_regl(port, S3C_UFCON));

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Which character status flags are we interested in?
	 */
	port->read_status_mask = S3C_UERSTAT_OVERRUN;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= S3C_UERSTAT_FRAME | S3C_UERSTAT_PARITY;

	/*
	 * Which character status flags should we ignore?
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= S3C_UERSTAT_OVERRUN;
	if (termios->c_iflag & IGNBRK && termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= S3C_UERSTAT_FRAME;

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= RXSTAT_DUMMY_READ;

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *s3c_serial_type(struct uart_port *port)
{
	switch (port->type) {
	case PORT_S3C2410:
		return "S3C";
	default:
		return NULL;
	}
}

#define MAP_SIZE (0x100)

static void s3c_serial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, MAP_SIZE);
}

static int s3c_serial_request_port(struct uart_port *port)
{
	const char *name = s3c_serial_portname(port);
	return request_mem_region(port->mapbase, MAP_SIZE, name) ? 0 : -EBUSY;
}

static void s3c_serial_config_port(struct uart_port *port, int flags)
{
	struct s3c_uart_info *info = s3c_port_to_info(port);

	if (flags & UART_CONFIG_TYPE && s3c_serial_request_port(port) == 0)
		port->type = info->type;
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int
s3c_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct s3c_uart_info *info = s3c_port_to_info(port);

	if (ser->type != PORT_UNKNOWN && ser->type != info->type)
		return -EINVAL;

	return 0;
}


#ifdef CONFIG_SERIAL_S5PC1XX_CONSOLE

static struct console s3c_serial_console;

#define S3C64XX_SERIAL_CONSOLE &s3c_serial_console
#else
#define S3C64XX_SERIAL_CONSOLE NULL
#endif

static struct uart_ops s3c_serial_ops = {
	.pm		= s3c_serial_pm,
	.tx_empty	= s3c_serial_tx_empty,
	.get_mctrl	= s3c_serial_get_mctrl,
	.set_mctrl	= s3c_serial_set_mctrl,
	.stop_tx	= s3c_serial_stop_tx,
	.start_tx	= s3c_serial_start_tx,
	.stop_rx	= s3c_serial_stop_rx,
	.enable_ms	= s3c_serial_enable_ms,
	.break_ctl	= s3c_serial_break_ctl,
	.startup	= s3c_serial_startup,
	.shutdown	= s3c_serial_shutdown,
	.set_termios	= s3c_serial_set_termios,
	.type		= s3c_serial_type,
	.release_port	= s3c_serial_release_port,
	.request_port	= s3c_serial_request_port,
	.config_port	= s3c_serial_config_port,
	.verify_port	= s3c_serial_verify_port,
};


static struct uart_driver s3c_uart_drv = {
	.owner		= THIS_MODULE,
	.dev_name	= "s3c_serial",
	.nr		= NR_PORTS,
	.cons		= S3C64XX_SERIAL_CONSOLE,
	.driver_name	= S3C_SERIAL_NAME,
	.major		= S3C_SERIAL_MAJOR,
	.minor		= S3C_SERIAL_MINOR,
};

static struct s3c_uart_port s3c_serial_ports[NR_PORTS] = {
	[0] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= S3C_IRQ_UART0,
			.uartclk	= 0,
			.fifosize	= UART_FIFO_SIZE,
			.ops		= &s3c_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 0,
		}
	},
	[1] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= S3C_IRQ_UART1,
			.uartclk	= 0,
			.fifosize	= UART_FIFO_SIZE,
			.ops		= &s3c_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 1,
		}
	},
#if NR_PORTS > 2
	[2] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= S3C_IRQ_UART2,
			.uartclk	= 0,
			.fifosize	= UART_FIFO_SIZE,
			.ops		= &s3c_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 2,
		}
	},
	[3] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= S3C_IRQ_UART3,
			.uartclk	= 0,
			.fifosize	= UART_FIFO_SIZE,
			.ops		= &s3c_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 3,
		}
	}
#endif
};

/* s3c_serial_resetport
 *
 * wrapper to call the specific reset for this port (reset the fifos
 * and the settings)
*/

static int s3c_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
#if 0
	struct s3c_uart_info *info = s3c_port_to_info(port);

	return (info->reset_port)(port, cfg);
#else
	/* ensure registers are setup */
	dbg("s3c_serial_resetport: port=%p (%08lx), cfg=%p\n",
	    port, port->mapbase, cfg);

	wr_regl(port, S3C_UCON,  cfg->ucon);
	wr_regl(port, S3C_ULCON, cfg->ulcon);

	/* reset both fifos */
	wr_regl(port, S3C_UFCON, cfg->ufcon | S3C_UFCON_RESETBOTH);
	wr_regl(port, S3C_UFCON, cfg->ufcon);

	return 0;

#endif
}

/* s3c_serial_init_port
 *
 * initialise a single serial port from the platform device given
 */

static int s3c_serial_init_port(struct s3c_uart_port *ourport,
				    struct s3c_uart_info *info,
				    struct platform_device *plat_dev)
{
	struct uart_port *port = &ourport->port;
	struct s3c2410_uartcfg *cfg;
	struct resource *res;

	dbg("s3c_serial_init_port: port=%p, plat_dev=%p\n", port, plat_dev);

	if (plat_dev == NULL)
		return -ENODEV;

	cfg = s3c_dev_to_cfg(&plat_dev->dev);

	if (port->mapbase != 0)
		return 0;

	if (cfg->hwport > NR_PORTS)
		return -EINVAL;

	/*GPIO setup for each port*/
	switch(port->line) {
	case 0:
		gpio_set_pin(S3C_GPA0, S3C_GPA0_UART_RXD0);
		gpio_set_pin(S3C_GPA1, S3C_GPA1_UART_TXD0);
		gpio_set_pin(S3C_GPA2, S3C_GPA2_UART_CTS0);
		gpio_set_pin(S3C_GPA3, S3C_GPA3_UART_RTS0);
		break;
	case 1:
		gpio_set_pin(S3C_GPA4, S3C_GPA4_UART_RXD1);
		gpio_set_pin(S3C_GPA5, S3C_GPA5_UART_TXD1);
		gpio_set_pin(S3C_GPA5, S3C_GPA6_UART_CTS1);
		gpio_set_pin(S3C_GPA7, S3C_GPA7_UART_RTS1);
		break;
	case 2:
		gpio_set_pin(S3C_GPB0, S3C_GPB0_UART_RXD2);
		gpio_set_pin(S3C_GPB1, S3C_GPB1_UART_TXD2);
		break;
	case 3:
		gpio_set_pin(S3C_GPB2, S3C_GPB2_UART_RXD3);
		gpio_set_pin(S3C_GPB3, S3C_GPB3_UART_TXD3);
		break;
	default:
		break;
	}

	/* setup info for port */
	port->dev	= &plat_dev->dev;
	ourport->info	= info;

	/* copy the info in from provided structure */
	ourport->port.fifosize = info->fifosize;

	dbg("s3c_serial_init_port: %p (hw %d)...\n", port, cfg->hwport);

	port->uartclk = UART_CLK;

	if (cfg->uart_flags & UPF_CONS_FLOW) {
		dbg("s3c_serial_init_port: enabling flow control\n");
		port->flags |= UPF_CONS_FLOW;
	}

	/* sort our the physical and virtual addresses for each UART */

	res = platform_get_resource(plat_dev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "failed to find memory resource for uart\n");
		return -EINVAL;
	}

	dbg("resource %p (%lx..%lx)\n", res, res->start, res->end);

	port->mapbase	= res->start;
	port->membase	= S3C24XX_VA_UART + (res->start - S3C24XX_PA_UART);
	port->irq	= platform_get_irq(plat_dev, 0);

	ourport->clk	= clk_get(&plat_dev->dev, "UART0");

	dbg("port: map=%08x, mem=%08x, irq=%d, clock=%ld\n",
	    port->mapbase, port->membase, port->irq, port->uartclk);

	/* reset the fifos (and setup the uart) */
	s3c_serial_resetport(port, cfg);
	return 0;
}

/* Device driver serial port probe */
static int probe_index = 0;

static int s3c6400_serial_probe(struct platform_device *dev,
				struct s3c_uart_info *info)
{
	struct s3c_uart_port *ourport;
	int ret;

	dbg("s3c_serial_probe(%p) %d\n", dev, probe_index);

	ourport = &s3c_serial_ports[probe_index];
	probe_index++;

	dbg("%s: initialising port %p...\n", __FUNCTION__, ourport);

	//ret = s3c_serial_init_port(ourport, &s3c_uart_inf, dev);
	ret = s3c_serial_init_port(ourport, info, dev);
	if (ret < 0)
		goto probe_err;

	dbg("%s: adding port\n", __FUNCTION__);
	uart_add_one_port(&s3c_uart_drv, &ourport->port);
	platform_set_drvdata(dev, &ourport->port);

	return 0;

 probe_err:
	return ret;
}

static int s3c_serial_remove(struct platform_device *dev)
{
	struct uart_port *port = s3c_dev_to_port(&dev->dev);

	if (port)
		uart_remove_one_port(&s3c_uart_drv, port);

	return 0;
}

/* UART power management code */

#ifdef CONFIG_PM

static int s3c_serial_suspend(struct platform_device *dev, pm_message_t state)
{
	struct uart_port *port = s3c_dev_to_port(&dev->dev);

	if (port)
		uart_suspend_port(&s3c_uart_drv, port);

	return 0;
}

static int s3c_serial_resume(struct platform_device *dev)
{
	struct uart_port *port = s3c_dev_to_port(&dev->dev);
	struct s3c_uart_port *ourport = to_ourport(port);

	if (port) {
		clk_enable(ourport->clk);
		s3c_serial_resetport(port, s3c_port_to_cfg(port));
		clk_disable(ourport->clk);

		uart_resume_port(&s3c_uart_drv, port);
	}

	return 0;
}

#else
#define s3c_serial_suspend NULL
#define s3c_serial_resume  NULL
#endif

static int s3c_serial_init(struct platform_driver *drv,
			       struct s3c_uart_info *info)
{
	dbg("s3c64xx_serial_init(%p,%p)\n", drv, info);
	return platform_driver_register(drv);
}

static struct s3c_uart_info s3c_uart_inf = {
	.name		= "Samsung S3C UART",
	.type		= PORT_S3C2410,
	.fifosize	= UART_FIFO_SIZE,
	.rx_fifomask	= S3C_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C_UFSTAT_RXFULL,
	.tx_fifofull	= S3C_UFSTAT_TXFULL,
	.tx_fifomask	= S3C_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C_UFSTAT_TXSHIFT,
	.get_clksrc	= s3c_serial_getsource_hw,
	.set_clksrc	= s3c_serial_setsource_hw,
//	.reset_port	= s3c_serial_resetport,
};

/* device management */

static int s3c_serial_probe(struct platform_device *dev)
{
	return s3c6400_serial_probe(dev, &s3c_uart_inf);
}

static struct platform_driver s3c_serial_drv = {
	.probe		= s3c_serial_probe,
	.remove		= s3c_serial_remove,
	.suspend	= s3c_serial_suspend,
	.resume		= s3c_serial_resume,
	.driver		= {
		.name	= "s3c-uart",
		.owner	= THIS_MODULE,
	},
};

static inline int s3c6400_serial_init(void)
{
	return s3c_serial_init(&s3c_serial_drv, &s3c_uart_inf);
}

static inline void s3c6400_serial_exit(void)
{
	platform_driver_unregister(&s3c_serial_drv);
}


/* module initialisation code */
static int __init s3c_serial_modinit(void)
{
	int ret;

	ret = uart_register_driver(&s3c_uart_drv);
	if (ret < 0) {
		printk(KERN_ERR "failed to register UART driver\n");
		return -1;
	}

	s3c6400_serial_init();
	
	return 0;
}

static void __exit s3c_serial_modexit(void)
{
	s3c6400_serial_exit();
	
	uart_unregister_driver(&s3c_uart_drv);
}

module_init(s3c_serial_modinit);
module_exit(s3c_serial_modexit);

/* ----------------------------------------------------------------------------------*/
/*                                Console code for UART                              */
/*-----------------------------------------------------------------------------------*/

#ifdef CONFIG_SERIAL_S5PC1XX_CONSOLE

static struct uart_port *cons_uart;

static int
s3c_serial_console_txrdy(struct uart_port *port, unsigned int ufcon)
{
	struct s3c_uart_info *info = s3c_port_to_info(port);
	unsigned long ufstat, utrstat;

	if (ufcon & S3C_UFCON_FIFOMODE) {
		/* fifo mode - check ammount of data in fifo registers... */
		ufstat = rd_regl(port, S3C_UFSTAT);
		return (ufstat & info->tx_fifofull) ? 0 : 1;
	}

	/* in non-fifo mode, we go and use the tx buffer empty */
	utrstat = rd_regl(port, S3C_UTRSTAT);
	return (utrstat & S3C_UTRSTAT_TXE) ? 1 : 0;
}

static void
s3c_serial_console_write(struct console *co, const char *s,
			     unsigned int count)
{
	int i;
	unsigned int ufcon = rd_regl(cons_uart, S3C_UFCON);

	for (i = 0; i < count; i++) {
		while (!s3c_serial_console_txrdy(cons_uart, ufcon))
			barrier();

		wr_regb(cons_uart, S3C_UTXH, s[i]);

		if (s[i] == '\n') {
			while (!s3c_serial_console_txrdy(cons_uart, ufcon))
				barrier();

			wr_regb(cons_uart, S3C_UTXH, '\r');
		}
	}
}

static void __init
s3c_serial_get_options(struct uart_port *port, int *baud,
			   int *parity, int *bits)
{
	struct s3c24xx_uart_clksrc clksrc;
	struct clk *clk;
	unsigned int ulcon;
	unsigned int ucon;
	unsigned int ubrdiv;
	unsigned long rate;

	ulcon  = rd_regl(port, S3C_ULCON);
	ucon   = rd_regl(port, S3C_UCON);
	ubrdiv = rd_regl(port, S3C_UBRDIV);

	dbg("s3c_serial_get_options: port=%p\n"
	    "registers: ulcon=%08x, ucon=%08x, ubdriv=%08x\n",
	    port, ulcon, ucon, ubrdiv);

	if ((ucon & 0xf) != 0) {
		/* consider the serial port configured if the tx/rx mode set */
		switch (ulcon & S3C_LCON_CSMASK) {
		case S3C_LCON_CS5:
			*bits = 5;
			break;

		case S3C_LCON_CS6:
			*bits = 6;
			break;

		case S3C_LCON_CS7:
			*bits = 7;
			break;

		case S3C_LCON_CS8:
		default:
			*bits = 8;
			break;
		}

		switch (ulcon & S3C_LCON_PMASK) {
		case S3C_LCON_PEVEN:
			*parity = 'e';
			break;

		case S3C_LCON_PODD:
			*parity = 'o';
			break;

		case S3C_LCON_PNONE:
		default:
			*parity = 'n';
		}

		/* now calculate the baud rate */
		s3c_serial_getsource(port, &clksrc);

		clk = clk_get(port->dev, clksrc.name);
		if (!IS_ERR(clk) && clk != NULL)
			rate = clk_get_rate(clk) / clksrc.divisor;
		else
			rate = 1;


		*baud = rate / ( 16 * (ubrdiv + 1));
		dbg("calculated baud %d\n", *baud);
	}

}

/* s3c_serial_init_ports
 *
 * initialise the serial ports from the machine provided initialisation
 * data.
*/

static int s3c_serial_init_ports(struct s3c_uart_info *info)
{
	struct s3c_uart_port *ptr = s3c_serial_ports;
	struct platform_device **platdev_ptr;
	int i;

	dbg("s3c_serial_init_ports: initialising ports...\n");

	platdev_ptr = s3c24xx_uart_devs;

	for (i = 0; i < NR_PORTS; i++, ptr++, platdev_ptr++) {
		s3c_serial_init_port(ptr, info, *platdev_ptr);
	}

	return 0;
}

static int __init
s3c_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	dbg("s3c_serial_console_setup: co=%p (%d), %s\n",
	    co, co->index, options);

	/* is this a valid port */

	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;

	port = &s3c_serial_ports[co->index].port;

	/* is the port configured? */

	if (port->mapbase == 0x0) {
		co->index = 0;
		port = &s3c_serial_ports[co->index].port;
	}

	cons_uart = port;

	dbg("s3c_serial_console_setup: port=%p (%d)\n", port, co->index);

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		s3c_serial_get_options(port, &baud, &parity, &bits);

	dbg("s3c_serial_console_setup: baud %d\n", baud);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

/* s3c_serial_initconsole
 *
 * initialise the console from one of the uart drivers
 */

static struct console s3c_serial_console =
{
	.name		= S3C_SERIAL_NAME,
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= s3c_serial_console_write,
	.setup		= s3c_serial_console_setup
};

static int s3c_serial_initconsole(void)
{
	dbg("s3c_serial_initconsole\n");

	s3c_serial_console.data = &s3c_uart_drv;
	s3c_serial_init_ports(&s3c_uart_inf);

	register_console(&s3c_serial_console);
	return 0;
}

console_initcall(s3c_serial_initconsole);

#endif /* CONFIG_SERIAL_S3C_CONSOLE */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_DESCRIPTION("Samsung S3C Serial port driver");
