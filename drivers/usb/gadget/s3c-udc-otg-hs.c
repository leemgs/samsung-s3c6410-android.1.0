/*
 * drivers/usb/gadget/s3c-udc-otg-hs.c
 * Samsung S3C on-chip full/high speed USB OTG 2.0 device controllers
 *
 * $Id: s3c-udc-otg-hs.c,v 1.12 2008/09/28 01:33:52 dasan Exp $*
 *
 * Copyright (C) 2006 for Samsung Electronics
 *
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
 */

#include "s3c-udc.h"
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/arch/regs-s3c-clock.h>
#include <asm/arch/regs-usb-otg-hs.h>

/*
   Select USB OTG Device Slave or DMA mode
   0 : Slave Mode
   1 : DMA   Mode (only use for File-backed Storage Gadget)
*/
#define OTG_DMA_MODE	0

#undef DEBUG_S3C_UDC_SETUP
#undef DEBUG_S3C_UDC_EP0
#undef DEBUG_S3C_UDC_ISR
#undef DEBUG_S3C_UDC_OUT_EP
#undef DEBUG_S3C_UDC_IN_EP
#undef DEBUG_S3C_UDC

//#define DEBUG_S3C_UDC_SETUP
//#define DEBUG_S3C_UDC_EP0
//#define DEBUG_S3C_UDC_ISR
//#define DEBUG_S3C_UDC_OUT_EP
//#define DEBUG_S3C_UDC_IN_EP
//#define DEBUG_S3C_UDC

#define EP0_CON		0
#define EP1_OUT		1
#define EP2_IN		2
#define EP3_IN		3
#define EP_MASK		0xF

#if defined(DEBUG_S3C_UDC_ISR) || defined(DEBUG_S3C_UDC_EP0)
static char *state_names[] = {
	"WAIT_FOR_SETUP",
	"DATA_STATE_XMIT",
	"DATA_STATE_NEED_ZLP",
	"WAIT_FOR_OUT_STATUS",
	"DATA_STATE_RECV"
	};
#endif

#ifdef DEBUG_S3C_UDC_SETUP
#define DEBUG_SETUP(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG_SETUP(fmt,args...) do {} while(0)
#endif

#ifdef DEBUG_S3C_UDC_EP0
#define DEBUG_EP0(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG_EP0(fmt,args...) do {} while(0)
#endif

#ifdef DEBUG_S3C_UDC
#define DEBUG(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG(fmt,args...) do {} while(0)
#endif

#ifdef DEBUG_S3C_UDC_ISR
#define DEBUG_ISR(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG_ISR(fmt,args...) do {} while(0)
#endif

#ifdef DEBUG_S3C_UDC_OUT_EP
#define DEBUG_OUT_EP(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG_OUT_EP(fmt,args...) do {} while(0)
#endif

#ifdef DEBUG_S3C_UDC_IN_EP
#define DEBUG_IN_EP(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG_IN_EP(fmt,args...) do {} while(0)
#endif


#define	DRIVER_DESC		"Samsung Dual-speed USB 2.0 OTG Device Controller"
#define	DRIVER_VERSION		__DATE__

struct s3c_udc	*the_controller;

static const char driver_name[] = "s3c-udc";
static const char driver_desc[] = DRIVER_DESC;
static const char ep0name[] = "ep0-control";

#if OTG_DMA_MODE
#define GINTMSK_INIT	(INT_OUT_EP|INT_IN_EP|INT_RESUME|INT_ENUMDONE|INT_RESET|INT_SUSPEND)
#define DOEPMSK_INIT	(CTRL_OUT_EP_SETUP_PHASE_DONE|AHB_ERROR|TRANSFER_DONE)
#define DIEPMSK_INIT	(NON_ISO_IN_EP_TIMEOUT|AHB_ERROR|TRANSFER_DONE)
#define GAHBCFG_INIT	(PTXFE_HALF|NPTXFE_HALF|MODE_DMA|BURST_INCR4|GBL_INT_UNMASK)
#else
#define GINTMSK_INIT	(INT_RESUME|INT_ENUMDONE|INT_RESET|INT_SUSPEND|INT_RX_FIFO_NOT_EMPTY)
#define DOEPMSK_INIT	(AHB_ERROR)
#define DIEPMSK_INIT	(NON_ISO_IN_EP_TIMEOUT|AHB_ERROR)
#define GAHBCFG_INIT	(PTXFE_HALF|NPTXFE_HALF|MODE_SLAVE|BURST_SINGLE|GBL_INT_UNMASK)
#endif

u32 tx_ep_num = 2;

// Max packet size
static u32 ep0_fifo_size = 64;
static u32 ep_fifo_size =  512;
static u32 ep_fifo_size2 = 1024;

struct usb_ctrlrequest ctrl;
static int set_interface_first = 0;

/*
  Local declarations.
*/
static int s3c_ep_enable(struct usb_ep *ep,
			     const struct usb_endpoint_descriptor *);
static int s3c_ep_disable(struct usb_ep *ep);
static struct usb_request *s3c_alloc_request(struct usb_ep *ep, gfp_t gfp_flags);
static void s3c_free_request(struct usb_ep *ep, struct usb_request *);

static int s3c_queue(struct usb_ep *ep, struct usb_request *, gfp_t gfp_flags);
static int s3c_dequeue(struct usb_ep *ep, struct usb_request *);
static int s3c_set_halt(struct usb_ep *ep, int);
static int s3c_fifo_status(struct usb_ep *ep);
static void s3c_fifo_flush(struct usb_ep *ep);
static void s3c_ep0_read(struct s3c_udc *dev);
static void s3c_ep0_kick(struct s3c_udc *dev, struct s3c_ep *ep);
static void s3c_handle_ep0(struct s3c_udc *dev);
static int s3c_ep0_write(struct s3c_udc *dev);

static void done(struct s3c_ep *ep, struct s3c_request *req,
		 int status);
static void stop_activity(struct s3c_udc *dev,
			  struct usb_gadget_driver *driver);
static int udc_enable(struct s3c_udc *dev);
static void udc_set_address(struct s3c_udc *dev, unsigned char address);
static void reconfig_usbd(void);

/* extern declarations in arch/arm/mach-s3c64xx*/
#define OTGD_PHY_CLK_VALUE	(0x20)	/* UTMI Interface, Oscillator */
extern void otg_phy_init(u32 otg_phy_clk);
extern void otg_phy_off(void);

static struct usb_ep_ops s3c_ep_ops = {
	.enable = s3c_ep_enable,
	.disable = s3c_ep_disable,

	.alloc_request = s3c_alloc_request,
	.free_request = s3c_free_request,

	.queue = s3c_queue,
	.dequeue = s3c_dequeue,

	.set_halt = s3c_set_halt,
	.fifo_status = s3c_fifo_status,
	.fifo_flush = s3c_fifo_flush,
};

#ifdef CONFIG_USB_GADGET_DEBUG_FILES

static const char proc_node_name[] = "driver/udc";

static int
udc_proc_read(char *page, char **start, off_t off, int count,
	      int *eof, void *_dev)
{
	char *buf = page;
	struct s3c_udc *dev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	/* basic device status */
	t = scnprintf(next, size,
		      DRIVER_DESC "\n"
		      "%s version: %s\n"
		      "Gadget driver: %s\n"
		      "\n",
		      driver_name, DRIVER_VERSION,
		      dev->driver ? dev->driver->driver.name : "(none)");
	size -= t;
	next += t;

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

#define create_proc_files() \
	create_proc_read_entry(proc_node_name, 0, NULL, udc_proc_read, dev)
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else	/* !CONFIG_USB_GADGET_DEBUG_FILES */

#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

#endif	/* CONFIG_USB_GADGET_DEBUG_FILES */

/*
 * 	udc_disable - disable USB device controller
 */
static void udc_disable(struct s3c_udc *dev)
{
	DEBUG_SETUP("%s: %p\n", __FUNCTION__, dev);

	udc_set_address(dev, 0);

	dev->ep0state = WAIT_FOR_SETUP;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->usb_address = 0;

	otg_phy_off();
}

/*
 * 	udc_reinit - initialize software state
 */
static void udc_reinit(struct s3c_udc *dev)
{
	u32 i;

	DEBUG_SETUP("%s: %p\n", __FUNCTION__, dev);

	/* device/ep0 records init */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = WAIT_FOR_SETUP;

	/* basic endpoint records init */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->desc = 0;
		ep->stopped = 0;
		INIT_LIST_HEAD(&ep->queue);
		ep->pio_irqs = 0;
	}

	/* the rest was statically initialized, and is read-only */
}

#define BYTES2MAXP(x)	(x / 8)
#define MAXP2BYTES(x)	(x * 8)

/* until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static int udc_enable(struct s3c_udc *dev)
{
	DEBUG_SETUP("%s: %p\n", __FUNCTION__, dev);

	otg_phy_init(OTGD_PHY_CLK_VALUE);
	reconfig_usbd();

	DEBUG_SETUP("S3C USB 2.0 OTG Controller Core Initialized : 0x%x\n",
			readl(S3C_UDC_OTG_GINTMSK));

	dev->gadget.speed = USB_SPEED_UNKNOWN;

	return 0;
}

/*
  Register entry point for the peripheral controller driver.
*/
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct s3c_udc *dev = the_controller;
	int retval;

	DEBUG_SETUP("%s: %s\n", __FUNCTION__, driver->driver.name);

	if (!driver
	    || (driver->speed != USB_SPEED_FULL && driver->speed != USB_SPEED_HIGH)
	    || !driver->bind
	    || !driver->unbind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;
	retval = device_add(&dev->gadget.dev);

	if(retval) { /* TODO */
		printk("target device_add failed, error %d\n", retval);
		return retval;
	}

	retval = driver->bind(&dev->gadget);
	if (retval) {
		printk("%s: bind to driver %s --> error %d\n", dev->gadget.name,
		       driver->driver.name, retval);
		device_del(&dev->gadget.dev);

		dev->driver = 0;
		dev->gadget.dev.driver = 0;
		return retval;
	}

	enable_irq(IRQ_OTG);

	printk("Registered gadget driver '%s'\n", driver->driver.name);
	udc_enable(dev);

	return 0;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

/*
  Unregister entry point for the peripheral controller driver.
*/
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct s3c_udc *dev = the_controller;
	unsigned long flags;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);
	dev->driver = 0;
	stop_activity(dev, driver);
	spin_unlock_irqrestore(&dev->lock, flags);

	driver->unbind(&dev->gadget);
	device_del(&dev->gadget.dev);

	disable_irq(IRQ_OTG);

	printk("Unregistered gadget driver '%s'\n", driver->driver.name);

	udc_disable(dev);

	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

#if OTG_DMA_MODE /* DMA Mode */
static int setdma_rx(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 *buf, ctrl;
	u32 length, pktcnt;

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);

	length = req->req.length - req->req.actual;
	dma_cache_maint(buf, length, DMA_FROM_DEVICE);

	if(length == 0)
		pktcnt = 1;
	else
		pktcnt = (length - 1)/(ep->ep.maxpacket) + 1;

	writel(virt_to_phys(buf), S3C_UDC_OTG_DOEPDMA1);

	writel((pktcnt<<19)|(length<<0), S3C_UDC_OTG_DOEPTSIZ1);

	DEBUG_OUT_EP("%s: RX DMA start : DOEPDMA1 = 0x%x, DOEPTSIZ1 = 0x%x, "
		     "pktcnt = %d, xfersize = %d\n",
			__FUNCTION__,
			readl(S3C_UDC_OTG_DOEPDMA1),
			readl(S3C_UDC_OTG_DOEPTSIZ1),
			pktcnt, length);

	ctrl =  readl(S3C_UDC_OTG_DOEPCTL1);
	writel(DEPCTL_EPENA|DEPCTL_CNAK|ctrl, S3C_UDC_OTG_DOEPCTL1);

	return 0;

}

static int setdma_tx(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 *buf, ctrl;
	u32 length, pktcnt;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);

	length = req->req.length - req->req.actual;
	req->req.actual += length;

	dma_cache_maint(buf, length, DMA_TO_DEVICE);

	if(length == 0)
		pktcnt = 1;
	else
		pktcnt = (length - 1)/(ep->ep.maxpacket) + 1;

	if(ep_index(ep) == EP0_CON) {
		writel(virt_to_phys(buf), S3C_UDC_OTG_DIEPDMA0);
		writel((pktcnt<<19)|(length<<0), (u32) S3C_UDC_OTG_DIEPTSIZ0);

		DEBUG_EP0("%s:TX DMA start : DIEPDMA0 = 0x%x, DIEPTSIZ0 = 0x%x, "
			  "pktcnt = %d, xfersize = %d\n",
				__FUNCTION__,
				readl(S3C_UDC_OTG_DIEPDMA0),
				readl(S3C_UDC_OTG_DIEPTSIZ0),
				pktcnt, length);

		ctrl =  readl(S3C_UDC_OTG_DIEPCTL0);
		writel(DEPCTL_EPENA|DEPCTL_CNAK|(EP2_IN<<11)| ctrl, (u32) S3C_UDC_OTG_DIEPCTL0);

	} else if ((ep_index(ep) == EP2_IN)) {
		writel(virt_to_phys(buf), S3C_UDC_OTG_DIEPDMA2);
		writel((pktcnt<<19)|(length<<0), S3C_UDC_OTG_DIEPTSIZ2);

		DEBUG_IN_EP("%s:TX DMA start : DIEPDMA2 = 0x%x, DIEPTSIZ2 = 0x%x, "
			    "pktcnt = %d, xfersize = %d\n",
				__FUNCTION__,
				readl(S3C_UDC_OTG_DIEPDMA2),
				readl(S3C_UDC_OTG_DIEPTSIZ2),
				pktcnt, length);

		ctrl =  readl(S3C_UDC_OTG_DIEPCTL2);
		writel(DEPCTL_EPENA|DEPCTL_CNAK|(EP2_IN<<11)| ctrl, (u32) S3C_UDC_OTG_DIEPCTL2);

	} else if ((ep_index(ep) == EP3_IN)) {

#if 1
		if (set_interface_first == 1) {
			DEBUG_IN_EP("%s: first packet write skipped after set_interface\n", __FUNCTION__);
			set_interface_first = 0;

			writel(virt_to_phys(buf), S3C_UDC_OTG_DIEPDMA3);
			writel((1<<19)|(0<<0), S3C_UDC_OTG_DIEPTSIZ3);

			ctrl =  readl(S3C_UDC_OTG_DIEPCTL3);
			writel(DEPCTL_EPENA|DEPCTL_CNAK|(EP2_IN<<11)| ctrl, (u32) S3C_UDC_OTG_DIEPCTL3);

//			done(ep, req, 0);
			return length;
		}
#endif
		writel(virt_to_phys(buf), S3C_UDC_OTG_DIEPDMA3);
		writel((pktcnt<<19)|(length<<0), S3C_UDC_OTG_DIEPTSIZ3);

		DEBUG_IN_EP("%s:TX DMA start : DIEPDMA3 = 0x%x, DIEPTSIZ3 = 0x%x, "
			    "pktcnt = %d, xfersize = %d\n",
				__FUNCTION__,
				readl(S3C_UDC_OTG_DIEPDMA3),
				readl(S3C_UDC_OTG_DIEPTSIZ3),
				pktcnt, length);

		ctrl =  readl(S3C_UDC_OTG_DIEPCTL3);
		writel(DEPCTL_EPENA|DEPCTL_CNAK|(EP2_IN<<11)| ctrl, (u32) S3C_UDC_OTG_DIEPCTL3);

	} else {
		printk("%s: --> Error Unused Endpoint!!\n",
			__FUNCTION__);
		BUG();
	}

	return length;
}

static void complete_rx(struct s3c_udc *dev, u32 ep_idx)
{
	struct s3c_ep *ep = &dev->ep[ep_idx];
	struct s3c_request *req;
	u32 csr, count_bytes, xfer_length, is_short = 0;

	if (unlikely(!(ep->desc))) {
		/* Throw packet away.. */
		printk("%s: No descriptor?!?\n", __FUNCTION__);
		return;
	}

	if (list_empty(&ep->queue)) {
		DEBUG_OUT_EP("%s: NULL REQ on OUT EP-%d\n", __FUNCTION__, ep_idx);
		return;

	}

	req = list_entry(ep->queue.next,
			struct s3c_request, queue);

	csr = readl(S3C_UDC_OTG_DOEPTSIZ1);
	count_bytes = (csr & 0x7fff);

		dma_cache_maint(req->req.buf, req->req.length, DMA_FROM_DEVICE);
	xfer_length = req->req.length-count_bytes;
	req->req.actual += min(xfer_length, req->req.length);
	is_short = (xfer_length < ep->ep.maxpacket);

	DEBUG_OUT_EP("%s: RX DMA done : %d/%d bytes received%s,"
		  "DOEPTSIZ1 = 0x%x, %d bytes remained\n",
			__FUNCTION__, req->req.actual, req->req.length,
			is_short ? "/S" : "", csr, count_bytes);

	if (is_short || req->req.actual == req->req.length) {
		done(ep, req, 0);

		if(!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next, struct s3c_request, queue);
			DEBUG_OUT_EP("%s: Next Rx request start...\n", __FUNCTION__);
			setdma_rx(ep, req);
		}
	}
}

static void complete_tx(struct s3c_udc *dev, u32 ep_idx)
{
	struct s3c_ep *ep = &dev->ep[ep_idx];
	struct s3c_request *req;
	u32 count_bytes = 0;

	if (unlikely(!(ep->desc))) {
		/* Throw packet away.. */
		printk("%s: No descriptor?!?\n", __FUNCTION__);
		return;
	}

	if (list_empty(&ep->queue)) {
		DEBUG_IN_EP("%s: NULL REQ on IN EP-%d\n", __FUNCTION__, ep_idx);
		return;

	}

	req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (ep_idx == EP2_IN) {
		count_bytes = (readl(S3C_UDC_OTG_DIEPTSIZ2)) & 0x7fff;
		req->req.actual = req->req.length-count_bytes;

		DEBUG_IN_EP("%s: TX DMA done : %d/%d bytes sent, DIEPTSIZ2 = 0x%x\n",
				__FUNCTION__, req->req.actual,
				req->req.length,
				readl(S3C_UDC_OTG_DIEPTSIZ2));

	} else if (ep_idx == EP3_IN) {
		count_bytes = (readl(S3C_UDC_OTG_DIEPTSIZ3)) & 0x7fff;
		req->req.actual = req->req.length-count_bytes;

		DEBUG_IN_EP("%s: TX DMA done : %d/%d bytes sent, DIEPTSIZ3 = 0x%x\n",
				__FUNCTION__, req->req.actual,
				req->req.length,
				readl(S3C_UDC_OTG_DIEPTSIZ3));
	} else
		printk("%s: --> Error Unused Endpoint-%d!!\n", __FUNCTION__, ep_idx);

	if (req->req.actual == req->req.length) {
		done(ep, req, 0);

		if(!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next, struct s3c_request, queue);
			DEBUG_IN_EP("%s: Next Tx request start...\n", __FUNCTION__);
			setdma_tx(ep, req);
		}
	}
}

#else /* Slave Mode */

/*-------------------------------------------------------------------------*/

/** Read to request from FIFO (max read == bytes in fifo)
 *  Return:  0 = still running, 1 = completed, negative = errno
 */
static int read_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 csr, gintmsk;
	u32 *buf;
	u32 bufferspace, count, count_bytes, is_short = 0;
	u32 fifo = ep->fifo;

	csr = readl(S3C_UDC_OTG_GRXSTSP);
	count_bytes = (csr & 0x7ff0)>>4;

	gintmsk = readl(S3C_UDC_OTG_GINTMSK);

	if(!count_bytes) {
		DEBUG_OUT_EP("%s: count_bytes %d bytes\n", __FUNCTION__, count_bytes);

		// Unmask USB OTG 2.0 interrupt source : INT_RX_FIFO_NOT_EMPTY
		writel(gintmsk | INT_RX_FIFO_NOT_EMPTY, S3C_UDC_OTG_GINTMSK);
		return 0;
	}

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	count = count_bytes / 4;
	if(count_bytes%4) count = count + 1;

	req->req.actual += min(count_bytes, bufferspace);

	is_short = (count_bytes < ep->ep.maxpacket);
	DEBUG_OUT_EP("%s: read %s, %d bytes%s req %p %d/%d GRXSTSP:0x%x\n",
		__FUNCTION__,
		ep->ep.name, count_bytes,
		is_short ? "/S" : "", req, req->req.actual, req->req.length, csr);

	while (likely(count-- != 0)) {
		u32 byte = (u32) readl(fifo);

		if (unlikely(bufferspace == 0)) {
			/* this happens when the driver's buffer
		 	* is smaller than what the host sent.
		 	* discard the extra data.
		 	*/
			if (req->req.status != -EOVERFLOW)
				printk("%s overflow %d\n", ep->ep.name, count);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = byte;
			bufferspace-=4;
		}
 	 }

	// Unmask USB OTG 2.0 interrupt source : INT_RX_FIFO_NOT_EMPTY
	writel(gintmsk | INT_RX_FIFO_NOT_EMPTY, S3C_UDC_OTG_GINTMSK);

	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		done(ep, req, 0);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

/* Inline code */
static __inline__ int write_packet(struct s3c_ep *ep,
				   struct s3c_request *req, int max)
{
	u32 *buf;
	int length, count;
	u32 fifo = ep->fifo, in_ctrl;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);

	length = req->req.length - req->req.actual;
	length = min(length, max);
	req->req.actual += length;

	DEBUG("%s: Write %d (max %d), fifo=0x%x\n",
		__FUNCTION__, length, max, fifo);

	if(ep_index(ep) == EP0_CON) {
		writel((1<<19)|(length<<0), (u32) S3C_UDC_OTG_DIEPTSIZ0);

		in_ctrl =  readl(S3C_UDC_OTG_DIEPCTL0);
		writel(DEPCTL_EPENA|DEPCTL_CNAK|(EP2_IN<<11)| in_ctrl, (u32) S3C_UDC_OTG_DIEPCTL0);

		DEBUG_EP0("%s:(DIEPTSIZ0):0x%x, (DIEPCTL0):0x%x, (GNPTXSTS):0x%x\n", __FUNCTION__,
			readl(S3C_UDC_OTG_DIEPTSIZ0),readl(S3C_UDC_OTG_DIEPCTL0),
			readl(S3C_UDC_OTG_GNPTXSTS));

		udelay(20);

	} else if ((ep_index(ep) == EP2_IN)) {
		writel((1<<19)|(length<<0), S3C_UDC_OTG_DIEPTSIZ2);

		in_ctrl =  readl(S3C_UDC_OTG_DIEPCTL2);
		writel(DEPCTL_EPENA|DEPCTL_CNAK|(EP2_IN<<11)| in_ctrl, (u32) S3C_UDC_OTG_DIEPCTL2);

		DEBUG_IN_EP("%s:(DIEPTSIZ2):0x%x, (DIEPCTL2):0x%x, (GNPTXSTS):0x%x\n", __FUNCTION__,
			readl(S3C_UDC_OTG_DIEPTSIZ2),readl(S3C_UDC_OTG_DIEPCTL2),
			readl(S3C_UDC_OTG_GNPTXSTS));

		udelay(20);

	} else if ((ep_index(ep) == EP3_IN)) {

		if (set_interface_first == 1) {
			DEBUG_IN_EP("%s: first packet write skipped after set_interface\n", __FUNCTION__);
			set_interface_first = 0;
			return length;
		}

		writel((1<<19)|(length<<0), S3C_UDC_OTG_DIEPTSIZ3);

		in_ctrl =  readl(S3C_UDC_OTG_DIEPCTL3);
		writel(DEPCTL_EPENA|DEPCTL_CNAK|(EP2_IN<<11)| in_ctrl, (u32) S3C_UDC_OTG_DIEPCTL3);

		DEBUG_IN_EP("%s:(DIEPTSIZ3):0x%x, (DIEPCTL3):0x%x, (GNPTXSTS):0x%x\n", __FUNCTION__,
			readl(S3C_UDC_OTG_DIEPTSIZ3),readl(S3C_UDC_OTG_DIEPCTL3),
			readl(S3C_UDC_OTG_GNPTXSTS));

		udelay(20);

	} else {
		printk("%s: --> Error Unused Endpoint!!\n",
			__FUNCTION__);
		BUG();
	}

	for (count=0;count<length;count+=4) {
	  	writel(*buf++, fifo);
	}
	return length;
}

/** Write request to FIFO (max write == maxp size)
 *  Return:  0 = still running, 1 = completed, negative = errno
 */
static int write_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 max, gintmsk;
	unsigned count;
	int is_last = 0, is_short = 0;

	gintmsk = readl(S3C_UDC_OTG_GINTMSK);

	max = le16_to_cpu(ep->desc->wMaxPacketSize);
	count = write_packet(ep, req, max);

	/* last packet is usually short (or a zlp) */
	if (unlikely(count != max))
		is_last = is_short = 1;
	else {
		if (likely(req->req.length != req->req.actual)
		    || req->req.zero)
			is_last = 0;
		else
			is_last = 1;
		/* interrupt/iso maxpacket may not fill the fifo */
		is_short = unlikely(max < ep_maxpacket(ep));
	}

	DEBUG_IN_EP("%s: wrote %s %d bytes%s%s req %p %d/%d\n",
			__FUNCTION__,
      			ep->ep.name, count,
     	 		is_last ? "/L" : "", is_short ? "/S" : "",
      			req, req->req.actual, req->req.length);

	/* requests complete when all IN data is in the FIFO */
	if (is_last) {
		if(!ep_index(ep)){
			printk("%s: --> Error EP0 must not come here!\n",
				__FUNCTION__);
			BUG();
		}
		writel(gintmsk&(~INT_TX_FIFO_EMPTY), S3C_UDC_OTG_GINTMSK);
		done(ep, req, 0);
		return 1;
	}

	// Unmask USB OTG 2.0 interrupt source : INT_TX_FIFO_EMPTY
	writel(gintmsk | INT_TX_FIFO_EMPTY, S3C_UDC_OTG_GINTMSK);
	return 0;
}

/* ********************************************************************************************* */
/* Bulk OUT (recv)
 */

static void s3c_out_epn(struct s3c_udc *dev, u32 ep_idx)
{
	struct s3c_ep *ep = &dev->ep[ep_idx];
	struct s3c_request *req;

	if (unlikely(!(ep->desc))) {
		/* Throw packet away.. */
		printk("%s: No descriptor?!?\n", __FUNCTION__);
		return;
	}

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next,
				struct s3c_request, queue);

	if (unlikely(!req)) {
		DEBUG_OUT_EP("%s: NULL REQ on OUT EP-%d\n", __FUNCTION__, ep_idx);
		return;

	} else {
		read_fifo(ep, req);
	}

}

/**
 * s3c_in_epn - handle IN interrupt
 */
static void s3c_in_epn(struct s3c_udc *dev, u32 ep_idx)
{
	struct s3c_ep *ep = &dev->ep[ep_idx];
	struct s3c_request *req;

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (unlikely(!req)) {
		DEBUG_IN_EP("%s: NULL REQ on IN EP-%d\n", __FUNCTION__, ep_idx);
		return;
	}
	else {
		write_fifo(ep, req);
	}

}
#endif

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct s3c_ep *ep, struct s3c_request *req, int status)
{
	unsigned int stopped = ep->stopped;

	DEBUG("%s: %s %p, req = %p, stopped = %d\n",
		__FUNCTION__, ep->ep.name, ep, &req->req, stopped);
	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN)
		DEBUG("complete %s req %p stat %d len %u/%u\n",
			ep->ep.name, &req->req, status,
			req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;

	spin_unlock(&ep->dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);

	ep->stopped = stopped;
}

/*
 * 	nuke - dequeue ALL requests
 */
void nuke(struct s3c_ep *ep, int status)
{
	struct s3c_request *req;

	DEBUG("%s: %s %p\n", __FUNCTION__, ep->ep.name, ep);

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct s3c_request, queue);
		done(ep, req, status);
	}
}

static void stop_activity(struct s3c_udc *dev,
			  struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c_ep *ep = &dev->ep[i];
		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock(&dev->lock);
		driver->disconnect(&dev->gadget);
		spin_lock(&dev->lock);
	}

	/* re-init driver-visible data structures */
	udc_reinit(dev);
}

static void reconfig_usbd(void)
{
	// 2. Soft-reset OTG Core and then unreset again.
	u32 uTemp = writel(CORE_SOFT_RESET, S3C_UDC_OTG_GRSTCTL);

	writel(	0<<15		// PHY Low Power Clock sel
		|1<<14		// Non-Periodic TxFIFO Rewind Enable
		|0x5<<10	// Turnaround time
		|0<<9|0<<8	// [0:HNP disable, 1:HNP enable][ 0:SRP disable, 1:SRP enable] H1= 1,1
		|0<<7		// Ulpi DDR sel
		|0<<6		// 0: high speed utmi+, 1: full speed serial
		|0<<4		// 0: utmi+, 1:ulpi
		|1<<3		// phy i/f  0:8bit, 1:16bit
		|0x7<<0,	// HS/FS Timeout*
		S3C_UDC_OTG_GUSBCFG);

	// 3. Put the OTG device core in the disconnected state.
	uTemp = readl(S3C_UDC_OTG_DCTL);
	uTemp |= SOFT_DISCONNECT;
	writel(uTemp, S3C_UDC_OTG_DCTL);

	udelay(20);

	// 4. Make the OTG device core exit from the disconnected state.
	uTemp = readl(S3C_UDC_OTG_DCTL);
	uTemp = uTemp & ~SOFT_DISCONNECT;
	writel(uTemp, S3C_UDC_OTG_DCTL);

	// 5. Configure OTG Core to initial settings of device mode.
	writel(1<<18|0x0<<0, S3C_UDC_OTG_DCFG);		// [][1: full speed(30Mhz) 0:high speed]

	mdelay(1);

	// 6. Unmask the core interrupts
	writel(GINTMSK_INIT, S3C_UDC_OTG_GINTMSK);

	// 7. Set NAK bit of EP0, EP1, EP2
	writel(DEPCTL_EPDIS|DEPCTL_SNAK|(0<<0), S3C_UDC_OTG_DOEPCTL0); /* EP0: Control OUT */
	writel(DEPCTL_EPDIS|DEPCTL_SNAK|(0<<0), S3C_UDC_OTG_DIEPCTL0); /* EP0: Control IN */

	writel(DEPCTL_EPDIS|DEPCTL_SNAK|DEPCTL_BULK_TYPE|(0<<0), S3C_UDC_OTG_DOEPCTL1); /* EP1:Data OUT */
	writel(DEPCTL_EPDIS|DEPCTL_SNAK|DEPCTL_BULK_TYPE|(0<<0), S3C_UDC_OTG_DIEPCTL2); /* EP2:Data IN */
	writel(DEPCTL_EPDIS|DEPCTL_SNAK|DEPCTL_BULK_TYPE|(0<<0), S3C_UDC_OTG_DIEPCTL3); /* EP3:IN Interrupt*/

	// 8. Unmask EP interrupts on IN EPs : 0, 2, 3
	//        		      OUT EPs : 0, 1
	writel( (((1<<EP1_OUT)|(1<<EP0_CON))<<16) |
		(1<<EP3_IN)|(1<<EP2_IN)|(1<<EP0_CON),
		S3C_UDC_OTG_DAINTMSK);

	// 9. Unmask device OUT EP common interrupts
	writel(DOEPMSK_INIT, S3C_UDC_OTG_DOEPMSK);

	// 10. Unmask device IN EP common interrupts
	writel(DIEPMSK_INIT, S3C_UDC_OTG_DIEPMSK);

	// 11. Set Rx FIFO Size
	writel(RX_FIFO_SIZE, S3C_UDC_OTG_GRXFSIZ);

	// 12. Set Non Periodic Tx FIFO Size
	writel(NPTX_FIFO_SIZE<<16| NPTX_FIFO_START_ADDR<<0, S3C_UDC_OTG_GNPTXFSIZ);

	// 13. Clear NAK bit of EP0, EP1, EP2
	// For Slave mode
	writel(DEPCTL_EPDIS|DEPCTL_CNAK|(0<<0), S3C_UDC_OTG_DOEPCTL0); /* EP0: Control OUT */

	// 14. Initialize OTG Link Core.
	writel(GAHBCFG_INIT, S3C_UDC_OTG_GAHBCFG);

}

void set_max_pktsize(struct s3c_udc *dev, enum usb_device_speed speed)
{
	u32 ep_ctrl;

	if (speed == USB_SPEED_HIGH) {
		ep0_fifo_size = 64;
		ep_fifo_size = 512;
		ep_fifo_size2 = 1024;
		dev->gadget.speed = USB_SPEED_HIGH;
	} else {
		ep0_fifo_size = 64;
		ep_fifo_size = 64;
		ep_fifo_size2 = 64;
		dev->gadget.speed = USB_SPEED_FULL;
	}

	dev->ep[0].ep.maxpacket = ep0_fifo_size;
	dev->ep[1].ep.maxpacket = ep_fifo_size;
	dev->ep[2].ep.maxpacket = ep_fifo_size;
	dev->ep[3].ep.maxpacket = ep_fifo_size;
	dev->ep[4].ep.maxpacket = ep_fifo_size;
	dev->ep[5].ep.maxpacket = ep_fifo_size2;
	dev->ep[6].ep.maxpacket = ep_fifo_size2;
	dev->ep[7].ep.maxpacket = ep_fifo_size2;
	dev->ep[8].ep.maxpacket = ep_fifo_size2;


	if (speed == USB_SPEED_HIGH) {
		// EP0 - Control IN (64 bytes)
		ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL0);
		writel(ep_ctrl|(0<<0), (u32) S3C_UDC_OTG_DIEPCTL0);

		// EP0 - Control OUT (64 bytes)
		ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL0);
		writel(ep_ctrl|(0<<0), (u32) S3C_UDC_OTG_DOEPCTL0);
	} else {
		// EP0 - Control IN (8 bytes)
		ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL0);
		writel(ep_ctrl|(3<<0), (u32) S3C_UDC_OTG_DIEPCTL0);

		// EP0 - Control OUT (8 bytes)
		ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL0);
		writel(ep_ctrl|(3<<0), (u32) S3C_UDC_OTG_DOEPCTL0);
	}


	// EP1 - Bulk Data OUT (512 bytes)
	ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL1);
	writel(ep_ctrl|(ep_fifo_size<<0), (u32) S3C_UDC_OTG_DOEPCTL1);

	// EP2 - Bulk Data IN (512 bytes)
	ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL2);
	writel(ep_ctrl|(ep_fifo_size<<0), (u32) S3C_UDC_OTG_DIEPCTL2);

	// EP3 - INTR Data IN (512 bytes)
	ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL3);
	writel(ep_ctrl|(ep_fifo_size<<0), (u32) S3C_UDC_OTG_DIEPCTL3);

}

static int reset_available = 1;
/*
 *	elfin usb client interrupt handler.
 */
static irqreturn_t s3c_udc_irq(int irq, void *_dev)
{
	struct s3c_udc *dev = _dev;
	u32 intr_status;
	u32 usb_status, ep_ctrl, gintmsk;

	spin_lock(&dev->lock);

	intr_status = readl(S3C_UDC_OTG_GINTSTS);
	gintmsk = readl(S3C_UDC_OTG_GINTMSK);

	DEBUG_ISR("\n**** %s : GINTSTS=0x%x(on state %s), GINTMSK : 0x%x\n",
			__FUNCTION__, intr_status, state_names[dev->ep0state], gintmsk);

	if (!intr_status) {
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (intr_status & INT_ENUMDONE) {
		DEBUG_SETUP("####################################\n");
		DEBUG_SETUP("    %s: Speed Detection interrupt\n",
				__FUNCTION__);
		writel(INT_ENUMDONE, S3C_UDC_OTG_GINTSTS);

		usb_status = (readl(S3C_UDC_OTG_DSTS) & 0x6);

		if (usb_status & (USB_FULL_30_60MHZ | USB_FULL_48MHZ)) {
			DEBUG_SETUP("    %s: Full Speed Detection\n",__FUNCTION__);
			set_max_pktsize(dev, USB_SPEED_FULL);

		} else {
			DEBUG_SETUP("    %s: High Speed Detection : 0x%x\n", __FUNCTION__, usb_status);
			set_max_pktsize(dev, USB_SPEED_HIGH);
		}
	}

	if (intr_status & INT_EARLY_SUSPEND) {
		DEBUG_SETUP("####################################\n");
		DEBUG_SETUP("    %s:Early suspend interrupt\n", __FUNCTION__);
		writel(INT_EARLY_SUSPEND, S3C_UDC_OTG_GINTSTS);
	}

	if (intr_status & INT_SUSPEND) {
		usb_status = readl(S3C_UDC_OTG_DSTS);
		DEBUG_SETUP("####################################\n");
		DEBUG_SETUP("    %s:Suspend interrupt :(DSTS):0x%x\n", __FUNCTION__, usb_status);
		writel(INT_SUSPEND, S3C_UDC_OTG_GINTSTS);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
		    && dev->driver
		    && dev->driver->suspend) {
			dev->driver->suspend(&dev->gadget);
		}
	}

	if (intr_status & INT_RESUME) {
		DEBUG_SETUP("####################################\n");
		DEBUG_SETUP("    %s: Resume interrupt\n", __FUNCTION__);
		writel(INT_RESUME, S3C_UDC_OTG_GINTSTS);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
		    && dev->driver
		    && dev->driver->resume) {
			dev->driver->resume(&dev->gadget);
		}
	}

	if (intr_status & INT_RESET) {
		usb_status = readl(S3C_UDC_OTG_GOTGCTL);
		DEBUG_SETUP("####################################\n");
		DEBUG_SETUP("    %s: Reset interrupt - (GOTGCTL):0x%x\n", __FUNCTION__, usb_status);
		writel(INT_RESET, S3C_UDC_OTG_GINTSTS);

		if((usb_status & 0xc0000) == (0x3 << 18)) {
			if(reset_available) {
				DEBUG_SETUP("     ===> OTG core got reset (%d)!! \n", reset_available);
				reconfig_usbd();
				dev->ep0state = WAIT_FOR_SETUP;
				reset_available = 0;
#if OTG_DMA_MODE
				writel((1 << 19)|sizeof(struct usb_ctrlrequest), S3C_UDC_OTG_DOEPTSIZ0);
				writel(virt_to_phys(&ctrl), S3C_UDC_OTG_DOEPDMA0);
				writel(DEPCTL_EPENA |DEPCTL_CNAK, S3C_UDC_OTG_DOEPCTL0);
#endif
			}
		} else {
			reset_available = 1;
			DEBUG_SETUP("      RESET handling skipped : reset_available : %d\n", reset_available);
		}
	}

#if OTG_DMA_MODE
	if ((intr_status & INT_IN_EP) || (intr_status & INT_OUT_EP)) {
		u32 ep_int, ep_int_status;

		gintmsk &= ~(INT_IN_EP|INT_OUT_EP);		// interrupt mask
		writel(gintmsk, S3C_UDC_OTG_GINTMSK);

		ep_int = readl(S3C_UDC_OTG_DAINT);
		DEBUG_ISR("\tDAINT : 0x%x \n", ep_int);

		if (ep_int & (1<<EP0_CON)) {		/* CONTROL IN endpont */

			ep_int_status = readl(S3C_UDC_OTG_DIEPINT0);
			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL0);
			DEBUG_EP0("\tEP0-IN : DIEPINT0 = 0x%x, DIEPCTL0 = 0x%x \n", ep_int_status, ep_ctrl);

			if (ep_int_status & TRANSFER_DONE) {
				DEBUG_EP0("\tEP0-IN transaction completed - (TX DMA done)\n");
				writel((1 << 19)|sizeof(struct usb_ctrlrequest), S3C_UDC_OTG_DOEPTSIZ0);
				writel(virt_to_phys(&ctrl), S3C_UDC_OTG_DOEPDMA0);
				writel(ep_ctrl|DEPCTL_EPENA|DEPCTL_CNAK, S3C_UDC_OTG_DOEPCTL0);
			}

			writel(ep_int_status, S3C_UDC_OTG_DIEPINT0); 		// Interrupt Clear

		} else if (ep_int & ((1<<EP0_CON)<<16)) {	/* CONTROL OUT endpont */

			ep_int_status = readl(S3C_UDC_OTG_DOEPINT0);
			ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL0);
			DEBUG_EP0("\tEP0-OUT : DOEPINT0 = 0x%x, DOEPCTL0 = 0x%x\n", ep_int_status, ep_ctrl);

			 if (ep_int_status & CTRL_OUT_EP_SETUP_PHASE_DONE) {

				DEBUG_EP0("\tSETUP packet(transaction) arrived\n");
				s3c_handle_ep0(dev);
			} else if (ep_int_status & TRANSFER_DONE) {

				DEBUG_EP0("\tEP0-OUT transaction completed - (RX DMA done)\n");
				writel((1 << 19)|sizeof(struct usb_ctrlrequest), S3C_UDC_OTG_DOEPTSIZ0);
				writel(virt_to_phys(&ctrl), S3C_UDC_OTG_DOEPDMA0);
				writel(ep_ctrl|DEPCTL_EPENA|DEPCTL_CNAK, S3C_UDC_OTG_DOEPCTL0);	// ep0 OUT enable, clear nak
			}

			writel(ep_int_status, S3C_UDC_OTG_DOEPINT0);	// Interrupt Clear

		} else if (ep_int & ((1<<EP1_OUT)<<16)) {		/* BULK OUT endpont */
			ep_int_status = readl(S3C_UDC_OTG_DOEPINT1);
			ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL1);

			DEBUG_OUT_EP("\tEP1-OUT : DOEPINT1 = 0x%x, DOEPCTL1 = 0x%x\n", ep_int_status, ep_ctrl);

			if (ep_int_status & TRANSFER_DONE) {
				DEBUG_OUT_EP("\tBULK OUT packet(transaction) arrived - (RX DMA done)\n");
				complete_rx(dev, EP1_OUT);
			}
			writel(ep_int_status, S3C_UDC_OTG_DOEPINT1); 		// ep1 Interrupt Clear

		} else if(ep_int & (1<<EP2_IN)) {			/* BULK IN endpont */
			ep_int_status = readl(S3C_UDC_OTG_DIEPINT2);
			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL2);

			DEBUG_IN_EP("\tEP2-IN : DIEPINT2 = 0x%x, DIEPCTL2 = 0x%x\n", ep_int_status, ep_ctrl);

			if (ep_int_status & TRANSFER_DONE) {
				DEBUG_IN_EP("\tBULK IN transaction completed - (TX DMA done)\n");
				complete_tx(dev, EP2_IN);
			}
			writel(ep_int_status, S3C_UDC_OTG_DIEPINT2); 		// ep2 Interrupt Clear

		} else if(ep_int & (1<<EP3_IN)) {			/* BULK IN endpont */
			ep_int_status = readl(S3C_UDC_OTG_DIEPINT3);
			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL3);

			DEBUG_IN_EP("\tEP3-IN : DIEPINT3 = 0x%x, DIEPCTL3 = 0x%x\n", ep_int_status, ep_ctrl);

			if (ep_int_status & TRANSFER_DONE) {
				DEBUG_IN_EP("\tBULK IN transaction completed - (TX DMA done)\n");
				complete_tx(dev, EP3_IN);
			}
			writel(ep_int_status, S3C_UDC_OTG_DIEPINT3); 		// ep2 Interrupt Clear
		}

		gintmsk |= (INT_IN_EP|INT_OUT_EP);
		writel(gintmsk, S3C_UDC_OTG_GINTMSK);

	}
#else
	if (intr_status & INT_RX_FIFO_NOT_EMPTY) {
		u32 grx_status, packet_status, ep_num, fifoCntByte = 0;

		// Mask USB OTG 2.0 interrupt source : INT_RX_FIFO_NOT_EMPTY
		gintmsk &= ~INT_RX_FIFO_NOT_EMPTY;
		writel(gintmsk, S3C_UDC_OTG_GINTMSK);

		grx_status = readl(S3C_UDC_OTG_GRXSTSR);
		DEBUG_ISR("    INT_RX_FIFO_NOT_EMPTY(GRXSTSR):0x%x, GINTMSK:0x%x\n", grx_status, gintmsk);

		packet_status = grx_status & 0x1E0000;
		fifoCntByte = (grx_status & 0x7ff0)>>4;
		ep_num = grx_status & EP_MASK;

		if (fifoCntByte) {

			if (packet_status == SETUP_PKT_RECEIVED)  {
				DEBUG_EP0("      => A SETUP data packet received : %d bytes\n", fifoCntByte);
				s3c_handle_ep0(dev);

				// Unmask USB OTG 2.0 interrupt source : INT_RX_FIFO_NOT_EMPTY
				gintmsk |= INT_RX_FIFO_NOT_EMPTY;

			} else if (packet_status == OUT_PKT_RECEIVED) {

				if(ep_num == EP1_OUT) {
					ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL1);
					DEBUG_ISR("      => A Bulk OUT data packet received : %d bytes, (DOEPCTL1):0x%x\n",
						fifoCntByte, ep_ctrl);
					s3c_out_epn(dev, 1);
					gintmsk = readl(S3C_UDC_OTG_GINTMSK);
					writel(ep_ctrl | DEPCTL_CNAK, S3C_UDC_OTG_DOEPCTL1);
				} else if (ep_num == EP0_CON) {
					ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL0);
					DEBUG_EP0("      => A CONTROL OUT data packet received : %d bytes, (DOEPCTL0):0x%x\n",
						fifoCntByte, ep_ctrl);
					dev->ep0state = DATA_STATE_RECV;
					s3c_ep0_read(dev);
					gintmsk |= INT_RX_FIFO_NOT_EMPTY;
				} else {
					DEBUG_ISR("      => Unused EP: %d bytes, (GRXSTSR):0x%x\n", fifoCntByte, grx_status);
				}
			} else {
				grx_status = readl(S3C_UDC_OTG_GRXSTSP);

				// Unmask USB OTG 2.0 interrupt source : INT_RX_FIFO_NOT_EMPTY
				gintmsk |= INT_RX_FIFO_NOT_EMPTY;

				DEBUG_ISR("      => A reserved packet received : %d bytes\n", fifoCntByte);
			}
		} else {
			if (dev->ep0state == DATA_STATE_XMIT) {
				ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL0);
				DEBUG_EP0("      => Write ep0 continue... (DOEPCTL0):0x%x\n", ep_ctrl);
				s3c_ep0_write(dev);
			}

			if (packet_status == SETUP_TRANSACTION_COMPLETED) {
				ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL0);
				DEBUG_EP0("      => A SETUP transaction completed (DOEPCTL0):0x%x\n", ep_ctrl);
				writel(ep_ctrl | DEPCTL_CNAK, S3C_UDC_OTG_DOEPCTL0);

			} else if (packet_status == OUT_TRANSFER_COMPLELTED) {
				if (ep_num == EP1_OUT) {
					ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL1);
					DEBUG_ISR("      => An OUT transaction completed (DOEPCTL1):0x%x\n", ep_ctrl);
					writel(ep_ctrl | DEPCTL_CNAK, S3C_UDC_OTG_DOEPCTL1);
				} else if (ep_num == EP0_CON) {
					ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL0);
					DEBUG_ISR("      => An OUT transaction completed (DOEPCTL0):0x%x\n", ep_ctrl);
					writel(ep_ctrl | DEPCTL_CNAK, S3C_UDC_OTG_DOEPCTL0);
				} else {
					DEBUG_ISR("      => Unused EP: %d bytes, (GRXSTSR):0x%x\n", fifoCntByte, grx_status);
				}
			} else if (packet_status == OUT_PKT_RECEIVED) {
				DEBUG_ISR("      => A  OUT PACKET RECEIVED (NO FIFO CNT BYTE)...(GRXSTSR):0x%x\n", grx_status);
			} else {
				DEBUG_ISR("      => A RESERVED PACKET RECEIVED (NO FIFO CNT BYTE)...(GRXSTSR):0x%x\n", grx_status);
			}

			grx_status = readl(S3C_UDC_OTG_GRXSTSP);

			// Unmask USB OTG 2.0 interrupt source : INT_RX_FIFO_NOT_EMPTY
			gintmsk |= INT_RX_FIFO_NOT_EMPTY;

		}

		// Un/Mask USB OTG 2.0 interrupt sources
		writel(gintmsk, S3C_UDC_OTG_GINTMSK);

		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}


	if (intr_status & INT_TX_FIFO_EMPTY) {
		DEBUG_ISR("    INT_TX_FIFO_EMPTY (GNPTXSTS):0x%x, (GINTMSK):0x%x, ep_num=%d\n",
				readl(S3C_UDC_OTG_GNPTXSTS),
				readl(S3C_UDC_OTG_GINTMSK),
				tx_ep_num);

		s3c_in_epn(dev, tx_ep_num);
	}
#endif
	spin_unlock(&dev->lock);

	return IRQ_HANDLED;
}

static int s3c_ep_enable(struct usb_ep *_ep,
			     const struct usb_endpoint_descriptor *desc)
{
	struct s3c_ep *ep;
	struct s3c_udc *dev;
	unsigned long flags;

	DEBUG("%s: %p\n", __FUNCTION__, _ep);

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || ep->bEndpointAddress != desc->bEndpointAddress
	    || ep_maxpacket(ep) < le16_to_cpu(desc->wMaxPacketSize)) {
		DEBUG("%s: bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
	    && ep->bmAttributes != USB_ENDPOINT_XFER_BULK
	    && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		DEBUG("%s: %s type mismatch\n", __FUNCTION__, _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
	     && le16_to_cpu(desc->wMaxPacketSize) != ep_maxpacket(ep))
	    || !desc->wMaxPacketSize) {
		DEBUG("%s: bad %s maxpacket\n", __FUNCTION__, _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DEBUG("%s: bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	ep->stopped = 0;
	ep->desc = desc;
	ep->pio_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);

	/* Reset halt state */
	s3c_set_halt(_ep, 0);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DEBUG("%s: enabled %s, stopped = %d, maxpacket = %d\n",
		__FUNCTION__, _ep->name, ep->stopped, ep->ep.maxpacket);
	return 0;
}

/** Disable EP
 */
static int s3c_ep_disable(struct usb_ep *_ep)
{
	struct s3c_ep *ep;
	unsigned long flags;

	DEBUG("%s: %p\n", __FUNCTION__, _ep);

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || !ep->desc) {
		DEBUG("%s: %s not enabled\n", __FUNCTION__,
		      _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* Nuke all pending requests */
	nuke(ep, -ESHUTDOWN);

	ep->desc = 0;
	ep->stopped = 1;

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DEBUG("%s: disabled %s\n", __FUNCTION__, _ep->name);
	return 0;
}

static struct usb_request *s3c_alloc_request(struct usb_ep *ep,
						 gfp_t gfp_flags)
{
	struct s3c_request *req;

	DEBUG("%s: %s %p\n", __FUNCTION__, ep->name, ep);

	req = kmalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void s3c_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct s3c_request *req;

	DEBUG("%s: %p\n", __FUNCTION__, ep);

	req = container_of(_req, struct s3c_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/** Queue one request
 *  Kickstart transfer if needed
 */
static int s3c_queue(struct usb_ep *_ep, struct usb_request *_req,
			 gfp_t gfp_flags)
{
	struct s3c_request *req;
	struct s3c_ep *ep;
	struct s3c_udc *dev;
	unsigned long flags;

	req = container_of(_req, struct s3c_request, req);
	if (unlikely(!_req || !_req->complete || !_req->buf
			|| !list_empty(&req->queue)))
	{
		DEBUG("%s: bad params\n", __FUNCTION__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct s3c_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DEBUG("%s: bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		DEBUG("%s: bogus device state %p\n", __FUNCTION__, dev->driver);
		return -ESHUTDOWN;
	}

	DEBUG("\n%s: %s queue req %p, len %d buf %p\n",
		__FUNCTION__, _ep->name, _req, _req->length, _req->buf);

	spin_lock_irqsave(&dev->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* kickstart this i/o queue? */
	DEBUG("%s: Add to ep=%d, Q empty=%d, stopped=%d\n",
		__FUNCTION__, ep_index(ep), list_empty(&ep->queue), ep->stopped);

	if (list_empty(&ep->queue) && likely(!ep->stopped)) {
		u32 csr;

		if (unlikely(ep_index(ep) == 0)) {
			/* EP0 */
			list_add_tail(&req->queue, &ep->queue);
			s3c_ep0_kick(dev, ep);
			req = 0;

		} else if (ep_is_in(ep)) {
			csr = readl((u32) S3C_UDC_OTG_GINTSTS);
			DEBUG_IN_EP("%s: ep_is_in, S3C_UDC_OTG_GINTSTS=0x%x\n",
				__FUNCTION__, csr);

#if OTG_DMA_MODE
			setdma_tx(ep, req);
			tx_ep_num = ep_index(ep);
#else
			if((csr & INT_TX_FIFO_EMPTY) &&
			   (write_fifo(ep, req) == 1)) {
				req = 0;
			} else {
				DEBUG("++++ IN-list_add_taill::req=%p, ep=%d\n",
					req, ep_index(ep));
				tx_ep_num = ep_index(ep);
			}
#endif
		} else {
			csr = readl((u32) S3C_UDC_OTG_GINTSTS);
			DEBUG_OUT_EP("%s: ep_is_out, S3C_UDC_OTG_GINTSTS=0x%x\n",
				__FUNCTION__, csr);

#if OTG_DMA_MODE
			setdma_rx(ep, req);
#else
			if((csr & INT_RX_FIFO_NOT_EMPTY) &&
			   (read_fifo(ep, req) == 1))
				req = 0;
			else
				DEBUG("++++ OUT-list_add_taill::req=%p, DOEPCTL1:0x%x\n",
					req, readl(S3C_UDC_OTG_DOEPCTL1));
#endif
		}
	}

	/* pio or dma irq handler advances the queue. */
	if (likely(req != 0))
		list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

/* dequeue JUST ONE request */
static int s3c_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct s3c_ep *ep;
	struct s3c_request *req;
	unsigned long flags;

	DEBUG("%s: %p\n", __FUNCTION__, _ep);

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

/** Halt specific EP
 *  Return 0 if success
 */
static int s3c_set_halt(struct usb_ep *_ep, int value)
{
	return 0;
}

/** Return bytes in EP FIFO
 */
static int s3c_fifo_status(struct usb_ep *_ep)
{
	int count = 0;
	struct s3c_ep *ep;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep) {
		DEBUG("%s: bad ep\n", __FUNCTION__);
		return -ENODEV;
	}

	DEBUG("%s: %d\n", __FUNCTION__, ep_index(ep));

	/* LPD can't report unclaimed bytes from IN fifos */
	if (ep_is_in(ep))
		return -EOPNOTSUPP;

	return count;
}

/** Flush EP FIFO
 */
static void s3c_fifo_flush(struct usb_ep *_ep)
{
	struct s3c_ep *ep;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DEBUG("%s: bad ep\n", __FUNCTION__);
		return;
	}

	DEBUG("%s: %d\n", __FUNCTION__, ep_index(ep));
}

/****************************************************************/
/* End Point 0 related functions                                */
/****************************************************************/

/* return:  0 = still running, 1 = completed, negative = errno */
static int write_fifo_ep0(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 max;
	unsigned count;
	int is_last;

	max = ep_maxpacket(ep);

	DEBUG_EP0("%s: max = %d\n", __FUNCTION__, max);

#if OTG_DMA_MODE
	count = setdma_tx(ep, req);
#else
	count = write_packet(ep, req, max);
#endif
	/* last packet is usually short (or a zlp) */
	if (likely(count != max))
		is_last = 1;
	else {
		if (likely(req->req.length != req->req.actual) || req->req.zero)
			is_last = 0;
		else
			is_last = 1;
	}

	DEBUG_EP0("%s: wrote %s %d bytes%s %d left %p\n", __FUNCTION__,
		  ep->ep.name, count,
		  is_last ? "/L" : "", req->req.length - req->req.actual, req);

	/* requests complete when all IN data is in the FIFO */
	if (is_last) {
		return 1;
	}

	return 0;
}

static __inline__ int s3c_fifo_read(struct s3c_ep *ep, u32 *cp, int max)
{
	int bytes;

#if OTG_DMA_MODE
	bytes = sizeof(struct usb_ctrlrequest);
	dma_cache_maint(&ctrl, bytes, DMA_FROM_DEVICE);
	DEBUG_EP0("%s: bytes=%d, ep_index=%d \n", __FUNCTION__, bytes, ep_index(ep));
#else
	int count;
	u32 grx_status = readl(S3C_UDC_OTG_GRXSTSP);
	bytes = (grx_status & 0x7ff0)>>4;

	DEBUG_EP0("%s: GRXSTSP=0x%x, bytes=%d, ep_index=%d, fifo=0x%x\n",
			__FUNCTION__, grx_status, bytes, ep_index(ep), ep->fifo);

	// 32 bits interface
	count = bytes / 4;

	while (count--) {
		*cp++ = (u32) readl(S3C_UDC_OTG_EP0_FIFO);
	}
#endif
	return bytes;
}

static int read_fifo_ep0(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 csr;
	u32 *buf;
	unsigned bufferspace, count, is_short, bytes;
	u32 fifo = ep->fifo;

	DEBUG_EP0("%s\n", __FUNCTION__);

	csr = readl(S3C_UDC_OTG_GRXSTSP);
	bytes = (csr & 0x7ff0)>>4;

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	/* read all bytes from this packet */
	if (likely((csr & EP_MASK) == EP0_CON)) {
		count = bytes / 4;
		req->req.actual += min(bytes, bufferspace);

	} else	{		// zlp
		count = 0;
		bytes = 0;
	}

	is_short = (bytes < ep->ep.maxpacket);
	DEBUG_EP0("%s: read %s %02x, %d bytes%s req %p %d/%d\n",
		  __FUNCTION__,
		  ep->ep.name, csr, bytes,
		  is_short ? "/S" : "", req, req->req.actual, req->req.length);

	while (likely(count-- != 0)) {
		u32 byte = (u32) readl(fifo);

		if (unlikely(bufferspace == 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				DEBUG_EP0("%s overflow %d\n", ep->ep.name,
					  count);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = byte;
			bufferspace = bufferspace - 4;
		}
	}

	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		return 1;
	}

	return 0;
}

/**
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function
 * after it decodes a set address setup packet.
 */
static void udc_set_address(struct s3c_udc *dev, unsigned char address)
{
	u32 tmp = readl(S3C_UDC_OTG_DCFG);
	writel(address << 4 | tmp, S3C_UDC_OTG_DCFG);

#if OTG_DMA_MODE
	writel((1 << 19)|(0<<0), S3C_UDC_OTG_DIEPTSIZ0);
#endif

	tmp = readl(S3C_UDC_OTG_DIEPCTL0);
	writel(DEPCTL_EPENA|DEPCTL_CNAK|tmp, S3C_UDC_OTG_DIEPCTL0); /* EP0: Control IN */

	DEBUG_EP0("%s: USB OTG 2.0 Device address=%d, DCFG=0x%x\n",
		__FUNCTION__, address, readl(S3C_UDC_OTG_DCFG));

	dev->usb_address = address;
}



static int first_time = 1;

static void s3c_ep0_read(struct s3c_udc *dev)
{
	struct s3c_request *req;
	struct s3c_ep *ep = &dev->ep[0];
	int ret;

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct s3c_request, queue);
	else {
		printk("%s: ---> BUG\n", __FUNCTION__);
		BUG();	//logic ensures		-jassi
		return;
	}

	DEBUG_EP0("%s: req.length = 0x%x, req.actual = 0x%x\n",
		__FUNCTION__, req->req.length, req->req.actual);

	if(req->req.length == 0) {
		dev->ep0state = WAIT_FOR_SETUP;
		first_time = 1;
		done(ep, req, 0);
		return;
	}

	if(!req->req.actual && first_time){	//for SetUp packet
		first_time = 0;
		return;
	}

	ret = read_fifo_ep0(ep, req);
	if (ret) {
		dev->ep0state = WAIT_FOR_SETUP;
		first_time = 1;
		done(ep, req, 0);
		return;
	}

}

/*
 * DATA_STATE_XMIT
 */
static int s3c_ep0_write(struct s3c_udc *dev)
{
	struct s3c_request *req;
	struct s3c_ep *ep = &dev->ep[0];
	int ret, need_zlp = 0;

	DEBUG_EP0("%s: ep0 write\n", __FUNCTION__);

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (!req) {
		DEBUG_EP0("%s: NULL REQ\n", __FUNCTION__);
		return 0;
	}

	DEBUG_EP0("%s: req.length = 0x%x, req.actual = 0x%x\n",
		__FUNCTION__, req->req.length, req->req.actual);

	if (req->req.length == 0) {
		dev->ep0state = WAIT_FOR_SETUP;
	   	done(ep, req, 0);
		return 1;
	}

	if (req->req.length - req->req.actual == ep0_fifo_size) {
		/* Next write will end with the packet size, */
		/* so we need Zero-length-packet */
		need_zlp = 1;
	}

	ret = write_fifo_ep0(ep, req);

	if ((ret == 1) && !need_zlp) {
		/* Last packet */
		DEBUG_EP0("%s: finished, waiting for status\n", __FUNCTION__);
		dev->ep0state = WAIT_FOR_SETUP;
	} else {
		DEBUG_EP0("%s: not finished\n", __FUNCTION__);
	}

	if (need_zlp) {
		DEBUG_EP0("%s: Need ZLP!\n", __FUNCTION__);
		dev->ep0state = DATA_STATE_NEED_ZLP;
	}

	if(ret)
	   done(ep, req, 0);

	return 1;
}

/*
 * WAIT_FOR_SETUP (OUT_PKT_RDY)
 */
static void s3c_ep0_setup(struct s3c_udc *dev)
{
	struct s3c_ep *ep = &dev->ep[0];
	int i, bytes, is_in;
	u32 ep_ctrl;

	/* Nuke all previous transfers */
	nuke(ep, -EPROTO);

	/* read control req from fifo (8 bytes) */
	bytes = s3c_fifo_read(ep, (u32 *)&ctrl, 8);

	DEBUG_SETUP("Read CTRL REQ %d bytes\n", bytes);
	DEBUG_SETUP("  CTRL.bRequestType = 0x%x (is_in %d)\n", ctrl.bRequestType,
		    ctrl.bRequestType & USB_DIR_IN);
	DEBUG_SETUP("  CTRL.bRequest = 0x%x\n", ctrl.bRequest);
	DEBUG_SETUP("  CTRL.wLength = 0x%x\n", ctrl.wLength);
	DEBUG_SETUP("  CTRL.wValue = 0x%x (%d)\n", ctrl.wValue, ctrl.wValue >> 8);
	DEBUG_SETUP("  CTRL.wIndex = 0x%x\n", ctrl.wIndex);

	/* Set direction of EP0 */
	if (likely(ctrl.bRequestType & USB_DIR_IN)) {
		ep->bEndpointAddress |= USB_DIR_IN;
		is_in = 1;
	} else {
		ep->bEndpointAddress &= ~USB_DIR_IN;
		is_in = 0;
	}

	dev->req_pending = 1;

	/* Handle some SETUP packets ourselves */
	switch (ctrl.bRequest) {
		case USB_REQ_SET_ADDRESS:
			if (ctrl.bRequestType
				!= (USB_TYPE_STANDARD | USB_RECIP_DEVICE))
				break;

			DEBUG_SETUP("%s: *** USB_REQ_SET_ADDRESS (%d)\n",
					__FUNCTION__, ctrl.wValue);
			udc_set_address(dev, ctrl.wValue);
			return;

		case USB_REQ_SET_CONFIGURATION :
			DEBUG_SETUP("============================================\n");
			DEBUG_SETUP("%s: USB_REQ_SET_CONFIGURATION (%d)\n",
					__FUNCTION__, ctrl.wValue);
config_change:
#if OTG_DMA_MODE
			writel((1 << 19)|(0<<0), S3C_UDC_OTG_DIEPTSIZ0);

			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL0);
			writel(DEPCTL_EPENA|DEPCTL_CNAK|ep_ctrl, S3C_UDC_OTG_DIEPCTL0); /* EP0: Control IN */

			// For Startng EP1 on this new configuration
			ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL1);
			writel(DEPCTL_CNAK|DEPCTL_BULK_TYPE|DEPCTL_USBACTEP|ep_ctrl, S3C_UDC_OTG_DOEPCTL1); /* EP1: Bulk OUT */

			// For starting EP2 on this new configuration
			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL2);
			writel(DEPCTL_CNAK|DEPCTL_BULK_TYPE|DEPCTL_USBACTEP|ep_ctrl, S3C_UDC_OTG_DIEPCTL2); /* EP2: Bulk IN */

			// For starting EP3 on this new configuration
			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL3);
			writel(DEPCTL_CNAK|DEPCTL_BULK_TYPE|DEPCTL_USBACTEP|ep_ctrl, S3C_UDC_OTG_DIEPCTL3); /* EP3: INTR IN */

#else
			// Just to send ZLP(Zero length Packet) to HOST in response to SET CONFIGURATION
			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL0);
			writel(DEPCTL_EPENA|DEPCTL_CNAK|ep_ctrl, S3C_UDC_OTG_DIEPCTL0); /* EP0: Control IN */

			// For Startng EP1 on this new configuration
			ep_ctrl = readl(S3C_UDC_OTG_DOEPCTL1);
			writel(DEPCTL_EPDIS|DEPCTL_CNAK|DEPCTL_BULK_TYPE|DEPCTL_USBACTEP|ep_ctrl, S3C_UDC_OTG_DOEPCTL1); /* EP1: Bulk OUT */

			// For starting EP2 on this new configuration
			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL2);
			writel(DEPCTL_BULK_TYPE|DEPCTL_USBACTEP|ep_ctrl, S3C_UDC_OTG_DIEPCTL2); /* EP2: Bulk IN */

			// For starting EP3 on this new configuration
			ep_ctrl = readl(S3C_UDC_OTG_DIEPCTL3);
			writel(DEPCTL_BULK_TYPE|DEPCTL_USBACTEP|ep_ctrl, S3C_UDC_OTG_DIEPCTL3); /* EP3: INTR IN */

#endif
			DEBUG_SETUP("%s:(DOEPCTL1):0x%x, (DIEPCTL2):0x%x, (DIEPCTL3):0x%x\n",
				__FUNCTION__,
				readl(S3C_UDC_OTG_DOEPCTL1),
				readl(S3C_UDC_OTG_DIEPCTL2),
				readl(S3C_UDC_OTG_DIEPCTL3));

			DEBUG_SETUP("============================================\n");

			reset_available = 1;
			dev->req_config = 1;
			break;

		case USB_REQ_GET_DESCRIPTOR:
			DEBUG_SETUP("%s: *** USB_REQ_GET_DESCRIPTOR  \n",__FUNCTION__);
			break;

		case USB_REQ_SET_INTERFACE:
			DEBUG_SETUP("%s: *** USB_REQ_SET_INTERFACE (%d)\n",
					__FUNCTION__, ctrl.wValue);

			set_interface_first = 1;
			goto config_change;
			break;

		case USB_REQ_GET_CONFIGURATION:
			DEBUG_SETUP("%s: *** USB_REQ_GET_CONFIGURATION  \n",__FUNCTION__);
			break;

		case USB_REQ_GET_STATUS:
			DEBUG_SETUP("%s: *** USB_REQ_GET_STATUS  \n",__FUNCTION__);
			break;

		default:
			DEBUG_SETUP("%s: *** Default of ctrl.bRequest=0x%x happened.\n",
					__FUNCTION__, ctrl.bRequest);
			break;
	}

	if (likely(dev->driver)) {
		/* device-2-host (IN) or no data setup command,
		 * process immediately */
		spin_unlock(&dev->lock);
		DEBUG_SETUP("%s: usb_ctrlrequest will be passed to fsg_setup()\n", __FUNCTION__);
		i = dev->driver->setup(&dev->gadget, &ctrl);
		spin_lock(&dev->lock);

		if (i < 0) {
			/* setup processing failed, force stall */
			DEBUG_SETUP("%s: gadget setup FAILED (stalling), setup returned %d\n",
				__FUNCTION__, i);
			/* ep->stopped = 1; */
			dev->ep0state = WAIT_FOR_SETUP;
		}
	}
}

/*
 * handle ep0 interrupt
 */
static void s3c_handle_ep0(struct s3c_udc *dev)
{
	if (dev->ep0state == WAIT_FOR_SETUP) {
		DEBUG_EP0("%s: WAIT_FOR_SETUP\n", __FUNCTION__);
		s3c_ep0_setup(dev);

	} else {
		DEBUG_EP0("%s: strange state!!(state = %s)\n",
			__FUNCTION__, state_names[dev->ep0state]);
	}
}

static void s3c_ep0_kick(struct s3c_udc *dev, struct s3c_ep *ep)
{
	DEBUG_EP0("%s: ep_is_in = %d\n", __FUNCTION__, ep_is_in(ep));
	if (ep_is_in(ep)) {
		dev->ep0state = DATA_STATE_XMIT;
		s3c_ep0_write(dev);
	} else {
		dev->ep0state = DATA_STATE_RECV;
		s3c_ep0_read(dev);
	}
}

/* ---------------------------------------------------------------------------
 * 	device-scoped parts of the api to the usb controller hardware
 * ---------------------------------------------------------------------------
 */

static int s3c_udc_get_frame(struct usb_gadget *_gadget)
{
	/*fram count number [21:8]*/
	u32 frame = readl(S3C_UDC_OTG_DSTS);

	DEBUG("%s: %p\n", __FUNCTION__, _gadget);
	return (frame & 0x3ff00);
}

static int s3c_udc_wakeup(struct usb_gadget *_gadget)
{
	return -ENOTSUPP;
}

static const struct usb_gadget_ops s3c_udc_ops = {
	.get_frame = s3c_udc_get_frame,
	.wakeup = s3c_udc_wakeup,
	/* current versions must always be self-powered */
};

static void nop_release(struct device *dev)
{
	DEBUG("%s %s\n", __FUNCTION__, dev->bus_id);
}

static struct s3c_udc memory = {
	.usb_address = 0,

	.gadget = {
		   .ops = &s3c_udc_ops,
		   .ep0 = &memory.ep[0].ep,
		   .name = driver_name,
		   .dev = {
			   .bus_id = "gadget",
			   .release = nop_release,
			   },
		   },

	/* control endpoint */
	.ep[0] = {
		  .ep = {
			 .name = ep0name,
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP0_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = 0,
		  .bmAttributes = 0,

		  .ep_type = ep_control,
		  .fifo = (u32) S3C_UDC_OTG_EP0_FIFO,
		  },

	/* first group of endpoints */
	.ep[1] = {
		  .ep = {
			 .name = "ep1-bulk",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = 1,
		  .bmAttributes = USB_ENDPOINT_XFER_BULK,

		  .ep_type = ep_bulk_out,
		  .fifo = (u32) S3C_UDC_OTG_EP1_FIFO,
		  },

	.ep[2] = {
		  .ep = {
			 .name = "ep2-bulk",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 2,
		  .bmAttributes = USB_ENDPOINT_XFER_BULK,

		  .ep_type = ep_bulk_in,
		  .fifo = (u32) S3C_UDC_OTG_EP2_FIFO,
		  },

	.ep[3] = {				// Though NOT USED XXX
		  .ep = {
			 .name = "ep3-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 3,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP3_FIFO,
		  },
	.ep[4] = {				// Though NOT USED XXX
		  .ep = {
			 .name = "ep4-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 4,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP4_FIFO,
		  },
	.ep[5] = {				// Though NOT USED XXX
		  .ep = {
			 .name = "ep5-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE2,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 5,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP5_FIFO,
		  },
	.ep[6] = {				// Though NOT USED XXX
		  .ep = {
			 .name = "ep6-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE2,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 6,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP6_FIFO,
		  },
	.ep[7] = {				// Though NOT USED XXX
		  .ep = {
			 .name = "ep7-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE2,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 7,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP7_FIFO,
		  },
	.ep[8] = {				// Though NOT USED XXX
		  .ep = {
			 .name = "ep8-int",
			 .ops = &s3c_ep_ops,
			 .maxpacket = EP_FIFO_SIZE2,
			 },
		  .dev = &memory,

		  .bEndpointAddress = USB_DIR_IN | 8,
		  .bmAttributes = USB_ENDPOINT_XFER_INT,

		  .ep_type = ep_interrupt,
		  .fifo = (u32) S3C_UDC_OTG_EP8_FIFO,
		  },
};

/*
 * 	probe - binds to the platform device
 */
static struct clk	*otg_clock = NULL;

static int s3c_udc_probe(struct platform_device *pdev)
{
	struct s3c_udc *dev = &memory;
	int retval;

	DEBUG("%s: %p\n", __FUNCTION__, pdev);

	spin_lock_init(&dev->lock);
	dev->dev = pdev;

	device_initialize(&dev->gadget.dev);
	dev->gadget.dev.parent = &pdev->dev;

	dev->gadget.is_dualspeed = 1;	// Hack only
	dev->gadget.is_otg = 0;
	dev->gadget.is_a_peripheral = 0;
	dev->gadget.b_hnp_enable = 0;
	dev->gadget.a_hnp_support = 0;
	dev->gadget.a_alt_hnp_support = 0;

	the_controller = dev;
	platform_set_drvdata(pdev, dev);

	otg_clock = clk_get(&pdev->dev, "otg");
	if (otg_clock == NULL) {
		printk(KERN_INFO "failed to find otg clock source\n");
		return -ENOENT;
	}
	clk_enable(otg_clock);

	udc_reinit(dev);

	local_irq_disable();

	/* irq setup after old hardware state is cleaned up */
	retval =
	    request_irq(IRQ_OTG, s3c_udc_irq, SA_INTERRUPT, driver_name,
			dev);

	if (retval != 0) {
		DEBUG(KERN_ERR "%s: can't get irq %i, err %d\n", driver_name,
		      IRQ_OTG, retval);
		return -EBUSY;
	}

	disable_irq(IRQ_OTG);
	local_irq_enable();
	create_proc_files();

	return retval;
}

static int s3c_udc_remove(struct platform_device *pdev)
{
	struct s3c_udc *dev = platform_get_drvdata(pdev);

	DEBUG("%s: %p\n", __FUNCTION__, pdev);

	if (otg_clock != NULL) {
		clk_disable(otg_clock);
		clk_put(otg_clock);
		otg_clock = NULL;
	}

	remove_proc_files();
	usb_gadget_unregister_driver(dev->driver);

	free_irq(IRQ_OTG, dev);

	platform_set_drvdata(pdev, 0);

	the_controller = 0;

	return 0;
}

#ifdef CONFIG_PM
static int s3c_udc_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int s3c_udc_resume(struct platform_device *dev)
{
	return 0;
}
#else
#define s3c_udc_suspend NULL
#define s3c_udc_resume  NULL
#endif /* CONFIG_PM */

/*-------------------------------------------------------------------------*/
static struct platform_driver s3c_udc_driver = {
	.probe		= s3c_udc_probe,
	.remove		= s3c_udc_remove,
	.suspend	= s3c_udc_suspend,
	.resume		= s3c_udc_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c2410-usbgadget",
	},
};

static int __init udc_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c_udc_driver);
	if(!ret)
	   printk("Loaded %s version %s %s\n", driver_name, DRIVER_VERSION,
	   		OTG_DMA_MODE? "(DMA Mode)" : "(Slave Mode)");

	return ret;
}

static void __exit udc_exit(void)
{
	platform_driver_unregister(&s3c_udc_driver);
	printk("Unloaded %s version %s\n", driver_name, DRIVER_VERSION);
}

module_init(udc_init);
module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Samsung");
MODULE_LICENSE("GPL");
