/*
 * drivers/usb/gadget/s3c-udc-hs.c
 * Samsung S3C on-chip full/high speed USB device controllers
 *
 * $Id: s3c-udc-hs.c,v 1.2 2008/03/05 02:14:45 ihlee215 Exp $*
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

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/irq.h>
#include <asm/arch/regs-irq.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-s3c2443-clock.h>
#include <asm/arch/regs-udc-hs.h>
#include "s3c-udc.h"

#undef DEBUG_S3C_UDC_SETUP
#undef DEBUG_S3C_UDC_EP0
#undef DEBUG_S3C_UDC

// USB Device DMA support
#define RX_DMA_MODE 0
#define TX_DMA_MODE 0
static int tx_dmaStart = 0;
static int rx_dmaStart = 0;


#ifdef DEBUG_S3C_UDC_SETUP
static char *state_names[] = {
	"WAIT_FOR_SETUP",
	"DATA_STATE_XMIT",
	"DATA_STATE_NEED_ZLP",
	"WAIT_FOR_OUT_STATUS",
	"DATA_STATE_RECV"
	};
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

#define	DRIVER_DESC		"Samsung Dual-speed USB Device Controller"
#define	DRIVER_VERSION		__DATE__

struct s3c_udc	*the_controller;

static const char driver_name[] = "s3c-udc";
static const char driver_desc[] = DRIVER_DESC;
static const char ep0name[] = "ep0-control";

// Max packet size
static u32 ep0_fifo_size = 64;
static u32 ep_fifo_size =  512;
static u32 ep_fifo_size2 = 1024;

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
static void s3c_ep0_kick(struct s3c_udc *dev, struct s3c_ep *ep);
static void s3c_handle_ep0(struct s3c_udc *dev);

static void done(struct s3c_ep *ep, struct s3c_request *req,
		 int status);
static void stop_activity(struct s3c_udc *dev,
			  struct usb_gadget_driver *driver);
static int udc_enable(struct s3c_udc *dev);
static void udc_set_address(struct s3c_udc *dev, unsigned char address);
static void reconfig_usbd(void);

static __inline__ u32 usb_read(u32 port, u8 ind)
{
	__raw_writel(ind, S3C_UDC_INDEX_REG);
	return __raw_readl(port);
}

static __inline__ void usb_write(u32 val, u32 port, u8 ind)
{
	__raw_writel(ind, S3C_UDC_INDEX_REG);
	__raw_writel(val, port);
}

static __inline__ void usb_set(u32 val, u32 port, u8 ind)
{
	__raw_writel(ind, S3C_UDC_INDEX_REG);
	__raw_writel(__raw_readl(port) | val, port);
}

static __inline__ void usb_clear(u32 val, u32 port, u8 ind)
{
	__raw_writel(ind, S3C_UDC_INDEX_REG);
	__raw_writel(__raw_readl(port) & ~val, port);
}

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


/* Inline code */
static __inline__ int write_packet(struct s3c_ep *ep,
				   struct s3c_request *req, int max)
{
	u16 *buf;
	int length, count;
	u32 fifo = ep->fifo;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);

	length = req->req.length - req->req.actual;
	length = min(length, max);
	req->req.actual += length;

	DEBUG("%s: Write %d (max %d), fifo=0x%x\n",
		__FUNCTION__, length, max, fifo);

	usb_write(length, (u32) S3C_UDC_BYTE_WRITE_CNT_REG, ep_index(ep));


	for (count=0;count<length;count+=2) {
	  		__raw_writel(*buf++, fifo);
	}
	
	return length;
}

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
	__raw_writel(__raw_readl(S3C_PWRCFG)&~(1<<4), S3C_PWRCFG);
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

	/* if reset by sleep wakeup, control the retention I/O cell */
	if (__raw_readl(S3C_RSTSTAT) & 0x8)
		__raw_writel(__raw_readl(S3C_RSTCON)|(1<<16), S3C_RSTCON);


	/* USB Port is Normal mode */
	__raw_writel(__raw_readl(S3C2410_MISCCR)&~(1<<12), S3C2410_MISCCR);

	/* PHY power enable */
	__raw_writel(__raw_readl(S3C_PWRCFG)|(1<<4), S3C_PWRCFG);

	/* USB device 2.0 must reset like bellow,
	 * 1st phy reset and after at least 10us, func_reset & host reset
	 * phy reset can reset bellow registers.
	 */
	/* PHY 2.0 S/W reset */
	__raw_writel((0<<2)|(0<<1)|(1<<0), S3C_URSTCON);
	udelay(20); /* phy reset must be asserted for at 10us */
	
	/*Function 2.0, Host 1.1 S/W reset*/
	__raw_writel((1<<2)|(1<<1)|(0<<0), S3C_URSTCON);
	__raw_writel((0<<2)|(0<<1)|(0<<0), S3C_URSTCON);
	
	/* 48Mhz,Crystal,External X-tal,device */
	__raw_writel((0<<3)|(0<<2)|(1<<1)|(0<<0), S3C_PHYCTRL);
	
	/* 48Mhz clock on ,PHY2.0 analog block power on
	 * XO block power on,XO block power in suspend mode,
	 * PHY 2.0 Pll power on ,suspend signal for save mode disable
	 */
	__raw_writel((1<<31)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0), S3C_PHYPWR);

	/* D+ pull up disable(VBUS detect), USB2.0 Function clock Enable,
	 * USB1.1 HOST disable, USB2.0 PHY test enable
	 */
	__raw_writel((0<<31)|(1<<2)|(0<<1)|(1<<0), S3C_UCLKCON);

	__raw_writel(IRQ_USBD, S3C2410_INTPND);
	__raw_writel(IRQ_USBD, S3C2410_SRCPND);

	reconfig_usbd();
	
	__raw_writel(__raw_readl(S3C2410_INTMSK)&~(IRQ_USBD), S3C2410_INTMSK);

	/* D+ pull up , USB2.0 Function clock Enable,
	 * USB1.1 HOST disable,USB2.0 PHY test enable
	 */
	__raw_writel((1<<31)|(1<<2)|(0<<1)|(1<<0), S3C_UCLKCON);
	
	DEBUG_SETUP("S3C2443 USB Controller Core Initialized\n");
	
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

	enable_irq(IRQ_USBD);

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

	disable_irq(IRQ_USBD);

	printk("Unregistered gadget driver '%s'\n", driver->driver.name);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*-------------------------------------------------------------------------*/

/** Write request to FIFO (max write == maxp size)
 *  Return:  0 = still running, 1 = completed, negative = errno
 */
static int write_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 max;
	unsigned count;
	int is_last = 0, is_short = 0;

#if TX_DMA_MODE
	if(tx_dmaStart) return 0;
	if ((req->req.actual == 0) && (req->req.length > ep->ep.maxpacket) && (!tx_dmaStart)) {

		DEBUG("TX_DMA_Start:: %s: read %s, bytes req %p %d/%d\n",
		__FUNCTION__, ep->ep.name, req, req->req.actual, req->req.length);

		usb_set(S3C_DMA_ENABLE,
				(u32) S3C_UDC_FCON_REG, ep_index(ep));

		usb_write(ep->ep.maxpacket, (u32) S3C_UDC_MAXP_REG, ep_index(ep));
		usb_write(ep->ep.maxpacket, (u32) S3C_UDC_BYTE_WRITE_CNT_REG, ep_index(ep));
		
		usb_set((u16)req->req.length,
				(u32) S3C_UDC_DMA_TOTAL_CNT1_REG, ep_index(ep));
		
		usb_set((u16)((req->req.length)>>16),
				(u32) S3C_UDC_DMA_TOTAL_CNT2_REG, ep_index(ep));

		usb_set(virt_to_phys(req->req.buf),
				(u32) S3C_UDC_DMA_MEM_BASE_ADDR_REG, ep_index(ep));
		
		usb_set(ep->ep.maxpacket,
				(u32) S3C_UDC_DMA_CNT_REG, ep_index(ep));
		
		usb_set(ep->ep.maxpacket,
				(u32) S3C_UDC_DMA_FIFO_CNT_REG, ep_index(ep));

		usb_set(S3C_MAX_BURST_INCR16,
				(u32) S3C_UDC_DMA_IF_CON_REG, ep_index(ep));

		usb_set(S3C_DMA_FLY_ENABLE|S3C_DMA_TX_START|S3C_USB_DMA_MODE,
				(u32) S3C_UDC_DMA_CON_REG, ep_index(ep));
		tx_dmaStart = 1;

		udelay(600);
		
	}
	else {
#endif 
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
	
		DEBUG("%s: wrote %s %d bytes%s%s req %p %d/%d\n", __FUNCTION__,
	      	ep->ep.name, count,
	     	 is_last ? "/L" : "", is_short ? "/S" : "",
	      	req, req->req.actual, req->req.length);

#if TX_DMA_MODE	
	}
#endif
	
	/* requests complete when all IN data is in the FIFO */
	if (is_last) {
		if(!ep_index(ep)){
			DEBUG("%s: --> Error EP0 must not come here!\n",
				__FUNCTION__);
			BUG();
		}
		done(ep, req, 0);
		return 1;
	}

	return 0;
}



/** Read to request from FIFO (max read == bytes in fifo)
 *  Return:  0 = still running, 1 = completed, negative = errno
 */
static int read_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 csr;
	u16 *buf;
	unsigned bufferspace, count, count_bytes, is_short = 0;
	u32 fifo = ep->fifo;

	csr = usb_read( (u32) S3C_UDC_EP_STATUS_REG, ep_index(ep));

	/* make sure there's a packet in the FIFO. */
	if (!(csr & S3C_UDC_EP_RX_SUCCESS)) {
		DEBUG("%s: Packet NOT ready!\n", __FUNCTION__);
		return -EINVAL;
	}

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	/* read all bytes from this packet */
	count = usb_read((u32) S3C_UDC_BYTE_READ_CNT_REG, ep_index(ep));
	if (csr & S3C_UDC_EP_LWO)
		count_bytes = count * 2 -1;
	else
		count_bytes = count * 2;


#if RX_DMA_MODE
	if(rx_dmaStart) return 0;
	
	if ((req->req.actual == 0) && (req->req.length > ep->ep.maxpacket) && (!rx_dmaStart)) {

		DEBUG("RX_DMA_Start :: %s: read %s, %d bytes req %p %d/%d CSR::0x%x\n",
		__FUNCTION__, ep->ep.name, count_bytes, req, req->req.actual, req->req.length, csr);
		

		usb_set(S3C_DMA_ENABLE,
				(u32) S3C_UDC_FCON_REG, ep_index(ep));
		
		usb_set((u16)req->req.length,
				(u32) S3C_UDC_DMA_TOTAL_CNT1_REG, ep_index(ep));
		
		usb_set((u16)((req->req.length)>>16),
				(u32) S3C_UDC_DMA_TOTAL_CNT2_REG, ep_index(ep));
		
		usb_set(ep->ep.maxpacket,
				(u32) S3C_UDC_DMA_CNT_REG, ep_index(ep));
		//usb_set(512,
				//(u32) S3C_UDC_DMA_FIFO_CNT_REG, ep_index(ep));

		usb_set(S3C_MAX_BURST_INCR16,
				(u32) S3C_UDC_DMA_IF_CON_REG, ep_index(ep));

		usb_set(virt_to_phys(buf),
				(u32) S3C_UDC_DMA_MEM_BASE_ADDR_REG, ep_index(ep));

		usb_set(S3C_DMA_FLY_ENABLE|S3C_DMA_RX_START|S3C_USB_DMA_MODE,
				(u32) S3C_UDC_DMA_CON_REG, ep_index(ep));
		rx_dmaStart = 1;

		
	}
	else {
#endif	
		req->req.actual += min(count_bytes, bufferspace);

		is_short = (count_bytes < ep->ep.maxpacket);
		DEBUG("%s: read %s, %d bytes%s req %p %d/%d\n",
			__FUNCTION__,
			ep->ep.name, count_bytes,
			is_short ? "/S" : "", req, req->req.actual, req->req.length);

		while (likely(count-- != 0)) {
			u16 byte = (u16) __raw_readl(fifo);

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
				bufferspace--;
			}
	 	 }

#if RX_DMA_MODE	
	}
#endif
	
	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		done(ep, req, 0);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct s3c_ep *ep, struct s3c_request *req, int status)
{
	unsigned int stopped = ep->stopped;

	DEBUG("%s: %p\n", __FUNCTION__, ep);
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

	DEBUG("%s: %p\n", __FUNCTION__, ep);

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct s3c_request, queue);
		done(ep, req, status);
	}
}

/**
 * s3c_in_epn - handle IN interrupt
 */
static void s3c_in_epn(struct s3c_udc *dev, u32 ep_idx)
{
	u32 csr;
	struct s3c_ep *ep = &dev->ep[ep_idx];
	struct s3c_request *req;

	csr = usb_read((u32) S3C_UDC_EP_STATUS_REG, ep_idx);
	DEBUG("%s: S3C_UDC_EP%d_STATUS_REG=0x%x\n", __FUNCTION__, ep_idx, csr);

	if (csr & S3C_UDC_EP_STALL) {
		DEBUG("%s: S3C_UDC_EP_STALL\n", __FUNCTION__);
		usb_set(S3C_UDC_EP_STALL, (u32) S3C_UDC_EP_STATUS_REG, ep_idx);
		return;
	}

	if (!ep->desc) {
		DEBUG("%s: NO EP DESC\n", __FUNCTION__);
		return;
	}

	
	if (csr & S3C_UDC_EP_DTCZ) {
		DEBUG("%s: TX_DMA :: DMA Total Count Zero:: S3C_UDC_EP%d_STATUS_REG=0x%x\n",
		__FUNCTION__, ep_index(ep), csr);
		
		tx_dmaStart = 0;
		usb_set(S3C_DMA_DISABLE,
				(u32) S3C_UDC_FCON_REG, ep_idx);
		
		usb_set(S3C_DMA_TX_STOP|S3C_USB_INT_MODE,
				(u32) S3C_UDC_DMA_CON_REG, ep_idx);
		
		usb_set(S3C_UDC_EP_DTCZ, (u32) S3C_UDC_EP_STATUS_REG, ep_idx);


		if (list_empty(&ep->queue))
			req = 0;
		else
			req = list_entry(ep->queue.next,
					struct s3c_request, queue);
		if (unlikely(!req)) {
			DEBUG("%s: TX_DMA :: NULL REQ\n", __FUNCTION__, ep_idx, req);
			//return;
		} else {
			DEBUG("%s: TX_DMA_DONE - REQ is %p (%d/%d) CSR:0x%x\n", __FUNCTION__, req, req->req.actual, req->req.length, csr);
			req->req.actual = req->req.length;
			done(ep, req, 0);
		}

	}
	

	if (csr & S3C_UDC_EP_TX_SUCCESS) {
		usb_set(S3C_UDC_EP_TX_SUCCESS,
				(u32) S3C_UDC_EP_STATUS_REG, ep_idx);
		
		if (list_empty(&ep->queue))
			req = 0;
		else
			req = list_entry(ep->queue.next, struct s3c_request, queue);

		if (unlikely(!req)) {
			DEBUG("%s:NULL REQ :: EP_TX_SUCCESS, req = %p CSR:0x%x\n", __FUNCTION__, req, csr);
			return;
		}
		else {
			DEBUG("%s: EP_TX_SUCCESS, req = %p CSR:0x%x\n", __FUNCTION__, req, csr);
			if ((write_fifo(ep, req)==0) && (csr & S3C_UDC_EP_PSIF_TWO))
				write_fifo(ep, req);
			
		}
	}

}

/* ********************************************************************************************* */
/* Bulk OUT (recv)
 */

static void s3c_out_epn(struct s3c_udc *dev, u32 ep_idx)
{
	struct s3c_ep *ep = &dev->ep[ep_idx];
	struct s3c_request *req;
	u32 csr;
	
	csr=usb_read((u32) S3C_UDC_EP_STATUS_REG, ep_index(ep));
	DEBUG("%s: S3C_UDC_EP%d_STATUS_REG=0x%x\n",
		__FUNCTION__, ep_index(ep), csr);

	if (unlikely(!(ep->desc))) {
		/* Throw packet away.. */
		printk("%s: No descriptor?!?\n", __FUNCTION__);
		return;
	}
	
	if (csr & S3C_UDC_EP_STALL) {
		DEBUG("%s: stall sent\n", __FUNCTION__);
		usb_set(S3C_UDC_EP_STALL, (u32) S3C_UDC_EP_STATUS_REG, ep_idx);
		return;
	}

	if (csr & S3C_UDC_EP_FIFO_FLUSH) {
		DEBUG("%s: fifo flush \n", __FUNCTION__);
		usb_set(S3C_UDC_EP_FIFO_FLUSH, (u32) S3C_UDC_EP_CON_REG, ep_idx);
		return;
	}
		

	if (csr & S3C_UDC_EP_DTCZ) {
		DEBUG("%s: DMA Total Count Zero:: S3C_UDC_EP%d_STATUS_REG=0x%x\n",
		__FUNCTION__, ep_index(ep), csr);
		
		rx_dmaStart = 0;
		usb_set(S3C_DMA_DISABLE,
				(u32) S3C_UDC_FCON_REG, ep_idx);
		
		usb_set(S3C_DMA_RX_STOP|S3C_USB_INT_MODE,
				(u32) S3C_UDC_DMA_CON_REG, ep_idx);
		
		usb_set(S3C_UDC_EP_DTCZ, (u32) S3C_UDC_EP_STATUS_REG, ep_idx);
		
		if (list_empty(&ep->queue))
			req = 0;
		else
			req = list_entry(ep->queue.next,
					struct s3c_request, queue);
		if (unlikely(!req)) {
			DEBUG("RX_DMA :: %s: NULL REQ\n", __FUNCTION__, ep_idx, req);
		} else {
			DEBUG("%s : RX_DMA_DONE - REQ : %p\n", __FUNCTION__, req);
			req->req.actual = req->req.length;
			done(ep, req, 0);
		}
	}


	if (csr & S3C_UDC_EP_RX_SUCCESS) {
		if(!rx_dmaStart) {
			if (list_empty(&ep->queue))
				req = 0;
			else
				req = list_entry(ep->queue.next,
					struct s3c_request, queue);

			if (unlikely(!req)) {
				DEBUG("%s: NULL REQ on ISR %d\n", __FUNCTION__, ep_idx);
				return;
			} else {
				if(((read_fifo(ep, req))==0) && (csr & S3C_UDC_EP_PSIF_TWO))
					read_fifo(ep, req);
			}
		}
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
	__raw_writel(0x04, S3C_UDC_EP_DIR_REG); /* EP1:OUT, EP2:IN */

	/* EP0~2 Interrupt enable */	
	__raw_writel(0x7, S3C_UDC_EP_INT_EN_REG);
	
	__raw_writel(0x0000, S3C_UDC_TEST_REG);

	/* error interrupt enable, 16bit bus, Little format,
	 * suspend&reset enable
	 */
	__raw_writel(S3C_UDC_DTZIEN_EN|S3C_UDC_RRD_EN
			|S3C_UDC_SUS_EN|S3C_UDC_RST_EN,
			S3C_UDC_SYS_CON_REG);
	
	__raw_writel(0x0000, S3C_UDC_EP0_CON_REG);

	/* EP1, EP2 dual FIFO mode enable */
	usb_write(0x0080, (u32) S3C_UDC_EP_CON_REG, 1);
	usb_write(0x0080, (u32) S3C_UDC_EP_CON_REG, 2);

	__raw_writel(0, S3C_UDC_INDEX_REG);

}

void set_max_pktsize(struct s3c_udc *dev, enum usb_device_speed speed)
{
	if (speed == USB_SPEED_HIGH)
	{
		ep0_fifo_size = 64;
		ep_fifo_size = 512;
		ep_fifo_size2 = 1024;
		dev->gadget.speed = USB_SPEED_HIGH;
	}
	else
	{
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
	
	usb_write(ep0_fifo_size, (u32) S3C_UDC_MAXP_REG, 0);
	usb_write(ep_fifo_size, (u32) S3C_UDC_MAXP_REG, 1);
	usb_write(ep_fifo_size, (u32) S3C_UDC_MAXP_REG, 2);
	usb_write(ep_fifo_size, (u32) S3C_UDC_MAXP_REG, 3);
	usb_write(ep_fifo_size, (u32) S3C_UDC_MAXP_REG, 4);
	usb_write(ep_fifo_size2, (u32) S3C_UDC_MAXP_REG, 5);
	usb_write(ep_fifo_size2, (u32) S3C_UDC_MAXP_REG, 6);
	usb_write(ep_fifo_size2, (u32) S3C_UDC_MAXP_REG, 7);
	usb_write(ep_fifo_size2, (u32) S3C_UDC_MAXP_REG, 8);
	
}

/*
 *	elfin usb client interrupt handler.
 */
static irqreturn_t s3c_udc_irq(int irq, void *_dev)
{
	struct s3c_udc *dev = _dev;
	u32 intr_out;
	u32 intr_in;
	u32 intr_status, intr_status_chk;
	
	spin_lock(&dev->lock);


	intr_status = __raw_readl(S3C_UDC_SYS_STATUS_REG);
	intr_out = intr_in = __raw_readl(S3C_UDC_EP_INT_REG);
	
	DEBUG_SETUP("\n\n%s: S3C_UDC_EP_INT_REG=0x%x, S3C_UDC_SYS_STATUS_REG=0x%x(on state %s)\n",
			__FUNCTION__, intr_out, intr_status, state_names[dev->ep0state]);

	/* We have only 3 usable eps now */
	intr_status_chk = intr_status & S3C_UDC_INT_CHECK;
	intr_in &= (S3C_UDC_INT_EP2 | S3C_UDC_INT_EP0);
	intr_out &= S3C_UDC_INT_EP1;


	if (!intr_out && !intr_in && !intr_status_chk){
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (intr_status) {
		if (intr_status & S3C_UDC_INT_VBUSON) {
			DEBUG_SETUP("%s: VBUSON interrupt\n", __FUNCTION__);
			__raw_writel(S3C_UDC_INT_VBUSON, S3C_UDC_SYS_STATUS_REG);
		}

		if (intr_status & S3C_UDC_INT_ERR) {
			DEBUG_SETUP("%s: Error interrupt\n", __FUNCTION__);
			__raw_writel(S3C_UDC_INT_ERR, S3C_UDC_SYS_STATUS_REG);
		}

		if (intr_status & S3C_UDC_INT_SDE) {
			DEBUG_SETUP("%s: Speed Detection interrupt\n",
					__FUNCTION__);
			__raw_writel(S3C_UDC_INT_SDE, S3C_UDC_SYS_STATUS_REG);

			if (intr_status & S3C_UDC_INT_HSP) {
				DEBUG_SETUP("%s: High Speed Detection\n",
						__FUNCTION__);
				set_max_pktsize(dev, USB_SPEED_HIGH);
			}
			else {
				DEBUG_SETUP("%s: Full Speed Detection\n",
						__FUNCTION__);
				set_max_pktsize(dev, USB_SPEED_FULL);
			}
		}
		
		if (intr_status & S3C_UDC_INT_SUSPEND) {
			DEBUG_SETUP("%s: SUSPEND interrupt\n", __FUNCTION__);
			__raw_writel(S3C_UDC_INT_SUSPEND, S3C_UDC_SYS_STATUS_REG);
			if (dev->gadget.speed != USB_SPEED_UNKNOWN
			    && dev->driver
			    && dev->driver->suspend) {
				dev->driver->suspend(&dev->gadget);
			}
		}

		if (intr_status & S3C_UDC_INT_RESUME) {
			DEBUG_SETUP("%s: RESUME interrupt\n", __FUNCTION__);
			__raw_writel(S3C_UDC_INT_RESUME, S3C_UDC_SYS_STATUS_REG);
			if (dev->gadget.speed != USB_SPEED_UNKNOWN
			    && dev->driver
			    && dev->driver->resume) {
				dev->driver->resume(&dev->gadget);
			}
		}

		if (intr_status & S3C_UDC_INT_RESET) {
			DEBUG_SETUP("%s: RESET interrupt\n", __FUNCTION__);
			__raw_writel(S3C_UDC_INT_RESET, S3C_UDC_SYS_STATUS_REG);
			reconfig_usbd();
			dev->ep0state = WAIT_FOR_SETUP;
		}
	}

	if (intr_in) {
		if (intr_in & S3C_UDC_INT_EP0){
			__raw_writel(S3C_UDC_INT_EP0, S3C_UDC_EP_INT_REG);
			s3c_handle_ep0(dev);
		}else if (intr_in & S3C_UDC_INT_EP2){
			__raw_writel(S3C_UDC_INT_EP2, S3C_UDC_EP_INT_REG);
			s3c_in_epn(dev, 2);	// hard coded !!!   
		}else{
			__raw_writel(S3C_UDC_INT_EP3, S3C_UDC_EP_INT_REG);
			s3c_in_epn(dev, 3);	// hard coded !!!   
		}
	}

	if (intr_out) {
		usb_write(S3C_UDC_INT_EP1, (u32) S3C_UDC_EP_INT_REG, 1);
		s3c_out_epn(dev, 1);		// hard coded !!!   
	}
	
	__raw_writel(IRQ_USBD, S3C2410_INTPND);
	__raw_writel(IRQ_USBD, S3C2410_SRCPND);
	
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

	DEBUG("%s: enabled %s\n", __FUNCTION__, _ep->name);
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

	DEBUG("%s: %p\n", __FUNCTION__, ep);

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

	DEBUG("%s: %p\n", __FUNCTION__, _ep);

	req = container_of(_req, struct s3c_request, req);
	if (unlikely
	    (!_req || !_req->complete || !_req->buf
	     || !list_empty(&req->queue))) {
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

	DEBUG("%s: %s queue req %p, len %d buf %p\n",
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
			csr = usb_read((u32) S3C_UDC_EP_STATUS_REG, ep_index(ep));
			DEBUG("%s: ep_is_in, S3C_UDC_EP%d_STATUS_REG=0x%x\n",
				__FUNCTION__, ep_index(ep), csr);
			
			if(!(csr & S3C_UDC_EP_TX_SUCCESS)
			   && (write_fifo(ep, req) == 1))
				req = 0;
			else
				DEBUG("IN-list_add_taill :: req =%p\n", req);
			
		} else {
			csr = usb_read((u32) S3C_UDC_EP_STATUS_REG, ep_index(ep));
			DEBUG("%s: ep_is_out, S3C_UDC_EP%d_STATUS_REG=0x%x\n",
				__FUNCTION__, ep_index(ep),csr);
			
			if((csr & S3C_UDC_EP_RX_SUCCESS)
			   && (read_fifo(ep, req) == 1))
				req = 0;
			else
				DEBUG("OUT-list_add_taill :: req =%p\n", req);
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
	u32 csr;
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

	csr = usb_read((u32) S3C_UDC_EP_STATUS_REG, ep_index(ep));
	if (ep->dev->gadget.speed != USB_SPEED_UNKNOWN ||
	    csr & S3C_UDC_EP_RX_SUCCESS) {
	    
		count = usb_read((u32) S3C_UDC_BYTE_READ_CNT_REG, ep_index(ep));
		
		if (usb_read((u32) S3C_UDC_EP_STATUS_REG, ep_index(ep))
			& S3C_UDC_EP_LWO)
			count = count * 2 -1;
		else
			count = count * 2;
	}

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

	count = write_packet(ep, req, max);

	/* last packet is usually short (or a zlp) */
	if (unlikely(count != max))
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

static __inline__ int s3c_fifo_read(struct s3c_ep *ep, u16 *cp, int max)
{
	int bytes;
	int count;
	
	count = usb_read((u32) S3C_UDC_BYTE_READ_CNT_REG, ep_index(ep));
	DEBUG_EP0("%s: count=%d, ep_index=%d, fifo=0x%x\n",
			__FUNCTION__, count, ep_index(ep), ep->fifo);
	bytes = count * 2;
	
	while (count--) {
		*cp++ = (u16) __raw_readl(S3C_UDC_EP0_FIFO_REG);
	}

	__raw_writel(S3C_UDC_EP0_RX_SUCCESS, S3C_UDC_EP0_STATUS_REG);/* clear */
	
	return bytes;
}

static int read_fifo_ep0(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 csr;
	u16 *buf;
	unsigned bufferspace, count, is_short, bytes;
	u32 fifo = ep->fifo;

	DEBUG_EP0("%s\n", __FUNCTION__);

	csr = __raw_readl(S3C_UDC_EP0_STATUS_REG);
	if (!(csr & S3C_UDC_EP0_RX_SUCCESS))
		return 0;

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	/* read all bytes from this packet */
	if (likely(csr & S3C_UDC_EP0_RX_SUCCESS)) {
		count = usb_read((u32) S3C_UDC_BYTE_READ_CNT_REG, ep_index(ep));
		if (csr & S3C_UDC_EP0_LWO)
			bytes = count * 2 -1;
		else
			bytes = count * 2;
			
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
		u16 byte = (u16) __raw_readl(fifo);

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
			bufferspace = bufferspace - 2;
		}
	}
	
	__raw_writel(S3C_UDC_EP0_RX_SUCCESS, S3C_UDC_EP0_STATUS_REG);/* clear */

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
	DEBUG_EP0("%s: address=%d, S3C_UDC_FUNC_ADDR_REG=0x%x\n",
		__FUNCTION__, address, __raw_readl(S3C_UDC_FUNC_ADDR_REG));
	dev->usb_address = address;
}

/*
 * DATA_STATE_RECV (OUT_PKT_RDY)
 */
static int first_time = 1;

static void s3c_ep0_read(struct s3c_udc *dev)
{
	struct s3c_request *req;
	struct s3c_ep *ep = &dev->ep[0];
	int ret;

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct s3c_request, queue);
	else {
		DEBUG_EP0("%s: ---> BUG\n", __FUNCTION__);
		BUG();	//logic ensures		-jassi
		return;
	}
	
	DEBUG_EP0("%s: req.length = 0x%x, req.actual = 0x%x\n",
		__FUNCTION__, req->req.length, req->req.actual);
	
	if(req->req.length == 0) {
		__raw_writel(S3C_UDC_EP0_RX_SUCCESS, S3C_UDC_EP0_STATUS_REG);
		dev->ep0state = WAIT_FOR_SETUP;
		first_time = 1;
		done(ep, req, 0);
		return;
	}

	if(!req->req.actual && first_time){	//for SetUp packet
		__raw_writel(S3C_UDC_EP0_RX_SUCCESS, S3C_UDC_EP0_STATUS_REG);
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
static void s3c_ep0_setup(struct s3c_udc *dev, u32 csr)
{
	struct s3c_ep *ep = &dev->ep[0];
	struct usb_ctrlrequest ctrl;
	int i, bytes, is_in;

	DEBUG_SETUP("%s: csr = 0x%x\n", __FUNCTION__, csr);

	/* Nuke all previous transfers */
	nuke(ep, -EPROTO);

	/* read control req from fifo (8 bytes) */
	bytes = s3c_fifo_read(ep, (u16 *)&ctrl, 8);

	DEBUG_SETUP("Read CTRL REQ %d bytes\n", bytes);
	DEBUG_SETUP("CTRL.bRequestType = 0x%x (is_in %d)\n", ctrl.bRequestType,
		    ctrl.bRequestType & USB_DIR_IN);
	DEBUG_SETUP("CTRL.bRequest = 0x%x\n", ctrl.bRequest);
	DEBUG_SETUP("CTRL.wLength = 0x%x\n", ctrl.wLength);
	DEBUG_SETUP("CTRL.wValue = 0x%x (%d)\n", ctrl.wValue, ctrl.wValue >> 8);
	DEBUG_SETUP("CTRL.wIndex = 0x%x\n", ctrl.wIndex);

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

			DEBUG_SETUP("%s: USB_REQ_SET_ADDRESS (%d)\n",
					__FUNCTION__, ctrl.wValue);
			udc_set_address(dev, ctrl.wValue);
			return;
		default:
			DEBUG_SETUP("%s: DFAULT\n", __FUNCTION__);
			break;
	}

	if (likely(dev->driver)) {
		/* device-2-host (IN) or no data setup command,
		 * process immediately */
		spin_unlock(&dev->lock);
		i = dev->driver->setup(&dev->gadget, &ctrl);
		spin_lock(&dev->lock);

		if (i < 0) {
			/* setup processing failed, force stall */
			printk("%s:  --> ERROR: gadget setup FAILED (stalling), setup returned %d\n",
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
	struct s3c_ep *ep = &dev->ep[0];
	u32 csr;

 	csr = __raw_readl(S3C_UDC_EP0_STATUS_REG);

	DEBUG_EP0("%s: S3C_UDC_EP0_STATUS_REG = 0x%x\n", __FUNCTION__, csr);

	/*
	 * if SENT_STALL is set
	 *      - clear the SENT_STALL bit
	 */
	if (csr & S3C_UDC_EP0_STALL) {
		printk("%s: S3C_UDC_EP0_STALL\n", __FUNCTION__);
		__raw_writel(S3C_UDC_EP0_STALL, S3C_UDC_EP0_STATUS_REG); /* clear */
		nuke(ep, -ECONNABORTED);
		dev->ep0state = WAIT_FOR_SETUP;
		return;
	}
		
	if (csr & S3C_UDC_EP0_TX_SUCCESS) {
		DEBUG_EP0("%s: EP0_TX_SUCCESS \n", __FUNCTION__);
		__raw_writel(S3C_UDC_EP0_TX_SUCCESS, S3C_UDC_EP0_STATUS_REG); /* clear */
	}

	if (csr & S3C_UDC_EP0_RX_SUCCESS) {
		if (dev->ep0state == WAIT_FOR_SETUP) {
			DEBUG_EP0("%s: WAIT_FOR_SETUP\n", __FUNCTION__);
			s3c_ep0_setup(dev, csr);
		} else {
			DEBUG_EP0("%s: strange state!!(state = %s)\n",
				__FUNCTION__, state_names[dev->ep0state]);
		}
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
	/*fram count number [10:0]*/
	u32 frame = __raw_readl(S3C_UDC_FRAME_NUM_REG);
	DEBUG("%s: %p\n", __FUNCTION__, _gadget);
	return (frame & 0x7ff);
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
		  .fifo = (u32) S3C_UDC_EP0_FIFO_REG,
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
		  .fifo = (u32) S3C_UDC_EP1_FIFO_REG,
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
		  .fifo = (u32) S3C_UDC_EP2_FIFO_REG,
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
		  .fifo = (u32) S3C_UDC_EP3_FIFO_REG,
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
		  .fifo = (u32) S3C_UDC_EP4_FIFO_REG,
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
		  .fifo = (u32) S3C_UDC_EP5_FIFO_REG,
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
		  .fifo = (u32) S3C_UDC_EP6_FIFO_REG,
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
		  .fifo = (u32) S3C_UDC_EP7_FIFO_REG,
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
		  .fifo = (u32) S3C_UDC_EP8_FIFO_REG,
		  },
};

/*
 * 	probe - binds to the platform device
 */
static struct clk	*udc_clock = NULL;

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

	udc_reinit(dev);

	udc_clock = clk_get(&pdev->dev, "usb-device");
	if (udc_clock == NULL) {
		printk(KERN_INFO "failed to find usb-device clock source\n");
		return -ENOENT;
	}
	clk_enable(udc_clock);

	local_irq_disable();

	/* irq setup after old hardware state is cleaned up */
	retval =
	    request_irq(IRQ_USBD, s3c_udc_irq, IRQF_DISABLED, driver_name,
			dev);
	if (retval != 0) {
		DEBUG(KERN_ERR "%s: can't get irq %i, err %d\n", driver_name,
		      IRQ_USBD, retval);
		return -EBUSY;
	}

	disable_irq(IRQ_USBD);
	local_irq_enable();
	create_proc_files();

	return retval;
}

static int s3c_udc_remove(struct platform_device *pdev)
{
	struct s3c_udc *dev = platform_get_drvdata(pdev);

	DEBUG("%s: %p\n", __FUNCTION__, pdev);

	udc_disable(dev);
	remove_proc_files();
	usb_gadget_unregister_driver(dev->driver);

	free_irq(IRQ_USBD, dev);

	platform_set_drvdata(pdev, 0);
	
	the_controller = 0;

	return 0;
}

/*-------------------------------------------------------------------------*/
static struct platform_driver s3c_udc_driver = {
	.probe		= s3c_udc_probe,
	.remove		= s3c_udc_remove,
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
	   printk("Loaded %s version %s\n", driver_name, DRIVER_VERSION);

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
