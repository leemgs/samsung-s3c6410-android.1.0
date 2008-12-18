/* spi-s3c2443.c
 *
 * Copyright (C) 2006 Samsung Electronics Co. Ltd.
 *
 * S3C2443 SPI Controller
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
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/arch-s3c2410/regs-s3c-clock.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-spi.h>
#include <asm/dma.h>

#include "spi-dev.h"
#include "hspi-s3c24xx.h"

#undef debug
//#define debug

#ifdef debug
#define DBG(x...)       printk(x)
#define DEBUG	printk("%s :: %d\n",__FUNCTION__,__LINE__)
void print_reg(struct s3c_spi *spi)
{
	printk("CH_CFG 	= 0x%08x\n",readl(spi->regs + S3C_CH_CFG));
	printk("CLK_CFG = 0x%08x\n",readl(spi->regs + S3C_CLK_CFG));
	printk("MODE_CFG = 0x%08x\n",readl(spi->regs + S3C_MODE_CFG));
	printk("SLAVE_CFG = 0x%08x\n",readl(spi->regs + S3C_SLAVE_SEL));
	printk("INT_EN 	= 0x%08x\n",readl(spi->regs + S3C_SPI_INT_EN));
	printk("SPI_STATUS = 0x%08x\n",readl(spi->regs + S3C_SPI_STATUS));
	printk("PACKET_CNT = 0x%08x\n",readl(spi->regs + S3C_PACKET_CNT));
	printk("PEND_CLR = 0x%08x\n",readl(spi->regs + S3C_PENDING_CLR));
}
#else
#define DEBUG
#define DBG(x...)       do { } while (0)
void print_reg(struct s3c_spi *spi)
{
}
#endif

static void s3c_spi_free(struct s3c_spi *spi)
{
	DEBUG;

	if (spi->clk != NULL && !IS_ERR(spi->clk)) {
		clk_disable(spi->clk);
		clk_put(spi->clk);
		spi->clk = NULL;
	}

	if (spi->regs != NULL) {
		iounmap(spi->regs);
		spi->regs = NULL;
	}

	if (spi->ioarea != NULL) {
		release_resource(spi->ioarea);
		kfree(spi->ioarea);
		spi->ioarea = NULL;
	}
}

static int s3c_spi_hw_init(struct s3c_spi *spi)
{

	/* program defaults into the registers */
	writel(readl(S3C2443_SCLKCON)|(1<<14), S3C2443_SCLKCON);
        writel(readl(S3C2443_PCLKCON)|(1<<6), S3C2443_PCLKCON);

	writel(readl(S3C24XX_MISCCR)|S3C24XX_MISCCR_SPISEL, S3C24XX_MISCCR);

	/* initialize the gpio */
	s3c2410_gpio_cfgpin(S3C2410_GPE11, S3C2410_GPE11_SPIMISO0);
	s3c2410_gpio_cfgpin(S3C2410_GPE12, S3C2410_GPE12_SPIMOSI0);
	s3c2410_gpio_cfgpin(S3C2410_GPE13, S3C2410_GPE13_SPICLK0);

	s3c2410_gpio_cfgpin(S3C2410_GPL13, S3C2410_GPL13_SS0);
	s3c2410_gpio_cfgpin(S3C2410_GPL14, S3C2410_GPL14_SS1);

	s3c2410_gpio_cfgpin(S3C2410_GPL11, S3C2410_GPL11_SPIMOSI1);
	s3c2410_gpio_cfgpin(S3C2410_GPL12, S3C2410_GPL12_SPIMISO1);
	s3c2410_gpio_cfgpin(S3C2410_GPL10, S3C2410_GPL10_SPICLK1);

#ifdef CONFIG_SPICLK_PCLK
	/*Enable PCLK into the HS SPI*/
	writel(readl(S3C2443_PCLKCON)|(1<<6), S3C2443_PCLKCON);

	clk_enable(spi->clk);

#elif defined CONFIG_SPICLK_EPLL
	/* implemetation when use EPLL clock */
	writel(0x800, S3C2443_LOCKCON1);
	writel( (readl( S3C2443_CLKSRC ) | (1 << 6) ), S3C2443_CLKSRC);  // EPLL Output

#if defined(CONFIG_CPU_S3C2443)
	writel((40<<16)|(1<<8)|(1<<0) ,S3C2443_EPLLCON);//96MHz
#elif defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)
	writel((48<<16)|(3<<8)|(1<<0) ,S3C2443_EPLLCON);//96MHz
#else
# error Any CPU Type is not selected
#endif

	writel( readl(S3C2443_EPLLCON)& (~(1<<24)) , S3C2443_EPLLCON );  //EPLL On
	writel(( readl(S3C2443_CLKDIV1) & (~(0x3<<24))) | (0x0 << 24) , S3C2443_CLKDIV1 ); // Epll Ratio is 1
	writel(( readl(S3C24XX_MISCCR) & (~(0x7<<8))) | (0x1 << 8) , S3C24XX_MISCCR );

	/* Epll, prescaler = 1					*/
	/* clock =  ( clock source / (2 * ( prescaler + 1)))	*/
	writel(0x501, spi->regs + S3C_CLK_CFG); 	//Use EPLL Clock and Clock On Prescaler 1
#else
#error you must define correct confige file.
#endif
	/* hspi software restet */

	writeb(readb(spi->regs + S3C_CH_CFG) | (1<<5), spi->regs + S3C_CH_CFG);
	writeb(readb(spi->regs + S3C_CH_CFG) & (~(1<<5)), spi->regs + S3C_CH_CFG);

	DEBUG;

	return 0;
}

static int s3c_spi_dma_init(struct s3c_spi *spi, int mode)
{
	DEBUG;

	// TX
	if (mode == 0) {
		s3c2410_dma_devconfig(spi->dma, S3C2410_DMASRC_MEM, S3C_SPI_DMA_HWCFG, S3C_SPI_TX_DATA_REG);
		s3c2410_dma_config(spi->dma, S3C_DMA_XFER_BYTE, S3C_DCON_SPI1);

	}

	// RX
	if (mode == 1) {
		s3c2410_dma_devconfig(spi->dma, S3C2410_DMASRC_HW, S3C_SPI_DMA_HWCFG, S3C_SPI_RX_DATA_REG);
		s3c2410_dma_config(spi->dma, S3C_DMA_XFER_BYTE, S3C_DCON_SPI1);

	}

	s3c2410_dma_setflags(spi->dma, S3C2410_DMAF_AUTOSTART);

	return 0;
}

static inline void s3c_spi_write_fifo(struct s3c_spi *spi)
{
	u32 wdata = 0;

	if (spi->msg->wbuf) {
		wdata = spi->msg->wbuf[spi->msg_ptr++];
	} else {
		spi->msg_ptr++;
		wdata = 0xff;
	}

	DBG("wdata = %x\n",wdata);
	writel(wdata, spi->regs + S3C_SPI_TX_DATA);
}

/* s3c_spi_master_complete
 *
 * complete the message and wake up the caller, using the given return code,
 * or zero to mean ok.
*/
static inline void s3c_spi_master_complete(struct s3c_spi *spi, int ret)
{
	DEBUG;

	spi->msg_ptr = 0;
	spi->msg_rd_ptr = 0;
	spi->msg->flags = 0;
	spi->msg = NULL;
	spi->msg_idx ++;
	spi->msg_num = 0;

	writel(0xff, spi->regs + S3C_PENDING_CLR);


	if (ret)
		spi->msg_idx = ret;
}

static int s3c_spi_done(struct s3c_spi *spi)
{
	u32 spi_clkcfg;
	DEBUG;

	spi_clkcfg = readl( spi->regs + S3C_CLK_CFG);
	spi_clkcfg &= SPI_ENCLK_DISABLE;
	writel( spi_clkcfg , spi->regs + S3C_CLK_CFG);

	return 0;

}

static inline void s3c_spi_stop(struct s3c_spi *spi, int ret)
{
	DEBUG;

	writel(0x0, spi->regs + S3C_SPI_INT_EN);
	writel(0x1f, spi->regs + S3C_PENDING_CLR);
	writel(0x0, spi->regs + S3C_CH_CFG);

	s3c_spi_done(spi);
	spi->state = STATE_IDLE;
	s3c_spi_master_complete(spi, ret);
	print_reg(spi);
        up(&spi->sem);
}

void s3c_spi_dma_cb(struct s3c2410_dma_chan *dma_ch, void *buf_id,
        int size, enum s3c2410_dma_buffresult result)
{
	struct s3c_spi *spi = (struct s3c_spi *)buf_id;
	unsigned long status = 0;
	DEBUG;

	status = readl(spi->regs + S3C_SPI_STATUS);

	pr_debug("DMA call back\n");

	if (spi->msg->wbuf)
		while (!(readl(spi->regs +S3C_SPI_STATUS) & SPI_STUS_TX_DONE)) {}

	s3c_spi_stop(spi, status);
}

/* s3c_spi_message_start
 *
 * configure the spi controler and transmit start of a message onto the bus
*/
static void s3c_spi_message_start(struct s3c_spi *spi)
{
	struct spi_msg *msg = spi->msg;

	u32 spi_chcfg = 0, spi_slavecfg, spi_inten= 0, spi_packet=0;

	u8 prescaler = 1;
	u32 spi_clkcfg = 0, spi_modecfg = 0 ;

	DEBUG;

	/* initialise the spi controller */
	s3c_spi_hw_init(spi);

	/* 1. Set transfer type (CPOL & CPHA set) */
	spi_chcfg = SPI_CH_RISING | SPI_CH_FORMAT_A;

	if (spi->msg->flags & SPI_M_MODE_MASTER) {
		spi_chcfg |= SPI_CH_MASTER;
	} else if(spi->msg->flags & SPI_M_MODE_SLAVE){
		spi_chcfg |= SPI_CH_SLAVE;
	}

	writel( spi_chcfg , spi->regs + S3C_CH_CFG);

	/* 2. Set clock configuration register */
	spi_clkcfg = SPI_ENCLK_ENABLE;

#if defined CONFIG_SPICLK_PCLK
	spi_clkcfg |= SPI_CLKSEL_PCLK;
#elif defined CONFIG_SPICLK_EPLL
	spi_clkcfg |= SPI_CLKSEL_ECLK;
#else
#error you must define correct confige file.
#endif
	writel( spi_clkcfg , spi->regs + S3C_CLK_CFG);

	spi_clkcfg = readl( spi->regs + S3C_CLK_CFG);

	/* SPI clockout = clock source / (2 * (prescaler +1)) */
	spi_clkcfg |= prescaler;
	writel( spi_clkcfg , spi->regs + S3C_CLK_CFG);

	/* 3. Set SPI MODE configuration register */
#ifdef CONFIG_WORD_TRANSIZE
	spi_modecfg = SPI_MODE_CH_TSZ_WORD;
#else
	spi_modecfg = SPI_MODE_CH_TSZ_BYTE;
#endif
	spi_modecfg |= SPI_MODE_TXDMA_OFF| SPI_MODE_SINGLE| SPI_MODE_RXDMA_OFF;

	if (msg->flags & SPI_M_DMA_MODE) {
		spi_modecfg |= SPI_MODE_TXDMA_ON| SPI_MODE_RXDMA_ON;
	}

	if (msg->wbuf)
		spi_modecfg |= ( 0x3f << 5); /* Tx FIFO trigger level in INT mode */
	if (msg->rbuf)
		spi_modecfg |= ( 0x3f << 11); /* Rx FIFO trigger level in INT mode */

	spi_modecfg |= ( 0x3ff << 19);
	writel(spi_modecfg, spi->regs + S3C_MODE_CFG);

	/* 4. Set SPI INT_EN register */

	if (msg->wbuf)
		spi_inten = SPI_INT_TX_FIFORDY_EN|SPI_INT_TX_UNDERRUN_EN|SPI_INT_TX_OVERRUN_EN;
	if (msg->rbuf){
		spi_inten = SPI_INT_RX_FIFORDY_EN|SPI_INT_RX_UNDERRUN_EN|SPI_INT_RX_OVERRUN_EN|SPI_INT_TRAILING_EN;
	}
	writel(spi_inten, spi->regs + S3C_SPI_INT_EN);

	writel(0x1f, spi->regs + S3C_PENDING_CLR);

	/* 5. Set Packet Count configuration register */
	spi_packet = SPI_PACKET_CNT_EN;
	spi_packet |= 0xffff;
	writel(spi_packet, spi->regs + S3C_PACKET_CNT);

	/* 6. Set Tx or Rx Channel on */
	spi_chcfg = readl(spi->regs + S3C_CH_CFG);

	spi_chcfg |= SPI_CH_TXCH_OFF | SPI_CH_RXCH_OFF;

	if (msg->wbuf)
		spi_chcfg |= SPI_CH_TXCH_ON;
	if (msg->rbuf)
		spi_chcfg |= SPI_CH_RXCH_ON;

	writel(spi_chcfg, spi->regs + S3C_CH_CFG);

	if (msg->flags & SPI_M_DMA_MODE) {

		if (msg->wbuf)
			spi->dma = DMACH_SPI_TX;
		if (msg->rbuf)
			spi->dma = DMACH_SPI_RX;

		if (s3c2410_dma_request(spi->dma, &s3c2443spi_dma_client, NULL)) {
			printk(KERN_WARNING  "unable to get DMA channel.\n" );
		}

		s3c2410_dma_set_buffdone_fn(spi->dma, s3c_spi_dma_cb);
		s3c2410_dma_set_opfn(spi->dma, NULL);


		if (msg->wbuf)
			s3c_spi_dma_init(spi, 0);
		if (msg->rbuf)
			s3c_spi_dma_init(spi, 1);

		s3c2410_dma_enqueue(spi->dma, (void *) spi, spi->dmabuf_addr, spi->msg->len);
	}
	/* 7. Set nSS low to start Tx or Rx operation */
	spi_slavecfg = readl(spi->regs + S3C_SLAVE_SEL);
	spi_slavecfg &= SPI_SLAVE_SIG_ACT;
	writel(spi_slavecfg, spi->regs + S3C_SLAVE_SEL);

	print_reg(spi);
}

/* is_msgend
 *
 * returns TRUE if we reached the end of the current message
*/

static inline int tx_msgend(struct s3c_spi *spi)
{
	return spi->msg_ptr >= spi->msg->len;
}

static inline int rx_msgend(struct s3c_spi *spi)
{
	return spi->msg_rd_ptr >= spi->msg->len;
}

/* spi_s3c_irq_nextbyte
 *
 * process an interrupt and work out what to do
 */
static void spi_s3c_irq_nextbyte(struct s3c_spi *spi, unsigned long spsta)
{
	DEBUG;
	DBG("spi->state = %d \n",spi->state);
	switch (spi->state) {
	case STATE_IDLE:
			DBG("%s: called in STATE_IDLE\n", __FUNCTION__);
		break;

	case STATE_STOP:
		udelay(200);
		s3c_spi_stop(spi, 0);
			DBG("%s: called in STATE_STOP\n", __FUNCTION__);
		break;

	case STATE_XFER_TX:
			DEBUG;
			print_reg(spi);
			DBG("msg_ptr = 0x%x, len = 0x%x \n", spi->msg_ptr ,spi->msg->len);
			while(!(tx_msgend(spi)))
			s3c_spi_write_fifo(spi);
			DEBUG;
			print_reg(spi);
			spi->state = STATE_STOP;
		break;
	case STATE_XFER_RX:
		DEBUG;
			print_reg(spi);
			DBG("msg_rd_ptr = 0x%x, len = 0x%x \n", spi->msg_rd_ptr ,spi->msg->len);
			while(!(rx_msgend(spi))){
		spi->msg->rbuf[spi->msg_rd_ptr++] = readl(spi->regs + S3C_SPI_RX_DATA);
				DBG("msg_rd_ptr = 0x%x, len = 0x%x \n", spi->msg_rd_ptr ,spi->msg->len);
				DBG("msg_rbuf = 0x%x\n", spi->msg->rbuf[spi->msg_rd_ptr - 1]);
		}
			DBG("msg_rd_ptr = 0x%x, len = 0x%x \n", spi->msg_rd_ptr ,spi->msg->len);
		DEBUG;
			print_reg(spi);
			s3c_spi_stop(spi, 0);
		break;
	default:
		dev_err(spi->dev, "%s: called with Invalid option\n", __FUNCTION__);
	}

	return;
}

/* s3c_spi_irq
 *
 * top level IRQ servicing routine
*/
static irqreturn_t s3c_spi_irq(int irqno, void *dev_id)
{
	struct s3c_spi *spi = dev_id;
	unsigned long spi_sts;

	spi_sts = readl(spi->regs + S3C_SPI_STATUS);

	if (spi_sts & SPI_STUS_RX_OVERRUN_ERR) {
		printk("hspi : Rx overrun error detected\n");
	}

	if (spi_sts & SPI_STUS_RX_UNDERRUN_ERR) {
		printk("hspi : Rx underrun error detected\n");
	}

	if (spi_sts & SPI_STUS_TX_OVERRUN_ERR) {
		printk("hspi : Tx overrun error detected\n");
	}

	if (spi_sts & SPI_STUS_TX_UNDERRUN_ERR) {
		printk("hspi : Tx underrun error detected\n");
	}

	/* pretty much this leaves us with the fact that we've
	 * transmitted or received whatever byte we last sent */
	spi_s3c_irq_nextbyte(spi, spi_sts);

	return IRQ_HANDLED;
}

static int s3c_spi_doxfer(struct s3c_spi *spi, struct spi_msg msgs[], int num)
{
	int ret;

	spin_lock_irq(&spi->lock);

	spi->msg     	= msgs;
	spi->msg_num 	= num;
	spi->msg_ptr 	= 0;
	spi->msg_rd_ptr = 0;
	spi->msg_idx 	= 0;

	if (spi->msg->flags & SPI_M_DMA_MODE) {
		spi->dmabuf_addr = spi->spidev.dmabuf;
		pr_debug("spi->dmabuf_addr = 0x%x\n",spi->dmabuf_addr);
	}

	if (spi->msg->wbuf) {
	DEBUG;
		spi->state   = STATE_XFER_TX;
	} else if (spi->msg->rbuf) {
	DEBUG;
		spi->state   = STATE_XFER_RX;
	} else {
		dev_err(spi->dev,"Unknown functionality \n");
		return -ESRCH;
	}
	s3c_spi_message_start(spi);
	DEBUG;

	if (down_interruptible(&spi->sem))
		return -EINTR;

	DEBUG;
	spin_unlock_irq(&spi->lock);

	DEBUG;
	ret = spi->msg_idx;

 	return ret;
}


/* s3c_spi_xfer
 *
 * first port of call from the spi bus code when an message needs
 * transfering across the spi bus.
*/
static int s3c_spi_xfer(struct spi_dev *spi_dev,
			struct spi_msg msgs[], int num)
{
	struct s3c_spi *spi = (struct s3c_spi *)spi_dev->algo_data;
	int retry;
	int ret;

	for (retry = 0; retry < spi_dev->retries; retry++) {

		ret = s3c_spi_doxfer(spi, msgs, num);

		print_reg(spi);
		if (ret != -EAGAIN)
			return ret;
		printk("Retrying transmission (%d)\n", retry);

		udelay(100);
	}

	DEBUG;
	return -EREMOTEIO;
}

static int s3c_spi_close(struct spi_dev *spi_dev)
{
	struct s3c_spi *spi = (struct s3c_spi *)spi_dev->algo_data;
	u32 spi_clkcfg;
	DEBUG;

	spi_clkcfg = readl( spi->regs + S3C_CLK_CFG);
	spi_clkcfg &= SPI_ENCLK_DISABLE;
	writel( spi_clkcfg , spi->regs + S3C_CLK_CFG);

	/* Buffer Clear after finish xfer */
	writel( 0x20, spi->regs + S3C_CH_CFG);
	writel( 0x0, spi->regs + S3C_CH_CFG);

	return 0;

}


/* spi bus registration info */

static struct spi_algorithm s3c_spi_algorithm = {
	.name			= "S3C2443-spi-algorithm",
	.master_xfer		= s3c_spi_xfer,
	.close			= s3c_spi_close,
};

static struct s3c_spi s3c_spi[2] = {
	[0] = {
		.lock	= SPIN_LOCK_UNLOCKED,
		.spidev	= {
			.algo			= &s3c_spi_algorithm,
			.retries		= 2,
			.timeout		= 5,
		}
	},
	[1] = {
		.lock	= SPIN_LOCK_UNLOCKED,
		.spidev	= {
			.algo			= &s3c_spi_algorithm,
			.retries		= 2,
			.timeout		= 5,
		}
	},
};


/* s3c_spi_probe
 *
 * called by the bus driver when a suitable device is found
*/

static int s3c_spi_probe(struct platform_device *pdev)
{
	struct s3c_spi *spi = &s3c_spi[pdev->id];
	struct resource *res;
	int ret;

	/* find the clock and enable it */
	sema_init(&spi->sem, 0);
	spi->nr = pdev->id;
	spi->dev = &pdev->dev;

	spi->clk = clk_get(&pdev->dev, "spi");

	if (IS_ERR(spi->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = -ENOENT;
		goto out;
	}

	/* map the registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto out;
	}

	spi->ioarea = request_mem_region(res->start, (res->end - res->start) + 1, pdev->name);

	if (spi->ioarea == NULL) {
		dev_err(&pdev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto out;
	}

	printk(KERN_ALERT "resource start : %x\n",res->start);

	spi->regs = ioremap(res->start, (res->end - res->start) + 1);

	if (spi->regs == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto out;
	}

	printk(KERN_ALERT "hspi registers %p (%p, %p)\n", spi->regs, spi->ioarea, res);

	/* setup info block for the spi core */

	spi->spidev.algo_data = spi;
	spi->spidev.dev.parent = &pdev->dev;
	spi->spidev.minor = spi->nr;
	init_MUTEX(&spi->spidev.bus_lock);

	/* find the IRQ for this unit (note, this relies on the init call to
	 * ensure no current IRQs pending
	 */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (res == NULL) {
		printk("hspi cannot find IRQ\n");
		ret = -ENOENT;
		goto out;
	}

	ret = request_irq(res->start, s3c_spi_irq, SA_INTERRUPT,
			pdev->name, spi);

	if (ret != 0) {
		printk("hspi cannot claim IRQ\n");
		goto out;
	}

	ret = spi_attach_spidev(&spi->spidev);

	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add adapter to spi core\n");
		goto out;
	}

	dev_set_drvdata(&pdev->dev, spi);

	dev_info(&pdev->dev, "%s: S3C SPI adapter\n", spi->dev->bus_id);

	printk("%s: S3C SPI adapter\n", spi->dev->bus_id);

out:
	if (ret < 0)
		s3c_spi_free(spi);

	return ret;
}

/* s3c_spi_remove
 *
 * called when device is removed from the bus
*/
static int s3c_spi_remove(struct platform_device *pdev)
{
	struct s3c_spi *spi = dev_get_drvdata(&pdev->dev);

	DEBUG;

	if (spi != NULL) {
		spi_detach_spidev(&spi->spidev);
		s3c_spi_free(spi);
		dev_set_drvdata(&pdev->dev, NULL);
	}

	return 0;
}

#ifdef CONFIG_PM
static int s3c_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct s3c_spi *hw = platform_get_drvdata(pdev);
	clk_disable(hw->clk);
	return 0;
}

static int s3c_spi_resume(struct platform_device *pdev)
{
	struct s3c_spi *hw = platform_get_drvdata(pdev);
	clk_enable(hw->clk);
	return 0;
}
#else
#define s3c_spi_suspend NULL
#define s3c_spi_resume  NULL
#endif

/* device driver for platform bus bits */
static struct platform_driver s3c_spi_driver = {
	.probe		= s3c_spi_probe,
	.remove		= s3c_spi_remove,
#ifdef CONFIG_PM
	.suspend	= s3c_spi_suspend,
	.resume		= s3c_spi_resume,
#endif
	.driver		= {
		.name	= "s3c2410-spi",
		.owner	= THIS_MODULE,
		.bus    = &platform_bus_type,
	},
};

static int __init s3c_spi_driver_init(void)
{
	printk(KERN_INFO "S3C2443 HSPI Driver \n");

	return platform_driver_register(&s3c_spi_driver);
}

static void __exit s3c_spi_driver_exit(void)
{
	platform_driver_unregister(&s3c_spi_driver);
}

module_init(s3c_spi_driver_init);
module_exit(s3c_spi_driver_exit);

MODULE_DESCRIPTION("S3C2443 SPI Bus driver");
MODULE_AUTHOR("Ryu Euiyoul<steven.ryu@samsung.com>");
MODULE_LICENSE("GPL");
