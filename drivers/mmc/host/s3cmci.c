/*
 *  linux/drivers/mmc/s3cmci.c - Samsung S3C SDI Interface driver
 *
 * $Id: s3cmci.c,v 1.1 2008/03/03 00:40:28 ihlee215 Exp $
 *
 *  Copyright (C) 2004 Thomas Kleffel, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/err.h>

#include <asm/dma.h>
#include <asm/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#include <asm/sizes.h>
#include <asm/mach/mmc.h>

#include <asm/arch/registers.h>
#include <asm/arch/clock.h>
#include <asm/arch/dma.h>

#ifdef CONFIG_S3CMCI_DEBUG
#define DBG(x...)       printk(x)
#else
#define DBG(x...)       do { } while (0)
#endif

#include "s3cmci.h"

#ifndef MHZ
#define MHZ (1000*1000)
#endif

#define DRIVER_NAME "s3c-sdi"
#define PFX DRIVER_NAME ": "

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

typedef enum {
	DMAP_READ,
	DMAP_WRITE,
} eDMAPurpose_t;

#ifdef CONFIG_S3CMCI_DEBUG
uint cmd_rec[40];
uint cmd_idx = 0;
#endif

static struct s3c_dma_client s3c_sdi_dma_client = {
	.name		= "s3c-sdi-dma",
};

/*
 * ISR for SDI Interface IRQ
 * Communication between driver and ISR works as follows:
 *   host->mrq 			points to current request
 *   host->complete_what	tells the ISR when the request is considered done
 *     COMPLETION_CMDSENT	  when the command was sent
 *     COMPLETION_RSPFIN          when a response was received
 *     COMPLETION_XFERFINISH	  when the data transfer is finished
 *     COMPLETION_XFERFINISH_RSPFIN both of the above.
 *   host->complete_request	is the completion-object the driver waits for
 *
 * 1) Driver sets up host->mrq and host->complete_what
 * 2) Driver prepares the transfer
 * 3) Driver enables interrupts
 * 4) Driver starts transfer
 * 5) Driver waits for host->complete_rquest
 * 6) ISR checks for request status (errors and success)
 * 6) ISR sets host->mrq->cmd->error and host->mrq->data->error
 * 7) ISR completes host->complete_request
 * 8) ISR disables interrupts
 * 9) Driver wakes up and takes care of the request
 */

static irqreturn_t s3c_sdi_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c_sdi_host *host;
	struct mmc_request *mrq;
	u32 sdi_csta, sdi_dsta, sdi_fsta;
	u32 sdi_cclear, sdi_dclear;
	unsigned long iflags;

	host = (struct s3c_sdi_host *)dev_id;

	/* Check for things not supposed to happen */
	if (!host)
		return IRQ_HANDLED;

	spin_lock_irqsave( &host->complete_lock, iflags);

	if ( host->complete_what==COMPLETION_NONE ) {
		goto clear_imask;
	}

	if (!host->mrq) {
		goto clear_imask;
	}

	mrq = host->mrq;

	sdi_csta = readl(host->base + S3C_SDICSTA);
	sdi_dsta = readl(host->base + S3C_SDIDSTA);
	sdi_fsta = readl(host->base + S3C_SDIFSTA);
	sdi_cclear = 0;
	sdi_dclear = 0;

	if (sdi_csta & S3C_SDICSTA_CMDTOUT) {
		mrq->cmd->error = MMC_ERR_TIMEOUT;
		DBG("cmd timeout\n");
		goto transfer_closed;
	}

	if (sdi_csta & S3C_SDICSTA_CMDSENT) {
		if (host->complete_what == COMPLETION_CMDSENT) {
			mrq->cmd->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		sdi_cclear |= S3C_SDICSTA_CMDSENT;
	}

	if (sdi_csta & S3C_SDICSTA_CRCFAIL) {
		if (mrq->cmd->flags & MMC_RSP_CRC) {
			DBG("cmd badcrc\n");
			mrq->cmd->error = MMC_ERR_BADCRC;
			goto transfer_closed;
		}

		sdi_cclear |= S3C_SDICSTA_CRCFAIL;
	}

	if (sdi_csta & S3C_SDICSTA_RSPFIN) {
		if (host->complete_what == COMPLETION_RSPFIN) {
			mrq->cmd->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN) {
			mrq->cmd->error = MMC_ERR_NONE;
			host->complete_what = COMPLETION_XFERFINISH;
		}

		sdi_cclear |= S3C_SDICSTA_RSPFIN;
	}

	if (sdi_fsta & S3C_SDIFSTA_FIFOFAIL) {
		printk(PFX "Unchartererd Waters ....\n");
		mrq->cmd->error = MMC_ERR_NONE;
		mrq->data->error = MMC_ERR_FIFO;
		goto transfer_closed;
	}

	if (sdi_dsta & S3C_SDIDSTA_RXCRCFAIL) {
		mrq->cmd->error = MMC_ERR_NONE;
		mrq->data->error = MMC_ERR_BADCRC;
		printk(PFX "crc error - RXCRCFAIL\n");
		goto transfer_closed;
	}

	if (sdi_dsta & S3C_SDIDSTA_CRCFAIL) {
		mrq->cmd->error = MMC_ERR_NONE;
		mrq->data->error = MMC_ERR_BADCRC;
		printk(PFX "crc error - TXCRCFAIL\n");
		goto transfer_closed;
	}

	if (sdi_dsta & S3C_SDIDSTA_DATATIMEOUT) {
		mrq->cmd->error = MMC_ERR_NONE;
		mrq->data->error = MMC_ERR_TIMEOUT;
#ifdef CONFIG_S3CMCI_DEBUG
		printk(PFX "timeout(%d) - DSTA: %08x\n", mrq->cmd->opcode, readl(host->base + S3C_SDIDSTA));
		{
			int i;
			printk("cmd_idx: %d\n", cmd_idx);
			for (i=0; i<40; i++) {
				printk("%02d: %d\n", i, cmd_rec[i]);
			}
		}
#endif
		goto transfer_closed;
	}

	if (sdi_dsta & S3C_SDIDSTA_XFERFINISH) {
		if (host->complete_what == COMPLETION_XFERFINISH) {
			mrq->cmd->error = MMC_ERR_NONE;
			mrq->data->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN) {
			mrq->data->error = MMC_ERR_NONE;
			host->complete_what = COMPLETION_RSPFIN;
		}

		sdi_dclear |= S3C_SDIDSTA_XFERFINISH;
	}

	writel(sdi_cclear, host->base + S3C_SDICSTA);
	writel(sdi_dclear, host->base + S3C_SDIDSTA);

	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ still waiting.\n");
	return IRQ_HANDLED;


transfer_closed:
	host->complete_what = COMPLETION_NONE;
	complete(&host->complete_request);
	writel(0, host->base + S3C_SDIIMSK);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ transfer closed.\n");
	return IRQ_HANDLED;

clear_imask:
	writel(0, host->base + S3C_SDIIMSK);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ clear imask.\n");
	return IRQ_HANDLED;
}

static void s3c_sdi_check_status(unsigned long data)
{
        struct s3c_sdi_host *host = (struct s3c_sdi_host *)data;

	s3c_sdi_irq(0, host, NULL);
}

/*
 * ISR for the CardDetect Pin
 */
static irqreturn_t s3c_sdi_irq_cd(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c_sdi_host *host = (struct s3c_sdi_host *)dev_id;
	mmc_detect_change(host->mmc, 0);

	DBG(PFX "mmc device mode is changed.\n");
	return IRQ_HANDLED;
}

#ifdef CONFIG_ARCH_MDIRAC3
void s3c_sdi_dma_done_callback (
 s3c_dma_subchan_t *dma_ch,
 void *buf_id,
 int size,
 s3c_dma_buffresult_t result
#else
void s3c_sdi_dma_done_callback (
 s3c_dma_chan_t *dma_ch,
 void *buf_id,
 int size,
 s3c_dma_buffresult_t result
)
#endif
{	unsigned long iflags;
	uint sdi_dcnt;
	struct s3c_sdi_host *host = (struct s3c_sdi_host *)buf_id;

	spin_lock_irqsave( &host->complete_lock, iflags);

	if (!host->mrq)
		goto out;
	if (!host->mrq->data)
		goto out;

	sdi_dcnt = readl(host->base + S3C_SDIDCNT);

	if (result!=S3C_RES_OK)
		goto fail_request;


	if (host->mrq->data->flags & MMC_DATA_READ) {
		if (sdi_dcnt > 0)
			goto fail_request;
	}

out:
	complete(&host->complete_dma);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	return;


fail_request:
	host->mrq->data->error = MMC_ERR_FAILED;
	host->complete_what = COMPLETION_NONE;
	complete(&host->complete_dma);
	complete(&host->complete_request);
	writel(0, host->base + S3C_SDIIMSK);
	DBG(PFX "dma fail\n");
	goto out;

}

#ifdef CONFIG_ARCH_MDIRAC3
void s3c_sdi_dma_setup(struct s3c_sdi_host *host, eDMAPurpose_t purpose)
{
	int flowctrl,dest_per,src_per;

	switch(purpose) {
	case DMAP_READ:
		dest_per = 0;
		src_per  = S3C_DMA3_SDMMC;
		flowctrl=S3C_DMA_PER2MEM;
		break;

	case DMAP_WRITE:
		dest_per= S3C_DMA3_SDMMC;
		src_per = 0;
		flowctrl=S3C_DMA_MEM2PER;
		break;

	default:
		//add something for other type of transfers
	//	printk("coming to default statement dma transfer\n");
		break;
	}

	s3c_dma_devconfig(host->dma,host->subchannel,flowctrl,src_per,dest_per, host->mem->start + S3C_SDIDAT);
	s3c_dma_config(host->dma,host->subchannel, 4,4);
	s3c_dma_set_buffdone_fn(host->dma,host->subchannel, s3c_sdi_dma_done_callback);
	s3c_dma_setflags(host->dma,host->subchannel, S3C_DMAF_AUTOSTART);
}
#else
void s3c_sdi_dma_setup(struct s3c_sdi_host *host, eDMAPurpose_t purpose)
{
	s3c_dmasrc_t source = 0;

	switch(purpose) {
	case DMAP_READ:
		source = S3C_DMASRC_HW;
		break;

	case DMAP_WRITE:
		source=S3C_DMASRC_MEM;
		break;
	}

	s3c_dma_devconfig(host->dma, source, 3, host->mem->start + S3C_SDIDAT);
	s3c_dma_config(host->dma, 4, 0, S3C_REQSEL_SDMMC | (1<<0));
	s3c_dma_set_buffdone_fn(host->dma, s3c_sdi_dma_done_callback);
	s3c_dma_setflags(host->dma, S3C_DMAF_AUTOSTART);
}
#endif

#define RSP_TYPE(x)	((x) & ~(MMC_RSP_BUSY|MMC_RSP_OPCODE))

static void s3c_sdi_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
 	struct s3c_sdi_host *host = mmc_priv(mmc);
	u32 sdi_carg, sdi_ccon, sdi_timer, sdi_fsta;
	u32 sdi_bsize, sdi_dcon = 0, sdi_imsk;
	u32 dma_dir = 0;

	WARN_ON(host->mrq != NULL);

	DBG("#############request: [CMD] opcode:0x%02x arg:0x%08x flags:%x retries:%u\n",
		mrq->cmd->opcode, mrq->cmd->arg, mrq->cmd->flags, mrq->cmd->retries);

	host->mrq = mrq;

	sdi_ccon = mrq->cmd->opcode & S3C_SDICCON_INDEX;
	sdi_ccon|= (S3C_SDICCON_SENDERHOST | S3C_SDICCON_CMDSTART);

#ifdef CONFIG_S3CMCI_DEBUG
	{
		cmd_rec[cmd_idx] = mrq->cmd->opcode;
		cmd_idx++;
		cmd_idx %= 40;
	}
#endif
	sdi_carg = mrq->cmd->arg;

	/* XXX: Timer value ?! */
	sdi_timer= 0x7fffff;

	sdi_bsize= 0;

	/* enable interrupts for transmission errors */
	sdi_imsk = (S3C_SDIIMSK_RESPONSEND | S3C_SDIIMSK_CRCSTATUS);

	host->complete_what = COMPLETION_CMDSENT;

	if (RSP_TYPE(mmc_resp_type(mrq->cmd))) {
		host->complete_what = COMPLETION_RSPFIN;

		sdi_ccon |= S3C_SDICCON_WAITRSP;
		sdi_imsk |= S3C_SDIIMSK_CMDTIMEOUT;
	} else {
		/* We need the CMDSENT-Interrupt only if we want are not waiting
		 * for a response
		 */
		sdi_imsk |= S3C_SDIIMSK_CMDSENT;
	}

	if (mrq->cmd->flags & MMC_RSP_136) {
		sdi_ccon|= S3C_SDICCON_LONGRSP;
	}

	if (mrq->cmd->flags & MMC_RSP_CRC) {
		sdi_imsk |= S3C_SDIIMSK_RESPONSECRC;
	}

	if (mrq->data) {
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;

		sdi_ccon|= S3C_SDICCON_WITHDATA;

		sdi_bsize = mrq->data->blksz;

		sdi_dcon  = (mrq->data->blocks & S3C_SDIDCON_BLKNUM_MASK);
		sdi_dcon |= S3C_SDIDCON_DMAEN;
		sdi_dcon |= S3C_SDIDCON_WORDTX;

		if (mmc->ios.bus_width == MMC_BUS_WIDTH_4) {
			sdi_dcon |= S3C_SDIDCON_WIDEBUS;
		}

		sdi_imsk |= 0xFFFFFFE0;

		DBG(PFX "request: [DAT] bsize:%u blocks:%u bytes:%u\n",
			sdi_bsize, mrq->data->blocks, mrq->data->blocks * sdi_bsize);

		if (!(mrq->data->flags & MMC_DATA_STREAM)) {
			sdi_dcon |= S3C_SDIDCON_BLOCKMODE;
		}

		if (mrq->data->flags & MMC_DATA_WRITE) {
			sdi_dcon |= S3C_SDIDCON_TXAFTERRESP;
			sdi_dcon |= S3C_SDIDCON_XFER_TX;
			sdi_dcon |= S3C_SDIDCON_DSTART;
			s3c_sdi_dma_setup(host, DMAP_WRITE);
			dma_dir = DMA_TO_DEVICE;
		} else {
			sdi_dcon |= S3C_SDIDCON_RXAFTERCMD;
			sdi_dcon |= S3C_SDIDCON_XFER_RX;
			sdi_dcon |= S3C_SDIDCON_DSTART;
			s3c_sdi_dma_setup(host, DMAP_READ);
			dma_dir = DMA_FROM_DEVICE;
		}

		sdi_fsta = S3C_SDIFSTA_FRST;
		__raw_writel(sdi_fsta,host->base + S3C_SDIFSTA);

		/* start DMA */
		dma_map_sg(mmc_dev(mmc), mrq->data->sg, mrq->data->sg_len, dma_dir);
		s3c_dma_enqueue(host->dma, (void *) host,
			sg_dma_address(mrq->data->sg),
			(mrq->data->blocks * mrq->data->blksz) );
	}

	host->mrq = mrq;
	init_completion(&host->complete_request);
	init_completion(&host->complete_dma);

	/* Clear command and data status registers */
	writel(0xFFFFFFFF, host->base + S3C_SDICSTA);
	writel(0xFFFFFFFF, host->base + S3C_SDIDSTA);

	/* Setup SDI controller */
	writel(sdi_bsize,host->base + S3C_SDIBSIZE);
	writel(sdi_timer,host->base + S3C_SDITIMER);
	writel(sdi_imsk,host->base + S3C_SDIIMSK);

	/* Setup SDI command argument and data control */
	writel(sdi_carg, host->base + S3C_SDICARG);
	writel(sdi_dcon, host->base + S3C_SDIDCON);
	/* This initiates transfer */
	writel(sdi_ccon, host->base + S3C_SDICCON);

	/* this wait is very important to sd/mmc run correctly.
	 * Without this blocking code, operation sequence may be crashed.
	 * by scsuh.
	 */
	/* Wait for transfer to complete */
	wait_for_completion(&host->complete_request);
	if (mrq->data && (host->mrq->data->error == MMC_ERR_NONE)) {
		wait_for_completion(&host->complete_dma);
		DBG("[DAT] DMA complete.\n");
		sdi_fsta = readl(host->base + S3C_SDIFSTA);
		writel(sdi_fsta,host->base + S3C_SDIFSTA);
	}

	/* Cleanup controller */
	writel(0, host->base + S3C_SDICARG);
	writel(0, host->base + S3C_SDIDCON);
	writel(0, host->base + S3C_SDICCON);
	writel(0, host->base + S3C_SDIIMSK);

	/* Read response */
	mrq->cmd->resp[0] = readl(host->base + S3C_SDIRSP0);
	mrq->cmd->resp[1] = readl(host->base + S3C_SDIRSP1);
	mrq->cmd->resp[2] = readl(host->base + S3C_SDIRSP2);
	mrq->cmd->resp[3] = readl(host->base + S3C_SDIRSP3);

	host->mrq = NULL;

	DBG(PFX "request done.\n");

	if (mrq->data) {
		dma_unmap_sg(mmc_dev(mmc), mrq->data->sg, mrq->data->sg_len, dma_dir);

		/* Calulate the amout of bytes transfer, but only if there was
		 * no error
		 */
		if (mrq->data->error == MMC_ERR_NONE)
			mrq->data->bytes_xfered = (mrq->data->blocks * mrq->data->blksz);
		else
			mrq->data->bytes_xfered = 0;

		/* If we had an error while transfering data we flush the
		 * DMA channel to clear out any garbage
		 */
		if (mrq->data->error != MMC_ERR_NONE) {
#ifdef CONFIG_ARCH_MDIRAC3
			s3c_dma_ctrl(host->dma,host->subchannel, S3C_DMAOP_FLUSH);
#else
			s3c_dma_ctrl(host->dma, S3C_DMAOP_FLUSH);
#endif
			DBG(PFX "flushing DMA.\n");
		}
		/* Issue stop command */
		if (mrq->data->stop)
			mmc_wait_for_cmd(mmc, mrq->data->stop, 3);
	}

	mrq->done(mrq);
}

static void s3c_sdi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct s3c_sdi_host *host = mmc_priv(mmc);
	u32 sdi_psc, sdi_con = 0, sdi_fsta;

	writel(0, host->base + S3C_SDICON);
	udelay(1000);

	DBG("%d, %d\n", ios->clock, ios->bus_width);
	/* Set power */
	switch(ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
#ifdef CONFIG_ARCH_S3C2460
		gpio_set_pin(S3C_GPA8, S3C_GPA8_SDIO_CMD);
		gpio_set_pin(S3C_GPA9, S3C_GPA9_SDIO_DAT0);
		gpio_set_pin(S3C_GPA10, S3C_GPA10_SDIO_DAT1);
		gpio_set_pin(S3C_GPA11, S3C_GPA11_SDIO_DAT2);
		gpio_set_pin(S3C_GPA12, S3C_GPA12_SDIO_DAT3);
		gpio_set_pin(S3C_GPA13, S3C_GPA13_SDIO_CLK);
		sdi_con = S3C_SDICON_CLKTYPE_MMC | (1 << 6);
#else
		gpio_set_pin(S3C_GPE5, S3C_GPE5_SDCLK);
		gpio_set_pin(S3C_GPE6, S3C_GPE6_SDCMD);
		gpio_set_pin(S3C_GPE7, S3C_GPE7_SDDAT0);
		gpio_set_pin(S3C_GPE8, S3C_GPE8_SDDAT1);
		gpio_set_pin(S3C_GPE9, S3C_GPE9_SDDAT2);
		gpio_set_pin(S3C_GPE10, S3C_GPE10_SDDAT3);
#endif

		if (mmc->mode == MMC_MODE_MMC) {
			sdi_con = S3C_SDICON_CLKTYPE_MMC;
		}
		break;

	case MMC_POWER_OFF:
	default:
		sdi_con = S3C_SDICON_RESET;
		writel(sdi_con, host->base + S3C_SDICON);
		while(readl(host->base + S3C_SDICON) & S3C_SDICON_RESET);
#ifdef CONFIG_ARCH_S3C2460
		gpio_direction_output(S3C_GPA13);
#else
		gpio_direction_output(S3C_GPE5);
#endif
		break;
	}

	sdi_fsta = S3C_SDIFSTA_FRST;
	writel(sdi_fsta,host->base + S3C_SDIFSTA);

	/* Set clock */
	for(sdi_psc=0;sdi_psc<254;sdi_psc++) {
		if ( (clk_get_rate(host->clk) / (sdi_psc+1)) <= ios->clock)
			break;
	}

	if (sdi_psc > 255)
		sdi_psc = 255;
	writel(sdi_psc, host->base + S3C_SDIPRE);

#ifdef CONFIG_S3CMCI_DEBUG
	if (sdi_psc < 10)
		printk("  clock is: %lu.%luMHz of %uMHz\n",
			clk_get_rate(host->clk)/(sdi_psc+1)/1000000,
			(clk_get_rate(host->clk)/(sdi_psc+1)/1000)%1000,
			(ios->clock)/1000000);
#endif

	/* Only SD clock can support more than 20MHz clock.
	 * by scsuh
	 */
	if (clk_get_rate(host->clk) / (sdi_psc+1) > 20 * MHZ)
		sdi_con &= ~S3C_SDICON_CLKTYPE_MMC;

	/* Set CLOCK_ENABLE */
	if (ios->clock)
		sdi_con |= S3C_SDICON_CLKENABLE;
	else
		sdi_con &=~S3C_SDICON_CLKENABLE;

	writel(sdi_con, host->base + S3C_SDICON);
	udelay(1000);
}

static struct mmc_host_ops s3c_sdi_ops = {
	.request	= s3c_sdi_request,
	.set_ios	= s3c_sdi_set_ios,
};

static int s3c_sdi_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct s3c_sdi_host *host;

	int ret;
#ifdef CONFIG_S3C2443_EVT1
	/* EXTINT0 S3C2443 EVT1 workaround */
	u32 tmp;
#endif

	mmc = mmc_alloc_host(sizeof(struct s3c_sdi_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto probe_out;
	}

	host = mmc_priv(mmc);
	spin_lock_init(&host->complete_lock);
	host->complete_what 	= COMPLETION_NONE;
	host->mmc 		= mmc;
#ifdef CONFIG_ARCH_S3C2460
	host->irq_cd		= IRQ_EINT3;
#elif defined(CONFIG_MACH_SMDK2443)
	host->irq_cd		= IRQ_EINT1;
#elif defined CONFIG_MACH_SMDK2412
	host->irq_cd		= IRQ_EINT18;
#endif
	host->dma		= S3C_SDI_DMA;

	host->mem = platform_get_resource(pdev, IORESOURCE_MEM ,0);
	if (!host->mem) {
		printk("failed to get io memory region resource.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->mem = request_mem_region(host->mem->start,
		RESSIZE(host->mem), pdev->name);

	if (!host->mem) {
		printk("failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	/* if there is an error here, check your SoC dependent code.
	 * You must have iotable that contains SDI in it.
	 * by scsuh.
	 */
        host->base = S3C_VA_SDI;
	host->irq = platform_get_irq(pdev, 0);

	if (host->irq == 0) {
		printk("failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto release_memory;
	}

	if (request_irq(host->irq, s3c_sdi_irq, 0, DRIVER_NAME, host)) {
		printk("failed to request sdi interrupt.\n");
		ret = -ENOENT;
		goto release_memory;
	}

#if defined(CONFIG_MACH_SMDK2443)
#ifdef CONFIG_S3C2443_EVT1
	/* EXTINT0 S3C2443 EVT1 workaround */
	tmp = __raw_readl(S3C_EXTINT0);
	s3c_swap_4bit(tmp);
	__raw_writel(tmp | (1<<7), S3C_EXTINT0);
#endif
	gpio_set_pin(S3C_GPF1, S3C_GPF1_EINT1);
#elif defined(CONFIG_ARCH_S3C2460)
	gpio_set_pin(S3C_GPJ3, S3C_GPJ3_EXT_INT3);
#elif defined (CONFIG_MACH_SMDK2412)
	gpio_set_pin(S3C_GPG10, S3C_GPG10_EINT18);
#endif

	set_irq_type(host->irq_cd, IRQT_BOTHEDGE);

	if (request_irq(host->irq_cd, s3c_sdi_irq_cd, SA_INTERRUPT, DRIVER_NAME, host)) {
		printk("failed to request card detect interrupt.\n" );
		ret = -ENOENT;
		goto probe_free_irq;
	}

	if (s3c_dma_request(S3C_SDI_DMA, &s3c_sdi_dma_client, NULL)) {
		printk("unable to get DMA channel.\n" );
		ret = -EBUSY;
		goto probe_free_irq_cd;
	}

	host->clk = clk_get(&pdev->dev, "sdi");
	if (IS_ERR(host->clk)) {
		printk("failed to find clock source.\n");
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto probe_free_host;
	}

	if ((ret = clk_enable(host->clk))) {
		printk("failed to enable clock source.\n");
		goto clk_free;
	}

	mmc->ops = &s3c_sdi_ops;
	mmc->ocr_avail = MMC_VDD_32_33|MMC_VDD_33_34;
	mmc->f_min = clk_get_rate(host->clk) / 512;
	/* you must make sure that our sdmmc block can support
	 * up to 25MHz. by scsuh
	 */
	mmc->f_max = 25 * MHZ;
	mmc->caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_MULTIWRITE);

	/*
	 * Since we only have a 16-bit data length register, we must
	 * ensure that we don't exceed 2^16-1 bytes in a single request.
	 * Choose 64 (512-byte) sectors as the limit.
	 */
	mmc->max_sectors = 64;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */

	mmc->max_seg_size = mmc->max_sectors << 9;
	printk(KERN_INFO PFX "probe: mapped sdi_base=%p irq=%u irq_cd=%u dma=%u.\n",
		host->base, host->irq, host->irq_cd, host->dma);
	platform_set_drvdata(pdev, mmc);

	init_timer(&host->timer);
        host->timer.data = (unsigned long)host;
        host->timer.function = s3c_sdi_check_status;
        host->timer.expires = jiffies + HZ;

	if ((ret = mmc_add_host(mmc))) {
		printk(KERN_INFO PFX "failed to add mmc host.\n");
		goto free_dmabuf;
	}

	printk(KERN_INFO PFX "initialization done.\n");
	return 0;

free_dmabuf:
	clk_disable(host->clk);

clk_free:
	clk_put(host->clk);

probe_free_irq_cd:
 	free_irq(host->irq_cd, host);

probe_free_irq:
 	free_irq(host->irq, host);

release_memory:
	release_mem_region(host->mem->start, RESSIZE(host->mem));

probe_free_host:
	mmc_free_host(mmc);

probe_out:
	return ret;
}

static int s3c_sdi_remove(struct platform_device *dev)
{
	struct mmc_host *mmc  = platform_get_drvdata(dev);
	struct s3c_sdi_host *host = mmc_priv(mmc);

	del_timer_sync(&host->timer);
	mmc_remove_host(mmc);

	clk_disable(host->clk);
	clk_put(host->clk);
 	free_irq(host->irq_cd, host);
 	free_irq(host->irq, host);
	mmc_free_host(mmc);

	return 0;
}

#ifdef CONFIG_PM
static int s3c_sdi_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct s3c_sdi_host *host = mmc_priv(mmc);
	int ret = 0;

	if (mmc)
#if 0
		mmc_remove_host(host);
#else
		ret = mmc_suspend_host(mmc, state);
	s3c_dma_free(host->dma, &s3c_sdi_dma_client);
#endif

	//clk_disable(host->clk);
	return ret;
}

static int s3c_sdi_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct s3c_sdi_host *host = mmc_priv(mmc);
	int ret = 0;

#if 0
	clk_enable(host->clk);
	init_timer(&host->timer);
        host->timer.data = (unsigned long)host;
        host->timer.function = s3c_sdi_check_status;
        host->timer.expires = jiffies + HZ;

	if ((ret = mmc_add_host(mmc))) {
		printk(KERN_INFO PFX "failed to add mmc host.\n");
	}

	printk(KERN_INFO PFX "initialization done.\n");
#else
	s3c_dma_request(host->dma, &s3c_sdi_dma_client, NULL);
	if (mmc)
		ret = mmc_resume_host(mmc);
#endif

	return ret;
}
#else
#define s3c_sdi_suspend	NULL
#define s3c_sdi_resume	NULL
#endif

static struct platform_driver s3c_sdi_driver =
{
        .probe          = s3c_sdi_probe,
        .remove         = s3c_sdi_remove,
	.suspend	= s3c_sdi_suspend,
	.resume		= s3c_sdi_resume,
	.driver		= {
		.name	= "s3c-sdi",
	},
};

static int __init s3c_sdi_init(void)
{
	return platform_driver_register(&s3c_sdi_driver);
}

static void __exit s3c_sdi_exit(void)
{
	platform_driver_unregister(&s3c_sdi_driver);
}
module_init(s3c_sdi_init);
module_exit(s3c_sdi_exit);

MODULE_DESCRIPTION("S3C Multimedia Card I/F Driver");
MODULE_LICENSE("GPL");

