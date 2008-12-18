/*
 * $Id: s3c-ide.c,v 1.9 2008/06/11 01:00:33 scsuh Exp $
 *
 * linux/drivers/ide/arm/s3c-ide.c
 * Copyright(C) Samsung Electronics 2006
 * Seung-Chull, Suh	<sc.suh@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <linux/init.h>
#include <linux/ide.h>
#include <linux/sysdev.h>
#include <linux/scatterlist.h>

#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>

#include "../ide-timing.h"

#include <asm/dma.h>
#include <asm/dma-mapping.h>

#include <asm/arch/regs-ide.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-s3c6400-clock.h>

#include <asm/io.h>
#include "s3c-ide.h"

#define DRV_NAME	"s3c-ide"
#define DRV_VERSION	"0.7"
#define DRV_AUTHOR	"Seung-Chull, Suh <sc.suh@samsung.com>"

#undef DBG_ATA
#ifdef DBG_ATA
#define DbgAta(x...) printk(x)
#else
#define DbgAta(x...) do {} while(0);
#endif

static s3c_ide_hwif_t s3c_ide_hwif;

static void __iomem *ata_base;
/*=========================================================================
 *          	    ata controller register functions
 *=========================================================================
 */
static inline int ata_status_check (ide_drive_t * drive, u8 startend)
{
	u8 stat;
	uint i;

	for (i=0; i<10000000; i++) {
		stat = HWIF(drive)->INB(IDE_STATUS_REG);
		if ((stat == 0x58) && (startend == 0))     // for starting
			return 0;
		else if ((stat == 0x50) && (startend == 1))// for ending
			return 0;
	}
	printk("got error in ata_status_check : %08x\n", stat);
	return -1;
}

static inline int bus_fifo_status_check (BUS_STATE status)
{
	u32 temp;
	uint i;

	for (i=0; i<10000000; i++) {
		temp = readl(ata_base + S3C_BUS_FIFO_STATUS);
		/* wait for IDLE */
		if ((temp == 0) && (status == IDLE)) {
			return 0;
		}
		/* wait for PAUSEW */
		else if (((temp >> 16) == 0x5) && (status == PAUSEW)) {
			return 0;
		}
		/* wait for PAUSER2 and read/write pointer 0 */
		else if ((temp == 0x60000) && (status == PAUSER2)) {
			return 0;
		}
	}
	return temp;
}

/*
 * return 0: OK
 * return -1: got error
 */
static inline int wait_for_host_ready (void)
{
	u32 temp;
	uint i;

	for (i=0; i<10000000; i++) {
		temp = readl(ata_base + S3C_ATA_FIFO_STATUS);
		if ((temp >> 28) == 0) {
			return 0;
		}
		DbgAta("S3C_ATA_FIFO_STATUS: %08x\n", temp);
	}
	printk("got error in host ready: %08x\n", temp);
	return -1;
}

#ifdef CONFIG_BLK_DEV_IDE_S3C_UDMA
static u8 read_dev_reg (uint reg)
{
	u8 temp;

	wait_for_host_ready();
	temp = readb(ata_base + reg);
	wait_for_host_ready();
	temp = readb(ata_base + S3C_ATA_PIO_RDATA);
	return temp;
}

#define STATUS_DEVICE_BUSY	0x80
static int wait_for_dev_ready (void)
{
	u8 temp;
	uint i;

	for (i=0; i<1000000; i++) {
		temp = read_dev_reg((uint)S3C_ATA_PIO_CSD);
//		printf("temp2: %08x\n", temp);
		if ((temp & STATUS_DEVICE_BUSY) == 0) {
			DbgAta("temp2: %08x\n", temp);
			DbgAta("wait_for_dev_ready: %d\n", i);
			return 0;
		}
	}

	printk("wait_for_dev_ready: error\n");
	return -1;
}
#endif

#if defined(CONFIG_BLK_DEV_IDE_S3C_UDMA) || defined(CONFIG_BLK_DEV_IDE_S3C_PDMA)
static void set_config_mode (ATA_MODE mode, int rw)
{
	u32 reg = readl(ata_base + S3C_ATA_CFG) & ~(0x39c);

	/* set mode */
	reg |= mode << 2;

	/* DMA write mode */
	if (mode && rw)
		reg |= 0x10;

	/* set ATA DMA auto mode (enable multi block transfer) */
	if (mode == UDMA)
		reg |= 0x380;

	writel(reg, ata_base + S3C_ATA_CFG);
	DbgAta("2:S3C_ATA_CFG = 0x%08x\n", readl(ata_base + S3C_ATA_CFG));
}
#endif

#if defined CONFIG_ARCH_S3C2443 || defined (CONFIG_CPU_S3C2450) 
static void init_buf_ctrl (void)
{
#if 0
	/* for EVT0 board */
	u32 reg = readl(S3C_GPBCON);

	reg &= 0xffffc0fc;
	reg |= (1<<12 | 1<<10 | 1<<8 | 1<<0);
	writel(reg, S3C_GPBCON);
	printk("GPBCON = 0x%08x\n", readl(S3C_GPBCON));
#else
	/* for EVT1 board */
	u32 reg = readl(S3C2410_GPADAT);

	reg &= ~(1<<5);
	writel(reg, S3C2410_GPADAT);

#endif
}

static void change_board_buf_mode (ATA_MODE mode, uint rw)
{
	u32 reg = readl(S3C2410_GPBDAT) & 0xffffff8e; /* 0,4,5,6 */

	if (mode == UDMA) {
		/* GPB0->high,GPB4->high => UDMA mode */
		reg |= (1<<4)|(1);
		if (rw)
			reg |= (1<<5);
	} else {
		/* GPB0->high,GPB4->low => PIO mode */
		reg |= (1<<6)|1;
	}
	writel(reg, S3C2410_GPBDAT);
}
#endif /* CONFIG_ARCH_S3C2443 */

static void set_ata_enable (uint on)
{
	u32 temp;

	temp = readl(ata_base + S3C_ATA_CTRL);

	if (on)
		writel(temp | 0x1, ata_base + S3C_ATA_CTRL);
	else
		writel(temp & 0xfffffffe, ata_base + S3C_ATA_CTRL);
}

static void set_endian_mode (uint little)
{
	u32 reg = readl(ata_base + S3C_ATA_CFG);

	/* set Little endian */
	if (little)
		reg &= (~0x40);
	else
		reg |= 0x40;

	writel(reg, ata_base + S3C_ATA_CFG);
}


static inline void set_trans_command (ATA_TRANSFER_CMD cmd)
{
	wait_for_host_ready();
	writel(cmd, ata_base + S3C_ATA_CMD);
}

/*
 * this filter only allows PIO and UDMA.
 * swdma and mwdma is not supported now. by scsuh
 */
static u8 s3c_ide_ratefilter (ide_drive_t *drive, u8 speed)
{
	if (drive->media != ide_disk)
		return min(speed, (u8)XFER_PIO_4);

	switch(speed) {
	case XFER_UDMA_6:
	case XFER_UDMA_5:
		speed = XFER_UDMA_4;// S3C6400 & S3C6410 can support upto UDMA4.
		break;
	case XFER_UDMA_4:
	case XFER_UDMA_3:
	case XFER_UDMA_2:
	case XFER_UDMA_1:
	case XFER_UDMA_0:
		break;
	default:
		speed = min(speed, (u8)XFER_PIO_4);
		break;
	}
	return speed;
}

static int s3c_ide_tune_chipset (ide_drive_t * drive, u8 xferspeed)
{
	ide_hwif_t *hwif = drive->hwif;
	s3c_ide_hwif_t *s3c_hwif = (s3c_ide_hwif_t *) hwif->hwif_data;

	u8 speed = s3c_ide_ratefilter(drive, xferspeed);

	if (XFER_PIO_0 <= speed && speed <= XFER_PIO_4) {
		ulong ata_cfg = readl(ata_base + S3C_ATA_CFG);

		switch (speed) {
		case XFER_PIO_0:
		case XFER_PIO_1:
		case XFER_PIO_2:
			ata_cfg &= (~0x2);	//IORDY disable
			break;
		case XFER_PIO_3:
		case XFER_PIO_4:
			ata_cfg |= 0x2;		//IORDY enable
			break;
		}

		writel(ata_cfg, ata_base + S3C_ATA_CFG);
		writel(s3c_hwif->piotime[speed-XFER_PIO_0], ata_base + S3C_ATA_PIO_TIME);
	} else {
		writel(s3c_hwif->piotime[0], ata_base + S3C_ATA_PIO_TIME);
		writel(s3c_hwif->udmatime[speed-XFER_UDMA_0], ata_base + S3C_ATA_UDMA_TIME);
	}
	return ide_config_drive_speed(drive, speed);
}

void read_status (uint index)
{
	u32 reg;
	reg = readl(ata_base + S3C_ATA_IRQ);
//	printk("%d: S3C_ATA_IRQ: %08x\n", index, reg);
	writel(reg, ata_base + S3C_ATA_IRQ);
}

int s3c_ide_ack_intr (ide_hwif_t *hwif)
{
#ifdef CONFIG_BLK_DEV_IDE_S3C_UDMA
	s3c_ide_hwif_t *s3c_hwif = (s3c_ide_hwif_t *) hwif->hwif_data;
#endif
	u32 reg;

	reg = readl(ata_base + S3C_ATA_IRQ);
	DbgAta("S3C_ATA_IRQ: %08x\n", reg);

#ifdef CONFIG_BLK_DEV_IDE_S3C_UDMA
	s3c_hwif->irq_sta = reg;
#endif

	return 1;
}

static void s3c_ide_tune_drive (ide_drive_t * drive, u8 pio)
{
	pio = ide_get_best_pio_mode(drive, 255, pio);
	(void) s3c_ide_tune_chipset(drive, (XFER_PIO_0 + pio));
}

/*
 * XXX: It is board-specific to support UDMA in smdk24xx.
 *      not supported: SMDK241X, SMDK2443
 *      by scsuh
 */
#ifdef CONFIG_BLK_DEV_IDE_S3C_UDMA

static void s3c_ide_dma_host_on (ide_drive_t * drive)
{
	DbgAta("##### %s\n", __FUNCTION__);
}

static int s3c_ide_dma_on (ide_drive_t * drive)
{
	DbgAta("##### %s\n", __FUNCTION__);
	drive->using_dma = 1;
	s3c_ide_dma_host_on(drive);
	return 0;
}

#if 0
static int s3c_ide_dma_check (ide_drive_t * drive)
{
	u8 speed;
	DbgAta("##### %s\n", __FUNCTION__);
	/* Is the drive in our DMA black list? */
	if (s3c_ide_hwif.black_list) {
		drive->using_dma = 0;

		/* Borrowed the warning message from ide-dma.c */
		printk(KERN_WARNING "%s: Disabling DMA for %s (blacklisted)\n",
		       drive->name, drive->id->model);
	} else
		drive->using_dma = 1;

	speed = ide_find_best_mode(drive, 0x47);
	DbgAta("speed: %08x\n", speed);

	if (drive->autodma && (speed & XFER_MODE) == XFER_UDMA) {
		s3c_ide_tune_chipset(drive, speed);
		return HWIF(drive)->ide_dma_on(drive);
	}

	HWIF(drive)->dma_off_quietly(drive);

	return 0;
}

static ide_startstop_t s3c_ide_dma_intr (ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	s3c_ide_hwif_t *s3c_hwif = (s3c_ide_hwif_t *) hwif->hwif_data;
	ulong reg = s3c_hwif->irq_sta;
	ide_hwgroup_t *hwgroup = hwif->hwgroup;

	DbgAta("reg = %08lx\n", reg);
	DbgAta("S3C_ATA_XFR_CNT: %08x\n", readl(S3C_ATA_XFR_CNT));
	DbgAta("index:%d\n", s3c_hwif->index);

	if (reg & 0x10) {
		DbgAta("write: %08lx %08lx\n",
			s3c_hwif->table[s3c_hwif->index].addr,
			s3c_hwif->table[s3c_hwif->index].len);
		writel(s3c_hwif->table[s3c_hwif->index].len-0x20, S3C_ATA_SBUF_SIZE);
		writel(s3c_hwif->table[s3c_hwif->index].addr, S3C_ATA_SBUF_START);
	} else if (reg & 0x08) {
		DbgAta("read: %08lx %08lx\n",
			s3c_hwif->table[s3c_hwif->index].addr,
			s3c_hwif->table[s3c_hwif->index].len);
		writel(s3c_hwif->table[s3c_hwif->index].len-0x20, S3C_ATA_TBUF_SIZE);
		writel(s3c_hwif->table[s3c_hwif->index].addr, S3C_ATA_TBUF_START);
	}
	s3c_hwif->index++;
	set_trans_command(ATA_CMD_CONTINUE);
	hwgroup->handler = s3c_ide_dma_intr;

	return 1;
}
#endif

static int s3c_ide_build_sglist (ide_drive_t * drive, struct request *rq)
{
	ide_hwif_t *hwif = drive->hwif;
	s3c_ide_hwif_t *s3c_hwif = (s3c_ide_hwif_t *) hwif->hwif_data;
	struct scatterlist *sg = hwif->sg_table;

	DbgAta("##### %s\n", __FUNCTION__);
	ide_map_sg(drive, rq);

	if (rq_data_dir(rq) == READ)
		hwif->sg_dma_direction = DMA_FROM_DEVICE;
	else
		hwif->sg_dma_direction = DMA_TO_DEVICE;

	return dma_map_sg(&s3c_hwif->dev->dev, sg, hwif->sg_nents, hwif->sg_dma_direction);
}

static int s3c_ide_build_dmatable (ide_drive_t * drive)
{
	int i, size, count = 0;
	ide_hwif_t *hwif = HWIF(drive);
	struct request *rq = HWGROUP(drive)->rq;

	s3c_ide_hwif_t *s3c_hwif = (s3c_ide_hwif_t *) hwif->hwif_data;
	struct scatterlist *sg;

	u32 addr_reg, size_reg;

	DbgAta("##### %s\n", __FUNCTION__);

	if (rq_data_dir(rq) == WRITE) {
		addr_reg = S3C_ATA_SBUF_START;
		size_reg = S3C_ATA_SBUF_SIZE;
	} else {
		addr_reg = S3C_ATA_TBUF_START;
		size_reg = S3C_ATA_TBUF_SIZE;
	}

	/* Save for interrupt context */
	s3c_hwif->drive = drive;

	/* Build sglist */
	hwif->sg_nents = size = s3c_ide_build_sglist(drive, rq);
	DbgAta("##### hwif->sg_nents %d\n", hwif->sg_nents);

	if (size > 1) {
		s3c_hwif->pseudo_dma = 1;
		DbgAta("sg_nents is more than 1: %d\n", hwif->sg_nents);
	}
	if (!size)
		return 0;

	/* fill the descriptors */
	sg = hwif->sg_table;
	for (i=0, sg = hwif->sg_table; i<size && sg_dma_len(sg); i++, sg++) {
		s3c_hwif->table[i].addr = sg_dma_address(sg);
		s3c_hwif->table[i].len = sg_dma_len(sg);
		count += s3c_hwif->table[i].len;
		DbgAta("data: %08lx %08lx\n",
			s3c_hwif->table[i].addr,
			s3c_hwif->table[i].len);
	}
	s3c_hwif->table[i].addr = 0;
	s3c_hwif->table[i].len = 0;
	s3c_hwif->queue_size = i;
	DbgAta("total size: %08x, %d\n", count, s3c_hwif->queue_size);

	DbgAta("rw: %08lx %08lx\n",
		s3c_hwif->table[0].addr,
		s3c_hwif->table[0].len);
	writel(s3c_hwif->table[0].len-0x1, ata_base + size_reg);
	writel(s3c_hwif->table[0].addr, ata_base + addr_reg);

	s3c_hwif->index = 1;

	writel(count, ata_base + S3C_ATA_XFR_NUM);

	return 1;
}

static int s3c_ide_dma_setup (ide_drive_t * drive)
{
	struct request *rq = HWGROUP(drive)->rq;

	if (!s3c_ide_build_dmatable(drive)) {
		ide_map_sg(drive, rq);
		DbgAta("return 1\n");
		return 1;
	}

	drive->waiting_for_dma = 1;
	return 0;
}

static void s3c_ide_dma_exec_cmd (ide_drive_t * drive, u8 command)
{
	wait_for_host_ready();

	/* issue cmd to drive */
	ide_execute_command(drive, command, &ide_dma_intr, (WAIT_CMD), NULL);
}

static void s3c_ide_dma_start (ide_drive_t * drive)
{
	struct request *rq = HWGROUP(drive)->rq;
	uint rw = (rq_data_dir(rq) == WRITE);

#if defined CONFIG_ARCH_S3C2443 || defined (CONFIG_CPU_S3C2450)
	/* apply at specific set using buffer control */
	change_board_buf_mode(UDMA, rw);
#endif
	ata_status_check(drive, 0);
	writel(0x00000003, ata_base + S3C_ATA_IRQ_MSK);

	wait_for_host_ready();
	/* must be added to wait for Device ready. by scsuh */
	wait_for_dev_ready();
	bus_fifo_status_check(IDLE);

	set_config_mode(UDMA, rw);
	set_trans_command(ATA_CMD_START);
	return;
}

static int s3c_ide_dma_end (ide_drive_t * drive)
{
	u32 stat = 0;
	ide_hwif_t *hwif = HWIF(drive);
	s3c_ide_hwif_t *s3c_hwif = (s3c_ide_hwif_t *) hwif->hwif_data;

	DbgAta("##### %s\n", __FUNCTION__);
	DbgAta("left: %08x, %08x\n", readl(ata_base + S3C_ATA_XFR_CNT),
				readl(ata_base + S3C_ATA_XFR_NUM));

#if defined CONFIG_ARCH_S3C2443 || defined (CONFIG_CPU_S3C2450)
	/* apply at specific set using buffer control */
	change_board_buf_mode(PIO_CPU, 0);
#endif
	wait_for_host_ready();

	stat = readl(ata_base + S3C_BUS_FIFO_STATUS);
	if ( (stat >> 16) == 0x5 ) { /* in case of PAUSEW. */
		writel(ATA_CMD_CONTINUE, ata_base + S3C_ATA_CMD);
	}

	bus_fifo_status_check(IDLE);
	writel(0x00000003, ata_base + S3C_ATA_IRQ_MSK);
	set_config_mode(PIO_CPU, 0);
	ata_status_check(drive, 1);

	drive->waiting_for_dma = 0;

	if (hwif->sg_nents) {
		dma_unmap_sg(&s3c_hwif->dev->dev, hwif->sg_table, hwif->sg_nents,
			     hwif->sg_dma_direction);
		hwif->sg_nents = 0;
	}
	return 0;
}

static int s3c_ide_dma_timeout (ide_drive_t * drive)
{
	printk(KERN_ERR "%s: DMA timeout occurred: ", drive->name);

	if (HWIF(drive)->ide_dma_test_irq(drive))
		return 0;

	return HWIF(drive)->ide_dma_end(drive);
}

/* XXX: we must implement correct function. by scsuh. */
static int s3c_ide_dma_test_irq (ide_drive_t * drive)
{
	return 1;
}

static void s3c_ide_dma_host_off (ide_drive_t * drive)
{
	u32 reg;

	DbgAta("##### %s\n", __FUNCTION__);

#if defined CONFIG_ARCH_S3C2443 || defined (CONFIG_CPU_S3C2450) 
	/* apply at specific set using buffer control */
	change_board_buf_mode(PIO_CPU, 0);
#endif
	set_config_mode(PIO_CPU, 0);
	mdelay(10);

	/* disable ATA DMA auto mode */
	reg = readl(ata_base + S3C_ATA_CFG);
	reg &= (~0x200);
	writel(reg, ata_base + S3C_ATA_CFG);
}

static void s3c_ide_dma_off_quietly (ide_drive_t * drive)
{
	drive->using_dma = 0;
	s3c_ide_dma_host_off(drive);
}

static int s3c_ide_dma_lostirq (ide_drive_t * drive)
{
	printk("%08x, %08x\n", readl(ata_base + S3C_ATA_IRQ),
				readl(ata_base + S3C_ATA_IRQ_MSK));
	printk("left: %08x, %08x\n", readl(ata_base + S3C_ATA_XFR_CNT),
				readl(ata_base + S3C_ATA_XFR_NUM));

	printk(KERN_ERR "%s: IRQ lost\n", drive->name);
	return 0;
}

#endif

static int s3c_ide_irq_hook (void * data)
{
#ifdef CONFIG_BLK_DEV_IDE_S3C_UDMA
	u32 stat = 0;
	ide_hwif_t *hwif = (ide_hwif_t *)data;
	s3c_ide_hwif_t *s3c_hwif = (s3c_ide_hwif_t *) hwif->hwif_data;
#endif

	u32 reg = readl(ata_base + S3C_ATA_IRQ);
	writel(reg, ata_base + S3C_ATA_IRQ);
	DbgAta("##### %s, %08x\n", __FUNCTION__, reg);

#ifdef CONFIG_BLK_DEV_IDE_S3C_UDMA
	if (s3c_hwif->pseudo_dma) {
		uint i;
		i = s3c_hwif->index;

		if (reg & 0x10) {
			DbgAta("write: %08lx %08lx\n",
				s3c_hwif->table[i].addr,
				s3c_hwif->table[i].len);
			bus_fifo_status_check(PAUSER2);
			writel(s3c_hwif->table[i].len-0x1, ata_base + S3C_ATA_SBUF_SIZE);
			writel(s3c_hwif->table[i].addr, ata_base + S3C_ATA_SBUF_START);
		} else if (reg & 0x08) {
			DbgAta("read: %08lx %08lx\n",
				s3c_hwif->table[i].addr,
				s3c_hwif->table[i].len);

			stat = bus_fifo_status_check(PAUSEW);
			if (stat == 0x60000) {
				writel(s3c_hwif->table[i].len-0x1, ata_base + S3C_ATA_SBUF_SIZE);
				writel(s3c_hwif->table[i].addr, ata_base + S3C_ATA_SBUF_START);
			}

			writel(s3c_hwif->table[i].len-0x1, ata_base + S3C_ATA_TBUF_SIZE);
			writel(s3c_hwif->table[i].addr, ata_base + S3C_ATA_TBUF_START);
		} else {
			DbgAta("unexpected interrupt : 0x%x\n", reg);
			return 1;
		}

		s3c_hwif->index++;
		if (s3c_hwif->queue_size == s3c_hwif->index) {
			s3c_hwif->pseudo_dma = 0;
			DbgAta("UDMA close : s3c_hwif->queue_size == s3c_hwif->index\n");
		}

		writel(ATA_CMD_CONTINUE, ata_base + S3C_ATA_CMD);
		return 1;
	}

	stat = readl(ata_base + S3C_BUS_FIFO_STATUS);

	if ( (stat >> 16) == 0x6 ) { /* in case of PAUSER2. */
		writel(ATA_CMD_CONTINUE, ata_base + S3C_ATA_CMD);
		return 1;
	} else if ( (stat >> 16) == 0x5 ) { /* in case of PAUSEW. */
		writel(ATA_CMD_CONTINUE, ata_base + S3C_ATA_CMD);
	}
#endif
	return 0;
}

static void s3c_ide_setup_init_value (s3c_ide_hwif_t * s3c_hwif)
{
	ulong t1, t2, teoc;
	ulong uTdvh1, uTdvs, uTrp, uTss, uTackenv;
	uint pio_t1[5] = {70, 50, 30, 30, 30};
	uint pio_t2[5] = {290, 290, 290, 80, 70};
	uint pio_teoc[5] = {20, 20, 10, 10, 10};

#if 0
	uint uUdmaTdvh[5] = {10, 10, 10, 10, 10};
	uint uUdmaTdvs[5] = {100, 60, 50, 35, 20};
	uint uUdmaTrp[5] = {160, 125, 100, 100, 100};
	uint uUdmaTss[5] = {50, 50, 50, 50, 50};
	uint uUdmaTackenvMin[5] = {20, 20, 20, 20, 20};
#else
	u32 uUdmaTdvh[5] = { 7, 7, 7, 7, 7 };
	u32 uUdmaTdvs[5] = { 70, 48, 31, 20, 7 };
	u32 uUdmaTrp[5] = { 160, 125, 100, 100, 100 };
	u32 uUdmaTss[5] = { 50, 50, 50, 50, 50 };
	u32 uUdmaTackenvMin[5] = { 20, 20, 20, 20, 20 };
#endif
	ulong cycle_time = (uint) (1000000000 / clk_get_rate(s3c_hwif->clk));

	uint i;

	for (i = 0; i < 5; i++) {
		t1 = (pio_t1[i] / cycle_time) & 0xff;
		t2 = (pio_t2[i] / cycle_time) & 0xff;
		teoc = (pio_teoc[i] / cycle_time) & 0x0f;
		s3c_hwif->piotime[i] = (teoc << 12) | (t2 << 4) | t1;
		DbgAta("PIO%dTIME = %08lx\n", i, s3c_hwif->piotime[i]);
	}

	for (i = 0; i < 5; i++) {
		uTdvh1 = (uUdmaTdvh[i] / cycle_time) & 0x0f;
		uTdvs = (uUdmaTdvs[i] / cycle_time) & 0xff;
		uTrp = (uUdmaTrp[i] / cycle_time) & 0xff;
		uTss = (uUdmaTss[i] / cycle_time) & 0x0f;
		uTackenv = (uUdmaTackenvMin[i] / cycle_time) & 0x0f;
		s3c_hwif->udmatime[i] = (uTdvh1 << 24) | (uTdvs << 16) | (uTrp << 8) | (uTss << 4) | uTackenv;
		DbgAta("UDMA%dTIME = %08lx\n", i, s3c_hwif->udmatime[i]);
	}
}

static void s3c_ide_setup_ports (hw_regs_t * hw, s3c_ide_hwif_t * s3c_hwif)
{
	int i;
	unsigned long *ata_regs = hw->io_ports;

	for (i = 0; i < IDE_IRQ_OFFSET; i++) {
		*ata_regs++ = (u32)ata_base + 0x1954 + (i << 2);
	}
}

/*=========================================================================
 *          	       ata controller register I/O fuctions
 *=========================================================================
 */

static void s3c_ide_OUTB (u8 addr, ulong reg)
{
	wait_for_host_ready();
	writeb(addr, reg);
}

static void s3c_ide_OUTBSYNC (ide_drive_t *drive, u8 addr, ulong port)
{
	wait_for_host_ready();
	writeb(addr, port);
}

static void s3c_ide_OUTW (u16 addr, ulong reg)
{
	wait_for_host_ready();
	writew(addr, reg);
}

static u8 s3c_ide_INB (ulong reg)
{
	u8 temp;

	wait_for_host_ready();
	temp = readb(reg);
	wait_for_host_ready();
	temp = readb(ata_base + S3C_ATA_PIO_RDATA);
	return temp;
}

static u16 s3c_ide_INW (ulong reg)
{
	u16 temp;

	wait_for_host_ready();
	temp = readw(reg);
	wait_for_host_ready();
	temp = readw(ata_base + S3C_ATA_PIO_RDATA);
	return temp;
}

#ifdef CONFIG_BLK_DEV_IDE_S3C_PDMA
/*
 * check transfer status and return result
 */
static int check_dma_done (void)
{
	u32 reg;
	uint i;

	for (i=0; i<0x10000000; i++) {
		reg = readl(ata_base + S3C_ATA_IRQ);
		if (reg & 0x01)
			break;
	}
	if (i > (0x10000000-5)) {
		printk("transmission timeout: %08x\n", reg);
		return -1;
	}

	reg = readl(ata_base + S3C_ATA_STATUS);
	if (reg & 3) {
		printk("\n\ntransmission failed: %08x\n", reg);
		printk("ata_irq: %08x\n", readl(ata_base + S3C_ATA_IRQ));
		printk("ata_cfg: %08x\n", readl(ata_base + S3C_ATA_CFG));
		return -1;
	}

	return 0;
}

static void s3c_ide_OUTSW (ulong port, void *addr, u32 count)
{
	dma_addr_t dma_addr;

	/* half-word count. we must double it. by scsuh */
	count = count << 1;

	DbgAta("@@@@@write: %p, %08x\n", addr, count);

	writel(0x000000ff, ata_base + S3C_ATA_IRQ_MSK);

	dma_addr = dma_map_single(NULL, addr, count, DMA_TO_DEVICE);
	writel(dma_addr, ata_base + S3C_ATA_SBUF_START);
	writel(count-1, ata_base + S3C_ATA_SBUF_SIZE);
	writel(count, ata_base + S3C_ATA_XFR_NUM);
	set_config_mode(PIO_DMA, 1);
	set_trans_command(ATA_CMD_START);
	check_dma_done();
	set_config_mode(PIO_CPU, 0);
	dma_unmap_single(NULL, dma_addr, count, DMA_TO_DEVICE);

	writel(0x0000001b, ata_base + S3C_ATA_IRQ);
	writel(0x0000001b, ata_base + S3C_ATA_IRQ_MSK);
}

static void s3c_ide_INSW (ulong port, void *addr, u32 count)
{
	dma_addr_t dma_addr;

	/* half-word count. we must double it. by scsuh */
	count = count << 1;

	DbgAta("@@@@@read: %p, %08x\n", addr, count);

	writel(0x000000ff, ata_base + S3C_ATA_IRQ_MSK);

	dma_addr = dma_map_single(NULL, addr, count, DMA_FROM_DEVICE);
	writel(dma_addr, ata_base + S3C_ATA_TBUF_START);
	writel(count-1, ata_base + S3C_ATA_TBUF_SIZE);
	writel(count, ata_base + S3C_ATA_XFR_NUM);
	set_config_mode(PIO_DMA, 0);
	set_trans_command(ATA_CMD_START);
	check_dma_done();
	set_config_mode(PIO_CPU, 0);
	dma_unmap_single(NULL, dma_addr, count, DMA_FROM_DEVICE);

	writel(0x0000001b, ata_base + S3C_ATA_IRQ);
	writel(0x0000001b, ata_base + S3C_ATA_IRQ_MSK);
}
#else
static void s3c_ide_OUTSW(ulong port, void *addr, u32 count)
{
	uint i;
	volatile u16 *temp_addr = (u16*)addr;

	for (i=0; i<count; i++, temp_addr++) {
		wait_for_host_ready();
		writel(*temp_addr, port);
	}
}

static void s3c_ide_INSW (ulong port, void *addr, u32 count)
{
	uint i;
	volatile u16 *temp_addr = (u16*)addr;

	for (i=0; i<count; i++, temp_addr++) {
		wait_for_host_ready();
		*temp_addr = readw(port);
		wait_for_host_ready();
		*temp_addr = readw(S3C_ATA_PIO_RDATA);
	}
}
#endif

static void change_mode_to_ata (void)
{
	/* TDelay 1 unit = 10us */
	mdelay(1);

	/* CF controller - True IDE mode setting */

	/* Output pad disable, Card power off, ATA mode */
	writel(0x07, ata_base + S3C_CFATA_MUX);
	mdelay(10);
	/* Output pad enable, Card power off, ATA mode */
	writel(0x03, ata_base + S3C_CFATA_MUX);
	mdelay(10);
	/* Output pad enable, Card power on, ATA mode */
	writel(0x01, ata_base + S3C_CFATA_MUX);
	mdelay(500);	/* wait for 500ms */
}

static int s3c_ide_probe (struct platform_device *pdev)
{
	s3c_ide_hwif_t *s3c_hwif = &s3c_ide_hwif;
	ide_hwif_t *hwif;
	hw_regs_t *hw;
	int ret = 0;
	ulong reg;
	u8 idx[4] = { 0xff, 0xff, 0xff, 0xff };

#ifdef CONFIG_BLK_DEV_IDE_S3C_UDMA
	char *mode = "PIO/UDMA";
#elif defined (CONFIG_BLK_DEV_IDE_S3C_PDMA)
	char *mode = "PIO_ONLY(DMA)";
#else
	char *mode = "PIO_ONLY";
#endif

	memset(&s3c_ide_hwif, 0, sizeof(s3c_ide_hwif));

	s3c_hwif->dev = pdev;
	s3c_hwif->irq = platform_get_irq(pdev, 0);

//	s3c_hwif->regbase = (u32)S3C24XX_VA_CFATA;
	ata_base = ioremap(S3C_PA_CFATA, S3C_SZ_CFATA);

	s3c_hwif->clk = clk_get(&pdev->dev, "cfata");
	if (IS_ERR(s3c_hwif->clk)) {
		printk("failed to find clock source.\n");
		ret = PTR_ERR(s3c_hwif->clk);
		s3c_hwif->clk = NULL;
		goto out;
	}

	if ((ret = clk_enable(s3c_hwif->clk))) {
		printk("failed to enable clock source.\n");
		goto out;
	}

	s3c_ide_setup_init_value(s3c_hwif);

	/* XXX: Because this code is 2443 specific
	 *      it will be moved to new functions
	 *      by scsuh.
	 */
#if defined CONFIG_ARCH_S3C2443 || defined (CONFIG_CPU_S3C2450) 
	{
		ulong *temp_adr;
		mdelay(1);	// TDelay 1 unit = 10us

		// GPIO, EBI setting
		temp_adr = S3C24XX_VA_EBI+8;
		reg = readl(temp_adr);
		writel(reg | (1 << 10) | (1 << 9), temp_adr);
		reg = readl(S3C2410_GPGCON);
		writel(reg | (3 << 30) | (3 << 28) | (3 << 26) | (3 << 24) | (3 << 22), S3C2410_GPGCON);
		reg = readl(S3C2410_GPACON);
		writel(reg | (1 << 27) | (1 << 11) | (1 << 14) | (1 << 13), S3C2410_GPACON);
		reg = readl(S3C2410_MISCCR);
		writel(reg & (~(1 << 30)), S3C2410_MISCCR);

		init_buf_ctrl();
		change_board_buf_mode(PIO_CPU, 0);

		/*  CF controller - True IDE mode setting */
		change_mode_to_ata();

		/*  Card configuration */
		writel(0x1C238, ata_base + S3C_ATA_PIO_TIME);
		writel(0x20B1362, ata_base + S3C_ATA_UDMA_TIME);
		set_endian_mode(1);
		set_ata_enable(1);
		mdelay(400);		// wait for 200ms, be positively necessary
	}
#elif defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)
	{
		mdelay(1);	// TDelay 1 unit = 10us

		reg = readl(S3C_MEM_SYS_CFG) & (~0x0000003f);
		writel(reg | 0x00000030, S3C_MEM_SYS_CFG);

#ifdef CONFIG_BLK_DEV_IDE_S3C_DIRECT
		{ /* direct mode */
			u32 temp;

			printk(" * CF EBI MODE : Direct Mode\n\n");
			reg = readl(S3C_MEM_SYS_CFG);
			writel(reg | 0x00004000, S3C_MEM_SYS_CFG);

			temp = readl(S3C_GPACON);

			// D: Set XhiDATA[15:0] pins as CF Data[15:0]
			writel(0x55555555, S3C_GPK0CON);
			writel(0x55555555, S3C_GPK1CON);
			// A: Set XhiADDR[2:0] pins as CF ADDR[2:0]
			reg = readl(S3C_GPL0CON) & (~0x00000fff);
			writel(reg | 0x00000666, S3C_GPL0CON);
			// C: Set Xhi control pins as CF control pins(IORDY, IOWR, IORD, CE[1], CE[0])
			writel(0x00066666, S3C_GPMCON);

			/* This line must be here because Uart ports(GPIO_A)
			 * are changed after setting GPIO_K.
			 */
			writel(temp, S3C_GPACON);
		}
#else
		{ /* indirect mode */
			printk(" * CF EBI MODE : InDirect Mode\n\n");
			reg = readl(S3C_MEM_SYS_CFG) & (~0x00004000);
			writel(reg, S3C_MEM_SYS_CFG);

			// C: Set Xhi control pins as CF control pins(IORDY, IOWR, IORD, CE[1], CE[0])
			writel(0x00066666, S3C_GPMCON);
		}
#endif

		change_mode_to_ata();

		writel(0x1C238, ata_base + S3C_ATA_PIO_TIME); /* 0x1C238 */
		writel(0x20B1362, ata_base + S3C_ATA_UDMA_TIME); /* 0x20B1362 */
		set_endian_mode(1);
		set_ata_enable(1);
		mdelay(400);	// wait for 200ms, be positively necessary

#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)
		reg = readl(S3C_GPNCON) & ~(0x3<<(2*8));
		writel(reg | (1<<(2*8)), S3C_GPNCON);

		reg = readl(S3C_GPNDAT);
		writel(reg | (1<<8), S3C_GPNDAT);

		mdelay(500);

		writel(reg & ~(1<<8), S3C_GPNDAT);
#endif
	}
#endif
	/* remove IRQ Status */
	writel(0x1f, ata_base + S3C_ATA_IRQ);
	writel(0x1f, ata_base + S3C_ATA_IRQ);
	writel(0x1b, ata_base + S3C_ATA_IRQ_MSK);

	hwif = &ide_hwifs[pdev->id];
	hw = &hwif->hw;
	hwif->irq = hw->irq = s3c_hwif->irq;
	hwif->chipset = ide_s3c;
	s3c_ide_setup_ports(hw, s3c_hwif);
	memcpy(hwif->io_ports, hw->io_ports, sizeof(hwif->io_ports));

	hwif->ultra_mask = 0x0;	/* Disable Ultra DMA */
	hwif->mwdma_mask = 0x0;
	hwif->swdma_mask = 0x0;

	hwif->noprobe = 0;
	hwif->drives[0].unmask = 1;
	hwif->drives[1].unmask = 1;

	/* hold should be on in all cases */
	hwif->hold = 1;
	hwif->mmio = 1;

	hwif->OUTB = &s3c_ide_OUTB;
	hwif->OUTBSYNC = &s3c_ide_OUTBSYNC;
	hwif->OUTW = &s3c_ide_OUTW;
	hwif->OUTSW = &s3c_ide_OUTSW;

	hwif->INB = &s3c_ide_INB;
	hwif->INW = &s3c_ide_INW;
	hwif->INSW = &s3c_ide_INSW;

	hwif->pio_mask = XFER_PIO_4;
	hwif->set_pio_mode = &s3c_ide_tune_drive;
	hwif->set_dma_mode = &s3c_ide_tune_chipset;

#ifdef CONFIG_IDE_HOOK_IRQ
	hwif->ide_irq_hook = &s3c_ide_irq_hook;
#endif

#if defined (CONFIG_BLK_DEV_IDE_S3C_UDMA)
	hwif->ultra_mask = 0x1f;	/* enable Ultra DMA 0 ~ 4 */ 
	hwif->dma_off_quietly = &s3c_ide_dma_off_quietly;
	hwif->dma_timeout = &s3c_ide_dma_timeout;

	hwif->dma_exec_cmd = &s3c_ide_dma_exec_cmd;
	hwif->dma_start = &s3c_ide_dma_start;
	hwif->ide_dma_end = &s3c_ide_dma_end;
	hwif->dma_setup = &s3c_ide_dma_setup;
	hwif->ide_dma_test_irq = &s3c_ide_dma_test_irq;
	hwif->dma_host_off = &s3c_ide_dma_host_off;
	hwif->dma_host_on = &s3c_ide_dma_host_on;
	hwif->dma_lost_irq = &s3c_ide_dma_lostirq;
	hwif->ide_dma_on = &s3c_ide_dma_on;

#else
	hwif->channel = 0;
	hwif->hold = 1;
	hwif->select_data = 0;	/* no chipset-specific code */
	hwif->config_data = 0;	/* no chipset-specific code */
#endif

	hwif->drives[0].autotune = 1;	/* 1=autotune, 2=noautotune, 0=default */
	hwif->drives[0].no_io_32bit = 1;

	s3c_ide_hwif.hwif = hwif;
	hwif->hwif_data = &s3c_ide_hwif;
	
	idx[0] = hwif->index;
	ide_device_add(idx);
	
	dev_set_drvdata(pdev, hwif);
	
	printk(KERN_INFO "S3C24XX IDE(builtin) configured for %s\n", mode);

out:
	return ret;
}

static int s3c_ide_remove(struct platform_device *dev)
{
	struct resource *res;
	ide_hwif_t *hwif = platform_get_drvdata(dev);

	ide_unregister(hwif - ide_hwifs);

	iounmap(ata_base);

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start);

	return 0;
}

static struct platform_driver s3c_ide_driver = {
	.probe = s3c_ide_probe,
	.remove = s3c_ide_remove,
	.driver = {
		.name = "s3c-ide",
	},
};


static int __init s3c_ide_init(void)
{
	return platform_driver_register(&s3c_ide_driver);
}

static void __exit s3c_ide_exit(void)
{
	platform_driver_unregister(&s3c_ide_driver);
}

module_init(s3c_ide_init);
module_exit(s3c_ide_exit);

static int s3c_ata_proc_read (
 char *buffer,
 char **buffer_location,
 off_t offset,
 int buffer_length,
 int *zero,
 void *ptr
)
{
#ifdef CONFIG_BLK_DEV_IDE_S3C_UDMA
	char *mode = "PIO/UDMA";
#elif defined (CONFIG_BLK_DEV_IDE_S3C_PDMA)
	char *mode = "PIO_ONLY(DMA)";
#else
	char *mode = "PIO_ONLY";
#endif

	printk("current mode is %s\n", mode);
	return 0;
}

static int s3c_ata_proc_write (
 struct file *file,
 const char *buffer,
 unsigned long count,
 void *data
)
{
	return 0;
}

static struct proc_dir_entry *evb_resource_dump;

int __init s3c_ata_rw_proc (void)
{
    evb_resource_dump = create_proc_entry("s3c-ata", 0666, &proc_root);
    evb_resource_dump->read_proc = s3c_ata_proc_read;
    evb_resource_dump->write_proc = s3c_ata_proc_write;
    evb_resource_dump->nlink = 1;
    return 0;
}
module_init(s3c_ata_rw_proc);
MODULE_DESCRIPTION("S3C ATA in CF/ATA Driver");
MODULE_LICENSE("GPL");
