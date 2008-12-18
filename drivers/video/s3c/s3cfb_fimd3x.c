/*
 * drivers/video/s3c/s3c24xxfb.c
 *
 * $Id: s3cfb_fimd3x.c,v 1.11 2008/09/18 07:27:06 jsgood Exp $
 *
 * Copyright (C) 2008 Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <asm/mach/map.h>
#include <asm/arch/regs-lcd.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-s3c-clock.h>

#if defined(CONFIG_PM)
#include <asm/plat-s3c24xx/pm.h>
#endif

#include "s3cfb.h"

int s3c_vs_offset = S3C_FB_DEFAULT_DISPLAY_OFFSET;
int s3c_osd_alpha_level = S3C_FB_MAX_ALPHA_LEVEL;
int s3c_display_brightness = S3C_FB_DEFAULT_BRIGHTNESS;
static int s3c_palette_win;

s3c_fimd_info_t s3c_fimd = {
	.vidcon0 = S3C_VIDCON0_VIDOUT_RGB_IF | S3C_VIDCON0_PNRMODE_RGB_P | S3C_VIDCON0_CLKDIR_DIVIDED | S3C_VIDCON0_VCLKEN_ENABLE | S3C_VIDCON0_CLKSEL_F_HCLK,

#if defined (CONFIG_FB_S3C_BPP_8)
	.wincon0 =  S3C_WINCONx_BYTSWP_ENABLE | S3C_WINCONx_BURSTLEN_4WORD | S3C_WINCONx_BPPMODE_F_8BPP_PAL,
	.wincon1 =  S3C_WINCONx_HAWSWP_ENABLE | S3C_WINCONx_BURSTLEN_4WORD | S3C_WINCONx_BPPMODE_F_16BPP_565 | S3C_WINCONx_BLD_PIX_PLANE | S3C_WINCONx_ALPHA_SEL_1,
	.bpp = S3C_FB_PIXEL_BPP_8,
	.bytes_per_pixel = 1,
	.wpalcon = S3C_WPALCON_W0PAL_16BIT,

#elif defined (CONFIG_FB_S3C_BPP_16)
	.wincon0 =  S3C_WINCONx_HAWSWP_ENABLE | S3C_WINCONx_BURSTLEN_4WORD | S3C_WINCONx_BPPMODE_F_16BPP_565,
	.wincon1 =  S3C_WINCONx_HAWSWP_ENABLE | S3C_WINCONx_BURSTLEN_4WORD | S3C_WINCONx_BPPMODE_F_16BPP_565 | S3C_WINCONx_BLD_PIX_PLANE | S3C_WINCONx_ALPHA_SEL_1,
	.bpp = S3C_FB_PIXEL_BPP_16,
	.bytes_per_pixel = 2,
	.wpalcon = S3C_WPALCON_W0PAL_16BIT,

#elif defined (CONFIG_FB_S3C_BPP_24)
	.wincon0 =  S3C_WINCONx_HAWSWP_DISABLE | S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BPPMODE_F_24BPP_888,
	.wincon1 =  S3C_WINCONx_HAWSWP_DISABLE | S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BPPMODE_F_24BPP_888 | S3C_WINCONx_BLD_PIX_PLANE | S3C_WINCONx_ALPHA_SEL_1,
	.bpp = S3C_FB_PIXEL_BPP_24,
	.bytes_per_pixel = 4,
	.wpalcon = S3C_WPALCON_W0PAL_24BIT,
#endif

	.vidosd1c = S3C_VIDOSDxC_ALPHA1_B(S3C_FB_MAX_ALPHA_LEVEL) | S3C_VIDOSDxC_ALPHA1_G(S3C_FB_MAX_ALPHA_LEVEL) |S3C_VIDOSDxC_ALPHA1_R(S3C_FB_MAX_ALPHA_LEVEL),
	.vidintcon = S3C_VIDINTCON0_FRAMESEL0_VSYNC | S3C_VIDINTCON0_FRAMESEL1_NONE | S3C_VIDINTCON0_INTFRMEN_ENABLE | S3C_VIDINTCON0_INTEN_ENABLE,

	.xoffset = 0,
	.yoffset = 0,

	.w1keycon0 = S3C_WxKEYCON0_KEYBLEN_DISABLE | S3C_WxKEYCON0_KEYEN_F_DISABLE | S3C_WxKEYCON0_DIRCON_MATCH_FG_IMAGE | S3C_WxKEYCON0_COMPKEY(0x0),
	.w1keycon1 = S3C_WxKEYCON1_COLVAL(0xffffff),
	.w2keycon0 = S3C_WxKEYCON0_KEYBLEN_DISABLE | S3C_WxKEYCON0_KEYEN_F_DISABLE | S3C_WxKEYCON0_DIRCON_MATCH_FG_IMAGE | S3C_WxKEYCON0_COMPKEY(0x0),
	.w2keycon1 = S3C_WxKEYCON1_COLVAL(0xffffff),
	.w3keycon0 = S3C_WxKEYCON0_KEYBLEN_DISABLE | S3C_WxKEYCON0_KEYEN_F_DISABLE | S3C_WxKEYCON0_DIRCON_MATCH_FG_IMAGE | S3C_WxKEYCON0_COMPKEY(0x0),
	.w3keycon1 = S3C_WxKEYCON1_COLVAL(0xffffff),
	.w4keycon0 = S3C_WxKEYCON0_KEYBLEN_DISABLE | S3C_WxKEYCON0_KEYEN_F_DISABLE | S3C_WxKEYCON0_DIRCON_MATCH_FG_IMAGE | S3C_WxKEYCON0_COMPKEY(0x0),
	.w4keycon1 = S3C_WxKEYCON1_COLVAL(0xffffff),

      	.sync = 0,
	.cmap_static = 1,
};

#if defined(CONFIG_S3C2450_PWM) || defined(CONFIG_S3C2416_PWM)
void s3cfb_set_brightness(int val)
{
	int channel = 3;	/* must use channel-3 */
	int usec = 0;		/* don't care value */
	unsigned long tcnt = 1000;
	unsigned long tcmp = 0;

	if (val < 0)
		val = 0;

	if (val > S3C_FB_MAX_BRIGHTNESS)
		val = S3C_FB_MAX_BRIGHTNESS;

	s3c_display_brightness = val;
	tcmp = val * 50;

	s3c2450_timer_setup (channel, usec, tcnt, tcmp);
}
#endif

#if defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)

static void s3cfb_change_buff(int req_win, int req_fb)
{
	/* Software-based trigger */
	writel((1 << 0), S3C_CPUTRIGCON2);
}

#endif

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
static int s3cfb_set_vs_registers(int vs_cmd)
{
	int page_width, offset;
	int shift_value;

	page_width = s3c_fimd.xres * s3c_fimd.bytes_per_pixel;
	offset = (s3c_fimd.xres_virtual - s3c_fimd.xres) * s3c_fimd.bytes_per_pixel;

	switch (vs_cmd){
	case S3C_FB_VS_SET:
		/* size of buffer */
		s3c_fimd.vidw00add2b0 = S3C_VIDWxxADD2_OFFSIZE_F(offset) | S3C_VIDWxxADD2_PAGEWIDTH_F(page_width);
		s3c_fimd.vidw00add2b1 = S3C_VIDWxxADD2_OFFSIZE_F(offset) | S3C_VIDWxxADD2_PAGEWIDTH_F(page_width);

		writel(s3c_fimd.vidw00add2b0, S3C_VIDW00ADD2B0);
		writel(s3c_fimd.vidw00add2b1, S3C_VIDW00ADD2B1);
		break;

	case S3C_FB_VS_MOVE_LEFT:
		if (s3c_fimd.xoffset < s3c_vs_offset)
			shift_value = s3c_fimd.xoffset;
		else
			shift_value = s3c_vs_offset;

		s3c_fimd.xoffset -= shift_value;

		/* For buffer start address */
		s3c_fimd.vidw00add0b0 = s3c_fimd.vidw00add0b0 - (s3c_fimd.bytes_per_pixel * shift_value);
		s3c_fimd.vidw00add0b1 = s3c_fimd.vidw00add0b1 - (s3c_fimd.bytes_per_pixel * shift_value);
		break;

	case S3C_FB_VS_MOVE_RIGHT:
		if ((s3c_vs_info.v_width - (s3c_fimd.xoffset + s3c_vs_info.width)) < (s3c_vs_offset))
			shift_value = s3c_vs_info.v_width - (s3c_fimd.xoffset + s3c_vs_info.width);
		else
			shift_value = s3c_vs_offset;

		s3c_fimd.xoffset += shift_value;

		/* For buffer start address */
		s3c_fimd.vidw00add0b0 = s3c_fimd.vidw00add0b0 + (s3c_fimd.bytes_per_pixel * shift_value);
		s3c_fimd.vidw00add0b1 = s3c_fimd.vidw00add0b1 + (s3c_fimd.bytes_per_pixel * shift_value);
		break;

	case S3C_FB_VS_MOVE_UP:
		if (s3c_fimd.yoffset < s3c_vs_offset)
			shift_value = s3c_fimd.yoffset;
		else
			shift_value = s3c_vs_offset;

		s3c_fimd.yoffset -= shift_value;

		/* For buffer start address */
		s3c_fimd.vidw00add0b0 = s3c_fimd.vidw00add0b0 - (s3c_fimd.xres_virtual * s3c_fimd.bytes_per_pixel * shift_value);
		s3c_fimd.vidw00add0b1 = s3c_fimd.vidw00add0b1 - (s3c_fimd.xres_virtual * s3c_fimd.bytes_per_pixel * shift_value);
		break;

	case S3C_FB_VS_MOVE_DOWN:
		if ((s3c_vs_info.v_height - (s3c_fimd.yoffset + s3c_vs_info.height)) < (s3c_vs_offset))
			shift_value = s3c_vs_info.v_height - (s3c_fimd.yoffset + s3c_vs_info.height);
		else
			shift_value = s3c_vs_offset;

		s3c_fimd.yoffset += shift_value;

		/* For buffer start address */
		s3c_fimd.vidw00add0b0 = s3c_fimd.vidw00add0b0 + (s3c_fimd.xres_virtual * s3c_fimd.bytes_per_pixel * shift_value);
		s3c_fimd.vidw00add0b1 = s3c_fimd.vidw00add0b1 + (s3c_fimd.xres_virtual * s3c_fimd.bytes_per_pixel * shift_value);
		break;

	default:
		return -EINVAL;
	}

	/* End address */
	s3c_fimd.vidw00add1b0 = S3C_VIDWxxADD1_VBASEL_F(s3c_fimd.vidw00add0b0 + (page_width + offset) * (s3c_fimd.yres));
	s3c_fimd.vidw00add1b1 = S3C_VIDWxxADD1_VBASEL_F(s3c_fimd.vidw00add0b1 + (page_width + offset) * (s3c_fimd.yres));

	writel(s3c_fimd.vidw00add0b0, S3C_VIDW00ADD0B0);
	writel(s3c_fimd.vidw00add0b1, S3C_VIDW00ADD0B1);
	writel(s3c_fimd.vidw00add1b0, S3C_VIDW00ADD1B0);
	writel(s3c_fimd.vidw00add1b1, S3C_VIDW00ADD1B1);

	return 0;
}
#endif

void s3cfb_write_palette(s3c_fb_info_t *fbi)
{
	unsigned int i;
	unsigned long ent;

	fbi->palette_ready = 0;

	writel((s3c_fimd.wpalcon | S3C_WPALCON_PALUPDATEEN), S3C_WPALCON);

	for (i = 0; i < 256; i++) {
		if ((ent = fbi->palette_buffer[i]) == S3C_FB_PALETTE_BUFF_CLEAR)
			continue;

		writel(ent, S3C_TFTPAL0(i));

		/* it seems the only way to know exactly
		 * if the palette wrote ok, is to check
		 * to see if the value verifies ok
		 */
		if (readl(S3C_TFTPAL0(i)) == ent) {
			fbi->palette_buffer[i] = S3C_FB_PALETTE_BUFF_CLEAR;
		} else {
			fbi->palette_ready = 1;   /* retry */
			printk("Retry writing into the palette\n");
		}
	}

	writel(s3c_fimd.wpalcon, S3C_WPALCON);
}

irqreturn_t s3cfb_irq(int irqno, void *param)
{
	if (s3c_fb_info[s3c_palette_win].palette_ready)
		s3cfb_write_palette(&s3c_fb_info[s3c_palette_win]);

	s3c_vsync_info.count++;
	wake_up_interruptible(&s3c_vsync_info.wait_queue);

	return IRQ_HANDLED;
}

int s3cfb_init_registers(s3c_fb_info_t *fbi)
{
	struct clk *lcd_clock;
	struct fb_var_screeninfo *var = &fbi->fb.var;
	unsigned long flags = 0, page_width = 0, offset = 0;
	unsigned long video_phy_temp_f1 = fbi->screen_dma_f1;
	unsigned long video_phy_temp_f2 = fbi->screen_dma_f2;
	int win_num =  fbi->win_id;

	/* Initialise LCD with values from hare */
	local_irq_save(flags);

	page_width = var->xres * s3c_fimd.bytes_per_pixel;
	offset = (var->xres_virtual - var->xres) * s3c_fimd.bytes_per_pixel;

	if (win_num == 0) {
		s3c_fimd.vidcon0 = s3c_fimd.vidcon0 & ~(S3C_VIDCON0_ENVID_ENABLE | S3C_VIDCON0_ENVID_F_ENABLE);
		writel(s3c_fimd.vidcon0, S3C_VIDCON0);

		lcd_clock = clk_get(NULL, "lcd");
		s3c_fimd.vidcon0 |= S3C_VIDCON0_CLKVAL_F((int) ((clk_get_rate(lcd_clock) / s3c_fimd.pixclock) - 1));

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
		offset = 0;
		s3c_fimd.vidw00add0b0 = video_phy_temp_f1;
		s3c_fimd.vidw00add0b1 = video_phy_temp_f2;
		s3c_fimd.vidw00add1b0 = S3C_VIDWxxADD1_VBASEL_F((unsigned long) video_phy_temp_f1 + (page_width + offset) * (var->yres));
		s3c_fimd.vidw00add1b1 = S3C_VIDWxxADD1_VBASEL_F((unsigned long) video_phy_temp_f2 + (page_width + offset) * (var->yres));
#endif

		writel(video_phy_temp_f2, S3C_VIDW00ADD0B1);
		writel(S3C_VIDWxxADD1_VBASEL_F((unsigned long) video_phy_temp_f2 + (page_width + offset) * (var->yres)), S3C_VIDW00ADD1B1);
		writel(S3C_VIDWxxADD2_OFFSIZE_F(offset) | (S3C_VIDWxxADD2_PAGEWIDTH_F(page_width)), S3C_VIDW00ADD2B1);
 	}

	writel(video_phy_temp_f1, S3C_VIDW00ADD0B0 + (0x08 * win_num));
	writel(S3C_VIDWxxADD1_VBASEL_F((unsigned long) video_phy_temp_f1 + (page_width + offset) * (var->yres)), S3C_VIDW00ADD1B0 + (0x08 * win_num));
	writel(S3C_VIDWxxADD2_OFFSIZE_F(offset) | (S3C_VIDWxxADD2_PAGEWIDTH_F(page_width)), S3C_VIDW00ADD2B0 + (0x08 * win_num));

	switch (win_num) {
	case 0:
		writel(s3c_fimd.wincon0, S3C_WINCON0);
		writel(s3c_fimd.vidcon0, S3C_VIDCON0);
		writel(s3c_fimd.vidcon1, S3C_VIDCON1);
		writel(s3c_fimd.vidtcon0, S3C_VIDTCON0);
		writel(s3c_fimd.vidtcon1, S3C_VIDTCON1);
		writel(s3c_fimd.vidtcon2, S3C_VIDTCON2);
		writel(s3c_fimd.vidintcon, S3C_VIDINTCON);
		writel(s3c_fimd.vidosd0a, S3C_VIDOSD0A);
		writel(s3c_fimd.vidosd0b, S3C_VIDOSD0B);
		writel(s3c_fimd.wpalcon, S3C_WPALCON);

		s3cfb_onoff_win(fbi, ON);
		break;

	case 1:
		writel(s3c_fimd.wincon1, S3C_WINCON1);
		writel(s3c_fimd.vidosd1a, S3C_VIDOSD1A);
		writel(s3c_fimd.vidosd1b, S3C_VIDOSD1B);
		writel(s3c_fimd.vidosd1c, S3C_VIDOSD1C);
		writel(s3c_fimd.wpalcon, S3C_WPALCON);

		s3cfb_onoff_win(fbi, OFF);
		break;
	}

	local_irq_restore(flags);

	return 0;
 }

void s3cfb_activate_var(s3c_fb_info_t *fbi, struct fb_var_screeninfo *var)
{
	DPRINTK("%s: var->bpp = %d\n", __FUNCTION__, var->bits_per_pixel);

	switch (var->bits_per_pixel) {
	case 8:
		s3c_fimd.wincon0 = S3C_WINCONx_BYTSWP_ENABLE | S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BPPMODE_F_8BPP_PAL;
		s3c_fimd.wincon1 = S3C_WINCONx_HAWSWP_ENABLE | S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BPPMODE_F_16BPP_565 | S3C_WINCONx_BLD_PIX_PLANE | S3C_WINCONx_ALPHA_SEL_1;
		s3c_fimd.bpp = S3C_FB_PIXEL_BPP_8;
		s3c_fimd.bytes_per_pixel = 1;
		s3c_fimd.wpalcon = S3C_WPALCON_W0PAL_24BIT;
		break;

	case 16:
		s3c_fimd.wincon0 = S3C_WINCONx_HAWSWP_ENABLE | S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BPPMODE_F_16BPP_565;
		s3c_fimd.wincon1 = S3C_WINCONx_HAWSWP_ENABLE | S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BPPMODE_F_16BPP_565 | S3C_WINCONx_BLD_PIX_PLANE | S3C_WINCONx_ALPHA_SEL_1;
		s3c_fimd.bpp = S3C_FB_PIXEL_BPP_16;
		s3c_fimd.bytes_per_pixel = 2;
		break;

	case 24:
		s3c_fimd.wincon0 = S3C_WINCONx_HAWSWP_DISABLE | S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BPPMODE_F_24BPP_888;
		s3c_fimd.wincon1 = S3C_WINCONx_HAWSWP_DISABLE | S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BPPMODE_F_24BPP_888 | S3C_WINCONx_BLD_PIX_PLANE | S3C_WINCONx_ALPHA_SEL_1;
        	s3c_fimd.bpp = S3C_FB_PIXEL_BPP_24;
		s3c_fimd.bytes_per_pixel = 4;
		break;

	case 32:
		s3c_fimd.bytes_per_pixel = 4;
		break;
	}

	/* write new registers */
	writel(s3c_fimd.wincon0, S3C_WINCON0);
	writel(s3c_fimd.wincon1, S3C_WINCON1);
	writel(s3c_fimd.wpalcon, S3C_WPALCON);
	writel(s3c_fimd.wincon0 | S3C_WINCONx_ENWIN_F_ENABLE, S3C_WINCON0);
	writel(s3c_fimd.vidcon0 | S3C_VIDCON0_ENVID_ENABLE | S3C_VIDCON0_ENVID_F_ENABLE, S3C_VIDCON0);
}

/* JJNAHM comment.
 * We had some problems related to frame buffer address.
 * We used 2 frame buffers (FB0 and FB1) and GTK used FB1.
 * When GTK launched, GTK set FB0's address to FB1's address.
 * (GTK calls s3c_fb_pan_display() and then it calls this s3c_fb_set_lcdaddr())
 * Even though fbi->win_id is not 0, above original codes set ONLY FB0's address.
 * So, I modified the codes like below.
 * It works by fbi->win_id value.
 * Below codes are not verified yet
 * and there are nothing about Double buffering features
 */
void s3cfb_set_fb_addr(s3c_fb_info_t *fbi)
{
	unsigned long video_phy_temp_f1 = fbi->screen_dma_f1;
	unsigned long start_address, end_address;
	unsigned int start;

	start = fbi->fb.fix.line_length * fbi->fb.var.yoffset;

	/* for buffer start address and end address */
	start_address = video_phy_temp_f1 + start;
	end_address = start_address + (fbi->fb.fix.line_length * fbi->fb.var.yres);

	switch (fbi->win_id)
	{
	case 0:
		s3c_fimd.vidw00add0b0 = start_address;
		s3c_fimd.vidw00add1b0 = end_address;
		__raw_writel(s3c_fimd.vidw00add0b0, S3C_VIDW00ADD0B0);
		__raw_writel(s3c_fimd.vidw00add1b0, S3C_VIDW00ADD1B0);
        	break;

	case 1:
		s3c_fimd.vidw01add0 = start_address;
		s3c_fimd.vidw01add1 = end_address;
		__raw_writel(s3c_fimd.vidw01add0, S3C_VIDW01ADD0);
		__raw_writel(s3c_fimd.vidw01add1, S3C_VIDW01ADD1);
		break;

	}
}

static int s3cfb_set_alpha_level(s3c_fb_info_t *fbi, unsigned int level)
{
	unsigned long alpha_val;
	int win_num = fbi->win_id;

	if (win_num == 0) {
		printk("WIN0 do not support alpha blending.\n");
		return -1;
	}

	alpha_val = S3C_VIDOSDxC_ALPHA1_B(level) | S3C_VIDOSDxC_ALPHA1_G(level) | S3C_VIDOSDxC_ALPHA1_R(level);
	writel(alpha_val, S3C_VIDOSD1C);

	return 0;
}

#if 0
static int s3cfb_set_alpha_mode(s3c_fb_info_t *fbi, int mode, int level)
{
	unsigned long alpha_val;
	int win_num = fbi->win_id;

	if (win_num == 0) {
		printk("WIN0 do not support alpha blending.\n");
		return -1;
	}

	switch (mode) {
	case 0: /* Plane Blending */
		writel(readl(S3C_WINCON0 + (0x04 * win_num)) | S3C_WINCONx_BLD_PIX_PLANE, S3C_WINCON0 + (0x04 * win_num));
		break;

	case 1: /* Pixel Blending & chroma(color) key */
		writel(readl(S3C_WINCON0 + (0x04 * win_num)) | S3C_WINCONx_BLD_PIX_PIXEL | S3C_WINCONx_ALPHA_SEL_0, S3C_WINCON0 + (0x04 * win_num));
		break;
	}

	s3c_osd_alpha_level = level;
	alpha_val = S3C_VIDOSDxC_ALPHA1_B(s3c_osd_alpha_level) | S3C_VIDOSDxC_ALPHA1_G(s3c_osd_alpha_level) | S3C_VIDOSDxC_ALPHA1_R(s3c_osd_alpha_level);
	writel(alpha_val, S3C_VIDOSD1C);

	return 0;
}
#endif

int s3cfb_set_win_position(s3c_fb_info_t *fbi, int left_x, int top_y, int width, int height)
{
	struct fb_var_screeninfo *var= &fbi->fb.var;
	int win_num = fbi->win_id;

	writel(S3C_VIDOSDxA_OSD_LTX_F(left_x) | S3C_VIDOSDxA_OSD_LTY_F(top_y), S3C_VIDOSD0A + (0x0c * win_num));
	writel(S3C_VIDOSDxB_OSD_RBX_F(width - 1 + left_x) | S3C_VIDOSDxB_OSD_RBY_F(height - 1 + top_y), S3C_VIDOSD0B + (0x0c * win_num));

	var->xoffset = left_x;
	var->yoffset = top_y;

	return 0;
}

int s3cfb_set_win_size(s3c_fb_info_t *fbi, int width, int height)
{
	struct fb_var_screeninfo *var= &fbi->fb.var;

	var->xres = width;
	var->yres = height;
	var->xres_virtual = width;
	var->yres_virtual = height;

	return 0;
}

int s3cfb_set_fb_size(s3c_fb_info_t *fbi)
{
	struct fb_var_screeninfo *var= &fbi->fb.var;
	int win_num = fbi->win_id;
	unsigned long offset = 0;
	unsigned long page_width = 0;
	unsigned long fb_size = 0;

	page_width = var->xres * s3c_fimd.bytes_per_pixel;
	offset = (var->xres_virtual - var->xres) * s3c_fimd.bytes_per_pixel;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	if (win_num == 0)
		offset=0;
#endif

	writel(S3C_VIDWxxADD1_VBASEL_F((unsigned long) readl(S3C_VIDW00ADD0B0 + (0x08 * win_num)) + (page_width + offset) * (var->yres)), S3C_VIDW00ADD1B0 + (0x08 * win_num));

	if (win_num == 1)
		writel(S3C_VIDWxxADD1_VBASEL_F((unsigned long) readl(S3C_VIDW00ADD0B1 + (0x08 * win_num)) + (page_width + offset) * (var->yres)), S3C_VIDW00ADD1B1 + (0x08 * win_num));

	/* size of frame buffer */
	fb_size = S3C_VIDWxxADD2_OFFSIZE_F(offset) | (S3C_VIDWxxADD2_PAGEWIDTH_F(page_width));
	writel(fb_size, S3C_VIDW00ADD2B0 + (0x08 * win_num));

	if (win_num == 0)
		writel(fb_size, S3C_VIDW00ADD2B1);

	return 0;
}

int s3cfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	s3c_fb_info_t *fbi = container_of(info, s3c_fb_info_t, fb);
	s3c_win_info_t win_info;
	s3c_color_key_info_t colkey_info;
	s3c_color_val_info_t colval_info;
	s3c_fb_dma_info_t dma_info;
	struct fb_var_screeninfo *var= &fbi->fb.var;
	unsigned int crt, alpha_level;

#if defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)
	unsigned int f_num_val;
#endif

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3c_vs_info_t vs_info;
#endif

	switch(cmd){
	case S3C_FB_GET_INFO:
		dma_info.map_dma_f1 = fbi->map_dma_f1;
		dma_info.map_dma_f2 = fbi->map_dma_f2;

		if(copy_to_user((void *) arg, (const void *) &dma_info, sizeof(s3c_fb_dma_info_t)))
			return -EFAULT;
		break;

	case S3C_FB_OSD_SET_INFO:
		if (copy_from_user(&win_info, (s3c_win_info_t *) arg, sizeof(s3c_win_info_t)))
			return -EFAULT;

		s3cfb_init_win(fbi, win_info.bpp, win_info.left_x, win_info.top_y, win_info.width, win_info.height, OFF);
		break;

	case S3C_FB_OSD_START:
		s3cfb_onoff_win(fbi, ON);
		break;

	case S3C_FB_OSD_STOP:
		s3cfb_set_alpha_level(fbi, S3C_FB_MAX_ALPHA_LEVEL);
		s3cfb_onoff_win(fbi, OFF);
		break;

	case S3C_FB_OSD_ALPHA_UP:
		alpha_level = readl(S3C_VIDOSD0C + (0x10 * fbi->win_id)) & 0xf;

		if (alpha_level < S3C_FB_MAX_ALPHA_LEVEL)
			alpha_level++;

		s3cfb_set_alpha_level(fbi, alpha_level);
		break;

	case S3C_FB_OSD_ALPHA_DOWN:
		alpha_level = readl(S3C_VIDOSD0C + (0x10 * fbi->win_id)) & 0xf;

		if (alpha_level > 0)
			alpha_level--;

		s3cfb_set_alpha_level(fbi, alpha_level);
		break;

	case S3C_FB_OSD_ALPHA_SET:
		if (copy_from_user(&alpha_level, (int *) arg, sizeof(int)))
			return -EFAULT;

		if (alpha_level > S3C_FB_MAX_ALPHA_LEVEL) 
			alpha_level = S3C_FB_MAX_ALPHA_LEVEL;
		else if (alpha_level < 0) 
			alpha_level = 0;

		s3cfb_set_alpha_level(fbi, alpha_level);
		break;

	case S3C_FB_OSD_MOVE_LEFT:
		if (var->xoffset > 0)
			var->xoffset--;

		s3cfb_set_win_position(fbi, var->xoffset, var->yoffset, var->xres, var->yres);
		break;

	case S3C_FB_OSD_MOVE_RIGHT:
		if (var->xoffset < (s3c_fimd.width - var->xres))
			var->xoffset++;

		s3cfb_set_win_position(fbi, var->xoffset, var->yoffset, var->xres, var->yres);
		break;

	case S3C_FB_OSD_MOVE_UP:
		if (var->yoffset > 0)
			var->yoffset--;

		s3cfb_set_win_position(fbi, var->xoffset, var->yoffset, var->xres, var->yres);
		break;

	case S3C_FB_OSD_MOVE_DOWN:
		if (var->yoffset < (s3c_fimd.height - var->yres))
			var->yoffset++;

		s3cfb_set_win_position(fbi, var->xoffset, var->yoffset, var->xres, var->yres);
		break;

	case FBIO_WAITFORVSYNC:
		if (get_user(crt, (unsigned int __user *)arg))
			return -EFAULT;

		return s3cfb_wait_for_vsync();

	case S3C_FB_COLOR_KEY_START:
		s3cfb_onoff_color_key(fbi, ON);
		break;

	case S3C_FB_COLOR_KEY_STOP:
		s3cfb_onoff_color_key(fbi, OFF);
		break;

	case S3C_FB_COLOR_KEY_ALPHA_START:
		s3cfb_onoff_color_key_alpha(fbi, ON);
		break;

	case S3C_FB_COLOR_KEY_ALPHA_STOP:
		s3cfb_onoff_color_key_alpha(fbi, OFF);
		break;

	case S3C_FB_COLOR_KEY_SET_INFO:
		if (copy_from_user(&colkey_info, (s3c_color_key_info_t *) arg, sizeof(s3c_color_key_info_t)))
			return -EFAULT;

		s3cfb_set_color_key_registers(fbi, colkey_info);
		break;

	case S3C_FB_COLOR_KEY_VALUE:
		if (copy_from_user(&colval_info, (s3c_color_val_info_t *) arg, sizeof(s3c_color_val_info_t)))
			return -EFAULT;

		s3cfb_set_color_value(fbi, colval_info);
		break;

	case S3C_FB_SET_VSYNC_INT:
		s3c_fimd.vidintcon &= ~S3C_VIDINTCON0_FRAMESEL0_MASK;
		s3c_fimd.vidintcon |= S3C_VIDINTCON0_FRAMESEL0_VSYNC;

		if (arg)
			s3c_fimd.vidintcon |= S3C_VIDINTCON0_INTFRMEN_ENABLE;
		else
			s3c_fimd.vidintcon &= ~S3C_VIDINTCON0_INTFRMEN_ENABLE;

		writel(s3c_fimd.vidintcon, S3C_VIDINTCON);
		break;

	case S3C_FB_GET_BRIGHTNESS:
		if (copy_to_user((void *)arg, (const void *) &s3c_display_brightness, sizeof(int)))
			return -EFAULT;
		break;

#if defined(CONFIG_S3C2450_PWM) || defined(CONFIG_S3C2416_PWM)
	case S3C_FB_SET_BRIGHTNESS:
		if (copy_from_user(&brightness, (int *) arg, sizeof(int)))
			return -EFAULT;

		s3cfb_set_brightness(brightness);
		break;
#endif

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	case S3C_FB_VS_START:
		s3c_fimd.wincon0 &= ~(S3C_WINCONx_ENWIN_F_ENABLE);
		writel(s3c_fimd.wincon0 | S3C_WINCONx_ENWIN_F_ENABLE, S3C_WINCON0);

		fbi->fb.var.xoffset = s3c_fimd.xoffset;
		fbi->fb.var.yoffset = s3c_fimd.yoffset;
		break;

	case S3C_FB_VS_STOP:
		break;

	case S3C_FB_VS_SET_INFO:
		if (copy_from_user(&vs_info, (s3c_vs_info_t *) arg, sizeof(s3c_vs_info_t)))
			return -EFAULT;

		if (s3cfb_set_vs_info(vs_info)) {
			printk("Error S3C_FB_VS_SET_INFO\n");
			return -EINVAL;
		}

		s3cfb_set_vs_registers(S3C_FB_VS_SET);

		fbi->fb.var.xoffset = s3c_fimd.xoffset;
		fbi->fb.var.yoffset = s3c_fimd.yoffset;
		break;

	case S3C_FB_VS_MOVE:
		s3cfb_set_vs_registers(arg);

		fbi->fb.var.xoffset = s3c_fimd.xoffset;
		fbi->fb.var.yoffset = s3c_fimd.yoffset;
		break;
#endif

#if defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)
	case S3C_FB_GET_NUM:
		if (copy_from_user((void *)&f_num_val, (const void *)arg, sizeof(u_int)))
			return -EFAULT;

		if (copy_to_user((void *)arg, (const void *) &f_num_val, sizeof(u_int)))
			return -EFAULT;

		break;

	case S3C_FB_CHANGE_REQ:
		s3cfb_change_buff(0, (int) arg);
		break;
#endif

	default:
		return -EINVAL;
	}

	return 0;
}

void s3cfb_pre_init(void)
{
	/* initialize the fimd specific */
	s3c_fimd.vidintcon &= ~S3C_VIDINTCON0_FRAMESEL0_MASK;
	s3c_fimd.vidintcon |= S3C_VIDINTCON0_FRAMESEL0_VSYNC;
	s3c_fimd.vidintcon |= S3C_VIDINTCON0_INTFRMEN_ENABLE;

	writel(s3c_fimd.vidintcon, S3C_VIDINTCON);
}

void s3cfb_set_gpio(void)
{
	unsigned long val;

	/* enable clock to LCD */
	val = readl(S3C2443_HCLKCON);
	val |= S3C2443_HCLKCON_LCDC;
	writel(val, S3C2443_HCLKCON);

	/* select TFT LCD type */
	val = readl(S3C2410_MISCCR);
	val |= (1 << 28);
	writel(val, S3C2410_MISCCR);

	/* VD */
	writel(0xaaaa02aa, S3C2410_GPCCON);
	writel(0xaaaaaaaa, S3C2410_GPDCON);
	writel(0x0, S3C2410_GPCUP);
	writel(0x0, S3C2410_GPDUP);

	/* backlight ON */
	s3c2410_gpio_cfgpin(S3C2410_GPB0, S3C2410_GPB0_OUTP);
	s3c2410_gpio_setpin(S3C2410_GPB0, 1);

	/* backlight dimming ON */
	s3c2410_gpio_cfgpin(S3C2410_GPB3, S3C2410_GPB3_OUTP);
	s3c2410_gpio_setpin(S3C2410_GPB3, 1);

	/* backlight dimming control pull-down disable*/
	s3c2410_gpio_pullup(S3C2410_GPB3, 0);

	/* module reset */
	s3c2410_gpio_cfgpin(S3C2410_GPB1, S3C2410_GPB1_OUTP);
	s3c2410_gpio_setpin(S3C2410_GPB1, 1);
	mdelay(100);

	s3c2410_gpio_setpin(S3C2410_GPB1, 0);
	mdelay(10);

	s3c2410_gpio_setpin(S3C2410_GPB1, 1);
	mdelay(10);
}

#if defined(CONFIG_PM)

static struct sleep_save s3c_lcd_save[] = {
};

/*
 *  Suspend
 */
int s3cfb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct fb_info *fbinfo = platform_get_drvdata(dev);
	s3c_fb_info_t *info = fbinfo->par;

	s3cfb_stop_lcd();
	s3c2410_pm_do_save(s3c_lcd_save, ARRAY_SIZE(s3c_lcd_save));

	/* sleep before disabling the clock, we need to ensure
	 * the LCD DMA engine is not going to get back on the bus
	 * before the clock goes off again (bjd) */

	msleep(1);
	clk_disable(info->clk);

	return 0;
}

/*
 *  Resume
 */
int s3cfb_resume(struct platform_device *dev)
{
	struct fb_info *fbinfo = platform_get_drvdata(dev);
	s3c_fb_info_t *info = fbinfo->par;

	clk_enable(info->clk);
	msleep(1);
	s3c2410_pm_do_restore(s3c_lcd_save, ARRAY_SIZE(s3c_lcd_save));

	s3cfb_init_hw();
	s3cfb_start_lcd();

	return 0;
}

#else

int s3cfb_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

int s3cfb_resume(struct platform_device *dev)
{
	return 0;
}

#endif

