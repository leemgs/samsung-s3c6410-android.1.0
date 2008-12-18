/*
 *  Copyright (C) 2004 Samsung Electronics
 *             SW.LEE <hitchcar@samsung.com>
 *            - based on Russell King : pcf8583.c
 * 	      - added  smdk24a0, smdk2440
 *            - added  poseidon (s3c24a0+wavecom)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Driver for FIMC2.x Camera Decoder
 *
 */

//#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>

//#define CAMIF_DEBUG

//#include <asm/arch/registers.h>
#include "../s3c_camif.h"
#include "3xa_sensor.h"

static struct i2c_driver sensor_driver;

/* This is an abstract CIS sensor for MSDMA input. */

camif_cis_t msdma_input = {
        itu_fmt:       CAMIF_ITU601,
        order422:      CAMIF_YCBYCR,		/* YCRYCB */
        camclk:        32000000, 		/* No effect */
        source_x:      800,
        source_y:      480,
        win_hor_ofst:  0,
        win_ver_ofst:  0,
        win_hor_ofst2: 0,
        win_ver_ofst2: 0,
        polarity_pclk: 0,
	polarity_vsync:1,
        polarity_href: 0,
        reset_type:CAMIF_EX_RESET_AH, /* Ref board has inverted signal */
        reset_udelay: 20000,
};

camif_cis_t interlace_input = {
        itu_fmt:       CAMIF_ITU601,
        order422:      CAMIF_YCBYCR,		/* YCRYCB */
        camclk:        32000000, 		/* No effect */
        source_x:      720,
        source_y:      243,
        win_hor_ofst:  0,
        win_ver_ofst:  0,
        win_hor_ofst2: 0,
        win_ver_ofst2: 0,
        polarity_pclk: 1,
	polarity_vsync:0,
        polarity_href: 0,
        reset_type:CAMIF_EX_RESET_AH, /* Ref board has inverted signal */
        reset_udelay: 20000,
};

#if defined(CONFIG_VIDEO_SAMSUNG_S5K3AA)
/* This is SXGA(1280x1024) camera but start from VGA(640x480) mode */
static camif_cis_t data = {
        itu_fmt:       CAMIF_ITU601,
        order422:      CAMIF_CBYCRY,		 /* YCRYCB */
        camclk:        48000000,		 /* No effect */
        source_x:      640,
        source_y:      480,
        win_hor_ofst:  112,
        win_ver_ofst:  72,
        win_hor_ofst2: 112,
        win_ver_ofst2: 72,
        polarity_pclk: 1,
	polarity_vsync:0,
        polarity_href: 0,

#ifdef CONFIG_CPU_S3C24A0A
        reset_type:CAMIF_EX_RESET_AL, 		/* Active Low */
#else
        reset_type:CAMIF_EX_RESET_AH, 		/* Ref board has inverted signal */
#endif
        reset_udelay: 1000,
};

s5k3xa_t s5k3aa_regs_mirror[S5K3AA_REGS];

#elif defined(CONFIG_VIDEO_SAMSUNG_S5K3BA)
/* This is UXGA(1600x1200) camera but start from SVGA(800x600) mode */
static camif_cis_t data = {
        itu_fmt:       CAMIF_ITU601,
        order422:      CAMIF_YCBYCR,		/* YCRYCB */
        camclk:        19000000,
        source_x:      640,
        source_y:      480,
        win_hor_ofst:  0,
        win_ver_ofst:  0,
        win_hor_ofst2: 0,
        win_ver_ofst2: 0,
        polarity_pclk: 1,
	polarity_vsync:1,
        polarity_href: 0,
        reset_type: CAMIF_EX_RESET_AL, 		/* Ref board has inverted signal */
        reset_udelay: 1000,
};

s5k3xa_t s5k3ba_regs_mirror[S5K3BA_REGS];
#else
#error No samsung CIS moudule here !
#endif

camif_cis_t* get_initialized_cis(void)
{
	if(data.init_sensor == 0) return NULL;
	return &data;
}

#define CAM_ID 0x5a

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { (CAM_ID >> 1), I2C_CLIENT_END };
static unsigned short *forces[] = { NULL };

static struct i2c_client_address_data addr_data = {
      normal_i2c:normal_addr,
      //normal_i2c_range:ignore,
      probe:ignore,
      //probe_range:ignore,
      ignore:ignore,
      //ignore_range:ignore,
      forces:forces,
};


unsigned char sensor_read(struct i2c_client *client, unsigned char subaddr)
{
	int ret;
	unsigned char buf[1];
	struct i2c_msg msg = { client->addr, 0, 1, buf };
	buf[0] = subaddr;

	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) {
		printk(" I2C write Error \n");
		return -EIO;
	}

	msg.flags = I2C_M_RD;
	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;

	return buf[0];
}


static int
sensor_write(struct i2c_client *client,
	     unsigned char subaddr, unsigned char val)
{
	unsigned char buf[2];
	struct i2c_msg msg = { client->addr, 0, 2, buf };

	buf[0] = subaddr;
	buf[1] = val;

	return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

#if defined(CONFIG_VIDEO_SAMSUNG_S5K3AA)
void inline sensor_init(struct i2c_client *sam_client)
{
	int i;
	i = (sizeof(s5k3aa_reg)/sizeof(s5k3aa_reg[0]));

	for (i = 0; i < S5K3AA_INIT_REGS; i++) {
		sensor_write(sam_client,
			     s5k3aa_reg[i].subaddr, s5k3aa_reg[i].value);
#if 0
			printk(KERN_ERR "Page:[%03d] Subaddr %02x = 0x%02x\n",i,
			       s5k3aa_reg[i].subaddr,
			       s5k3aa_reg[i].value);
#endif
	}

#ifdef YOU_WANT_TO_CHECK_IMG_SENSOR
	for (i = 0; i < S5K3AA_INIT_REGS; i++) {
		if (s5k3aa_reg[i].subaddr == PAGE_ADDRESS) {
			sensor_write(sam_client,
				     s5k3aa_reg[i].subaddr,
				     s5k3aa_reg[i].value);

			printk(KERN_ERR "Page: Subaddr %02x = 0x%02x\n",
			       s5k3aa_reg[i].subaddr,
			       s5k3aa_reg[i].value);


		} else {
			s5k3aa_regs_mirror[i].subaddr =
			    s5k3aa_reg[i].subaddr;
			s5k3aa_regs_mirror[i].value =
			    s5k3aa_read(sam_client, s5k3aa_reg[i].subaddr);
			printk(KERN_ERR "Subaddr %02x = 0x%02x\n",
			       s5k3aa_reg[i].subaddr,
			       s5k3aa_regs_mirror[i].value);
		}
	}
#endif
}

#elif defined(CONFIG_VIDEO_SAMSUNG_S5K3BA)
void inline sensor_init(struct i2c_client *sam_client)
{
	int i;
	i = (sizeof(s5k3ba_reg)/sizeof(s5k3ba_reg[0]));

	for (i = 0; i < S5K3BA_INIT_REGS; i++) {
		sensor_write(sam_client,
			     s5k3ba_reg[i].subaddr, s5k3ba_reg[i].value);
	}
}
#else
#error No samsung CIS moudule !
#endif

static int
s5k3xa_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(struct i2c_client));	

	strcpy(c->name, "S5K3XA");
	c->addr = addr;
	c->adapter = adap;
	c->driver = &sensor_driver;
	c->data = &data;
	data.sensor = c;

	s3c_camif_register_sensor(c);

	return i2c_attach_client(c);
}

static int sensor_attach_adapter(struct i2c_adapter *adap)
{
	s3c_camif_open_sensor(&data);

	return i2c_probe(adap, &addr_data, s5k3xa_attach);
}

static int sensor_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	s3c_camif_unregister_sensor(client);
	return 0;
}

/* Purpose:
    This fucntion only for SXGA Camera : 3AA
    This fucntion only for UXGA Camera : 3BA
*/
static int change_sensor_size(struct i2c_client *client, int size)
{
	int i;

	switch (size) {
#if defined(CONFIG_VIDEO_SAMSUNG_S5K3AA)
	case SENSOR_VGA:
		for (i = 0; i < S5K3AA_VGA_REGS; i++) {
			sensor_write(client, s5k3aa_reg_vga[i].subaddr,
				     s5k3aa_reg_vga[i].value);
		}
		break;
	case SENSOR_SXGA:
		for (i = 0; i < S5K3AA_SXGA_REGS; i++) {
			sensor_write(client, s5k3aa_reg_sxga[i].subaddr,
				     s5k3aa_reg_sxga[i].value);
		}
		break;

#elif defined(CONFIG_VIDEO_SAMSUNG_S5K3BA)
	case SENSOR_VGA:
		for (i = 0; i < S5K3BA_VGA_REGS; i++) {
			sensor_write(client, s5k3ba_reg_vga[i].subaddr,
				     s5k3ba_reg_vga[i].value);
		}
		break;

	case SENSOR_SVGA:
		for (i = 0; i < S5K3BA_INIT_REGS; i++) {
			sensor_write(client, s5k3ba_reg[i].subaddr,
				     s5k3ba_reg[i].value);
		}
		break;
	case SENSOR_UXGA:
		for (i = 0; i < S5K3BA_UXGA_REGS; i++) {
			sensor_write(client, s5k3ba_reg_uxga[i].subaddr,
				     s5k3ba_reg_uxga[i].value);
		}
		break;
#else
#error No samsung CIS moudule !
#endif
	default:
		panic("3xa_sensor.c: unexpect value \n");
	}

	return 0;
}

static int change_sensor_wb(struct i2c_client *client, int type)
{
       printk("[ *** Page 0, 3XA Sensor White Balance Mode ***]\n");

#if defined(CONFIG_VIDEO_SAMSUNG_S5K3AA)
       sensor_write(client, 0xEC, 0x0);
       sensor_write(client, 0x30, type);
#elif defined(CONFIG_VIDEO_SAMSUNG_S5K3BA)
       sensor_write(client, 0xFC, 0x0);
       sensor_write(client, 0x30, type);
#endif

       switch(type){
           case 0:
           default:
                printk(" -> AWB auto mode ]\n");
                break;
           case 1:
                printk(" -> Indoor 3100 mode ]\n");
                break;
           case 2:
                printk(" -> Outdoor 5100 mode ]\n");
                break;
           case 3:
                printk(" -> Indoor 2000 mode ]\n");
                break;
           case 4:
                printk(" -> AE/AWB halt ]\n");
                break;
           case 5:
                printk(" -> Cloudy(6000) mode ]\n");
                break;
           case 6:
                printk(" -> Sunny(8000) mode ]\n");
                break;
       }

       return 0;
}

static int
sensor_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case SENSOR_INIT:
		sensor_init(client);
		printk(KERN_INFO "External Camera initialized\n");
		break;

	case USER_ADD:
		//MOD_INC_USE_COUNT;
		break;

	case USER_EXIT:
		//MOD_DEC_USE_COUNT;
		break;

	case SENSOR_VGA:
		change_sensor_size(client, SENSOR_VGA);
		break;

	case SENSOR_SVGA:
		change_sensor_size(client, SENSOR_SVGA);
		break;

	case SENSOR_SXGA:
		change_sensor_size(client, SENSOR_SXGA);
		break;

	case SENSOR_UXGA:
		change_sensor_size(client, SENSOR_UXGA);
		break;
/* Todo
	case SENSOR_BRIGHTNESS:
		change_sensor_setting();
		break;
*/
	case SENSOR_WB:
        	printk("[ *** 3XA Sensor White Balance , No mode ***]\n");
        	change_sensor_wb(client, (int) arg);
        	break;

	default:
		panic("3xa_sensor.c : Unexpect Sensor Command \n");
		break;
	}

	return 0;
}

static struct i2c_driver sensor_driver = {
	.driver = {
		.name = "s5k3xa",
	},
      //name:"S5K3XA",
      .id = I2C_DRIVERID_S5K3XA,
      //flags:I2C_DF_NOTIFY,
      .attach_adapter = sensor_attach_adapter,
      .detach_client = sensor_detach,
      .command = sensor_command
};

static __init int camif_sensor_init(void)
{
	return i2c_add_driver(&sensor_driver);
}


static __init void camif_sensor_exit(void)
{
	i2c_del_driver(&sensor_driver);
}

module_init(camif_sensor_init)
module_exit(camif_sensor_exit)

MODULE_AUTHOR("Yeom, Youngran <yeom@samsung.com>");
MODULE_DESCRIPTION("I2C Client Driver For Fimc3.x V4L2 Driver");
MODULE_LICENSE("GPL");

