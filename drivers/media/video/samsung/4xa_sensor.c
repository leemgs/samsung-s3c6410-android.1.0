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

#include "../s3c_camif.h"
#include "4xa_sensor.h"

static struct i2c_driver sensor_driver;

/* This is an abstract CIS sensor for MSDMA input. */

camif_cis_t msdma_input = {
        itu_fmt:       CAMIF_ITU601,
	order422:      CAMIF_CBYCRY,	/* another case: YCRYCB */
	camclk:        44000000,	/* for 20 fps: 44MHz, for 12 fps(more stable): 26MHz */
	source_x:      800,
	source_y:      600,
	win_hor_ofst:  0,
	win_ver_ofst:  0,
	win_hor_ofst2: 0,
	win_ver_ofst2: 0,
	polarity_pclk: 0,
	polarity_vsync:1,
	polarity_href: 0,
	reset_type:CAMIF_EX_RESET_AL,
	reset_udelay: 5000,
};

camif_cis_t interlace_input = {
        itu_fmt:       CAMIF_ITU601,
	order422:      CAMIF_CBYCRY,	/* another case: YCRYCB */
	camclk:        44000000,	/* for 20 fps: 44MHz, for 12 fps(more stable): 26MHz */
	source_x:      800,
	source_y:      600,
	win_hor_ofst:  0,
	win_ver_ofst:  0,
	win_hor_ofst2: 0,
	win_ver_ofst2: 0,
	polarity_pclk: 0,
	polarity_vsync:1,
	polarity_href: 0,
	reset_type:CAMIF_EX_RESET_AL,
	reset_udelay: 5000,
};

#if defined(CONFIG_VIDEO_SAMSUNG_S5K4BA)
static camif_cis_t data = {
	itu_fmt:       CAMIF_ITU601,
	order422:      CAMIF_YCBYCR,
	camclk:        44000000,	/* for 20 fps: 44MHz, for 12 fps(more stable): 26MHz */
	source_x:      800,
	source_y:      600,
	win_hor_ofst:  40,
	win_ver_ofst:  0,
	win_hor_ofst2: 40,
	win_ver_ofst2: 0,
	polarity_pclk: 0,
	polarity_vsync:1,
	polarity_href: 0,
	reset_type:CAMIF_EX_RESET_AL,
	reset_udelay: 5000,
};

s5k4xa_t s5k4ba_regs_mirror[S5K4BA_REGS];
#else
#error No samsung CIS moudule here !
#endif

camif_cis_t* get_initialized_cis(void)
{
	if (data.init_sensor == 0)
		return NULL;

	return &data;
}

#define CAM_ID 0x5a

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { (CAM_ID >> 1), I2C_CLIENT_END };
static unsigned short *forces[] = { NULL };

static struct i2c_client_address_data addr_data = {
      normal_i2c:normal_addr,
      /* normal_i2c_range:ignore, */
      probe:ignore,
      /* probe_range:ignore, */
      ignore:ignore,
      /* ignore_range:ignore, */
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

#if defined(CONFIG_VIDEO_SAMSUNG_S5K4BA)
void inline sensor_init(struct i2c_client *sam_client)
{
	int i;

	i = (sizeof(s5k4ba_reg)/sizeof(s5k4ba_reg[0]));
	for (i = 0; i < S5K4BA_INIT_REGS; i++) {
		sensor_write(sam_client,
			     s5k4ba_reg[i].subaddr, s5k4ba_reg[i].value);
	}
}
#else
#error No samsung CIS moudule !
#endif

static int
s5k4xa_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(struct i2c_client));	

	strcpy(c->name, "S5K4XA");
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
	return i2c_probe(adap, &addr_data, s5k4xa_attach);
}

static int sensor_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	s3c_camif_unregister_sensor(client);
	return 0;
}

/* Purpose:
    This fucntion only for SVGA Camera : 4BA
*/
static int change_sensor_size(struct i2c_client *client, int size)
{
	int i;

	switch (size) {
#if defined(CONFIG_VIDEO_SAMSUNG_S5K4BA)
	case SENSOR_QSVGA:
		for (i = 0; i < S5K4BA_QSVGA_REGS; i++) {
			sensor_write(client, s5k4ba_reg_qsvga[i].subaddr,
				     s5k4ba_reg_qsvga[i].value);
		}
		break;

	case SENSOR_SVGA:
 		for (i = 0; i < S5K4BA_SVGA_REGS; i++) {
			sensor_write(client, s5k4ba_reg_svga[i].subaddr,
				     s5k4ba_reg_svga[i].value);
		}
		break;
#else
#error No samsung CIS moudule !
#endif
	default:
		panic("4xa_sensor.c: unexpect value\n");
	}

	return 0;
}

static int change_sensor_wb(struct i2c_client *client, int type)
{
       printk("[ *** Page 0, 4XA Sensor White Balance Mode ***]\n");

#if defined(CONFIG_VIDEO_SAMSUNG_S5K4BA)
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
		break;

	case USER_EXIT:
		break;

	case SENSOR_QSVGA:
		change_sensor_size(client, SENSOR_QSVGA);
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
        	printk("[ *** 4XA Sensor White Balance , No mode ***]\n");
        	change_sensor_wb(client, (int) arg);
        	break;

	default:
		panic("4xa_sensor.c : Unexpect Sensor Command \n");
		break;
	}

	return 0;
}

static struct i2c_driver sensor_driver = {
	.driver = {
		.name = "s5k4xa",
	},
      .id = I2C_DRIVERID_S5K4XA,
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

MODULE_AUTHOR("Jinsung, Yang <jsgood.yang@samsung.com>");
MODULE_DESCRIPTION("I2C Client Driver for FIMC V4L2 Driver");
MODULE_LICENSE("GPL");

