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
#include "mt9p012.h"

static struct i2c_driver sensor_driver;

camif_cis_t msdma_input;

#define APTINA_MT9P012_INIT_RES_SXGA
//#define APTINA_MT9P012_INIT_RES_QSXGA

static camif_cis_t data = {
	itu_fmt:       CAMIF_ITU601,
	order422:      CAMIF_CBYCRY,
	camclk:        16000000,

#if defined(APTINA_MT9P012_INIT_RES_SXGA)
	source_x:      1296,
	source_y:      972,
#elif defined(APTINA_MT9P012_INIT_RES_QSXGA)
	source_x:      2592,
	source_y:      1944,
#endif

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

camif_cis_t* get_initialized_cis(void)
{
	if (data.init_sensor == 0)
		return NULL;

	return &data;
}

#define CAM_ID 0x78

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


unsigned char sensor_read(struct i2c_client *client, unsigned char val0)
{
	int ret;
	unsigned char buf[1];
	struct i2c_msg msg = { client->addr, 0, 1, buf };
	buf[0] = val0;

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
	     unsigned char val0, unsigned char val1, unsigned char val2, unsigned char val3)
{
	unsigned char buf[4];
	struct i2c_msg msg = { client->addr, 0, 4, buf };

	buf[0] = val0;
	buf[1] = val1;
	buf[2] = val2;
	buf[3] = val3;

	return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

void inline sensor_init(struct i2c_client *sam_client)
{
	int i;

	for (i = 0; i < MT9P012_INIT_REGS; i++) {
		if (mt9p012_init_reg[i].value0 == 0xde && mt9p012_init_reg[i].value1 == 0xde && mt9p012_init_reg[i].value2 == 0xde) {
			mdelay(mt9p012_init_reg[i].value3);
			continue;
		}

		sensor_write(sam_client, mt9p012_init_reg[i].value0, mt9p012_init_reg[i].value1,
			     mt9p012_init_reg[i].value2, mt9p012_init_reg[i].value3);
	}

#if defined(APTINA_MT9P012_INIT_RES_QSXGA)
	for (i = 0; i < MT9P012_QSXGA_REGS; i++) {
		sensor_write(sam_client, mt9p012_qsxga_reg[i].value0, mt9p012_qsxga_reg[i].value1,
			     mt9p012_qsxga_reg[i].value2, mt9p012_qsxga_reg[i].value3);
	}
#endif
}

static int
mt9p012_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(struct i2c_client));	

	strcpy(c->name, "mt9p012");
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
	return i2c_probe(adap, &addr_data, mt9p012_attach);
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
	case SENSOR_SXGA_MT9P012:
 		for (i = 0; i < MT9P012_SXGA_REGS; i++) {
			sensor_write(client, mt9p012_sxga_reg[i].value0, mt9p012_sxga_reg[i].value1,
				mt9p012_sxga_reg[i].value2, mt9p012_sxga_reg[i].value3);
		}
		break;

	case SENSOR_QSXGA_MT9P012:
 		for (i = 0; i < MT9P012_QSXGA_REGS; i++) {
			sensor_write(client, mt9p012_qsxga_reg[i].value0, mt9p012_qsxga_reg[i].value1,
				mt9p012_qsxga_reg[i].value2, mt9p012_qsxga_reg[i].value3);
		}
		break;

	default:
		printk("mt9p012.c: unexpected value\n");
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

	case SENSOR_SXGA_MT9P012:
		change_sensor_size(client, SENSOR_SXGA_MT9P012);
		break;

	case SENSOR_QSXGA_MT9P012:
		change_sensor_size(client, SENSOR_QSXGA_MT9P012);
		break;

	case USER_ADD:
		break;

	case USER_EXIT:
		break;

	default:
		printk("mt9p012.c: Unexpected sensor command\n");
		break;
	}

	return 0;
}

static struct i2c_driver sensor_driver = {
	.driver = {
		.name = "mt9p012",
	},
      .id = I2C_DRIVERID_MT9P012,
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
MODULE_DESCRIPTION("I2C Client Driver For FIMC V4L2 Driver");
MODULE_LICENSE("GPL");

