/*
 * ADV7180 Analog decoder driver for S3C2443X
 * 
 * Jaecheol Lee <jc.lee@samsung.com>
 * 2007.08.08	
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/video_decoder.h>
#include <media/v4l2-dev.h>

//#include <asm/arch/registers.h>
#include "../s3c_camif.h"
#include "adv7180.h"



static const char *sensor_version =
    "adv7180.c, v 1.00 2007/08/08 jc.lee@samsung.com";


static struct i2c_driver i2c_driver_adv7180;

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
        itu_fmt:       CAMIF_ITU656,
        order422:      CAMIF_CBYCRY,		/* YCRYCB */
        camclk:        32000000, 		/* No effect */
        source_x:      720,       
        source_y:      243,
        win_hor_ofst:  0,
        win_ver_ofst:  0,
        win_hor_ofst2: 0,
        win_ver_ofst2: 0,
        polarity_pclk: 0,
	polarity_vsync:0,
        polarity_href: 0,
        reset_type:CAMIF_EX_RESET_AH, /* Ref board has inverted signal */
        reset_udelay: 20000,
};


extern camif_cis_t* get_initialized_cis();
camif_cis_t* get_initialized_cis() {
	if(interlace_input.init_sensor == 0) return NULL;
	return &interlace_input;

}

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { (I2C_ADV7180 >> 1), I2C_CLIENT_END };
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


static inline int
adv7180_write (struct i2c_client *client,
	       u8                 reg,
	       u8                 value)
{
	struct adv7180 *decoder = i2c_get_clientdata(client);
	decoder->reg[reg] = value;
	return i2c_smbus_write_byte_data(client, reg, value);
}

static inline int
adv7180_read (struct i2c_client *client,
	      u8                 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int
adv7180_write_block (struct i2c_client *client,
		     const u8          *data,
		     unsigned int       len)
{
	int ret = -1;
	u8 reg;

	/* the adv7180 has an autoincrement function, use it if
	 * the adapter understands raw I2C */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		/* do raw I2C, not smbus compatible */
		struct adv7180 *decoder = i2c_get_clientdata(client);
		struct i2c_msg msg;
		u8 block_data[32];

		msg.addr = client->addr;
		msg.flags = 0;
		while (len >= 2) {
			msg.buf = (char *) block_data;
			msg.len = 0;
			block_data[msg.len++] = reg = data[0];
			do {
				block_data[msg.len++] =
				    decoder->reg[reg++] = data[1];
				len -= 2;
				data += 2;
			} while (len >= 2 && data[0] == reg &&
				 msg.len < 32);
			if ((ret = i2c_transfer(client->adapter,
						&msg, 1)) < 0)
				break;
		}
	} else {
		/* do some slow I2C emulation kind of thing */
		while (len >= 2) {
			reg = *data++;
			if ((ret = adv7180_write(client, reg,
						 *data++)) < 0)
				break;
			len -= 2;
		}
	}

	return ret;
}


static int
adv7180_detect_client (struct i2c_adapter *adapter,
		       int                 address,
		       int                 kind)
{
	int i;
	struct i2c_client *client;
	struct adv7180 *decoder;
	char *dname;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_EMUL))
		return 0;

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (client == 0)
		return -ENOMEM;
	client->addr = address;
	client->adapter = adapter;
	client->driver = &i2c_driver_adv7180;
	if ((client->addr == I2C_ADV7180 >> 1) ||
	    (client->addr == (I2C_ADV7180 >> 1) + 1)) {
		dname = adv7180_name;
	} else {
		/* We should never get here!!! */
		kfree(client);
		return 0;
	}

	client->data = &interlace_input;
	interlace_input.sensor = client;
	camif_register_cis(client);
	
	strlcpy(I2C_NAME(client), dname, sizeof(I2C_NAME(client)));

	decoder = kzalloc(sizeof(struct adv7180), GFP_KERNEL);
	if (decoder == NULL) {
		kfree(client);
		return -ENOMEM;
	}
	decoder->norm = VIDEO_MODE_NTSC;
	decoder->input = 0;
	decoder->enable = 1;
	i2c_set_clientdata(client, decoder);

	i = i2c_attach_client(client);
	if (i) {
		kfree(client);
		kfree(decoder);
		return i;
	}

	i = adv7180_write_block(client, init_composite, sizeof(init_composite));
	if (i >= 0) {

	}
	if (i < 0) {
	}

	return 0;
}



static int adv7180_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, adv7180_detect_client);
}

static int adv7180_detach_client(struct i2c_client *client)
{
	int err;
	err = i2c_detach_client(client);
	if(err) {
		return err;
	}
	camif_unregister_cis(client);

	kfree(client);
	return 0;
}

static int
adv7180_command (struct i2c_client *client,
		 unsigned int       cmd,
		 void              *arg)
{
	int tmp;
	u64 endtime;
	
	struct adv7180 *decoder = i2c_get_clientdata(client);

	switch (cmd) {

	case 0:
		/* This is just for testing!!! */
		adv7180_write_block(client, init_composite,
				    sizeof(init_composite));
	        break;
		
	case USER_ADD:
		adv7180_write(client, 0x0f, (1<<7)); // Reset
		endtime = get_jiffies_64() + RESET_DELAY;
		while(jiffies < endtime); // Delay for a 5 ms
		break;
		
	case USER_EXIT:
		adv7180_write(client, 0x0f, (1<<5)); // PWRDWN : 1
		break;

	case DECODER_INIT:
		break;		

	case DECODER_GET_STATUS:
	{
		int *iarg = arg;

		tmp = adv7180_read(client, 0x10) & 0x000000ff; // STATUS1
		tmp = (tmp | (adv7180_read(client, 0x11) & 0x000000ff)<<8); //  + IDENTIFICATION
		tmp = (tmp | (adv7180_read(client, 0x12) & 0x000000ff)<<16); // + STATUS2
		tmp = (tmp | (adv7180_read(client, 0x13) & 0x000000ff)<<24); // + STATUS3

		*iarg = tmp;
	}
		break;

	case DECODER_GET_CAPABILITIES:
	{
		struct video_decoder_capability *cap = arg;

		cap->flags = VIDEO_DECODER_NTSC |
			     VIDEO_DECODER_PAL; /* well, hacky */
		cap->inputs = 2;
		cap->outputs = 1;
	}
		break;

	case DECODER_SET_NORM:
	{
		int iarg = *(int *) arg;
		switch (iarg) {

		case 0:	// Composite
			adv7180_write_block(client, init_composite,
					    sizeof(init_composite));
			break;

		case 1:	//S-VIDEO
			adv7180_write_block(client, init_svideo,
					    sizeof(init_svideo));
			break;

		case 2:	// Component
			adv7180_write_block(client, init_component,
			    sizeof(init_component));
			break;
		default:
			return -EINVAL;

		}

		decoder->norm = iarg;
	}
		break;

	case DECODER_SET_INPUT:
	{
		int iarg = *(int *) arg;
		decoder->input = iarg;

		switch(decoder->input) {

		case CVBS:
			adv7180_write_block(client, init_composite,
					    sizeof(init_composite));
			break;
			
		case SVIDEO:
			adv7180_write_block(client, init_svideo,
					    sizeof(init_svideo));
			break;
			
		case YPbPr:
			adv7180_write_block(client, init_component,
			    sizeof(init_component));
			break;
			
		default:
			return -EINVAL;

		}
	}
		break;

	case DECODER_SET_OUTPUT:
	{
		int *iarg = arg;

		if (*iarg != 0) {
			return -EINVAL;
		}
	}
		break;

	case DECODER_ENABLE_OUTPUT:
	{
		int *iarg = arg;

		decoder->enable = !!*iarg;
	}
		break;

	case DECODER_SET_GPIO:
	{
		int *iarg = arg;
		switch(*iarg) {
		case 0:	// Pin 37 output is Field signal
			adv7180_write(client, 0x58, 0x00);
			break;
			
		case 1: // Output is Vsync signal
			adv7180_write(client, 0x58, (1<<0));
			break;
			
		default:
			return -EINVAL;
		}
	}
		break;
		
	default:
		return -EINVAL;
	}

	return 0;
}

static struct i2c_driver i2c_driver_adv7180 = {
	.driver = {
		.name = "adv7180",
	},
      .id = I2C_DRIVERID_ADV7170, /* Must be fixed!!! */

      .attach_adapter = adv7180_attach_adapter,
      .detach_client = adv7180_detach_client,
      .command = adv7180_command
};

static __init int adv7180_init(void)
{
	return i2c_add_driver(&i2c_driver_adv7180);
}


static __init void adv7180_exit(void)
{
	i2c_del_driver(&i2c_driver_adv7180);
}

module_init(adv7180_init)
module_exit(adv7180_exit)

MODULE_AUTHOR("JaeCheol Lee <jc.lee@samsung.com>");
MODULE_DESCRIPTION("I2C Client Driver For ADV7180 video decoder");
MODULE_LICENSE("GPL");

