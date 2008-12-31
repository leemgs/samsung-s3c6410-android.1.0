/*
 * Copyright (c) 2008 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C64xx DVFS interface with LTC3714 DCDC convertor power
 * 2008.02.28. basic scheme update to here
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/stat.h>
#include <linux/proc_fs.h>

#include <linux/delay.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/gpio.h>
#include "s3c64xx-dvfs.h"

#define DEV_NAME	"s3c64xx-dvfs"
#define DEV_MAJOR	240

#define XTAL 		12*1000*1000	/* Clock source 12Mhz */
#define MAX_APLL_RATIO	7	/* See S3C6400 user manual 3-7 */
#define MAX_HCLK2_RATIO	7	/* See S3C6400 user manual 0-1 */
#define Mhz 		1000*1000

#define SUPPORT_PROC_FS

#ifdef SUPPORT_PROC_FS
struct proc_dir_entry *proc_root_fp	= NULL;
struct proc_dir_entry *proc_voltage_fp 	= NULL;
struct proc_dir_entry *proc_freq_fp 	= NULL;
struct proc_dir_entry *proc_step_fp 	= NULL;

char proc_voltage_str[PAGE_SIZE-80] 	= { 0,};
char proc_freq_str[PAGE_SIZE-80] 	= { 0,};
char proc_step_str[PAGE_SIZE-80] 	= { 0,};
#endif

/* it's depend on DCDC regurator specification
 * if change parts, you must be change table and configuration*/
static const unsigned int voltage_table[32] = {
	1750, 1700, 1650, 1600, 1550, 1500, 1450, 1400,
	1350, 1300, 1250, 1200, 1150, 1100, 1050, 1000,
	975, 950, 925, 900, 875, 850, 825, 800,
	775, 750, 725, 700, 675, 650, 625, 600,
};

#define NUMBER_OF_STEP 	4

/*Frequency step define section  */
#define MDIV		0
#define PDIV		1
#define SDIV		2
#define ARM_RATIO 	3
#define HCLK_RATIO 	4
#define DVFS		MDIV

#define ARM_VOLT_STEP 0
#define INT_VOLT_STEP 1
#define FREQ_STEP 2

static const unsigned int pll_mps_table[][5] = {
	{533, 	6, 	1, 	0,	1}, /* step 0 ARM:(533 / 1)=533 MHz ,HCLKx2:266 HCLK: 133*/
	{400, 	6, 	1, 	0,	1}, /* step 1 ARM:(400 / 1)=400 MHz ,HCLKx2:266 HCLK: 133*/
	{400, 	6, 	1, 	1,	1}, /* step 2 ARM:(400 / 2)=200 MHz ,HCLKx2:266 HCLK: 133*/
	{400, 	6, 	1, 	3,	2}, /* step 3 ARM:(400 / 4)=100 MHz ,HCLKx2:133  HCLK: 66 ==> careful linked device*/
/*	{MDIV, PDIV, SDIV, ARM RATIO, HCLKx2 RATIO } */
};

static const unsigned int dvfs_step[NUMBER_OF_STEP][3] = {
	{ 1200	,  1200,	0}, /* ARM 1.2Volt, INT 1.2V, pll mode 0 533-133*/
	{ 1000	,  1000, 	1}, /* ARM 1.2Volt, INT 1.2V, pll mode 2 200-133*/
	{ 950	,  1000,  	2}, /* ARM 1.2Volt, INT 1.2V, pll mode 2 200-133*/
	{ 900	,  1000,  	2}, /* ARM 1.2Volt, INT 1.2V, pll mode 3 100- 66*/
/*    {Voltage, Frequency_STEP} */
};

static int previous_dvfs_step;

static s3c_dvfs_info dvfs_info;

static void set_dvfs_gpio(void)
{
	/* We can control the voltage of ARM core and Internal block by setting GPIO */
	/* GPN(11~15). First of all, we should set these gpio to output mode.	 */
	/* GPIO configuration	*/
	GPIO_CFG(LTC3714_DATA1, LTC3714_OUTP1);
	GPIO_CFG(LTC3714_DATA2, LTC3714_OUTP2);
	GPIO_CFG(LTC3714_DATA3, LTC3714_OUTP3);
	GPIO_CFG(LTC3714_DATA4, LTC3714_OUTP4);
	GPIO_CFG(LTC3714_DATA5, LTC3714_OUTP5);

	/* Pull-up/down disable */
	GPIO_PULLUP(LTC3714_DATA1, 0x0);
	GPIO_PULLUP(LTC3714_DATA2, 0x0);
	GPIO_PULLUP(LTC3714_DATA3, 0x0);
	GPIO_PULLUP(LTC3714_DATA4, 0x0);
	GPIO_PULLUP(LTC3714_DATA5, 0x0);


	/* Latch control signal*/
	/* CORE_REG_OE: GPL9, ARM_REG_OE: GPL8, INT_REG_LE: GPL10*/
	GPIO_CFG(ARM_LE	, ARM_LE_OUTP);
	GPIO_CFG(DVS_OE	, DVS_OE_OUTP);
	GPIO_CFG(INT_LE	, INT_LE_OUTP);

	GPIO_PULLUP(ARM_LE, 0x0);
	GPIO_PULLUP(DVS_OE, 0x0);
	GPIO_PULLUP(INT_LE, 0x0);

}

/* Set LTC3714 voltage regulator 		*/
/* Input : pwr : 1(ARM), 2(Internal), 3(Both)	*/
/*         voltage : 1mV step			*/
static int set_ltc3714(unsigned int pwr, unsigned int voltage)
{
	int position = 0;

	if(voltage > voltage_table[0] || voltage < voltage_table[31]) {
		printk("[ERROR]: voltage value over limits!!!");
		return -EINVAL;
	}

	if(voltage > voltage_table[16]) { // 1750 ~ 1000 mV
		for(position = 15; position >= 0; position --) {
			if(voltage_table[position] == voltage) break;
		}

	}
	else if(voltage >= voltage_table[31]) {	//975 ~ 600 mV
		for(position = 31; position >= 16; position --) {
			if(voltage_table[position] == voltage) break;
		}
	}
	else {
		printk("[error]: Can't find adquate voltage table list value\n");
		return -EINVAL;
	}

	printk("Founded postion :[%d] \n",position);

	position &=0x1f;

	__raw_writel((__raw_readl(S3C_GPNDAT)&~(0x1f<<11))|(position<<11), S3C_GPNDAT);

	if(pwr == ARM_VOLT_STEP) {		//ARM Voltage Control => ARM_REG_LE => Output H => Data Changed

		CONTROl_SET(ARM_LE, 0x1);
		udelay(10);
		CONTROl_SET(ARM_LE, 0x0);
	} else if(pwr == INT_VOLT_STEP) {	// INT Voltage Control
		CONTROl_SET(INT_LE, 0x1);
		udelay(10);
		CONTROl_SET(INT_LE, 0x0);
	} else {
		printk("[error]: set_ltc3714, check mode [pwr] value\n");
		return -EINVAL;
	}


	return 0;

}

/* Set CLK_DIV0 register with specific APLL divider value*/
static int set_freq_divider(unsigned int clk_source , unsigned int val)
{
	unsigned int tmp;

	if (clk_source == ARM_RATIO){
		if(val > MAX_APLL_RATIO) {
			printk(KERN_ERR "Freq divider value(APLL_RATIO) is out of spec\n");
			printk(KERN_ERR "APLL_RATIO : 0~7\n");
			return -EINVAL;
		}
		dvfs_info.freq_divider = (val + 1);
		tmp = __raw_readl(ARM_CLK_DIV)&~ARM_DIV_MASK;
		tmp |= val<<ARM_DIV_RATIO_BIT;
		__raw_writel(tmp, ARM_CLK_DIV);
	} else if (clk_source == HCLK_RATIO){
                if(val > MAX_HCLK2_RATIO) {
                        printk(KERN_ERR "Freq divider value is out of spec\n");
                        printk(KERN_ERR "HCLK_PostDivider RATIO : 0,1 \n");
                        return -EINVAL;
                }
		/* it's only for setting the HCLK or system Clock */
                tmp =  __raw_readl(ARM_CLK_DIV) & ~HCLK_DIV_MASK;
                tmp |= (val<<HCLK_DIV_RATIO_BIT);
                __raw_writel(tmp, ARM_CLK_DIV);
	} else {
			printk(KERN_ERR " It's wrong clock post divider path \n");
			return -EINVAL;
	}

	return 0;
}

/* Set APLL P,M,S value to make specific CPU frequency*/
/* Only support Async mode */
static int set_pll(unsigned int freq_level)
{
	unsigned int val,err;

	if (freq_level < NUMBER_OF_STEP ){

	err = set_freq_divider(ARM_RATIO,  pll_mps_table[freq_level][ARM_RATIO]);
	if(err)
		return err;

	err = set_freq_divider(HCLK_RATIO, pll_mps_table[freq_level][HCLK_RATIO]);
	if(err)
		return err;
/*
     it's only guarantee for  chaiging PLL on S3C64xx
     if you want to change dynamic freq. you must check stability of system
*/
	val = PLL_CALC_VAL(pll_mps_table[freq_level][MDIV],\
			                    pll_mps_table[freq_level][PDIV],\
			                    pll_mps_table[freq_level][SDIV]);
	__raw_writel(val, ARM_PLL_CON);
/*
     it does not guarantee for  chaiging PLL on S3C2443,S3C2450 and S3C2416
     but we guide to set DVFS or divide arm clock output
*/

	}
	else{
		printk("It's wrong DFS %d range  \n",freq_level);
	}

	return 0;
}

/* Other modules can use this function to change power status */
/* Especially, ARM Dynamic power interface can control DVFS by accessing this */
int set_dvfs_step(unsigned int step)
{
	int err;

	if(step >= NUMBER_OF_STEP) {
		printk(KERN_ERR "DVFS step is out of range(0 ~ %2d)\n",NUMBER_OF_STEP-1);
		return -EINVAL;
	}
	//printk("set_dvfs_step[%d]\n",step);

	if(previous_dvfs_step < step) { // decreasing CPU freq & Voltage
		/* CPU Frequency control */
		err = set_pll(dvfs_step[step][FREQ_STEP]);

		/* CPU voltage control */
		dvfs_info.pwr_type = ARM_VOLT_STEP;
		dvfs_info.new_voltage = dvfs_step[step][ARM_VOLT_STEP];
		err = set_ltc3714(dvfs_info.pwr_type, dvfs_info.new_voltage);
		if(err == 0){
			dvfs_info.curr_voltage_arm = dvfs_info.new_voltage;
		}


		dvfs_info.pwr_type = INT_VOLT_STEP;
		dvfs_info.new_voltage = dvfs_step[step][INT_VOLT_STEP];
		err = set_ltc3714(dvfs_info.pwr_type, dvfs_info.new_voltage);
		if(err == 0){
			dvfs_info.curr_voltage_internal = dvfs_info.new_voltage;
		}

	} else if(previous_dvfs_step > step) { // increasing CPU freq & Voltage

                /* CPU voltage control */
                dvfs_info.pwr_type = ARM_VOLT_STEP;
                dvfs_info.new_voltage = dvfs_step[step][ARM_VOLT_STEP];
                err = set_ltc3714(dvfs_info.pwr_type, dvfs_info.new_voltage);
                if(err == 0){
                        dvfs_info.curr_voltage_arm = dvfs_info.new_voltage;
                }

                dvfs_info.pwr_type = INT_VOLT_STEP;
                dvfs_info.new_voltage = dvfs_step[step][INT_VOLT_STEP];
                err = set_ltc3714(dvfs_info.pwr_type, dvfs_info.new_voltage);
                if(err == 0){
                        dvfs_info.curr_voltage_internal = dvfs_info.new_voltage;
                }

               /* CPU Frequency control */
                err = set_pll(dvfs_step[step][FREQ_STEP]);

	} else {
	}
	previous_dvfs_step = step;

	return 0;
}

EXPORT_SYMBOL(set_dvfs_step);

int s3c_dvfs_open(struct inode *inode, struct file *filp)
{
	int num = MINOR(inode->i_rdev);

	printk("s3c_dvfs_open -> minor : %d\n", num);

	return 0;
}

ssize_t s3c_dvfs_read(struct file *filp, char *buf, size_t count,
			loff_t *f_pos)
{
	return 0;
}

ssize_t s3c_dvfs_write(struct file *filp, const char *buf, size_t count,
			loff_t *f_pos)
{
	return 0;
}

int s3c_dvfs_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	int err, size;

	if(_IOC_TYPE(cmd) != DVFS_IOCTL_MAGIC) return -EINVAL;
	if(_IOC_NR(cmd) >= DVFS_MAXNR) return -EINVAL;

	size = _IOC_SIZE(cmd);

	switch(cmd) {
		case DVFS_ON:
			break;
		case DVFS_OFF:
			break;
		case DVFS_GET_STATUS:
			err=copy_to_user((void *) arg, (const void *) &dvfs_info, (unsigned long) size);
			break;
		case DVFS_SET_STATUS:
			err=copy_from_user((void *) &dvfs_info, (const void *) arg, size);
			err = set_ltc3714(dvfs_info.pwr_type, dvfs_info.new_voltage);

			if(err != 0) return err;

			if(dvfs_info.pwr_type == ARM_VOLT_STEP) {
				dvfs_info.curr_voltage_arm = dvfs_info.new_voltage;
			} else if(dvfs_info.pwr_type == INT_VOLT_STEP) {
				dvfs_info.curr_voltage_internal = dvfs_info.new_voltage;
			} else {
			}
			break;
		default:
			break;
	}
	return 0;
}

int s3c_dvfs_release(struct inode *inode, struct file *filp)
{
	printk("s3c_dvfs_release \n");
	return 0;
}

struct file_operations s3c_dvfs_fops =
{
	.owner		= THIS_MODULE,
	.read		= s3c_dvfs_read,
	.write		= s3c_dvfs_write,
	.ioctl		= s3c_dvfs_ioctl,
	.open		= s3c_dvfs_open,
	.release	= s3c_dvfs_release,
};

#ifdef SUPPORT_PROC_FS
int read_proc_voltage(char *page, char **start, off_t off,
											int count, int *eof, void *data_unused)
{
	char *buf;

	buf = page;
	buf += sprintf(buf, "ARM = [%d]mV, Internal = [%d]mV\n", dvfs_info.curr_voltage_arm,dvfs_info.curr_voltage_internal);

	return buf - page;
}

int write_proc_voltage(struct file *file, const char __user *buffer,
												unsigned long count, void *data)
{
	int len, err;
	char *realdata;

	realdata = (char *) data;

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len - 1] == '\n')
		realdata[--len] = 0;

	dvfs_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	err = set_ltc3714(dvfs_info.pwr_type, dvfs_info.new_voltage);

	if(err == 0) {
		if(dvfs_info.pwr_type == ARM_VOLT_STEP) {
			dvfs_info.curr_voltage_arm = dvfs_info.new_voltage;
		} else if(dvfs_info.pwr_type == INT_VOLT_STEP) {
			dvfs_info.curr_voltage_internal = dvfs_info.new_voltage;
		} else {
		}
	}

	return count;
}

int read_proc_freq(char *page, char **start, off_t off,
											int count, int *eof, void *data_unused)
{
	char *buf;
	unsigned int arm_freq;

	arm_freq = GET_ARM_CLOCK(XTAL);
	dvfs_info.freq_divider = READ_ARM_DIV;
	dvfs_info.curr_freq = arm_freq/dvfs_info.freq_divider;

	buf = page;
	buf += sprintf(buf, "Freq = [%d]MHz\n", dvfs_info.curr_freq/1000000);

	return buf - page;
}

int write_proc_freq(struct file *file, const char __user *buffer,
												unsigned long count, void *data)
{
	int len, tmp, err;
	char *realdata;

	realdata = (char *) data;

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len - 1] == '\n')
		realdata[--len] = 0;
	tmp = simple_strtoul(realdata, NULL, 10);

	printk("level %d \n",tmp);

	err = set_freq_divider(ARM_RATIO,  tmp);
        if(err){
		printk("level err %d \n",err);
                return err;
	}

	return count;
}

int write_proc_step(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len, tmp, err;
	char *realdata;

	realdata = (char *) data;

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;
	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len - 1] == '\n')
		realdata[--len] = 0;
	tmp = simple_strtoul(realdata, NULL, 10);

	err = set_dvfs_step(tmp);

	return count;
}

#endif

int s3c_dvfs_init(void)
{
	int result;

	/* Initialize structure */
	previous_dvfs_step = -1; /* 1st initial operating*/
	set_dvfs_gpio();
	set_dvfs_step(0);	// Set Maximum or default initial performance set
	CONTROl_SET(DVS_OE, 0x1);  /* Just open target power voltage*/
	printk("4444444\n");

	result = register_chrdev(DEV_MAJOR, DEV_NAME, &s3c_dvfs_fops);

	printk("%d 55555555\n",result);

	if(result < 0)
		return result;

#ifdef SUPPORT_PROC_FS
	proc_root_fp = proc_mkdir("dvfs", 0);

	proc_voltage_fp = create_proc_entry("voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_voltage_str;
		proc_voltage_fp->read_proc = read_proc_voltage;
		proc_voltage_fp->write_proc = write_proc_voltage;
	}

	proc_freq_fp = create_proc_entry("frequency", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_freq_fp) {
		proc_freq_fp->data = proc_freq_str;
		proc_freq_fp->read_proc = read_proc_freq;
		proc_freq_fp->write_proc = write_proc_freq;
	}

	proc_step_fp = create_proc_entry("step", S_IFREG | S_IWUSR, proc_root_fp);
	if(proc_step_fp) {
		proc_step_fp->data = proc_step_str;
		proc_step_fp->read_proc = NULL;
		proc_step_fp->write_proc = write_proc_step;
	}
#endif

	return 0;
}

void s3c_dvfs_exit(void)
{
	unregister_chrdev(DEV_MAJOR, DEV_NAME);
	remove_proc_entry("voltage", proc_root_fp);
	remove_proc_entry("frequency", proc_root_fp);
	remove_proc_entry("step", proc_root_fp);
	remove_proc_entry("dvfs", 0);
}

module_init(s3c_dvfs_init);
module_exit(s3c_dvfs_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("S3C64xx DVFS Driver");
MODULE_AUTHOR("Jongpill.Lee, <boyko.lee@samsung.com>");
