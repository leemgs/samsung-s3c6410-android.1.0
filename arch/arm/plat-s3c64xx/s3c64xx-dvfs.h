/*
	2008.04.14. All of AP are  merged single application and definitin
	in order to be matcing dvs and dfs concept
	These are confirm on SMDK2450, SMDK2416,SMDK6400,SMDK6410
*/
#ifndef __S3C64XXDVFS_H_
#define __S3C64XXDVFS_H_

#define DVFS_IOCTL_MAGIC	'd'

typedef struct {
	unsigned int pwr_type;			//ARM:ARMV_STEP, INT:INTV_STEP
	unsigned int curr_voltage_arm;		//ARM core voltage
	unsigned int curr_voltage_internal;	//Internal block voltage
	unsigned int new_voltage;
	unsigned int curr_freq;
	unsigned int freq_divider;	// APLL_RATIO + 1
	unsigned int size;
} s3c_dvfs_info;

#define DVFS_ON			_IO(DVFS_IOCTL_MAGIC, 0)
#define DVFS_OFF			_IO(DVFS_IOCTL_MAGIC, 1)
#define DVFS_GET_STATUS	_IOR(DVFS_IOCTL_MAGIC, 2, s3c_dvfs_info)
#define DVFS_SET_STATUS	_IOW(DVFS_IOCTL_MAGIC, 3, s3c_dvfs_info)

#define DVFS_MAXNR	4

/* Board dependency SIGNAL and GPIO configuration */
#include <asm/arch-s3c2410/regs-s3c-clock.h>

#define ARM_LE  	S3C_GPL8
#define DVS_OE     S3C_GPL9
#define INT_LE 	S3C_GPL10

#define CONTROl_SET(pin,to) gpio_set_value(pin, to);
#define ARM_LE_OUTP   S3C_GPL8_OUTP
#define DVS_OE_OUTP  S3C_GPL9_OUTP
#define INT_LE_OUTP 	 S3C_GPL10_OUTP

#define LTC3714_DATA1  S3C_GPN11
#define LTC3714_DATA2  S3C_GPN12
#define LTC3714_DATA3  S3C_GPN13
#define LTC3714_DATA4  S3C_GPN14
#define LTC3714_DATA5  S3C_GPN15

#define LTC3714_OUTP1  S3C_GPN11_OUTP
#define LTC3714_OUTP2  S3C_GPN12_OUTP
#define LTC3714_OUTP3  S3C_GPN13_OUTP
#define LTC3714_OUTP4  S3C_GPN14_OUTP
#define LTC3714_OUTP5  S3C_GPN15_OUTP

#define GPIO_CFG(pin,to)	gpio_set_pin((pin),(to))
#define GPIO_PULLUP(pin,to)	gpio_pullup((pin),(to))

#define ARM_PLL_CON 	    S3C_APLL_CON
#define ARM_CLK_DIV	     S3C_CLK_DIV0
#define ARM_DIV_RATIO_BIT	0
#define ARM_DIV_MASK    (0xf<<ARM_DIV_RATIO_BIT)
#define HCLK_DIV_RATIO_BIT	9
#define HCLK_DIV_MASK  (0x7<<HCLK_DIV_RATIO_BIT)

#define READ_ARM_DIV    ((__raw_readl(ARM_CLK_DIV)&ARM_DIV_MASK) + 1)
#define PLL_CALC_VAL(MDIV,PDIV,SDIV)	((1<<31)|(MDIV)<<16 |(PDIV)<<8 |(SDIV))
#define GET_ARM_CLOCK(baseclk)	s3c6400_get_pll(__raw_readl(S3C_APLL_CON),baseclk)

#endif //__S3CDVS_H_
