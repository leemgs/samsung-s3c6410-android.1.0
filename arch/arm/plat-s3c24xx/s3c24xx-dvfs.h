/*
	2008.04.14. All of AP are  merged single application and definitin
	in order to be matcing dvs and dfs concept
	These are confirm on SMDK2450, SMDK2416
*/
#ifndef __S3C24XXDVFS_H_
#define __S3C24XXDVFS_H_

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

#define ARM_LE  	S3C2410_GPF5
#define DVS_OE     S3C2410_GPF6
#define INT_LE 	S3C2410_GPF7

#define CONTROl_SET(pin,to) s3c2410_gpio_setpin(pin, to);
#define ARM_LE_OUTP   S3C2410_GPF5_OUTP
#define DVS_OE_OUTP  S3C2410_GPF6_OUTP
#define INT_LE_OUTP 	 S3C2410_GPF7_OUTP

#define LTC3714_DATA1  S3C2410_GPC0
#define LTC3714_DATA2  S3C2410_GPC5
#define LTC3714_DATA3  S3C2410_GPC6
#define LTC3714_DATA4  S3C2410_GPC7
#define LTC3714_DATA5  S3C2410_GPB2

#define LTC3714_OUTP1  S3C2410_GPC0_OUTP
#define LTC3714_OUTP2  S3C2410_GPC5_OUTP
#define LTC3714_OUTP3  S3C2410_GPC6_OUTP
#define LTC3714_OUTP4  S3C2410_GPC7_OUTP
#define LTC3714_OUTP5  S3C2410_GPB2_OUTP

#define GPIO_CFG(pin, fnc)	s3c2410_gpio_cfgpin((pin),(fnc))
#define GPIO_PULLUP(pin, to)	s3c2410_gpio_pullup((pin),(to))

#define ARM_PLL_CON 	     	S3C2443_MPLLCON
#define ARM_CLK_DIV	     	S3C2443_CLKDIV0
#define MPLL_CON		S3C2443_MPLLCON
#define ARM_DIV_RATIO_BIT	9
#define ARM_DIV_MASK    (0xf<<ARM_DIV_RATIO_BIT)
#define HCLK_DIV_RATIO_BIT	0
#define HCLK_DIV_MASK   (0x3<<HCLK_DIV_RATIO_BIT)
#define DVFS_MASK   	     (0x1<<13)

#define READ_ARM_DIV    (((__raw_readl(ARM_CLK_DIV)&ARM_DIV_MASK)>>9) + 1)
#define PLL_CALC_VAL(MDIV,PDIV,SDIV)	((MDIV)<<14 |(PDIV)<<5 |(SDIV))
#define GET_ARM_CLOCK(baseclk)	s3c2443_get_mpll(__raw_readl(S3C2443_MPLLCON),baseclk)



#endif //__S3C24XXDVS_H_
