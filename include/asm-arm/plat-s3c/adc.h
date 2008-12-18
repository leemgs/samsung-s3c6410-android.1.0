/* linux/include/asm-arm/plat-s3c/adc.h
 *
 * Copyright (c) 2008 Jongpill Lee <Jongpill.Lee>
 *
 * This program is header file for ADC Driver in s3c64xx of SAMSUNG ELECTRONICS
 *
 */

extern unsigned int s3c_adc_convert(void __iomem * reg_base, unsigned int s3c_adc_port);
extern struct s3c_adc_cfg s3c_adc_platform;

struct s3c_adc_request
{
	/* for linked list */
	struct list_head *list;
	/* after finish ADC sampling, s3c_adc_request function call this function with three parameter */
	void (*callback)( int channel, unsigned long int param, unsigned short sample);
	/* for private data */
	unsigned long int param;
	/* selected channel for ADC sampling */
	int channel;
};

struct s3c_adc_cfg{
	/* if you need to use some platform data, add in here*/
	int delay;
	int presc;
	int resolution;
};
