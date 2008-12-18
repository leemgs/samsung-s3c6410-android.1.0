/* linux/include/asm-arm/plat-s3c24xx/common-smdk.h
 *
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Common code for SMDK2410 and SMDK2440 boards
 *
 * http://www.fluff.org/ben/smdk2440/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

extern struct platform_device s3c_device_lcd;
extern struct platform_device s3c_device_nand;

extern struct platform_device s3c_device_onenand;
extern struct flash_platform_data s3c_onenand_data;

extern void smdk_machine_init(void);
