/* linux/include/asm-arm/arch-s3c2410/memory.h
 *  from linux/include/asm-arm/arch-rpc/memory.h
 *
 *  Copyright (C) 1996,1997,1998 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#if defined (CONFIG_PLAT_S5PC1XX) 
#define PHYS_OFFSET	UL(0x20000000)

#elif defined (CONFIG_PLAT_S3C64XX) 
#define PHYS_OFFSET	UL(0x50000000)

#else
/*
 * DRAM starts at 0x30000000 for S3C2410/S3C2440
 * and at 0x0C000000 for S3C2400
 */
#ifdef CONFIG_CPU_S3C2400
#define PHYS_OFFSET	UL(0x0C000000)
#else
#define PHYS_OFFSET	UL(0x30000000)
#endif

#endif

#define __virt_to_bus(x) __virt_to_phys(x)
#define __bus_to_virt(x) __phys_to_virt(x)

#endif
