#ifndef _REGS_S3C_CLOCK_H_
#define _REGS_S3C_CLOCK_H_

#if defined(CONFIG_PLAT_S3C24XX)
#include <asm/arch/regs-s3c2443-clock.h>
#elif defined(CONFIG_PLAT_S3C64XX)
#include <asm/arch/regs-s3c6400-clock.h>
#elif defined(CONFIG_PLAT_S5PC1XX)
#include <asm/arch/regs-s5pc100-clock.h>
#endif

#endif

