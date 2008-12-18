# arch/arm/plat-s3c64xx/Makefile
#
# Copyright 2007 Simtec Electronics
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=


# Core files

obj-y				+= ../plat-s3c24xx/cpu.o
obj-$(CONFIG_PLAT_S5PC1XX)	+= irq-pl192.o
obj-y				+= ../plat-s3c24xx/devs.o
obj-$(CONFIG_PLAT_S5PC1XX)	+= time-s5pc1xx.o
obj-y				+= clock.o
obj-y				+= ../plat-s3c24xx/gpio.o
obj-$(CONFIG_PLAT_S5PC1XX)	+= gpio.o

# Architecture dependant builds

#obj-$(CONFIG_S5PC1XX_DMA)	+= dma-pl330.o dma-pl330-microcode.c
obj-$(CONFIG_MACH_SMDK)		+= ../plat-s3c24xx/common-smdk.o
