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
obj-$(CONFIG_PLAT_S3C64XX)	+= irq-pl192.o
obj-y				+= ../plat-s3c24xx/devs.o
obj-y				+= ../plat-s3c24xx/gpio.o
obj-$(CONFIG_PLAT_S3C64XX)	+= time-s3c64xx.o
obj-y				+= clock.o

# Architecture dependant builds

obj-$(CONFIG_S3C64XX_DMA)	+= dma-pl080.o
obj-$(CONFIG_S3C_ADC)		+= adc-s3c64xx.o
obj-$(CONFIG_MACH_SMDK)		+= ../plat-s3c24xx/common-smdk.o
obj-$(CONFIG_PM)		+= pm.o
obj-$(CONFIG_PM)		+= sleep.o
