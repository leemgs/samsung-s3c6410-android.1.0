
#
# Makefile for the linux kernel.
#

# Object file lists.
obj-y			:= 
led-y 			:= leds.o
obj-n			:=
obj-			:=

# S3C6400 support files
obj-$(CONFIG_CPU_S3C6400)	+= s3c6400.o clock.o irq.o dma.o
obj-$(CONFIG_S3C64XX_DMA_SYSFS)	+= dma-sysfs.o
obj-$(CONFIG_S3C6400_PM)	+= pm.o

# Power Management support

# smdk6400  machine specific support
obj-$(CONFIG_MACH_SMDK6400)	+= mach-smdk6400.o

# LEDs support
led-$(CONFIG_MACH_SMDK6400)	+= leds-s3c6400.o
obj-$(CONFIG_LEDS)		+= $(led-y)

