
#
# Makefile for the linux kernel.
#

# Object file lists.
obj-y			:= 
led-y 			:= leds.o
obj-n			:=
obj-			:=

# smdk6410 machine-specific support
obj-$(CONFIG_CPU_S3C6410)	+= s3c6410.o clock.o
obj-$(CONFIG_MACH_SMDK6410)	+= mach-smdk6410.o

obj-$(CONFIG_CPU_S3C6410)	+= irq.o dma.o
obj-$(CONFIG_S3C64XX_DMA_SYSFS)	+= dma-sysfs.o
obj-$(CONFIG_S3C6410_PM)	+= pm.o

# Power Management support

# LEDs support
led-$(CONFIG_MACH_SMDK6410)	+= leds-s3c6410.o
obj-$(CONFIG_LEDS)		+= $(led-y)

# PWM Support

obj-$(CONFIG_PWM)               += pwm/

