KBUILD_EXTRA_SYMBOLS := /tmp/Module.symvers

ifneq ($(KERNELRELEASE),)
	hid-logitech-y		:= hid-lg.o
	hid-logitech-$(CONFIG_LOGITECH_FF)	+= hid-lgff.o
	hid-logitech-$(CONFIG_LOGIRUMBLEPAD2_FF)	+= hid-lg2ff.o
	hid-logitech-$(CONFIG_LOGIG940_FF)	+= hid-lg3ff.o
	hid-logitech-$(CONFIG_LOGIWHEELS_FF)	+= hid-lg4ff.o
	obj-m += hid-logitech.o

else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

endif
