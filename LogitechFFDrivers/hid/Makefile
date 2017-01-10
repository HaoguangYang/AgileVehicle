ifneq ($(KERNELRELEASE),)
	hid-y			:= hid-core.o hid-input.o
	hid-$(CONFIG_DEBUG_FS)		+= hid-debug.o
	hid-$(CONFIG_HIDRAW)		+= hidraw.o

	obj-m		+= hid.o

else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

endif
