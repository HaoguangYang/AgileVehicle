KBUILD_EXTRA_SYMBOLS := /tmp/Module.symvers
KBUILD_CFLAGS += -g3

ifneq ($(KERNELRELEASE),)
	obj-m += klgd_ff_plugin.o

else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

endif
