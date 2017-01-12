# You will want to fix this line
KBUILD_EXTRA_SYMBOLS := /home/madcat/Devel/KLGD/Module.symvers
KBUILD_EXTRA_SYMBOLS += /home/madcat/Devel/KLGD_FFModule/plugin/Module.symvers
KBUILD_CFLAGS += -g3

ifneq ($(KERNELRELEASE),)
	obj-m += klgdff.o

else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

endif
