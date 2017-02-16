obj-m+=netlink.o

EXTRA_CFLAGS := -I/lib/modules/3.8.13-xenomai-r78/build/include/xenomai

all::
	make -C /lib/modules/3.8.13-xenomai-r78/build M=$(PWD) modules

modules:
	@echo "$(CFLAGS)"

clean::
	make -C /lib/modules/3.8.13-xenomai-r78/build M=$(PWD) clean
