obj-m+=fifo_skt.o

EXTRA_CFLAGS := -I/lib/modules/3.8.13xenomai-bone53/build/include/xenomai

all::
	make -C /lib/modules/3.8.13xenomai-bone53/build M=$(PWD) modules

modules:
	@echo "$(CFLAGS)"

clean::
	make -C /lib/modules/3.8.13xenomai-bone53/build M=$(PWD) clean
