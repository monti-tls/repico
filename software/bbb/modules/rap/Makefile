#ccflags-y := -std=gnu99 -Wno-declaration-after-statement
obj-m += rap.o

all:
	@make --no-print-directory -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	@make --no-print-directory -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
