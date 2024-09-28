#
# Kernel makefile for TDMB(MTV818A)
#

EXTRA_CFLAGS += -I. -I./src -I$(PRJ_ROOT_DIR)/src

ccflags-y += -Idrivers/misc/mtv -Idrivers/misc/mtv/src

#obj-m	+= mtv818a.o

#mtv818a-objs		

obj-y      := mtv.o mtv_gpio.o
obj-y      += mtv_spi.o
obj-y      += mtv_isr.o 
obj-y      += raontv.o 
obj-y      += raontv_port.o
obj-y      += raontv_rf.o
obj-y      += raontv_isdbt.o

	
