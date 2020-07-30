#ccflags-y += -D QUALCOMM_SMB1357
ccflags-y += -I$(srctree)/drivers/misc/mediatek/power/$(CONFIG_MTK_PLATFORM)
ccflags-y += -D DONGLE_CABLE_INSERT=1
#ccflags-y += -D QUICK_CHARGE_SUPPORT

obj-$(CONFIG_SLIMPORT_COLORADO3)	+= co3.o
co3-objs := slimport.o \
	    slimport_tx_drv.o \
	    slimport_tx_cec.o \
	    anx_slimport.o \
#	    quick_charge.o 
