#
# Makefile for USB Core files and filesystem
#

#QMU Flags
ccflags-$(CONFIG_MTK_MUSB_QMU_SUPPORT) += -DMUSB_QMU_SUPPORT
##ccflags-$(CONFIG_MTK_MUSB_QMU_SUPPORT) += -DQMU_TASKLET
ccflags-$(CONFIG_MTK_MUSB_HOST_QMU_SUPPORT) += -DMUSB_QMU_SUPPORT_HOST
ccflags-$(CONFIG_MTK_MUSB_HOST_QMU_SUPPORT) += -DMUSB_QMU__HOST_BULK_RX_AUTO_SPLIT

usbcore-y := usb.o hub.o hcd.o urb.o message.o driver.o
usbcore-y += config.o file.o buffer.o sysfs.o endpoint.o
usbcore-y += devio.o notify.o generic.o quirks.o devices.o
usbcore-y += port.o

usbcore-$(CONFIG_PCI)		+= hcd-pci.o
usbcore-$(CONFIG_ACPI)		+= usb-acpi.o

obj-$(CONFIG_USB)		+= usbcore.o
