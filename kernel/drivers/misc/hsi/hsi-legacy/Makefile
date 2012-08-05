#
# Makefile for HSI drivers
#
u8500_hsi-objs := 	hsi_driver.o hsi_driver_int.o hsi_driver_dma.o \
			hsi_driver_if.o hsi_driver_bus.o

obj-$(CONFIG_U8500_HSI_LEGACY)	+= u8500_hsi.o

obj-$(CONFIG_U8500_HSI_TEST_PROTOCOL_DRIVER) += hsi_test_protocol_driver.o
