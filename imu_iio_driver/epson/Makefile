#
# Makefile for Epson IMUs
#

epson_imus-y             := epson_imus_core.o
obj-$(CONFIG_EPSON_IMUS) += epson_imus.o

epson_lib-y += epson.o
epson_lib-$(CONFIG_IIO_EPSON_LIB_BUFFER) += epson_buffer.o
obj-$(CONFIG_IIO_EPSON_LIB) += epson_lib.o
