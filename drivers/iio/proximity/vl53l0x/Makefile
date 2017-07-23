#
# Makefile for the vl53L0X drivers.
#

ccflags-$(CONFIG_IIO_STMVL53L0X)	+= -Idrivers/iio/proximity/vl53l0x/inc

obj-$(CONFIG_IIO_STMVL53L0X)			+= vl53l0x.o
vl53l0x-objs				:= vl53l0x_module.o src/vl53l0x_api_calibration.o src/vl53l0x_api_core.o src/vl53l0x_api_ranging.o src/vl53l0x_api_strings.o src/vl53l0x_api.o src/vl53l0x_platform.o src/vl53l0x_i2c_platform.o
