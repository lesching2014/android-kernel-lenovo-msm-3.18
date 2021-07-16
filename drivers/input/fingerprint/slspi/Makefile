#
# by warren zhao
#
SPI_DRV_MODE_SEL := PT

KBUILD_CFLAGS += -DGSL_DEV_SEL_device=spi_device -DGSL_DEV_SEL_driver=spi_driver -DGSL_DEV_SEL_dev_put=spi_dev_put -DGSL_DEV_SEL_set_drvdata=spi_set_drvdata -DGSL_DEV_SEL_get_drvdata=spi_get_drvdata -DGSL_DEV_SEL_register_driver=spi_register_driver -DGSL_DEV_SEL_unregister_driver=spi_unregister_driver

ifeq ($(SPI_DRV_MODE_SEL),PT)
KBUILD_CFLAGS += -DSLPT
KBUILD_CFLAGS += -UGSL_DEV_SEL_device -UGSL_DEV_SEL_driver -UGSL_DEV_SEL_dev_put -UGSL_DEV_SEL_set_drvdata -UGSL_DEV_SEL_get_drvdata -UGSL_DEV_SEL_driver -UGSL_DEV_SEL_register_driver -UGSL_DEV_SEL_unregister_driver -DGSL_DEV_SEL_device=platform_device -DGSL_DEV_SEL_driver=platform_driver -DGSL_DEV_SEL_dev_put=platform_dev_put -DGSL_DEV_SEL_set_drvdata=platform_set_drvdata -DGSL_DEV_SEL_get_drvdata=platform_get_drvdata -DGSL_DEV_SEL_register_driver=platform_register_driver -DGSL_DEV_SEL_unregister_driver=platform_unregister_driver -Dplatform_register_driver=platform_driver_register -Dplatform_unregister_driver=platform_driver_unregister -Dplatform_dev_put=platform_device_put            
endif

obj-$(CONFIG_SLSPI_DRIVER) += slspi.o slspi_board.o
obj-$(CONFIG_SLSPI_DRIVER) += sl_proc.o
