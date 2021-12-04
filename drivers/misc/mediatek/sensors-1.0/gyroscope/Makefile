# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include
obj-y   +=  gyroscope.o gyro_factory.o


obj-$(CONFIG_MTK_GYROHUB)      +=  gyrohub/
obj-$(CONFIG_MTK_LSM6DS3HGY)   +=  lsm6ds3hgy/
obj-$(CONFIG_MTK_BMI160GY_I2C) +=  bmi160gy-i2c/
obj-$(CONFIG_MTK_MPU6050GY)   +=  mpu6050gy/

## new compile condition according to ProjectConfig.mk by xen 20150325
ifeq ($(CUSTOM_KERNEL_GYROSCOPE),yes)
ifneq ($(findstring lsm6ds3gy_new, $(strip $(YK_CUSTOM_KERNEL_GYROSCOPE))),)
obj-y   +=  lsm6ds3gy-new/
endif
ifneq ($(findstring mpu6050gy, $(strip $(YK_CUSTOM_KERNEL_GYROSCOPE))),)
obj-y   +=  mpu6050gy/
endif

ifneq ($(findstring icm206xxgy_new, $(strip $(YK_CUSTOM_KERNEL_GYROSCOPE))),)
obj-y   +=  icm206xxgy-new/
endif

endif
