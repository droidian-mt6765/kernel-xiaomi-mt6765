/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __CUST_ACC_H__
#define __CUST_ACC_H__

#include <linux/of.h>
#include <linux/types.h>
#define G_CUST_I2C_ADDR_NUM 2

//xjl 20200618 for yk673v6
#if defined(CONFIG_CUSTOM_KERNEL_SENSORHUB)
#undef USE_OLD_SENSOR_DTS_ARCH_ACCEL_GYRO
#else
#define USE_OLD_SENSOR_DTS_ARCH_ACCEL_GYRO
#endif

struct acc_hw {
	int i2c_num;   /*!< the i2c bus used by the chip */
	int direction; /*!< the direction of the chip */
	int power_id;  /*!< the VDD LDO ID of the chip */
	int power_vol; /*!< the VDD Power Voltage used by the chip */
	int firlen;    /*!< the length of low pass filter */
	int (*power)(struct acc_hw *hw, unsigned int on, char *devname);
	/*!< i2c address list,for chips which has different addresses with
	 * different HW layout.
	 */
	unsigned char i2c_addr[G_CUST_I2C_ADDR_NUM];
	int power_vio_id;  /*!< the VIO LDO ID of the chip */
	int power_vio_vol; /*!< the VIO Power Voltage used by the chip */
	bool is_batch_supported;
};

#ifndef USE_OLD_SENSOR_DTS_ARCH_ACCEL_GYRO
int get_accel_dts_func(struct device_node *node, struct acc_hw *hw);
#else
int get_accel_dts_func(const char *, struct acc_hw*);
#endif
#endif
