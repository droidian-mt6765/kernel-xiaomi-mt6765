/*
 * mir3da sensor driver
 * Copyright (C) 2016 MiraMEMS, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/ioctl.h>
#include <hwmsensor.h>
#include <linux/miscdevice.h>
#include "../step_counter.h"
#include "../../accelerometer/mir3da/mir3da_cust.h"
#include "../../accelerometer/mir3da/mir3da_core.h"

#include "../../drivers/misc/mediatek/sensors-1.0/accelerometer/inc/accel.h"

//#define MI_DATA(format, ...)            if(DEBUG_DATA&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
//#define MI_MSG(format, ...)             if(DEBUG_MSG&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
//#define MI_ERR(format, ...)             if(DEBUG_ERR&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
//#define MI_FUN                          if(DEBUG_FUNC&Log_level){printk(KERN_ERR MI_TAG "%s is called, line: %d\n", __FUNCTION__,__LINE__);}


static atomic_t mir3da_sc_trace;

static int mir3da_sc_init_flag =  -1;

static int mir3da_sc_local_init(void);
static int mir3da_sc_remove(void);
static struct step_c_init_info mir3da_sc_init_info = {
	.name = "MIR3DA_SC",
	.init = mir3da_sc_local_init,
	.uninit = mir3da_sc_remove,
//	struct platform_driver *platform_diver_addr;
};


/*=======================================================================================*/
/* I2C Primitive Functions Section							 */
/*=======================================================================================*/

/* Share i2c function with Accelerometer		*/
/* Function is defined in accelerometer/mir3da	*/

/*=======================================================================================*/
/* Vendor Specific Functions Section							 */
/*=======================================================================================*/

/*---------------- read step count --------------------------------------------------*/
static int mir3da_sc_ReadSensorData(uint32_t *value)
{
	*value = mir3da_sc_get_step();

	pr_info("Step Counter Data - %04x\n", (*value));

	return 0;
}

/*---------------- init step count, need to finish for step_count ------------------*/
static int mir3da_sc_init_client(bool enable)
{
	//int res = 0;

	printk("zzz mir3da_sc_init_client");
	mir3da_setPowerMode(MIR3DA_SENSOR_TYPE_STEP_COUNTER, true);
	mir3da_step_register_init();
	mir3da_setPowerMode(MIR3DA_SENSOR_TYPE_STEP_COUNTER, enable);
	
	printk("zzz exit mir3da_sc_init");

	pr_info("mir3da_sc_init_client OK!\n");

	// return MIR3DA_SUCCESS;
	return 0;
}


/*=======================================================================================*/
/* Driver Attribute Section								 */
/*=======================================================================================*/

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&mir3da_sc_trace));

	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if (!kstrtoint(buf, 16, &trace))
		atomic_set(&mir3da_sc_trace, trace);
	else
		pr_err("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;


	return res;
}


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value,   store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value,   NULL);

static struct driver_attribute *mir3da_sc_attr_list[] = {
	&driver_attr_trace,		/*trace log*/
	&driver_attr_status,

};
/*----------------------------------------------------------------------------*/
static int mir3da_sc_create_attr(struct device_driver *driver)
{
	int idx;
	int res = 0;
	int num = (int)(sizeof(mir3da_sc_attr_list)/sizeof(mir3da_sc_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		res = driver_create_file(driver, mir3da_sc_attr_list[idx]);
		if (0 != res) {
			pr_err("driver_create_file (%s) = %d\n", mir3da_sc_attr_list[idx]->attr.name, res);
			break;
		}
	}
	return res;
}

static int mir3da_sc_delete_attr(struct device_driver *driver)
{
	int idx;
	int res = 0;
	int num = (int)(sizeof(mir3da_sc_attr_list)/sizeof(mir3da_sc_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, mir3da_sc_attr_list[idx]);

	return res;
}

/*=======================================================================================*/
/* Misc - Factory Mode (IOCTL) Device Driver Section					 */
/*=======================================================================================*/

static struct miscdevice mir3da_sc_device = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "step_counter",
};

/*=======================================================================================*/
/* Misc - I2C HAL Support Section							 */
/*=======================================================================================*/

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int mir3da_sc_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	printk("zzz mir3da_sc_open_report_data");
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
//do on/off
static int mir3da_sc_enable_nodata(int en)
{
	bool power = false;
	printk("%s is called, line: %d\n", __FUNCTION__,__LINE__);
	
	if (1 == en) {
		power = true;
	}else{
		power = false;
	}
	
	mir3da_setPowerMode(MIR3DA_SENSOR_TYPE_STEP_COUNTER, power);
	
	return 0;
}

static int mir3da_floor_c_get_data(uint32_t *value, int *status)
{
	uint32_t step_count = 0;
	mir3da_sc_ReadSensorData(&step_count);

	*value = step_count;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
static int mir3da_sc_floor_c_set_delay(u64 delay)
{
	return 0;
}

static int mir3da_sc_step_c_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int mir3da_sc_step_c_flush(void)
{
	return step_c_flush_report();	
	//return 0;
}

static int mir3da_sc_step_d_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int mir3da_sc_step_d_flush(void)
{
	return step_d_flush_report();
	//return 0;
}

static int mir3da_sc_floor_c_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int mir3da_sc_floor_c_flush(void)
{
	return floor_c_flush_report();
}

static int mir3da_sc_smd_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int mir3da_sc_smd_flush(void)
{
	return smd_flush_report();
}

static int mir3da_sc_floor_c_enable_nodata(int en)
{

	pr_info("mir3da_sc_floor_c_enable_nodata OK!\n");
	return 0;
}

static int mir3da_sc_step_c_set_delay(u64 delay)
{
	return 0;
}

static int mir3da_sc_step_d_set_delay(u64 delay)
{
	return 0;
}

static int mir3da_sc_enable_significant(int en)
{
	return 0;
}

static int mir3da_sc_enable_step_detect(int en)
{
	return 0;
}

static int mir3da_sc_get_data(uint32_t *value, int *status)
{
	uint32_t step_count = 0;

	mir3da_sc_ReadSensorData(&step_count);

	*value = step_count;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}



static int mir3da_sc_get_data_step_d(uint32_t *value, int *status)
{
	return 0;
}

static int mir3da_sc_get_data_significant(uint32_t *value, int *status)
{
	return 0;
}

/*=======================================================================================*/
/* HAL Attribute Registration Section							 */
/*=======================================================================================*/

static int mir3da_sc_attr_create(void)
{
	struct step_c_control_path ctl = {0};
	struct step_c_data_path data = {0};
	int res = 0;
	
	printk("zzz mir3da_sc_attr_create");

	pr_info();

	res = mir3da_sc_init_client(false);
	if (res)
		goto exit_init_failed;

	/* misc_register() for factory mode, engineer mode and so on */
	res = misc_register(&mir3da_sc_device);
	if (res) {
		pr_err("mir3da_sc_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}

	/* Crate platform_driver attribute */
	res = mir3da_sc_create_attr(&(mir3da_sc_init_info.platform_diver_addr->driver));
	if (res) {
		pr_err("mir3da create attribute err = %d\n", res);
		goto exit_create_attr_failed;
	}

	/*-----------------------------------------------------------*/
	/* Fill and Register the step_c_control_path and step_c_data_path */
	/*-----------------------------------------------------------*/

	/* Fill the step_c_control_path */
	ctl.open_report_data = mir3da_sc_open_report_data;		/* return 0 */
	ctl.enable_nodata = mir3da_sc_enable_nodata;			/* return 0 */
	ctl.enable_step_detect  = mir3da_sc_enable_step_detect;	/* return 0 */
	ctl.enable_significant  = mir3da_sc_enable_significant;	/* return 0 */
	ctl.enable_floor_c = mir3da_sc_floor_c_enable_nodata;
	ctl.step_c_set_delay  = mir3da_sc_step_c_set_delay;		/* return 0 */
	ctl.step_d_set_delay  = mir3da_sc_step_d_set_delay;		/* return 0 */
	
	ctl.floor_c_set_delay  = mir3da_sc_floor_c_set_delay;	/* return 0 */
	ctl.step_c_batch = mir3da_sc_step_c_batch;				/* return 0 */
	ctl.step_c_flush = mir3da_sc_step_c_flush;
	ctl.step_d_batch = mir3da_sc_step_d_batch;				/* return 0 */
	ctl.step_d_flush = mir3da_sc_step_d_flush;
	ctl.smd_batch = mir3da_sc_smd_batch;					/* return 0 */
	ctl.smd_flush = mir3da_sc_smd_flush;
	ctl.floor_c_batch = mir3da_sc_floor_c_batch;			/* return 0 */
	ctl.floor_c_flush = mir3da_sc_floor_c_flush;
	ctl.is_report_input_direct = false;
	ctl.is_counter_support_batch = false;
	ctl.is_detector_support_batch = true;
	ctl.is_smd_support_batch = false;
	ctl.is_floor_c_support_batch = false;
		
	/* Register the step_c_control_path */
	res = step_c_register_control_path(&ctl);
	if (res) {
		pr_err("step_c_register_control_path err\n");
		goto exit_kfree;
	}

	/* Fill the step_c_data_path */
	data.get_data = mir3da_sc_get_data;			/* it's key point */
	data.get_data_step_d = mir3da_sc_get_data_step_d;				/* return 0 */
	data.get_data_significant = mir3da_sc_get_data_significant;		/* return 0 */
	data.get_data_floor_c = mir3da_floor_c_get_data;
	data.vender_div = 1000;

	/* Register the step_c_data_path */
	res = step_c_register_data_path(&data);
	if (res) {
		pr_err("step_c_register_data_path err = %d\n", res);
		goto exit_kfree;
	}

	/* Set init_flag = 0 and return */
	mir3da_sc_init_flag = 0;

	printk("zzz exit mir3da_sc_attr_create");

	pr_info("%s: OK\n", __func__);
	return 0;

exit_misc_device_register_failed:
	misc_deregister(&mir3da_sc_device);
exit_create_attr_failed:
exit_init_failed:
exit_kfree:
/* exit: */
	mir3da_sc_init_flag =  -1;
	pr_err("%s: err = %d\n", __func__, res);
	return res;
}

static int mir3da_sc_attr_remove(void)
{
	int res = 0;

	res = mir3da_sc_delete_attr(&(mir3da_sc_init_info.platform_diver_addr->driver));
	if (res)
		pr_err("mir3da_sc_delete_attr fail: %d\n", res);
	else
		res = 0;

	misc_deregister(&mir3da_sc_device);
	if (res)
		pr_err("misc_deregister fail: %d\n", res);
	else
		res = 0;

	return res;
}

/*=======================================================================================*/
/* Kernel Module Section								 */
/*=======================================================================================*/

static int mir3da_sc_remove(void)
{
	mir3da_sc_attr_remove();

	return 0;
}
extern int mir3da_init_flag;
static int mir3da_sc_local_init(void)
{
	if (-1 == mir3da_init_flag)
		return -1;

	mir3da_sc_attr_create();

	printk("zzz mir3da_sc_local_init");

	if (-1 == mir3da_sc_init_flag)
		return -1;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init mir3da_sc_init(void)
{
	step_c_driver_add(&mir3da_sc_init_info);

	printk("zzz _init mir3da_sc_init");

	return 0;
}

static void __exit mir3da_sc_exit(void)
{

}

/*----------------------------------------------------------------------------*/
module_init(mir3da_sc_init);
module_exit(mir3da_sc_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mir3da step counter driver");
