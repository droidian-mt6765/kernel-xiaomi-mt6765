/*
 * Copyright (C) 2012 Senodia Corporation.
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

#include "cust_mag.h"
#include "st480.h"
#include "mag.h"
/*----------------------------------------------------------------------------*/
#define MSE_TAG                  "MSENSOR"
#define MSE_FUN(f)               pr_debug(MSE_TAG" %s\r\n", __func__)
#define MSE_ERR(fmt, args...)    pr_err(MSE_TAG" %s %d : \r\n"fmt, __func__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)    pr_debug(MSE_TAG fmt, ##args)

#define MSE_VER(fmt, args...)   ((void)0)

#define POWER_NONE_MACRO MT65XX_POWER_NONE
static int st480_init_flag =0;

/*----------------------------------------------------------------------------*/
#define ST480_AXIS_X            0
#define ST480_AXIS_Y            1
#define ST480_AXIS_Z            2
#define ST480_AXES_NUM          3

#define ST480_BUFSIZE          256


/* Maintain  cust info here */
static struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

static int st480_init_flag;	/* 0<==>OK -1 <==> fail */


#define CONVERT_O_DIV		1000
#define CONVERT_M_DIV		1000

/*----------------------------------------------------------------------------*/
/*static struct i2c_board_info i2c_st480 __initdata = { I2C_BOARD_INFO("st480", (0X0c)) }; */
/*static struct platform_driver st480_sensor_driver;*/

/* For  driver get cust info */
struct mag_hw *get_cust_mag(void)
{
	return &mag_cust;
}

static DECLARE_WAIT_QUEUE_HEAD(open_wq);

struct st480_data {
	struct i2c_client *client;
	struct mag_hw *hw;
	struct hwmsen_convert cvt;
	atomic_t layout;
	atomic_t trace;
};

static struct st480_data *st480;

static atomic_t m_flag;
static atomic_t open_flag;

static atomic_t dev_open_count;

volatile static short st480d_delay = ST480_DEFAULT_DELAY;
volatile static short st480d_name = 480;
int st480_deviceid = 0x0c;

struct mag_3{
	s16  mag_x,
	     mag_y,
	     mag_z;
};
volatile static struct mag_3 mag;
static int	st480_local_init(void);
static int st480_remove(void);
static int st480_get_data(int* x ,int* y,int* z, int* status);

static struct mag_init_info st480_init_info = {
	.name = "st480",
	.init = st480_local_init,
	.uninit = st480_remove,
};

/*----------------------------------------------------------------------------*/
static void st480_power(struct mag_hw *hw, unsigned int on) 
{
#if 0
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)
	{        
		MSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)
		{
			MSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "st480")) 
			{
				MSE_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "st480")) 
			{
				MSE_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
#endif	
}


/*
 * st480 i2c transfer
 * read/write
 */
static int st480_i2c_transfer_data(struct i2c_client *client, int len, char *buf, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = len,
		 .buf = buf,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = buf,
		 },
	};

	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		pr_err(" st480　megnetic_i2c_read_data: transfer error\n");
		return EIO;
	} else
		return 0;
}

/*
 * Device detect and init
 *
 */
static int client_init(struct i2c_client *client)
{
	int ret;
	unsigned char buf[5];
	unsigned char data[1];

	MSE_FUN();
	memset(buf, 0, 5);
	memset(data, 0, 1);

	buf[0] = READ_REGISTER_CMD;
	buf[1] = 0x00;
	ret = 0;

	while (st480_i2c_transfer_data(client, 2, buf, 3) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 2, buf, 3) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			return -EIO;
		}
	}

/***
	if(buf[2] != ST480_DEVICE_ID)
	{
		MSE_ERR("st480 ic not exist!");
		return -ENODEV;
	}
**/

	MSE_LOG("st480 device id is %x\n", buf[2]);

/* init register step 1 */
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = ONE_INIT_DATA_HIGH;
	buf[2] = ONE_INIT_DATA_LOW;
	buf[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			return -EIO;
		}
	}

/* init register step 2 */
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TWO_INIT_DATA_HIGH;
	buf[2] = TWO_INIT_DATA_LOW;
	buf[3] = TWO_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			return -EIO;
		}
	}

/* set calibration register */
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = CALIBRATION_DATA_HIGH;
	buf[2] = CALIBRATION_DATA_LOW;
	buf[3] = CALIBRATION_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			return -EIO;
		}
	}

/* set mode config */
	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(client, 1, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 1, buf, 1) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			return -EIO;
		}
	}

	return 0;
}

static void st480_work_func(int *x, int *y, int *z)
{
	char buffer[7];
	int ret;
	int mag_tmp[ST480_AXES_NUM];
	int data[3]={0,0,0}, raw[3]={0,0,0};

	memset(buffer, 0, 7);

	buffer[0] = READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 7) != 0) {
		ret++;

		if (st480_i2c_transfer_data(st480->client, 1, buffer, 7) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			break;
		}
	}

	if (!((buffer[0] >> 4) & 0X01)) {
		raw[0] = ((buffer[1] << 8) | buffer[2]);
		raw[1] = ((buffer[3] << 8) | buffer[4]);
		raw[2] = ((buffer[5] << 8) | buffer[6]);

		/* MSE_LOG("st480 raw data: x = %d, y = %d, z = %d\n",mag.mag_x,mag.mag_y,mag.mag_z); */
	}
	mag_tmp[st480->cvt.map[ST480_AXIS_X]] = st480->cvt.sign[ST480_AXIS_X] * raw[0];
	mag_tmp[st480->cvt.map[ST480_AXIS_Y]] = st480->cvt.sign[ST480_AXIS_Y] * raw[1];
	mag_tmp[st480->cvt.map[ST480_AXIS_Z]] = st480->cvt.sign[ST480_AXIS_Z] * raw[2];

	*x = data[0] = mag_tmp[0];
	*y = data[1] = mag_tmp[1];
	*z = data[2] = mag_tmp[2];

	buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 1) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			break;
		}
	}
}

/*----------------------------------------------------------------------------*/
static int st480_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 30)) {
		return -1;
	}
	if (!(st480->client)) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "st480 Chip");
	return 0;
}



static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[256];

	st480_ReadChipInfo(strbuf, 256);
	return sprintf(buf, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	char strbuf[ST480_BUFSIZE];
	int x, y, z;
	st480_work_func(&x, &y, &z);
	sprintf(strbuf, "%04x %04x %04x", x, y, z);
	/* st480_ReadSensorData(strbuf, ST480_BUFSIZE); */
	return sprintf(buf, "%s\n", strbuf);
}


#if 0
static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
	char strbuf[ST480_BUFSIZE];

	st480_ReadCaliData(strbuf, ST480_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct st480_data *data = st480;

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		       data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0],
		       data->cvt.sign[1], data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1],
		       data->cvt.map[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct st480_data *data = st480;
	int layout = 0;

	if (1 == sscanf(buf, "%d", &layout)) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {
			MSE_ERR("HWMSEN_GET_CONVERT function OK!\r\n");
		} else if (!hwmsen_get_convert(data->hw->direction, &data->cvt)) {
			MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		} else {
			MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else {
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct st480_data *data = st480;
	ssize_t len = 0;

	if (data->hw) {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
				data->hw->i2c_num, data->hw->direction, data->hw->power_id,
				data->hw->power_vol);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct st480_data *obj = st480;

	if (NULL == obj) {
		MSE_ERR("ST480_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct st480_data *obj = st480;
	int trace;
	if(NULL == obj )
	{
		MSE_ERR("st480_i2c_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else 
	{
		//	MSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
/*----------------------------------------------------------------------------*/
static struct driver_attribute *st480_attr_list[] = {
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
};

/*----------------------------------------------------------------------------*/
static int st480_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(st480_attr_list) / sizeof(st480_attr_list[0]));

	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		if ((err = driver_create_file(driver, st480_attr_list[idx]))) {
			MSE_ERR("driver_create_file (%s) = %d\n", st480_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int st480_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(st480_attr_list) / sizeof(st480_attr_list[0]));

	if (driver == NULL) {
		return -EINVAL;
	}


	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, st480_attr_list[idx]);
	}


	return err;
}


static int st480_enable(int en)
{
	int value = 0;
	int err = 0;
	value = en;
	
	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
		
	}
	else
	{
		atomic_set(&m_flag, 0);
		atomic_set(&open_flag, 0);
		
	}	
	wake_up(&open_wq);
	return err;
}

static int st480_set_delay(u64 ns)
{
	int value = 0;
	value = (int)ns/1000/1000;	// convert into ms
	if(value < ST480_DEFAULT_DELAY)
		value = ST480_DEFAULT_DELAY;
	st480d_delay = value;
	return 0;
}

static int st480_open_report_data(int open)
{
        return 0;
}

int16_t temp_magdata[3]={0,0,0};

static int st480_get_data(int* x ,int* y,int* z, int* status)
{
	int16_t data[3]={0,0,0};

	int ret;
	
	char buffer[7];
	//int ret;
	int mag_tmp[ST480_AXES_NUM];

	memset(buffer, 0, 7);

	buffer[0] = READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 7) != 0) {
		ret++;

		if (st480_i2c_transfer_data(st480->client, 1, buffer, 7) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			break;
		}
	}

	if (!((buffer[0] >> 4) & 0X01)) {
		data[0]  = ((buffer[1] << 8) | buffer[2]);
		data[1]  = ((buffer[3] << 8) | buffer[4]);
		data[2]  = ((buffer[5] << 8) | buffer[6]);

		/* MSE_LOG("st480 raw data: x = %d, y = %d, z = %d\n",mag.mag_x,mag.mag_y,mag.mag_z); */
	}
	mag_tmp[st480->cvt.map[ST480_AXIS_X]] = st480->cvt.sign[ST480_AXIS_X] * data[0] ;
	mag_tmp[st480->cvt.map[ST480_AXIS_Y]] = st480->cvt.sign[ST480_AXIS_Y] * data[1] ;
	mag_tmp[st480->cvt.map[ST480_AXIS_Z]] = st480->cvt.sign[ST480_AXIS_Z] * data[2] ;

	if( mag_tmp[0]!=0 && mag_tmp[1]!=0)
	{
			temp_magdata[0] = mag_tmp[0];
			temp_magdata[1] = mag_tmp[1];
			temp_magdata[2] = mag_tmp[2];
	}
	
	if( mag_tmp[0]==0 && mag_tmp[1]==0)
	{
		mag_tmp[0] = temp_magdata[0];
			mag_tmp[1] = temp_magdata[1];
				mag_tmp[2] = temp_magdata[2];
	}
	
	*x = mag_tmp[0] * CONVERT_M_DIV;
	*y = mag_tmp[1] * CONVERT_M_DIV;
	*z = mag_tmp[2]* CONVERT_M_DIV;
	*status = 3;
	printk("st480 data x = %d, y = %d, z = %d\n",*x,*y,*z);
	buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 1) == 0) {
			break;
		}
		if (ret > MAX_FAILURE_COUNT) {
			break;
		}
	}

	
	return 0;
}


static int st480_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int st480_flush(void)
{
	return mag_flush_report();
}

static int st480_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
        int err;

        err = st480_enable(enabledisable == true ? 1 : 0);
        if (err) {
                MSE_ERR("%s enable sensor failed!\n", __func__);
                return -1;
        }
        err = st480_batch(0, sample_periods_ms * 1000000, 0);
        if (err) {
                MSE_ERR("%s enable set batch failed!\n", __func__);
                return -1;
        }
        return 0;
}
 static int st480_factory_get_data(int32_t data[3], int *status)
 {
       /* get raw data */
       return  st480_get_data(&data[0], &data[1], &data[2], status);
}

static int st480_factory_get_raw_data(int32_t data[3])
{
        MSE_ERR("do not support st480_factory_get_raw_data!\n");
        return 0;
}
static int st480_factory_enable_calibration(void)
{
        return 0;
}
static int st480_factory_clear_cali(void)
{
        return 0;
}

/*----------------------------------------------------------------------------*/
static int st480_factory_set_cali(int32_t data[3])
{
        return 0;
}
static int st480_factory_get_cali(int32_t data[3])
{
        return 0;
}
static int st480_factory_do_self_test(void)
{
        return 0;
}


#if SENSOR_AUTO_TEST
static int sensor_test_read(void)
{
	int *x,*y,*z;
	st480_work_func(&x, &y, &z);
	return 0;
}

static int auto_test_read(void *unused)
{
	while (1) {
		sensor_test_read();
		msleep(200);
	}
	return 0;
}
#endif


static struct mag_factory_fops st480_factory_fops = {
	.enable_sensor = st480_factory_enable_sensor,
	.get_data = st480_factory_get_data,
	.get_raw_data = st480_factory_get_raw_data,
	.enable_calibration = st480_factory_enable_calibration,
	.clear_cali = st480_factory_clear_cali,
	.set_cali = st480_factory_set_cali,
	.get_cali = st480_factory_get_cali,
	.do_self_test = st480_factory_do_self_test,
};

static struct mag_factory_public st480_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &st480_factory_fops,
};






static int st480_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	//int ret;
#ifndef USE_OLD_SENSOR_DTS_ARCH
	struct device_node *node = NULL;
#endif
	struct mag_control_path ctl={0};
	struct mag_data_path mag_data={0};

	//printk("st480_i2c_probe\n");

	MSE_FUN();
#if SENSOR_AUTO_TEST
	struct task_struct *thread;
#endif

#ifndef USE_OLD_SENSOR_DTS_ARCH
	node = of_find_compatible_node(NULL, NULL, "mediatek,st480");
	if (node == NULL) {
		MSE_ERR("dts node not found. node = NULL\n");
		return -EFAULT;
	}

	err = get_mag_dts_func(node, hw);
	if (err) {
		MSE_ERR("get dts info fail\n");
		return -EFAULT;
	}
#endif

	/* Allocate memory for driver data */
	st480 = kzalloc(sizeof(struct st480_data), GFP_KERNEL);
	if (!st480) {
		MSE_ERR("SENODIA st480_i2c_probe: memory allocation failed.\n");
		err = -ENOMEM;
		goto exit1;
	}

	memset(st480, 0, sizeof(struct st480_data));

	st480->hw = hw;
	err = hwmsen_get_convert(st480->hw->direction, &st480->cvt);
	if (err) {
		MSE_ERR("invalid direction: %d\n", st480->hw->direction);
		goto exit1;
	}


	atomic_set(&st480->layout, st480->hw->direction);
	atomic_set(&st480->trace, 0);
	init_waitqueue_head(&open_wq);
	client->addr = 0x0C; //st480->hw->i2c_addr[0]; //xen 20180124
	st480->client = client;
	i2c_set_clientdata(client, st480);

	if (client_init(st480->client) != 0) {
		MSE_ERR("st480mc setup error!\n");
		goto exit2;
	}

	err = mag_factory_device_register(&st480_factory_device);
	if (err) {
		MSE_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}
	
	if((err = st480_create_attr(&(st480_init_info.platform_diver_addr->driver))))
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit3;
	}

	ctl.is_use_common_factory = false;
	ctl.enable = st480_enable;
	ctl.set_delay  = st480_set_delay;
	ctl.open_report_data = st480_open_report_data;
	ctl.batch = st480_batch;
	ctl.flush = st480_flush;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = st480->hw->is_batch_supported;
	strlcpy(ctl.libinfo.libname, "st480", sizeof(ctl.libinfo.libname));
	ctl.libinfo.layout = atomic_read(&st480->layout);
	ctl.libinfo.deviceid = st480_deviceid;

	err = mag_register_control_path(&ctl);

	if(err)
	{
		MSE_ERR("register mag control path err\n");

		goto exit2;
	}

	mag_data.div = CONVERT_M_DIV;
	mag_data.get_data = st480_get_data;

	err = mag_register_data_path(&mag_data);
	if (err) 
	{
		MSE_ERR("register data control path err\n");
		goto exit2;
	}

	/* As default, report all information */
	atomic_set(&m_flag, 0);
//	atomic_set(&o_flag, 0);
	atomic_set(&open_flag, 0);

#if SENSOR_AUTO_TEST
	thread = kthread_run(auto_test_read, NULL, "st480_read_test");
#endif
	st480_init_flag = 1;  

	printk("st480 compass probed successfully.");
	return 0;

	//misc_deregister(&st350d_device);
exit_misc_device_register_failed:
exit3:
exit2:
	kfree(st480);
exit1:
	st480_init_flag = -1;
	return err;

}

static int st480_i2c_remove(struct i2c_client *client)
{
	int err;

	MSE_FUN();
	err = st480_delete_attr(&(st480_init_info.platform_diver_addr->driver));
	if (err)
		MSE_ERR("st480_delete_attr fail: %d\n", err);

	i2c_unregister_device(st480->client);
	kfree(i2c_get_clientdata(st480->client));
	 mag_factory_device_deregister(&st480_factory_device);
//	misc_deregister(&st480d_device);

	st480 = NULL;

	MSE_LOG("st480 i2c successfully removed.");
	return 0;
}

static const struct i2c_device_id st480_i2c_id[] = {{ST480_I2C_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	{},
};
#endif

static struct i2c_driver st480_i2c_driver = {
	.driver = {
		.name  = ST480_I2C_NAME,
		#ifdef CONFIG_OF
		.of_match_table = mag_of_match,
		#endif
	},
	.probe      = st480_i2c_probe,
	.remove     = st480_i2c_remove,
	.id_table = st480_i2c_id,
};

static int st480_remove(void)
{ 
	i2c_del_driver(&st480_i2c_driver);
	return 0;
}


static int st480_local_init(void)
{
	MSE_FUN();
	printk("st480_local_init\n");
	st480_power(hw, 1);
 //	atomic_set(&dev_open_count, 0);
	/* mmc328x_force[0] = hw->i2c_num; */

	if (i2c_add_driver(&st480_i2c_driver)) {
		MSE_ERR("add driver error\n");
		return -1;
	}
	if (-1 == st480_init_flag) {
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/

static int __init st480_init(void)
{
#ifdef USE_OLD_SENSOR_DTS_ARCH
	int ret = 0;
	const char *name = "mediatek,st480";
	printk("st480_init\n");
	ret = get_mag_dts_func(name, hw);
	if (ret)
		MSE_ERR("get dts info fail\n");
#endif
	MSE_LOG("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	//printk("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	//printk("%s: addr=%d\n", __func__, hw->i2c_addr[0]);
	mag_driver_add(&st480_init_info);
	return 0;
}

static void __exit st480_exit(void)
{
	MSE_FUN();

	//platform_driver_unregister(&st350_sensor_driver);
}
module_init(st480_init);
module_exit(st480_exit);

MODULE_AUTHOR("MTK");
MODULE_DESCRIPTION("Senodia ST480 linux driver for MTK");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
