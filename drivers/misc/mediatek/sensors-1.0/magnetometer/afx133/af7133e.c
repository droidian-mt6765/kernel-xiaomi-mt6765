/* drivers/i2c/chips/af7133.c - AF7133 compass driver
 *
 * Copyright (C) 2017 VTC Technology Inc.
 * Author: Gary Huang <gary.huang@voltafield.com>
 *         George Tseng <george.tseng@voltafield.com>
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
 * For Android 8.0
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <hwmsensor.h>
#include <sensors_io.h>
#include <linux/types.h>


#include <hwmsen_helper.h>
 
#include "cust_mag.h"
#include "mag.h"
#include "af7133e.h"

static int af7133e_af8133i_local_init(void);
static int af7133e_af8133i_remove(void);
static int af7133e_af8133i_enable(int en);
static int af7133e_af8133i_get_data(int *x,int *y, int *z,int *status);

/*----------------------------------------------------------------------------*/
/*-------------------------------Android_O------------------------------------*/
/*----------------------------------------------------------------------------*/
#define AF7133E_AF8133I_DEBUG 1
#define AF7133E_AF8133I_DEV_NAME     "af7133e_af8133i" //"af7133e_af8133i"
#define DRIVER_VERSION               "2.2.1"
#define DRIVER_RELEASE               "20171113"
#define DETECT 0
/*----------------------------------------------------------------------------*/
#define AF7133E_AF8133I_DEFAULT_DELAY     100
#define AF7133E_AF8133I_DELAY_MIN     		10
#define AF7133E_AF8133I_DELAY_MAX     		100
/*----------------------------------------------------------------------------*/
#if AF7133E_AF8133I_DEBUG
#define MSE_TAG               "[AF7133E_AF8133I]"
#define MSE_ERR(fmt, args...)	printk(MSE_TAG fmt, ##args)
#define MSE_LOG(fmt, args...)	printk(MSE_TAG fmt, ##args)
#define MSE_FUN(f)            printk(MSE_TAG" %s\r\n", __FUNCTION__)
#else
#define MSE_TAG
#define MSE_ERR(fmt, args...)	do {} while (0)
#define MSE_LOG(fmt, args...)	do {} while (0)
#define MSE_FUN(f)            do {} while (0)
#endif
//#define MSE_ERR(fmt, arg...)             pr_err("<<-af7133e ERROR->> [line=%d]"fmt"\n",__LINE__,##arg)
//#define MSE_LOG(fmt, args...)	pr_debug("<<-af7133e FUNC->> Func:%s@Line:%d\n", __func__, __LINE__)
/*----------------------------------------------------------------------------*/
static struct mag_init_info af7133e_af8133i_init_info = {
	.name   = AF7133E_AF8133I_DEV_NAME,
	.init   = af7133e_af8133i_local_init,
	.uninit = af7133e_af8133i_remove,	
};
static DECLARE_WAIT_QUEUE_HEAD(open_wq);
/*----------------------------------------------------------------------------*/
static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);
/*----------------------------------------------------------------------------*/
//static short af7133e_af8133i__delay = AF7133E_AF8133I_DEFAULT_DELAY;
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_init_flag = -1;  //0:ok,,-1:fail
/*----------------------------------------------------------------------------*/
static struct i2c_client *af7133e_af8133i_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
/* Maintain  cust info here */
struct mag_hw mag_cust;
/* For  driver get cust info */
static struct mag_hw *hw = &mag_cust;
/*----------------------------------------------------------------------------*/
#define FIND_SW_OFFSET_LOOP    5
#define FIND_SW_OFFSET_INDEX   2

static int mag_offset[3];

static unsigned char ADC_log = 0x16;
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//static struct i2c_board_info __initdata i2c_af7133e_af8133i={ I2C_BOARD_INFO("af7133e_af8133i", AF7133E_AF8133I_I2C_ADDRESS)};  //7-bit address
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int af7133e_af8133i_i2c_remove(struct i2c_client *client);
#if DETECT
static int af7133e_af8133i_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#endif
/*----------------------------------------------------------------------------*/
typedef enum {
    VTC_TRC_DEBUG  = 0x01,
} AMI_TRC;
/*----------------------------------------------------------------------------*/
struct af7133e_af8133i_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    struct hwmsen_convert   cvt;
    atomic_t layout;   
    atomic_t trace;
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	{},
};
#endif
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id af7133e_af8133i_i2c_id[] = {{AF7133E_AF8133I_DEV_NAME,0},{}};
/*----------------------------------------------------------------------------*/
static struct i2c_driver af7133e_af8133i_i2c_driver = {
	.driver = 
	{
	    //.owner  = THIS_MODULE,
	    .name   = AF7133E_AF8133I_DEV_NAME,
#ifdef CONFIG_OF   
  .of_match_table = mag_of_match,
#endif
  },
	.probe      = af7133e_af8133i_i2c_probe,
	.remove     = af7133e_af8133i_i2c_remove,
#if DETECT
	.detect     = af7133e_af8133i_i2c_detect,
#endif
	.id_table   = af7133e_af8133i_i2c_id,
};
/*----------------------------------------------------------------------------*/
static DEFINE_MUTEX(af7133e_af8133i_mutex);
/*----------------------------------------------------------------------------*/
#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1
/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/
static void af7133e_af8133i_power(struct mag_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;
	
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static int VTC_i2c_Rx(struct i2c_client *client, char *rxData, int length)
{
        uint8_t retry;
        struct i2c_msg msgs[2];

        msgs[0].addr = client->addr;
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = rxData;

        msgs[1].addr = client->addr;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = length;
        msgs[1].buf = rxData;

            for (retry = 0; retry < 3; retry++) 
        {
                if (i2c_transfer(client->adapter, msgs, 2) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry >= 3) 
        {
                printk(KERN_ERR "[fuking]addr:%#x  %s: retry over 3\n", client->addr, __func__);
                return -EIO;
        } 
        else
                return 0;
}
/*----------------------------------------------------------------------------*/
static int VTC_i2c_Tx(struct i2c_client *client, char *txData, int length)
{
        int retry;
        struct i2c_msg msg[] = 
        {
                {
                        .addr = client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = txData,
                },
        };

        for (retry = 0; retry <= 3; retry++) 
        {
                if (i2c_transfer(client->adapter, msg, 1) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry > 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                printk("[fuking] %s::%d addr:%#x\n", __func__, __LINE__, client->addr);
                return -EIO;
        }
        else
                return 0;
}
/*----------------------------------------------------------------------------*/
int af7133e_af8133i_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
{

	if(i2c_flag == I2C_FLAG_READ)
	{
	  return (VTC_i2c_Rx(client, buf, count>>8));
	}
	else if(i2c_flag == I2C_FLAG_WRITE)
	{
	  return (VTC_i2c_Tx(client, buf, count));
  }
  
  return 0;
}

/*----------------------------------------------------------------------------*/
static int af7133_get_mag_offset(void)
{
  char databuf[10];
  int i, j;
  int mag_pos[3][FIND_SW_OFFSET_LOOP];
  int mag_neg[3][FIND_SW_OFFSET_LOOP];
  
  if(NULL == af7133e_af8133i_i2c_client)
	{
		return -2;
	}
  
  // initial offset
  for(i=0;i<3;i++)
  {
  	mag_offset[i] = 0;
  }
	
	// get reset data
	databuf[0] = 0x14;
	databuf[1] = 0x34;
	af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	for(i=0;i<FIND_SW_OFFSET_LOOP;i++)
	{
	  databuf[0] = 0x0A;
	  databuf[1] = 0x01;
	  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);		
		
		mdelay(2);
	
	  databuf[0] = 0x03;
	  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x601, I2C_FLAG_READ);
		
		mag_neg[0][i] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
	  mag_neg[1][i] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
	  mag_neg[2][i] = ((int) databuf[5]) << 8 | ((int) databuf[4]);

    for(j=0;j<3;j++) mag_neg[j][i] = (mag_neg[j][i] > 32767) ? (mag_neg[j][i] - 65536) : mag_neg[j][i];
  }
  
	// get set data
	databuf[0] = 0x14;
	databuf[1] = 0x38;
	af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	for(i=0;i<FIND_SW_OFFSET_LOOP;i++)
	{
	  databuf[0] = 0x0A;
	  databuf[1] = 0x01;
	  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);		
		
		mdelay(2);
		
	  databuf[0] = 0x03;
	  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x601, I2C_FLAG_READ);
	  		
		mag_pos[0][i] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
	  mag_pos[1][i] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
	  mag_pos[2][i] = ((int) databuf[5]) << 8 | ((int) databuf[4]);

    for(j=0;j<3;j++) mag_pos[j][i] = (mag_pos[j][i] > 32767) ? (mag_pos[j][i] - 65536) : mag_pos[j][i];
  }

  // sort data
  for(i=0;i<3;i++)
  {
    for(j=0;j<(FIND_SW_OFFSET_LOOP-1);j++)
    {
      int k;
      for(k=0;k<(FIND_SW_OFFSET_LOOP-1);k++)
      {
        if(mag_neg[i][k] < mag_neg[i][k+1])
        {
          int tmp = mag_neg[i][k];
          mag_neg[i][k] = mag_neg[i][k+1];
          mag_neg[i][k+1] = tmp;
        }
        if(mag_pos[i][k] < mag_pos[i][k+1])
        {
          int tmp = mag_pos[i][k];
          mag_pos[i][k] = mag_pos[i][k+1];
          mag_pos[i][k+1] = tmp;
        }
      }
    }
    
    // find sensor offset 
    mag_offset[i] = (mag_pos[i][(FIND_SW_OFFSET_INDEX)] + mag_neg[i][FIND_SW_OFFSET_INDEX]) / 2; 
  }
      
	return 0;
}	
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_Chipset_Init(int mode)
{
	u8 databuf[2];

    if(NULL == af7133e_af8133i_i2c_client)
	{
		return -2;
	}
  
  if(mode != AF7133E_AF8133I_MODE_RESUME)
  {
	  databuf[0] = 0x11;
	  databuf[1] = 0x81; 
    af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);   
  
    mdelay(15);
  
	  databuf[0] = 0x10;
	  databuf[1] = 0x55; 
    af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);   
  
	  databuf[0] = 0x00;
	  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x101, I2C_FLAG_READ);
	  if(databuf[0] != AF7133E_AF8133I_PCODE_VALUE)
	  {
		  MSE_ERR("af7133e_af8133i PCODE is incorrect: %d\n", databuf[0]);
		  return -3;
	  } 
	  else
   	{
		  printk("%s chip id:%#x\n",__func__,databuf[0]);
	  }
  }
    
	databuf[0] = 0x10;
	databuf[1] = 0x55; 
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
	databuf[0] = 0x14;
	databuf[1] = 0x38;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  

  databuf[0] = 0x33;
  databuf[1] = ADC_log;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  

  databuf[0] = 0x0B;
  databuf[1] = 0x3C;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  databuf[0] = 0x13;
  databuf[1] = 0x00;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  databuf[0] = 0x0A;
  databuf[1] = 0x01;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE); 

  mdelay(3);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_SetADC(void)
{
	unsigned char databuf[2];
	int err;
	
	if(NULL == af7133e_af8133i_i2c_client)
	{
		return -2;
	}
	
	databuf[0] = 0x10;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x101, I2C_FLAG_READ);
  
	databuf[0] = 0x10;
	databuf[1] = 0x55;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
	databuf[0] = 0x14;
	databuf[1] = 0x38;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	databuf[0] = 0x1F;  
	err = af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x101, I2C_FLAG_READ);  

	if(err < 0)
		return err;
	
	if(databuf[0] & 0x04)
		ADC_log = 0x14;
	else if(databuf[0] & 0x02)
		ADC_log = 0x17;
	else if(databuf[0] & 0x01)
		ADC_log = 0x15;
	else
		ADC_log = 0x16;	

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_ReadSensorData(int *buf)
{
  unsigned char databuf[10];
  int output[3];
  int i;
  
  if(NULL == af7133e_af8133i_i2c_client)
  {
  	*buf = 0;
  	return -2;
  }    
  
  databuf[0] = AF7133E_AF8133I_REG_STATUS;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x101, I2C_FLAG_READ);
   
  // We can read all measured data in once
  databuf[0] = AF7133E_AF8133I_REG_DATA;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
  output[0] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
  output[1] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
  output[2] = ((int) databuf[5]) << 8 | ((int) databuf[4]);
  
  for(i=0;i<3;i++) output[i] = (output[i] > 32767) ? (output[i] - 65536) : output[i];
  
  for(i=0;i<3;i++) buf[i] = output[i] - mag_offset[i];
  
  //confirm register setting
#if 0
  databuf[0] = 0x0B;
  databuf[1] = 0x3C;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  databuf[0] = 0x13;
  databuf[1] = 0x00;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  databuf[0] = 0x14;
  databuf[1] = 0x38;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  databuf[0] = 0x33;
  databuf[1] = ADC_log;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
#endif
  //Next data
  databuf[0] = 0x0A;
  databuf[1] = 0x01;
  af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[10];
	sprintf(strbuf, "af7133ed");
	return sprintf(buf, "%s", strbuf);		
}
/*----------------------------------------------------------------------------*/ 
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char databuf[2];
	
	databuf[0] = AF7133E_AF8133I_REG_PCODE;
	af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x101, I2C_FLAG_READ); 	
  	
	if(AF7133E_AF8133I_PCODE_VALUE != databuf[0])
	{
		printk("af7133e_af8133i PCODE is incorrect: %d\n", databuf[0]);
	} 

	return sprintf(buf, "%s\n", databuf);       
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int databuf[3]={0};
	af7133e_af8133i_ReadSensorData(databuf);
	return sprintf(buf, "%d %d %d\n", databuf[0],databuf[1],databuf[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	int databuf[3] = {0};
	char strbuf[32];

	af7133e_af8133i_ReadSensorData(databuf);

	tmp[0] = databuf[0] / CONVERT_M_DIV;
	tmp[1] = databuf[1] / CONVERT_M_DIV;
	tmp[2] = databuf[2] / CONVERT_M_DIV;
	
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);	
	
	return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/ 
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct af7133e_af8133i_i2c_data *data;

	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		             data->hw->direction,
		             atomic_read(&data->layout),
		             data->cvt.sign[0],
		             data->cvt.sign[1],
		             data->cvt.sign[2],
		             data->cvt.map[0],
		             data->cvt.map[1],
		             data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
  struct af7133e_af8133i_i2c_data *data;
	int layout;

	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

    layout = 0;
	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct af7133e_af8133i_i2c_data *data;
	ssize_t len;

	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	len = 0;
	if(data->hw)
	{
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n", 
			              data->hw->i2c_num, 
			              data->hw->direction, 
			              data->hw->power_id, 
			              data->hw->power_vol);
	}
	else
	{
		len += snprintf(buf + len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf + len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_status_value(struct device_driver *ddri, const char *buf, size_t count)
{
  struct af7133e_af8133i_i2c_data *data;
  int value;

	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

  value = simple_strtol(buf, NULL, 10);
    
  data->hw->direction = value;
  
	if(hwmsen_get_convert(value, &data->cvt)<0)
	{
		MSE_ERR("invalid direction: %d\n", value);
	}

	atomic_set(&data->layout, value);
	return count;    

}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
  struct af7133e_af8133i_i2c_data *data;
	ssize_t res;

	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}	

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&data->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
  struct af7133e_af8133i_i2c_data *data;
  int trace = 0;

	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&data->trace, trace);
	}
	else 
	{
		MSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;    
}

/*----------------------------shipment test------------------------------------------------*/
/*!
 @return If @a testdata is in the range of between @a lolimit and @a hilimit,
 the return value is 1, otherwise -1.
 @param[in] testno   A pointer to a text string.
 @param[in] testname A pointer to a text string.
 @param[in] testdata A data to be tested.
 @param[in] lolimit  The maximum allowable value of @a testdata.
 @param[in] hilimit  The minimum allowable value of @a testdata.
 @param[in,out] pf_total
 */
 /*----------------------------------------------------------------------------*/
int AF7133E_AF8133I_TEST_DATA(const char testno[], const char testname[], const int testdata,
	const int lolimit, const int hilimit, int *pf_total)
{
	int pf;			/* Pass;1, Fail;-1 */

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		MSE_LOG("--------------------------------------------------------------------\n");
		MSE_LOG(" Test No. Test Name	Fail	Test Data	[	 Low	High]\n");
		MSE_LOG("--------------------------------------------------------------------\n");
		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		MSE_LOG("--------------------------------------------------------------------\n");
		if (*pf_total == 1)
			MSE_LOG("Factory shipment test was passed.\n\n");
		else
			MSE_LOG("Factory shipment test was failed.\n\n");
		
		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit))
			pf = 1;
		else
			pf = -1;

	/* display result */
	  MSE_LOG(" %7s  %-10s	 %c	%9d	[%9d	%9d]\n",
		testno, testname, ((pf == 1) ? ('.') : ('F')), testdata,
		lolimit, hilimit);
	}

	/* Pass/Fail check */
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1))
			*pf_total = 1;		/* Pass */
		else
			*pf_total = -1;		/* Fail */
	}
	return pf;
}
/*----------------------------------------------------------------------------*/
/*!
 Execute "Onboard Function Test" (NOT includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
 /*----------------------------------------------------------------------------*/
int FST_AF7133E_AF8133I(void)
{
	int  pf_total;  /* p/f flag for this subtest */
	u8   databuf[6];
	int  value[3];
	int  pos[3];
  int  neg[3];
  int  offset[3];
  int  i;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	pf_total = 1;

	/* *********************************************** */
	/* Step1 */
	/* *********************************************** */

	/* Reset device. */
	af7133e_af8133i_Chipset_Init(0);

	/* Read values from WIA. */
	databuf[0] = 0x00;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x101, I2C_FLAG_READ) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

	/* TEST */
	AF7133E_AF8133I_TEST_DATA("T1", "AF7133E_AF8133I Product Code", (int)databuf[0], AF7133E_AF8133I_PCODE_VALUE, AF7133E_AF8133I_PCODE_VALUE, &pf_total);

	/* Find offset by SW set_reset */
	// neg data (reset)
  databuf[0] = 0x14;
	databuf[1] = 0x34;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

  databuf[0] = 0x0A;
	databuf[1] = 0x01;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}
	mdelay(2);

	databuf[0] = 0x03;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

  neg[0] = databuf[0] | (databuf[1] << 8);
  neg[1] = databuf[2] | (databuf[3] << 8);
  neg[2] = databuf[4] | (databuf[5] << 8);
  for(i=0;i<3;i++) neg[i] = (neg[i] > 32767) ? (neg[i] - 65536) : neg[i];

  // pos data (reset)
  databuf[0] = 0x14;
	databuf[1] = 0x38;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

  databuf[0] = 0x0A;
	databuf[1] = 0x01;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}
	mdelay(2);

	databuf[0] = 0x03;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

	pos[0] = databuf[0] | (databuf[1] << 8);
  pos[1] = databuf[2] | (databuf[3] << 8);
  pos[2] = databuf[4] | (databuf[5] << 8);
  for(i=0;i<3;i++) pos[i] = (pos[i] > 32767) ? (pos[i] - 65536) : pos[i];

  // offset
  for(i=0;i<3;i++) offset[i] = (pos[i] + neg[i]) / 2;

	/* TEST */
	AF7133E_AF8133I_TEST_DATA("T2_1", "AF7133E_AF8133I x-axis offset", offset[0], -15000, 15000, &pf_total);
	AF7133E_AF8133I_TEST_DATA("T2_2", "AF7133E_AF8133I y-axis offset", offset[1], -15000, 15000, &pf_total);
	AF7133E_AF8133I_TEST_DATA("T2_3", "AF7133E_AF8133I z-axis offset", offset[2], -15000, 15000, &pf_total);

	/* *********************************************** */
	/* Step2 */
	/* *********************************************** */

  /* Set to Self-test mode */
  databuf[0] = 0x14;
	databuf[1] = 0x3C;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

	// Enable BIST function
  databuf[0] = 0x0C;
	databuf[1] = 0x63;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

  // positive measurement
  databuf[0] = 0x0A;
	databuf[1] = 0x0A;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

  mdelay(3);

	databuf[0] = 0x03;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

	pos[0] = databuf[0] | (databuf[1] << 8);
  pos[1] = databuf[2] | (databuf[3] << 8);
  pos[2] = databuf[4] | (databuf[5] << 8);
  for(i=0;i<3;i++) pos[i] = (pos[i] > 32767) ? (pos[i] - 65536) : pos[i];

  databuf[0] = 0x0A;
	databuf[1] = 0x00;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

	// negative measurement
  databuf[0] = 0x0A;
	databuf[1] = 0x0E;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

  mdelay(3);

	databuf[0] = 0x03;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

  neg[0] = databuf[0] | (databuf[1] << 8);
  neg[1] = databuf[2] | (databuf[3] << 8);
  neg[1] = databuf[4] | (databuf[5] << 8);
  for(i=0;i<3;i++) neg[i] = (neg[i] > 32767) ? (neg[i] - 65536) : neg[i];

  //
  value[0] = pos[0] - neg[0];
  value[1] = -(pos[1] - neg[1]);
  //value[2] = pos[2] - neg[2];

	/* TEST */
	AF7133E_AF8133I_TEST_DATA("T3_1", "AF7133E_AF8133I BIST test 1", value[0], 3000, 13000, &pf_total);
	AF7133E_AF8133I_TEST_DATA("T3_2", "AF7133E_AF8133I BIST test 2", value[1], 3000, 13000, &pf_total);
	//AF7133E_AF8133I_TEST_DATA("T3_2", "AF7133E_AF8133I BIST test 3", value[2], 3000, 13000, &pf_total);

	/* Set to normal mode */
  databuf[0] = 0x0A;
	databuf[1] = 0x00;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

  databuf[0] = 0x0C;
	databuf[1] = 0x28;
	if (af7133e_af8133i_i2c_master_operate(af7133e_af8133i_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("af7133e_af8133i I2C Error.\n");
		return 0;
	}

	return pf_total;
}
/*----------------------------------------------------------------------------*/
/*!
 Execute "Onboard Function Test" (includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
 /*----------------------------------------------------------------------------*/
int AF7133E_AF8133I_FctShipmntTestProcess_Body(void)
{
	int pf_total = 1;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	AF7133E_AF8133I_TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

	/* *********************************************** */
	/* Step 1 to 2 */
	/* *********************************************** */
	pf_total = FST_AF7133E_AF8133I();

	/* *********************************************** */
	/* Judge Test Result */
	/* *********************************************** */
	AF7133E_AF8133I_TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

	return pf_total;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_shipment_test(struct device_driver *ddri, const char *buf, size_t count)
{
	/* struct i2c_client *client = this_client; */
	/* struct af8133_i2c_data *data = i2c_get_clientdata(client); */
	/* int layout = 0; */

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;

	res = AF7133E_AF8133I_FctShipmntTestProcess_Body();
	if (1 == res) {
		MSE_LOG("shipment_test pass\n");
		strncpy(result, "y", sizeof(result));
	} else if (-1 == res) {
		MSE_LOG("shipment_test fail\n");
		strncpy(result, "n", sizeof(result));
	} else {
		MSE_LOG("shipment_test NaN\n");
		strncpy(result, "NaN", sizeof(result));
	}

	return sprintf(buf, "%s\n", result);
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
//static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(calidata,    S_IRUGO, show_calidata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(status,      S_IRUGO | S_IWUSR, show_status_value, store_status_value);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
static DRIVER_ATTR(shipmenttest,S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *af7133e_af8133i_attr_list[] = {
  &driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	//&driver_attr_posturedata,
	&driver_attr_calidata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
	&driver_attr_shipmenttest,
};
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(af7133e_af8133i_attr_list)/sizeof(af7133e_af8133i_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, af7133e_af8133i_attr_list[idx])))
		{            
			MSE_ERR("driver_create_file (%s) = %d\n", af7133e_af8133i_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(af7133e_af8133i_attr_list)/sizeof(af7133e_af8133i_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, af7133e_af8133i_attr_list[idx]);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;
  struct af7133e_af8133i_i2c_data *data;
	
	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
  
	value = (int)samplingPeriodNs / 1000 / 1000;
	
	MSE_LOG("af7133e_af8133i mag set delay = (%d) ok.\n", value);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_flush(void)
{
	return mag_flush_report();
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = af7133e_af8133i_enable(enabledisable == true ? 1 : 0);
	if (err) {
		MSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = af7133e_af8133i_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MSE_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_factory_get_data(int32_t data[3], int *status)
{
	/* get raw data */
	return  af7133e_af8133i_get_data(&data[0], &data[1], &data[2], status);
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_factory_get_raw_data(int32_t data[3])
{
	MSE_LOG("do not support af7133e_af8133i_factory_get_raw_data!\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_factory_enable_calibration(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_factory_clear_cali(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_factory_set_cali(int32_t data[3])
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_factory_get_cali(int32_t data[3])
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_factory_do_self_test(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct mag_factory_fops af7133e_af8133i_factory_fops = {
	.enable_sensor      = af7133e_af8133i_factory_enable_sensor,
	.get_data           = af7133e_af8133i_factory_get_data,
	.get_raw_data       = af7133e_af8133i_factory_get_raw_data,
	.enable_calibration = af7133e_af8133i_factory_enable_calibration,
	.clear_cali         = af7133e_af8133i_factory_clear_cali,
	.set_cali           = af7133e_af8133i_factory_set_cali,
	.get_cali           = af7133e_af8133i_factory_get_cali,
	.do_self_test       = af7133e_af8133i_factory_do_self_test,
};
/*----------------------------------------------------------------------------*/
static struct mag_factory_public af7133e_af8133i_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &af7133e_af8133i_factory_fops,
};
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_open_report_data(int en)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_set_delay(u64 delay)
{
	int value = 0;
  struct af7133e_af8133i_i2c_data *data;
	
	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	value = (int)delay / 1000 / 1000;

	return 0;
}
/*----------------------------------------------------------------------------*/
#if DETECT
static int af7133e_af8133i_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, AF7133E_AF8133I_DEV_NAME);
    return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_enable(int en)
{
	int value = 0;
  struct af7133e_af8133i_i2c_data *data;
  
	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	value = en;
	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}	
	}
	wake_up(&open_wq);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_get_data(int *x,int *y, int *z,int *status)
{
  struct af7133e_af8133i_i2c_data *data;
	int databuf[3]={0};
	
	if(NULL == af7133e_af8133i_i2c_client)
	{
		MSE_ERR("af7133e_af8133i_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af7133e_af8133i_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
	
	af7133e_af8133i_ReadSensorData(databuf);

	*x = databuf[0]*CONVERT_M; //\A1\A1\A1\A1//james fix
	*y = databuf[1]*CONVERT_M; //\A1\A1\A1\A1//james fix
	*z = databuf[2]*CONVERT_M; //\A1\A1\A1\A1//james fix
	*status = 1;

#if DEBUG
	if (atomic_read(&data->trace) & VTC_TRC_DEBUG) {
			MSE_LOG("%s get data: %d, %d, %d. divide %d, status %d!", __func__,
			*x, *y, *z, CONVERT_M, *status);
	}
#endif

	return 0;
}
/*----------------------------------------------------------------------------*/ 
static int af7133e_af8133i_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client=NULL;
	struct af7133e_af8133i_i2c_data *data=NULL;
	int err = 0;
	
	struct mag_control_path ctl_path ={0};
	struct mag_data_path dat_path = {0};
	  
#ifndef USE_OLD_SENSOR_DTS_ARCH
	err = get_mag_dts_func(client->dev.of_node, hw);
	if (err < 0) 
	{
		MSE_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit;
	}
#else
	//do nothing
#endif  	
	printk("af7133e_af8133i_i2c_probe\n");
	if (!(data = kmalloc(sizeof(struct af7133e_af8133i_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct af7133e_af8133i_i2c_data));
	printk("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	printk("%s: addr=%d\n", __func__, hw->i2c_addr[0]);	
	data->hw = hw;

	err = hwmsen_get_convert(data->hw->direction, &data->cvt);
	if(err)
	{
		MSE_ERR("invalid direction: %d\n", data->hw->direction);
		goto exit;
	}

	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);	
	init_waitqueue_head(&open_wq);  
	client->addr = hw->i2c_addr[0];
	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	af7133e_af8133i_i2c_client = new_client;
  

	if((err = af7133e_af8133i_SetADC()))
	{
  	MSE_ERR("get af7133e ADC setting error\n");
		goto exit_init_failed;
	}

	if((err = af7133e_af8133i_Chipset_Init(AF7133E_AF8133I_MODE_IDLE)))
	{
		MSE_ERR("af7133e register initial fail\n");
		goto exit_init_failed;
	}

	if((err = af7133_get_mag_offset()))
	{
	 	MSE_ERR("get af7133e offset error\n");
	 	goto exit_init_failed;
	}  

	/* Register sysfs attribute */
	if((err = af7133e_af8133i_create_attr(&af7133e_af8133i_init_info.platform_diver_addr->driver)))
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}
	
	err = mag_factory_device_register(&af7133e_af8133i_factory_device);
	if (err)
	{
		MSE_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}
	
	ctl_path.is_use_common_factory = false;
	ctl_path.open_report_data   = af7133e_af8133i_open_report_data;
	ctl_path.enable 			      = af7133e_af8133i_enable;
	ctl_path.set_delay 		      = af7133e_af8133i_set_delay;
	ctl_path.is_report_input_direct = false;
	ctl_path.is_support_batch   = data->hw->is_batch_supported;
	ctl_path.batch = af7133e_af8133i_batch;
	ctl_path.flush = af7133e_af8133i_flush;
	//strlcpy(ctl_path.libinfo.libname, "libvtclib", sizeof(ctl_path.libinfo.libname));
	strcpy(ctl_path.libinfo.libname ,"vtclib");  //james fix
	ctl_path.libinfo.layout = hw->direction;    //james add
	ctl_path.libinfo.deviceid = AF7133E_AF8133I_PCODE_VALUE;//james add	
	err = mag_register_control_path(&ctl_path);

	if(err < 0)
	{
		MSE_ERR("mag_register_control_path failed!\n");
		goto exit_misc_device_register_failed;
	}

	dat_path.div = CONVERT_M_DIV;
	dat_path.get_data = af7133e_af8133i_get_data;

	err = mag_register_data_path(&dat_path);
	if(err < 0)
	{
		MSE_ERR("mag_register_control_path failed!\n");
		goto exit_misc_device_register_failed;
	}
  
	MSE_ERR("%s: OK\n", __func__);

	af7133e_af8133i_init_flag = 0;

	return 0;

	exit_sysfs_create_group_failed:   
	exit_init_failed:
	exit_misc_device_register_failed:
	kfree(data);
	exit:
	MSE_ERR("%s: err = %d\n", __func__, err);

	af7133e_af8133i_init_flag = -1;

	return err;
}
/*----------------------------------------------------------------------------*/ 
static int af7133e_af8133i_i2c_remove(struct i2c_client *client)
{
	int err;	

	if((err = af7133e_af8133i_delete_attr(&af7133e_af8133i_init_info.platform_diver_addr->driver)))
	{
		MSE_ERR("af7133e_af8133i_delete_attr fail: %d\n", err);
	}

	af7133e_af8133i_i2c_client = NULL;
	i2c_unregister_device(client);
	mag_factory_device_deregister(&af7133e_af8133i_factory_device);
	kfree(i2c_get_clientdata(client));	
	return 0;
}
/*----------------------------------------------------------------------------*/ 
static int af7133e_af8133i_local_init(void)
{
	printk("af7133e_af8133i_local_init\n");   
	atomic_set(&dev_open_count, 0);

	if(i2c_add_driver(&af7133e_af8133i_i2c_driver))
	{
        printk("[fuking] %s::%d\n",__func__, __LINE__);
		MSE_ERR("add driver error\n");
		return -1;
	} 
	
	printk("[fuking] %s::%d\n",__func__, __LINE__);
	if(-1 == af7133e_af8133i_init_flag)	
	{	   
		return -1;	
	}

	printk("%s done\n",__func__);
    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af7133e_af8133i_remove(void)
{
	MSE_FUN(); 
	af7133e_af8133i_power(hw, 0);   
	atomic_set(&dev_open_count, 0);  
	i2c_del_driver(&af7133e_af8133i_i2c_driver);
	af7133e_af8133i_init_flag = -1;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init af7133e_af8133i_init(void)
{
#ifdef USE_OLD_SENSOR_DTS_ARCH
	int ret = 0;
	const char *name = "mediatek,af8133x";
	printk("af8133x_init\n");
	ret = get_mag_dts_func(name, hw);
	if (ret)
		MSE_ERR("get dts info fail\n");
#endif
	printk("af8133x_init-2\n");
	printk("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	printk("%s: addr=%d\n", __func__, hw->i2c_addr[0]);
	mag_driver_add(&af7133e_af8133i_init_info);
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit af7133e_af8133i_exit(void)
{
	MSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(af7133e_af8133i_init);
module_exit(af7133e_af8133i_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Gary Huang, George Tseng");
MODULE_DESCRIPTION("AF7133E_AF8133I m-Sensor driver");
MODULE_LICENSE("VTC");
MODULE_VERSION(DRIVER_VERSION);
