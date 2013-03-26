/*
 *  mma7660.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor 
 *
 *  Copyright (C) 2009-2010 Freescale Semiconductor Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>//test
#define MMA7660_DRV_NAME	"mma7660"
#define SENSOR_NAME 			MMA7660_DRV_NAME
#define MMA7660_DIPLAY_VENDOR "Freescale Semiconductor Inc."
#define MMA7660_XOUT			0x00
#define MMA7660_YOUT			0x01
#define MMA7660_ZOUT			0x02
#define MMA7660_TILT			0x03
#define MMA7660_SRST			0x04
#define MMA7660_SPCNT			0x05
#define MMA7660_INTSU			0x06
#define MMA7660_MODE			0x07
#define MMA7660_SR				0x08
#define MMA7660_PDET			0x09
#define MMA7660_PD				0x0A

#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		100
#define INPUT_FUZZ	2
#define INPUT_FLAT	2
#define A13ASENSOR_DURATION_DEFAULT	        20

#define SENSOR_MMA_IOCTL_BASE 		234

#define IOCTL_SENSOR_SET_DELAY_ACCEL   	_IO(SENSOR_MMA_IOCTL_BASE, 100)
#define IOCTL_SENSOR_GET_DELAY_ACCEL   	_IO(SENSOR_MMA_IOCTL_BASE, 101)
#define IOCTL_SENSOR_GET_STATE_ACCEL   	_IO(SENSOR_MMA_IOCTL_BASE, 102)
#define IOCTL_SENSOR_SET_STATE_ACCEL		_IO(SENSOR_MMA_IOCTL_BASE, 103)
#define IOCTL_SENSOR_GET_DATA_ACCEL		_IO(SENSOR_MMA_IOCTL_BASE, 104)

#define IOCTL_MSENSOR_SET_DELAY_MAGNE   	_IO(SENSOR_MMA_IOCTL_BASE, 200)
#define IOCTL_MSENSOR_GET_DATA_MAGNE		_IO(SENSOR_MMA_IOCTL_BASE, 201)
#define IOCTL_MSENSOR_GET_STATE_MAGNE   	_IO(SENSOR_MMA_IOCTL_BASE, 202)
#define IOCTL_MSENSOR_SET_STATE_MAGNE	_IO(SENSOR_MMA_IOCTL_BASE, 203)

#define IOCTL_SENSOR_GET_NAME   _IO(SENSOR_MMA_IOCTL_BASE, 301)
#define IOCTL_SENSOR_GET_VENDOR   _IO(SENSOR_MMA_IOCTL_BASE, 302)

#define IOCTL_SENSOR_GET_CONVERT_PARA   _IO(SENSOR_MMA_IOCTL_BASE, 401)
//struct wake_lock  lock;
/*
 * Defines
 */
#define assert(expr)\
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

#define MK_MMA7660_SR(FILT, AWSR, AMSR)\
	(FILT<<5 | AWSR<<3 | AMSR)

#define MK_MMA7660_MODE(IAH, IPP, SCPS, ASE, AWE, TON, MODE)\
	(IAH<<7 | IPP<<6 | SCPS<<5 | ASE<<4 | AWE<<3 | TON<<2 | MODE)

#define MK_MMA7660_INTSU(SHINTX, SHINTY, SHINTZ, GINT, ASINT, PDINT, PLINT, FBINT)\
	(SHINTX<<7 | SHINTY<<6 | SHINTZ<<5 | GINT<<4 | ASINT<<3 | PDINT<<2 | PLINT<<1 | FBINT)


#define MODE_CHANGE_DELAY_MS 100
static struct i2c_client *mma7660_i2c_client;
int g_sensor_module_flg;

static int gsensor_direct_x				 =1;
static int gsensor_direct_y				 =1;
static int gsensor_direct_z				 =1;
static int gsensor_xy_revert			 =0;

struct mma7660_data {
	struct mutex lock;
	struct i2c_client *client;
	struct work_struct  work;
	struct workqueue_struct *mma7660_wq;
	struct hrtimer timer;
	struct device *device;
	struct input_dev *input_dev;
	int use_count;
	int enabled;
	volatile unsigned int duration;
	int use_irq; 
	int irq;
	unsigned long irqflags;
	int gpio;
	unsigned int map[3];
	int inv[3];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	//volatile int suspend_indator;
#endif
} ;

static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
static __u32 twi_id = 0;
static int sensor_statu=0;
volatile static short a13asensor_duration = A13ASENSOR_DURATION_DEFAULT;
volatile static short a13asensor_state_flag = 1;
static struct mma7660_data *mma7660_data_t=NULL;
static ssize_t enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	
	error = strict_strtoul(buf, 10, &data);
	//printk("%s :%d\n",__func__,data);
	if(error) {
		pr_err("%s strict_strtoul error\n", __FUNCTION__);
		goto exit;
	}
	
	if(data){
		if(mma7660_data_t!=NULL)
		hrtimer_start(&mma7660_data_t->timer, ktime_set(0, a13asensor_duration*1000000), HRTIMER_MODE_REL);
		error = i2c_smbus_write_byte_data(mma7660_i2c_client, MMA7660_MODE,
					MK_MMA7660_MODE(0, 1, 0, 0, 0, 0, 1));
		assert(error==0);
		//sensor_statu=1;
	}else{
		if(mma7660_data_t!=NULL)
		hrtimer_cancel(&mma7660_data_t->timer);
		error = i2c_smbus_write_byte_data(mma7660_i2c_client, MMA7660_MODE,
					MK_MMA7660_MODE(0, 0, 0, 0, 0, 0, 0));
		assert(error==0);
		//sensor_statu=0;
	}
	return count;

exit:
	return error;
	
}

static ssize_t delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if(error) {
		pr_err("%s strict_strtoul error\n", __FUNCTION__);
		goto exit;
	}
	a13asensor_duration=data;
	//printk("a10asensor_duration:%d\n",a13asensor_duration);
	return count;

exit:
	return error;
}

static ssize_t delay_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "a13asensor_duration value is %d\n", a13asensor_duration);
}

static DEVICE_ATTR(enable, 436,
		NULL, enable_store);
		
static DEVICE_ATTR(delay, 436,
		delay_show, delay_store);
static void mma7660_read_xyz(int idx, s8 *pf)
{
	s32 result;

	//assert(mma7660_i2c_client);
	do
	{
		result=i2c_smbus_read_byte_data(mma7660_i2c_client, idx+MMA7660_XOUT);
		//assert(result>=0);
	}while(result&(1<<6)); //read again if alert
	*pf = (result&(1<<5)) ? (result|(~0x0000003f)) : (result&0x0000003f);
}
static void report_abs(struct mma7660_data *data)
{
	int i;
	s8 xyz[3]; 
	s16 x, y, z;

	for(i=0; i<3; i++)
		mma7660_read_xyz(i, &xyz[i]);

	/* convert signed 8bits to signed 16bits */
	
	x = ((((short)xyz[0]) << 8) >> 8);
	y = ((((short)xyz[1]) << 8) >> 8);
	z = ((((short)xyz[2]) << 8) >> 8);
	
	if(gsensor_xy_revert){
	swap(x,y);
	}
	
	//printk("x:%d y:%d z:%d\n",x,y,z);
	input_report_abs(data->input_dev, ABS_X, x*gsensor_direct_x);
	input_report_abs(data->input_dev, ABS_Y, y*gsensor_direct_y);
	input_report_abs(data->input_dev, ABS_Z, z*gsensor_direct_z);

	input_sync(data->input_dev);
}

static void mma7660_work_func(struct work_struct *work)
{
	struct mma7660_data *data = container_of(work, struct mma7660_data, work);

		report_abs(data);
}

static enum hrtimer_restart mma7660_timer_func(struct hrtimer *timer)
{
	struct mma7660_data *data = container_of(timer, struct mma7660_data, timer);
	//printk("a13asensor_duration timer:%d\n",a13asensor_duration);
	queue_work(data->mma7660_wq, &data->work);
	hrtimer_start(&data->timer, ktime_set(0, a13asensor_duration*1000000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static long mma7660_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	

	switch (cmd) {
		case IOCTL_SENSOR_SET_DELAY_ACCEL:
			if(copy_from_user((void *)&a13asensor_duration, (void __user *) arg, sizeof(short))!=0){
				printk("copy from error in %s.\n",__func__);
			}

			break;

		case IOCTL_SENSOR_GET_DELAY_ACCEL:
			if(copy_to_user((void __user *) arg, (const void *)&a13asensor_duration, sizeof(short))!=0){
				printk("copy to error in %s.\n",__func__);
			} 

			break;

		case IOCTL_SENSOR_GET_STATE_ACCEL:
			if(copy_to_user((void __user *) arg, (const void *)&a13asensor_state_flag, sizeof(short))!=0){
				printk("copy to error in %s.\n",__func__);
			}

			break;

		case IOCTL_SENSOR_SET_STATE_ACCEL:
			if(copy_from_user((void *)&a13asensor_state_flag, (void __user *) arg, sizeof(short))!=0){
				printk("copy from error in %s.\n",__func__);
			}     

			break;
		case IOCTL_SENSOR_GET_NAME:
			if(copy_to_user((void __user *) arg,(const void *)SENSOR_NAME, sizeof(SENSOR_NAME))!=0){
				printk("copy to error in %s.\n",__func__);
			}     			
			break;		

		case IOCTL_SENSOR_GET_VENDOR:
			if(copy_to_user((void __user *) arg,(const void *)MMA7660_DIPLAY_VENDOR, sizeof(MMA7660_DIPLAY_VENDOR))!=0){
				printk("copy to error in %s.\n",__func__);
			}     			
			break;
/*
		case IOCTL_SENSOR_GET_CONVERT_PARA:
			convert_para = DMARD06_CONVERT_PARAMETER;
			if(copy_to_user((void __user *) arg,(const void *)&convert_para,sizeof(float))!=0){
				printk("copy to error in %s.\n",__func__);
			}     			
*/
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int mma7660_open(struct inode *inode, struct file *filp)
{
	int ret;
	ret = nonseekable_open(inode, filp);
	return ret;
}

static int mma7660_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations a13asensor_fops =
{
	.owner	= THIS_MODULE,
	.open       	= mma7660_open,
	.release    	= mma7660_release,
	.unlocked_ioctl = mma7660_ioctl,
};

static struct miscdevice mma7660_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma7660",
	.fops = &a13asensor_fops,
};

/*
 *
 * gsensor_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int gsensor_fetch_sysconfig_para(void)
{
	int ret = -1;
	int device_used = -1;
	__u32 twi_addr = 0;
	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
		
	printk("========%s===================\n", __func__);
	 
	if(SCRIPT_PARSER_OK != (ret = script_parser_fetch("gsensor_para", "gsensor_used", &device_used, 1))){
	                pr_err("%s: script_parser_fetch err.ret = %d. \n", __func__, ret);
	                goto script_parser_fetch_err;
	}
	if(1 == device_used){
		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("gsensor_para", "gsensor_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
			pr_err("%s: line: %d script_parser_fetch err. \n", __func__, __LINE__);
			goto script_parser_fetch_err;
		}
		if(strcmp(SENSOR_NAME, name)){
			pr_err("%s: name %s does not match SENSOR_NAME. \n", __func__, name);
			pr_err(SENSOR_NAME);
			//ret = 1;
			return ret;
		}
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			goto script_parser_fetch_err;
		}
		u_i2c_addr.dirty_addr_buf[0] = twi_addr;
		u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
		printk("%s: after: gsensor_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", \
			__func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);

		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_twi_id", &twi_id, 1)){
			pr_err("%s: script_parser_fetch err. \n", name);
			goto script_parser_fetch_err;
		}
		printk("%s: twi_id is %d. \n", __func__, twi_id);
	
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_direct_x", &gsensor_direct_x, sizeof(gsensor_direct_x)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			gsensor_direct_x=1;
		}
		
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_direct_y", &gsensor_direct_y, sizeof(gsensor_direct_y)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			gsensor_direct_y=1;
		}
		
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_direct_z", &gsensor_direct_z, sizeof(gsensor_direct_z)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			gsensor_direct_z=1;
		}
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_xy_revert", &gsensor_xy_revert, sizeof(gsensor_xy_revert)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			gsensor_xy_revert=0;
		}
		#if 0
		gsensor_direct_x				 =-1;
		gsensor_direct_y				 =1;
		gsensor_direct_z				 =-1;
		gsensor_xy_revert 			 =0;
		#endif
		ret = 0;
		
	}else{
		pr_err("%s: gsensor_unused. \n",  __func__);
		ret = -1;
	}

	return ret;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;

}

/**
 * gsensor_detect - Device detection callback for automatic device creation
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
int gsensor_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	
	if(twi_id == adapter->nr){
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, SENSOR_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}

/*
 * Initialization function
 */
static int mma7660_init_client(struct i2c_client *client)
{
	int result;

	mma7660_i2c_client = client;

	if(0)
	{
		/*
		 * Probe the device. We try to set the device to Test Mode and then to
		 * write & verify XYZ registers
		 */
		result = i2c_smbus_write_byte_data(client, MMA7660_MODE,MK_MMA7660_MODE(0, 0, 0, 0, 0, 1, 0));
		assert(result==0);
		mdelay(MODE_CHANGE_DELAY_MS);

		result = i2c_smbus_write_byte_data(client, MMA7660_XOUT, 0x2a);
		assert(result==0);
		
		result = i2c_smbus_write_byte_data(client, MMA7660_YOUT, 0x15);
		assert(result==0);

		result = i2c_smbus_write_byte_data(client, MMA7660_ZOUT, 0x3f);
		assert(result==0);

		result = i2c_smbus_read_byte_data(client, MMA7660_XOUT);

		result= i2c_smbus_read_byte_data(client, MMA7660_YOUT);

		result= i2c_smbus_read_byte_data(client, MMA7660_ZOUT);
		assert(result=0x3f);
	}
	// Enable Orientation Detection Logic
	result = i2c_smbus_write_byte_data(client, 
		MMA7660_MODE, MK_MMA7660_MODE(0, 0, 0, 0, 0, 0, 0)); //enter standby
	assert(result==0);
	printk("result:%d\n",result);
	if(result!=0)
		return result;
		
	result = i2c_smbus_write_byte_data(client, 
		MMA7660_SR, MK_MMA7660_SR(2, 2, 1)); 
	assert(result==0);
	if(result!=0)  
	return result;   

	result = i2c_smbus_write_byte_data(client, 
		MMA7660_INTSU, MK_MMA7660_INTSU(0, 0, 0, 0, 1, 0, 1, 1)); 
	assert(result==0);
	if(result!=0)  
	return result;   

	result = i2c_smbus_write_byte_data(client, 
		MMA7660_SPCNT, 0xA0); 
	assert(result==0);
if(result!=0)  
	return result;   

	result = i2c_smbus_write_byte_data(client, 
		MMA7660_MODE, MK_MMA7660_MODE(0, 1, 0, 0, 0, 0, 1)); 
	assert(result==0);
if(result!=0)  
	return result;   

	mdelay(MODE_CHANGE_DELAY_MS);

	return result;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mma7660_early_suspend(struct early_suspend *h)
{
	int result;
	struct mma7660_data *data;
	//printk(KERN_INFO "mma7660 early suspend\n");

	data = container_of(h, struct mma7660_data, early_suspend);
	hrtimer_cancel(&data->timer);
	result = i2c_smbus_write_byte_data(mma7660_i2c_client, 
		MMA7660_MODE, MK_MMA7660_MODE(0, 0, 0, 0, 0, 0, 0));
	assert(result==0);
	
}

static void mma7660_late_resume(struct early_suspend *h)
{
	int result;
	struct mma7660_data *data;
	//printk(KERN_INFO "mma7660 late resume\n");
	data = container_of(h, struct mma7660_data, early_suspend);
	//mma7660_data.suspend_indator = 0;
	result = i2c_smbus_write_byte_data(mma7660_i2c_client, 
		MMA7660_MODE, MK_MMA7660_MODE(0, 1, 0, 0, 0, 0, 1));
	assert(result==0);
	
	hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int  mma7660_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int ret = 0;
	struct mma7660_data *data;
 	int result;
	printk(KERN_INFO "mma7660 probe\n");
	//wake_lock_init(&lock, WAKE_LOCK_SUSPEND , "test_for_lock"); 
	// wake_lock(&lock);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	data = kzalloc(sizeof(struct mma7660_data), GFP_KERNEL);
	if(data == NULL)
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	mma7660_data_t=data;
	data->mma7660_wq = create_singlethread_workqueue("mma7660_wq");
	if (!data->mma7660_wq )
	{
		ret = -ENOMEM;
		goto err_create_workqueue_failed;
	}
	INIT_WORK(&data->work, mma7660_work_func);
	//mutex_init(&data->lock);
	a13asensor_duration = A13ASENSOR_DURATION_DEFAULT;
	a13asensor_state_flag = 1;
	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		ret = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}
	
	data->client = client;
	i2c_set_clientdata(client, data);
	
	/* Initialize the MMA7660 chip */
	result = mma7660_init_client(client);
	assert(result==0);
	if(result!=0)  
	goto err_chip_init_failed;

	
	printk("mma7660 i2c ok\n");
	
	set_bit(EV_ABS, data->input_dev->evbit);
	

	input_set_abs_params(data->input_dev, ABS_X, -512, 512, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Y, -512, 512, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Z, -512, 512, INPUT_FUZZ, INPUT_FLAT);
	
	data->input_dev->name = "mma7660";
	
	ret = input_register_device(data->input_dev);
	
	if (ret) {
		goto exit_input_register_device_failed;
	}
	ret = misc_register(&mma7660_device);
	if (ret) {
		goto exit_misc_device_register_failed;
	}
	
	ret = device_create_file(&data->input_dev->dev, &dev_attr_enable);
	if(ret)
		printk("create enable file for mma7660 err!\n");

	ret = device_create_file(&data->input_dev->dev, &dev_attr_delay);
	if(ret)
		printk("create delay file for mma7660 err!\n");
	
	//if (!data->use_irq){
	printk("use hrtimer\n");
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = mma7660_timer_func;
	hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	//}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mma7660_early_suspend;
	data->early_suspend.resume = mma7660_late_resume;
	register_early_suspend(&data->early_suspend);
	//mma7660_data.suspend_indator = 0;
#endif
	data->enabled = 1;
	g_sensor_module_flg = 1;
	printk("mma7660 probe ok \n");
	return 0;
exit_misc_device_register_failed:
exit_input_register_device_failed:
	input_free_device(data->input_dev);
err_chip_init_failed:
exit_input_dev_alloc_failed:
	destroy_workqueue(data->mma7660_wq);	
err_create_workqueue_failed:
	kfree(data);	
err_alloc_data_failed:
err_check_functionality_failed:
	printk("mma7660 probe failed \n");
	return ret;
}

static int __devexit mma7660_remove(struct i2c_client *client)
{
	int result;
	struct mma7660_data *data = i2c_get_clientdata(client);
	
	
	hrtimer_cancel(&data->timer);
	result = i2c_smbus_write_byte_data(client,MMA7660_MODE, MK_MMA7660_MODE(0, 0, 0, 0, 0, 0, 0));
	assert(result==0);
	input_unregister_device(data->input_dev);	
	misc_deregister(&mma7660_device);
	//sysfs_remove_group(&client->dev.kobj, &dmard06_group);
	kfree(data);

	return result;
}

static const struct i2c_device_id mma7660_id[] = {
	{ MMA7660_DRV_NAME, 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mma7660_id);

static struct i2c_driver mma7660_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= MMA7660_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= mma7660_probe,
	.remove	= __devexit_p(mma7660_remove),
	.id_table = mma7660_id,
	.address_list	= u_i2c_addr.normal_i2c,
};

static int __init mma7660_init(void)
{
	
	int ret = -1;
	printk("======%s=========. \n", __func__);
	
	if(gsensor_fetch_sysconfig_para()){
		printk("%s: err.\n", __func__);
		return -1;
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	mma7660_driver.detect = gsensor_detect;

	ret = i2c_add_driver(&mma7660_driver);
	if (ret < 0) {
		printk(KERN_INFO "add mma7660 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add mma7660 i2c driver ret:%d\n",ret);
	
	return ret;
	
}

static void __exit mma7660_exit(void)
{

	printk(KERN_INFO "remove mma7660 i2c driver.\n");
	i2c_del_driver(&mma7660_driver);
	
}

MODULE_AUTHOR("liutianmin");
MODULE_DESCRIPTION("MMA7660 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

module_init(mma7660_init);
module_exit(mma7660_exit);
