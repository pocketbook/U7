/*  Date: 2011/4/8 11:00:00
 *  Revision: 2.5
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file dmard06.c
   brief This file contains all function implementations for the dmard06 in linux

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>

#define SENSOR_NAME 			"dmard06"



#if 1
#define DPRINTK printk
#else
#define DPRINTK 
#endif

//change direction of raw out data

//extern int g_sensor_module_flg;
/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
static __u32 twi_id = 0;
//volatile unsigned char dmard06_on_off=0;
//static int dmard06_pin_hd;
//static char dmard06_on_off_str[32];

#define G_0_REVERSE	-1
#define G_1_REVERSE	-1
#define G_2_REVERSE	-1

static int gsensor_direct_x				 =1;
static int gsensor_direct_y				 =1;
static int gsensor_direct_z				 =1;
static int gsensor_xy_revert			 =0;

#define SENSOR_DMARD_IOCTL_BASE 		234

#define IOCTL_SENSOR_SET_DELAY_ACCEL   	_IO(SENSOR_DMARD_IOCTL_BASE, 100)
#define IOCTL_SENSOR_GET_DELAY_ACCEL   	_IO(SENSOR_DMARD_IOCTL_BASE, 101)
#define IOCTL_SENSOR_GET_STATE_ACCEL   	_IO(SENSOR_DMARD_IOCTL_BASE, 102)
#define IOCTL_SENSOR_SET_STATE_ACCEL		_IO(SENSOR_DMARD_IOCTL_BASE, 103)
#define IOCTL_SENSOR_GET_DATA_ACCEL		_IO(SENSOR_DMARD_IOCTL_BASE, 104)

#define IOCTL_MSENSOR_SET_DELAY_MAGNE   	_IO(SENSOR_DMARD_IOCTL_BASE, 200)
#define IOCTL_MSENSOR_GET_DATA_MAGNE		_IO(SENSOR_DMARD_IOCTL_BASE, 201)
#define IOCTL_MSENSOR_GET_STATE_MAGNE   	_IO(SENSOR_DMARD_IOCTL_BASE, 202)
#define IOCTL_MSENSOR_SET_STATE_MAGNE	_IO(SENSOR_DMARD_IOCTL_BASE, 203)

#define IOCTL_SENSOR_GET_NAME   _IO(SENSOR_DMARD_IOCTL_BASE, 301)
#define IOCTL_SENSOR_GET_VENDOR   _IO(SENSOR_DMARD_IOCTL_BASE, 302)

#define IOCTL_SENSOR_GET_CONVERT_PARA   _IO(SENSOR_DMARD_IOCTL_BASE, 401)


#define DMARD06_CONVERT_PARAMETER       (1.5f * (9.80665f) / 256.0f)
#define DMARD06_DISPLAY_NAME         "dmard06"
#define DMARD06_DIPLAY_VENDOR        "domintech"

#define X_OUT 					0x41
#define CONTROL_REGISTER		0x44
#define SW_RESET 				0x53
#define WHO_AM_I 				0x0f
#define WHO_AM_I_VALUE 		0x06

#define DMARD06_I2C_NAME			"dmard06"
#define A10ASENSOR_DEV_COUNT	        1
#define A10ASENSOR_DURATION_DEFAULT	        40

#define MAX_RETRY				20
#define INPUT_FUZZ  0
#define INPUT_FLAT  0

struct dmard06_data {
	//struct mutex lock;
	struct i2c_client *client;
	struct work_struct  work;
	struct workqueue_struct *dmard06_wq;
	struct hrtimer timer;
	struct device *device;
	struct input_dev *input_dev;
	int use_irq; 
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

volatile static short a10asensor_duration = A10ASENSOR_DURATION_DEFAULT;
//volatile static short a10asensor_state_flag = 1;
static struct dmard06_data *dmard06_data_p=NULL;
#if 0
static ssize_t dmard06_map_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dmard06_data *data;
	int i;
	data = i2c_get_clientdata(client);
	for (i = 0; i< 3; i++)
	{
		if(data->inv[i] == 1)
		{
			switch(data->map[i])
			{
				case ABS_X:
					buf[i] = 'x';
					break;
				case ABS_Y:
					buf[i] = 'y';
					break;
				case ABS_Z:
					buf[i] = 'z';
					break;
				default:
					buf[i] = '_';
					break;
			}
		}
		else
		{
			switch(data->map[i])
			{
				case ABS_X:
					buf[i] = 'X';
					break;
				case ABS_Y:
					buf[i] = 'Y';
					break;
				case ABS_Z:
					buf[i] = 'Z';
					break;
				default:
					buf[i] = '-';
					break;
			}
		}
	}
	sprintf(buf+3,"\r\n");
	return 5;
}


static ssize_t dmard06_map_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dmard06_data *data;
	int i;
	data = i2c_get_clientdata(client);

	if(count < 3) return -EINVAL;

	for(i = 0; i< 3; i++)
	{
		switch(buf[i])
		{
			case 'x':
				data->map[i] = ABS_X;
				data->inv[i] = 1;
				break;
			case 'y':
				data->map[i] = ABS_Y;
				data->inv[i] = 1;
				break;
			case 'z':
				data->map[i] = ABS_Z;
				data->inv[i] = 1;
				break;
			case 'X':
				data->map[i] = ABS_X;
				data->inv[i] = -1;
				break;
			case 'Y':
				data->map[i] = ABS_Y;
				data->inv[i] = -1;
				break;
			case 'Z':
				data->map[i] = ABS_Z;
				data->inv[i] = -1;
				break;
			default:
				return -EINVAL;
		}
	}

	return count;
}
#endif
static int dmard06_chip_init(struct dmard06_data *sensor_data)
{

	char cAddress = 0 , cData = 0;

	cAddress = SW_RESET;
	i2c_master_send( sensor_data->client, (char*)&cAddress, 1);
	i2c_master_recv( sensor_data->client, (char*)&cData, 1);
	printk(KERN_INFO "i2c Read SW_RESET = %x \n", cData);

	cAddress = WHO_AM_I;
	i2c_master_send( sensor_data->client, (char*)&cAddress, 1);
	i2c_master_recv( sensor_data->client, (char*)&cData, 1);	
	printk(KERN_INFO "i2c Read WHO_AM_I = %d \n", cData);

	if(( cData&0x00FF) == WHO_AM_I_VALUE) 
	{
		printk("dmard06 gsensor registered I2C driver!\n");
	}else{
		printk("dmard06 gsensor I2C err = %d!\n",cData);
		return -1;
	}

	return 0;
}

static ssize_t dmard06_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	printk("%s\n",__func__);
	error = strict_strtoul(buf, 10, &data);
	if(error) {
		pr_err("%s strict_strtoul error\n", __FUNCTION__);
		goto exit;
	}
	#if 1
	if(data){
		//msleep(10);
		//dmard06_chip_init(dmard06_data_t);
		if(dmard06_data_p!=NULL)
		hrtimer_start(&dmard06_data_p->timer, ktime_set(0, a10asensor_duration*1000000), HRTIMER_MODE_REL);
	}else{
		if(dmard06_data_p!=NULL)
		hrtimer_cancel(&dmard06_data_p->timer);
	}
	#endif
	return count;
	
exit:
	return error;
	
}

static ssize_t dmard06_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	printk("%s\n",__func__);
	error = strict_strtoul(buf, 10, &data);
	if(error) {
		pr_err("%s strict_strtoul error\n", __FUNCTION__);
		goto exit;
	}
	a10asensor_duration=data;
	//printk("a10asensor_duration:%d\n",a10asensor_duration);
	return count;

exit:
	return error;
}

static ssize_t dmard06_delay_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "a10asensor_duration value is %d\n", a10asensor_duration);
}

#if 0
static DEVICE_ATTR(map, S_IWUSR | S_IRUGO, dmard06_map_show, dmard06_map_store);
#endif
static DEVICE_ATTR(enable, 436,
		NULL, dmard06_enable_store);
		
static DEVICE_ATTR(delay, 436,
		dmard06_delay_show, dmard06_delay_store);

static struct attribute* dmard06_attrs[] =
{
	//&dev_attr_map.attr,
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group dmard06_group =
{
	.attrs = dmard06_attrs,
};

static int dmard06_i2c_read_xyz(struct dmard06_data *data, u8 *pX, u8 *pY, u8 *pZ)
{
	int ret;
	u8 axis[3];

	ret = i2c_smbus_read_i2c_block_data(data->client , X_OUT , 3 , axis);
	if(ret < 0){
		printk("dmard06_i2c_read_xyz err->%d \n" , ret);
		return -EAGAIN;
	}

	//printk("dmard06 [%x][%x][%x] \n" , axis[0] , axis[1] , axis[2]);

	*pX = axis[0] >> 1;
	*pY = axis[1] >> 1;
	*pZ = axis[2] >> 1;
	
	

#if 0
       if (1 == dir_to_py_nx_pz)
	{
		*pY = axis[0];
		*pX= -axis[1];
		*pZ = axis[2];
	}
	if(1 == dir_to_ny_nx_pz)
	{
		*pY =-axis[0];
		*pX =-axis[1];
		*pZ = axis[2];
		
	}
	else if (1 == dir_to_px_ny_pz)
	{
		*pX = axis[0];
		*pY= -axis[1];
		*pZ = axis[2];
	}
	else
	{
		*pX = axis[0];
		*pY= axis[1];
		*pZ = axis[2];
	}
	#endif
	//printk("px:%d py:%d pz:%d\n",*pX,*pY,*pZ);
	return 0;
}


static void dmard06_xyz_read_and_filter(struct dmard06_data *data)
{
	u8 x8, y8, z8;
	 short x,y,z;

	if(dmard06_i2c_read_xyz(data, &x8, &y8, &z8)){
		return;
	}

	x = (short)(x8 << 9) >> 6;
	y = (short)(y8 << 9) >> 6;
	z = (short)(z8 << 9) >> 6;

	//x = x * 100 / 138;
	//y = y * 100 / 138;
	//z = z * 100 / 138;
	
	if(gsensor_xy_revert){
	swap(x,y);
	}
	
	input_report_abs(data->input_dev, ABS_X, x*gsensor_direct_x);
	input_report_abs(data->input_dev, ABS_Y, y*gsensor_direct_y);
	input_report_abs(data->input_dev, ABS_Z, z*gsensor_direct_z);
	//printk("dmard06:x=%x,y=%x,z=%x\n",x*gsensor_direct_x,y*gsensor_direct_y,z*gsensor_direct_z);
	input_sync(data->input_dev);
	
}


static void dmard06_work_func(struct work_struct *work)
{
	struct dmard06_data *data = container_of(work, struct dmard06_data, work);

		dmard06_xyz_read_and_filter(data);
}

static enum hrtimer_restart dmard06_timer_func(struct hrtimer *timer)
{
	struct dmard06_data *data = container_of(timer, struct dmard06_data, timer);
	
	queue_work(data->dmard06_wq, &data->work);
	hrtimer_start(&data->timer, ktime_set(0, a10asensor_duration*1000000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}



static long dmard06_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	float convert_para=0.0f;
	printk("=======DMARD06_CONVERT_PARAMETER %s.\n",__func__);
	switch (cmd) {
		case IOCTL_SENSOR_SET_DELAY_ACCEL:
			if(copy_from_user((void *)&a10asensor_duration, (void __user *) arg, sizeof(short))!=0){
				printk("copy from error in %s.\n",__func__);
			}

			break;

		case IOCTL_SENSOR_GET_DELAY_ACCEL:
			if(copy_to_user((void __user *) arg, (const void *)&a10asensor_duration, sizeof(short))!=0){
				printk("copy to error in %s.\n",__func__);
			} 

			break;
#if 0
		case IOCTL_SENSOR_GET_STATE_ACCEL:
			if(copy_to_user((void __user *) arg, (const void *)&a10asensor_state_flag, sizeof(short))!=0){
				printk("copy to error in %s.\n",__func__);
			}

			break;

		case IOCTL_SENSOR_SET_STATE_ACCEL:
			if(copy_from_user((void *)&a10asensor_state_flag, (void __user *) arg, sizeof(short))!=0){
				printk("copy from error in %s.\n",__func__);
			}     

			break;
#endif
		case IOCTL_SENSOR_GET_NAME:
			if(copy_to_user((void __user *) arg,(const void *)DMARD06_DISPLAY_NAME, sizeof(DMARD06_DISPLAY_NAME))!=0){
				printk("copy to error in %s.\n",__func__);
			}     			
			break;		

		case IOCTL_SENSOR_GET_VENDOR:
			if(copy_to_user((void __user *) arg,(const void *)DMARD06_DIPLAY_VENDOR, sizeof(DMARD06_DIPLAY_VENDOR))!=0){
				printk("copy to error in %s.\n",__func__);
			}     			
			break;

		case IOCTL_SENSOR_GET_CONVERT_PARA:
			convert_para = DMARD06_CONVERT_PARAMETER;
			
			if(copy_to_user((void __user *) arg,(const void *)&convert_para,sizeof(float))!=0){
				printk("copy to error in %s.\n",__func__);
			}     			

		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}


static int dmard06_open(struct inode *inode, struct file *filp)
{
	int ret;
	ret = nonseekable_open(inode, filp);
	return ret;
}

static int dmard06_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations a10asensor_fops =
{
	.owner	= THIS_MODULE,
	.open       	= dmard06_open,
	.release    	= dmard06_release,
	.unlocked_ioctl = dmard06_ioctl,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void dmard06_early_suspend(struct early_suspend *handler)
{
	struct dmard06_data *data;
	char dmard06_address;
	char dmard06_data;

	//printk("dmard06_early_suspend 2 \n");

	data = container_of(handler, struct dmard06_data, early_suspend);

	hrtimer_cancel(&data->timer);

	dmard06_address = CONTROL_REGISTER;
	dmard06_data = i2c_smbus_read_byte_data(data->client, dmard06_address);
	dmard06_data &= 0x1F;
	i2c_smbus_write_byte_data(data->client, dmard06_address,dmard06_data);	
}

static void dmard06_early_resume(struct early_suspend *handler)
{
	struct dmard06_data *data;
	char dmard06_address;
	char dmard06_data;

	//printk("dmard06_early_resume 2\n");

	data = container_of(handler, struct dmard06_data, early_suspend);

	dmard06_address = CONTROL_REGISTER;
	dmard06_data = i2c_smbus_read_byte_data(data->client, dmard06_address);
	dmard06_data &= 0x1F;
	dmard06_data |= 0x20;
	i2c_smbus_write_byte_data(data->client, dmard06_address,dmard06_data);	

	hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
}
#endif

static struct miscdevice dmard06_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dmard06",
	.fops = &a10asensor_fops,
};

/**
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
	char name[I2C_NAME_SIZE]={0};
	char dir[25]={0};
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;

	printk("========%s===================\n", __func__);

	if(SCRIPT_PARSER_OK != (ret = script_parser_fetch("gsensor_para", "gsensor_used", &device_used, 1))){
		pr_err("%s: script_parser_fetch err.ret = %d. \n", __func__, ret);
		goto script_parser_fetch_err;
	}
	if(1 == device_used){
		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("gsensor_para", "gsensor_name1", (int *)(&name), &type, sizeof(name)/sizeof(int))){
			pr_err("%s: line: %d script_parser_fetch err. \n", __func__, __LINE__);
			goto script_parser_fetch_err;
		}
		if(strcmp(SENSOR_NAME, name)){
			pr_err("%s: name %s does not match SENSOR_NAME. \n", __func__, name);
			pr_err(SENSOR_NAME);
			//ret = 1;
			return ret;
		}
			//wisky-lxh@20120112,change gsensor direction
		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("gsensor_para", "gsensor_change_dir1", (int *)(&dir), &type, sizeof(dir)/sizeof(int))){
			pr_err("%s: line: %d script_parser_fetch err.,%s\n", __func__, __LINE__,dir);
			
		}
		printk("name:%s,gsensor_change_dir:%s\n",name,dir);
					

		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_twi_addr1", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
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
		
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_direct_x1", &gsensor_direct_x, sizeof(gsensor_direct_x)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			gsensor_direct_x=1;
		}
		printk("gsensor_direct_x:%d\n",gsensor_direct_x);
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_direct_y1", &gsensor_direct_y, sizeof(gsensor_direct_y)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			gsensor_direct_y=1;
		}
		printk("gsensor_direct_y:%d\n",gsensor_direct_y);
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_direct_z1", &gsensor_direct_z, sizeof(gsensor_direct_z)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			gsensor_direct_z=1;
		}
		printk("gsensor_direct_z:%d\n",gsensor_direct_z);
		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_xy_revert1", &gsensor_xy_revert, sizeof(gsensor_xy_revert)/sizeof(__u32))){
			pr_err("%s: line: %d: script_parser_fetch err. \n", name, __LINE__);
			gsensor_xy_revert=0;
		}
		printk("gsensor_xy_revert:%d\n",gsensor_xy_revert);
		#if 0
		gsensor_direct_x				 =1;
		gsensor_direct_y			   =1;
		gsensor_direct_z				 =-1;
		gsensor_xy_revert			   =1;
		#endif
#if 0
		dmard06_pin_hd = gpio_request_ex("gsensor_para",NULL);
		if (dmard06_pin_hd==-1) {
			printk("dmard06 pin request error!\n");
		}
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


static int dmard06_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct dmard06_data *data;

	 
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	data = kzalloc(sizeof(struct dmard06_data), GFP_KERNEL);
	if(data == NULL)
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	dmard06_data_p=data;
	data->dmard06_wq = create_singlethread_workqueue("dmard06_wq");
	if (!data->dmard06_wq )
	{
		ret = -ENOMEM;
		goto err_create_workqueue_failed;
	}
	INIT_WORK(&data->work, dmard06_work_func);
	//mutex_init(&data->lock);

	a10asensor_duration = A10ASENSOR_DURATION_DEFAULT;
	//a10asensor_state_flag = 1;

	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		ret = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}

	data->client = client;
	i2c_set_clientdata(client, data);	
	ret = dmard06_chip_init(data);
	if (ret < 0) {
		goto err_chip_init_failed;
	}
	//g_sensor_module_flg = 1;
	printk("dmard06 i2c ok\n");
	set_bit(EV_ABS, data->input_dev->evbit);
	//data->map[0] = G_0;
	//data->map[1] = G_1;
	//data->map[2] = G_2;
	//data->inv[0] = G_0_REVERSE;
	//data->inv[1] = G_1_REVERSE;
	//data->inv[2] = G_2_REVERSE;

	input_set_abs_params(data->input_dev, ABS_X, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Y, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Z, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);

	data->input_dev->name = "dmard06";

	ret = input_register_device(data->input_dev);
	if (ret) {
		goto exit_input_register_device_failed;
	}

	ret = misc_register(&dmard06_device);
	if (ret) {
		goto exit_misc_device_register_failed;
	}
#if 0
	ret = sysfs_create_group(&client->dev.kobj, &dmard06_group);
	if(ret){
	printk("create file for dmard06 err!\n");
	}
#endif
	ret = device_create_file(&data->input_dev->dev, &dev_attr_enable);
	if(ret)
		printk("create enable file for dmard06 err!\n");

	ret = device_create_file(&data->input_dev->dev, &dev_attr_delay);
	if(ret)
		printk("create delay file for dmard06 err!\n");

	if (!data->use_irq){
		printk("use hrtimer\n");
		hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		data->timer.function = dmard06_timer_func;
		hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.suspend = dmard06_early_suspend;
	data->early_suspend.resume = dmard06_early_resume;
	register_early_suspend(&data->early_suspend);
#endif
	//data->enabled = 1;
	//strcpy(dmard06_on_off_str,"gsensor_int2");
	//gpio_set_one_pin_io_status(dmard06_pin_hd,0,dmard06_on_off_str);
	printk("dmard06 probe ok \n");

	return 0;
exit_misc_device_register_failed:
exit_input_register_device_failed:
	input_free_device(data->input_dev);
err_chip_init_failed:
exit_input_dev_alloc_failed:
	destroy_workqueue(data->dmard06_wq);	
err_create_workqueue_failed:
	kfree(data);	
err_alloc_data_failed:
err_check_functionality_failed:
	printk("dmard06 probe failed \n");
	return ret;

}

static int dmard06_remove(struct i2c_client *client)
{
	struct dmard06_data *data = i2c_get_clientdata(client);

	hrtimer_cancel(&data->timer);
	input_unregister_device(data->input_dev);	
	//gpio_release(dmard06_pin_hd, 2);
	misc_deregister(&dmard06_device);
	//sysfs_remove_group(&client->dev.kobj, &dmard06_group);
	device_remove_file(&data->input_dev->dev, &dev_attr_enable);
	device_remove_file(&data->input_dev->dev, &dev_attr_delay);
	
	kfree(data);
	return 0;
}


static const struct i2c_device_id dmard06_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, dmard06_id);

static struct i2c_driver dmard06_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
	},
	.id_table	= dmard06_id,
	.probe		= dmard06_probe,
	.remove		= dmard06_remove,
	.address_list	= u_i2c_addr.normal_i2c,
};

static int __init dmard06_init(void)
{
	int ret = -1;
	printk("dmard06: init\n");
	/*
	if(g_sensor_module_flg){
		printk("NOTE:other g-sensor modules already insmod...\n");
		return -1;
	}
*/
	if(gsensor_fetch_sysconfig_para()){
		printk("%s: err.\n", __func__);
		return -1;
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
			__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	dmard06_driver.detect = gsensor_detect;

	ret = i2c_add_driver(&dmard06_driver);

	return ret;
}

static void __exit dmard06_exit(void)
{
	i2c_del_driver(&dmard06_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("dmard06 driver");
MODULE_LICENSE("GPL");

module_init(dmard06_init);
module_exit(dmard06_exit);

