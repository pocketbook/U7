/*
 *  afa750.c - Linux kernel modules for 3-Axis Orientation/Motion
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

#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>

/*
 * Defines
 */
#define assert(expr)\
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

#define AFA750_DRV_NAME	"afa750"
#define SENSOR_NAME 			AFA750_DRV_NAME

#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		50


#define AFA_FULLRES_MAX_VAL 32767 
#define AFA_FULLRES_MIN_VAL 32768 

#define DATAX0		0x10	/* R   X-Axis Data 0 */
#define DATAX1		0x11	/* R   X-Axis Data 1 */
#define DATAY0		0x12	/* R   Y-Axis Data 0 */
#define DATAY1		0x13	/* R   Y-Axis Data 1 */
#define DATAZ0		0x14	/* R   Z-Axis Data 0 */
#define DATAZ1		0x15	/* R   Z-Axis Data 1 */

#define WHO_AM_I    0x37
#define WHO_AM_I_VALUE1    60
#define WHO_AM_I_VALUE2    61

#define MODE_CHANGE_DELAY_MS 100

static struct device *hwmon_dev;
static struct i2c_client *afa750_i2c_client;

struct afa750_data_s {
    struct i2c_client       *client;
    struct input_polled_dev *pollDev; 
    struct mutex interval_mutex; 
    
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	volatile int suspend_indator;
#endif
} afa750_data;

static struct input_polled_dev *afa750_idev;

/* Addresses to scan */

static const unsigned short normal_i2c[2] = {0x3d,I2C_CLIENT_END};

static __u32 twi_id = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void afa750_early_suspend(struct early_suspend *h);
static void afa750_late_resume(struct early_suspend *h);
#endif


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

		
	printk("========%s===================\n", __func__);
	 
	if(SCRIPT_PARSER_OK != (ret = script_parser_fetch("gsensor_para", "gsensor_used", &device_used, 1))){
	                pr_err("%s: script_parser_fetch err.ret = %d. \n", __func__, ret);
	                goto script_parser_fetch_err;
	}
	if(1 == device_used){



		if(SCRIPT_PARSER_OK != script_parser_fetch("gsensor_para", "gsensor_twi_id", &twi_id, 1)){
			pr_err("%s: script_parser_fetch err. \n", __func__);
			goto script_parser_fetch_err;
		}
		printk("%s: twi_id is %d. \n", __func__, twi_id);

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
	int ret = -1;
	
	if(twi_id == adapter->nr){
		pr_info("Begin gsensor i2c test\n");
		ret = i2c_smbus_read_byte_data(client,WHO_AM_I);
	    printk("Read ID value is: 0x%x\n",ret);
	    if (((ret &0x00FF) == WHO_AM_I_VALUE1)|| ((ret &0x00FF) == WHO_AM_I_VALUE2)) {
	    
	                pr_info("afa750 Sensortec Device detected!\n" );
    				strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);
                    return 0; 
	    
	    }else{
	         pr_info("afa750 not found!\n" );
	         return -ENODEV;
	    }
	}else{
		return -ENODEV;
	}
}


static ssize_t afa750_enable_store(struct device *dev,
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

	if(data) {
		error = i2c_smbus_write_byte_data(afa750_i2c_client, 0x03,0x04);
		assert(error==0);
	} else {
		error = i2c_smbus_write_byte_data(afa750_i2c_client, 0x03,0x02);
		assert(error==0);
	}

	return count;

exit:
	return error;
}

static ssize_t afa750_delay_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t count)
{
   unsigned long data;
	int error;
	struct i2c_client *client = afa750_i2c_client;
	struct afa750_data_s *afa750 = NULL;

    afa750   = i2c_get_clientdata(client);
    printk("delay store %d\n", __LINE__);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > POLL_INTERVAL_MAX)
		data = POLL_INTERVAL_MAX;
    
    mutex_lock(&afa750->interval_mutex);
    afa750->pollDev->poll_interval = data;
    mutex_unlock(&afa750->interval_mutex);

	return count;
		}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		NULL, afa750_enable_store);
		
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		NULL, afa750_delay_store);

static struct attribute *afa750_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	NULL
};

static struct attribute_group afa750_attribute_group = {
	.attrs = afa750_attributes
};






static void report_abs(void)
{
    short buf[3]={0,0,0};
    //s8 buf[6]={0};
    s16 x = 0, y = 0, z = 0;
  if(i2c_smbus_read_i2c_block_data(afa750_i2c_client, DATAX0, DATAZ1 - DATAX0 + 1, buf) < 6){
     printk("FrancesLog****: smbus read block fialed \n");
   }
	
//	x = (s16) le16_to_cpu(buf[0] | ((buf[1] << 8) & 0xff00) );
//	y = (s16) le16_to_cpu(buf[2] | ((buf[3] << 8) & 0xff00) );
//	z = (s16) le16_to_cpu(buf[4] | ((buf[5] << 8) & 0xff00) );
	x = (s16) le16_to_cpu(buf[0]);
	y = (s16) le16_to_cpu(buf[1]);
	z = (s16) le16_to_cpu(buf[2]);
	input_report_abs(afa750_idev->input, ABS_X, x);
	input_report_abs(afa750_idev->input, ABS_Y, y);
	input_report_abs(afa750_idev->input, ABS_Z, z);
	
	//pr_info("x[0] = %d, y[1] = %d, z[2] = %d. \n", x, y, z);

	input_sync(afa750_idev->input);
}

static void afa750_dev_poll(struct input_polled_dev *dev)
{
    if(0 == afa750_data.suspend_indator){
	report_abs();
   } 
} 

/*
 * I2C init/probing/exit functions
 */

static int __devinit afa750_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct input_dev *idev;
	struct i2c_adapter *adapter;
    struct afa750_data_s* data = &afa750_data;
    
    client->addr = 0x3d;
	printk(KERN_INFO "afa750 probe\n");
	afa750_i2c_client = client;
	adapter = to_i2c_adapter(client->dev.parent);
 	result = i2c_check_functionality(adapter,
 					 I2C_FUNC_SMBUS_BYTE |
 					 I2C_FUNC_SMBUS_BYTE_DATA);
	assert(result);


	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));

	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);
  
	/*input poll device register */
	afa750_idev = input_allocate_polled_device();
	if (!afa750_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}
	afa750_idev->poll = afa750_dev_poll;
	afa750_idev->poll_interval = POLL_INTERVAL;
	afa750_idev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = afa750_idev->input;
	idev->name = AFA750_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);


		input_set_abs_params(idev, ABS_X, -AFA_FULLRES_MIN_VAL, AFA_FULLRES_MAX_VAL, 0, 0);
		input_set_abs_params(idev, ABS_Y, -AFA_FULLRES_MIN_VAL, AFA_FULLRES_MAX_VAL, 0, 0);
		input_set_abs_params(idev, ABS_Z, -AFA_FULLRES_MIN_VAL, AFA_FULLRES_MAX_VAL, 0, 0);
	
	result = input_register_polled_device(afa750_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}
	result = sysfs_create_group(&afa750_idev->input->dev.kobj, &afa750_attribute_group);

	if(result) {
		dev_err(&client->dev, "create sys failed\n");
	}
	
	 data->client  = client;
    data->pollDev = afa750_idev;
	i2c_set_clientdata(client, data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	afa750_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	afa750_data.early_suspend.suspend = afa750_early_suspend;
	afa750_data.early_suspend.resume = afa750_late_resume;
	register_early_suspend(&afa750_data.early_suspend);
	afa750_data.suspend_indator = 0;
#endif

	return result;
}

static int __devexit afa750_remove(struct i2c_client *client)
{
	int result = 0;
    assert(result==0);
	hwmon_device_unregister(hwmon_dev);
	#ifdef CONFIG_HAS_EARLYSUSPEND	
	  unregister_early_suspend(&afa750_data.early_suspend);
	#endif
	input_unregister_polled_device(afa750_idev);
	input_free_polled_device(afa750_idev);
	i2c_set_clientdata(afa750_i2c_client, NULL);	

	return result;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void afa750_early_suspend(struct early_suspend *h)
{
	int result = 0;
	
	printk("afa750 early suspend\n");
	afa750_data.suspend_indator = 1;
    assert(result==0);

	return;
}

static void afa750_late_resume(struct early_suspend *h)
{
	int result = 0;
	printk("afa750 late resume\n");
    afa750_data.suspend_indator = 0;
    assert(result==0);
	return;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id afa750_id[] = {
	{ AFA750_DRV_NAME, 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, afa750_id);

static struct i2c_driver afa750_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= AFA750_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= afa750_probe,
	.remove	= __devexit_p(afa750_remove),
	.id_table = afa750_id,
	.address_list	= normal_i2c,
};

static int __init afa750_init(void)
{
	int ret = -1;
	printk("======%s=========. \n", __func__);
	
	if(gsensor_fetch_sysconfig_para()){
		printk("%s: err.\n", __func__);
		return -1;
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, normal_i2c[0], normal_i2c[1]);

	afa750_driver.detect = gsensor_detect;

	ret = i2c_add_driver(&afa750_driver);
	if (ret < 0) {
		printk(KERN_INFO "add afa750 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add afa750 i2c driver\n");

	return ret;
}

static void __exit afa750_exit(void)
{
	printk(KERN_INFO "remove afa750 i2c driver.\n");
	sysfs_remove_group(&afa750_idev->input->dev.kobj, &afa750_attribute_group);
	i2c_del_driver(&afa750_driver);
}

MODULE_AUTHOR("Chen Gang <gang.chen@freescale.com>");
MODULE_DESCRIPTION("afa750 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

module_init(afa750_init);
module_exit(afa750_exit);

