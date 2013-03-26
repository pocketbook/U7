/*
 *  mxc6225.c - Linux kernel modules for Detection Sensor  
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

#define MXC622X_DRV_NAME	"mxc622x"
#define SENSOR_NAME 		MXC622X_DRV_NAME


#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		50
#define INPUT_FUZZ	2
#define INPUT_FLAT	2

#define MXC622X_REG_CTRL		0x04
#define MXC622X_REG_DATA		0x00

/* MXC622X control bit */
#define MXC622X_CTRL_PWRON		0x00	/* power on */
#define MXC622X_CTRL_PWRDN		0x80	/* power donw */


#define MODE_CHANGE_DELAY_MS 100

static struct device *hwmon_dev;
static struct i2c_client *mxc622x_i2c_client;

struct mxc622x_data_s {
    struct i2c_client       *client;
    struct input_polled_dev *pollDev; 
    struct mutex interval_mutex; 
    
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	volatile int suspend_indator;
#endif
} mxc622x_data;


/* Addresses to scan */
static const unsigned short normal_i2c[] = {0x15,I2C_CLIENT_END};
static __u32 twi_id = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxc622x_early_suspend(struct early_suspend *h);
static void mxc622x_late_resume(struct early_suspend *h);
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

//Function as i2c_master_send, and return 1 if operation is successful. 
static int i2c_write_bytes(struct i2c_client *client, uint8_t *data, uint16_t len)
{
	struct i2c_msg msg;
	int ret=-1;
	
	msg.flags = !I2C_M_RD;//Ð´ÏûÏ¢
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = data;		
	
	ret=i2c_transfer(client->adapter, &msg,1);
	return ret;
}
static bool gsensor_i2c_test(struct i2c_client * client)
{
	uint8_t test_data[1] = { 0 };	//only write a data address.
    int ret = -1;
		ret =i2c_write_bytes(client, test_data, 1);	//Test i2c.

	
	return ret==1 ? true : false;
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
	int ret = 0;

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -ENODEV;
    
	if(twi_id == adapter->nr){
            pr_info("%s: addr= %x\n",__func__,client->addr);

            ret = gsensor_i2c_test(client);
        	if(!ret){
        		pr_info("%s:I2C connection might be something wrong or maybe the other gsensor equipment! \n",__func__);
        		return -ENODEV;
        	}else{           	    
            	pr_info("I2C connection sucess!\n");
            	strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);
    		    return 0;	
	             }

	}else{
		return -ENODEV;
	}
}

static ssize_t mxc622x_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = mxc622x_i2c_client;
	struct mxc622x_data_s *mxc622x = NULL;

    mxc622x    = i2c_get_clientdata(client);
    printk("delay store %d\n", __LINE__);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > POLL_INTERVAL_MAX)
		data = POLL_INTERVAL_MAX;
    
    mutex_lock(&mxc622x->interval_mutex);
    mxc622x->pollDev->poll_interval = data;
    mutex_unlock(&mxc622x->interval_mutex);

	return count;
}

static ssize_t mxc622x_enable_store(struct device *dev,
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
		error = i2c_smbus_write_byte_data(mxc622x_i2c_client, MXC622X_REG_CTRL,MXC622X_CTRL_PWRON);
		assert(error==0);
	} else {
		error = i2c_smbus_write_byte_data(mxc622x_i2c_client, MXC622X_REG_CTRL,MXC622X_CTRL_PWRDN);
		assert(error==0);
	}

	return count;

exit:
	return error;
}


static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		NULL, mxc622x_enable_store);

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		NULL,  mxc622x_delay_store);

static struct attribute *mxc622x_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	NULL
};

static struct attribute_group mxc622x_attribute_group = {
	.attrs = mxc622x_attributes
};

static struct input_polled_dev *mxc622x_idev;

static int report_abs(void)
{
	s8 xyz[2] = {0x00}; 
	s16 x, y, z = 0;
	int ret;

	 
	ret = i2c_smbus_write_byte_data(mxc622x_i2c_client, 0x04, 0x02);
	assert(ret==0);
	
	memset(&xyz,0,sizeof(xyz));
	ret=i2c_smbus_read_i2c_block_data(mxc622x_i2c_client, 0x00, 2,xyz);
	if(ret < 2)
	{
	    printk("read data error!\n");
	}
//	pr_info("xyz[0] = 0x%hx, xyz[1] = 0x%hx \n", xyz[0], xyz[1]);

//	/* convert signed 8bits to signed 16bits */
	x = ((xyz[0] ) << 8) >> 8;
	y = ((xyz[1] ) << 8) >> 8;
    
    if((x == 0) || (y == 0)){
    
               return 0;
    
    }
	
	input_report_abs(mxc622x_idev->input, ABS_X, x);
	input_report_abs(mxc622x_idev->input, ABS_Y, y);
	input_report_abs(mxc622x_idev->input, ABS_Z, z);
     
	input_sync(mxc622x_idev->input);
	return 1;
}

static void mxc622x_dev_poll(struct input_polled_dev *dev)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	if(0 == mxc622x_data.suspend_indator){
		report_abs();
	}
#else
	report_abs();
#endif
} 

/*
 * I2C init/probing/exit functions
 */

static int __devinit mxc622x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct i2c_adapter *adapter;
    struct mxc622x_data_s* data = &mxc622x_data;
 
	printk(KERN_INFO "mxc622x probe\n");
	mxc622x_i2c_client = client;
	adapter = to_i2c_adapter(client->dev.parent);
 	result = i2c_check_functionality(adapter,
 					 I2C_FUNC_SMBUS_BYTE |
 					 I2C_FUNC_SMBUS_BYTE_DATA);
	assert(result);


	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));

	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);
  
	/*input poll device register */
	mxc622x_idev = input_allocate_polled_device();
	if (!mxc622x_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}
	mxc622x_idev->poll = mxc622x_dev_poll;
	mxc622x_idev->poll_interval = POLL_INTERVAL;
	mxc622x_idev->poll_interval_max = POLL_INTERVAL_MAX;
	mxc622x_idev->input->name = MXC622X_DRV_NAME;
	mxc622x_idev->input->id.bustype = BUS_I2C;
	mxc622x_idev->input->evbit[0] = BIT_MASK(EV_ABS);
    mutex_init(&data->interval_mutex);

	input_set_abs_params(mxc622x_idev->input, ABS_X, -128, 128, 0, 0);
	input_set_abs_params(mxc622x_idev->input, ABS_Y, -128, 128, 0, 0);
	input_set_abs_params(mxc622x_idev->input, ABS_Z, -128, 128, 0, 0);
	
	result = input_register_polled_device(mxc622x_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}
	result = sysfs_create_group(&mxc622x_idev->input->dev.kobj, &mxc622x_attribute_group);

	if(result) {
		dev_err(&client->dev, "create sys failed\n");
	}

    data->client  = client;
    data->pollDev = mxc622x_idev;
	i2c_set_clientdata(client, data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	mxc622x_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	mxc622x_data.early_suspend.suspend = mxc622x_early_suspend;
	mxc622x_data.early_suspend.resume = mxc622x_late_resume;
	register_early_suspend(&mxc622x_data.early_suspend);
	mxc622x_data.suspend_indator = 0;
#endif

	return result;
}

static int __devexit mxc622x_remove(struct i2c_client *client)
{
	int result;
    result = i2c_smbus_write_byte_data(mxc622x_i2c_client, MXC622X_REG_CTRL, MXC622X_CTRL_PWRDN);
	assert(result==0);
	hwmon_device_unregister(hwmon_dev);
	#ifdef CONFIG_HAS_EARLYSUSPEND	
	  unregister_early_suspend(&mxc622x_data.early_suspend);
	#endif
	input_unregister_polled_device(mxc622x_idev);
	input_free_polled_device(mxc622x_idev);
	i2c_set_clientdata(mxc622x_i2c_client, NULL);	

	return result;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxc622x_early_suspend(struct early_suspend *h)
{
	int result;
	printk(KERN_INFO "mxc6225 early suspend\n");
	mxc622x_data.suspend_indator = 1;
	result = i2c_smbus_write_byte_data(mxc622x_i2c_client, MXC622X_REG_CTRL, MXC622X_CTRL_PWRDN);
	assert(result==0);
	return;
}

static void mxc622x_late_resume(struct early_suspend *h)
{
	int result;
	printk(KERN_INFO "mxc6225 late resume\n");
	mxc622x_data.suspend_indator = 0;
	result = i2c_smbus_write_byte_data(mxc622x_i2c_client, MXC622X_REG_CTRL, MXC622X_CTRL_PWRON);
	assert(result==0);
	return;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id mxc622x_id[] = {
	{ MXC622X_DRV_NAME, 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxc622x_id);

static struct i2c_driver mxc622x_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= MXC622X_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= mxc622x_probe,
	.remove	= __devexit_p(mxc622x_remove),
	.id_table = mxc622x_id,
	.address_list	= normal_i2c,
};

static int __init mxc622x_init(void)
{
	int ret = -1;
	printk("======%s=========. \n", __func__);
	
	if(gsensor_fetch_sysconfig_para()){
		printk("%s: err.\n", __func__);
		return -1;
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, normal_i2c[0], normal_i2c[1]);

	mxc622x_driver.detect = gsensor_detect;


	ret = i2c_add_driver(&mxc622x_driver);
	if (ret < 0) {
		printk(KERN_INFO "add mxc622x i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add mxc622x i2c driver\n");

	return ret;
}

static void __exit mxc622x_exit(void)
{
	printk(KERN_INFO "remove mxc622x i2c driver.\n");
	sysfs_remove_group(&mxc622x_idev->input->dev.kobj, &mxc622x_attribute_group);
	i2c_del_driver(&mxc622x_driver);
}

MODULE_AUTHOR("yin");
MODULE_DESCRIPTION("mxc622x Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

module_init(mxc622x_init);
module_exit(mxc622x_exit);

