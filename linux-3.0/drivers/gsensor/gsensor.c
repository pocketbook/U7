/*
 *  gsensor.c - Linux kernel modules for  Detection Sensor 
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

#define assert(expr)\
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

#define GSENSOR_NAME	"gsensor"
#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		20
#define SUPPORT_NUMBER      10

#define MODE_CHANGE_DELAY_MS 100

static struct device *hwmon_dev;
static struct i2c_client *gsensor_i2c_client;

struct gsensor_data_s {
    struct i2c_client       *client;
    struct input_polled_dev *pollDev; 
    struct mutex interval_mutex; 
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	volatile int suspend_indator;
#endif
} gsensor_data;

static struct input_polled_dev *gsensor_idev;

/* Addresses to scan */
static const unsigned short normal_i2c[] = {0x11, I2C_CLIENT_END};

static __u32 twi_id = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
static void gsensor_early_suspend(struct early_suspend *h);
static void gsensor_late_resume(struct early_suspend *h);
#endif

struct base_info{
             char name[64];
             unsigned short i2c_address;
             unsigned short chip_id_reg;
};
static struct base_info sensors[SUPPORT_NUMBER] = {{"bma250",0x18,0x00},{"bma222",0x08,0x00},{"bma150",0x38,0x00},\
                                                  {"mma7660",0x4c,0x03},{"dmard06",0x1c,0x0f},{"mma8452c",0x1c,0x0d},\
                                                  {"mma8452d",0x1d,0x0d},{"kxtik",0x0f,0x0f},{"mxc622x",0x15,0x08},{"afa750",0x3d,0x37}};


static int i2c_address_number;
int report_value = 0x0f;
int report_num = 1;

enum{
     BMA150_CHIP_ID = 0x02,
     BMA250_CHIP_ID = 0x03,
     KIONIX_ACCEL_WHO_AM_I_KXTIK = 0x05,
     KIONIX_ACCEL_WHO_AM_I_KXTJ9 = 0x08,
     MMA8452_ID = 0x2A,
     DMARD06_WHO_AM_I_VALUE = 0x06, 
     AFA750_VALUE1 = 0x3d,
     AFA750_VALUE2 = 0x3c,   
     };   


static int gsensor_fetch_sysconfig_para(void)
{
	int ret = -1;
	int i=0,j=0;
	int device_used = -1;
	int class_value = 0;
	char buf[64];	
	char name[256] = {0};
	
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;	
	printk("========%s===================\n", __func__);
	 
	if(SCRIPT_PARSER_OK != (ret = script_parser_fetch("gsensor_para", "gsensor_used", &device_used, 1))){
	                pr_err("%s: script_parser_fetch err.ret = %d. \n", __func__, ret);
	                goto script_parser_fetch_err;
	}
	if(1 == device_used){
		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("gsensor_para", "gsensor_unuse_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
			pr_err("%s: line: %d script_parser_fetch err. \n", __func__, __LINE__);
		}
        name[strlen(name)] = ',';
	    printk("name: %s\n",name);
	    while(name[i++]){
                if(name[i - 1] != ','){
	               buf[j++] = name[i - 1];	               
	                continue;
	             }
            	   for(class_value = 0; class_value < SUPPORT_NUMBER; class_value++ )
                        {
            		    		if (!strncmp(buf, sensors[class_value].name, strlen(sensors[class_value].name))) {
            		                     sensors[class_value].i2c_address = 0x00; 
            		                     printk("address:0x%x \n",sensors[class_value].i2c_address);
            		                     break;              		                             
                     	        }
                		}
                 j = 0;
	             memset(&buf,0,sizeof(buf));
    	}

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
	int ret;
	uint8_t test_data[1] = { 0 };	//only write a data address.
	
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
	int ret;
	
    i2c_address_number = 0;
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -ENODEV;
    
	if(twi_id == adapter->nr){
	        while(i2c_address_number < SUPPORT_NUMBER){
	            
                	       if(sensors[i2c_address_number].i2c_address == 0x00) {
                	            i2c_address_number++;
                	            continue;
                	        }
                	        client->addr = sensors[i2c_address_number].i2c_address;
                            pr_info("%s: addr= %x\n",__func__,client->addr);
                            ret = gsensor_i2c_test(client);
                        	if(!ret){
                        	    i2c_address_number++;
                        		continue;
                        	}else{           	    
                            	pr_info("I2C connection sucess!\n");
    
                	         }  
    
    	        if (strncmp(sensors[i2c_address_number].name,"mma7660",strlen("mma7660"))){
    	           if(strncmp(sensors[i2c_address_number].name,"mxc622x",strlen("mxc622x"))){
    	              ret = i2c_smbus_read_byte_data(client,sensors[i2c_address_number].chip_id_reg);
    	              ret = ret &0x00FF;
    	           }else{
    	                 	  report_value = 0x06;
                	  	      pr_info("mxc622x Device detected!");  
    	                      break;
    	                 }
               }else{
                      report_value = 0x02;
    	              printk( "%s:mma7660 equipment is detected!\n",__func__);
    	              break;
               }
    	         printk("line:%d,i2c_address_number:%d,ret = %d\n",__LINE__,i2c_address_number,ret);
    	         switch(ret)
    	         {
    	            case 0x02:
    	            case 0x03:
                           report_value = 0x01;
                           pr_info("Bosch Sensortec Device detected!");
                           break;
    	            case 0x05:
    	            case 0x08:
                              report_value = 0x05;
            	  	          pr_info("%s:this accelerometer is a KXTIK.\n",__func__);
                              break;
                   	case 0x06: 
    	                      report_value = 0x03;
    	                      printk( "%s: DMARD06 equipment is detected!\n",__func__);
                		      break;
    	           case 0x2A:
    	                    report_value = 0x04;
                		    printk( "%s: mma8452 equipment is detected!\n",__func__);
                		    break;
                		      	                                         
    	             case 0x3c:
    	             case 0x3d:
    	                       report_value = 0x07;
            	               printk( "%s:afa750 equipment is detected!\n",__func__);
    	                       break;      
    	             default:
    	                break;
    	                    
    	        
    	          }
    	          
    	          if(report_value != 0x0f ){
    	             break;
    	          }
    	          i2c_address_number++;
 
        }

          strlcpy(info->type, GSENSOR_NAME, I2C_NAME_SIZE);
          return 0;

	}else{
		return -ENODEV;
	}
}

static ssize_t gsensor_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = gsensor_i2c_client;
	struct gsensor_data_s *gsensor = NULL;

    gsensor    = i2c_get_clientdata(client);
    printk("delay store %d\n", __LINE__);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > POLL_INTERVAL_MAX)
		data = POLL_INTERVAL_MAX;
    
    mutex_lock(&gsensor->interval_mutex);
    gsensor->pollDev->poll_interval = data;
    mutex_unlock(&gsensor->interval_mutex);

	return count;
}

static ssize_t gsensor_enable_store(struct device *dev,
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

   report_num = data ? 1 : 0;

	return count;

exit:
	return error;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		NULL, gsensor_enable_store);

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		NULL,  gsensor_delay_store);

static struct attribute *gsensor_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	NULL
};

static struct attribute_group gsensor_attribute_group = {
	.attrs = gsensor_attributes
};
static void report_abs(void)
{

    input_report_abs(gsensor_idev->input, ABS_MISC, 0x00000000);
    msleep(100);
	input_report_abs(gsensor_idev->input, ABS_MISC,report_value );
	input_sync(gsensor_idev->input);
}
static void gsensor_dev_poll(struct input_polled_dev *dev)
{
  //if(report_num){
            report_abs();
 // }

} 



/*
 * I2C init/probing/exit functions
 */

static int __devinit gsensor_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct i2c_adapter *adapter;
    struct gsensor_data_s* data = &gsensor_data;
	
	printk(KERN_INFO "gsensor probe\n");
	gsensor_i2c_client = client;
	adapter = to_i2c_adapter(client->dev.parent);
 	result = i2c_check_functionality(adapter,
 					 I2C_FUNC_SMBUS_BYTE |
 					 I2C_FUNC_SMBUS_BYTE_DATA);
	assert(result);

	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));

	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);
  
	/*input poll device register */
	gsensor_idev = input_allocate_polled_device();
	if (!gsensor_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}
	
	gsensor_idev->poll = gsensor_dev_poll;
	gsensor_idev->poll_interval = POLL_INTERVAL;
	gsensor_idev->poll_interval_max = POLL_INTERVAL_MAX;

	gsensor_idev->input->name = GSENSOR_NAME;
	gsensor_idev->input->id.bustype = BUS_I2C;
	gsensor_idev->input->evbit[0] = BIT_MASK(EV_ABS);

    input_set_abs_params(gsensor_idev->input, ABS_MISC, 0, 512, 0, 0);
	
	result = input_register_polled_device(gsensor_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}
	
    result = sysfs_create_group(&gsensor_idev->input->dev.kobj, &gsensor_attribute_group);


	if(result) {
		dev_err(&client->dev, "create sys failed\n");
	}

    data->client  = client;
    data->pollDev = gsensor_idev;
	i2c_set_clientdata(client, data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	gsensor_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	gsensor_data.early_suspend.suspend = gsensor_early_suspend;
	gsensor_data.early_suspend.resume = gsensor_late_resume;
	register_early_suspend(&gsensor_data.early_suspend);
	gsensor_data.suspend_indator = 0;
#endif

	return result;
}

static int __devexit gsensor_remove(struct i2c_client *client)
{
	int result = 0;
	hwmon_device_unregister(hwmon_dev);
	#ifdef CONFIG_HAS_EARLYSUSPEND	
	  unregister_early_suspend(&gsensor_data.early_suspend);
	#endif
	input_unregister_polled_device(gsensor_idev);
	input_free_polled_device(gsensor_idev);
	i2c_set_clientdata(gsensor_i2c_client, NULL);	

	return result;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gsensor_early_suspend(struct early_suspend *h)
{
}

static void gsensor_late_resume(struct early_suspend *h)
{
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id gsensor_id[] = {
	{ GSENSOR_NAME, 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c,gsensor_id);

static struct i2c_driver gsensor_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= GSENSOR_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= gsensor_probe,
	.remove	= __devexit_p(gsensor_remove),
	.id_table = gsensor_id,
	.address_list	= normal_i2c,
};

static int __init gsensor_init(void)
{
	int ret = -1;
	printk("======%s=========. \n", __func__);
	
	if(gsensor_fetch_sysconfig_para()){
		printk("%s: err.\n", __func__);
		return -1;
	}

    gsensor_driver.detect = gsensor_detect;
	

	ret = i2c_add_driver(&gsensor_driver);
	if (ret < 0) {
		printk(KERN_INFO "add gsensor  driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add gsensor  driver\n");
	


	return ret;
}

static void __exit gsensor_exit(void)
{
	printk(KERN_INFO "remove gsensor  driver.\n");
	sysfs_remove_group(&gsensor_idev->input->dev.kobj, &gsensor_attribute_group);
	i2c_del_driver(&gsensor_driver);
}

MODULE_AUTHOR("Emma yin");
MODULE_DESCRIPTION("GSENSOR  Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

module_init(gsensor_init);
module_exit(gsensor_exit);

