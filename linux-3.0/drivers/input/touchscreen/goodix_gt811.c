/*---------------------------------------------------------------------------------------------------------
 * driver/input/touchscreen/goodix_touch.c
 *
 * Copyright(c) 2010 Goodix Technology Corp.     
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
 * Change Date: 
 *		2010.11.11, add point_queue's definiens.     
 *                             
 * 		2011.03.09, rewrite point_queue's definiens.  
 *   
 * 		2011.05.12, delete point_queue for Android 2.2/Android 2.3 and so on.
 *                                                                                              
 *---------------------------------------------------------------------------------------------------------*/
#include <linux/i2c.h>
#include <linux/input.h>
#include "goodix_gt811.h"
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>
 
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>
#include "ctp_platform_ops.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif

//#define GT811_I2C_SPEED 	(400*1000)

#define GT811_READ_TOUCH_ADDR_H   0x07
#define GT811_READ_TOUCH_ADDR_L   0x21
#define GT811_READ_KEY_ADDR_H     0x07
#define GT811_READ_KEY_ADDR_L     0x22
#define GT811_READ_COOR_ADDR_H    0x07
#define GT811_READ_COOR_ADDR_L    0x23
#define GT811_RESOLUTION_LOC      71
#define GT811_TRIGGER_LOC         66

#define ADDR_MAX_LENGTH     2
#define ADDR_LENGTH         ADDR_MAX_LENGTH
//#undef CONFIG_HAS_EARLYSUSPEND
#define GT811_MAX_SUPPORT_POINT	5
#define CTP_IRQ_NO				(gpio_int_info[0].port_num)
#define CTP_IRQ_MODE			(NEGATIVE_EDGE)
#define PEN_DOWN 1
#define PEN_RELEASE 0
extern int input_ctp_module_flg;
struct goodix_gt811_data {
	u8 bad_data;
    u8 irq_is_disable;
	int retry;
	int panel_type;
	char phys[32];		
	struct i2c_client *client;
	struct input_dev *input_dev;
	uint8_t use_irq;
	uint8_t use_shutdown;
	uint32_t gpio_shutdown;
	uint32_t gpio_irq;
	uint32_t screen_width;
	uint32_t screen_height;
	uint32_t reported_finger_count;
	bool pendown;
	uint16_t touch_x[GT811_MAX_SUPPORT_POINT];
	uint16_t touch_y[GT811_MAX_SUPPORT_POINT];
	struct ts_event		event;
	struct hrtimer timer;
	struct work_struct  work;
	int (*power)(struct goodix_gt811_data * ts, int on);
#ifdef CONFIG_HAS_EARLYSUSPEND	
    struct early_suspend early_suspend;
#endif
};

//wisky-lxh@20120216
static uint8_t *fw_config_p =NULL;
static unsigned int FW_LEN = 0;
static uint8_t CONFIG_M828AH_GT811_QIUTIAN[]=
{
	0x06,0xa2,/*config address*/
	0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,
	0x10,0x12,0xE4,0x44,0xD4,0x44,0xC4,0x44,
	0xB4,0x44,0xA4,0x44,0x94,0x44,0x84,0x44,
	0x74,0x44,0x64,0x44,0x54,0x44,0x44,0x44,
	0x34,0x44,0x24,0x44,0x14,0x44,0x04,0x44,
	0xF4,0x44,0x0B,0x13,0x68,0x68,0x68,0x1C,
	0x1C,0x1C,0x0F,0x0F,0x0A,0x43,0x31,0x5D,
	0x03,0x00,0x05,0xE0,0x01,0x20,0x03,0x00,
	0x00,0x2F,0x2B,0x31,0x2D,0x00,0x00,0x24,
	0x14,0x26,0x06,0x00,0x00,0x00,0x00,0x00,
	0x14,0x10,0xE1,0x01,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01
};


const char *goodix_gt811_name = "goodix_gt811";
#define CTP_NAME			GOODIX_GT811_I2C_NAME
static struct workqueue_struct *goodix_gt811_wq;
static void* __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static int gpio_wakeup_hdle = 0;
static int gpio_io_hdle = 0;
static int gpio_power_hdle = 0;
#define X_DIFF (800)
static int screen_max_x;
static int screen_max_y;
static int revert_x_flag = 0;
static int revert_y_flag = 0;

#define SCREEN_MAX_HEIGHT	480//(screen_max_y)
#define SCREEN_MAX_WIDTH   800// (screen_max_x)

#define WISKY_TOUCH_HEIGHT 600
#define WISKY_TOUCH_WIDTH  800

static __u32 twi_addr = 0;
static __u32 twi_id = 0;
static user_gpio_set_t  gpio_int_info[1];
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};
/* Addresses to scan */
union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
//停用设备
#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_gt811_early_suspend(struct early_suspend *h)
{
	int ret;
	int reg_val;
	
	struct goodix_gt811_data *ts = container_of(h, struct goodix_gt811_data, early_suspend);
    struct i2c_client * client = ts->client;
    #ifdef PRINT_SUSPEND_INFO
        printk("enter earlysuspend: goodix_ts_suspend. \n");
    #endif    
   
    //disable_irq(ts->gpio_irq);
	ret = cancel_work_sync(&ts->work);	
		
	if (ts->power) {
		ret = ts->power(ts,0);
		if (ret < 0)
			dev_warn(&client->dev, "goodix_gt811 power off failed\n");
	}

	//disable irq
	 reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
        reg_val &=~(1<<IRQ_EINT11);
        writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
		
	return ;
}

//重新唤醒
static void goodix_gt811_early_resume(struct early_suspend *h)
{
	int ret;
	int reg_val;
	
	struct goodix_gt811_data *ts = container_of(h, struct goodix_gt811_data, early_suspend);
    struct i2c_client * client = ts->client;
    
#ifdef PRINT_SUSPEND_INFO
        printk("enter laterresume: goodix_ts_resume. \n");
#endif 

	if (ts->power) {
		ret = ts->power(ts, 1);
		if (ret < 0)
			dev_warn(&client->dev, "%s power on failed\n", goodix_gt811_name);
	}

    //enable irq
     reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
     reg_val |=(1<<IRQ_EINT11);
     writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
		
	return ;
}
#else
#ifdef CONFIG_PM
//停用设备
static int goodix_gt811_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	
#ifdef PRINT_SUSPEND_INFO
        printk("enter: goodix_ts_suspend. \n");
#endif         
        //disable_irq(ts->gpio_irq);
	ret = cancel_work_sync(&ts->work);	
		
	if (ts->power) {
		ret = ts->power(ts,0);
		if (ret < 0)
			dev_warn(&client->dev, "%s power off failed\n", f3x_ts_name);
	}

	
	return 0;
}

//重新唤醒
static int goodix_gt811_resume(struct i2c_client *client)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	
#ifdef PRINT_SUSPEND_INFO
        printk("enter: goodix_ts_resume. \n");
#endif 

	if (ts->power) {
		ret = ts->power(ts, 1);
		if (ret < 0)
			dev_warn(&client->dev, "%s power on failed\n", f3x_ts_name);
	}

        //enable_irq(ts->gpio_irq);
	return 0;
}
#endif
#endif


/*used by GT80X-IAP module */
struct i2c_client * i2c_connect_client = NULL;
EXPORT_SYMBOL(i2c_connect_client);
/**********************************************************************	
本程序中I2C通信方式为：
	7bit从机地址｜读写位 + buf（数据地址+读写数据）
	 --------------------------------------------------------------------
	｜  从机地址   ｜ buf[0](数据地址) | buf[1]~buf[MAX-1](写入或读取到的数据)  |
	 --------------------------------------------------------------------
	移植前请根据自身主控格式修改！！
***********************************************************************/

//Function as i2c_master_receive, and return 2 if operation is successful.
static int gt811_read_bytes(struct i2c_client *client, uint8_t *buf, uint16_t len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;

    //发送写地址
    msgs[0].flags = !I2C_M_RD; //写消息
    msgs[0].addr = client->addr;
	//msgs[0].scl_rate = GT811_I2C_SPEED;
    msgs[0].len = 2;
    msgs[0].buf = &buf[0];
    //接收数据
    msgs[1].flags = I2C_M_RD;//读消息
    msgs[1].addr = client->addr;
	//msgs[1].scl_rate = GT811_I2C_SPEED;
    msgs[1].len = len - ADDR_LENGTH;
    msgs[1].buf = &buf[2];

    ret=i2c_transfer(client->adapter, msgs, 2);

    return ret;

}

//Function as i2c_master_send, and return 1 if operation is successful. 
static int gt811_write_bytes(struct i2c_client *client, uint8_t *data, uint16_t len)
{
  struct i2c_msg msg;
    s32 ret=-1;

    //发送设备地址
    msg.flags = !I2C_M_RD;//写消息
    msg.addr = client->addr;
	//msg.scl_rate = GT811_I2C_SPEED;
    msg.len = len;
    msg.buf = data;        

    ret = i2c_transfer(client->adapter, &msg, 1);
	
    return ret;
}

/*******************************************************
功能：
	发送前缀命令
	
	ts:	client私有数据结构体
return：
    成功返回1
*******************************************************/
static s32 gt81x_i2c_pre_cmd(struct goodix_gt811_data *ts)
{
    s32 ret;
    u8 pre_cmd_data[2] = {0x0f, 0xff};

    ret = gt811_write_bytes(ts->client, pre_cmd_data, 2);
    return ret;
}

/*******************************************************
功能：
	发送后缀命令
	
	ts:	client私有数据结构体
return：
    成功返回1
*******************************************************/
static s32 gt81x_i2c_end_cmd(struct goodix_gt811_data *ts)
{
    s32 ret;
    u8 end_cmd_data[2] = {0x80, 0x00};    

    ret = gt811_write_bytes(ts->client, end_cmd_data, 2);
    return ret;//*/
}


//Function as i2c_master_send, and return 1 if operation is successful. 
static int goodix_gt811_iic_test(struct i2c_client * client)
{
	struct i2c_msg msg;
	int ret = -1;
	uint8_t data[0];
	int i;
	for(i =0; i<256;i++) {
		msg.flags = !I2C_M_RD;//写消息
		msg.addr = i;
		msg.len = 1;
		msg.buf = data;		

		ret = i2c_transfer(client->adapter, &msg,1);
		if(ret == 1) {
		  printk("IIC TEST OK addr = %x\n",i);
		  break;
		}
		mdelay(5);
    }
	return ret;
}

/*读取GT8105的版本号并打印*/
static int  goodix_gt811_read_version(struct goodix_gt811_data *ts, char **version)
{
	int ret = -1, count = 0;
	char *version_data;
	char *p;
	
	*version = (char *)kmalloc(18,GFP_KERNEL);
	version_data = *version;
	if(!version_data)
		return -ENOMEM;
	p = version_data;
	memset(version_data, 0, sizeof(version_data));
	version_data[0]=240;
#if 0
	if(ts->green_wake_mode)			//WAKEUP GREEN MODE
	{
		disable_irq(ts->client->irq);
		gpio_direction_output(INT_PORT, 0);
		msleep(5);
		s3c_gpio_cfgpin(INT_PORT, INT_CFG);
		enable_irq(ts->client->irq);
	}
#endif
	ret=gt811_read_bytes(ts->client,version_data, 17);
	if (ret < 0) 
		return ret;
	version_data[17]='\0';
	
	if(*p == '\0')
		return 0; 	
	do 					
	{
		if((*p > 122) || (*p < 48 && *p != 32) || (*p >57 && *p  < 65) 
			||(*p > 90 && *p < 97 && *p  != '_'))		//check illeqal character
			count++;
	}while(*++p != '\0' );
	if(count > 2)
		return 0;
	else 
		return 1;	
}
/*******************************************************
功能：
	GT80X初始化函数，用于发送配置信息
参数：
	ts:	struct goodix_gt811_data
return：
	执行结果码，0表示正常执行
*******************************************************/
static bool goodix_gt811_init_panel(struct goodix_gt811_data *ts, u8 send)
{
	s32 ret = -1;
	Ha_debug("%s called!\n", __func__);
    if (send)
    {
        ret = gt811_write_bytes(ts->client,fw_config_p, FW_LEN);
        if (ret <= 0)
        {
        	Ha_debug("%s:gt81x_i2c_write_bytes failed!\n", __FUNCTION__);
            return fail;
        }
        gt81x_i2c_end_cmd(ts);
        msleep(10);
    }
    return success;
}

static s32 touch_num(u8 value, s32 max)
{
    s32 tmp = 0;

    while((tmp < max) && value)
    {
        if ((value & 0x01) == 1)
        {
            tmp++;
        }
        value = value >> 1;
    }

    return tmp;
}

#ifdef GT811_BORDER_ADJUST
static u16 adjust_xvalue(u16 val) 
{
	if (val < 27  && val > 7)
		return val * 137 / 127 + 2;
	else
		return val;
}

#define MAX_ADJUST_Y1 SCREEN_MAX_WIDTH - 54
#define MAX_ADJUST_Y2 SCREEN_MAX_WIDTH - 35
#define MAX_ADJUST_Y3 SCREEN_MAX_WIDTH - 14
#define MAX_ADJUST_Y4 SCREEN_MAX_WIDTH - 9
static u16 adjust_yvalue(u16 val) 
{
	if (val <= 12 && val > 1)
		return val * 10 / 11 - 1;
	else if (val > MAX_ADJUST_Y1 && val < MAX_ADJUST_Y2) 
		return val * 126 / 127 + 3;
	else if (val >= MAX_ADJUST_Y2 && val < MAX_ADJUST_Y3)
		return val * 136 / 137 + 2;
	else if (val >= MAX_ADJUST_Y3 && val < MAX_ADJUST_Y4)
		return val * 136 / 137 + 3;
	else if (val >= MAX_ADJUST_Y4)
		return (val * 177 / 176) % SCREEN_MAX_WIDTH;
	else 
		return val;
}
#endif
/*******************************************************	
功能：
	触摸屏工作函数
	由中断触发，接受1组坐标数据，校验后再分析输出
参数：
	ts:	client私有数据结构体
return：
	执行结果码，0表示正常执行
********************************************************/
static void goodix_gt811_work_func(struct work_struct *work)
{

//static struct point_node pointer[MAX_FINGER_NUM];
	//static uint8_t finger_last = 0;	//last time fingers' state

	//struct point_node * p = NULL;
	//uint8_t read_position = 0;
	uint8_t point_data[2 + 2 + 6 + 5 * MAX_FINGER_NUM + 1] = {GT811_READ_TOUCH_ADDR_H,GT811_READ_TOUCH_ADDR_L,0, 0};
	uint8_t finger;				//record which finger is changed
	//uint8_t check_sum = 0;
	u16 X_value, Y_value;
	uint8_t pressure;
	//int count = 0;
	int ret = -1,i; 
	//int err_code=0;
	//s32 i = 0, j = 0;
	int data_offset = 0;
	uint8_t finger_current[MAX_FINGER_NUM] = {0};        //当前触摸按键的手指索引
	struct goodix_gt811_data *ts = container_of(work, struct goodix_gt811_data, work);

	ret = gt811_read_bytes(ts->client, point_data, sizeof(point_data));
	gt81x_i2c_end_cmd(ts);
	
	finger = (u8)touch_num(point_data[2] & 0x1f, MAX_FINGER_NUM);
	Ha_debug("%s: finger = %d\n", __func__, finger);
	if (finger == 0) {
		//pressure = 0;
		goto NO_FINGER_TOUCH;
	}
	
	for (i = 0; i < MAX_FINGER_NUM; i++) {
        finger_current[i] = !!(point_data[2] & (0x01 << i));
		Ha_debug("figc = %d\n", finger_current[i]);
    }
	
	data_offset = 4;
	for (i = 0; i < MAX_FINGER_NUM; i++) {
		if (finger_current[i]) {
			X_value = point_data[data_offset + 0] << 8;
			/* the data not continuous, 0x733-0x738 is reserved*/
			if (i == 3) {
				data_offset += 6;
			}
			X_value = X_value | point_data[data_offset + 1];
			Y_value = point_data[data_offset + 2] << 8;
			Y_value = Y_value | point_data[data_offset + 3];
			//printk("the org val X:%d-Y:%d\n", X_value, Y_value);
		#ifdef GT811_BORDER_ADJUST
			//X_value = adjust_xvalue(X_value);
			//Y_value = adjust_yvalue(Y_value);
		#endif
			pressure = point_data[data_offset + 4];
			X_value = SCREEN_MAX_HEIGHT - X_value;
			Y_value = SCREEN_MAX_WIDTH - Y_value;
			//printk("after adjust X:%d-Y:%d\n", X_value, Y_value);

			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0x7f);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, Y_value);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, X_value);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			input_mt_sync(ts->input_dev);
		}
		else if (i == 3) {
			/* the data not continuous, 0x733-0x738 is reserved*/
			data_offset += 6;
		}
		data_offset += 5;
	}
	input_sync(ts->input_dev);
	return;
NO_FINGER_TOUCH:
	//for(i = 0; i < MAX_FINGER_NUM; i++) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev, BTN_2, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input_dev);
		//input_sync(ts->input);	
	//}
	input_sync(ts->input_dev);
	return;
}

/*******************************************************	
功能：
	中断响应函数
	由中断触发，调度触摸屏处理函数运行
********************************************************/
static irqreturn_t goodix_gt811_irq_handler(int irq, void *dev_id)
{

	struct goodix_gt811_data *ts = dev_id;	
	int reg_val;	
	//Ha_debug("==========------%s Interrupt-----==========\n", __func__); 
	//printk("%s\n",__FUNCTION__);
	//clear the IRQ_EINT21 interrupt pending
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
     
	if(reg_val & (1 << (IRQ_EINT_USED_GOODIX)))
	{	
		//Ha_debug("==IRQ_EINT21=\n");
		writel(reg_val & (1 << (IRQ_EINT_USED_GOODIX)), gpio_addr + PIO_INT_STAT_OFFSET);
		queue_work(goodix_gt811_wq, &ts->work);
	}
	else
	{
	    Ha_debug("Other Interrupt\n");
	    return IRQ_NONE;
	}

    return IRQ_HANDLED;
}

/*******************************************************	
功能：
	GT80X的电源管理
参数：
	on:设置GT80X运行模式，0为进入Sleep模式
return：
	是否设置成功，小于0表示设置失败
********************************************************/
static int goodix_gt811_power(struct goodix_gt811_data * ts, int on)
{
	int ret = 0;
	
	switch(on) 
	{
		case 0:
			gpio_write_one_pin_value(gpio_io_hdle, 1, "ctp_io_port");
			ret = 1;
			break;
		case 1:
		        gpio_write_one_pin_value(gpio_io_hdle, 0, "ctp_io_port");
		        ret = 1;
			break;	
	}
	//dev_dbg(&ts->client->dev, "Set Guitar's Shutdown %s. Ret:%d.\n", on?"LOW":"HIGH", ret);
	return ret;
}

//Test i2c to check device. Before it SHUTDOWN port Must be low state 30ms or more.
static bool goodix_gt811_i2c_test(struct i2c_client * client)
{
	int ret, retry;
	uint8_t test_data[1] = { 0 };	//only write a data address.
	
	for(retry=0; retry < 5; retry++)
	{
		ret =gt811_write_bytes(client, test_data, 1);	//Test i2c.
		if (ret == 1)
			break;
		msleep(5);
	}
	
	return ret==1 ? true : false;
}

static void ctp_wakeup(void)
{
	if(gpio_wakeup_hdle)
	{
	gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup");
	mdelay(100);
	gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup");	
    mdelay(300);
	gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup");	
    mdelay(100);
  }
		
	
	return;
}

static void ctp_free_platform_resource(void)
{
	printk(" goodix touch =======%s=========.\n", __func__);
	if(gpio_addr){
		iounmap(gpio_addr);
	}
	
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	
	if(gpio_wakeup_hdle){
		gpio_release(gpio_wakeup_hdle, 2);
	}
	

	if(gpio_power_hdle){
		gpio_release(gpio_power_hdle, 2);
	}
	return;
}
/**
 * ctp_clear_penirq - clear int pending
 *
 */
static void ctp_clear_penirq(void)
{
	int reg_val;
	
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if((reg_val = (reg_val&(1<<(CTP_IRQ_NO))))){
		           
		writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	}
	return;
}
 
static int ctp_set_irq_mode(char *major_key , char *subkey, ext_int_mode int_mode)
{
	int ret = 0;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;
	//config gpio to int mode
	
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	printk("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);
                                                               
	ctp_clear_penirq();
             
  //reg_val = readl(gpio_addr + PIO_INT_DEB_OFFSET);
  //reg_val |= 1;
  //writel(reg_val,gpio_addr + PIO_INT_DEB_OFFSET);           
                                                               
	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET); 
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);
  
  

	udelay(1);
#endif

request_tp_int_port_failed:
	return ret;  
}

/*******************************************************	
功能：
	触摸屏探测函数
	在注册驱动时调用（要求存在对应的client）；
	用于IO,中断等资源申请；设备注册；触摸屏初始化等工作
参数：
	client：待驱动的设备结构体
	id：设备ID
return：
	执行结果码，0表示正常执行
********************************************************/
static int goodix_gt811_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct goodix_gt811_data *ts;
	int ret = 0;
	int err;

	int retry;
	pr_info("===============================GT811 Probe===========================\n");
	Ha_debug("max_x = %d, max_y = %d\n", screen_max_x, screen_max_y);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "System need I2C function.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
		
	err = ctp_set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);
	
	if(0 != err){
		printk("%s:ctp_ops.set_irq_mode err. \n", __func__);
		goto exit_set_irq_mode;
	}
		
	
	i2c_connect_client = client;				//used by Guitar Updating.
  ts->gpio_irq = INT_PORT;
#define TEST_I2C_TRANSFER
#ifdef TEST_I2C_TRANSFER
	//TODO: used to set speed of i2c transfer. Should be change as your paltform.
	ret = goodix_gt811_i2c_test(client);
	if(!ret)
	{
		pr_info("Warnning: I2C connection might be something wrong!\n");
		goto err_i2c_failed;
	}
	pr_info("===== goodix i2c test ok=======\n");
#endif
	
	
	goodix_gt811_wq = create_singlethread_workqueue("goodix_gt811_wq");
    if (!goodix_gt811_wq) {
    	printk(KERN_ALERT "Creat %s workqueue failed.\n", GOODIX_GT811_I2C_NAME);
   		goto 	exit_create_singlethread;  	
    }
	
	INIT_WORK(&ts->work, goodix_gt811_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) 
	{
		ret = -ENOMEM;
		dev_dbg(&client->dev,"Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	
#ifndef GOODIX_MULTI_TOUCH	
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	input_set_abs_params(ts->input_dev, ABS_X, 0, SCREEN_MAX_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, SCREEN_MAX_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);	
	
#else
	ts->input_dev->absbit[0] = BIT_MASK(ABS_MT_TRACKING_ID) |
	BIT_MASK(ABS_MT_TOUCH_MAJOR)| BIT_MASK(ABS_MT_WIDTH_MAJOR) |
	BIT_MASK(ABS_MT_POSITION_X) | BIT_MASK(ABS_MT_POSITION_Y); 	// for android
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_HEIGHT, 0, 0);	
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MAX_FINGER_NUM, 0, 0);	
#endif	
	ts->bad_data = 0;
#ifdef FOR_TSLIB_TEST
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
#endif

	sprintf(ts->phys, "input/goodix_gt811");
	ts->input_dev->name = GOODIX_GT811_I2C_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 0x1105;	

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,"Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	flush_workqueue(goodix_gt811_wq);	
	//ts->power = goodix_gt811_power;
	msleep(30);	
	#define MAX_RETRY_NUM  5
	
    for (retry = 0; retry < MAX_RETRY_NUM; retry++)
    {
        if (success == goodix_gt811_init_panel(ts, 1))
        {
            printk("Initialize successfully!\n");
            break;
        }
    }
   // input_ctp_module_flg =1;
	if (retry >= MAX_RETRY_NUM)
    {
        ts->bad_data = 1;
		Ha_debug("Initialize failed!\n");
        goto err_init_godix_ts;
    }
	input_ctp_module_flg =1;
	
#ifdef CONFIG_HAS_EARLYSUSPEND	
    printk("==register_early_suspend =\n");	
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;	
    ts->early_suspend.suspend = goodix_gt811_early_suspend;
    ts->early_suspend.resume	= goodix_gt811_early_resume;	
    register_early_suspend(&ts->early_suspend);
#endif

	err =  request_irq(SW_INT_IRQNO_PIO, goodix_gt811_irq_handler, IRQF_TRIGGER_RISING | IRQF_SHARED, client->name, ts);

	if (err < 0) {
		pr_info( "goodix_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
		
    pr_info("========Probe Ok================\n");
	return 0;

exit_irq_request_failed:
	enable_irq(SW_INT_IRQNO_PIO);
err_init_godix_ts:	
	cancel_work_sync(&ts->work);
	destroy_workqueue(goodix_gt811_wq);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
    i2c_set_clientdata(client, NULL);
exit_create_singlethread:
err_i2c_failed:
exit_set_irq_mode:
kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	ctp_free_platform_resource();
	return ret;
}


/*******************************************************	
功能：
	驱动资源释放
参数：
	client：设备结构体
return：
	执行结果码，0表示正常执行
********************************************************/
static int goodix_gt811_remove(struct i2c_client *client)
{
	struct goodix_gt811_data *ts = i2c_get_clientdata(client);
	dev_notice(&client->dev,"The driver is removing...\n");
	#ifdef CONFIG_HAS_EARLYSUSPEND	
	    unregister_early_suspend(&ts->early_suspend);	
	#endif
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	if(ts->input_dev)
		kfree(ts->input_dev);
	kfree(ts);
	return 0;
}

//wisky-lxh@20111219
static void goodix_gt811_shutdown(struct i2c_client *client)
{
	 //gpio_write_one_pin_value(gpio_power_hdle, 0, "ctp_power_port");
	 Ha_debug("goodix_gt811_shutdown\n");
}
//end-wisky-lxh@20111219

//可用于该驱动的 设备名—设备ID 列表
//only one client
static const struct i2c_device_id goodix_gt811_id[] = {
	{ GOODIX_GT811_I2C_NAME, 0 },
	{ }
};

//设备驱动结构体
static struct i2c_driver goodix_gt811_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= goodix_gt811_probe,
	.remove		= goodix_gt811_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
#else
	.suspend	= goodix_gt811_suspend,
	.resume		= goodix_gt811_resume,
#endif
	.id_table	= goodix_gt811_id,
	.driver = {
		.name	= GOODIX_GT811_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.address_list	= u_i2c_addr.normal_i2c,
	.shutdown = goodix_gt811_shutdown,
};

static int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}
/**
 * ctp_init_platform_resource - initialize platform related resource
 * return value: 0 : success
 *               -EIO :  i/o err.
 *
 */

static int ctp_init_platform_resource(void)
{
	int ret = -1;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	if(!gpio_addr) {
	    goto exit_ioremap_failed;	
	}
	
	gpio_int_hdle = gpio_request_ex("ctp_para", "ctp_int_port");
	if(!gpio_int_hdle) {
		printk("touch panel IRQ_EINT21_para request gpio fail!\n");
	    goto exit_gpio_int_request_failed;
	}

	//reset
	gpio_wakeup_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if(!gpio_wakeup_hdle) {
		printk("touch panel tp_wakeup request gpio fail!\n");
		goto exit_gpio_wakeup_request_failed;
	}
	return 0;

exit_ioremap_failed:
exit_gpio_int_request_failed:
exit_gpio_wakeup_request_failed:	
	ctp_free_platform_resource();
	return ret;
}

/**
 * ctp_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE]={0};
	char tp_name[25] ={0};	
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
	
    if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)){
        Ha_debug("goodix_ts: script_parser_fetch err. \n");
        goto script_parser_fetch_err;
    }
    if(1 != ctp_used){
        Ha_debug("goodix_ts: ctp_unused. \n");
        return -1;
    }

    if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name1", (int *)(&name), &type, sizeof(name)/sizeof(int))){
            Ha_debug("goodix_ts_init: script_parser_fetch err. \n");
            goto script_parser_fetch_err;
    }
    if(strcmp(GOODIX_GT811_I2C_NAME, name)){
        Ha_debug("goodix_ts_init: name %s does not match GOODIX_GT811_I2C_NAME %s. \n", name,GOODIX_GT811_I2C_NAME);
        return -1;
    }
    
    if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)){
        Ha_debug("goodix_ts: script_parser_fetch err. \n");
        goto script_parser_fetch_err;
    }
    Ha_debug("goodix_ts: screen_max_x = %d. \n", screen_max_x);
    
    if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)){
        Ha_debug("goodix_ts: script_parser_fetch err. \n");
        goto script_parser_fetch_err;
    }
    Ha_debug("goodix_ts: screen_max_y = %d. \n", screen_max_y);

    
    
    if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1)){
        Ha_debug("goodix_ts: script_parser_fetch err. \n");
        goto script_parser_fetch_err;
    }
    pr_info("goodix_ts: revert_x_flag = %d. \n", revert_x_flag);

    if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1)){
        Ha_debug("goodix_ts: script_parser_fetch err. \n");
        goto script_parser_fetch_err;
    }
    Ha_debug("goodix_ts: revert_y_flag = %d. \n", revert_y_flag);
    //wisky-lxh@20120216
	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_tp_name1", (int *)(&tp_name), &type, sizeof(tp_name)/sizeof(int))){
			pr_err("%s: script_parser_fetch err. \n", __func__);
			//goto script_parser_fetch_err;
	}	
	if (!strcmp("m828ah_goodix_qiutian", tp_name))
	{
			
		fw_config_p = CONFIG_M828AH_GT811_QIUTIAN;
		FW_LEN = sizeof(CONFIG_M828AH_GT811_QIUTIAN)/sizeof(CONFIG_M828AH_GT811_QIUTIAN[0]);
		printk("tp_name:%s,%d,\n",tp_name,FW_LEN);
	}	
	else
	{
		fw_config_p = CONFIG_M828AH_GT811_QIUTIAN;
		FW_LEN = sizeof(CONFIG_M828AH_GT811_QIUTIAN)/sizeof(CONFIG_M828AH_GT811_QIUTIAN[0]);
		printk("tp_name:%s,%d,\n",tp_name,FW_LEN);
	}
	
	
		if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr1", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err.ctp_twi_addr... \n", name);
		goto script_parser_fetch_err;
	}
	
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	printk("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	//printk("%s: after: ctp_twi_addr is 0x%x, u32_dirty_addr_buf: 0x%hx. u32_dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u32_dirty_addr_buf[0],u32_dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. ctp_twi_id...\n", name);
		goto script_parser_fetch_err;
	}
	printk("%s: ctp_twi_id is %d. \n", __func__, twi_id);	
	return 0;
	script_parser_fetch_err:
	printk("=========script_parser_fetch_err============\n");
	return ret;
}

//驱动加载函数
static int __devinit goodix_gt811_init(void)
{
	
	
		printk("goodix_gt811_init\n");
		if(input_ctp_module_flg){
			printk("NOTE:other touch modules already insmod...\n");
			return -1;
		}
	
		if(ctp_fetch_sysconfig_para()){
			printk("%s: err.\n", __func__);
			return -1;
		}
		
		if(0 != ctp_init_platform_resource()){
		printk("%s:ctp_ops.init_platform_resource err. \n", __func__);   
		return -1; 
	}
		ctp_wakeup();
		goodix_gt811_driver.detect = ctp_detect;	
    i2c_add_driver(&goodix_gt811_driver);


	return 0; 
}

//驱动卸载函数
static void __exit goodix_gt811_exit(void)
{
	i2c_del_driver(&goodix_gt811_driver);
	if (goodix_gt811_wq)
		destroy_workqueue(goodix_gt811_wq);
}

late_initcall(goodix_gt811_init);
module_exit(goodix_gt811_exit);

MODULE_DESCRIPTION("Goodix Gt811 Touchscreen Driver@wisky-lxh-20111228");
MODULE_LICENSE("GPL v2");

