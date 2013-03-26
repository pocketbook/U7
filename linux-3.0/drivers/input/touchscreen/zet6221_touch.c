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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
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

#include "zet6221_touch.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

#include <linux/module.h>  
#include <linux/init.h>  
#include <linux/fs.h>  
#include <linux/uaccess.h>  
#include <asm/stat.h>


//#define ZET6221_I2C_SPEED 	(400*1000)
#define P_MAX	1
#define ZET6221_READ_TOUCH_ADDR_H   0x07
#define ZET6221_READ_TOUCH_ADDR_L   0x21
#define ZET6221_READ_KEY_ADDR_H     0x07
#define ZET6221_READ_KEY_ADDR_L     0x22
#define ZET6221_READ_COOR_ADDR_H    0x07
#define ZET6221_READ_COOR_ADDR_L    0x23
#define ZET6221_RESOLUTION_LOC      71
#define ZET6221_TRIGGER_LOC         66


//#define MAX_FINGER_NUM	5
#define MicroTimeTInterupt	(25000000)
static __u32 twi_addr = 0;
static __u32 twi_id = 0;
static user_gpio_set_t  gpio_int_info[1];
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};
#define CTP_IRQ_NO				(gpio_int_info[0].port_num)
#define CTP_IRQ_MODE			(NEGATIVE_EDGE)
/* Addresses to scan */
union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};

extern int usb_valid_flag;//add for charge mode
int ChargeChange=-1;//add for charge mode
extern void Setbit_forMD7008(void);//add for  md7008
extern void Clearbit_forMD7008(void);//add for  md7008


#define ZET6221_PEN_DOWN 0
#define ZET6221_PEN_RELEASE 1
static int status[5]={0};
static int status_for_change[5]={0};
struct zet6221_data {
	int finger;
	struct i2c_client *client;
	struct input_dev *input_dev;
	uint32_t gpio_irq;	
	uint16_t touch_x[MAX_FINGER_NUM];
	uint16_t touch_y[MAX_FINGER_NUM];
	struct hrtimer timer;
	struct work_struct  work;
	struct work_struct  work1;
	int enable;
#ifdef CONFIG_HAS_EARLYSUSPEND	
    struct early_suspend early_suspend;
#endif
};

//albert++
u8 debounce_number[MAX_FINGER_NUM];

int input_ctp_module_flg=0;
EXPORT_SYMBOL_GPL(input_ctp_module_flg);
//#define CTP_NAME "zet6221_ts"
const char *zet6221_name = "zet6221-ts";
static struct workqueue_struct *zet6221_wq;
static struct workqueue_struct *zet6221_wq1;
static void* __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static int gpio_wakeup_hdle = 0;
static int ctp_hold=0;
static int ctp_hold_value=-1;
static int hold_value_change=-1;

static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag=0;
static int no_touch_flag=0;
static int ResolutionX; //(SCREEN_MAX_WIDTH);
static int ResolutionY; //(SCREEN_MAX_HEIGHT);
#define SCREEN_MAX_HEIGHT	 (ResolutionY)//640//480//(screen_max_y)
#define SCREEN_MAX_WIDTH   (ResolutionX)//960//800// (screen_max_x)

static int Pendown=0;
static u16 FingerNum=0;
static u16 KeyNum=0;
static int bufLength=0;	
static u8 inChargerMode=0;
static u8 pc[8];

//static u16 fb[8] = {0x3EEA,0x3EED,0x3EF0,0x3EF3,0x3EF6,0x3EF9,0x3EFC,0x3EFF};
//static u16 fb[8] = {0x3DF1,0x3DF4,0x3DF7,0x3DFA,0x3EF6,0x3EF9,0x3EFC,0x3EFF};
static unsigned char *fw_zet6221_config_p = NULL;
static unsigned char CTPM_FW_M876A_ZET6221_JIAGUANG[] __initdata = 
{
	#include "zet6221_config/m876a_zet6221_jiaguang.i"	
};
static unsigned char CTPM_FW_M876A_ZET6221_RUISHI[] __initdata = 
{
	#include "zet6221_config/m876a_zet6221_ruishi.i" 
};
static unsigned char CTPM_FW_M876A_ZET6221_QIUTIAN[] __initdata = 
{
	#include "zet6221_config/876a-03.h"
};
static unsigned char CTPM_FW_M876A_ZET6221_PINGBO[] __initdata = 
{
#include "zet6221_config/m876a_zet6221_pingbo.i"
};
static unsigned char CTPM_FW_M828A_ZET6221_QIUTIAN[] __initdata = 
{
	#include "zet6221_config/m828a_zet6221_qiutian.i"
};
static unsigned char CTPM_FW_T720A_ZET6221_QIUTIAN[] __initdata = 
{
	#include "zet6221_config/t720_zet6221_qiutian.i"
};
static unsigned char CTPM_FW_MD7008A_ZET6221_HANGQING[] __initdata = 
{
	#include "zet6221_config/wisky_JG_MD7008_70_10x15_v32020814312-20121021.h"
	//#include "zet6221_config/md7008a_zet6221_hangqing.i"
};
static unsigned char CTPM_FW_T801A_ZET6221_QIUTIAN[] __initdata = 
{
#include "zet6221_config/t801_zet6221_pb.i"
};
static unsigned char CTPM_FW_U7A_ZET6221_QIUTIAN[] __initdata = 
{
#include"zet6221_config/u7_zet6221_qiutian.i"
};
static unsigned char CTPM_FW_U7A_ZET6221_CHENGXIN[] __initdata = 
{
#include "zet6221_config/u7_zet6221_chengxin.i"
};
static unsigned char CTPM_FW_U7A_ZET6221_RUISHI[] __initdata = 
{
#include "zet6221_config/u7_zet6221_ruishi.i"
};
static unsigned char CTPM_FW_L7A_ZET6221_LENHENG[] __initdata = 
{
#include "zet6221_config/L7_Zet6221_LH.i"
};
static unsigned char CTPM_FW_M858A_ZET6221_QIUTIAN[] __initdata = 
{
#include "zet6221_config/m858_zet6221_qiutian.i" 
//"zet6221_config/m858_zet6221_pingbo.i"
};
 
static unsigned char CTPM_FW_M858A_ZET6221_PINGBO[] __initdata = 
{
#include"zet6221_config/m858_zet6221_pingbo.i"
};
static unsigned int FW_ZET6221_LEN = 0;

#define ZET6221_UPGRADE_FW	 
#define ZET6221_TPINFO 
#if (defined(ZET6221_UPGRADE_FW) || defined(ZET6221_TPINFO))
	#include "zet6221_downloader.c"
#endif

static ssize_t zet6221_enable_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", ctp_hold_value);
}
static DEVICE_ATTR(enable,S_IRUGO|S_IWUSR, zet6221_enable_show,NULL);


static void disable_tp_irq(void)
{
	int reg_val;
	reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
  reg_val &=~(1<<IRQ_EINT11);
  writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
}

static void enable_tp_irq(void)
{
	int reg_val;
	reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
  reg_val |=(1<<IRQ_EINT11);
  writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
}


//Function as i2c_master_receive, and return 2 if operation is successful.
static int zet6221_read_bytes(struct i2c_client *client, uint8_t *buf, uint16_t len)
{
    struct i2c_msg msg;
    s32 ret=-1;
	
    msg.flags = I2C_M_RD;//读消息
    msg.addr = client->addr;
	//msgs[1].scl_rate = zet6221_I2C_SPEED;
    msg.len = len ;
    msg.buf = buf;

    ret=i2c_transfer(client->adapter, &msg, 1);
	
    return ret;

}
//Function as i2c_master_send, and return 1 if operation is successful. 
static int zet6221_write_bytes(struct i2c_client *client, uint8_t *data, uint16_t len)
{
  struct i2c_msg msg;
    s32 ret=-1;

    //发送设备地址
    msg.flags = !I2C_M_RD;//写消息
    msg.addr = client->addr;
//msg.scl_rate = zet6221_I2C_SPEED;
    msg.len = len;
    msg.buf = data;        

    ret = i2c_transfer(client->adapter, &msg, 1);
	
    return ret;
}
/*
 *
 * add for write charge_mode_enable cmd
 *
 *
 */
static void ts_write_charge_enable_cmd(struct zet6221_data *ts)
{

	u8 ts_write_charge_cmd[1] = {0xb5}; 
	int ret=0;
	
	ret=zet6221_write_bytes(ts->client, ts_write_charge_cmd, 1);
	

}
/*
 *
 * add for write charge_mode_enable cmd 
 *
 *
 */
static void ts_write_charge_disable_cmd(struct zet6221_data *ts)
{
	
	
	u8 ts_write_cmd[1] = {0xb6}; 
	int ret=0;	
	ret=zet6221_write_bytes(ts->client, ts_write_cmd, 1);
}
static void ctp_wakeup(void)
{
	
		//pr_info("%s. \n", __func__);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(30);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(30);
	
	return;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void zet6221_early_suspend(struct early_suspend *h)
{
	int ret=0;
	u8 ts_sleep_cmd[1] = {0xb1}; 
		struct zet6221_data *ts = container_of(h, struct zet6221_data, early_suspend);
    #ifdef PRINT_SUSPEND_INFO
        printk("enter earlysuspend: zet6221_suspend. \n");
    #endif    
    
        hrtimer_cancel(&ts->timer);
	 ret = cancel_work_sync(&ts->work);	
	 cancel_work_sync(&ts->work1);	
        disable_tp_irq();
	 //Sleep Mode
	 #if 1		  	
	 ret=zet6221_write_bytes(ts->client, ts_sleep_cmd, 1);
	 #endif
	 
	 return ;
}

//重新唤醒
static void zet6221_early_resume(struct early_suspend *h)
{
	
	//int reg_val;
  struct zet6221_data *ts = container_of(h, struct zet6221_data, early_suspend);
#ifdef PRINT_SUSPEND_INFO
        printk("enter laterresume: zet6221_resume. \n");
#endif 

	
    ctp_wakeup();
    
    if(usb_valid_flag == 1) {
			ts_write_charge_enable_cmd(ts);
			
		}else if(usb_valid_flag == 0)
		{
			ts_write_charge_disable_cmd(ts);
		}
	if(ctp_hold)
	{
		ctp_hold_value=gpio_read_one_pin_value(ctp_hold,"ctp_hold");
		//printk("%s:ctp_hold_value:%d\n",__func__,ctp_hold_value);
		 hrtimer_start(&ts->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
		if(!ctp_hold_value)
		{
		return ;
		}
		enable_tp_irq();
	}else{
	 hrtimer_start(&ts->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	 enable_tp_irq();
	}
		
     
	
	
    		
	return ;
}
#else
#ifdef CONFIG_PM
//停用设备
static int zet6221_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct zet6221_data *ts = i2c_get_clientdata(client);
	
#ifdef PRINT_SUSPEND_INFO
        printk("enter: zet6221_suspend. \n");
#endif         

  hrtimer_cancel(&ts->timer);
  
	ret = cancel_work_sync(&ts->work);	
	cancel_work_sync(&ts->work1);
	
	return 0;
}

//重新唤醒
static int zet6221_resume(struct i2c_client *client)
{
	int ret;
	struct zet6221_ts_data *ts = i2c_get_clientdata(client);
	
#ifdef PRINT_SUSPEND_INFO
        printk("enter: zet6221_ts_resume. \n");
#endif 

	hrtimer_start(&ts->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);

	return 0;
}
#endif
#endif

static enum hrtimer_restart zet6221_timer(struct hrtimer *timer)
{	
	struct zet6221_data *ts = container_of(timer, struct zet6221_data, timer);
	queue_work(zet6221_wq1, &ts->work1);
	hrtimer_start(&ts->timer, ktime_set(0, MicroTimeTInterupt*20), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void write_cmd_work(struct work_struct *work1)
{
	struct zet6221_data *ts = container_of(work1, struct zet6221_data, work1);
	char **uevent_envp = NULL;
	char *lock[2]    = { "tp status=lock", NULL };
	char *unlock[2]   = { "tp status=unlock", NULL };
	
	if(ctp_hold)
	{
		ctp_hold_value=gpio_read_one_pin_value(ctp_hold,"ctp_hold");
		//printk("ctp_hold:%d\n",gpio_read_one_pin_value(ctp_hold,"ctp_hold"));
		if(ctp_hold_value!=hold_value_change){	
			if(ctp_hold_value==1)
			{
				Setbit_forMD7008();
			//	enable_tp_irq();
				uevent_envp=unlock;
			}
			else if(ctp_hold_value==0)
			{
				Clearbit_forMD7008();
			//	disable_tp_irq();
				uevent_envp=lock;
			}
			hold_value_change=ctp_hold_value;
			kobject_uevent_env(&(ts->input_dev->dev.kobj), KOBJ_CHANGE, uevent_envp);
			DEBUG_COOR("%s: sent uevent %s\n", __func__, uevent_envp[0]);
			DEBUG_COOR("hold_value_change:%d\n",hold_value_change);
		}
	}
	
	if(ZET6221_PEN_DOWN==Pendown)
	{		
		return ;
	}

	if((usb_valid_flag != ChargeChange))
	{	
		if(usb_valid_flag == 1) 
		{			
			ts_write_charge_enable_cmd(ts);								
		}else if(usb_valid_flag == 0)
		{			
			ts_write_charge_disable_cmd(ts);			
		}
		
		DEBUG_COOR("--%s--:%d\n",__func__,usb_valid_flag);
		ChargeChange = usb_valid_flag;
		DEBUG_COOR("finger up ChargeChange :%d\n",ChargeChange);
		DEBUG_COOR("PendownStatue:%d\n",PendownStatue);
	}

}

/*******************************************************	
功能：
	触摸屏工作函数
	由中断触发，接受1组坐标数据，校验后再分析输出
参数：
	ts:	client私有数据结构体
return：
	执行结果码，0表示正常执行
********************************************************/

static inline  int  zet6221_read_values(struct zet6221_data *ts)
{
	int err = 0;
	u8 read_buf[25] = {0};
	
	int  i = 0;
	ts->finger=0;
	
	err =zet6221_read_bytes(ts->client, read_buf, bufLength);//19
		if (err < 0)
		{
			DEBUG("read coorlen failed\n");
			goto err_exit;
		}
		
	for(i = 0; i < FingerNum; i++) {
		if (read_buf[1] & ((0x80)>>i)){
			
			ts->touch_x[i]=((read_buf[3+4*i])>>4)*256 + (u8)read_buf[(3+4*i)+1];
			ts->touch_y[i]=((read_buf[3+4*i]) & 0x0f)*256 + (u8)read_buf[(3+4*i)+2];
			ts->finger ++;
			status[i]=1;
			debounce_number[i]=0; //albert++
		}
		else{
			//ts->touch_x[i]=0;
			//ts->touch_y[i]=0;
			//status[i]=0;
			//albert++
			if(debounce_number[i]>3)
			{
				ts->touch_x[i]=0;
				ts->touch_y[i]=0;
				status[i]=0;
				debounce_number[i]=0;
			}
			debounce_number[i]++;

		}
	}
	if(ts->finger == 0)
		{
			//no_touch_flag++;
			DEBUG_COOR("no_touch_flag:%d\n",no_touch_flag);
			if(no_touch_flag<1)
			{
					no_touch_flag++;
					return ZET6221_PEN_RELEASE;
			}else
			{
					goto no_touch;	
			}

		}
		
	DEBUG_COOR("x=%d y=%d\n",ts->touch_x[0],ts->touch_y[0]);
	no_touch_flag=0;
	for(i = 0; i < FingerNum; i++)
	{
			if(1 == exchange_x_y_flag)
			swap(ts->touch_x[i], ts->touch_y[i]);
			if(revert_x_flag)
				ts->touch_x[i]=SCREEN_MAX_WIDTH - ts->touch_x[i];			
			if(revert_y_flag)
				ts->touch_y[i]=SCREEN_MAX_HEIGHT -ts->touch_y[i];
				
			
			if(status[i])
				{
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0x7f);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 1);
					if(debounce_number[i]==0) //albert++
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,  ts->touch_x[i]);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  ts->touch_y[i]);
					}
					input_mt_sync(ts->input_dev);
				}
			else
				{
					if(status_for_change[i]!=status[i]){
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_mt_sync(ts->input_dev);
					}
				}
			status_for_change[i]=status[i];
	}


	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_sync(ts->input_dev);
		
	return ZET6221_PEN_DOWN;
	
no_touch:
    		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev, BTN_2, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);
		no_touch_flag=0;
err_exit:

	return ZET6221_PEN_RELEASE;
}

static void zet6221_work_func(struct work_struct *work)
{

	struct zet6221_data *ts = container_of(work, struct zet6221_data, work);
			Pendown=zet6221_read_values(ts);
		
}

/*******************************************************	
功能：
	中断响应函数
	由中断触发，调度触摸屏处理函数运行
********************************************************/
static irqreturn_t zet6221_irq_handler(int irq, void *dev_id)
{

	struct zet6221_data *ts = dev_id;	
	int reg_val;	
	DEBUG_COOR("==========------%s Interrupt-----==========\n", __func__); 

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
     
	if(reg_val & (1 << (IRQ_EINT_USED_ZET6221)))
	{	
		writel(reg_val & (1 << (IRQ_EINT_USED_ZET6221)),  gpio_addr + PIO_INT_STAT_OFFSET);
		queue_work(zet6221_wq, &ts->work);
	}
	else
	{
	    printk("Other Interrupt\n");
	    return IRQ_NONE;
	}
	
    return IRQ_HANDLED;
}

static void ctp_free_platform_resource(void)
{
	DEBUG(" zet6221 touch =======%s=========.\n", __func__);
	if(gpio_addr){
		iounmap(gpio_addr);
	}
	
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	
	if(gpio_wakeup_hdle){
		gpio_release(gpio_wakeup_hdle, 2);
	}
	
	if(ctp_hold){
	gpio_release(ctp_hold, 2);
	}

	return;
}

static void ctp_clear_penirq(void)
{
	int reg_val;
	
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if((reg_val = (reg_val&(1<<(CTP_IRQ_NO))))){
		DEBUG("==CTP_IRQ_NO=\n");              
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
	
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		//WPRINTK("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	DEBUG("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	DEBUG(" INTERRUPT CONFIG\n");
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	DEBUG("reg_num:%d reg_addr:%d\n",reg_num,reg_addr);
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);
                                                               
	ctp_clear_penirq();
//设置最高频率中断         
//  reg_val = readl(gpio_addr + 0x218);
//  reg_val |= 1;
//  writel(reg_val,gpio_addr + 0x218);           
                                                               
	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET); 
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);

	udelay(1);
#endif

request_tp_int_port_failed:
	return ret;  
}

static bool zet6221_i2c_test(struct i2c_client * client)
{
	int ret, retry;
	uint8_t test_data[1] = { 0 };	//only write a data address.
	
	if(fw_zet6221_config_p==NULL)
		return 0;
	
	for(retry=0; retry < 5; retry++)
	{
		ret =zet6221_write_bytes(client, test_data, 1);	//Test i2c.
		if (ret == 1)
			break;
		msleep(5);
	}
	
	return ret == 1 ? true : false;
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
static int zet6221_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct zet6221_data *ts;
	int ret = 0;
	int count = 0;
	int err;
	int cnt = 0;

	printk("Zet6221 Probe COMPILE TIME:%s, %s.\n", __DATE__, __TIME__);

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
		
	DEBUG_PROBE("ResolutionX=%d ResolutionY=%d\n",ResolutionX,ResolutionY);
	
#define TEST_I2C_TRANSFER
#ifdef TEST_I2C_TRANSFER
	DEBUG_PROBE("Begin ZET6221 i2c test\n");
	ret = zet6221_i2c_test(client);
	if(!ret) {
		printk("error: I2C connection might be something wrong!\n");
		goto err_i2c_failed;
	}
	DEBUG_PROBE("===== ZET6221 i2c test ok=======\n");
#endif

#ifdef ZET6221_UPGRADE_FW
zet_download:
	#define RETRY_DOWNLOAD_CNT  2
	
	zet6221_downloader(client);
	cnt++;
#endif
	
#ifdef ZET6221_TPINFO
	#define REPORT_POLLING_TIME 5
	do {	
		ctp_wakeup();
		if(zet6221_ts_get_report_mode_t(client)==0)  //get IC info by delay 
		{
			bufLength  = 3 + 4 * MAX_FINGER_NUM;		
		} else {
			//bootloader
			if(zet6221_ts_version()!=0)
				break;
		}	
		count++;
	}while(count < REPORT_POLLING_TIME);
#ifdef ZET6221_UPGRADE_FW	
	//bootloader
	if(count == REPORT_POLLING_TIME&& cnt < RETRY_DOWNLOAD_CNT)
		goto zet_download;
#endif
	
#else
		bufLength  = 3 + 4 * MAX_FINGER_NUM;
#endif
	
	ts->gpio_irq = SW_INT_IRQNO_PIO;
	err = ctp_set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);

	if(0 != err){
		printk("%s:ctp_ops.set_irq_mode err. \n", __func__);
		goto exit_set_irq_mode;
	}

	INIT_WORK(&ts->work1, write_cmd_work);
	zet6221_wq1 = create_singlethread_workqueue("zet6221_wq1"); // workqueue
	if (!zet6221_wq1) {	
		printk(KERN_ALERT "Creat %s workqueue failed.\n", ZET6221_I2C_NAME);	
		goto exit_create_singlethread;
		
	}
	INIT_WORK(&ts->work, zet6221_work_func);
	zet6221_wq = create_singlethread_workqueue("zet6221_wq");
  if (!zet6221_wq) {
    	printk(KERN_ALERT "Creat %s workqueue failed.\n", ZET6221_I2C_NAME);
   goto 	exit_create_singlethread;
    		
    }
	
	ts->client = client;
	i2c_set_clientdata(client, ts);
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) 
	{
		ret = -ENOMEM;
		dev_dbg(&client->dev,"Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) ;
	
#ifndef ZET6221_MULTI_TOUCH	
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
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ResolutionX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ResolutionY, 0, 0);	
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MAX_FINGER_NUM, 0, 0);
	DEBUG_PROBE("tp-size:wxh(%dx%d)",SCREEN_MAX_WIDTH,SCREEN_MAX_HEIGHT);	
#endif	


	ts->input_dev->name = ZET6221_I2C_NAME;
	ts->input_dev->phys = "input/zet6221";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 0x1105;	

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,"Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	
	flush_workqueue(zet6221_wq);	
	flush_workqueue(zet6221_wq1);
	msleep(30);	
	
#ifdef CONFIG_HAS_EARLYSUSPEND	
    DEBUG_PROBE("==register_early_suspend =\n");	
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;	
    ts->early_suspend.suspend = zet6221_early_suspend;
    ts->early_suspend.resume	= zet6221_early_resume;	
    register_early_suspend(&ts->early_suspend);
#endif
		//for charge mode
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = zet6221_timer;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		
	err =  request_irq(ts->gpio_irq , zet6221_irq_handler, IRQF_TRIGGER_FALLING|IRQF_SHARED|IRQF_PROBE_SHARED, client->name, ts);

	if (err < 0) {
		pr_info( "zet6221_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	err = device_create_file(&ts->input_dev->dev, &dev_attr_enable);
	if(err)
		printk("create file for enable err!\n");

	
	input_ctp_module_flg=1;
	return 0;

	
	
exit_irq_request_failed:
enable_irq(SW_INT_IRQNO_PIO);
input_unregister_device(ts->input_dev);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
    i2c_set_clientdata(client, NULL);
exit_create_singlethread:
	if (zet6221_wq)
		destroy_workqueue(zet6221_wq);
	if(zet6221_wq1)
		destroy_workqueue(zet6221_wq1);
exit_set_irq_mode:
err_i2c_failed:
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
static int zet6221_remove(struct i2c_client *client)
{
	struct zet6221_data *ts = i2c_get_clientdata(client);
	dev_notice(&client->dev,"The driver is removing...\n");
	#ifdef CONFIG_HAS_EARLYSUSPEND	
	    unregister_early_suspend(&ts->early_suspend);	
	#endif
	//disable_irq(SW_INT_IRQNO_PIO);
	free_irq(SW_INT_IRQNO_PIO, ts);
	device_remove_file(&ts->input_dev->dev, &dev_attr_enable);
	
	//disable_irq(ts->gpio_irq);
	hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	
	if (zet6221_wq){
			cancel_work_sync(&ts->work);	
			flush_workqueue(zet6221_wq);		
			destroy_workqueue(zet6221_wq);
			zet6221_wq=NULL;
		}
	if(zet6221_wq1){
		cancel_work_sync(&ts->work1);
		flush_workqueue(zet6221_wq1);
		destroy_workqueue(zet6221_wq1);
		zet6221_wq1=NULL;
	}
	if(ts->input_dev)
		kfree(ts->input_dev);
	kfree(ts);
	ts=NULL;
	i2c_set_clientdata(client, NULL);
	ctp_free_platform_resource();
	if(input_ctp_module_flg==1)
	input_ctp_module_flg=0;
	return 0;
}

//可用于该驱动的 设备名—设备ID 列表
//only one client
static const struct i2c_device_id zet6221_id[] = {
	{ ZET6221_I2C_NAME, 0 },
	{ }
};

//设备驱动结构体
static struct i2c_driver zet6221_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= zet6221_probe,
	.remove		= zet6221_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
#else
	.suspend	= zet6221_suspend,
	.resume		= zet6221_resume,
#endif
	.id_table	= zet6221_id,
	.driver = {
		.name	= ZET6221_I2C_NAME,
		.owner = THIS_MODULE,
	},
	//.shutdown = zet6221_shutdown,
	.address_list	= u_i2c_addr.normal_i2c,
};

static int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		DEBUG_PROBE("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, ZET6221_I2C_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, ZET6221_I2C_NAME, I2C_NAME_SIZE);
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
	int ret=-1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE]={0};
	char tp_name[25] ={0};
	char MachName[25]={0};
	int x=0,y=0;
	
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
	
	DEBUG_PROBE("zet6221_init\n");

		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("product", "machine", (int *)(&MachName), &type, sizeof(MachName)/sizeof(int))){
		pr_err("%s: line: %d script_parser_fetch err.,%s\n", __func__, __LINE__,MachName);
			
		}
		DEBUG_PROBE("tp match machine:%s\n",MachName);

    if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)){
        DEBUG_PROBE("ZET6221_ts: script_parser_fetch err. \n");
        goto script_parser_fetch_err;
    }
    if(1 != ctp_used){
        DEBUG_PROBE("ZET6221_ts: ctp_unused. \n");
        return -1;
    }

    if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name2", (int *)(&name), &type, sizeof(name)/sizeof(int))){
            DEBUG_PROBE("ZET6221_ts_init: script_parser_fetch err. \n");
            goto script_parser_fetch_err;
    }
    if(strcmp(ZET6221_I2C_NAME, name)){
        DEBUG_PROBE("ZET6221_ts_init: name %s does not match ZET6221_I2C_NAME %s. \n", name,ZET6221_I2C_NAME);
        return -1;
    }

	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_tp_name2", (int *)(&tp_name), &type, sizeof(tp_name)/sizeof(int))){
			pr_err("%s: script_parser_fetch err. \n", __func__);
			goto script_parser_fetch_err;
	}
	//兼容模块使用
	getxy(&x,&y);
	#if 1
	if (!strcmp("M876-A13", MachName))
	{
		
		if(y>1850&&y<2200){
			DEBUG_PROBE("Y catch\n");
		if(x>350&&x<770)
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN[0]);
			
			DEBUG_PROBE("CTPM_FW_M876A_ZET6221_QIUTIAN\n");
		}
		else if(x>1160&&x<1570)
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_PINGBO;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_PINGBO)/sizeof(CTPM_FW_M876A_ZET6221_PINGBO[0]);
			
			DEBUG_PROBE("CTPM_FW_M876A_ZET6221_PINGBO\n");
		}
		else if(x>2065&&x<2475)
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_RUISHI;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_RUISHI)/sizeof(CTPM_FW_M876A_ZET6221_RUISHI[0]);
			
			DEBUG_PROBE("CTPM_FW_M876A_ZET6221_RUISHI\n");
		}
		else
		{
			DEBUG_PROBE("use defualt\n");
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN[0]);
			
			DEBUG_PROBE("CTPM_FW_M876A_ZET6221_QIUTIAN\n");
		}
	}else {
			DEBUG_PROBE("Y do not catch  Y is:%d\n",y);
			DEBUG_PROBE("use defualt\n");
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN[0]);		
			DEBUG_PROBE("CTPM_FW_M876A_ZET6221_QIUTIAN\n");
	}
	ResolutionX=960;
	ResolutionY=640;
	DEBUG_PROBE("ctp mach %s\n",MachName);
	}	
	else if(!strcmp("M828-A13", MachName))
	{
		fw_zet6221_config_p = CTPM_FW_M828A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M828A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M828A_ZET6221_QIUTIAN[0]);
		ResolutionX=960;
		ResolutionY=640;
		DEBUG_PROBE("ctp mach %s\n",MachName);
	}
	else if(!strcmp("MD7008-A13", MachName))
	{
		fw_zet6221_config_p = CTPM_FW_MD7008A_ZET6221_HANGQING;
		FW_ZET6221_LEN = sizeof(CTPM_FW_MD7008A_ZET6221_HANGQING)/sizeof(CTPM_FW_MD7008A_ZET6221_HANGQING[0]);
		ResolutionX=960;
		ResolutionY=640;
		ctp_hold = gpio_request_ex("ctp_para", "ctp_hold");
		if(!ctp_hold) {
		DEBUG_PROBE("ctp_hold request gpio fail!\n");
	   // goto exit_gpio_int_request_failed;
	}
	
		DEBUG_PROBE("ctp mach %s\n",MachName);
	}
	else if(!strcmp("T720-A13", MachName))
	{
		fw_zet6221_config_p = CTPM_FW_T720A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_T720A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_T720A_ZET6221_QIUTIAN[0]);
		ResolutionX=960;
		ResolutionY=640;
		DEBUG_PROBE("ctp mach %s\n",MachName);
	}
	else if(!strcmp("U7-A13", MachName))
	{
		
		if(y>1850&&y<2200){
			DEBUG_PROBE("Y catch\n");
		if(x>350&&x<770)
		{
			fw_zet6221_config_p = CTPM_FW_U7A_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_U7A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_U7A_ZET6221_QIUTIAN[0]);
			DEBUG_PROBE("CTPM_FW_U7A_ZET6221_QIUTIAN\n");
		}
		else if(x>2065&&x<2475)
		{
			fw_zet6221_config_p = CTPM_FW_U7A_ZET6221_RUISHI;
			FW_ZET6221_LEN = sizeof(CTPM_FW_U7A_ZET6221_RUISHI)/sizeof(CTPM_FW_U7A_ZET6221_RUISHI[0]);
			DEBUG_PROBE("CTPM_FW_U7A_ZET6221_RUISHI\n");
		}
		else if(x>2800&&x<3200)
		{
			fw_zet6221_config_p = CTPM_FW_U7A_ZET6221_CHENGXIN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_U7A_ZET6221_CHENGXIN)/sizeof(CTPM_FW_U7A_ZET6221_CHENGXIN[0]);			
			DEBUG_PROBE("CTPM_FW_U7A_ZET6221_CHENGXIN\n");
		}
		else
		{
			DEBUG_PROBE("use defualt\n");
			fw_zet6221_config_p = CTPM_FW_U7A_ZET6221_CHENGXIN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_U7A_ZET6221_CHENGXIN)/sizeof(CTPM_FW_U7A_ZET6221_CHENGXIN[0]);
			DEBUG_PROBE("CTPM_FW_U7A_ZET6221_CHENGXIN\n");
		}
	}else {
			DEBUG_PROBE("Y do not catch  Y is:%d\n",y);
			DEBUG_PROBE("use defualt\n");
			fw_zet6221_config_p = CTPM_FW_U7A_ZET6221_CHENGXIN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_U7A_ZET6221_CHENGXIN)/sizeof(CTPM_FW_U7A_ZET6221_CHENGXIN[0]);
			DEBUG_PROBE("CTPM_FW_U7A_ZET6221_CHENGXIN\n");
	}
		
		ResolutionX=1280;
		ResolutionY=768;
		DEBUG_PROBE("ctp mach %s\n",MachName);
		DEBUG_PROBE("ResolutionX:%d ResolutionY:%d\n",ResolutionX,ResolutionY);		

		//goto script_parser_fetch_err;
	}
	else if(!strcmp("L7-A13", MachName))
	{
		
		fw_zet6221_config_p = CTPM_FW_L7A_ZET6221_LENHENG;
		FW_ZET6221_LEN = sizeof(CTPM_FW_L7A_ZET6221_LENHENG)/sizeof(CTPM_FW_L7A_ZET6221_LENHENG[0]);
		ResolutionX=960;
		ResolutionY=640;
		DEBUG_PROBE("ctp mach %s\n",MachName);
		DEBUG_PROBE("ResolutionX:%d ResolutionY:%d\n",ResolutionX,ResolutionY);
		//goto script_parser_fetch_err;
	}
	else if(!strcmp("T801-A13", MachName))
	{
		
		fw_zet6221_config_p = CTPM_FW_T801A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_T801A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_T801A_ZET6221_QIUTIAN[0]);
		ResolutionX=1024;
		ResolutionY=768;
		DEBUG_PROBE("ctp mach %s\n",MachName);
		//goto script_parser_fetch_err;
	}
	else if(!strcmp("M858-A13", MachName))
	{
		int ctp_hold_pin=-1;
		
		
		ResolutionX=960;
		ResolutionY=640;
		DEBUG_PROBE("ctp mach %s\n",MachName);
		
		ctp_hold = gpio_request_ex("ctp_para", "ctp_hold");
		if(!ctp_hold) {
		DEBUG_PROBE("ctp_hold request gpio fail!\n");
		}
		ctp_hold_pin=gpio_read_one_pin_value(ctp_hold,"ctp_hold");
		//高电平
		if(ctp_hold_pin){	
		fw_zet6221_config_p = CTPM_FW_M858A_ZET6221_PINGBO;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M858A_ZET6221_PINGBO)/sizeof(CTPM_FW_M858A_ZET6221_PINGBO[0]);
		DEBUG_PROBE("ctp_hold_pin high\n");
		}
		//低电平
		else if(!ctp_hold_pin){
		fw_zet6221_config_p = CTPM_FW_M858A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M858A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M858A_ZET6221_QIUTIAN[0]);
		DEBUG_PROBE("ctp_hold_pin low\n");	
		}
		//悬空
		else{
		fw_zet6221_config_p = CTPM_FW_M858A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M858A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M858A_ZET6221_QIUTIAN[0]);
		}
		//释放
		if(ctp_hold){
		gpio_release(ctp_hold, 2);
		ctp_hold=0;
		}
		
		//goto script_parser_fetch_err;
	}
	else
	{
		fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_JIAGUANG;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_JIAGUANG)/sizeof(CTPM_FW_M876A_ZET6221_JIAGUANG[0]);
		ResolutionX=960;
		ResolutionY=640;
		DEBUG_PROBE("use default M876 \n");
	}
	#endif
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr2", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err.ctp_twi_addr... \n", name);
		goto script_parser_fetch_err;
	}
	
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	DEBUG_PROBE("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	//printk("%s: after: ctp_twi_addr is 0x%x, u32_dirty_addr_buf: 0x%hx. u32_dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u32_dirty_addr_buf[0],u32_dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. ctp_twi_id...\n", name);
		goto script_parser_fetch_err;
	}
	DEBUG_PROBE("%s: ctp_twi_id is %d. \n", __func__, twi_id);	
	
	
	
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	DEBUG_PROBE("%s: revert_x_flag = %d. \n", __func__, revert_x_flag);
  
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	//revert_y_flag=0;
	DEBUG_PROBE("%s: revert_y_flag = %d. \n", __func__, revert_y_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1)){
		pr_err("ft5x_ts: script_parser_fetch err. \n");
		goto script_parser_fetch_err;
	}
	DEBUG_PROBE("%s: exchange_x_y_flag = %d. \n", __func__, exchange_x_y_flag);
	return 0;
	script_parser_fetch_err:
	printk("=========script_parser_fetch_err============\n");
	return ret;
}


//驱动加载函数
static int __devinit zet6221_init(void)
{
	int ret = 0;
	if(input_ctp_module_flg){
		DEBUG_PROBE("NOTE:other touch modules already insmod...\n");
		return -1;
	}
	
	
	if(ctp_fetch_sysconfig_para()){
			printk("%s: err.\n", __func__);
			return -1;
	}
	
	
	if(0 != ctp_init_platform_resource()){
		DEBUG_PROBE("%s:ctp_ops.init_platform_resource err. \n", __func__);   
		return -1; 
	}
	ctp_wakeup();
	zet6221_driver.detect = ctp_detect;	
   
	ret=i2c_add_driver(&zet6221_driver);


	return ret; 
}

//驱动卸载函数
static void __exit zet6221_exit(void)
{
	i2c_del_driver(&zet6221_driver);	
}

late_initcall(zet6221_init);
module_exit(zet6221_exit);

MODULE_DESCRIPTION("zet6221 Touchscreen Driver");
MODULE_LICENSE("GPL v2");

