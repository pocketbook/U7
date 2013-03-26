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
int ChargeChange=0;//add for charge mode

#define ZET6221_PEN_DOWN 0
#define ZET6221_PEN_RELEASE 1
static int status[5]={0};
static int status_for_change[5]={0};
struct zet6221_data {
//	u8 bad_data;
//    u8 irq_is_disable;
//	int retry;
//	int panel_type;
	char phys[32];	
	int finger;
	struct i2c_client *client;
	struct input_dev *input_dev;
	uint8_t use_irq;
//	uint8_t use_shutdown;
//	uint32_t gpio_shutdown;
	uint32_t gpio_irq;
//	uint32_t screen_width;
//	uint32_t screen_height;
	
	uint16_t touch_x[MAX_FINGER_NUM];
	uint16_t touch_y[MAX_FINGER_NUM];
//	struct ts_event		event;
	struct hrtimer timer;
	struct work_struct  work;
	struct work_struct  work1;
	//int (*power)(struct zet6221_data * ts, int on);
#ifdef CONFIG_HAS_EARLYSUSPEND	
    struct early_suspend early_suspend;
#endif
};


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
//static int gpio_io_hdle = 0;
//static int gpio_power_hdle = 0;

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
	#include "zet6221_config/m876a_zet6221_qiutian.i"
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
static unsigned char CTPM_FW_MD7008A_ZET6221_QIUTIAN[] __initdata = 
{
	#include "zet6221_config/md7008a_zet6221_qiutian.i"
};
static unsigned char CTPM_FW_T801A_ZET6221_QIUTIAN[] __initdata = 
{
#include "zet6221_config/t801_zet6221_pb.i"
};
static unsigned char CTPM_FW_U7A_ZET6221_QIUTIAN[] __initdata = 
{
	#include"zet6221_config/u7_zet6221_qiutian.i"
//#include "zet6221_config/u7_zet6221_chengxin.i"
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
static unsigned int FW_ZET6221_LEN = 0;
//static struct i2c_client *this_client;
#define ZET6221_UPGRADE_FW	 
#define ZET6221_TPINFO 
#ifdef  ZET6221_UPGRADE_FW
	#include "zet6221_downloader.c"
#endif
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
/**********************************************************************	
本程序中I2C通信方式为：
	7bit从机地址｜读写位 + buf（数据地址+读写数据）
	 --------------------------------------------------------------------
	｜  从机地址   ｜ buf[0](数据地址) | buf[1]~buf[MAX-1](写入或读取到的数据)  |
	 --------------------------------------------------------------------
	移植前请根据自身主控格式修改！！
***********************************************************************/

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
	//printk("%s is running ==========\n",__FUNCTION__);
	ret=zet6221_write_bytes(ts->client, ts_write_cmd, 1);
	//printk("%s:%d\n",__func__,ret);
}
static void ctp_wakeup(void)
{
	
		pr_info("%s. \n", __func__);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(10);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(20);
	
	return;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void zet6221_early_suspend(struct early_suspend *h)
{
	int ret;
	//int reg_val;
	
		struct zet6221_data *ts = container_of(h, struct zet6221_data, early_suspend);
    //struct i2c_client * client = ts->client;
    #ifdef PRINT_SUSPEND_INFO
        printk("enter earlysuspend: zet6221_suspend. \n");
    #endif    
    
   hrtimer_cancel(&ts->timer);
 // hrtimer_cancel(&ts->timer); 
 //disable_irq(ts->gpio_irq);
	ret = cancel_work_sync(&ts->work);	
	cancel_work_sync(&ts->work1);	
	

	//disable irq
	 /*reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
   reg_val &=~(1<<IRQ_EINT11);
   writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);*/
   disable_tp_irq();
		
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
    hrtimer_start(&ts->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
    if(usb_valid_flag == 1) {
			ts_write_charge_enable_cmd(ts);
			
		}else if(usb_valid_flag == 0)
		{
			ts_write_charge_disable_cmd(ts);
		}
    /*
    reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
    reg_val |=(1<<IRQ_EINT11);
    writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
    */
    enable_tp_irq();
		
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
        //disable_irq(ts->gpio_irq);
  
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

        //enable_irq(ts->gpio_irq);
	return 0;
}
#endif
#endif

static enum hrtimer_restart zet6221_timer(struct hrtimer *timer)
{	
	struct zet6221_data *ts = container_of(timer, struct zet6221_data, timer);
	queue_work(zet6221_wq1, &ts->work1);
	hrtimer_start(&ts->timer, ktime_set(0, MicroTimeTInterupt*40*2), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}




static void write_cmd_work(struct work_struct *work1)
{
	struct zet6221_data *ts = container_of(work1, struct zet6221_data, work1);
	
	//printk("Pendown:%d\n",Pendown);
	if(ZET6221_PEN_DOWN==Pendown)
	{		
		return ;
	}

	if((usb_valid_flag != ChargeChange))
		//||Pendown==ZET6221_PEN_RELEASE)
	//if(ZET6221_PEN_RELEASE==Pendown)
	{	
		//Pendown++;
		if(usb_valid_flag == 1) 
		{			
			ts_write_charge_enable_cmd(ts);								
		}else if(usb_valid_flag == 0)
		{			
			ts_write_charge_disable_cmd(ts);			
		}
		
		printk("--%s--:%d\n",__func__,usb_valid_flag);
		ChargeChange = usb_valid_flag;
		//PendownStatue = Pendown;
		//printk("finger up ChargeChange :%d\n",ChargeChange);
		//printk("PendownStatue:%d\n",PendownStatue);
	}

}

#if 0
u8 zet6221_ts_get_xy_from_panel(struct i2c_client *client, u32 *x, u32 *y, u32 *z, u32 *pr, u32 *ky)
{
	u8  ts_data[70];
	int ret;
	int i;
	
	memset(ts_data,0,70);

	ret=zet6221_i2c_read_tsdata(client, ts_data, bufLength);
	
	*pr = ts_data[1];
	//*pr = (*pr << 8) | ts_data[2];
		
	for(i=0;i<FingerNum;i++)
	{
		x[i]=(u8)((ts_data[3+4*i])>>4)*256 + (u8)ts_data[(3+4*i)+1];
		y[i]=(u8)((ts_data[3+4*i]) & 0x0f)*256 + (u8)ts_data[(3+4*i)+2];
		//z[i]=(u8)((ts_data[(3+4*i)+3]) & 0x0f);
	}
		
	//if key enable
	if(KeyNum > 0)
		*ky = ts_data[3+4*FingerNum];

	return ts_data[0];
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
//static void zet6221_ts_work(struct work_struct *_work)
#if 0
static inline  int  zet6221_read_values(struct zet6221_data *ts)
{
	u32 x[MAX_FINGER_NUM], y[MAX_FINGER_NUM], z[MAX_FINGER_NUM], pr, ky, points;
	u32 px,py,pz;
	u8 ret;
	u8 pressure;
	int i;

	if (bufLength == 0)
	{
		return;
	}

	ret = zet6221_ts_get_xy_from_panel(ts->client, x, y, z, &pr, &ky);
	//printk("ret:%x",ret);
	if(ret == 0x3C)
	{

	//	DPRINTK( "x1= %d, y1= %d x2= %d, y2= %d [PR] = %d [KY] = %d\n", x[0], y[0], x[1], y[1], pr, ky);
		
		points = pr;
		//printk("points:%x\n",points);
		#if defined(TRANSLATE_ENABLE)
		touch_coordinate_traslating(x, y, points);
		#endif

		for(i=0;i<FingerNum;i++){
			pressure = (points & ((0x80)>>i));//(points >> (MAX_FINGER_NUM-i-1)) & 0x1;
			//printk("pressure:%d\n",pressure);
			if(pressure)
			{
				px = x[i];
				py = y[i];
				//pz = z[i];
				//printk("px:%d py:%d pz:%d\n",px,py,pz);
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
	    		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, P_MAX);
	    		//input_report_abs(ts->input, ABS_MT_POSITION_X, x[i]);
	    		//input_report_abs(ts->input, ABS_MT_POSITION_Y, y[i]);
	    		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, px);
	    		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, py);
	    		input_mt_sync(ts->input_dev);
	    		printk("tp i:%d\n",i);

			}else
			{
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_mt_sync(ts->input_dev);
				
			}
		}

		input_sync(ts->input_dev);		
	}
	else
		printk("ret !=3C\n");

}
#else

static inline  int  zet6221_read_values(struct zet6221_data *ts)
{
	int err = 0;
	u8 read_buf[25] = {0};
	
	int  i = 0;
	ts->finger=0;
	
	Ha_debug("before zet6221_read_bytes ...\n");
	err =zet6221_read_bytes(ts->client, read_buf, bufLength);//19
		if (err < 0)
		{
			Ha_debug("read coorlen failed\n");
			goto err_exit;
		}
		if(ctp_hold)
	{
		//printk("ctp_hold:%d\n",gpio_read_one_pin_value(ctp_hold,"ctp_hold"));
		if(!gpio_read_one_pin_value(ctp_hold,"ctp_hold")){	
		return ZET6221_PEN_RELEASE;	
		}
	}
	for(i = 0; i < FingerNum; i++) {
		if (read_buf[1] & ((0x80)>>i)){
			//ts->touch_x[ts->finger]=((read_buf[3+4*i])>>4)*256 + (u8)read_buf[(3+4*i)+1];
			ts->touch_x[i]=((read_buf[3+4*i])>>4)*256 + (u8)read_buf[(3+4*i)+1];
			//ts->touch_y[ts->finger]=((read_buf[3+4*i]) & 0x0f)*256 + (u8)read_buf[(3+4*i)+2];
			ts->touch_y[i]=((read_buf[3+4*i]) & 0x0f)*256 + (u8)read_buf[(3+4*i)+2];
			ts->finger ++;
			status[i]=1;
		}
		else{
			ts->touch_x[i]=0;
			ts->touch_y[i]=0;
			status[i]=0;
		}
	}
	if(ts->finger == 0)
		{
			//no_touch_flag++;
			//printk("no_touch_flag:%d\n",no_touch_flag);
			if(no_touch_flag<1)
			{
					no_touch_flag++;
					//pendown = ZET6221_PEN_DOWN;
					return ZET6221_PEN_RELEASE;
			}else
			{
					goto no_touch;	
			}

		}
		
	//printk("x=%d y=%d\n",ts->touch_x[0],ts->touch_y[0]);
	no_touch_flag=0;
	//printk("revert_x_flag :%d revert_y_flag:%d\n",revert_x_flag,revert_y_flag);
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
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X,  ts->touch_x[i]);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  ts->touch_y[i]);
					input_mt_sync(ts->input_dev);
					//printk("ts->touch_x[%d]=%d,ts->touch_y[%d]=%d\n",i,ts->touch_x[i],i,ts->touch_y[i]);
					//printk("i:%d %d\n",i,status[i]);
				}
			else
				{
						if(status_for_change[i]!=status[i]){
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_mt_sync(ts->input_dev);
						//printk("i:%d %d\n",i,status[i]);
					}
				}
			status_for_change[i]=status[i];
	}


	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
	
	//pendown = ZET6221_PEN_DOWN;
	
	return ZET6221_PEN_DOWN;
	
no_touch:
	
	//if(ZET6221_PEN_DOWN == pendown)
	{
		
		//pendown = ZET6221_PEN_RELEASE;
		
//		for(i=0;i<MAX_FINGER_NUM;i++){
//			ts->touch_x[i] = 0;
//			ts->touch_y[i] = 0;			
//		}

    input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev, BTN_2, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);
		no_touch_flag=0;
		
		//hrtimer_cancel(&ts->timer);
	}

err_exit:

	return ZET6221_PEN_RELEASE;
}
#endif
static void zet6221_work_func(struct work_struct *work)
{

	struct zet6221_data *ts = container_of(work, struct zet6221_data, work);
	
#ifdef USE_TIME
			if(!gpio_read_one_pin_value(gpio_int_hdle, "ctp_int_port"))
#endif
			Pendown=zet6221_read_values(ts);
			
			//printk("pendown:%d\n",pendown);
}

/*******************************************************	
功能：
	中断响应函数
	由中断触发，调度触摸屏处理函数运行
********************************************************/
static irqreturn_t zet6221_irq_handler(int irq, void *dev_id)
{
//printk("%s\n",__FUNCTION__);
	struct zet6221_data *ts = dev_id;	
	int reg_val;	
	//printk("==========------%s Interrupt-----==========\n", __func__); 

	
	//clear the IRQ_EINT11 interrupt pending
	
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
     
	if(reg_val & (1 << (IRQ_EINT_USED_ZET6221)))
	{	
		//printk("==IRQ_EINT11=\n");
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
	printk(" zet6221 touch =======%s=========.\n", __func__);
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
		printk("==CTP_IRQ_NO=\n");              
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
	printk("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	printk(" INTERRUPT CONFIG\n");
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	printk("reg_num:%d reg_addr:%d\n",reg_num,reg_addr);
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);
                                                               
	ctp_clear_penirq();
             
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
//Test i2c to check device. Before it SHUTDOWN port Must be low state 30ms or more.
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

	pr_info("===============================Zet6221 Probe===========================\n");

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
		
	
	
#define TEST_I2C_TRANSFER
#ifdef TEST_I2C_TRANSFER
	//TODO: used to set speed of i2c transfer. Should be change as your paltform.
	printk("Begin ZET6221 i2c test\n");
	ret = zet6221_i2c_test(client);
	if(!ret) {
		printk("error: I2C connection might be something wrong!\n");
		goto err_i2c_failed;
	}
	printk("===== ZET6221 i2c test ok=======\n");
#endif

#ifdef ZET6221_UPGRADE_FW
zet_download:
	#define RETRY_DOWNLOAD_CNT  2
	
	zet6221_downloader(client);
	cnt++;
#endif
	
#ifdef ZET6221_TPINFO
	//udelay(100);
	#define REPORT_POLLING_TIME 5
	do {	
		ctp_wakeup();
		if(zet6221_ts_get_report_mode_t(client)==0)  //get IC info by delay 
		{
			Ha_debug("read infor failed!\n");
			//ResolutionX = SCREEN_MAX_WIDTH;
			//ResolutionY = SCREEN_MAX_HEIGHT;
			FingerNum = MAX_FINGER_NUM;
			bufLength  = 3 + 4 * FingerNum;		
		} else {
			//bootloader
			if(zet6221_ts_version()!=0)
				break;
		}	
		count++;
	}while(count < REPORT_POLLING_TIME);
	
	//bootloader
	if(count == REPORT_POLLING_TIME&& cnt < RETRY_DOWNLOAD_CNT)
		goto zet_download;
	
#else
	//ResolutionX = SCREEN_MAX_WIDTH;
	//ResolutionY = SCREEN_MAX_HEIGHT;
	FingerNum = MAX_FINGER_NUM;
	//KeyNum = KEY_NUMBER;
	if(KeyNum==0)
		bufLength  = 3 + 4 * FingerNum;
	else
		bufLength  = 3 + 4 * FingerNum + 1;
#endif
	
	ts->gpio_irq = SW_INT_IRQNO_PIO;
	err = ctp_set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);

	if(0 != err){
		printk("%s:ctp_ops.set_irq_mode err. \n", __func__);
		goto exit_set_irq_mode;
	}
	
/* ++ write_cmd */
	INIT_WORK(&ts->work1, write_cmd_work);
	zet6221_wq1 = create_singlethread_workqueue("zet6221_wq1"); // workqueue
	if (!zet6221_wq1) {	
		printk(KERN_ALERT "Creat %s workqueue failed.\n", ZET6221_I2C_NAME);	
		goto exit_create_singlethread;
		
	}
/* ++ write_cmd */
	zet6221_wq = create_singlethread_workqueue("zet6221_wq");
  if (!zet6221_wq) {
    	printk(KERN_ALERT "Creat %s workqueue failed.\n", ZET6221_I2C_NAME);
   goto 	exit_create_singlethread;
    		
    }
	INIT_WORK(&ts->work, zet6221_work_func);
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
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_HEIGHT, 0, 0);	
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MAX_FINGER_NUM, 0, 0);
	printk("tp-size:wxh(%dx%d)",SCREEN_MAX_WIDTH,SCREEN_MAX_HEIGHT);	
#endif	
	//ts->bad_data = 0;
#ifdef FOR_TSLIB_TEST
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
#endif

	sprintf(ts->phys, "input/zet6221");
	ts->input_dev->name = ZET6221_I2C_NAME;
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
	
	flush_workqueue(zet6221_wq);	
	flush_workqueue(zet6221_wq1);
	msleep(30);	
	//input_ctp_module_flg =1;
	
	
#ifdef CONFIG_HAS_EARLYSUSPEND	
    printk("==register_early_suspend =\n");	
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;	
    ts->early_suspend.suspend = zet6221_early_suspend;
    ts->early_suspend.resume	= zet6221_early_resume;	
    register_early_suspend(&ts->early_suspend);
#endif


		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = zet6221_timer;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		
	err =  request_irq(ts->gpio_irq , zet6221_irq_handler, IRQF_TRIGGER_FALLING|IRQF_SHARED|IRQF_PROBE_SHARED, client->name, ts);

	if (err < 0) {
		pr_info( "zet6221_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	
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
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	hrtimer_cancel(&ts->timer);
	if(ts->input_dev)
		kfree(ts->input_dev);
	kfree(ts);
	if(input_ctp_module_flg==1)
	input_ctp_module_flg=0;
	return 0;
}

static void zet6221_shutdown(struct i2c_client *client)
{
	 //gpio_write_one_pin_value(gpio_power_hdle, 0, "ctp_power_port");
	 Ha_debug("zet6221_shutdown\n");
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
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
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
#define IRQ_TP                 (29)
#define TP_BASSADDRESS         (0xf1c25000)
#define TP_CTRL0               (0x00)
#define TP_CTRL1               (0x04)
#define TP_CTRL2               (0x08)
#define TP_CTRL3               (0x0c)
#define TP_INT_FIFOC           (0x10)
#define TP_INT_FIFOS           (0x14)
#define TP_TPR                 (0x18)
#define TP_CDAT                (0x1c)
#define TEMP_DATA              (0x20)
#define TP_DATA                (0x24)


#define ADC_FIRST_DLY          (0x1<<24)
#define ADC_FIRST_DLY_MODE     (0x1<<23) 
#define ADC_CLK_SELECT         (0x0<<22)
#define ADC_CLK_DIVIDER        (0x2<<20)    
//#define CLK                    (6)
#define CLK                    (7)
#define FS_DIV                 (CLK<<16)
#define ACQ                    (0x3f)
#define T_ACQ                  (ACQ)

#define STYLUS_UP_DEBOUNCE     (0<<12)
#define STYLUS_UP_DEBOUCE_EN   (0<<9)
#define TOUCH_PAN_CALI_EN      (1<<6)
#define TP_DUAL_EN             (1<<5)
#define TP_MODE_EN             (1<<4)
#define TP_ADC_SELECT          (0<<3)
#define ADC_CHAN_SELECT        (0)

//#define TP_SENSITIVE_ADJUST    (0xf<<28)
#define TP_SENSITIVE_ADJUST    (0xf<<28)       //mark by young for test angda 5" 0xc
#define TP_MODE_SELECT         (0x1<<26)
#define PRE_MEA_EN             (0x1<<24)
#define PRE_MEA_THRE_CNT       (0x1f40<<0)         //0x1f40


#define FILTER_EN              (1<<2)
#define FILTER_TYPE            (0x01<<0)

#define TP_DATA_IRQ_EN         (1<<16)
#define TP_DATA_XY_CHANGE      (0<<13)       //tp_exchange_x_y
#define TP_FIFO_TRIG_LEVEL     (3<<8)
#define TP_FIFO_FLUSH          (1<<4)
#define TP_UP_IRQ_EN           (1<<1)
#define TP_DOWN_IRQ_EN         (1<<0)

#define FIFO_DATA_PENDING      (1<<16)
#define TP_UP_PENDING          (1<<1)
#define TP_DOWN_PENDING        (1<<0)

#define SINGLE_TOUCH_MODE      (1)
#define CHANGING_TO_DOUBLE_TOUCH_MODE (2)
#define DOUBLE_TOUCH_MODE      (3)
#define UP_TOUCH_MODE           (4)

#define SINGLE_CNT_LIMIT       (40)
#define DOUBLE_CNT_LIMIT       (2)
#define UP_TO_SINGLE_CNT_LIMIT (10)
                              
#define TPDATA_MASK            (0xfff)
#define FILTER_NOISE_LOWER_LIMIT  (2)
#define MAX_DELTA_X             (700-100)     
#define MAX_DELTA_Y             (1200-200)
#define X_TURN_POINT            (330)         
#define X_COMPENSATE            (4*X_TURN_POINT)
#define Y_TURN_POINT            (260)         
#define Y_COMPENSATE            (2*Y_TURN_POINT)
static void  tp_init(void)
{
  
    #ifdef TP_FREQ_DEBUG
    writel(0x0028001f, TP_BASSADDRESS + TP_CTRL0);
    #else
    writel(ADC_CLK_DIVIDER|FS_DIV|T_ACQ, TP_BASSADDRESS + TP_CTRL0);	   
    #endif
    //TP_CTRL2: 0xc4000000
    writel(TP_SENSITIVE_ADJUST|TP_MODE_SELECT,TP_BASSADDRESS + TP_CTRL2); 
    

    //TP_CTRL3: 0x05
    #ifdef TP_FREQ_DEBUG
    writel(0x06, TP_BASSADDRESS + TP_CTRL3);
    #else
    writel(FILTER_EN|FILTER_TYPE,TP_BASSADDRESS + TP_CTRL3);
    #endif
    
    #ifdef TP_TEMP_DEBUG
        //TP_INT_FIFOC: 0x00010313
        writel(TP_DATA_IRQ_EN|TP_FIFO_TRIG_LEVEL|TP_FIFO_FLUSH|TP_UP_IRQ_EN|TP_DOWN_IRQ_EN|0x40000, TP_BASSADDRESS + TP_INT_FIFOC);
        writel(0x10fff, TP_BASSADDRESS + TP_TPR);
    #else
        //TP_INT_FIFOC: 0x00010313
        writel(TP_DATA_IRQ_EN|TP_FIFO_TRIG_LEVEL|TP_FIFO_FLUSH|TP_UP_IRQ_EN|TP_DOWN_IRQ_EN, TP_BASSADDRESS + TP_INT_FIFOC);
    #endif
    writel(TP_DATA_XY_CHANGE|STYLUS_UP_DEBOUNCE|STYLUS_UP_DEBOUCE_EN|TP_DUAL_EN|TP_MODE_EN,TP_BASSADDRESS + TP_CTRL1);
    //return (0);
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
	
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
	
	Ha_debug("zet6221_init\n");

		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("product", "machine", (int *)(&MachName), &type, sizeof(MachName)/sizeof(int))){
		pr_err("%s: line: %d script_parser_fetch err.,%s\n", __func__, __LINE__,MachName);
			
		}
		printk("tp match machine:%s\n",MachName);

    if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)){
        Ha_debug("ZET6221_ts: script_parser_fetch err. \n");
        goto script_parser_fetch_err;
    }
    if(1 != ctp_used){
        Ha_debug("ZET6221_ts: ctp_unused. \n");
        return -1;
    }

    if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name2", (int *)(&name), &type, sizeof(name)/sizeof(int))){
            Ha_debug("ZET6221_ts_init: script_parser_fetch err. \n");
            goto script_parser_fetch_err;
    }
    if(strcmp(ZET6221_I2C_NAME, name)){
        Ha_debug("ZET6221_ts_init: name %s does not match ZET6221_I2C_NAME %s. \n", name,ZET6221_I2C_NAME);
        return -1;
    }

	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_tp_name2", (int *)(&tp_name), &type, sizeof(tp_name)/sizeof(int))){
			pr_err("%s: script_parser_fetch err. \n", __func__);
			goto script_parser_fetch_err;
	}
/**********************/		
#if 1
#if 0
#define IRQ_TP                 (29)
#define TP_BASSADDRESS         (0xf1c25000)
#define TP_CTRL0               (0x00)
#define TP_CTRL1               (0x04)
#define TP_CTRL2               (0x08)
#define TP_CTRL3               (0x0c)
#define TP_INT_FIFOC           (0x10)
#define TP_INT_FIFOS           (0x14)
#define TP_DATA                (0x24)
#define ADC_CLK_DIVIDER        (0x2<<20)
#define CLK                    (7)
#define FS_DIV								 (CLK<<16)
#define TP_DATA_IRQ_EN         (1<<16)
#define TP_FIFO_TRIG_LEVEL     (3<<8)
#define FIFO_DATA_PENDING      (1<<16)	
#define TP_MODE_EN             (1<<4)
#define TP_ADC_SELECT          (1<<3)
#define ADC_CHAN_SELECT        (0)
#endif


//{
	
	unsigned int reg_val,breaktime=0;
	int x=0,y=0;
	int reg_fifoc;
	int trytime=0;	
	tp_init();	
	while(1)
	{
		
		reg_val  = readl(TP_BASSADDRESS + TP_INT_FIFOS);
		if(reg_val&FIFO_DATA_PENDING)
		{
			x+=readl(TP_BASSADDRESS + TP_DATA);
      //printk("x:%d\n",x);               
      y+=readl(TP_BASSADDRESS + TP_DATA); 
      //printk("y:%d\n",y);     
      readl(TP_BASSADDRESS + TP_DATA);
      //printk("dx:%d\n",dx);  
      readl(TP_BASSADDRESS + TP_DATA); 
      //printk("dy:%d\n",dy); 
      //flush fifo
      reg_fifoc = readl(TP_BASSADDRESS+TP_INT_FIFOC);
      reg_fifoc |= TP_FIFO_FLUSH;
      writel(reg_fifoc, TP_BASSADDRESS+TP_INT_FIFOC);       
      writel(reg_val&FIFO_DATA_PENDING,TP_BASSADDRESS + TP_INT_FIFOS);
      breaktime++;
		}
				
		trytime++;
		mdelay(1);
		//printk("trytime:%d\n",trytime);
		//schedule_timeout(HZ);
		//schedule_timeout_interruptible(HZ);
		if(breaktime>=10)
			break;
		if(trytime>=1000)
		{
			printk("trytime>=%d\n",trytime);
			break;
		}
			
	}
	if(breaktime>0){
	x=x/breaktime;
	y=y/breaktime;
	}
	printk("x:%d y:%d\n",x,y);
	
//}
#endif
/**********************/
	
	
	#if 1
	if (!strcmp("M876-A13", MachName))
	{
		#if 0
		if(!strcmp("m876a13_zet6221_ruishi", tp_name))
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_RUISHI;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_RUISHI)/sizeof(CTPM_FW_M876A_ZET6221_RUISHI[0]);
			ResolutionX=960;
			ResolutionY=640;
			printk("ctp mach %s\n",MachName);
			printk("tp name :%s\n",tp_name);
		}

		else if(!strcmp("m876a13_zet6221_jiaguang", tp_name))
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_JIAGUANG;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_JIAGUANG)/sizeof(CTPM_FW_M876A_ZET6221_JIAGUANG[0]);
			ResolutionX=960;
			ResolutionY=640;
			printk("ctp mach %s\n",MachName);
			printk("tp name :%s\n",tp_name);
		}
		else if(!strcmp("m876a13_zet6221_qiutian", tp_name))
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN[0]);
			ResolutionX=960;
			ResolutionY=640;
			printk("ctp mach %s\n",MachName);
			printk("tp name :%s\n",tp_name);
		}
		else
		{
		fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_JIAGUANG;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_JIAGUANG)/sizeof(CTPM_FW_M876A_ZET6221_JIAGUANG[0]);
		ResolutionX=960;
		ResolutionY=640;
		printk("ctp mach %s\n",MachName);
		}
		#endif
		if(y>1850&&y<2200){
			printk("Y catch\n");
		if(x>350&&x<770)
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN[0]);
			ResolutionX=960;
			ResolutionY=640;
			printk("ctp mach %s\n",MachName);
			printk("CTPM_FW_M876A_ZET6221_QIUTIAN\n");
		}
		else if(x>1160&&x<1570)
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_PINGBO;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_PINGBO)/sizeof(CTPM_FW_M876A_ZET6221_PINGBO[0]);
			ResolutionX=960;
			ResolutionY=640;
			printk("ctp mach %s\n",MachName);
			printk("CTPM_FW_M876A_ZET6221_PINGBO\n");
		}
		else if(x>2065&&x<2475)
		{
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_RUISHI;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_RUISHI)/sizeof(CTPM_FW_M876A_ZET6221_RUISHI[0]);
			ResolutionX=960;
			ResolutionY=640;
			printk("ctp mach %s\n",MachName);
			printk("CTPM_FW_M876A_ZET6221_RUISHI\n");
		}
		else
		{
			printk("use defualt\n");
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN[0]);
			ResolutionX=960;
			ResolutionY=640;
			printk("ctp mach %s\n",MachName);
			printk("CTPM_FW_M876A_ZET6221_QIUTIAN\n");
		}
	}else {
			printk("Y do not catch  Y is:%d\n",y);
			printk("use defualt\n");
			fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_QIUTIAN;
			FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M876A_ZET6221_QIUTIAN[0]);
			ResolutionX=960;
			ResolutionY=640;
			printk("ctp mach %s\n",MachName);
			printk("CTPM_FW_M876A_ZET6221_QIUTIAN\n");
	}
	}	
	else if(!strcmp("M828-A13", MachName))
	{
		fw_zet6221_config_p = CTPM_FW_M828A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M828A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M828A_ZET6221_QIUTIAN[0]);
		ResolutionX=960;
		ResolutionY=640;
		printk("ctp mach %s\n",MachName);
	}
	else if(!strcmp("MD7008-A13", MachName))
	{
		fw_zet6221_config_p = CTPM_FW_MD7008A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_MD7008A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_MD7008A_ZET6221_QIUTIAN[0]);
		ResolutionX=960;
		ResolutionY=640;
		ctp_hold = gpio_request_ex("ctp_para", "ctp_hold");
		if(!ctp_hold) {
		printk("ctp_hold request gpio fail!\n");
	   // goto exit_gpio_int_request_failed;
	}
	
		
		
			
	
		printk("ctp mach %s\n",MachName);
	}
	else if(!strcmp("T720-A13", MachName))
	{
		fw_zet6221_config_p = CTPM_FW_T720A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_T720A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_T720A_ZET6221_QIUTIAN[0]);
		ResolutionX=960;
		ResolutionY=640;
		printk("ctp mach %s\n",MachName);
	}
	else if(!strcmp("U7-A13", MachName))
	{
		
		fw_zet6221_config_p = CTPM_FW_U7A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_U7A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_U7A_ZET6221_QIUTIAN[0]);
		ResolutionX=1280;
		ResolutionY=768;
		printk("ctp mach %s\n",MachName);
		printk("ResolutionX:%d ResolutionY:%d\n",ResolutionX,ResolutionY);
		//goto script_parser_fetch_err;
	}
	else if(!strcmp("L7-A13", MachName))
	{
		
		fw_zet6221_config_p = CTPM_FW_L7A_ZET6221_LENHENG;
		FW_ZET6221_LEN = sizeof(CTPM_FW_L7A_ZET6221_LENHENG)/sizeof(CTPM_FW_L7A_ZET6221_LENHENG[0]);
		ResolutionX=960;
		ResolutionY=640;
		printk("ctp mach %s\n",MachName);
		printk("ResolutionX:%d ResolutionY:%d\n",ResolutionX,ResolutionY);
		//goto script_parser_fetch_err;
	}
	else if(!strcmp("T801-A13", MachName))
	{
		
		fw_zet6221_config_p = CTPM_FW_T801A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_T801A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_T801A_ZET6221_QIUTIAN[0]);
		ResolutionX=1024;
		ResolutionY=768;
		printk("ctp mach %s\n",MachName);
		//goto script_parser_fetch_err;
	}
	else if(!strcmp("M858-A13", MachName))
	{
		
		fw_zet6221_config_p = CTPM_FW_M858A_ZET6221_QIUTIAN;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M858A_ZET6221_QIUTIAN)/sizeof(CTPM_FW_M858A_ZET6221_QIUTIAN[0]);
		ResolutionX=960;
		ResolutionY=640;
		printk("ctp mach %s\n",MachName);
		//goto script_parser_fetch_err;
	}
	else
	{
		fw_zet6221_config_p = CTPM_FW_M876A_ZET6221_JIAGUANG;
		FW_ZET6221_LEN = sizeof(CTPM_FW_M876A_ZET6221_JIAGUANG)/sizeof(CTPM_FW_M876A_ZET6221_JIAGUANG[0]);
		ResolutionX=960;
		ResolutionY=640;
		printk("use default M876 \n");
	}
	#endif
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr2", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
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
	
	
	
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_x_flag = %d. \n", __func__, revert_x_flag);
  
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	//revert_y_flag=0;
	pr_info("%s: revert_y_flag = %d. \n", __func__, revert_y_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1)){
		pr_err("ft5x_ts: script_parser_fetch err. \n");
		goto script_parser_fetch_err;
	}
	pr_info("%s: exchange_x_y_flag = %d. \n", __func__, exchange_x_y_flag);
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
	zet6221_driver.detect = ctp_detect;	
   
	ret=i2c_add_driver(&zet6221_driver);


	return ret; 
}

//驱动卸载函数
static void __exit zet6221_exit(void)
{
	i2c_del_driver(&zet6221_driver);
	if (zet6221_wq)
		destroy_workqueue(zet6221_wq);
	if(zet6221_wq1)
		destroy_workqueue(zet6221_wq1);
	
		
}

late_initcall(zet6221_init);
module_exit(zet6221_exit);

MODULE_DESCRIPTION("zet6221 Touchscreen Driver");
MODULE_LICENSE("GPL v2");

