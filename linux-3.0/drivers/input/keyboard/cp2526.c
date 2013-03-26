/*
 * drivers/input/keyboard/hv2605_keypad.c
 *
 * HV2605 KEYPAD driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>
//#include <mach/script_v2.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>

//#include <mach/gpio_v2.h>
//#include <mach/irqs.h>
#include "cp2526.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif
//#define KEY_DEBUG
//#define PRINT_STATUS_INFO
//#define  PRINT_SUSPEND_INFO
/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
static __u32 twi_id = 0;
static struct i2c_client *this_client;
static struct hv_keypad_data *hv_keypad;
static void* __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
#if 1
static int cp2526_gpio_led_hdle = 0;
#endif
//wisky-lxh@20120215,battery full flag
//extern int bat_full;
#if 0
static int gpio_pwr_hdle = 0;
#endif
struct key_event {
	int	key_val;
	int key_last;
	int	key_status;
};
static unsigned int sun4i_scankeycodes[5]=
{
	[0 ]  = KEY_RESERVED,
	[1 ]  = KEY_HOME,
	[2 ] = KEY_MENU,         
	[3 ] = KEY_RESERVED,       
	[4 ] = KEY_BACK,        
	
};
struct hv_keypad_data {
	struct input_dev     *input_dev;
	struct key_event     event;
 	struct delayed_work  work;
  	struct workqueue_struct *queue;
	uint32_t gpio_irq;
};
//#define KEY_DEBUG
//===============================================
#ifdef KEY_DEBUG
#define WPRINTK(format, args...)	do {		\
		printk(KERN_INFO "<WISKY-DEBUG> " format , ## args);	\
		} while (0)
#else
#define WPRINTK(format, args...)
#endif

#define CP2526_CONFIG_DATA_LEN		8	//config data see CP2526 spec.
static const u16 cp2526_config_data[CP2526_CONFIG_DATA_LEN] = {
	0x8000, 
	0x0000,
	0x1000,
	0x00FF,
	0x00FF,
	0x00FF,
	0x0001,
	0x0000,
};

//#define CP2526_LED_OFF_DELAY			//delay some time to turn off led after touch  不支持
struct delayed_work led_delay_work;
#define CP2526_LED_OFF_TIME 300
static void cp2526_touchkey_led_set( int enable);
//==================================================
#ifdef PRINT_STATUS_INFO
#define print_status_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_status_info(fmt, args...)   //
#endif

static struct i2c_msg tx_msg[] = {{0},};
static 	struct i2c_msg rx_msgs[] = {{0},{0},};

#ifdef CONFIG_HAS_EARLYSUSPEND
struct hv2605_keyboard_data {
    struct early_suspend early_suspend;
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct hv2605_keyboard_data *keyboard_data;
#endif


//////////////////////////////////////////////////////////
/*
*	return positive number: success
*	return negative number: fail
*/
static int cp2526_write_regs(struct i2c_client *client,uint8_t reg, const short *buf, unsigned len)
{
	int ret;
	//ret   = hv_i2c_rxdata();
	//ret = i2c_master_reg16_send(client, reg, buf, len, NULL);
	
	unsigned char buftmp[3]={0xff,0xff,0xff};
	struct i2c_msg msg;
	//memset(buf, 0xFF, sizeof(buf));
	len=len<<1;
	buftmp[0]=reg;
	buftmp[1]=*(unsigned short *)buf>>8;
	buftmp[2]=*buf&0xff;
	
	
	msg.addr = client->addr;
	msg.flags =  0;//写消息//0;
	msg.len = len+1;
	msg.buf = (char *)buftmp;	
	
	ret = i2c_transfer(client->adapter, &msg, 1); // 
	if(ret<0)	printk("i2c_master_send Error !ret=%d\n",-ret);

		
	return ret;
}

/*
*	return positive number: success and return read bytes
*	return negative:fail with error code
*/
//这里读的数据和供应商提供的应该读到的数据是高低位相反的，比如读0寄存器芯片标志本应该是0x2528,但这个函数读到的却是0x2825，如果和芯片资料对照时候要注意这一点
//在程序里面独到的值我都用高低位调换的值为准来处理，和供应商的代码有些不同。
static int cp2526_read_regs(struct i2c_client *client,uint8_t reg, short *buf, unsigned len)
{
	int ret;
	unsigned char buftmp[2];
	struct i2c_msg msg[2];
	//int ret;
	
	//len=len<<1;
	
	//memset(buf, 0xFF, sizeof(buf));
	msg[0].addr = client->addr;
	msg[0].flags = !I2C_M_RD;;
	msg[0].len = 1;
	msg[0].buf = &reg;
	
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf =buftmp;
	
	
	//printk("cp2526_read_regs:reg:%x\n",reg);
	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret<0)	WPRINTK("i2c_transfer Error !\n");
	//printk("data0:%d data1:%d\n",buftmp[0],buftmp[1]);
	//printk("<<<<<<--r--------client->addr = %d.\n",client->addr);
	//printk("<<<<<<--r--------client->name = %s.\n",client->name);
	*buf=buftmp[0]<<8|buftmp[1];

	
	return ret;
	
	
	//ret = i2c_master_reg16_recv(client, reg, buf, len, NULL);
}


static void cp2526_touchkey_led_set(int enable)
{
#if 1
	if(cp2526_gpio_led_hdle){
		if(enable)
		{
			//printk("turn on !\n");
			gpio_write_one_pin_value(cp2526_gpio_led_hdle, 1, "tkey_led");
		}
		else
		{
			//printk("turn off !\n");
			gpio_write_one_pin_value(cp2526_gpio_led_hdle, 0, "tkey_led");
		}
	}
#else
	short data;
	if(enable){
	data=0x003f;
	cp2526_write_regs(this_client,0x20,&data,1);
	//printk("turn on led\n");
	}	
	else{
	data=0x0000;
	cp2526_write_regs(this_client,0x20,&data,1);
	//printk("turn off led\n");
	}
#endif
}

/*
*	return positive number: success
*	return negative number: fail
*/
static int cp2526_powerup_setup(void)
{
	int ret;
	u16 readBuffer[1];

	ret=cp2526_read_regs(this_client,0x00,readBuffer,1);
	#ifdef KEY_DEBUG
	printk("%s-%d: cp2526 setup I2C 1 ret=%d\n", __FUNCTION__, __LINE__, ret);
	#endif
	
	ret=cp2526_read_regs(this_client,0x00,readBuffer,1);
	#ifdef KEY_DEBUG
	printk("%s-%d: cp2526 setup I2C 2 ret=%d\n", __FUNCTION__, __LINE__, ret);
	#endif
	
	ret=cp2526_write_regs(this_client,0x01,&cp2526_config_data[0],1);   /* 复位芯片 */
	#ifdef KEY_DEBUG
	printk("%s-%d: cp2526 setup I2C 3 ret=%d\n", __FUNCTION__, __LINE__, ret);
	#endif

	if(ret<0)
	{
		printk("%s-%d: cp2526 setup I2C error ret=%d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}
	#ifdef KEY_DEBUG
    //printk("%s-%d:readBuffer[0]=%d\n", __FUNCTION__, __LINE__,readBuffer[0] );
    #endif
    
	if(readBuffer[0]==0x2528)  /* 读芯片标识,如果是2825,标识正确,通讯正常 */
	{  
		cp2526_write_regs(this_client,0x01,&cp2526_config_data[0],1);   /* 复位芯片 */
		cp2526_write_regs(this_client,0x01,&cp2526_config_data[1],1);   /* 设置芯片使用的通道,对于不使用的通道进行关闭,如只打开前6个通道,则为0x00C0 */
		cp2526_write_regs(this_client,0x02,&cp2526_config_data[2],1);   /* 设置为GPIO0~7为扩展口,直接输出模式*/
		cp2526_write_regs(this_client,0x03,&cp2526_config_data[3],1);   /* 设置邻键抑制,一次只能一个按键有效 */
		cp2526_write_regs(this_client,0x05,&cp2526_config_data[4],1);   /* 按键状态发生变化时,产生中断*/
		cp2526_write_regs(this_client,0x06,&cp2526_config_data[5],1);   /* 打开中断*/
		cp2526_write_regs(this_client,0x21,&cp2526_config_data[6],1);   /* 使能GPIO0扩展功能，用来控制按键灯*/
		
		//cp2526_write_regs(this_client,0x21,&cp2526_config_data[6],1);   /* 使能GPIO0扩展功能，用来控制按键灯*/
		
		cp2526_write_regs(this_client,0x28,&cp2526_config_data[7],1);   /* 设置长按有限时间10s */
		
		
		cp2526_read_regs(this_client,0x33,readBuffer,1); /*清空中断寄存器*/
		printk("%s-%d: read cp2526 chip ID OK\n", __FUNCTION__, __LINE__);
	}
	else
	{
		printk("%s-%d: read cp2526 chip ID error\n", __FUNCTION__, __LINE__);
		ret=-EFAULT;	 /*关断触摸芯片,或报警处理*/
	}

//while(1)
//{
//	ret=cp2526_read_regs(this_client,0x00,readBuffer,1);
//	printk("%s-%d: cp2526 setup I2C 1 ret=%d\n", __FUNCTION__, __LINE__, ret);
//	mdelay(50);
//}
	
	return ret;
}

//////////////////////////////////////////////////////////

//static int hv_i2c_rxdata(char *rxdata, int length)
//{
//	int ret;
//
//    rx_msgs[0].addr	= this_client->addr;
//	rx_msgs[0].flags	= I2C_M_RD;
//	rx_msgs[0].len	= 1;
//	rx_msgs[0].buf	= rxdata;
//
//    //msleep(1);
//	ret = i2c_transfer(this_client->adapter, rx_msgs, 1);
//	//printk("msg i2c read: 0x%x\n", this_client->addr);
//	//printk("HV IIC read data\n");
//	if (ret < 0){
//		//pr_info("msg %s i2c read error: 0x%x\n", __func__, this_client->addr);
//	}
//
//	return ret;
//}

//static int hv_i2c_txdata(char *txdata, int length)
//{
//	int ret;
//    tx_msg[0].addr   = this_client->addr;
//    tx_msg[0].flags  = 0;
//    tx_msg[0].len    = length;
//    tx_msg[0].buf    = txdata;
//
//   	//msleep(1);
//	ret = i2c_transfer(this_client->adapter, tx_msg, 1);
//	if (ret < 0)
//		pr_err("%s i2c write error: %d\n", __func__, ret);
//
//	return ret;
//}

static int hv_init(void)
{
 
    int ret = -1;
    struct hv_keypad_data *data = i2c_get_clientdata(this_client);
    struct key_event *event = &data->event;
    event->key_status = 2;
    event->key_last = 0;
    ret = cp2526_powerup_setup();
    if (ret > 1) {
        printk("write reg failed!  ret: %d",  ret);
        return -1;
    }

    return 0;
}

static void hv_keypad_release(void)
{
	struct hv_keypad_data *data = i2c_get_clientdata(this_client);
	struct key_event *event = &data->event;
	input_report_key(data->input_dev,sun4i_scankeycodes[event->key_val], 0);
	#ifdef KEY_DEBUG
		printk("======Release the key %d  sun4i_scankeycodes[event->key_val]:%d=====\n",event->key_val,sun4i_scankeycodes[event->key_val]);
	#endif
	input_sync(data->input_dev);
	event->key_status = 2;
	event->key_last = 0;
}


//static int hv_read_data(void)
//{
//	struct hv_keypad_data *data = i2c_get_clientdata(this_client);
//	struct key_event *event = &data->event;
//	u8 buf[2] = {0};
//	int ret = -1;
//	//memset(event, 0, sizeof(struct key_event));
//	ret = hv_i2c_rxdata(buf, 1);
//    if(ret==1){
//    	if((0xff != buf[0])&& (0 != buf[0])){
//            event->key_val    = buf[0];
//    	}else if(0xff == buf[0]){
//            //event->key_val = event->key_last;
//            event->key_val    = buf[0];
//            //printk("receive 0xff. \n");
//    	}
//    	print_status_info("157 event->key_status = %d\n",event->key_status);
//
//    }else{
//    	if(event->key_status ==1){
//            event->key_status = 0;
//    	}
//        print_status_info("163 event->key_status = %d\n",event->key_status);
//
//    }
//
//	return ret;
//}
//static unsigned int hv_keycard_map[5] ={1,2,3,4,5};
static void hv_report_value(void)
{
	struct hv_keypad_data *data = i2c_get_clientdata(this_client);
	struct key_event *event = &data->event;

	input_report_key(data->input_dev, sun4i_scankeycodes[event->key_val], 1);
	#ifdef KEY_DEBUG
		printk("Press the key %d sun4i_scankeycodes:%d\n",event->key_val,sun4i_scankeycodes[event->key_val]);
	#endif
	input_sync(data->input_dev);
	event->key_status = 1;
}

#if defined(CP2526_LED_OFF_DELAY)
static void cp2526_led_delaywork(struct work_struct *work)
{
	cp2526_touchkey_led_set( 0);
}
#endif

static void hv_read_loop(struct work_struct *work)
{
	int ret = -1;
	struct hv_keypad_data *data = i2c_get_clientdata(this_client);
	struct key_event *event = &data->event;
	//printk("==========Begin Read Data============\n");
    //ret = hv_read_data();
	short ReadBuffer[3]={0};
	int keycode=0,key=0,i;
	static int back_key_cnt = 0;


	//printk("gpio_read_one_pin_value:%d\n",gpio_read_one_pin_value(gpio_int_hdle,"tkey_int"));
	ret = cp2526_read_regs(this_client,0x33, &ReadBuffer[0], 1);
	ret = cp2526_read_regs(this_client,0x31, &ReadBuffer[1], 1);
	ret = cp2526_read_regs(this_client,0x30, &ReadBuffer[2], 1);
	
	//
	
	if(0x7000==ReadBuffer[0]||0x3000==ReadBuffer[0]||0x3003==ReadBuffer[0]||0x3005==ReadBuffer[0]||0x3007==ReadBuffer[0])
		//return ;
		goto restart_delaywork;
	//if(ReadBuffer[1]==3||ReadBuffer[1]==5||ReadBuffer[1]==7)
	//	return ;
	//printk("0:%x 1:%x 2:%x\n",ReadBuffer[0],ReadBuffer[1],ReadBuffer[2]);
#if 0
	//cd huang@20110610^do not report key when multi-touch
	if(0x300 == ReadBuffer[2] || 0x500 == ReadBuffer[2] || 0x600 == ReadBuffer[2] || 0x700 == ReadBuffer[2]){

		WPRINTK("muti-key touch,do not repot...\n");
		goto restart_delaywork;
	}
#endif
	
	//read valid data
#if 0
	for(i=0;i<2;i++)
	{
		if(ReadBuffer[1]&(0x0100>>i))	/*如果标志位为1, 表示此通道按键处于按下状态*/
		{
			key=i+1;	  /*返回按键通道信息*/
			break;				  
		}
	}
#endif
	key=ReadBuffer[1];
  event->key_val = key;
#ifdef KEY_DEBUG
	printk(" ReadBuffer[1]=%d,key=%d\n", ReadBuffer[1],key);
	printk("ret:%d\n",ret);
#endif

	
	    if(ret==2)
    {
    	
        	switch(event->key_val)
        	{
        		case 1:
        		case 2:
        		case 3:
        		case 4:
        		case 5:
        		    #ifdef KEY_DEBUG
        		    printk("key_last=%d,key_val=%d.\n",event->key_last,event->key_val);
        		    #endif
        		    
        		    if(event->key_last != event->key_val){
                        event->key_last = event->key_val;
                        hv_report_value();
				cp2526_touchkey_led_set(1);
                    }
        		    break;

        		/*case 0xff:
            	    if(event->key_status == 1){
                        event->key_val = event->key_last;
                        hv_keypad_release();
            	    }
            		break;*/

        		default :
        		      if(event->key_status == 1){
                        event->key_val = event->key_last;
                        hv_keypad_release();
			  #if defined(CP2526_LED_OFF_DELAY)
			queue_delayed_work(hv_keypad->queue, &led_delay_work, msecs_to_jiffies(CP2526_LED_OFF_TIME));
			#else
			cp2526_touchkey_led_set( 0);
			#endif			
            	    }
        		    //hv_keypad_release();
        		    break;

        	}

    	    print_status_info("225 event->key_status = %d\n",event->key_status);
    }else if(event->key_status ==0){
            hv_keypad_release();
    		print_status_info("233 event->key_status = %d\n",event->key_status);
    }

restart_delaywork:
	queue_delayed_work(hv_keypad->queue, &hv_keypad->work, DELAY_PERIOD);

}


/*******************************************************	
功能：
	中断响应函数
	由中断触发，调度触摸按键 处理函数运行
********************************************************/
static irqreturn_t  tkey_irq_handler(int irq, void *dev_id)
{
	//struct hv_keypad_data *ts = dev_id;	
	int reg_val;	
	

	//clear the IRQ_EINT21 interrupt pending
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
    WPRINTK("==========------tkey Interrupt-----============,reg_val=%d\n",reg_val);  
	if(reg_val&(1<<(IRQ_EINT_USED_TKEY)))
	{	
		WPRINTK("==IRQ_EINT21=\n");
		writel(reg_val&(1<<(IRQ_EINT_USED_TKEY)),gpio_addr + PIO_INT_STAT_OFFSET);
		//queue_work(hv_keypad->queue, &hv_keypad->work);
		queue_delayed_work(hv_keypad->queue, &hv_keypad->work, 2);
	}
	else
	{
	    WPRINTK("Other Interrupt\n");
		return IRQ_NONE;
	}
	return IRQ_HANDLED;
}







#ifdef CONFIG_HAS_EARLYSUSPEND
static void hv2605_keyboard_suspend(struct early_suspend *h)
{
    #ifdef PRINT_SUSPEND_INFO
        printk("enter earlysuspend: hv2605_keyboard_suspend. \n");
    #endif
    cancel_delayed_work_sync(&hv_keypad->work);
	//wisky-lxh@20120215,if battery full,don't set led off
	//if (bat_full == 0)
	cp2526_touchkey_led_set(0);
    return ;
}
static void hv2605_keyboard_resume(struct early_suspend *h)
{
	u16 com = 0x0000,ret;
	u16 readBuffer[1];
    #ifdef PRINT_SUSPEND_INFO
        printk("enter laterresume: hv2605_keyboard_resume. \n");
    #endif

	cp2526_write_regs(this_client,0x01,&com,1); /* 所有通道正常工作*/
	cp2526_read_regs(this_client,0x33,readBuffer,1); /*清空中断寄存器*/
    queue_delayed_work(hv_keypad->queue, &hv_keypad->work, DELAY_PERIOD);
    return ;
}
#else
#endif
#ifdef CONFIG_PM
static int cp2526_suspend(struct i2c_client *client, pm_message_t mesg)
{

	return 0;
}

static int cp2526_resume(struct i2c_client *client)
{
#if 0
gpio_write_one_pin_value(gpio_pwr_hdle, 1, "tkey_power_port");
#endif
	cp2526_powerup_setup();
	
	return 0;
}
#endif


static int hv_cp2625_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct input_dev *input_dev;
	int err = 0;
	int i;
	int reg_val;

	printk("==========================hv_keypad_probe Begin===========================\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk("============exit_check_functionality_failed===========================");
		goto exit_check_functionality_failed;
	}
	#ifdef KEY_DEBUG
	printk("%s,1.... \n",__FUNCTION__ );
	#endif
	
	hv_keypad = kzalloc(sizeof(*hv_keypad), GFP_KERNEL);
	if (!hv_keypad)	{
		err = -ENOMEM;
		printk("============exit_alloc_data_failed===========================");
		goto exit_alloc_data_failed;
	}
	
	//=============================
	#if 0
	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	if(!gpio_addr) {
	    err = -EIO;
	    goto exit_ioremap_failed;	
	}
	hv_keypad->gpio_irq = INT_PORT;

	//Config IRQ_EINT19 Negative Edge Interrupt
        reg_val = readl(gpio_addr + PIO_INT_CFG2_OFFSET);
        reg_val &=(~(7<<12));
       reg_val |=(1<<12);  
        writel(reg_val,gpio_addr + PIO_INT_CFG2_OFFSET);
        
        //Enable IRQ_EINT11 of PIO Interrupt
        reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
        reg_val |=(1<<IRQ_EINT_USED_TKEY);
        writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
		
	//config gpio INT:
	gpio_int_hdle = gpio_request_ex("tkey_para", "tkey_int");
	if(!gpio_int_hdle) {
		printk("tkey panel IRQ_EINT19_para request gpio fail!\n");
	    goto exit_gpio_int_request_failed;
	}
	#endif
#if 1
	//config gpio LED :
	cp2526_gpio_led_hdle = gpio_request_ex("tkey_para", "tkey_led");
	if(cp2526_gpio_led_hdle) {
		printk("get tkey panel led request !\n");
		gpio_write_one_pin_value(cp2526_gpio_led_hdle, 0, "tkey_led");
	}
#endif
#if 0
	gpio_pwr_hdle = gpio_request_ex("tkey_para", "tkey_power_port");
	if(gpio_pwr_hdle) {
		//printk("tkey panel tkey_power_port request gpio fail!\n");
		gpio_write_one_pin_value(gpio_pwr_hdle, 1, "tkey_power_port");

	}
#endif
	
//	printk("%s,2.... \n",__FUNCTION__ );
	//==============================
	this_client = client;
	i2c_set_clientdata(client, hv_keypad);

#if defined(CP2526_LED_OFF_DELAY)
	INIT_DELAYED_WORK(&led_delay_work, cp2526_led_delaywork);
#endif
	INIT_DELAYED_WORK(&hv_keypad->work, hv_read_loop);
	hv_keypad->queue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!hv_keypad->queue) {
		err = -ESRCH;
		printk("============exit_create_singlethread===========================");
		goto exit_create_singlethread;
	}

    #ifdef KEY_DEBUG
	printk("%s,3.... \n",__FUNCTION__ );
	#endif
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		printk("============exit_input_dev_alloc_failed===========================");
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
    #ifdef KEY_DEBUG
	printk("%s,4.... \n",__FUNCTION__ );
	#endif
	
	hv_keypad->input_dev = input_dev;

	input_dev->name = HV_NAME;
	input_dev->phys = "sun4ikbd/inputx";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	input_dev->evbit[0] = BIT_MASK(EV_KEY);

	//for (i = 1; i < 6; i++)
	//{
	//	set_bit(i, input_dev->keybit);
	//	
	//}
	
	for (i = 0; i < 5; i++)
		set_bit(sun4i_scankeycodes[i], input_dev->keybit);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"hv_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

    hv_init();
    #ifdef KEY_DEBUG
	printk("%s,4.... \n",__FUNCTION__ );
	#endif
	queue_delayed_work(hv_keypad->queue, &hv_keypad->work, DELAY_PERIOD);
	#if 0
	err =  request_irq(SW_INT_IRQNO_PIO, tkey_irq_handler, IRQF_TRIGGER_FALLING | IRQF_SHARED, client->name, hv_keypad);
	if (err < 0) {
		pr_info( "goodix_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	#endif

    #ifdef KEY_DEBUG
	printk("%s,6.... \n",__FUNCTION__ );
	#endif
	
#ifdef CONFIG_HAS_EARLYSUSPEND
    printk("==register_early_suspend =\n");
    keyboard_data = kzalloc(sizeof(*keyboard_data), GFP_KERNEL);
	if (keyboard_data == NULL) {
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}
    keyboard_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    keyboard_data->early_suspend.suspend = hv2605_keyboard_suspend;
    keyboard_data->early_suspend.resume	= hv2605_keyboard_resume;
    register_early_suspend(&keyboard_data->early_suspend);
#endif

cp2526_touchkey_led_set(0);
	printk("==cp2526 probe ======over =\n");

    return 0;

exit_irq_request_failed:
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	#if 0
err_i2c_failed:
exit_gpio_wakeup_request_failed:
exit_gpio_int_request_failed:
exit_ioremap_failed:
    if(gpio_addr){
        iounmap(gpio_addr);
    }
	#endif
	kfree(hv_keypad);
exit_alloc_data_failed:
exit_check_functionality_failed:
#ifdef CONFIG_HAS_EARLYSUSPEND
 err_alloc_data_failed:
#endif
	return err;
}

static int __devexit hv_keypad_remove(struct i2c_client *client)
{

	struct hv_keypad_data *hv_keypadc = i2c_get_clientdata(client);
	input_unregister_device(hv_keypadc->input_dev);
	kfree(hv_keypad);
	printk("==hv_keypad_remove=\n");
	//cancel_work_sync(&zt_ts->work);
	//destroy_workqueue(zt_ts->queue);
	cancel_delayed_work_sync(&hv_keypad->work);
	destroy_workqueue(hv_keypad->queue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id hv_keypad_id[] = {
	{ HV_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, hv_keypad_id);

static struct i2c_driver hv_keypad_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= hv_cp2625_probe,
	.remove		= __devexit_p(hv_keypad_remove),
	.id_table	= hv_keypad_id,
	.driver	= {
		.name	=    HV_NAME,
		.owner	=    THIS_MODULE,
	},
	.address_list	= u_i2c_addr.normal_i2c,
#ifdef CONFIG_PM
	.suspend	= cp2526_suspend,
	.resume = cp2526_resume,
#endif
};
//	.addr           = 0x2C,//0b1100 010x
static int tkey_fetch_sysconfig_para(void)
{
	int ret = -1;
	int device_used = -1;
	__u32 twi_addr = 0;

	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
	
	//__u32 twi_id = 0;

	printk("========HV Inital ===================\n");
	if(SCRIPT_PARSER_OK != script_parser_fetch("tkey_para", "tkey_used", &device_used, 1)){
	                pr_err("hv_keyboard: script_parser_fetch err. \n");
	                goto script_parser_fetch_err;
	}
	if(1 == device_used){
		if(SCRIPT_PARSER_OK != script_parser_fetch_ex("tkey_para", "tkey_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
			pr_err("%s: script_parser_fetch err. \n", __func__);
			goto script_parser_fetch_err;
		}
		if(strcmp(HV_NAME, name)){
			pr_err("%s: name %s does not match HV_NAME. \n", __func__, name);
			pr_err(HV_NAME);
			//ret = 1;
			return ret;
		}
		if(SCRIPT_PARSER_OK != script_parser_fetch("tkey_para", "tkey_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
			pr_err("%s: script_parser_fetch err. \n", name);
			goto script_parser_fetch_err;
		}
		u_i2c_addr.dirty_addr_buf[0] = twi_addr;
		u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
		printk("%s: after: tkey_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", \
		__func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
		
		if(SCRIPT_PARSER_OK != script_parser_fetch("tkey_para", "tkey_twi_id", &twi_id, 1)){
			pr_err("%s: script_parser_fetch err. \n", name);
			goto script_parser_fetch_err;
		}
		printk("%s: tkey_twi_id is %d. \n", __func__, twi_id);
		
	}else{
		pr_err("%s: tkey_unused. \n",  __func__);
		ret = -1;
	}

	return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;

}
int tkey_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, HV_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, HV_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}
static int __init hv_keypad_init(void)
{
    int ret = -1;
    int hv_keypad_used = -1;
#if 0
	printk("========HV Inital ===================\n");

	if(SCRIPT_PARSER_OK != script_parser_fetch("tkey_para", "tkey_used", &hv_keypad_used, 1)){
        pr_err("hv_keypad: script_parser_fetch err. \n");
        goto script_parser_fetch_err;
	}
	if(1 != hv_keypad_used){
        pr_err("hv_keypad: hv_keypad_unused. \n");
        return 0;
	}
#endif
	printk("===========================%s=====================\n", __func__);

	if(tkey_fetch_sysconfig_para()){
		printk("%s: err.\n", __func__);
		return -1;
	}

	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	hv_keypad_driver.detect = tkey_detect;	
	ret = i2c_add_driver(&hv_keypad_driver);
	printk("%s,ret= %d \n",__FUNCTION__ ,ret);
script_parser_fetch_err:
    return ret;

}



static void __exit hv_keypad_exit(void)
{
	i2c_del_driver(&hv_keypad_driver);
	return;
}
module_init(hv_keypad_init);
module_exit(hv_keypad_exit);

MODULE_AUTHOR("<zhengdixu@allwinnertech.com>");
MODULE_DESCRIPTION("hv keypad driver");
MODULE_LICENSE("GPL");


