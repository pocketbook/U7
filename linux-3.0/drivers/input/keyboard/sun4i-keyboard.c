/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
*
* Copyright (c) 2011
*
* ChangeLog
*
*
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/keyboard.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/timer.h> 
#include <mach/sys_config.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif

#if 0
#define  KEY_DEBUG
#define  KEY_DEBUG_LEVEL2
#define  PRINT_SUSPEND_INFO
#endif
#define  KEY_MAX_CNT  		(8)//(13)

 #define INPUT_DEV_NAME	("sun4i-keyboard")
#define  KEY_BASSADDRESS	(0xf1c22800)
#define  LRADC_CTRL		(0x00)
#define  LRADC_INTC		(0x04)
#define  LRADC_INT_STA 		(0x08)
#define  LRADC_DATA0		(0x0c)
#define  LRADC_DATA1		(0x10)

#define  FIRST_CONCERT_DLY		(2<<24)
#define  CHAN				(0x3)
#define  ADC_CHAN_SELECT		(CHAN<<22)
#define  LRADC_KEY_MODE		(0)
#define  KEY_MODE_SELECT		(LRADC_KEY_MODE<<12)
#define  LEVELB_VOL			(0<<4)

#define  LRADC_HOLD_EN		(1<<6)

#define  LRADC_SAMPLE_32HZ		(3<<2)
#define  LRADC_SAMPLE_62HZ		(2<<2)
#define  LRADC_SAMPLE_125HZ		(1<<2)
#define  LRADC_SAMPLE_250HZ		(0<<2)


#define  LRADC_EN			(1<<0)

#define  LRADC_ADC1_UP_EN		(1<<12)
#define  LRADC_ADC1_DOWN_EN		(1<<9)
#define  LRADC_ADC1_DATA_EN		(1<<8)

#define  LRADC_ADC0_UP_EN		(1<<4)
#define  LRADC_ADC0_DOWN_EN		(1<<1)
#define  LRADC_ADC0_DATA_EN		(1<<0)

#define  LRADC_ADC1_UPPEND		(1<<12)
#define  LRADC_ADC1_DOWNPEND	(1<<9)
#define  LRADC_ADC1_DATAPEND		(1<<8)


#define  LRADC_ADC0_UPPEND 		(1<<4)
#define  LRADC_ADC0_DOWNPEND	(1<<1)
#define  LRADC_ADC0_DATAPEND		(1<<0)

#define EVB
//#define CUSTUM
#define ONE_CHANNEL
#define MODE_0V2
//#define MODE_0V15
//#define TWO_CHANNEL
#ifdef MODE_0V2
//standard of key maping
//0.2V mode	 

#define REPORT_START_NUM			(2)
#define REPORT_KEY_LOW_LIMIT_COUNT		(1)
#define MAX_CYCLE_COUNTER			(100)
//#define REPORT_REPEAT_KEY_BY_INPUT_CORE
//#define REPORT_REPEAT_KEY_FROM_HW
#define INITIAL_VALUE				(0Xff)

static unsigned char keypad_mapindex[64] =
{
    0,0,0,0,0,0,0,0,               //key 1, 8个， 0-7
    1,1,1,1,1,1,1,                 //key 2, 7个， 8-14
    2,2,2,2,2,2,2,                 //key 3, 7个， 15-21
    3,3,3,3,3,3,                   //key 4, 6个， 22-27
    4,4,4,4,4,4,                   //key 5, 6个， 28-33
    5,5,5,5,5,5,                   //key 6, 6个， 34-39
    6,6,6,6,6,6,6,6,6,6,           //key 7, 10个，40-49
    7,7,7,7,7,7,7,7,7,7,7,7,7,7    //key 8, 17个，50-63
};
#endif
                        
#ifdef MODE_0V15
//0.15V mode
static unsigned char keypad_mapindex[64] =
{
	0,0,0,                      //key1
	1,1,1,1,1,                  //key2
	2,2,2,2,2,
	3,3,3,3,
	4,4,4,4,4,
	5,5,5,5,5,
	6,6,6,6,6,
	7,7,7,7,
	8,8,8,8,8,
	9,9,9,9,9,
	10,10,10,10,
	11,11,11,11,
	12,12,12,12,12,12,12,12,12,12 //key13
};
#endif

#ifdef EVB
static unsigned int sun4i_scankeycodes[KEY_MAX_CNT];
/*
=
{
	[0 ] = KEY_VOLUMEUP,       
	[1 ] = KEY_VOLUMEDOWN,  
	//[0 ] = KEY_BACK,
	//[1 ] = KEY_MENU,  
	[2 ] = KEY_MENU,         
	[3 ] = KEY_MENU,       
	[4 ] = KEY_HOME,   
	[5 ] = KEY_ESC, 
	[6 ] = KEY_HOME,//KEY_ENTER,       
	[7 ] = KEY_RESERVED,
	//[8 ] = KEY_RESERVED,
	//[9 ] = KEY_RESERVED,
	//[10] = KEY_RESERVED,
	//[11] = KEY_RESERVED,
	//[12] = KEY_RESERVED,
};

*/
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND	
struct sun4i_keyboard_data {
    struct early_suspend early_suspend;
};
#endif

static volatile unsigned int key_val;
static struct input_dev *sun4ikbd_dev;
static unsigned char scancode;

static unsigned char key_cnt = 0;
static unsigned char cycle_buffer[REPORT_START_NUM] = {0};
static unsigned char transfer_code = INITIAL_VALUE;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct sun4i_keyboard_data *keyboard_data;
#endif
static int keyboard_debug=0;
static int suspend_use=1;
static ssize_t keydebugstore(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	if(buf[0] == '1'){
		keyboard_debug = 1; 
	}
	else{
		keyboard_debug = 0;         
	}        
	return count;
}

static ssize_t keydebugshow(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "keyboard-debug value is %d\n", keyboard_debug);
}
static DEVICE_ATTR(keydebug,S_IRUGO|S_IWUSR, keydebugshow,keydebugstore);


void Setbit_forMD7008(void)
{
		set_bit(sun4i_scankeycodes[3], sun4ikbd_dev->keybit);
		set_bit(sun4i_scankeycodes[6], sun4ikbd_dev->keybit);
}
EXPORT_SYMBOL_GPL(Setbit_forMD7008);
void Clearbit_forMD7008(void)
{
	     clear_bit(sun4i_scankeycodes[3], sun4ikbd_dev->keybit);
	     clear_bit(sun4i_scankeycodes[6], sun4ikbd_dev->keybit);
}
EXPORT_SYMBOL_GPL(Clearbit_forMD7008);
//停用设备
#ifdef CONFIG_HAS_EARLYSUSPEND
static void sun4i_keyboard_suspend(struct early_suspend *h)
{
	/*int ret;
	struct sun4i_keyboard_data *ts = container_of(h, struct sun4i_keyboard_data, early_suspend);
      */
#ifdef PRINT_SUSPEND_INFO
	printk("[%s] enter standby state: %d. \n", __FUNCTION__, (int)standby_type);
#endif
    
	if (NORMAL_STANDBY == standby_type) {

		writel(0,KEY_BASSADDRESS + LRADC_CTRL);
	/*process for super standby*/	
	} else if(SUPER_STANDBY == standby_type) {
		;
	}	
    
	return ;
}

//重新唤醒
static void sun4i_keyboard_resume(struct early_suspend *h)
{
/*int ret;
	struct sun4i_keyboard_data *ts = container_of(h, struct sun4i_keyboard_data, early_suspend);
*/
#ifdef PRINT_SUSPEND_INFO
	printk("[%s] return from standby state: %d. \n", __FUNCTION__, (int)standby_type);
#endif
	/*process for normal standby*/
	if (NORMAL_STANDBY == standby_type) {
	writel(FIRST_CONCERT_DLY|LEVELB_VOL|KEY_MODE_SELECT|LRADC_HOLD_EN|ADC_CHAN_SELECT|LRADC_SAMPLE_125HZ|LRADC_EN,KEY_BASSADDRESS + LRADC_CTRL);
	/*process for super standby*/	
	} else if(SUPER_STANDBY == standby_type) {
		#ifdef ONE_CHANNEL
			writel(LRADC_ADC0_DOWN_EN|LRADC_ADC0_UP_EN|LRADC_ADC0_DATA_EN,KEY_BASSADDRESS + LRADC_INTC);	
			writel(FIRST_CONCERT_DLY|LEVELB_VOL|KEY_MODE_SELECT|LRADC_HOLD_EN|ADC_CHAN_SELECT|LRADC_SAMPLE_125HZ|LRADC_EN,KEY_BASSADDRESS + LRADC_CTRL);
			//writel(FIRST_CONCERT_DLY|LEVELB_VOL|KEY_MODE_SELECT|ADC_CHAN_SELECT|LRADC_SAMPLE_62HZ|LRADC_EN,KEY_BASSADDRESS + LRADC_CTRL);
		#else
		#endif
	}
	
	return ; 
}
#else

#endif


static irqreturn_t sun4i_isr_key(int irq, void *dummy)
{
	unsigned int  reg_val;
	int judge_flag = 0;
	int loop = 0;
	
	#ifdef KEY_DEBUG
	    printk("Key Interrupt\n");
  	#endif
	reg_val  = readl(KEY_BASSADDRESS + LRADC_INT_STA);
	//writel(reg_val,KEY_BASSADDRESS + LRADC_INT_STA);

	if(reg_val&LRADC_ADC0_DOWNPEND)
	{
		#ifdef KEY_DEBUG
		    printk("key down\n");
		#endif
	}
	
	if(reg_val&LRADC_ADC0_DATAPEND)
	{
		key_val = readl(KEY_BASSADDRESS+LRADC_DATA0);
		
		if(key_val < 0x3f)
		{

		cycle_buffer[key_cnt%REPORT_START_NUM] = key_val&0x3f;

		if((key_cnt + 1) < REPORT_START_NUM)
		{

			//do not report key message
			//printk("do not report key message\n");
		}else{
		   // printk("key_cnt = %d \n",key_cnt);
			//scancode = cycle_buffer[(key_cnt-2)%REPORT_START_NUM];
			if(cycle_buffer[(key_cnt - REPORT_START_NUM + 1)%REPORT_START_NUM] \
			== cycle_buffer[(key_cnt - REPORT_START_NUM + 2)%REPORT_START_NUM])
			{
			key_val = cycle_buffer[(key_cnt - REPORT_START_NUM + 1)%REPORT_START_NUM];
			scancode = keypad_mapindex[key_val&0x3f];
			judge_flag = 1;

			}
#if 0  
<<<<<<< HEAD
			
			if((!judge_flag) && cycle_buffer[(key_cnt - REPORT_START_NUM + 4)%REPORT_START_NUM] \
			== cycle_buffer[(key_cnt - REPORT_START_NUM + 5)%REPORT_START_NUM])
=======
#endif
#ifdef KEY_DEBUG
			printk("cycle_buffer[(key_cnt - 1)] = 0x%x,cycle_buffer[(key_cnt -  2)]) = 0x%x\n",cycle_buffer[(key_cnt -  1)],cycle_buffer[(key_cnt -  2)]);
#endif
			if((!judge_flag) && (cycle_buffer[(key_cnt - REPORT_START_NUM + 1)%REPORT_START_NUM] \
			== cycle_buffer[(key_cnt - REPORT_START_NUM + 2)%REPORT_START_NUM]))
#if 0
>>>>>>> a1x-dev
#endif
			{

			key_val = cycle_buffer[(key_cnt - REPORT_START_NUM + 1)%REPORT_START_NUM];
			scancode = keypad_mapindex[key_val&0x3f];
			judge_flag = 1;
			                           

			} 

			if(1 == judge_flag)
			{
//#ifdef KEY_DEBUG_LEVEL2
				if(keyboard_debug){
				printk("report data: key_val :%8d transfer_code: %8d , scancode: %8d\n",\
				key_val, transfer_code, scancode);				
				printk("sun4i_scankeycodes==key value is %d\n",sun4i_scankeycodes[transfer_code]);
//#endif			
				}
				
				if(transfer_code == scancode){
				//report repeat key value
#ifdef REPORT_REPEAT_KEY_FROM_HW
				input_report_key(sun4ikbd_dev, sun4i_scankeycodes[scancode], 0);
				input_sync(sun4ikbd_dev);
				input_report_key(sun4ikbd_dev, sun4i_scankeycodes[scancode], 1);
				input_sync(sun4ikbd_dev);
#else
				//do not report key value
#endif
				}else if(INITIAL_VALUE != transfer_code){                               
				//report previous key value up signal + report current key value down
				input_report_key(sun4ikbd_dev, sun4i_scankeycodes[transfer_code], 0);
				input_sync(sun4ikbd_dev);
				input_report_key(sun4ikbd_dev, sun4i_scankeycodes[scancode], 1);
				input_sync(sun4ikbd_dev);
				transfer_code = scancode;

				}else{
				//INITIAL_VALUE == transfer_code, first time to report key event
				input_report_key(sun4ikbd_dev, sun4i_scankeycodes[scancode], 1);
				input_sync(sun4ikbd_dev);
				transfer_code = scancode;
				}

			}

			}
			key_cnt++;
			if(key_cnt > 2 * MAX_CYCLE_COUNTER ){
			key_cnt -= MAX_CYCLE_COUNTER;
			}

		}
	}
        
	if(reg_val&LRADC_ADC0_UPPEND)
	{
		if(key_cnt > REPORT_START_NUM)
		{
			if(INITIAL_VALUE != transfer_code)
			{
//#ifdef KEY_DEBUG_LEVEL2
			if(keyboard_debug){
			printk("report data: key_val :%8d transfer_code: %8d \n",key_val, transfer_code);
			}
//#endif
			input_report_key(sun4ikbd_dev, sun4i_scankeycodes[transfer_code], 0);
			input_sync(sun4ikbd_dev);
			}

		}else if( key_cnt >= REPORT_KEY_LOW_LIMIT_COUNT){   
			//rely on hardware first_delay work, need to be verified!
			if(cycle_buffer[0] == cycle_buffer[1]){
				key_val = cycle_buffer[0];
				scancode = keypad_mapindex[key_val&0x3f];
//#ifdef KEY_DEBUG_LEVEL2
				if(keyboard_debug){
				printk("report data: key_val :%8d scancode: %8d \n",key_val, scancode);
				}
//#endif
				input_report_key(sun4ikbd_dev, sun4i_scankeycodes[scancode], 1);
				input_sync(sun4ikbd_dev);   
				input_report_key(sun4ikbd_dev, sun4i_scankeycodes[scancode], 0);
				input_sync(sun4ikbd_dev);  
			}

		}

#ifdef KEY_DEBUG
		printk("key up \n");
#endif

		key_cnt = 0;
		judge_flag = 0;
		transfer_code = INITIAL_VALUE;
		for(loop = 0; loop < REPORT_START_NUM; loop++)
		{
			cycle_buffer[loop] = 0; 
		}

	}
	
	writel(reg_val,KEY_BASSADDRESS + LRADC_INT_STA);
	return IRQ_HANDLED;
}

static int __init sun4ikbd_init(void)
{
	int i;
	int err =0;	
	char mach_name[20]={0};
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;
#ifdef KEY_DEBUG
	printk("sun4ikbd_init \n");
#endif
	sun4ikbd_dev = input_allocate_device();
	if (!sun4ikbd_dev) {
		printk(KERN_ERR "sun4ikbd: not enough memory for input device\n");
		err = -ENOMEM;
		goto fail1;
	}
#if 1
	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("product", "machine", (int *)(&mach_name), &type, sizeof(mach_name)/sizeof(int))){
		pr_err("%s: line: %d script_parser_fetch err.,%s\n", __func__, __LINE__,mach_name);
			
	}
	printk("key board machine:%s\n",mach_name);
	if(!strcmp("M876-A13", mach_name)){
	printk("choose M876-A13\n");
	sun4i_scankeycodes[0 ] = KEY_VOLUMEUP;     //used
	sun4i_scankeycodes[1 ] = KEY_VOLUMEDOWN;   //used
	sun4i_scankeycodes[2 ] = KEY_MENU;         
	sun4i_scankeycodes[3 ] = KEY_MENU;       
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC;
	sun4i_scankeycodes[6 ] = KEY_MENU;         //used
	sun4i_scankeycodes[7 ] = KEY_RESERVED;
	/*
	sun4i_scankeycodes[0 ] = KEY_BACK;     
	sun4i_scankeycodes[1 ] = KEY_MENU;  
	sun4i_scankeycodes[2 ] = KEY_MENU;         
	sun4i_scankeycodes[3 ] = KEY_MENU;       
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC;
	sun4i_scankeycodes[6 ] = KEY_HOME;      
	sun4i_scankeycodes[7 ] = KEY_RESERVED;*/
				
	}	
	else if(!strcmp("M828-A13", mach_name)){
	sun4i_scankeycodes[0 ] = KEY_VOLUMEUP;     
	sun4i_scankeycodes[1 ] = KEY_VOLUMEDOWN;  
	sun4i_scankeycodes[2 ] = KEY_MENU;         
	sun4i_scankeycodes[3 ] = KEY_MENU;       
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC;
	sun4i_scankeycodes[6 ] = KEY_HOME;   
	sun4i_scankeycodes[7 ] = KEY_RESERVED;		
	}	
	else if(!strcmp("T712-A13", mach_name)){
	sun4i_scankeycodes[0 ] = KEY_VOLUMEDOWN; 			
	sun4i_scankeycodes[1 ] = KEY_VOLUMEDOWN;   
	sun4i_scankeycodes[2 ] = KEY_VOLUMEUP;        
	sun4i_scankeycodes[3 ] = KEY_BACK;    
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC; 
	sun4i_scankeycodes[6 ] = KEY_ENTER;        
	sun4i_scankeycodes[7 ] = KEY_RESERVED;
	}
	else if(!strcmp("BK7032-A13", mach_name)){
	sun4i_scankeycodes[0 ] = KEY_VOLUMEUP; //KEY_VOLUMEDOWN; 			//used  
	sun4i_scankeycodes[1 ] = KEY_VOLUMEDOWN;   
	sun4i_scankeycodes[2 ] = KEY_RESERVED;//KEY_VOLUMEUP;         //used  
	sun4i_scankeycodes[3 ] = KEY_MENU;//KEY_BACK;                 //used  
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC; 
	sun4i_scankeycodes[6 ] =KEY_BACK;//KEY_MENU;;                 //used  
	sun4i_scankeycodes[7 ] = KEY_RESERVED;

	}
	else if(!strcmp("T717-A13", mach_name)){
	sun4i_scankeycodes[0 ] = KEY_VOLUMEDOWN; 			
	sun4i_scankeycodes[1 ] = KEY_VOLUMEUP;   
	sun4i_scankeycodes[2 ] = KEY_VOLUMEUP;        
	sun4i_scankeycodes[3 ] = KEY_BACK;    
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC; 
	sun4i_scankeycodes[6 ] = KEY_ENTER;        
	sun4i_scankeycodes[7 ] = KEY_RESERVED;
	}
	else if(!strcmp("T720-A13", mach_name)){
	sun4i_scankeycodes[0 ] = KEY_PAGEDOWN; 			
	sun4i_scankeycodes[1 ] = KEY_PAGEUP;   
	sun4i_scankeycodes[2 ] = KEY_PAGEUP;        
	sun4i_scankeycodes[3 ] = KEY_MENU;    
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC; 
	sun4i_scankeycodes[6 ] = KEY_BACK;        
	sun4i_scankeycodes[7 ] = KEY_RESERVED;
	}
	else if(!strcmp("U7-A13", mach_name)){
	printk("choose U7-A13\n");
	sun4i_scankeycodes[0 ] = KEY_VOLUMEUP;     //used  
	sun4i_scankeycodes[1 ] = KEY_MENU;  
	sun4i_scankeycodes[2 ] = KEY_MENU;         
	sun4i_scankeycodes[3 ] = KEY_MENU;       
	sun4i_scankeycodes[4 ] = KEY_VOLUMEDOWN;     //used  
	sun4i_scankeycodes[5 ] = KEY_ESC;
	sun4i_scankeycodes[6 ] = KEY_HOME;      
	sun4i_scankeycodes[7 ] = KEY_RESERVED;
				
	}	
	else if(!strcmp("T801-A13", mach_name)){
		printk("choose T801-A13\n");
	sun4i_scankeycodes[0 ] = KEY_VOLUMEDOWN; 			
	sun4i_scankeycodes[1 ] = KEY_VOLUMEUP;   
	sun4i_scankeycodes[2 ] = KEY_VOLUMEUP;        
	sun4i_scankeycodes[3 ] = KEY_HOME;    
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC; 
	sun4i_scankeycodes[6 ] = KEY_BACK;        
	sun4i_scankeycodes[7 ] = KEY_RESERVED;	
	}
	else if(!strcmp("M858-A13", mach_name))
	{
	printk("choose M858-A13\n");
	sun4i_scankeycodes[0 ] = KEY_BACK;     
	sun4i_scankeycodes[1 ] = KEY_MENU;  
	sun4i_scankeycodes[2 ] = KEY_MENU;         
	sun4i_scankeycodes[3 ] = KEY_MENU;       
	sun4i_scankeycodes[4 ] = KEY_BACK;   
	sun4i_scankeycodes[5 ] = KEY_ESC;
	sun4i_scankeycodes[6 ] = KEY_HOME;      
	sun4i_scankeycodes[7 ] = KEY_RESERVED;
	}
	else if(!strcmp("L7-A13", mach_name))
	{
	sun4i_scankeycodes[0 ] = KEY_VOLUMEUP;     
	sun4i_scankeycodes[1 ] = KEY_VOLUMEDOWN;  
	sun4i_scankeycodes[2 ] = KEY_MENU;         
	sun4i_scankeycodes[3 ] = KEY_MENU;       
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC;
	sun4i_scankeycodes[6 ] = KEY_HOME;      
	sun4i_scankeycodes[7 ] = KEY_RESERVED;
	suspend_use=0;
	}
	else if(!strcmp("MD7008-A13", mach_name))
	{
	sun4i_scankeycodes[0 ] = KEY_VOLUMEUP;  //used  
	sun4i_scankeycodes[1 ] = KEY_VOLUMEDOWN;//used      
	sun4i_scankeycodes[2 ] = KEY_MENU;   
	sun4i_scankeycodes[3 ] = KEY_BACK;      //used 
	sun4i_scankeycodes[4 ] = KEY_BACK; 
	sun4i_scankeycodes[5 ] = KEY_ESC;
	sun4i_scankeycodes[6 ] = KEY_HOME;      //used  
	sun4i_scankeycodes[7 ] = KEY_RESERVED;

	}
	else{
	
	printk("err!========================sun4i-keyboard mach fail\n");
	printk("mach_name cannot mach anthing so choose default!\n");
	printk("choose M876-A13\n");
	sun4i_scankeycodes[0 ] = KEY_BACK;     
	sun4i_scankeycodes[1 ] = KEY_MENU;  
	sun4i_scankeycodes[2 ] = KEY_MENU;         
	sun4i_scankeycodes[3 ] = KEY_MENU;       
	sun4i_scankeycodes[4 ] = KEY_HOME;   
	sun4i_scankeycodes[5 ] = KEY_ESC;
	sun4i_scankeycodes[6 ] = KEY_HOME;      
	sun4i_scankeycodes[7 ] = KEY_RESERVED;
	
		
	}
	
	
#endif

	sun4ikbd_dev->name = INPUT_DEV_NAME;  
	sun4ikbd_dev->phys = "sun4ikbd/input0"; 
	sun4ikbd_dev->id.bustype = BUS_HOST;      
	sun4ikbd_dev->id.vendor = 0x0001;
	sun4ikbd_dev->id.product = 0x0001;
	sun4ikbd_dev->id.version = 0x0100;

#ifdef REPORT_REPEAT_KEY_BY_INPUT_CORE
	sun4ikbd_dev->evbit[0] = BIT_MASK(EV_KEY)|BIT_MASK(EV_REP);
	printk("REPORT_REPEAT_KEY_BY_INPUT_CORE is defined, support report repeat key value. \n");
#else
	sun4ikbd_dev->evbit[0] = BIT_MASK(EV_KEY);
#endif

	for (i = 0; i < KEY_MAX_CNT; i++)
		set_bit(sun4i_scankeycodes[i], sun4ikbd_dev->keybit);
	
#ifdef ONE_CHANNEL
	writel(LRADC_ADC0_DOWN_EN|LRADC_ADC0_UP_EN|LRADC_ADC0_DATA_EN,KEY_BASSADDRESS + LRADC_INTC);	
	writel(FIRST_CONCERT_DLY|LEVELB_VOL|KEY_MODE_SELECT|LRADC_HOLD_EN|ADC_CHAN_SELECT|LRADC_SAMPLE_125HZ|LRADC_EN,KEY_BASSADDRESS + LRADC_CTRL);
	//writel(FIRST_CONCERT_DLY|LEVELB_VOL|KEY_MODE_SELECT|ADC_CHAN_SELECT|LRADC_SAMPLE_62HZ|LRADC_EN,KEY_BASSADDRESS + LRADC_CTRL);

#else
#endif


	if (request_irq(SW_INT_IRQNO_LRADC, sun4i_isr_key, 0, "sun4ikbd", NULL)){
		err = -EBUSY;
		printk("request irq failure. \n");
		goto fail2;
	}

	err = input_register_device(sun4ikbd_dev);
	if (err)
		goto fail3;

	err = device_create_file(&sun4ikbd_dev->dev, &dev_attr_keydebug);
	if(err)
		printk("create file for keyboard err!\n");
if(suspend_use)	{
#ifdef CONFIG_HAS_EARLYSUSPEND	
	printk("==register_early_suspend =\n");
	keyboard_data = kzalloc(sizeof(*keyboard_data), GFP_KERNEL);
	if (keyboard_data == NULL) {
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	keyboard_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 3;	
	keyboard_data->early_suspend.suspend = sun4i_keyboard_suspend;
	keyboard_data->early_suspend.resume	= sun4i_keyboard_resume;
	register_early_suspend(&keyboard_data->early_suspend);
#endif
}
	return 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
 err_alloc_data_failed:
#endif
 fail3:	
	free_irq(SW_INT_IRQNO_LRADC, sun4i_isr_key);
 fail2:	
	input_free_device(sun4ikbd_dev);
 fail1:
     ;
#ifdef KEY_DEBUG
	printk("sun4ikbd_init failed. \n");
#endif

 return err;
}

static void __exit sun4ikbd_exit(void)
{
	if(suspend_use)	{
#ifdef CONFIG_HAS_EARLYSUSPEND	
	 unregister_early_suspend(&keyboard_data->early_suspend);	
#endif
}
	device_remove_file(&sun4ikbd_dev->dev, &dev_attr_keydebug);
	free_irq(SW_INT_IRQNO_LRADC, sun4i_isr_key);
	input_unregister_device(sun4ikbd_dev);
}

module_init(sun4ikbd_init);
module_exit(sun4ikbd_exit);


MODULE_AUTHOR(" <@>");
MODULE_DESCRIPTION("sun4i-keyboard driver");
MODULE_LICENSE("GPL");



