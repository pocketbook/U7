/*
 * include/linux/goodix_touch.h
 *
 * Copyright (C) 2008 Goodix, Inc.
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
 */

#ifndef 	_LINUX_ZET6221_H
#define		_LINUX_ZET6221_H


#include <linux/i2c.h>
#include <linux/input.h>
#define fail    0
#define success 1

#define false   0
#define true    1

#define ZET6221_I2C_NAME "zet6221-ts"
#define IRQ_EINT11                 (11)
#define IRQ_EINT_USED_ZET6221  IRQ_EINT11
#define INT_PORT                     (SW_INT_IRQNO_PIO)

#define ZET6221_MULTI_TOUCH
#ifndef ZET6221_MULTI_TOUCH
	#define MAX_FINGER_NUM 5
#else
	#define MAX_FINGER_NUM 5				//鏈�澶ф敮鎸佹墜鎸囨暟(<=5)
#endif
#if defined(INT_PORT)
	#if MAX_FINGER_NUM <= 3
	#define READ_BYTES_NUM 1+2+MAX_FINGER_NUM*5
	#elif MAX_FINGER_NUM == 4
	#define READ_BYTES_NUM 1+28
	#elif MAX_FINGER_NUM == 5
	#define READ_BYTES_NUM 1+34
	#endif
#else	
	#define READ_BYTES_NUM 1+34
#endif

//#define ZET6221_BORDER_ADJUST
//#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)


//#define ZET6221_DEBUG 0

#ifdef ZET6221_DEBUG
#define DEBUG(fmt, arg...) printk("<--DEBUG-->"fmt, ##arg)
#else
#define DEBUG(fmt, arg...)
#endif

#ifdef ZET6221_DEBUG_PROBE
#define DEBUG_PROBE(fmt, arg...) printk("<--DEBUG-->"fmt, ##arg)
#else
#define DEBUG_PROBE(fmt, arg...)
#endif

#ifdef ZET6221_DEBUG
#define NOTICE(fmt, arg...) printk("<--GT-NOTICE-->"fmt, ##arg)
#else
#define NOTICE(fmt, arg...)
#endif

#ifdef ZET6221_DEBUG
#define WARNING(fmt, arg...) printk("<--GT-WARNING-->"fmt, ##arg)
#else
#define WARNING(fmt, arg...)
#endif

#ifdef ZET6221_DEBUG
#define DEBUG_MSG(fmt, arg...) printk("<--GT msg-->"fmt, ##arg)
#else
#define DEBUG_MSG(fmt, arg...)
#endif

#ifdef ZET6221_DEBUG
#define DEBUG_UPDATE(fmt, arg...) printk("<--GT update-->"fmt, ##arg)
#else
#define DEBUG_UPDATE(fmt, arg...)
#endif 

#ifdef ZET6221_DEBUG
#define DEBUG_COOR(fmt, arg...) printk(fmt, ##arg)
#define DEBUG_COORD
#else
#define DEBUG_COOR(fmt, arg...)
#endif

#ifdef ZET6221_DEBUG
#define DEBUG_ARRAY(array, num)   do{\
                                   int i;\
                                   u8* a = array;\
                                   for (i = 0; i < (num); i++)\
                                   {\
                                       printk("%02x   ", (a)[i]);\
                                       if ((i + 1 ) %10 == 0)\
                                       {\
                                           printk("\n");\
                                       }\
                                   }\
                                   printk("\n");\
                                  }while(0)
#else
#define DEBUG_ARRAY(array, num) 
#endif 

#define ADDR_MAX_LENGTH     2
#define ADDR_LENGTH         ADDR_MAX_LENGTH

/*tp兼容所用宏x1,x2,y1,y2*/
#define IRQ_TP                			 (29)
#define TP_BASSADDRESS         	 (0xf1c25000)
#define TP_CTRL0               		 (0x00)
#define TP_CTRL1              		 (0x04)
#define TP_CTRL2               		 (0x08)
#define TP_CTRL3               		 (0x0c)
#define TP_INT_FIFOC           		 (0x10)
#define TP_INT_FIFOS           		 (0x14)
#define TP_TPR                			 (0x18)
#define TP_CDAT                		 (0x1c)
#define TEMP_DATA             		 (0x20)
#define TP_DATA               		 (0x24)


#define ADC_FIRST_DLY         		 (0x1<<24)
#define ADC_FIRST_DLY_MODE       (0x1<<23) 
#define ADC_CLK_SELECT                (0x0<<22)
#define ADC_CLK_DIVIDER              (0x2<<20)    
//#define CLK                    (6)
#define CLK                    			 (7)
#define FS_DIV                 			 (CLK<<16)
#define ACQ                    			 (0x3f)
#define T_ACQ                  			 (ACQ)

#define STYLUS_UP_DEBOUNCE       (0<<12)
#define STYLUS_UP_DEBOUCE_EN   (0<<9)
#define TOUCH_PAN_CALI_EN         (1<<6)
#define TP_DUAL_EN            		 (1<<5)
#define TP_MODE_EN             		 (1<<4)
#define TP_ADC_SELECT          	 (0<<3)
#define ADC_CHAN_SELECT        	 (0)

//#define TP_SENSITIVE_ADJUST    (0xf<<28)
#define TP_SENSITIVE_ADJUST       (0xf<<28)       //mark by young for test angda 5" 0xc
#define TP_MODE_SELECT               (0x1<<26)
#define PRE_MEA_EN                       (0x1<<24)
#define PRE_MEA_THRE_CNT           (0x1f40<<0)         //0x1f40


#define FILTER_EN              		 (1<<2)
#define FILTER_TYPE            		 (0x01<<0)

#define TP_DATA_IRQ_EN         	 (1<<16)
#define TP_DATA_XY_CHANGE         (0<<13)       //tp_exchange_x_y
#define TP_FIFO_TRIG_LEVEL          (3<<8)
#define TP_FIFO_FLUSH                   (1<<4)
#define TP_UP_IRQ_EN           		  (1<<1)
#define TP_DOWN_IRQ_EN         	  (1<<0)

#define FIFO_DATA_PENDING      	  (1<<16)
#define TP_UP_PENDING          	  (1<<1)
#define TP_DOWN_PENDING        	  (1<<0)

#define SINGLE_TOUCH_MODE      	   (1)
#define CHANGING_TO_DOUBLE_TOUCH_MODE (2)
#define DOUBLE_TOUCH_MODE      	   (3)
#define UP_TOUCH_MODE           	   (4)

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
/*
R108=1.5K      X=560   Y=2067  秋田微 
R108=4.7K      X=1366  Y=2068  平波 
R108=12K       X=2269  Y=2067  瑞视光电 
R108=27K       X=3025  Y=2067  诚信 
R108=56K       X=3507  Y=2067  未定义 
*/
static void getxy(int *x0,int *y0)
{	
#if 1	
	unsigned int reg_val;
	int x=0,y=0,breaktime=0;
	int reg_fifoc;
	int trytime=0;	
	tp_init();	
	while(1)
	{
		
		reg_val  = readl(TP_BASSADDRESS + TP_INT_FIFOS);
		if(reg_val&FIFO_DATA_PENDING)
		{
			x+=readl(TP_BASSADDRESS + TP_DATA);
                   
		      y+=readl(TP_BASSADDRESS + TP_DATA); 
		     
		      readl(TP_BASSADDRESS + TP_DATA);
		     
		      readl(TP_BASSADDRESS + TP_DATA); 
      
		      //flush fifo
		      reg_fifoc = readl(TP_BASSADDRESS+TP_INT_FIFOC);
		      reg_fifoc |= TP_FIFO_FLUSH;
		      writel(reg_fifoc, TP_BASSADDRESS+TP_INT_FIFOC);       
		      writel(reg_val&FIFO_DATA_PENDING,TP_BASSADDRESS + TP_INT_FIFOS);
		      breaktime++;
		}
				
		trytime++;
		mdelay(1);
		if(breaktime>=10)
			break;
		if(trytime>=1000)
		{
			DEBUG_PROBE("trytime>=%d\n",trytime);
			break;
		}
			
	}
	if(breaktime>0){
	x=x/breaktime;
	y=y/breaktime;
	}
	printk("%s:x:%d y:%d\n",__func__,x,y);

	*x0=x;*y0=y;
#endif
/**********************/
}

#endif /* _LINUX_GOODIX_TOUCH_H */

