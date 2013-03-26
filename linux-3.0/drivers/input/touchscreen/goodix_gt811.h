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

#ifndef 	_LINUX_GOODIX_GT811_H
#define		_LINUX_GOODIX_GT811_H


#include <linux/i2c.h>
#include <linux/input.h>
#define fail    0
#define success 1

#define false   0
#define true    1

#define GT_IRQ_RISING       1
#define GT_IRQ_FALLING      0
#define INT_TRIGGER         GT_IRQ_FALLING
#define POLL_TIME           10        //actual query spacing interval:POLL_TIME+6

#define GOODIX_GT811_I2C_NAME "goodix_gt811"
#define GUITAR_GT811
//触摸屏的分辨率
#define GT811_MAX_HEIGHT 	7680	
#define GT811_MAX_WIDTH	 	5120
//显示屏的分辨率，根据具体平台更改，与触摸屏映射坐标相关


// gpio base address
#define PIO_BASE_ADDRESS             (0x01c20800)
#define PIO_RANGE_SIZE               (0x400)

#define IRQ_EINT21                   (21) 
#define IRQ_EINT29                   (29) 
#define IRQ_EINT11                 (11)
#define IRQ_EINT_USED_GOODIX  IRQ_EINT11

#define PIO_INT_STAT_OFFSET          (0x214)
#define PIO_INT_CTRL_OFFSET          (0x210)
#define PIO_INT_CFG1_OFFSET          (0x204)
#define PIO_INT_CFG2_OFFSET          (0x208)
#define PIO_INT_CFG3_OFFSET          (0x20c)

#define SHUTDOWN_PORT                ()
#define INT_PORT                     (SW_INT_IRQNO_PIO)

#define GOODIX_MULTI_TOUCH
#ifndef GOODIX_MULTI_TOUCH
	#define MAX_FINGER_NUM 5
#else
	#define MAX_FINGER_NUM 5				//最大支持手指数(<=5)
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

#define GT811_BORDER_ADJUST
//#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

enum finger_state {
#define FLAG_MASK 0x01
	FLAG_UP = 0,
	FLAG_DOWN = 1,
	FLAG_INVALID = 2,
};


struct point_node
{
	uint8_t id;
	//uint8_t retry;
	enum finger_state state;
	uint8_t pressure;
	unsigned int x;
	unsigned int y;
};
struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

/* Notice: This definition used by platform_data.
 * It should be move this struct info to platform head file such as plat/ts.h.
 * If not used in client, it will be NULL in function of goodix_ts_probe. 
 */ 
struct goodix_i2c_platform_data {
	uint32_t gpio_irq;			//IRQ port, use macro such as "gpio_to_irq" to get Interrupt Number.
	uint32_t irq_cfg;			//IRQ port config, must refer to master's Datasheet.
	uint32_t gpio_shutdown;		        //Shutdown port number
	uint32_t shutdown_cfg;		        //Shutdown port config
	uint32_t screen_width;		        //screen width
	uint32_t screen_height;		        //screen height
}; 

//#define GT811_DEBUG 1

#ifdef GT811_DEBUG
#define Ha_debug(fmt, arg...) printk(KERN_INFO "<ha--debug> " fmt, ##arg)
#else
#define Ha_debug(fmt, arg...)
#endif

#ifdef GT811_DEBUG
#define DEBUG(fmt, arg...) printk("<--GT-DEBUG-->"fmt, ##arg)
#else
#define DEBUG(fmt, arg...)
#endif

#ifdef GT811_DEBUG
#define NOTICE(fmt, arg...) printk("<--GT-NOTICE-->"fmt, ##arg)
#else
#define NOTICE(fmt, arg...)
#endif

#ifdef GT811_DEBUG
#define WARNING(fmt, arg...) printk("<--GT-WARNING-->"fmt, ##arg)
#else
#define WARNING(fmt, arg...)
#endif

#ifdef GT811_DEBUG
#define DEBUG_MSG(fmt, arg...) printk("<--GT msg-->"fmt, ##arg)
#else
#define DEBUG_MSG(fmt, arg...)
#endif

#ifdef GT811_DEBUG
#define DEBUG_UPDATE(fmt, arg...) printk("<--GT update-->"fmt, ##arg)
#else
#define DEBUG_UPDATE(fmt, arg...)
#endif 

#ifdef GT811_DEBUG
#define DEBUG_COOR(fmt, arg...) printk(fmt, ##arg)
#define DEBUG_COORD
#else
#define DEBUG_COOR(fmt, arg...)
#endif

#ifdef GT811_DEBUG
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

#endif /* _LINUX_GOODIX_TOUCH_H */

