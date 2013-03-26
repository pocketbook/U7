/*
 * \file        portpm.c
 * \brief       
 *
 * \version     1.0.0
 * \date        2012-08-29
 * \author      chenjian <chenjian@allwinnertech.com>
 *
 * Copyright (c) 2012 Allwinner Technology. All Rights Reserved.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>


#include "../sun5i_usb/include/sw_usb_config.h"

#ifdef CONFIG_HAS_EARLYSUSPEND    
#include <linux/pm.h>    
#include <linux/earlysuspend.h>
#endif

#define V_1A_DEFAULT        3550000    
#define C_1A_DEFAULT        10       
#define V_500MA_DEFAULT     0      
#define C_500MA_DEFAULT     10       
#define V_DISABLE_DEFAULT   3400000       
#define C_DISABLE_DEFAULT   5 

struct pm_gpio_t{
    __u32           hdle;
    user_gpio_set_t gpio_set;	
};

struct pm_cfg{
    int restrict_1a;
    int restrict_500ma;    
    int v_1a;
    int c_1a;
    int v_500ma;
    int c_500ma;
    int v_disable;
    int c_disable;
    struct pm_gpio_t ctrl_gpio;	
};


#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend standby;
#endif

static __u32 thread_run_flag = 0;
static __u32 thread_stopped_flag = 0;
static __u32 thread_suspend_flag = 0;

static __u32 power_status = 0;
static __s32 ctrlio_status = -1;
static __s32 ignore_usbc_num[3] = {0};

extern int get_hcd0_connect_status(void);
extern int get_ehci_connect_status(int usbc_no);
extern int get_ohci_connect_status(int usbc_no);

extern int hcd0_set_vbus(int is_on);
extern int ehci_set_vbus(int usbc_no, int is_on);
extern int ohci_set_vbus(int usbc_no, int is_on);
extern int hci_get_vbus_status(int usbc_no);
extern int hcd0_get_vbus_status(void);

struct pm_cfg pm_cfg;

static int (*do_pm)(int, int, int);

static struct class *portpm_class;
static struct device *portpm_dev;

static int get_pm_cfg(struct pm_cfg *cfg)
{
    s32 ret = 0;
    __u32 usb_wifi_used = 0;
    __s32 usb_wifi_usbc_num = 0;
    __s32 usb_wifi_usbc_init_state = -1;
    
    ret = script_parser_fetch("port_pm", "restrict_1a", (int *)&(cfg->restrict_1a), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:restrict_1a failed\n");
	}
	
	ret = script_parser_fetch("port_pm", "restrict_500ma", (int *)&(cfg->restrict_500ma), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:restrict_500ma failed\n");
	}
	
	ret = script_parser_fetch("port_pm", "v_1a", (int *)&(cfg->v_1a), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:v_1a failed\n");
	}
	
	ret = script_parser_fetch("port_pm", "c_1a", (int *)&(cfg->c_1a), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:c_1a failed\n");
	}
	
	ret = script_parser_fetch("port_pm", "v_500ma", (int *)&(cfg->v_500ma), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:v_500ma failed\n");
	}
	
	ret = script_parser_fetch("port_pm", "c_500ma", (int *)&(cfg->c_500ma), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:c_500ma failed\n");
	}
	
	ret = script_parser_fetch("port_pm", "v_disable", (int *)&(cfg->v_disable), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:v_disable failed\n");
	}
	
	ret = script_parser_fetch("port_pm", "c_disable", (int *)&(cfg->c_disable), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:c_disable failed\n");
	}
	
	ret = script_parser_fetch("port_pm", "ctrl_gpio", (int *)&(cfg->ctrl_gpio.gpio_set), 64);
	if(ret != 0){
		printk("ERR: get pm_cfg:ctrl_gpio failed\n");
	}
	else
	    printk("ctrl gpio_name = %s,\n"
	           "port = %d, port_num = %d,\n"
	           "mul_sel = %d, pull = %d,\n" 
	           "drv_level = %d, data = %d\n", 
	            cfg->ctrl_gpio.gpio_set.gpio_name, 
	            cfg->ctrl_gpio.gpio_set.port, cfg->ctrl_gpio.gpio_set.port_num,
	            cfg->ctrl_gpio.gpio_set.mul_sel, cfg->ctrl_gpio.gpio_set.pull,
	            cfg->ctrl_gpio.gpio_set.drv_level, cfg->ctrl_gpio.gpio_set.data);
	            
	/* ----------get usb_wifi_usbc_num------------- */
    ret = script_parser_fetch("usb_wifi_para", "usb_wifi_used", (int *)&usb_wifi_used, 64);
	if(ret != 0){
		printk("ERR: script_parser_fetch usb_wifi_usbc_num failed\n");
		ret = -ENOMEM;
		return ret;
	}
	if(usb_wifi_used){	    
	    char *usbc_num = NULL;
    	ret = script_parser_fetch("usb_wifi_para", "usb_wifi_usbc_num", (int *)&usb_wifi_usbc_num, 64);
    	if(ret != 0){
    		printk("ERR: script_parser_fetch usb_wifi_usbc_num failed\n");
    		ret = -ENOMEM;
    		return ret;
    	}    	

    	if(usb_wifi_usbc_num == 1)
    	    usbc_num = "usbc1";
    	else if(usb_wifi_usbc_num == 2)
    	    usbc_num = "usbc2";
    	else
    	    printk("wifi use usb num %d invalid\n", usb_wifi_usbc_num);

    	if(usbc_num){
        	ret = script_parser_fetch(usbc_num, "usb_host_init_state", (int *)&usb_wifi_usbc_init_state, 64);
        	if(ret != 0){
        		printk("ERR: script_parser_fetch usb_wifi_usbc_num failed\n");
        		ret = -ENOMEM;
        		return ret;    
            }
    	} 

    	if(usb_wifi_usbc_init_state == 0)
    	    ignore_usbc_num[usb_wifi_usbc_num] = 1;
	}
	
	return 0;
}

static void check_pm_cfg(struct pm_cfg *cfg)
{
    if(!cfg->v_1a)
        cfg->v_1a = V_1A_DEFAULT;

    if(!cfg->c_1a)
        cfg->c_1a = C_1A_DEFAULT;

    if(!cfg->v_500ma)
        cfg->v_500ma = V_500MA_DEFAULT; 

    if(!cfg->c_500ma)
        cfg->c_500ma = C_500MA_DEFAULT;  

    if(!cfg->v_disable)
        cfg->v_disable = V_DISABLE_DEFAULT;    

    if(!cfg->c_disable)
        cfg->c_disable = C_DISABLE_DEFAULT;  

}

static void printk_pm_cfg(struct pm_cfg *cfg)
{
    printk("restrict_1a = %d, restrict_500ma = %d\n"
           "v_1a = %d, c_1a = %d, v_500ma = %d, c_500ma = %d\n"
           "v_disable = %d, c_disable = %d\n"
           "ignore_usbc_num[0] = %d\n"
           "ignore_usbc_num[1] = %d\n"
           "ignore_usbc_num[2] = %d\n", 
               cfg->restrict_1a, cfg->restrict_500ma, 
               cfg->v_1a, cfg->c_1a, cfg->v_500ma, cfg->c_500ma,
               cfg->v_disable, cfg->c_disable,
               ignore_usbc_num[0], ignore_usbc_num[1], ignore_usbc_num[2]);
}

static int get_connect_status(void)
{
    int connect_status = 0;
    
    #if defined(CONFIG_USB_SW_SUN5I_HCD0)
    if(!ignore_usbc_num[0])
        connect_status |= get_hcd0_connect_status();
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_EHCI0)
    if(!ignore_usbc_num[1])
        connect_status |= get_ehci_connect_status(1);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_OHCI0)
    if(!ignore_usbc_num[1])
        connect_status |= get_ohci_connect_status(1);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_EHCI1)
    if(!ignore_usbc_num[2])
        connect_status |= get_ehci_connect_status(2);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_OHCI1)
    if(!ignore_usbc_num[2])
        connect_status |= get_ohci_connect_status(2);
    #endif

    return connect_status;
}

/*
value 
0 : restrict to 500ma
1 : restrict to 1a
*/
static int set_ctrl_gpio(int is_on)
{      
    int ret = 0;
    if(ctrlio_status == is_on)
        return 0;
    if(pm_cfg.ctrl_gpio.hdle){        
    	printk("set ctrl gpio %s\n", is_on ? "on" : "off");        
	    ret = gpio_write_one_pin_value(pm_cfg.ctrl_gpio.hdle, is_on, NULL);		    
    }

    ctrlio_status = is_on;
        
    return ret;
}

static int get_ctrl_gpio_status(void)
{    
    if(pm_cfg.ctrl_gpio.hdle)
        ctrlio_status = gpio_read_one_pin_value(pm_cfg.ctrl_gpio.hdle, NULL);	   
    
    return ctrlio_status;
}

static int get_vbus_status(void)
{
    int status = 0;

    #if defined(CONFIG_USB_SW_SUN5I_HCD0)
    if(!ignore_usbc_num[0])
        status |= hcd0_get_vbus_status();
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_EHCI0)
    if(!ignore_usbc_num[1])
        status |= hci_get_vbus_status(1);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_OHCI0)
    if(!ignore_usbc_num[1])
        status |= hci_get_vbus_status(1);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_EHCI1)
    if(!ignore_usbc_num[2])
        status |= hci_get_vbus_status(2);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_OHCI1)
    if(!ignore_usbc_num[2])
        status |= hci_get_vbus_status(2);
    #endif

    return status;
}

static int portpm_notifier(int on_off)
{
    char *poweron[2]    = { "USB_PORT_STATE=POWER ON", NULL };
	char *poweroff[2]   = { "USB_PORT_STATE=POWER OFF", NULL };	
	
	char **uevent_envp = on_off ? poweron : poweroff;
	printk("portpm send uevent %s\n", uevent_envp[0]);
    
	kobject_uevent_env(&portpm_dev->kobj, KOBJ_CHANGE, uevent_envp);

	return 0;
}

static int set_vbus(int on_off)
{            
    if(get_vbus_status() == on_off)
        return 0;
        
    printk("portpm set vbus %s\n", on_off ? "on" : "off");  
    
    if(!thread_suspend_flag)
        portpm_notifier(on_off);
        
    power_status = on_off;
    
    #if defined(CONFIG_USB_SW_SUN5I_HCD0)
    if(!ignore_usbc_num[0])
        hcd0_set_vbus(on_off);      
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_EHCI0)    
    if(!ignore_usbc_num[1])
        ehci_set_vbus(1, on_off);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_OHCI0)    
    if(!ignore_usbc_num[1])
        ohci_set_vbus(1, on_off);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_EHCI1)    
    if(!ignore_usbc_num[2])
        ehci_set_vbus(2, on_off);
    #endif

    #if defined(CONFIG_USB_SW_SUN5I_OHCI1)    
    if(!ignore_usbc_num[2])
        ohci_set_vbus(2, on_off);
    #endif

    return 0;
}

#define BOUNCE_THRESHOLD 2
static __u32 cnt_1a = BOUNCE_THRESHOLD - 1;
static __u32 cnt_500ma = BOUNCE_THRESHOLD - 1;
static __u32 cnt_on = BOUNCE_THRESHOLD - 1;

static int do_1a_500ma(int connect, int voltage, int capacity)
{
    if(!connect){
        cnt_on = 0;        
        if(voltage > pm_cfg.v_1a && capacity > pm_cfg.c_1a){
            cnt_1a++;
            cnt_500ma = 0;
            if(cnt_1a > BOUNCE_THRESHOLD){
                set_vbus(1);
                set_ctrl_gpio(1);
            }
        }
        else if(voltage > pm_cfg.v_500ma && capacity > pm_cfg.c_500ma){
            cnt_1a = 0;
            cnt_500ma++;            
            if(cnt_500ma > BOUNCE_THRESHOLD){
                set_vbus(1);
                set_ctrl_gpio(0);
            }
        }
        else{
            cnt_1a = 0;
            cnt_500ma = 0;
            set_vbus(0);            
        }            
    }
    else{        
        cnt_1a = 0;
        cnt_500ma = 0;        
        if(voltage > pm_cfg.v_disable && capacity > pm_cfg.c_disable){
            cnt_on++;            
            if(cnt_on > BOUNCE_THRESHOLD)
                set_vbus(1);  
        }
        else{
            cnt_on = 0;
            set_vbus(0); 
        }
    }
    return 0;
}

static int do_1a(int connect, int voltage, int capacity)
{        
    if(!connect){
        cnt_on = 0;        
        if(voltage > pm_cfg.v_1a && capacity > pm_cfg.c_1a){
            cnt_1a++;        
            if(cnt_1a > BOUNCE_THRESHOLD){
                set_vbus(1);
                set_ctrl_gpio(1);
            }
        }        
        else{
            cnt_1a = 0;           
            set_vbus(0);          
        }            
    }
    else{        
        cnt_1a = 0;        
        if(voltage > pm_cfg.v_disable && capacity > pm_cfg.c_disable){
            cnt_on++;            
            if(cnt_on > BOUNCE_THRESHOLD)
                set_vbus(1);  
        }
        else{
            cnt_on = 0;
            set_vbus(0); 
        }         
    }    
    
    return 0;
}

static int do_500ma(int connect, int voltage, int capacity)
{    
    if(!connect){
        cnt_on = 0;        
        if(voltage > pm_cfg.v_500ma && capacity > pm_cfg.c_500ma){
            cnt_500ma++;            
            if(cnt_500ma > BOUNCE_THRESHOLD){
                set_vbus(1);
                set_ctrl_gpio(0);
            }
        }
        else{
            cnt_500ma = 0;
            set_vbus(0);          
        }            
    }
    else{        
        cnt_500ma = 0;        
        if(voltage > pm_cfg.v_disable && capacity > pm_cfg.c_disable){
            cnt_on++;            
            if(cnt_on > BOUNCE_THRESHOLD)
                set_vbus(1);  
        }
        else{
            cnt_on = 0;
            set_vbus(0); 
        }         
    }            
    
    return 0;
}

#define BUFLEN 32

static int get_battery_status(void)
{
    struct file *filep;
    loff_t pos;
    char buf[BUFLEN];
    
    filep=filp_open("/sys/class/power_supply/battery/present", O_RDONLY, 0);        
    if(IS_ERR(filep)){
        printk("open present fail\n");
        return 0;
    }
   
    pos = 0;
    vfs_read(filep, (char __user *)buf, BUFLEN, &pos);
    filp_close(filep, 0);

    if(!strncmp((const char *)buf, "0", 1))
        return 0;
    else
        return 1;   
}

static int get_charge_status(void)
{
    struct file *filep;
    loff_t pos;
    char buf[BUFLEN];
    
    filep=filp_open("/sys/class/power_supply/battery/status", O_RDONLY, 0);        
    if(IS_ERR(filep)){
        printk("open status fail\n");
        return 0;
    }
   
    pos = 0;
    vfs_read(filep, (char __user *)buf, BUFLEN, &pos);
    filp_close(filep, 0);

    if(!strncmp((const char *)buf, "Charging", 8) ||
            !strncmp((const char *)buf, "Full", 4))
        return 1;
    else 
        return 0;
}

static int get_voltage(void)
{
    struct file *filep;
    loff_t pos;
    char buf[BUFLEN];
    int ret, voltage;
    
    filep=filp_open("/sys/class/power_supply/battery/voltage_now", O_RDONLY, 0);        
    if(IS_ERR(filep)){
        printk("open voltage fail\n");
        return 0;
    }
   
    pos = 0;
    vfs_read(filep, (char __user *)buf, BUFLEN, &pos);
    filp_close(filep, 0);
    
    ret = sscanf(buf, "%d\n", &voltage);
    if(ret != 1)
        return 0;
    else
        return voltage;
}

static int get_capacity(void)
{
    struct file *filep;
    loff_t pos;
    char buf[BUFLEN];
    int ret, capacity;
    
    filep=filp_open("/sys/class/power_supply/battery/capacity", O_RDONLY, 0);        
    if(IS_ERR(filep)){
        printk("open capacity fail\n");
        return 0;
    }
    pos = 0;
    vfs_read(filep, (char __user *)buf, BUFLEN, &pos);
    filp_close(filep, 0); 
    ret = sscanf(buf, "%d\n", &capacity);
    if(ret != 1)
        return 0;
    else
        return capacity;
}

static int usb_port_pm_thread(void * pArg)
{
    int connect, old_connect = 0, voltage, old_voltage = 0, capacity, old_capacity = 0;
            
    while(thread_run_flag){
        msleep(1000);
        
        if(thread_suspend_flag)
            continue;        

        if(!get_battery_status()){//battery not exist            
            set_vbus(1);
            if(pm_cfg.restrict_1a)
                set_ctrl_gpio(1);
            else
                set_ctrl_gpio(0);
            continue;
        }          
        
        if(get_charge_status()){//charging or full            
            set_vbus(1);
            if(pm_cfg.restrict_1a)
                set_ctrl_gpio(1);
            else
                set_ctrl_gpio(0);
            continue;
        }      
            
        voltage = get_voltage();
        if(voltage < 2000000)
            continue;
        
        capacity = get_capacity();
        connect = get_connect_status();
        if(voltage != old_voltage || capacity != old_capacity){  
            printk("connect = %d, voltage = %d, capacity = %d\n", connect, voltage, capacity);            
            old_connect = connect;
            old_voltage = voltage;
            old_capacity = capacity;
            do_pm(connect, voltage, capacity);
        }                
    }

    thread_stopped_flag = 1;

    return 0;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{	
	char *state = NULL;
	
	if (power_status)
		state = "POWER ON";
	else
		state = "POWER OFF";
	
	return sprintf(buf, "%s\n", state);
}
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);


#ifdef CONFIG_PM

static int portpm_suspend(struct device *dev)
{
    printk("portpm_suspend\n");
    thread_suspend_flag = 1;
    set_vbus(0);
    set_ctrl_gpio(0);
    return 0;
}

static int portpm_resume(struct device *dev)
{
    printk("portpm_resume\n");
    thread_suspend_flag = 0;
    return 0;
}

static const struct dev_pm_ops portpm_pmops = {
	.suspend	= portpm_suspend,
	.resume		= portpm_resume,
};

#define PORTPM_PMOPS  &portpm_pmops

#else

#define PORTPM_PMOPS NULL

#endif

static int portpm_probe(struct platform_device *pdev)
{
    int ret;
    struct task_struct *pm_task = NULL;
    printk("portpm probe begin\n");      
    
    get_pm_cfg(&pm_cfg);
    //check_pm_cfg(&pm_cfg);
    printk_pm_cfg(&pm_cfg);
    
    pm_cfg.ctrl_gpio.hdle = gpio_request(&pm_cfg.ctrl_gpio.gpio_set, 1);
    if(pm_cfg.ctrl_gpio.hdle){
        /* set config, ouput */
        gpio_set_one_pin_io_status(pm_cfg.ctrl_gpio.hdle, 1, NULL);        
        /* default is pull down */
        gpio_set_one_pin_pull(pm_cfg.ctrl_gpio.hdle, 2, NULL);
    }else    
        printk("request ctrl gpio failed\n");   
        
    if(pm_cfg.restrict_1a && pm_cfg.restrict_500ma){
        do_pm = do_1a_500ma;
    }
    else if(pm_cfg.restrict_1a){
        do_pm = do_1a;
    }
    else if(pm_cfg.restrict_500ma){
        do_pm = do_500ma;
    }
    else{
        do_pm = do_1a;
    }        

    //create sysfs file
    portpm_class = class_create(THIS_MODULE, "usb_port_pm");
    if (IS_ERR(portpm_class))
        return PTR_ERR(portpm_class);
        
    portpm_dev = device_create(portpm_class, NULL,
                    MKDEV(0, 0), NULL, "port_pm");
    if (IS_ERR(portpm_dev)){
        printk("portpm_dev create failed\n");     
        class_destroy(portpm_class);
        return PTR_ERR(portpm_dev);
    }

    ret = device_create_file(portpm_dev, &dev_attr_state);
    if (ret) {
        printk("portpm_dev create file failed\n");   
        device_destroy(portpm_class, portpm_dev->devt);
        class_destroy(portpm_class);
        return ret;
    }
    
    thread_run_flag = 1;
    thread_stopped_flag = 0;
    pm_task = kthread_create(usb_port_pm_thread, &pm_cfg, "usb-port-power-management");
    if(IS_ERR(pm_task)){
        printk("ERR: usb-port-power-management thread create failed\n");
        return -1;
    }
 
    wake_up_process(pm_task);
    
    return 0;
}

static int portpm_remove(struct platform_device *pdev)
{
    thread_run_flag = 0;
	while(!thread_stopped_flag){
		printk("waitting for usb-port-power-management thread stop\n");
		msleep(10);
	}
	do_pm = NULL;
	if(pm_cfg.ctrl_gpio.hdle){
		gpio_release(pm_cfg.ctrl_gpio.hdle, 0);
		pm_cfg.ctrl_gpio.hdle = 0;
	}	
}

static struct platform_driver portpm_driver = {
	.probe		= portpm_probe,
	.remove		= portpm_remove,	
	.driver		= {
		.name	= "port_pm",
		.bus	= &platform_bus_type,
		.owner	= THIS_MODULE,
		.pm	    = PORTPM_PMOPS,
	},
};

static struct platform_device portpm_device = {
	.name	= "port_pm",
	.id		= -1,
	.dev = {
		.platform_data	= &pm_cfg,
	},
};

static int __init portpm_init(void)
{
    int ret;
    printk("portpm init---------\n"); 
    platform_device_register(&portpm_device);
    platform_driver_register(&portpm_driver);
	return 0;
}

static void __exit portpm_exit(void)
{
    printk("portpm exit---------\n");
    platform_device_unregister(&portpm_device);
    platform_driver_unregister(&portpm_driver);
}


late_initcall(portpm_init);
module_exit(portpm_exit);

