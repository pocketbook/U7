#ifndef __LINUX_CP2526_H__
#define __LINUX_CP2526_H__



#define HV_NAME	"hv_keypad"

struct cp2526_keypad_platform_data{
	u16	intr;		/* irq number	*/
};

#if 0
struct touchkey_button_data{
	char *desc;//keycode description
	char key_data;	//touch key I2C read key data
	int keycode;	//key code
	int keycode_long_press;//long press key code
};


struct cp2526_platform_data {
	u16 model;
	struct touchkey_button_data *buttons;
	int button_num;
	int (*io_init)(void);
	void (*io_deinit)(void);
};
#endif

// gpio base address
#define PIO_BASE_ADDRESS             (0x01c20800)
#define PIO_RANGE_SIZE               (0x400)

#define PIOA_CFG1_REG    (PIO_BASE_ADDRESS+0x4)
#define PIOA_DATA        (PIO_BASE_ADDRESS+0x10)  
#define DELAY_PERIOD     (20)




#define IRQ_EINT19                   (19) 
#define IRQ_EINT_USED_TKEY  IRQ_EINT19

#define PIO_INT_STAT_OFFSET          (0x214)
#define PIO_INT_CTRL_OFFSET          (0x210)
#define PIO_INT_CFG1_OFFSET          (0x204)
#define PIO_INT_CFG2_OFFSET          (0x208)
#define PIO_INT_CFG3_OFFSET          (0x20c)
#define PIO_INT_USED_CFG  PIO_INT_CFG2_OFFSET

#define SHUTDOWN_PORT                ()
#define INT_PORT                     (SW_INT_IRQNO_PIO)



#endif //__LINUX_CP2526_H__

