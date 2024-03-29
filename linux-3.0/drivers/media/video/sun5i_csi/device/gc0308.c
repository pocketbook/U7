/*
 * A V4L2 driver for GalaxyCore GC0308 cameras.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>//linux-3.0
#include <linux/io.h>
//#include <mach/gpio_v2.h>
#include <mach/sys_config.h>
#include <linux/regulator/consumer.h>
#include <mach/system.h>
#include "../../../../power/axp_power/axp-gpio.h"
#if defined CONFIG_ARCH_SUN4I
#include "../include/sun4i_csi_core.h"
#include "../include/sun4i_dev_csi.h"
#elif defined CONFIG_ARCH_SUN5I
#include "../include/sun5i_csi_core.h"
#include "../include/sun5i_dev_csi.h"
#endif
#include <linux/mutex.h>
MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for GalaxyCore GC0308 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN   		0
#if(DEV_DBG_EN == 1)		
#define csi_dev_dbg(x,arg...) printk(KERN_INFO"[CSI_DEBUG][GC0308]"x,##arg)
#else
#define csi_dev_dbg(x,arg...) 
#endif
#define csi_dev_err(x,arg...) printk(KERN_INFO"[CSI_ERR][GC0308]"x,##arg)
#define csi_dev_print(x,arg...) printk(KERN_INFO"[CSI][GC0308]"x,##arg)

#define MCLK (24*1000*1000)
#define VREF_POL	CSI_HIGH
#define HREF_POL	CSI_HIGH
#define CLK_POL		CSI_RISING
#define IO_CFG		0						//0 for csi0

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0


#define V4L2_IDENT_SENSOR 0x0308

#define REG_TERM 0xff
#define VAL_TERM 0xff


#define REG_ADDR_STEP 1
#define REG_DATA_STEP 1
#define REG_STEP 			(REG_ADDR_STEP+REG_DATA_STEP)

//extern struct mutex videodev_lock_test;
//extern int videodev_lock_num;
extern unsigned first_flag ;
/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define VGA_WIDTH		640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH		352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 20

/*
 * The gc0308 sits on i2c with ID 0x42
 */
#define I2C_ADDR 0x42

/* Registers */

/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */
__csi_subdev_info_t ccm_info_con = 
{
	.mclk 	= MCLK,
	.vref 	= VREF_POL,
	.href 	= HREF_POL,
	.clock	= CLK_POL,
	.iocfg	= IO_CFG,
};

struct sensor_info {
	struct v4l2_subdev sd;
	struct sensor_format_struct *fmt;  /* Current format */
	__csi_subdev_info_t *ccm_info;
	int	width;
	int	height;
	int brightness;
	int	contrast;
	int saturation;
	int hue;
	int hflip;
	int vflip;
	int gain;
	int autogain;
	int exp;
	enum v4l2_exposure_auto_type autoexp;
	int autowb;
	enum v4l2_whiteblance wb;
	enum v4l2_colorfx clrfx;
	enum v4l2_flash_mode flash_mode;
	u8 clkrc;			/* Clock divider value */
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}


struct regval_list {
	unsigned char reg_num[REG_ADDR_STEP];
	unsigned char value[REG_DATA_STEP];
};


/*
 * The default register settings
 *
 */

static struct regval_list sensor_default_regs_24M[] = {
{{0xfe},{0x80}},
{{0xff},{0x64}},	//reset delay 100ms
{{0xfe},{0x00}},																					 
//MCLK=24MHz 10fps
{{0x0f},{0x01}},	 //0x00                                        
{{0x01},{0x32}},   //0x6a                                        
{{0x02},{0x70}},   //0x70                                        
{{0xe2},{0x00}},                                           
{{0xe3},{0x78}},                                           
{{0xe4},{0x02}},                                           
{{0xe5},{0x58}},   //20fps                                         
{{0xe6},{0x02}},                                           
{{0xe7},{0xd0}},                                           
{{0xe8},{0x03}},                                           
{{0xe9},{0x48}},                                           
{{0xea},{0x03}},                                           
{{0xeb},{0xc0}},  //12.5 fps
}; 
 
static struct regval_list sensor_default_regs[] = {

	// Initail Sequence Write In.

	/*init registers code.*/
	{{0xfe },{0x80}},  	

	{{0xfe },{0x00}},        // set page0


	{{0xd2 },{0x10}},  // close AEC
	{{0x22 },{0x55}},  // close AWB

	{{0x5a },{0x56}},
	{{0x5b },{0x40}},
	{{0x5c },{0x4a}},			

	{{0x22 },{0x57}}, // Open AWB

	{{0x01 },{0xfa}},
	{{0x02 },{0x70}},
	{{0x0f },{0x01}},

	{{0x03 },{0x01}},
	{{0x04 },{0x2c}},

	{{0xe2 },{0x00}},	//anti-flicker step [11:8]
	{{0xe3 },{0x78}},  //anti-flicker step [7:0]

	{{0xe4 },{0x02}},  //exp level 1  16.67fps
	{{0xe5 },{0x58}},
	{{0xe6 },{0x02}},  //exp level 2  12.5fps
	{{0xe7 },{0xd0}},
	{{0xe8 },{0x03}},  //exp level 3  8.33fps
	{{0xe9 },{0x48}},
	{{0xea },{0x03}},  //exp level 4  4.00fps
	{{0xeb },{0xc0}},

	{{0x05 },{0x00}},
	{{0x06 },{0x00}},
	{{0x07 },{0x00}},
	{{0x08 },{0x00}},
	{{0x09 },{0x01}},
	{{0x0a },{0xe8}},
	{{0x0b },{0x02}},
	{{0x0c },{0x88}},
	{{0x0d },{0x02}},
	{{0x0e },{0x02}},
	{{0x10 },{0x22}},
	{{0x11 },{0xfd}},
	{{0x12 },{0x2a}},
	{{0x13 },{0x00}},

	{{0x14 },{0x11}},// change direction  10:normal , 11:H SWITCH,12: V SWITCH, 13:H&V SWITCH   JAMES ADDED

	{{0x15 },{0x0a}},
	{{0x16 },{0x05}},
	{{0x17 },{0x01}},
	{{0x18 },{0x44}},
	{{0x19 },{0x44}},
	{{0x1a },{0x1e}},
	{{0x1b },{0x00}},
	{{0x1c },{0xc1}},
	{{0x1d },{0x08}},
	{{0x1e },{0x60}},
	{{0x1f },{0x13}}, //pad drv ,00 03 13 1f 3f james remarked


	{{0x20 },{0xff}},
	{{0x21 },{0xf8}},
	{{0x22 },{0x57}},
	{{0x24 },{0xa2}},  // yuv order
	{{0x25 },{0x0f}},
	     
	//output sync_mode       
	{{0x26 },{0x03}},//vsync  maybe need changed, value is 0x02
	{{0x2f },{0x01}},
	{{0x30 },{0xf7}},
	{{0x31 },{0x50}},
	{{0x32 },{0x00}},
	{{0x39 },{0x04}},
	{{0x3a },{0x18}},
	{{0x3b },{0x20}},
	{{0x3c },{0x00}},
	{{0x3d },{0x00}},
	{{0x3e },{0x00}},
	{{0x3f },{0x00}},
	{{0x50 },{0x10}},
	{{0x53 },{0x82}},
	{{0x54 },{0x80}},
	{{0x55 },{0x80}},
	{{0x56 },{0x82}},
	{{0x8b },{0x40}},
	{{0x8c },{0x40}},
	{{0x8d },{0x40}},
	{{0x8e },{0x2e}},
	{{0x8f },{0x2e}},
	{{0x90 },{0x2e}},
	{{0x91 },{0x3c}},
	{{0x92 },{0x50}},
	{{0x5d },{0x12}},
	{{0x5e },{0x1a}},
	{{0x5f },{0x24}},
	{{0x60 },{0x07}},
	{{0x61 },{0x15}},
	{{0x62 },{0x08}},
	{{0x64 },{0x03}},
	{{0x66 },{0xe8}},
	{{0x67 },{0x86}},
	{{0x68 },{0xa2}},
	{{0x69 },{0x18}},
	{{0x6a },{0x0f}},
	{{0x6b },{0x00}},
	{{0x6c },{0x5f}},
	{{0x6d },{0x8f}},
	{{0x6e },{0x55}},
	{{0x6f },{0x38}},
	{{0x70 },{0x15}},
	{{0x71 },{0x33}},
	{{0x72 },{0xdc}},
	{{0x73 },{0x80}},
	{{0x74 },{0x02}},
	{{0x75 },{0x3f}},
	{{0x76 },{0x02}},
	{{0x77 },{0x36}},
	{{0x78 },{0x88}},
	{{0x79 },{0x81}},
	{{0x7a },{0x81}},
	{{0x7b },{0x22}},
	{{0x7c },{0xff}},
	{{0x93 },{0x48}},
	{{0x94 },{0x00}},
	{{0x95 },{0x05}},
	{{0x96 },{0xe8}},
	{{0x97 },{0x40}},
	{{0x98 },{0xf0}},
	{{0xb1 },{0x38}},
	{{0xb2 },{0x38}},
	{{0xbd },{0x38}},
	{{0xbe },{0x36}},
	{{0xd0 },{0xc9}},
	{{0xd1 },{0x10}},
	//{{0xd2 },{0x90}},
	{{0xd3 },{0x80}},
	{{0xd5 },{0xf2}},
	{{0xd6 },{0x16}},
	{{0xdb },{0x92}},
	{{0xdc },{0xa5}},
	{{0xdf },{0x23}},
	{{0xd9 },{0x00}},
	{{0xda },{0x00}},
	{{0xe0 },{0x09}},
	{{0xec },{0x20}},
	{{0xed },{0x04}},
	{{0xee },{0xa0}},
	{{0xef },{0x40}},
	{{0x80 },{0x03}},
	{{0x80 },{0x03}},
	///////gamma/////
	{{0x9F },{0x10}},
	{{0xA0 },{0x20}},
	{{0xA1 },{0x38}},
	{{0xA2 },{0x4E}},
	{{0xA3 },{0x63}},
	{{0xA4 },{0x76}},
	{{0xA5 },{0x87}},
	{{0xA6 },{0xA2}},
	{{0xA7 },{0xB8}},
	{{0xA8 },{0xCA}},
	{{0xA9 },{0xD8}},
	{{0xAA },{0xE3}},
	{{0xAB },{0xEB}},
	{{0xAC },{0xF0}},
	{{0xAD },{0xF8}},
	{{0xAE },{0xFD}},
	{{0xAF },{0xFF}},
/*************************
{{0x9F}, {0x0B}},   //case 1:      //smallest gamma 
{{0xA0},{0x16}},   
{{0xA1},{0x29}},   
{{0xA2},{0x3C}},   
{{0xA3},{0x4F}},   
{{0xA4},{0x5F}},   
{{0xA5},{0x6F}},   
{{0xA6},{0x8A}},   
{{0xA7},{0x9F}},   
{{0xA8},{0xB4}},   
{{0xA9},{0xC6}},   
{{0xAA},{0xD3}},   
{{0xAB},{0xDD}},    
{{0xAC},{0xE5}},    
{{0xAD},{0xF1}},   
{{0xAE},{0xFA}},   
{{0xAF},{0xFF}},   	

		
{{0x9F},{ 0x0E}},   //case 2:	
{{0xA0},{ 0x1C}},   
{{0xA1},{ 0x34}},   
{{0xA2},{ 0x48}},   
{{0xA3},{ 0x5A}},   
{{0xA4},{ 0x6B}},   
{{0xA5},{ 0x7B}},   
{{0xA6},{ 0x95}},   
{{0xA7},{ 0xAB}},   
{{0xA8},{ 0xBF}},   
{{0xA9},{ 0xCE}},   
{{0xAA},{ 0xD9}},   
{{0xAB},{ 0xE4}},    
{{0xAC},{ 0xEC}},   
{{0xAD},{ 0xF7}},   
{{0xAE},{ 0xFD}},   
{{0xAF},{ 0xFF}},   



{{0x9F},{ 0x10}},   ///case 3:
{{0xA0},{ 0x20}},   
{{0xA1},{ 0x38}},   
{{0xA2},{ 0x4E}},   
{{0xA3},{ 0x63}},   
{{0xA4},{ 0x76}},   
{{0xA5},{ 0x87}},   
{{0xA6},{ 0xA2}},   
{{0xA7},{ 0xB8}},   
{{0xA8},{ 0xCA}},   
{{0xA9},{ 0xD8}},   
{{0xAA},{ 0xE3}},   
{{0xAB},{ 0xEB}},   
{{0xAC},{ 0xF0}},   
{{0xAD},{ 0xF8}},   
{{0xAE},{ 0xFD}},   
{{0xAF},{ 0xFF}},   


{{0x9F},{0x14}},        //case 4                                   
{{0xA0},{0x28}},                                           
{{0xA1},{0x44}},                                           
{{0xA2},{0x5d}},                                           
{{0xA3},{0x72}},                                           
{{0xA4},{0x86}},                                           
{{0xA5},{0x95}},                                           
{{0xA6},{0xb1}},                                           
{{0xA7},{0xc6}},                                           
{{0xA8},{0xd5}},                                           
{{0xA9},{0xe1}},                                           
{{0xAA},{0xea}},                                           
{{0xAB},{0xf1}},                                           
{{0xAC},{0xf5}},                                           
{{0xAD},{0xFb}},                                           
{{0xAE},{0xFe}},                                           
{{0xAF},{0xFF}},  


{{0x9F},{ 0x15}},   //case 5:	// largest gamma curve
{{0xA0},{ 0x2A}},   
{{0xA1},{ 0x4A}},   
{{0xA2},{ 0x67}},   
{{0xA3},{ 0x79}},   
{{0xA4},{ 0x8C}},   
{{0xA5},{ 0x9A}},   
{{0xA6},{ 0xB3}},   
{{0xA7},{ 0xC5}},   
{{0xA8},{ 0xD5}},   
{{0xA9},{ 0xDF}},   
{{0xAA},{ 0xE8}},   
{{0xAB},{ 0xEE}},   
{{0xAC},{ 0xF3}},   
{{0xAD},{ 0xFA}},   
{{0xAE},{ 0xFD}},   
{{0xAF},{ 0xFF}},   
  ****************************/     
	{{0xc0 },{0x00}},
	{{0xc1 },{0x10}},
	{{0xc2 },{0x1C}},
	{{0xc3 },{0x30}},
	{{0xc4 },{0x43}},
	{{0xc5 },{0x54}},
	{{0xc6 },{0x65}},
	{{0xc7 },{0x75}},
	{{0xc8 },{0x93}},
	{{0xc9 },{0xB0}},
	{{0xca },{0xCB}},
	{{0xcb },{0xE6}},
	{{0xcc },{0xFF}},
	{{0xf0 },{0x02}},
	{{0xf1 },{0x01}},
	{{0xf2 },{0x01}},
	{{0xf3 },{0x30}},
	{{0xf9 },{0x9f}},
	{{0xfa },{0x78}},

	//---------------------------------------------------------------
	{{0xfe },{0x01}},  //set page 1

	{{0x00 },{0xf5}},
	{{0x02 },{0x1a}},
	{{0x0a },{0xa0}},
	{{0x0b },{0x60}},
	{{0x0c },{0x08}},
	{{0x0e },{0x4c}},
	{{0x0f },{0x39}},
	{{0x11 },{0x3f}},
	{{0x12 },{0x72}},
	{{0x13 },{0x13}},
	{{0x14 },{0x42}},
	{{0x15 },{0x43}},
	{{0x16 },{0xc2}},
	{{0x17 },{0xa8}},
	{{0x18 },{0x18}},
	{{0x19 },{0x40}},
	{{0x1a },{0xd0}},
	{{0x1b },{0xf5}},
	{{0x70 },{0x40}},
	{{0x71 },{0x58}},
	{{0x72 },{0x30}},
	{{0x73 },{0x48}},
	{{0x74 },{0x20}},
	{{0x75 },{0x60}},
	{{0x77 },{0x20}},
	{{0x78 },{0x32}},
	{{0x30 },{0x03}},
	{{0x31 },{0x40}},
	{{0x32 },{0xe0}},
	{{0x33 },{0xe0}},
	{{0x34 },{0xe0}},
	{{0x35 },{0xb0}},
	{{0x36 },{0xc0}},
	{{0x37 },{0xc0}},
	{{0x38 },{0x04}},
	{{0x39 },{0x09}},
	{{0x3a },{0x12}},
	{{0x3b },{0x1C}},
	{{0x3c },{0x28}},
	{{0x3d },{0x31}},
	{{0x3e },{0x44}},
	{{0x3f },{0x57}},
	{{0x40 },{0x6C}},
	{{0x41 },{0x81}},
	{{0x42 },{0x94}},
	{{0x43 },{0xA7}},
	{{0x44 },{0xB8}},
	{{0x45 },{0xD6}},
	{{0x46 },{0xEE}},
	{{0x47 },{0x0d}},

	{{0xfe },{0x00}},  

	{{0xd2 },{0x90}}, // Open AEC at last.  


	/////////////////////////////////////////////////////////
	//-----------Update the registers 2010/07/07-------------//
	///////////////////////////////////////////////////////////

	{{0xfe },{0x00}},//set Page0

	{{0x10 },{0x26}},                               
	{{0x11 },{0x0d}},// fd,modified by mormo 2010/07/06                               
	{{0x1a },{0x2a}},// 1e,modified by mormo 2010/07/06                                  

	{{0x1c },{0x49}}, // c1,modified by mormo 2010/07/06                                 
	{{0x1d },{0x9a}}, // 08,modified by mormo 2010/07/06                                 
	{{0x1e },{0x61}}, // 60,modified by mormo 2010/07/06                                 

	{{0x3a },{0x20}},

	{{0x50 },{0x14}},// 10,modified by mormo 2010/07/06                               
	{{0x53 },{0x80}},                                
	{{0x56 },{0x80}},

	{{0x8b },{0x20}}, //LSC                                 
	{{0x8c },{0x20}},                                
	{{0x8d },{0x20}},                                
	{{0x8e },{0x14}},                                
	{{0x8f },{0x10}},                                
	{{0x90 },{0x14}},                                

	{{0x94 },{0x02}},                                
	{{0x95 },{0x07}},                                
	{{0x96 },{0xe0}},                                

	{{0xb1 },{0x40}}, // YCPT                                 
	{{0xb2 },{0x40}},                                
	{{0xb3 },{0x40}},
	{{0xb6 },{0xe0}},

	{{0xd0 },{0xcb}}, // AECT  c9,modifed by mormo 2010/07/06                                
	{{0xd3 },{0x48}}, // 80,modified by mormor 2010/07/06                           

	{{0xf2 },{0x02}},                                
	{{0xf7 },{0x12}},
	{{0xf8 },{0x0a}},



	{{0xfe },{0x01}},//set  Page1

	{{0x02 },{0x20}},
	{{0x04 },{0x10}},
	{{0x05 },{0x08}},
	{{0x06 },{0x20}},
	{{0x08 },{0x0a}},

	{{0x0e },{0x44}},                                
	{{0x0f },{0x32}},
	{{0x10 },{0x41}},                                
	{{0x11 },{0x37}},                                
	{{0x12 },{0x22}},                                
	{{0x13 },{0x19}},                                
	{{0x14 },{0x44}},                                
	{{0x15 },{0x44}},

	{{0x19 },{0x50}},                                
	{{0x1a },{0xd8}}, 

	{{0x32 },{0x10}}, 

	{{0x35 },{0x00}},                                
	{{0x36 },{0x80}},                                
	{{0x37 },{0x00}}, 
	//-----------Update the registers end---------//

	{{0xfe },{0x00}},// set back for page1
	/*Customer can adjust GAMMA, MIRROR & UPSIDEDOWN here!*/
	//GC0308_GAMMA_Select(3}}
};


/*
 * The white balance settings
 * Here only tune the R G B channel gain. 
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_auto_regs[] = {
	{{0x5a},{0x56}},//for AWB can adjust back, first write the RGB wb gain, and then enable AWB
	{{0x5b},{0x40}},
	{{0x5c},{0x4a}},
	{{0x22},{0x57}}
};

static struct regval_list sensor_wb_cloud_regs[] = {
	{{0x22},{0x55}},
	{{0x5a},{0x8c}},
	{{0x5b},{0x50}},
	{{0x5c},{0x40}}
};

static struct regval_list sensor_wb_daylight_regs[] = {
	//tai yang guang
	//{{0x22},{0x55}},
	{{0x22},{0x55}},
	{{0x5a},{0x74}},
	{{0x5b},{0x52}},
	{{0x5c},{0x40}}
};

static struct regval_list sensor_wb_incandescence_regs[] = {
	//bai re guang
	{{0x22},{0x55}},
	{{0x5a},{0x48}},
	{{0x5b},{0x40}},
	{{0x5c},{0x5c}}
};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	//ri guang deng
	{{0x22},{0x55}},
	{{0x5a},{0x40}},
	{{0x5b},{0x42}},
	{{0x5c},{0x50}}
};

static struct regval_list sensor_wb_tungsten_regs[] = {
	//wu si deng
	{{0x22},{0x55}},
	{{0x5a},{0x40}},
	{{0x5b},{0x54}},
	{{0x5c},{0x70}}
};

/*
 * The color effect settings
 */
static struct regval_list sensor_colorfx_none_regs[] = {
	{{0x23},{0x00}},
	{{0x2d},{0x0a}},
	{{0x20},{0xff}},
	{{0xd2},{0x90}},
	{{0x73},{0x00}},
	{{0x77},{0x54}},
	{{0xb3},{0x40}},
	{{0xb4},{0x80}},
	{{0xba},{0x00}},
	{{0xbb},{0x00}}
};

static struct regval_list sensor_colorfx_bw_regs[] = {
	{{0x23},{0x02}},
	{{0x2d},{0x0a}},
	{{0x20},{0xff}},
	{{0xd2},{0x90}},
	{{0x73},{0x00}},
	
	{{0xb3},{0x40}},
	{{0xb4},{0x80}},
	{{0xba},{0x00}},
	{{0xbb},{0x00}}
};

static struct regval_list sensor_colorfx_sepia_regs[] = {
	{{0x23},{0x02}},
	{{0x2d},{0x0a}},
	{{0x20},{0xff}},
	{{0xd2},{0x90}},
	{{0x73},{0x00}},
	
	{{0xb3},{0x40}},
	{{0xb4},{0x80}},
	{{0xba},{0xd0}},
	{{0xbb},{0x28}}
};

static struct regval_list sensor_colorfx_negative_regs[] = {
	{{0x23},{0x01}},
	{{0x2d},{0x0a}},
	{{0x20},{0xff}},
	{{0xd2},{0x90}},
	{{0x73},{0x00}},
	
	{{0xb3},{0x40}},
	{{0xb4},{0x80}},
	{{0xba},{0x00}},
	{{0xbb},{0x00}}
};

static struct regval_list sensor_colorfx_emboss_regs[] = {
	{{0x23},{0x02}},
	{{0x2d},{0x0a}},
	{{0x20},{0xbf}},
	{{0xd2},{0x10}},
	{{0x73},{0x01}},
	
	{{0x51},{0x40}},
	{{0x52},{0x40}},
	
	{{0xb3},{0x40}},
	{{0xb4},{0x80}},
	{{0xba},{0x00}},
	{{0xbb},{0x00}}
};

static struct regval_list sensor_colorfx_sketch_regs[] = {
	{{0x23},{0x02}},
	{{0x2d},{0x0a}},
	{{0x20},{0xbf}},
	{{0xd2},{0x10}},
	{{0x73},{0x01}},
	
	{{0x51},{0x40}},
	{{0x52},{0x40}},
	
	{{0xb3},{0x98}},
	{{0xb4},{0x06}},
	{{0xba},{0x00}},
	{{0xbb},{0x00}}
};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {
	{{0x23},{0x02}},
	{{0x2d},{0x0a}},
	{{0x20},{0xff}},
	{{0xd2},{0x90}},
	{{0x73},{0x00}},
	
	{{0xb3},{0x40}},
	{{0xb4},{0x80}},
	{{0xba},{0x50}},
	{{0xbb},{0xe0}}
};

static struct regval_list sensor_colorfx_grass_green_regs[] = {
	{{0x23},{0x02}},
	{{0x2d},{0x0a}},
	{{0x20},{0xff}},
	{{0xd2},{0x90}},
	{{0x73},{0x88}},
	
	{{0xb3},{0x40}},
	{{0xb4},{0x80}},
	{{0xba},{0xc0}},
	{{0xbb},{0xc0}}
};

static struct regval_list sensor_colorfx_skin_whiten_regs[] = {
	{{0x23},{0x02}},
	{{0x2d},{0x0a}},
	{{0x20},{0xbf}},
	{{0xd2},{0x10}},
	{{0x73},{0x01}},
	      
	{{0x51},{0x40}}, 
	{{0x52},{0x40}}, 
 
	{{0xb3},{0x60}},
	{{0xb4},{0x40}},
	{{0xba},{0x00}},
	{{0xbb},{0x00}}
};

static struct regval_list sensor_colorfx_vivid_regs[] = {
//NULL
};

/*
 * The brightness setttings
 */
static struct regval_list sensor_brightness_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_zero_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos4_regs[] = {
//NULL
};

/*
 * The contrast setttings
 */
static struct regval_list sensor_contrast_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_zero_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos4_regs[] = {
//NULL
};

/*
 * The saturation setttings
 */
static struct regval_list sensor_saturation_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_zero_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos4_regs[] = {
//NULL
};

/*
 * The exposure target setttings
 */
static struct regval_list sensor_ev_neg4_regs[] = {
	{{0xb5},{0xc0}},
	{{0xd3},{0x28}}
};

static struct regval_list sensor_ev_neg3_regs[] = {
	{{0xb5},{0xd0}},
	{{0xd3},{0x30}}
};

static struct regval_list sensor_ev_neg2_regs[] = {
	{{0xb5},{0xe0}},
	{{0xd3},{0x38}}
};

static struct regval_list sensor_ev_neg1_regs[] = {
	{{0xb5},{0xf0}},
	{{0xd3},{0x40}}
};

static struct regval_list sensor_ev_zero_regs[] = {
	{{0xb5},{0x00}},
	{{0xd3},{0x48}}
};

static struct regval_list sensor_ev_pos1_regs[] = {
	{{0xb5},{0x10}},
	{{0xd3},{0x50}}
};

static struct regval_list sensor_ev_pos2_regs[] = {
	{{0xb5},{0x20}},
	{{0xd3},{0x58}}
};

static struct regval_list sensor_ev_pos3_regs[] = {
	{{0xb5},{0x30}},
	{{0xd3},{0x60}}
};

static struct regval_list sensor_ev_pos4_regs[] = {
	{{0xb5},{0x40}},
	{{0xd3},{0x68}}
};


/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 * 
 */


static struct regval_list sensor_fmt_yuv422_yuyv[] = {
	{{0x24},{0xa2}}	//YCbYCr
};

static struct regval_list sensor_fmt_yuv422_yvyu[] = {
	{{0x24},{0xa3}}	//YCrYCb
};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {
	{{0x24},{0xa1}}	//CrYCbY
};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {
	{{0x24},{0xa0}}	//CbYCrY
};

static struct regval_list sensor_fmt_raw[] = {
	{{0x24},{0xb7}}//raw
};



/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[REG_STEP];
	struct i2c_msg msg;
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
		data[i] = reg[i];
	
	data[REG_ADDR_STEP] = 0xff;
	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		csi_dev_err("Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	
	msg.flags = I2C_M_RD;
	msg.len = REG_DATA_STEP;
	msg.buf = &data[REG_ADDR_STEP];
	
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		for(i = 0; i < REG_DATA_STEP; i++)
			value[i] = data[i+REG_ADDR_STEP];
		ret = 0;
	}
	else {
		csi_dev_err("Error %d on register read\n", ret);
	}
	return ret;
}


static int sensor_write(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[REG_STEP];
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
			data[i] = reg[i];
	for(i = REG_ADDR_STEP; i < REG_STEP; i++)
			data[i] = value[i-REG_ADDR_STEP];
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_STEP;
	msg.buf = data;

	
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}
	else if (ret < 0) {
		csi_dev_err("sensor_write error!\n");
	}
	return ret;
}


/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *vals , uint size)
{
	int i,ret;
	
	if (size == 0)
		return -EINVAL;
	
	for(i = 0; i < size ; i++)
	{
		if(vals->reg_num[0] == 0xff) {
			mdelay(vals->value[0]);
		}	else {
			ret = sensor_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
			{
				csi_dev_err("sensor_write_err!\n");
				return ret;
			}
		}
		udelay(500);
		vals++;
	}

	return 0;
}

/*
 * CSI GPIO control
 */
static void csi_gpio_write(struct v4l2_subdev *sd, user_gpio_set_t *gpio, int status)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
		
  if(gpio->port == 0xffff) {
    axp_gpio_set_io(gpio->port_num, 1);
    axp_gpio_set_value(gpio->port_num, status); 
  } else {
    gpio_write_one_pin_value(dev->csi_pin_hd,status,(char *)&gpio->gpio_name);
  }
}

static void csi_gpio_set_status(struct v4l2_subdev *sd, user_gpio_set_t *gpio, int status)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
		
  if(gpio->port == 0xffff) {
    axp_gpio_set_io(gpio->port_num, status);
  } else {
    gpio_set_one_pin_io_status(dev->csi_pin_hd,status,(char *)&gpio->gpio_name);
  }
}

/*
 * Stuff that knows about the sensor.
 */
 
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
  //make sure that no device can access i2c bus during sensor initial or power down
  //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
  i2c_lock_adapter(client->adapter);
  
  //insure that clk_disable() and clk_enable() are called in pair 
  //when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF
  switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			csi_dev_dbg("CSI_SUBDEV_STBY_ON\n");
			//reset off io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			//standby on io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(10);
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(10);
			//inactive mclk after stadby in
			clk_disable(dev->csi_module_clk);
			//reset on io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			break;
		case CSI_SUBDEV_STBY_OFF:
			csi_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
			//active mclk before stadby out
			clk_enable(dev->csi_module_clk);
			mdelay(10);
			//standby off io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			mdelay(10);
			//reset off io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(30);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		case CSI_SUBDEV_PWR_ON:
			csi_dev_dbg("CSI_SUBDEV_PWR_ON\n");
			//power on reset
			csi_gpio_set_status(sd,&dev->standby_io,1);//set the gpio to output
			csi_gpio_set_status(sd,&dev->reset_io,1);//set the gpio to output
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			//reset on io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			//active mclk before power on
			clk_enable(dev->csi_module_clk);
			mdelay(10);
			//power supply
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_ON);
			mdelay(10);
			if(dev->dvdd) {
				regulator_enable(dev->dvdd);
				mdelay(10);
			}
			if(dev->avdd) {
				regulator_enable(dev->avdd);
				mdelay(10);
			}
			if(dev->iovdd) {
				regulator_enable(dev->iovdd);
				mdelay(10);
			}
			//standby off io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			mdelay(10);
			//reset after power on
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(30);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		case CSI_SUBDEV_PWR_OFF:
			csi_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
			//standby and reset io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			//power supply off
			if(dev->iovdd) {
				regulator_disable(dev->iovdd);
				mdelay(10);
			}
			if(dev->avdd) {
				regulator_disable(dev->avdd);
				mdelay(10);
			}
			if(dev->dvdd) {
				regulator_disable(dev->dvdd);
				mdelay(10);	
			}
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_OFF);
			mdelay(10);
			//inactive mclk after power off
			clk_disable(dev->csi_module_clk);
			//set the io to hi-z
			csi_gpio_set_status(sd,&dev->reset_io,0);//set the gpio to input
			csi_gpio_set_status(sd,&dev->standby_io,0);//set the gpio to input
			break;
		default:
			return -EINVAL;
	}		

	//remember to unlock i2c adapter, so the device can access the i2c bus again
	i2c_unlock_adapter(client->adapter);	
	return 0;
}
 
static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);

	switch(val)
	{
		case CSI_SUBDEV_RST_OFF:
			csi_dev_dbg("CSI_SUBDEV_RST_OFF\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		case CSI_SUBDEV_RST_ON:
			csi_dev_dbg("CSI_SUBDEV_RST_ON\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			break;
		case CSI_SUBDEV_RST_PUL:
			csi_dev_dbg("CSI_SUBDEV_RST_PUL\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(30);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		default:
			return -EINVAL;
	}
		
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	int ret;
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00; //PAGE 0x00
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_detect!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x00;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}

	if(regs.value[0] != 0x9b)
		return -ENODEV;
	
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	csi_dev_dbg("sensor_init\n");
	
	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		csi_dev_err("chip found is not an target chip.\n");
		return ret;
	}
	
	sensor_write_array(sd, sensor_default_regs_24M , ARRAY_SIZE(sensor_default_regs_24M));
	return sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret=0;
	
	switch(cmd){
		case CSI_SUBDEV_CMD_GET_INFO: 
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;
			
			csi_dev_dbg("CSI_SUBDEV_CMD_GET_INFO\n");
			
			ccm_info->mclk 	=	info->ccm_info->mclk ;
			ccm_info->vref 	=	info->ccm_info->vref ;
			ccm_info->href 	=	info->ccm_info->href ;
			ccm_info->clock	=	info->ccm_info->clock;
			ccm_info->iocfg	=	info->ccm_info->iocfg;
			
			csi_dev_dbg("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			break;
		}
		case CSI_SUBDEV_CMD_SET_INFO:
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;
			
			csi_dev_dbg("CSI_SUBDEV_CMD_SET_INFO\n");
			
			info->ccm_info->mclk 	=	ccm_info->mclk 	;
			info->ccm_info->vref 	=	ccm_info->vref 	;
			info->ccm_info->href 	=	ccm_info->href 	;
			info->ccm_info->clock	=	ccm_info->clock	;
			info->ccm_info->iocfg	=	ccm_info->iocfg	;
			
			csi_dev_dbg("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			
			break;
		}
		default:
			return -EINVAL;
	}		
		return ret;
}


/*
 * Store information about the video data format. 
 */
static struct sensor_format_struct {
	__u8 *desc;
	//__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;//linux-3.0
	struct regval_list *regs;
	int	regs_size;
	int bpp;   /* Bytes per pixel */
} sensor_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yvyu,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yvyu),
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_vyuy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_vyuy),
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,//linux-3.0
		.regs 		= sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp		= 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

static struct regval_list sensor_vga_regs[] = {
{{0xfe},{0x01}},
{{0x54},{0x11}},
{{0x55},{0x03}},
{{0x56},{0x00}},
{{0x57},{0x00}},
{{0x58},{0x00}},
{{0x59},{0x00}},
{{0xfe},{0x00}},
{{0x46},{0x00}},
{{0xfe},{0x00}}
};
static struct regval_list sensor_qvga_regs[] = {
{{0xfe},{0x01}},
{{0x54},{0x22}},  //1/2 subsample 
{{0x55},{0x03}}, 
{{0x56},{0x00}}, 
{{0x57},{0x00}}, 
{{0x58},{0x00}}, 
{{0x59},{0x00}},
{{0xfe},{0x00}},
{{0x46},{0x80}},//enable crop window mode
{{0x47},{0x00}},  
{{0x48},{0x00}},  
{{0x49},{0x00}}, 
{{0x4a},{0xf0}},//240
{{0x4b},{0x01}},  
{{0x4c},{0x40}}, //320
{{0xfe},{0x00}}
};
static struct regval_list sensor_qcif_regs[] = {
{{0xfe},{0x01}},
{{0x54},{0x33}},  //1/3 subsample 
{{0x55},{0x03}}, 
{{0x56},{0x00}}, 
{{0x57},{0x00}}, 
{{0x58},{0x00}}, 
{{0x59},{0x00}},               
{{0xfe},{0x00}},
{{0x46},{0x80}},//enable crop window mode
{{0x47},{0x00}},  
{{0x48},{0x00}},  
{{0x49},{0x00}}, 
{{0x4a},{0x90}},//144
{{0x4b},{0x00}},  
{{0x4c},{0xb0}}, //176
{{0xfe},{0x00}},
};
/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size {
	int	width;
	int	height;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	int regs_size;
	int (*set_size) (struct v4l2_subdev *sd);
/* h/vref stuff */
} sensor_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.regs 		= sensor_vga_regs,
		.regs_size	= ARRAY_SIZE(sensor_vga_regs),
		.set_size	= NULL,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.regs 		= sensor_qvga_regs,
		.regs_size	= ARRAY_SIZE(sensor_qvga_regs),
		.set_size		= NULL,
	},
	/*QCIF*/
	/* QVGA */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.regs 		= sensor_qcif_regs,
		.regs_size	= ARRAY_SIZE(sensor_qcif_regs),
		.set_size		= NULL,
	}
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

 
 
 
static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)//linux-3.0
{
//	struct sensor_format_struct *ofmt;

	if (index >= N_FMTS)//linux-3.0
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;//linux-3.0

//	ofmt = sensor_formats + fmt->index;
//	fmt->flags = 0;
//	strcpy(fmt->description, ofmt->desc);
//	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
		//struct v4l2_format *fmt,
		struct v4l2_mbus_framefmt *fmt,//linux-3.0
		struct sensor_format_struct **ret_fmt,
		struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
//	struct v4l2_pix_format *pix = &fmt->fmt.pix;//linux-3.0
	csi_dev_dbg("sensor_try_fmt_internal\n");
	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)//linux-3.0
			break;
	
	if (index >= N_FMTS) {
		/* default to first format */
		index = 0;
		fmt->code = sensor_formats[0].mbus_code;//linux-3.0
	}
	
	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
		
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0
	
	
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)//linux-3.0
			break;
	
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;//linux-3.0
	fmt->height = wsize->height;//linux-3.0
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;//linux-3.0
	//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0
	
	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);
	
	csi_dev_dbg("sensor_s_fmt\n");
	
	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;
	
		
	sensor_write_array(sd, sensor_fmt->regs , sensor_fmt->regs_size);
	
	ret = 0;
	if (wsize->regs)
	{
		ret = sensor_write_array(sd, wsize->regs , wsize->regs_size);
		if (ret < 0)
			return ret;
	}
	
	if (wsize->set_size)
	{
		ret = wsize->set_size(sd);
		if (ret < 0)
			return ret;
	}
	
	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	
	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	//struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = SENSOR_FRAME_RATE;
	
	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
//	struct v4l2_captureparm *cp = &parms->parm.capture;
	//struct v4l2_fract *tpf = &cp->timeperframe;
	//struct sensor_info *info = to_state(sd);
	//int div;

//	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
//		return -EINVAL;
//	if (cp->extendedmode != 0)
//		return -EINVAL;

//	if (tpf->numerator == 0 || tpf->denominator == 0)
//		div = 1;  /* Reset to full rate */
//	else
//		div = (tpf->numerator*SENSOR_FRAME_RATE)/tpf->denominator;
//		
//	if (div == 0)
//		div = 1;
//	else if (div > CLK_SCALE)
//		div = CLK_SCALE;
//	info->clkrc = (info->clkrc & 0x80) | div;
//	tpf->numerator = 1;
//	tpf->denominator = sensor_FRAME_RATE/div;
//sensor_write(sd, REG_CLKRC, info->clkrc);
	return 0;
}


/* 
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */
static int sensor_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	/* see sensor_s_parm and sensor_g_parm for the meaning of value */
	
	switch (qc->id) {
//	case V4L2_CID_BRIGHTNESS:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_CONTRAST:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_SATURATION:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_HUE:
//		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
//	case V4L2_CID_GAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
//	case V4L2_CID_AUTOGAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 0);
	case V4L2_CID_EXPOSURE_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_DO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_COLORFX:
		return v4l2_ctrl_query_fill(qc, 0, 9, 1, 0);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);	
	}
	return -EINVAL;
}

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00;		//page 0
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_g_hflip!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x14;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}
	
	regs.value[0] &= (1<<0);
	regs.value[0] = regs.value[0]>>0;		//0x14 bit0 is mirror
		
	*value = regs.value[0];

	info->hflip = *value;
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00;		//page 0
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x14;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}
	
	switch (value) {
		case 0:
		  regs.value[0] &= 0xfe;
			break;
		case 1:
			regs.value[0] |= 0x01;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	
	mdelay(100);
	
	info->hflip = value;
	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00;		//page 0
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_g_vflip!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x14;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_vflip!\n");
		return ret;
	}
	
	regs.value[0] &= (1<<1);
	regs.value[0] = regs.value[0]>>1;		//0x14 bit1 is upsidedown
		
	*value = regs.value[0];

	info->vflip = *value;
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00;		//page 0
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x14;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_vflip!\n");
		return ret;
	}
	
	switch (value) {
		case 0:
		  regs.value[0] &= 0xfd;
			break;
		case 1:
			regs.value[0] |= 0x02;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}
	
	mdelay(100);
	
	info->vflip = value;
	return 0;
}

static int sensor_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_autogain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xd2;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_autoexp!\n");
		return ret;
	}

	regs.value[0] &= 0x80;
	if (regs.value[0] == 0x80) {
		*value = V4L2_EXPOSURE_AUTO;
	}
	else
	{
		*value = V4L2_EXPOSURE_MANUAL;
	}
	
	info->autoexp = *value;
	return 0;
}

static int sensor_s_autoexp(struct v4l2_subdev *sd,
		enum v4l2_exposure_auto_type value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xd2;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_autoexp!\n");
		return ret;
	}

	switch (value) {
		case V4L2_EXPOSURE_AUTO:
		  regs.value[0] |= 0x80;
			break;
		case V4L2_EXPOSURE_MANUAL:
			regs.value[0] &= 0x7f;
			break;
		case V4L2_EXPOSURE_SHUTTER_PRIORITY:
			return -EINVAL;    
		case V4L2_EXPOSURE_APERTURE_PRIORITY:
			return -EINVAL;
		default:
			return -EINVAL;
	}
		
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autoexp!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->autoexp = value;
	return 0;
}

static int sensor_g_autowb(struct v4l2_subdev *sd, int *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0x22;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_autowb!\n");
		return ret;
	}

	regs.value[0] &= (1<<1);
	regs.value[0] = regs.value[0]>>1;		//0x22 bit1 is awb enable
		
	*value = regs.value[0];
	info->autowb = *value;
	
	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_autowb!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x22;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_autowb!\n");
		return ret;
	}

	switch(value) {
	case 0:
		regs.value[0] &= 0xfd;
		break;
	case 1:
		regs.value[0] |= 0x02;
		break;
	default:
		break;
	}	
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autowb!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->autowb = value;
	return 0;
}

static int sensor_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_hue(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}
/* *********************************************end of ******************************************** */

static int sensor_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->brightness;
	return 0;
}

static int sensor_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_brightness_neg4_regs, ARRAY_SIZE(sensor_brightness_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_brightness_neg3_regs, ARRAY_SIZE(sensor_brightness_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_brightness_neg2_regs, ARRAY_SIZE(sensor_brightness_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_brightness_neg1_regs, ARRAY_SIZE(sensor_brightness_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_brightness_zero_regs, ARRAY_SIZE(sensor_brightness_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_brightness_pos1_regs, ARRAY_SIZE(sensor_brightness_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_brightness_pos2_regs, ARRAY_SIZE(sensor_brightness_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_brightness_pos3_regs, ARRAY_SIZE(sensor_brightness_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_brightness_pos4_regs, ARRAY_SIZE(sensor_brightness_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_brightness!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->brightness = value;
	return 0;
}

static int sensor_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->contrast;
	return 0;
}

static int sensor_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_contrast_neg4_regs, ARRAY_SIZE(sensor_contrast_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_contrast_neg3_regs, ARRAY_SIZE(sensor_contrast_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_contrast_neg2_regs, ARRAY_SIZE(sensor_contrast_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_contrast_neg1_regs, ARRAY_SIZE(sensor_contrast_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_contrast_zero_regs, ARRAY_SIZE(sensor_contrast_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_contrast_pos1_regs, ARRAY_SIZE(sensor_contrast_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_contrast_pos2_regs, ARRAY_SIZE(sensor_contrast_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_contrast_pos3_regs, ARRAY_SIZE(sensor_contrast_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_contrast_pos4_regs, ARRAY_SIZE(sensor_contrast_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_contrast!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->contrast = value;
	return 0;
}

static int sensor_g_saturation(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->saturation;
	return 0;
}

static int sensor_s_saturation(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_saturation_neg4_regs, ARRAY_SIZE(sensor_saturation_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_saturation_neg3_regs, ARRAY_SIZE(sensor_saturation_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_saturation_neg2_regs, ARRAY_SIZE(sensor_saturation_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_saturation_neg1_regs, ARRAY_SIZE(sensor_saturation_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_saturation_zero_regs, ARRAY_SIZE(sensor_saturation_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_saturation_pos1_regs, ARRAY_SIZE(sensor_saturation_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_saturation_pos2_regs, ARRAY_SIZE(sensor_saturation_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_saturation_pos3_regs, ARRAY_SIZE(sensor_saturation_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_saturation_pos4_regs, ARRAY_SIZE(sensor_saturation_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_saturation!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->saturation = value;
	return 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_ev_neg4_regs, ARRAY_SIZE(sensor_ev_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_ev_neg3_regs, ARRAY_SIZE(sensor_ev_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_ev_neg2_regs, ARRAY_SIZE(sensor_ev_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_ev_neg1_regs, ARRAY_SIZE(sensor_ev_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_ev_zero_regs, ARRAY_SIZE(sensor_ev_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_ev_pos1_regs, ARRAY_SIZE(sensor_ev_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_ev_pos2_regs, ARRAY_SIZE(sensor_ev_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_ev_pos3_regs, ARRAY_SIZE(sensor_ev_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_ev_pos4_regs, ARRAY_SIZE(sensor_ev_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_exp!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->exp = value;
	return 0;
}

static int sensor_g_wb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_whiteblance *wb_type = (enum v4l2_whiteblance*)value;
	
	*wb_type = info->wb;
	
	return 0;
}

static int sensor_s_wb(struct v4l2_subdev *sd,
		enum v4l2_whiteblance value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	
	//sensor_write_array(sd, sensor_wb_test, ARRAY_SIZE(sensor_wb_test));
	if (value == V4L2_WB_AUTO) {
//		ret = sensor_s_autowb(sd, 1);
//		return ret;
			ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
			info->autowb = 1;
	} 
	else {
//		ret = sensor_s_autowb(sd, 0);
//		if(ret < 0) {
//			csi_dev_err("sensor_s_autowb error, return %x!\n",ret);
//			return ret;
//		}
		switch (value) {
			case V4L2_WB_CLOUD:
			  ret = sensor_write_array(sd, sensor_wb_cloud_regs, ARRAY_SIZE(sensor_wb_cloud_regs));
				break;
			case V4L2_WB_DAYLIGHT:
				ret = sensor_write_array(sd, sensor_wb_daylight_regs, ARRAY_SIZE(sensor_wb_daylight_regs));
				break;
			case V4L2_WB_INCANDESCENCE:
				ret = sensor_write_array(sd, sensor_wb_incandescence_regs, ARRAY_SIZE(sensor_wb_incandescence_regs));
				break;    
			case V4L2_WB_FLUORESCENT:
				ret = sensor_write_array(sd, sensor_wb_fluorescent_regs, ARRAY_SIZE(sensor_wb_fluorescent_regs));
				break;
			case V4L2_WB_TUNGSTEN:  
				ret = sensor_write_array(sd, sensor_wb_tungsten_regs, ARRAY_SIZE(sensor_wb_tungsten_regs));
				break;
			default:
				return -EINVAL;
		} 
		info->autowb = 0;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_s_wb error, return %x!\n",ret);
		return ret;
	}
	
	//mdelay(300);
	//sensor_write_array(sd, sensor_wb_test1, ARRAY_SIZE(sensor_wb_test1));
	info->wb = value;
	return 0;
}

static int sensor_g_colorfx(struct v4l2_subdev *sd,
		__s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_colorfx *clrfx_type = (enum v4l2_colorfx*)value;
	
	*clrfx_type = info->clrfx;
	return 0;
}

static int sensor_s_colorfx(struct v4l2_subdev *sd,
		enum v4l2_colorfx value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
	case V4L2_COLORFX_NONE:
	  ret = sensor_write_array(sd, sensor_colorfx_none_regs, ARRAY_SIZE(sensor_colorfx_none_regs));
		break;
	case V4L2_COLORFX_BW:
		ret = sensor_write_array(sd, sensor_colorfx_bw_regs, ARRAY_SIZE(sensor_colorfx_bw_regs));
		break;  
	case V4L2_COLORFX_SEPIA:
		ret = sensor_write_array(sd, sensor_colorfx_sepia_regs, ARRAY_SIZE(sensor_colorfx_sepia_regs));
		break;   
	case V4L2_COLORFX_NEGATIVE:
		ret = sensor_write_array(sd, sensor_colorfx_negative_regs, ARRAY_SIZE(sensor_colorfx_negative_regs));
		break;
	case V4L2_COLORFX_EMBOSS:   
		ret = sensor_write_array(sd, sensor_colorfx_emboss_regs, ARRAY_SIZE(sensor_colorfx_emboss_regs));
		break;
	case V4L2_COLORFX_SKETCH:     
		ret = sensor_write_array(sd, sensor_colorfx_sketch_regs, ARRAY_SIZE(sensor_colorfx_sketch_regs));
		break;
	case V4L2_COLORFX_SKY_BLUE:
		ret = sensor_write_array(sd, sensor_colorfx_sky_blue_regs, ARRAY_SIZE(sensor_colorfx_sky_blue_regs));
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		ret = sensor_write_array(sd, sensor_colorfx_grass_green_regs, ARRAY_SIZE(sensor_colorfx_grass_green_regs));
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		ret = sensor_write_array(sd, sensor_colorfx_skin_whiten_regs, ARRAY_SIZE(sensor_colorfx_skin_whiten_regs));
		break;
	case V4L2_COLORFX_VIVID:
		ret = sensor_write_array(sd, sensor_colorfx_vivid_regs, ARRAY_SIZE(sensor_colorfx_vivid_regs));
		break;
	default:
		return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_s_colorfx error, return %x!\n",ret);
		return ret;
	}
	
	mdelay(10);
	
	info->clrfx = value;
	return 0;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_flash_mode *flash_mode = (enum v4l2_flash_mode*)value;
	
	*flash_mode = info->flash_mode;
	return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
    enum v4l2_flash_mode value)
{
	struct sensor_info *info = to_state(sd);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	int flash_on,flash_off;
	
	flash_on = (dev->flash_pol!=0)?1:0;
	flash_off = (flash_on==1)?0:1;
	
	switch (value) {
	case V4L2_FLASH_MODE_OFF:
		csi_gpio_write(sd,&dev->flash_io,flash_off);
		break;
	case V4L2_FLASH_MODE_AUTO:
		return -EINVAL;
		break;  
	case V4L2_FLASH_MODE_ON:
		csi_gpio_write(sd,&dev->flash_io,flash_on);
		break;   
	case V4L2_FLASH_MODE_TORCH:
		return -EINVAL;
		break;
	case V4L2_FLASH_MODE_RED_EYE:   
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	
	info->flash_mode = value;
	return 0;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_g_saturation(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return sensor_g_hue(sd, &ctrl->value);	
	case V4L2_CID_VFLIP:
		return sensor_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_g_hflip(sd, &ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_g_autogain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_g_autoexp(sd, &ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_g_wb(sd, &ctrl->value);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_g_autowb(sd, &ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_g_colorfx(sd,	&ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
		return sensor_g_flash_mode(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_s_saturation(sd, ctrl->value);
	case V4L2_CID_HUE:
		return sensor_s_hue(sd, ctrl->value);		
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_s_autogain(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_s_autoexp(sd,
				(enum v4l2_exposure_auto_type) ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		//printk("V4L2_CID_DO_WHITE_BALANCE\n");
		//mutex_lock(&videodev_lock_test);		
		///videodev_lock_num=3;			
		//mutex_unlock(&videodev_lock_test);
		
		 ret=sensor_s_wb(sd,(enum v4l2_whiteblance) ctrl->value);	
		
		 return ret;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_s_autowb(sd, ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_s_colorfx(sd,
				(enum v4l2_colorfx) ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return sensor_s_flash_mode(sd,
	      (enum v4l2_flash_mode) ctrl->value);
	}
	return -EINVAL;
}

static int sensor_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt = sensor_enum_fmt,//linux-3.0
	.try_mbus_fmt = sensor_try_fmt,//linux-3.0
	.s_mbus_fmt = sensor_s_fmt,//linux-3.0
	.s_parm = sensor_s_parm,//linux-3.0
	.g_parm = sensor_g_parm,//linux-3.0
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
//	int ret;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

	info->fmt = &sensor_formats[0];
	info->ccm_info = &ccm_info_con;
	
	info->brightness = 0;
	info->contrast = 0;
	info->saturation = 0;
	info->hue = 0;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 1;
	info->exp = 0;
	info->autoexp = 0;
	info->autowb = 1;
	info->wb = 0;
	info->clrfx = 0;
	
//	info->clkrc = 1;	/* 30fps */

	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "gc0308", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

//linux-3.0
static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
	  .name = "gc0308",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
  i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);