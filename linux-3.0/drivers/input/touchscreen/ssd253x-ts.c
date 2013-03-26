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
#include "ssd253x-ts.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

#define CONFIG_TOUCHSCREEN_SSL_DEBUG	
#undef  CONFIG_TOUCHSCREEN_SSL_DEBUG
extern int input_ctp_module_flg;
#define DEVICE_ID_REG                    2
#define VERSION_ID_REG                 3
#define AUTO_INIT_RST_REG          68
#define EVENT_STATUS                   121
#define EVENT_MSK_REG                 122
#define IRQ_MSK_REG                     123
#define FINGER01_REG                    124
#define EVENT_STACK                   	 128
#define EVENT_FIFO_SCLR               135
#define TIMESTAMP_REG                 136
#define SELFCAP_STATUS_REG         185		
#define CTP_IRQ_NO				(gpio_int_info[0].port_num)
#define CTP_IRQ_MODE			(NEGATIVE_EDGE)
#if 1 
#define WPRINTK(format, args...)	do {		\
		printk(KERN_INFO "<WISKY-DEBUG> " format , ## args);	\
		} while (0)
#else
#define WPRINTK(format, args...)
#endif


#define CTP_NAME			SSD253X_I2C_NAME
#define	TOUCH_INT_NO	SW_INT_IRQNO_PIO    //GPIO :set the interrupt 
#define SSD253X_I2C_NAME	"ssd253x-ts"
/*
struct ChipSetting {
	char No;
	char Reg;
	char Data1;
	char Data2;
};*/
static void* __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static int gpio_wakeup_hdle = 0;
static int gpio_tp_choose_hdle=0;
static int screen_max_x;
static int screen_max_y;
//static int revert_x_flag = 0;
//static int revert_y_flag = 0;
//static int exchange_x_y_flag = 0;
static __u32 twi_addr = 0;
static __u32 twi_id = 0;
static unsigned int FW_LEN = 0;
static  struct ChipSetting *fw_config_p =NULL;

union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};


static user_gpio_set_t  gpio_int_info[1];
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};
void deviceReset(struct i2c_client *client);
void deviceResume(struct i2c_client *client);
void deviceSuspend(struct i2c_client *client);
void SSD253xdeviceInit(struct i2c_client *client); 

static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int ssd253x_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_early_suspend(struct early_suspend *h);
static void ssd253x_ts_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer);
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id);
static struct workqueue_struct *ssd253x_wq;

struct ssl_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct hrtimer timer;
	struct work_struct  ssl_work;
#ifdef	CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif 

	int irq;
	int use_irq;
	int FingerNo;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];

	int Resolution;
	int EventStatus;
	int FingerDetect;

	int sFingerX[FINGERNO];
	int sFingerY[FINGERNO];
	int pFingerX[FINGERNO];
	int pFingerY[FINGERNO];
};

int ssd253x_record,ssd253x_current,ssd253x_timer_flag; //add by hjc

int ReadRegister(struct i2c_client *client,uint8_t reg,int ByteNo)
{
	unsigned char buf[4];
	struct i2c_msg msg[2];
	int ret;

	memset(buf, 0xFF, sizeof(buf));
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ByteNo;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		ReadRegister: i2c_transfer Error !\n");
	else		printk("		ReadRegister: i2c_transfer OK !\n");
	#endif

	if(ByteNo==1) return (int)((unsigned int)buf[0]<<0);
	if(ByteNo==2) return (int)((unsigned int)buf[1]<<0)|((unsigned int)buf[0]<<8);
	if(ByteNo==3) return (int)((unsigned int)buf[2]<<0)|((unsigned int)buf[1]<<8)|((unsigned int)buf[0]<<16);
	if(ByteNo==4) return (int)((unsigned int)buf[3]<<0)|((unsigned int)buf[2]<<8)|((unsigned int)buf[1]<<16)|(buf[0]<<24);
	return 0;
}

void WriteRegister(struct i2c_client *client,uint8_t Reg,unsigned char Data1,unsigned char Data2,int ByteNo)
{	
	struct i2c_msg msg;
	unsigned char buf[4];
	int ret;

	buf[0]=Reg;
	buf[1]=Data1;
	buf[2]=Data2;
	buf[3]=0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ByteNo+1;
	msg.buf = (char *)buf;
	ret = i2c_transfer(client->adapter, &msg, 1);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		WriteRegister: i2c_master_send Error !\n");
	else		printk("		WriteRegister: i2c_master_send OK !\n");
	#endif
}
//Function as i2c_master_send, and return 1 if operation is successful. 
static int ssd253x_write_bytes(struct i2c_client *client, uint8_t *data, uint16_t len)
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

/**
 * ctp_free_platform_resource - corresponding with ctp_init_platform_resource
 *
 */
static void ctp_free_platform_resource(void)
{
	printk("=======%s=========.\n", __func__);
	if(gpio_addr){
		iounmap(gpio_addr);
	}
	
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	
	if(gpio_wakeup_hdle){
		gpio_release(gpio_wakeup_hdle, 2);
	}
	
	return;
}
void SSD253xdeviceInit(struct i2c_client *client)
{	
	
	int i;
	int ret =0;

	if (fw_config_p == NULL){
		printk("fw_config_p == NULL\n");
		return ;
	}
	for(i=0;i<FW_LEN;i++)
	{

		 WriteRegister(client,fw_config_p[i].Reg,fw_config_p[i].Data1,\
		fw_config_p[i].Data2,fw_config_p[i].No);
		if (ret < 0){
			printk("SSD253xdeviceInit ret < 0\n");
			return ;
		}
			
	}
	
	//mdelay(100);
}

void deviceReset(struct i2c_client *client)
{	
	int i;
	for(i=0;i<sizeof(Reset)/sizeof(Reset[0]);i++)
	{
		WriteRegister(	client,Reset[i].Reg,
				Reset[i].Data1,Reset[i].Data2,
				Reset[i].No);
	}
	mdelay(150);
	
}

void deviceResume(struct i2c_client *client)    
{	
	#if 1
	//RST pin pull down
	printk("ssd253x  resume\n");
	gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup");
	//gpio_direction_output(SHUTDOWN_PORT, 0);
	mdelay(5);
	gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup");
	//gpio_direction_output(SHUTDOWN_PORT, 1);
	mdelay(2);

	deviceReset(client);
	SSD253xdeviceInit(client);
	
	
	#else
	int i;
	//int timeout=10;
	//int status;
	for(i=0;i<sizeof(Resume)/sizeof(Resume[0]);i++)
	{
		WriteRegister(	client,Resume[i].Reg,
				Resume[i].Data1,Resume[i].Data2,
				Resume[i].No);
		mdelay(150);
	}
	/*
	do {
		status=ReadRegister(client,0x26,1);
		printk("		deviceResume: status : %d !\n",status);
		if(status==Resume[2].Data1) break;
		mdelay(1);
	}while(timeout--); // Check the status
	*/
	#endif
	
}

void deviceSuspend(struct i2c_client *client)
{	
	#if 1
	printk("ssd253x suspend\n");
	//RST pin pull down
	gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup");
	//gpio_direction_output(SHUTDOWN_PORT, 0);
	mdelay(5);
	gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup");
	//gpio_direction_output(SHUTDOWN_PORT, 1);
	mdelay(2);
	
	#else
	int i;
	//int timeout=10;
	//int status;
	
	/*
	WriteRegister(	client,Suspend[0].Reg,
			Suspend[0].Data1,Suspend[0].Data2,
			Suspend[0].No);
	do {
		status=ReadRegister(client,0x26,1);
		if(status==Suspend[0].Data1) break;
		mdelay(1);				
	}while(timeout--);
	*/
	
	for(i=0;i<sizeof(Suspend)/sizeof(Suspend[0]);i++)
	{
		WriteRegister(	client,Suspend[i].Reg,
				Suspend[i].Data1,Suspend[i].Data2,
				Suspend[i].No);
		mdelay(200);
	}
	#endif
}

#define Mode RunningAverageMode
#define Dist RunningAverageDist
void RunningAverage(unsigned short *xpos,unsigned short *ypos,int No,struct ssl_ts_priv *ssl_priv)
{	
	int FilterMode[4][2]={{0,8},{5,3},{6,2},{7,1}};
	int dx,dy;
	int X,Y;

	X=*xpos;
	Y=*ypos;
	if((ssl_priv->pFingerX[No]!=0x0FFF)&&(X!=0x0FFF))
	{
		dx=abs(ssl_priv->pFingerX[No]-X);
		dy=abs(ssl_priv->pFingerY[No]-Y);
		if(dx+dy<Dist*64)
		{
			ssl_priv->pFingerX[No]=(FilterMode[Mode][0]*ssl_priv->pFingerX[No]+FilterMode[Mode][1]*X)/8;
			ssl_priv->pFingerY[No]=(FilterMode[Mode][0]*ssl_priv->pFingerY[No]+FilterMode[Mode][1]*Y)/8;
		}
		else
		{
			ssl_priv->pFingerX[No]=X;
			ssl_priv->pFingerY[No]=Y;
		}
	}
	else
	{
		ssl_priv->pFingerX[No]=X;
		ssl_priv->pFingerY[No]=Y;
	}
	*xpos=ssl_priv->pFingerX[No];
	*ypos=ssl_priv->pFingerY[No];
}

void FingerCheckSwap(int *FingerX,int *FingerY,int *FingerP,int FingerNo,int *sFingerX,int *sFingerY)
{
  	int i,j;
  	int index1,index2;
  	int Vx,Vy;
  	int Ux,Uy;
  	int R1x,R1y;
  	int R2x,R2y;
	for(i=0;i<FingerNo;i++)
  	{
 		index1=i;
	    	if( FingerX[index1]!=0xFFF)
		if(sFingerX[index1]!=0xFFF) 
		{
			for(j=i+1;j<FingerNo+3;j++)
			{
				index2=j%FingerNo;
	    			if( FingerX[index2]!=0xFFF)
				if(sFingerX[index2]!=0xFFF) 
		    		{
					Ux=sFingerX[index1]-sFingerX[index2];
					Uy=sFingerY[index1]-sFingerY[index2];      
					Vx= FingerX[index1]- FingerX[index2];
					Vy= FingerY[index1]- FingerY[index2];					

					R1x=Ux-Vx;
					R1y=Uy-Vy;
					R2x=Ux+Vx;
					R2y=Uy+Vy;
							
					R1x=R1x*R1x;
					R1y=R1y*R1y; 
					R2x=R2x*R2x;
					R2y=R2y*R2y;

					if(R1x+R1y>R2x+R2y)
				    	{
				    		Ux=FingerX[index1];
						Uy=FingerY[index1];
						Vx=FingerP[index1];
							          
						FingerX[index1]=FingerX[index2];
						FingerY[index1]=FingerY[index2];
						FingerP[index1]=FingerP[index2];
							
						FingerX[index2]=Ux;
						FingerY[index2]=Uy;
						FingerP[index2]=Vx;
					}
					break;
			    	}
			}
		}
  	}        
  	for(i=0;i<FingerNo;i++)
  	{
    		sFingerX[i]=FingerX[i];
    		sFingerY[i]=FingerY[i];
  	}
}


#ifdef SSD253x_CUT_EDGE
static int ssd253x_ts_cut_edge(unsigned short pos,unsigned short x_y)
{
	u8 cut_value = 18; //cut_value < 32
	if(pos == 0xfff)
	{
		return pos;
	}
	if(x_y) //xpos 64-->96  //MAX=896
	{
		pos = pos + cut_value;
		pos = SSDS53X_SCREEN_MAX_X * pos / (896+cut_value*2-5);
		return pos;
	}
	else    //ypos  //MAX=576
	{
			pos = pos + cut_value;
			
		pos = SSDS53X_SCREEN_MAX_Y* pos / (576+cut_value*2-5);
		return pos;		
	}
	
	
}
#endif

static void ssd253x_ts_work(struct work_struct *work)
{
	int i;
	unsigned short xpos=0, ypos=0;
	int FingerInfo;
	int EventStatus;
	int EventStatus_Info;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	//int FingerP[FINGERNO];
	int clrFlag=0;
	int timer_status;
	#ifdef SSD253x_TOUCH_KEY
	u8 btn_status;
	static bool key[4] = { 0, 0, 0, 0 }; 
	#endif
	

	struct ssl_ts_priv *ssl_priv = container_of(work,struct ssl_ts_priv,ssl_work);
	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_work!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif

	if(!ssd253x_timer_flag)
	{
		timer_status = ReadRegister(ssl_priv->client,TIMESTAMP_REG,2);
		if(!ssd253x_record)                                      
		{
				ssd253x_record = timer_status/1000;   			
		}
		
		ssd253x_current = timer_status/1000;               
		
		if((ssd253x_current - ssd253x_record) > 10)
		{
		WriteRegister(ssl_priv->client,AUTO_INIT_RST_REG,0x00,0x00,1);
		ssd253x_record = 0;
		ssd253x_timer_flag = 1;
		}
	}

	#ifdef SSD253x_TOUCH_KEY
		KeyInfo = ReadRegister(ssl_priv->client,0xB9,1);
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		if(KeyInfo<0)	printk("		ssd253x_ts_work: i2c_transfer Error !\n");
		#endif
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("ssd253x_ts_work read 0xB9,KeyInfo is %x\n",KeyInfo);
		#endif
		if(KeyInfo & 0x0f){
			switch(KeyInfo & 0x0f){
			case 1:
			key[0] = 1;
			input_event(ssl_priv->input,EV_KEY, key_code[0], 1);
			break;
			case 2:
			key[1] = 1;
			input_event(ssl_priv->input,EV_KEY, key_code[1], 1);
			break;
			case 4:
			key[2] = 1;
			input_event(ssl_priv->input,EV_KEY, key_code[2], 1);
			break;
			case 8:
			key[3] = 1;
			input_event(ssl_priv->input,EV_KEY, key_code[3], 1);
			break;
			default:
			break;
			}
			hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
		goto work_touch;
		}
		for(i = 0; i < 4; i++)
		{
			if(key[i])
			{
				key[i] = 0;
				input_event(ssl_priv->input, EV_KEY, key_code[i], 0);
			}
		}
	work_touch:
	#endif

	EventStatus = ReadRegister(ssl_priv->client,EVENT_STATUS,2);
	EventStatus_Info=EventStatus;
	if((EventStatus_Info>>3)&0x1)
	{
		WriteRegister(ssl_priv->client,0xA2,0x00,0x00,1);	
	}
	EventStatus=EventStatus>>4;
	ssl_priv->FingerDetect=0;
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		if((EventStatus>>i)&0x1)
		{
			FingerInfo=ReadRegister(ssl_priv->client,FINGER01_REG+i,4);
			ypos = ((FingerInfo>>4)&0xF00)|((FingerInfo>>24)&0xFF);
			xpos = ((FingerInfo>>0)&0xF00)|((FingerInfo>>16)&0xFF);
			//width= ((FingerInfo>>4)&0x00F);	
			//printk("xpos:%d ypos=%d\n",xpos,ypos);
			if(xpos!=0xFFF)
			{
				ssl_priv->FingerDetect++;

				#ifdef SSD253x_CUT_EDGE
				xpos = ssd253x_ts_cut_edge(xpos, 1);
				ypos = ssd253x_ts_cut_edge(ypos, 0);
				//printk("after cut_edge xpos:%d ypos=%d\n",xpos,ypos);
				#endif
			}
			else 
			{
				// This part is to avoid asyn problem when the finger leaves
				//printk("		ssd253x_ts_work: Correct %x\n",EventStatus);
				EventStatus=EventStatus&~(1<<i);
				clrFlag=1;
			}
		}
		else
		{
			xpos=ypos=0xFFF;
			//width=0;
			clrFlag=1;
		}
		FingerX[i]=xpos;
		FingerY[i]=ypos;
		//FingerP[i]=width;
	}
	if(ssl_priv->use_irq==1) ;//enable_irq(ssl_priv->irq);
	if(ssl_priv->use_irq==2)
	{
		if(ssl_priv->FingerDetect==0) ;//enable_irq(ssl_priv->irq);
		else hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	}
	if(clrFlag) WriteRegister(ssl_priv->client,EVENT_FIFO_SCLR,0x01,0x00,1);

	//if(ssl_priv->input->id.product==0x2533)
	//if(ssl_priv->input->id.version==0x0101) 
		//FingerCheckSwap(FingerX,FingerY,FingerP,ssl_priv->FingerNo,ssl_priv->sFingerX,ssl_priv->sFingerY);
		//printk("after FingerCheckSwap xpos:%d ypos=%d\n",FingerX[0],FingerY[0]);
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		xpos=FingerX[i];
		ypos=FingerY[i];
		//width=FingerP[i];
		
		//if(ssl_priv->input->id.product==0x2533)
		//{
			//if(ssl_priv->input->id.version==0x0101) 
				//RunningAverage(&xpos,&ypos,i,ssl_priv);
				//printk("after RunningAverage xpos:%d ypos=%d\n",xpos,ypos);
			//if(ssl_priv->input->id.version==0x0102) RunningAverage(&xpos,&ypos,i,ssl_priv);
		//}

		if(xpos!=0xFFF)
		{
			input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);  
			input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, xpos);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ypos);
			input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, 200);//width);
			input_mt_sync(ssl_priv->input);

			//#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			#if 0
			if(i==0) printk("		ssd253x_ts_work: X = 0x%d , Y = 0x%d, W = 0x%d\n",xpos,ypos,width);
			#endif
		}
		else if(ssl_priv->FingerX[i]!=0xFFF)
		{
			input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
			input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(ssl_priv->input);
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			if(i==0) printk("	release	ssd253x_ts_work: X = 0x%x , Y = 0x%x, W = 0x%x\n",xpos,ypos,width);
			#endif
		}
		ssl_priv->FingerX[i]=FingerX[i];
		ssl_priv->FingerY[i]=FingerY[i];
		//ssl_priv->FingerP[i]=width;
	}		
	ssl_priv->EventStatus=EventStatus;	
	input_sync(ssl_priv->input);
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
		WPRINTK("==CTP_IRQ_NO=\n");              
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
	WPRINTK("%s: config gpio to int mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		WPRINTK("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	WPRINTK("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	WPRINTK(" INTERRUPT CONFIG\n");
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
//Test i2c to check device. Before it SHUTDOWN port Must be low state 30ms or more.
static bool ssd253x_i2c_test(struct i2c_client * client)
{
	int ret, retry;
	uint8_t test_data[1] = { 0 };	//only write a data address.
	
	for(retry=0; retry < 5; retry++)
	{
		ret =ssd253x_write_bytes(client, test_data, 1);	//Test i2c.
		if (ret == 1)
			break;
		msleep(5);
	}
	
	return ret == 1 ? true : false;
}
static int ssd253x_ts_probe(struct i2c_client *client,const struct i2c_device_id *idp)
{
	struct ssl_ts_priv *ssl_priv;
	struct input_dev *ssl_input;
	int error;
	int i;
  int err,ret;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_probe!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: need I2C_FUNC_I2C\n");
		#endif
		goto err_check_functionality_failed;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: i2c Check OK!\n");
		printk("		ssd253x_ts_probe: i2c_client name : %s\n",client->name);
		#endif
	}

	ssl_priv = kzalloc(sizeof(*ssl_priv), GFP_KERNEL);
	if (!ssl_priv)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: kzalloc Error!\n");
		#endif
		error=-ENODEV;
		goto	err_alloc_data_failed;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: kzalloc OK!\n");
		#endif
	}
#define TEST_I2C_TRANSFER
#ifdef TEST_I2C_TRANSFER
	//TODO: used to set speed of i2c transfer. Should be change as your paltform.
	ret = ssd253x_i2c_test(client);
	if(!ret)
	{
		pr_info("Warnning: I2C connection might be something wrong!\n");
		goto err_i2c_failed;
	}
	pr_info("===== ssd253x i2c test ok=======\n");
#endif
	
	
	err = ctp_set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);
	
	if(0 != err){
		printk("%s:ctp_ops.set_irq_mode err. \n", __func__);
		goto exit_set_irq_mode;
	}
	
	
	
	dev_set_drvdata(&client->dev, ssl_priv);
	
	ssl_input = input_allocate_device();
	if (!ssl_input)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_allocate_device Error\n");
		#endif
		error=-ENODEV;
		goto	err_input_dev_alloc_failed;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_allocate_device OK\n");
		#endif
	}

	ssl_input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN) ;
	ssl_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) | BIT_MASK(BTN_2);
	ssl_input->name = client->name;
	ssl_input->id.bustype = BUS_I2C;
	ssl_input->id.vendor  = 0x2878; // Modify for Vendor ID
	ssl_input->dev.parent = &client->dev;
	

	input_set_drvdata(ssl_input, ssl_priv);
	ssl_priv->client = client;
	ssl_priv->input = ssl_input;
	ssl_priv->use_irq = 2;
	ssl_priv->irq = TOUCH_INT_NO;
	ssl_priv->FingerNo=FINGERNO;
	ssl_priv->Resolution=64;

	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		//ssl_priv->FingerP[i]=0;
		// For Finger Check Swap
		ssl_priv->sFingerX[i]=0xFFF;
		ssl_priv->sFingerY[i]=0xFFF;

		// For Adaptive Running Average
		ssl_priv->pFingerX[i]=0xFFF;
		ssl_priv->pFingerY[i]=0xFFF;
	}

	deviceReset(client);
	printk("SSL Touchscreen I2C Address: 0x%02X\n",client->addr);
	ssl_input->id.product = ReadRegister(client, DEVICE_ID_REG,2);
	ssl_input->id.version = ReadRegister(client,VERSION_ID_REG,2);
	ssl_input->id.product = ReadRegister(client, DEVICE_ID_REG,2);
	ssl_input->id.version = ReadRegister(client,VERSION_ID_REG,2);
	printk("SSL Touchscreen Device ID  : 0x%04X\n",ssl_input->id.product);
	printk("SSL Touchscreen Version ID : 0x%04X\n",ssl_input->id.version);

	SSD253xdeviceInit(client);
	WriteRegister(client,EVENT_FIFO_SCLR,0x01,0x00,1); // clear Event FiFo
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("		ssd253X_ts_probe: %04XdeviceInit OK!\n",ssl_input->id.product);
	#endif

	if(ssl_priv->input->id.product==0x2531)		
		ssl_priv->Resolution=32;
	else if(ssl_priv->input->id.product==0x2533)	
		ssl_priv->Resolution=64;
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: ssl_input->id.product Error\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}

	input_set_abs_params(ssl_input, ABS_MT_TRACKING_ID, 0,16, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_WIDTH_MAJOR, 0, 8, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_X,  0,SSDS53X_SCREEN_MAX_X + 1, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_Y,  0,SSDS53X_SCREEN_MAX_Y + 1, 0, 0);

	#ifdef SSD253x_TOUCH_KEY
	set_bit(KEY_MENU, ssl_input->keybit);
	set_bit(KEY_HOME, ssl_input->keybit);
	set_bit(KEY_BACK, ssl_input->keybit);
	set_bit(KEY_SEARCH, ssl_input->keybit);
	#endif

	
	ssd253x_wq = create_singlethread_workqueue("ssd253x_wq");
	if (!ssd253x_wq)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue Error!\n");
		#endif
		return -ENOMEM;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue OK!\n");
		#endif
	}	
	INIT_WORK(&ssl_priv->ssl_work, ssd253x_ts_work);
	error = input_register_device(ssl_input);
	if(error)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_register_device input Error!\n");
		#endif
		error=-ENODEV;
		goto	err_input_register_device_failed;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_register_device input OK!\n");
		#endif
	}
   input_ctp_module_flg=1;
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2))
	{
		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_FALLING | IRQF_SHARED, client->name,ssl_priv);
		if(error)
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd253x_ts_probe: request_irq Error!\n");
			#endif
			error=-ENODEV;
			goto exit_irq_request_failed;
		}
		else
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd253x_ts_probe: request_irq OK!\n");
			#endif
		}	
		
		//enable_irq(ssl_priv->irq);
	}

	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2))
	{
		hrtimer_init(&ssl_priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ssl_priv->timer.function = ssd253x_ts_timer;
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: timer_init OK!\n");
		#endif
		//hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef	CONFIG_HAS_EARLYSUSPEND
	ssl_priv->early_suspend.suspend = ssd253x_ts_early_suspend;
	ssl_priv->early_suspend.resume  = ssd253x_ts_late_resume;
	ssl_priv->early_suspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB +1;
	register_early_suspend(&ssl_priv->early_suspend);
#endif 
	return 0;

exit_irq_request_failed:	
enable_irq(SW_INT_IRQNO_PIO);	
input_unregister_device(ssl_input);
err_input_register_device_failed:	
err1:
		input_free_device(ssl_input);
err_input_dev_alloc_failed:	
	dev_set_drvdata(&client->dev, NULL);
exit_set_irq_mode:
err_i2c_failed:
	kfree(ssl_priv);
err_alloc_data_failed:
err_check_functionality_failed:
ctp_free_platform_resource();
	return error;
}


static int ssd253x_ts_resume(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	int reg_val;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_resume!                |\n");
	printk("+-----------------------------------------+\n");
	#endif
	deviceResume(client);
	ssd253x_timer_flag = 0;
	if(ssl_priv->use_irq){
	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET); 
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);
	
			
	}		//enable_irq(ssl_priv->irq);
	else 
		hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	int reg_val;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_suspend!               |\n");
	printk("+-----------------------------------------+\n");
	#endif
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	deviceSuspend(client);		
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) 
		{
		reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET); 
		reg_val &= ~(1 << (gpio_int_info[0].port_num));
		writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);	
		}
		//disable_irq(ssl_priv->irq);	// free_irq()按平台实际情况使用或disable
	return 0;
}

#ifdef	CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_late_resume(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_late_resume!           |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_resume(ssl_priv->client);
}
static void ssd253x_ts_early_suspend(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_early_suspend!         |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_suspend(ssl_priv->client, PMSG_SUSPEND);
}
#endif

static int ssd253x_ts_remove(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_remove !               |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) 
		//disable_irq(ssl_priv->irq);// free_irq()按平台实际情况使用或disable
		free_irq(SW_INT_IRQNO_PIO, ssl_priv);
	
	#ifdef CONFIG_HAS_EARLYSUSPEND	
		unregister_early_suspend(&ssl_priv->early_suspend);
	#endif
		
	input_unregister_device(ssl_priv->input);
	input_free_device(ssl_priv->input);
	kfree(ssl_priv);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id)
{
	int reg_val;
	struct ssl_ts_priv *ssl_priv = dev_id;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_isr!                   |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	
	//disable_irq_nosync(ssl_priv->irq);             //
	//printk("tp irq\n");
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
     
	if(reg_val&(1<<(CTP_IRQ_NO)))
	{	
		//printk("==CTP_IRQ_NO=\n");
		writel(reg_val&(1<<(CTP_IRQ_NO)),gpio_addr + PIO_INT_STAT_OFFSET);
		queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	}
	else
	{
	    printk("Other Interrupt\n");
	    return IRQ_NONE;
	}
	return IRQ_HANDLED;	
	
}

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer)
{
	struct ssl_ts_priv *ssl_priv = container_of(timer, struct ssl_ts_priv, timer);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_timer!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	//printk("|	ssd253x_ts_timer!                 |\n");
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	if(ssl_priv->use_irq==0) hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static const struct i2c_device_id ssd253x_ts_id[] = {
	{ SSD253X_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);

static struct i2c_driver ssd253x_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = SSD253X_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ssd253x_ts_probe,
	.remove = ssd253x_ts_remove,
#ifndef	CONFIG_HAS_EARLYSUSPEND
	.suspend = ssd253x_ts_suspend,
	.resume = ssd253x_ts_resume,
#endif
	.id_table = ssd253x_ts_id,
	.address_list	= u_i2c_addr.normal_i2c,
};

int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		WPRINTK("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}



/**
 * ctp_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
 
 #if 0
 static int  tp_init(void)
{
    
    //TP_CTRL0: 0x0027003f
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
    //TP_CTRL1: 0x00000070 -> 0x00000030
    writel(TP_DATA_XY_CHANGE|STYLUS_UP_DEBOUNCE|STYLUS_UP_DEBOUCE_EN|TP_DUAL_EN|TP_MODE_EN,TP_BASSADDRESS + TP_CTRL1);
    
    return (0);
}
#endif
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

 int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	//int ctp_used = -1;
	char name[I2C_NAME_SIZE];
	char tp_name[30];	
  char mach_name[12];
  
  //unsigned int tp_adc_val ,time_out,try_cnt,tp_adc_cnt = 0;	
	//unsigned int  reg_val;
  //volatile unsigned int key_val;
  //unsigned int reg_fifoc;
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;

	memset(mach_name,0,12);
	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("product", "machine", (int *)(mach_name), &type, sizeof(mach_name)/sizeof(int))){
		pr_err("%s: line: %d script_parser_fetch err.,%s\n", __func__, __LINE__,mach_name);
			
	}
	WPRINTK("machine:%s\n",mach_name);
		
	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(name), &type, sizeof(name)/sizeof(int))){
		pr_err("%s line:%d %s: script_parser_fetch err. \n","ctp_name",__LINE__, __func__);
		printk("ctp_name");
		goto script_parser_fetch_err;
	}
	
	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_tp_name", (int *)(tp_name), &type, sizeof(tp_name)/sizeof(int))){
		pr_err("%s line:%d %s: script_parser_fetch err. \n","ctp_name",__LINE__, __func__);
		printk("ctp_tp_name");
		goto script_parser_fetch_err;
	}
 
	if(strcmp(CTP_NAME, name)){
		pr_err("%s: name %s does not match CTP_NAME. \n", __func__, name);
		pr_err(CTP_NAME);
		//ret = 1;
		return ret;
	}
	
	
	#if 0
	tp_adc_val = 0;
	time_out = 0;
	try_cnt = 0;
	while(1)
	{
		//tp_init();
		//writel(LRADC_ADC1_DATA_EN,KEY_BASSADDRESS);	
		//writel(FIRST_CONCERT_DLY|LEVELB_VOL|KEY_MODE_SELECT|LRADC_HOLD_EN|ADC_CHAN_SELECT|LRADC_SAMPLE_62HZ|LRADC_EN,KEY_BASSADDRESS + LRADC_CTRL);
		do{
		mdelay(1);
		reg_val  = readl(TP_BASSADDRESS + TP_INT_FIFOS);
		time_out++;
		printk("time_out:%d reg_val:0x%x\n",time_out,reg_val);
		if (time_out == 200)
			break;
		}while(!(reg_val&FIFO_DATA_PENDING));
		
		key_val = readl(TP_BASSADDRESS + TP_DATA);
		
		reg_fifoc = readl(TP_BASSADDRESS+TP_INT_FIFOC);
    reg_fifoc |= TP_FIFO_FLUSH;
    writel(reg_fifoc, TP_BASSADDRESS+TP_INT_FIFOC); 
		
		{
			tp_adc_val+=key_val;
			tp_adc_cnt++;
			printk("tp_val:0x%x,cnt:%d\n",tp_adc_val,tp_adc_cnt);
			if (tp_adc_cnt == 3)
				break;
		}
		try_cnt++;
		time_out = 0;
		if (try_cnt == 5)
			break;
	}
	printk("read tp adc val:%d tp_adc_cnt:%d\n",tp_adc_val,tp_adc_cnt);
	if (tp_adc_cnt != 0)
		tp_adc_val = tp_adc_val /tp_adc_cnt;
	printk("read tp adc val:0x%x\n",tp_adc_val);
	
	for(tp_adc_cnt=0;tp_adc_cnt<5;tp_adc_cnt++)
	{
	tp_adc_val = readl(TP_BASSADDRESS + TP_DATA);
	printk("read tp adc val:0x%x\n",tp_adc_val);
	}
	#endif
	if(!strcmp("M876-A13", mach_name)){
				fw_config_p = CONFIG_M876A13_SS253X_QIUTIAN;
				FW_LEN = sizeof(CONFIG_M876A13_SS253X_QIUTIAN)/sizeof(CONFIG_M876A13_SS253X_QIUTIAN[0]);
				WPRINTK("CONFIG_M876AC_SS253X_QIUTIAN\n");
	}	
	else if(!strcmp("M828-A13", mach_name)){
#if 0				
				gpio_tp_choose_hdle = gpio_request_ex("ctp_para", "ctp_io_port");
				if(!gpio_tp_choose_hdle) {
					pr_warning("touch panel gpio_tp_choose_hdle request gpio fail!\n");
				}
				gpio_set_one_pin_io_status(gpio_tp_choose_hdle, 1, "ctp_io_port");
				//for(try_cnt=0;try_cnt<5;try_cnt++)
				gpio_write_one_pin_value(gpio_tp_choose_hdle, 1, "ctp_io_port");
				
				gpio_set_one_pin_io_status(gpio_tp_choose_hdle, 0, "ctp_io_port");
				
				if(gpio_read_one_pin_value(gpio_tp_choose_hdle, "ctp_io_port"))
				{
				printk("tp is pingbo\n");	
				fw_config_p = CONFIG_M828AC_SS253X_PINGBO;
				FW_LEN = sizeof(CONFIG_M828AC_SS253X_PINGBO)/sizeof(CONFIG_M828AC_SS253X_PINGBO[0]);
				WPRINTK("CONFIG_M828AC_SS253X_PINGBO\n");		
				}
				else
				{
				printk("tp is qiutian\n");
				fw_config_p = CONFIG_M828AC_SS253X_QIUTIAN;
				FW_LEN = sizeof(CONFIG_M828AC_SS253X_QIUTIAN)/sizeof(CONFIG_M828AC_SS253X_QIUTIAN[0]);
				WPRINTK("CONFIG_M828AC_SS253X_QIUTIAN\n");		
				
				}
#else
				fw_config_p = CONFIG_M828AC_SS253X_PINGBO;
				FW_LEN = sizeof(CONFIG_M828AC_SS253X_PINGBO)/sizeof(CONFIG_M828AC_SS253X_PINGBO[0]);
				printk("CONFIG_M828AC_SS253X_PINGBO\n");
#endif
	}	
	else if(!strcmp("BK7032", mach_name)){
				fw_config_p = CONFIG_BK7032_SS253X_ZHONGCHU;
				FW_LEN = sizeof(CONFIG_BK7032_SS253X_ZHONGCHU)/sizeof(CONFIG_BK7032_SS253X_ZHONGCHU[0]);
				WPRINTK("CONFIG_M876AC_SS253X_QIUTIAN\n");
	
	}
	else if(!strcmp("U7-A13", mach_name)){
				fw_config_p = CONFIG_M876A13_SS253X_QIUTIAN;
				FW_LEN = sizeof(CONFIG_M876A13_SS253X_QIUTIAN)/sizeof(CONFIG_M876A13_SS253X_QIUTIAN[0]);
				WPRINTK("CONFIG_M876AC_SS253X_QIUTIAN\n");
	}	
	else{
			WPRINTK("mach_name(%s)  mach failed! try to mach ctp_tp_name \n",mach_name);
			
			if(!strcmp("m828ac_ss253x_pingbo", tp_name)){
			fw_config_p = CONFIG_M828AC_SS253X_PINGBO;
			FW_LEN = sizeof(CONFIG_M828AC_SS253X_PINGBO)/sizeof(CONFIG_M828AC_SS253X_PINGBO[0]);
			WPRINTK("driver(%s) mach  tp_name success(m828ac_ss253x_pingbo)\n",name);
			}
			else if(!strcmp("m876a13_ss253x_qiutian",tp_name)){
			fw_config_p = CONFIG_M876A13_SS253X_QIUTIAN;
			FW_LEN = sizeof(CONFIG_M876A13_SS253X_QIUTIAN)/sizeof(CONFIG_M876A13_SS253X_QIUTIAN[0]);
			WPRINTK("driver(%s) mach  ctp_tp_name success(m876a13_ss253x_qiutian)\n",name);
			}
			else if(!strcmp("bk7032a13_ss253x_zhongchu",tp_name)){
			fw_config_p = CONFIG_BK7032_SS253X_ZHONGCHU;
			FW_LEN = sizeof(CONFIG_BK7032_SS253X_ZHONGCHU)/sizeof(CONFIG_BK7032_SS253X_ZHONGCHU[0]);
			WPRINTK("driver(%s) mach  tp_name success(bk7032a13_ss253x_zhongchu)\n",name);		
			}
			else {	
			WPRINTK("ctp_tp_name mach also failed!\n");
			//fw_config_p = NULL;
			//FW_LEN = 0;
			fw_config_p = CONFIG_M876A13_SS253X_QIUTIAN;
			FW_LEN = sizeof(CONFIG_M876A13_SS253X_QIUTIAN)/sizeof(CONFIG_M876A13_SS253X_QIUTIAN[0]);
			WPRINTK("driver(%s) mach  ctp_tp_name success(m876a13_ss253x_qiutian)\n",name);
			}
	}
	
	//end-wisky-lxh@20110111	

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%d %s: script_parser_fetch err. \n",__LINE__, name);
		goto script_parser_fetch_err;
	}
	//big-endian or small-endian?
	//printk("%s: before: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	WPRINTK("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	//printk("%s: after: ctp_twi_addr is 0x%x, u32_dirty_addr_buf: 0x%hx. u32_dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u32_dirty_addr_buf[0],u32_dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%d %s: script_parser_fetch err. \n",__LINE__, name);
		goto script_parser_fetch_err;
	}
	WPRINTK("%s: ctp_twi_id is %d. \n", __func__, twi_id);
	
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)){
		pr_err("%d %s: script_parser_fetch err. \n", __LINE__,__func__);
		goto script_parser_fetch_err;
	}
	WPRINTK("%s: screen_max_x = %d. \n", __func__, screen_max_x);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)){
		pr_err("%d %s: script_parser_fetch err. \n", __LINE__,__func__);
		goto script_parser_fetch_err;
	}
	WPRINTK("%s: screen_max_y = %d. \n", __func__, screen_max_y);

	return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
}
//static char banner[] __initdata = KERN_INFO "SSL Touchscreen driver, (c) 2011 Solomon Systech Ltd.\n";
static int __init ssd253x_ts_init(void)
{
	int ret = -1;
	

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG	
	printk("+-----------------------------------------+\n");
	printk("|	SSL_ts_init!                      |\n");
	printk("+-----------------------------------------+\n");
	#endif
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
	
	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);
	
	pr_info("ssd253x_ts_init\n");
	gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup");
	mdelay(10);
	gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup");	
  mdelay(10);	
	gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup");	
  mdelay(10);	
	ssd253x_ts_driver.detect = ctp_detect;

	ret = i2c_add_driver(&ssd253x_ts_driver);

		
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret) printk("ssd253x_ts_init: i2c_add_driver Error! \n");
	else    printk("ssd253x_ts_init: i2c_add_driver OK! \n");
	#endif


	return ret; 	
	


}

static void __exit ssd253x_ts_exit(void)
{
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_exit!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif
	i2c_del_driver(&ssd253x_ts_driver);
	
	flush_workqueue(ssd253x_wq);
	if (ssd253x_wq) destroy_workqueue(ssd253x_wq);
		
	ctp_free_platform_resource();
}

module_init(ssd253x_ts_init);
module_exit(ssd253x_ts_exit);

MODULE_AUTHOR("Solomon Systech Ltd - Design Technology, Icarus Choi");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ssd253x Touchscreen Driver 1.3");
