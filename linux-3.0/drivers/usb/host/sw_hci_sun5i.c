/*
*************************************************************************************
*                         			      Linux
*					           USB Host Controller Driver
*
*				        (c) Copyright 2006-2012, SoftWinners Co,Ld.
*							       All Rights Reserved
*
* File Name 	: sw_hci_sun5i.c
*
* Author 		: javen
*
* Description 	: Include file for AW1623 HCI Host Controller Driver
*
* Notes         :
*
* History 		:
*      <author>    		<time>       	<version >    		<desc>
*    yangnaitian      2011-5-24            1.0          create this file
*    javen            2011-7-18            1.1          �����ʱ�ӿ��غ͹��翪��
*
*************************************************************************************
*/


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/dma-mapping.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/irqs.h>

#include  <mach/clock.h>
#include <mach/sys_config.h>

#include "../../power/axp_power/axp-gpio.h"
#include <linux/regulator/consumer.h>

#include  "sw_hci_sun5i.h"


#define  USB_CONTROLLER_NUM         2
static char* usbc_name[USB_CONTROLLER_NUM] 			= {"usbc0", "usbc1"};
static char* usbc_ahb_ehci_name[USB_CONTROLLER_NUM]  = {"", "ahb_ehci0"};
static char* usbc_ahb_ohci_name[USB_CONTROLLER_NUM]  = {"", "ahb_ohci0"};
static char* usbc_phy_gate_name[USB_CONTROLLER_NUM] 	= {"usb_phy", "usb_phy"};
static char* ohci_phy_gate_name[USB_CONTROLLER_NUM]  = {"", "usb_ohci0"};
static char* usbc_phy_reset_name[USB_CONTROLLER_NUM] = {"usb_phy0", "usb_phy1"};

static u32 usbc_base[USB_CONTROLLER_NUM] 			= {SW_VA_USB0_IO_BASE, SW_VA_USB1_IO_BASE};
static u32 ehci_irq_no[USB_CONTROLLER_NUM] 			= {0, SW_INT_SRC_EHCI0};
static u32 ohci_irq_no[USB_CONTROLLER_NUM] 			= {0, SW_INT_SRC_OHCI0};

static int usb1_set_vbus_cnt = 0;
static int usb2_set_vbus_cnt = 0;
static u32 usb_enable_passly_cnt = 0;
static u32 usb_enable_configure_cnt = 0;

#define  USB_EXTERN_PIN_LDO_MASK     200

/*
*******************************************************************************
*                     get_usb_cfg
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static s32 get_usb_cfg(struct sw_hci_hcd *sw_hci)
{
	__s32 ret = 0;

	/* usbc enable */
	ret = script_parser_fetch(usbc_name[sw_hci->usbc_no], "usb_used", (int *)&sw_hci->used, 64);
	if(ret != 0){
		DMSG_PANIC("ERR: get usbc2 enable failed\n");
		//return -1;
	}

	/* request gpio */
	ret = script_parser_fetch(usbc_name[sw_hci->usbc_no], "usb_drv_vbus_gpio", (int *)&sw_hci->drv_vbus_gpio_set, 64);
	if(ret != 0){
		DMSG_PANIC("ERR: get usb_drv_vbus_gpio usbc%d(%s) id failed\n", sw_hci->usbc_no, usbc_name[sw_hci->usbc_no]);
		//return -1;
	}

	/* request gpio */
	ret = script_parser_fetch(usbc_name[sw_hci->usbc_no], "usb_drv_vbus_1_gpio", (int *)&sw_hci->drv_vbus_1_gpio_set, 64);
	if(ret != 0){
		DMSG_PANIC("ERR: get usb_drv_vbus_1_gpio usbc%d(%s) id failed\n", sw_hci->usbc_no, usbc_name[sw_hci->usbc_no]);
		//return -1;
	}

	/* usbc controller type */
	ret = script_parser_fetch(usbc_name[sw_hci->usbc_no], "usb_controller_type", (int *)&sw_hci->usb_controller_type, 64);
	if(ret != 0){
		DMSG_PANIC("ERR: get usbc controller type %d(%s) id failed\n", sw_hci->usbc_no, usbc_name[sw_hci->usbc_no]);
	}

	if(sw_hci->drv_vbus_gpio_set.port){
		sw_hci->drv_vbus_gpio_valid = 1;
	}else{
		DMSG_PANIC("ERR: %s(drv vbus) is invalid\n", sw_hci->hci_name);
		sw_hci->drv_vbus_gpio_valid = 0;
	}

	if(sw_hci->drv_vbus_1_gpio_set.port){
		sw_hci->drv_vbus_1_gpio_valid = 1;
	}else{
		DMSG_PANIC("ERR: %s(drv vbus_1) is invalid\n", sw_hci->hci_name);
		sw_hci->drv_vbus_1_gpio_valid = 0;
	}

	/* host_init_state */
	ret = script_parser_fetch(usbc_name[sw_hci->usbc_no], "usb_host_init_state", (int *)&(sw_hci->host_init_state), 64);
	if(ret != 0){
		DMSG_PANIC("ERR: script_parser_fetch host_init_state failed\n");
		sw_hci->host_init_state = 0;
		//return -1;
	}

	return 0;
}

/*
*******************************************************************************
*                     USBC_Phy_GetCsr
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static __u32 USBC_Phy_GetCsr(__u32 usbc_no)
{
	__u32 val = 0x0;

	switch(usbc_no){
		case 0:
			val = SW_VA_USB0_IO_BASE + 0x404;
		break;

		case 1:
			val = SW_VA_USB0_IO_BASE + 0x404;
		break;

		case 2:
			val = SW_VA_USB0_IO_BASE + 0x404;
		break;

		default:
		break;
	}

	return val;
}

/*
*******************************************************************************
*                     USBC_Phy_TpRead
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
#if 0
static __u32 USBC_Phy_TpRead(__u32 usbc_no, __u32 addr, __u32 len)
{
	__u32 temp = 0, ret = 0;
	__u32 i=0;
	__u32 j=0;

	for(j = len; j > 0; j--)
	{
		/* set  the bit address to be read */
		temp = USBC_Readl(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0xff << 8);
		temp |= ((addr + j -1) << 8);
		USBC_Writel(temp, USBC_Phy_GetCsr(usbc_no));

		for(i = 0; i < 0x4; i++);

		temp = USBC_Readl(USBC_Phy_GetCsr(usbc_no));
		ret <<= 1;
		ret |= ((temp >> (16 + usbc_no)) & 0x1);
	}

	return ret;
}
#endif

/*
*******************************************************************************
*                     USBC_Phy_TpWrite
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static __u32 USBC_Phy_TpWrite(__u32 usbc_no, __u32 addr, __u32 data, __u32 len)
{
	__u32 temp = 0, dtmp = 0;
//	__u32 i=0;
	__u32 j=0;

	dtmp = data;
	for(j = 0; j < len; j++)
	{
		/* set  the bit address to be write */
		temp = USBC_Readl(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0xff << 8);
		temp |= ((addr + j) << 8);
		USBC_Writel(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0x1 << 7);
		temp |= (dtmp & 0x1) << 7;
		temp &= ~(0x1 << (usbc_no << 1));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp |= (0x1 << (usbc_no << 1));
		USBC_Writeb( temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0x1 << (usbc_no <<1 ));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));
		dtmp >>= 1;
	}

	return data;
}

/*
*******************************************************************************
*                     USBC_Phy_Read
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
#if 0
static __u32 USBC_Phy_Read(__u32 usbc_no, __u32 addr, __u32 len)
{
	return USBC_Phy_TpRead(usbc_no, addr, len);
}
#endif

/*
*******************************************************************************
*                     USBC_Phy_Write
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static __u32 USBC_Phy_Write(__u32 usbc_no, __u32 addr, __u32 data, __u32 len)
{
	return USBC_Phy_TpWrite(usbc_no, addr, data, len);
}

/*
*******************************************************************************
*                     UsbPhyInit
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void UsbPhyInit(__u32 usbc_no)
{
//	DMSG_INFO("csr1: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Readl(USBC_Phy_GetCsr(usbc_no)));

    /* ����45ŷ�迹 */
	if(usbc_no == 0){
	    USBC_Phy_Write(usbc_no, 0x0c, 0x01, 1);
	}

//	DMSG_INFO("csr2-0: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x0c, 1));

    /* ���� USB0 PHY �ķ��Ⱥ����� */
	USBC_Phy_Write(usbc_no, 0x20, 0x14, 5);

//	DMSG_INFO("csr2-1: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x20, 5));

    /* ���� disconnect ��ֵ */
	USBC_Phy_Write(usbc_no, 0x2a, 2, 2);

//	DMSG_INFO("csr2: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x2a, 2));
//	DMSG_INFO("csr3: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Readl(USBC_Phy_GetCsr(usbc_no)));

	return;
}
#if 1
/*
*******************************************************************************
*                     clock_init
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static s32 clock_init(struct sw_hci_hcd *sw_hci, u32 ohci)
{
    if(ohci){  /* ohci */
    	sw_hci->sie_clk = clk_get(NULL, usbc_ahb_ohci_name[sw_hci->usbc_no]);
    	if (IS_ERR(sw_hci->sie_clk)){
    		DMSG_PANIC("ERR: get ohci%d abh clk failed.\n", (sw_hci->usbc_no - 1));
    		goto failed;
    	}

    	sw_hci->ohci_gate = clk_get(NULL, ohci_phy_gate_name[sw_hci->usbc_no]);
    	if (IS_ERR(sw_hci->ohci_gate)){
    		DMSG_PANIC("ERR: get ohci%d gate clk failed.\n", (sw_hci->usbc_no - 1));
    		goto failed;
    	}
	}else{  /* ehci */
    	sw_hci->sie_clk = clk_get(NULL, usbc_ahb_ehci_name[sw_hci->usbc_no]);
    	if (IS_ERR(sw_hci->sie_clk)){
    		DMSG_PANIC("ERR: get ehci%d abh clk failed.\n", (sw_hci->usbc_no - 1));
    		goto failed;
    	}
	}

	sw_hci->phy_gate = clk_get(NULL, usbc_phy_gate_name[sw_hci->usbc_no]);
	if (IS_ERR(sw_hci->phy_gate)){
		DMSG_PANIC("ERR: get usb%d phy_gate failed.\n", sw_hci->usbc_no);
		goto failed;
	}

	sw_hci->phy_reset = clk_get(NULL, usbc_phy_reset_name[sw_hci->usbc_no]);
	if (IS_ERR(sw_hci->phy_reset)){
		DMSG_PANIC("ERR: get usb%d phy_reset failed.\n", sw_hci->usbc_no);
		goto failed;
	}

	return 0;

failed:
	if(sw_hci->sie_clk){
		clk_put(sw_hci->sie_clk);
		sw_hci->sie_clk = NULL;
	}

	if(sw_hci->phy_gate){
		clk_put(sw_hci->phy_gate);
		sw_hci->phy_gate = NULL;
	}

	if(sw_hci->phy_reset){
		clk_put(sw_hci->phy_reset);
		sw_hci->phy_reset = NULL;
	}

	if(sw_hci->ohci_gate){
	    clk_put(sw_hci->ohci_gate);
		sw_hci->ohci_gate = NULL;
	}

	return -1;
}

/*
*******************************************************************************
*                     clock_exit
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static s32 clock_exit(struct sw_hci_hcd *sw_hci, u32 ohci)
{
	if(sw_hci->ohci_gate){
		clk_put(sw_hci->ohci_gate);
		sw_hci->ohci_gate = NULL;
	}

	if(sw_hci->sie_clk){
		clk_put(sw_hci->sie_clk);
		sw_hci->sie_clk = NULL;
	}

	if(sw_hci->phy_gate){
		clk_put(sw_hci->phy_gate);
		sw_hci->phy_gate = NULL;
	}

	if(sw_hci->phy_reset){
		clk_put(sw_hci->phy_reset);
		sw_hci->phy_reset = NULL;
	}

	return 0;
}

/*
*******************************************************************************
*                     open_clock
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int open_clock(struct sw_hci_hcd *sw_hci, u32 ohci)
{
 	DMSG_INFO("[%s]: open clock\n", sw_hci->hci_name);

    if(sw_hci->sie_clk && sw_hci->phy_gate
       && sw_hci->phy_reset && !sw_hci->clk_is_open){
        sw_hci->clk_is_open = 1;

 	    clk_enable(sw_hci->phy_gate);
	    clk_enable(sw_hci->phy_reset);
		  clk_reset(sw_hci->phy_reset, 0);

        if(ohci && sw_hci->ohci_gate){
            clk_enable(sw_hci->ohci_gate);
        }

        mdelay(10);

   	    clk_enable(sw_hci->sie_clk);

        mdelay(10);

    	UsbPhyInit(sw_hci->usbc_no);
    }else{
		DMSG_PANIC("[%s]: wrn: open clock failed, (0x%p, 0x%p, 0x%p, %d, 0x%p)\n",
			      sw_hci->hci_name,
			      sw_hci->sie_clk, sw_hci->phy_gate,
			      sw_hci->phy_reset, sw_hci->clk_is_open,
			      sw_hci->ohci_gate);
	}

    DMSG_DEBUG("[%s]: open clock, 0x60(0x%x), 0xcc(0x%x)\n",
              sw_hci->hci_name,
              (u32)USBC_Readl(SW_VA_CCM_IO_BASE + 0x60),
              (u32)USBC_Readl(SW_VA_CCM_IO_BASE + 0xcc));

	return 0;
}

/*
*******************************************************************************
*                     close_clock
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int close_clock(struct sw_hci_hcd *sw_hci, u32 ohci)
{
 	DMSG_INFO("[%s]: close clock\n", sw_hci->hci_name);

    if(sw_hci->sie_clk && sw_hci->phy_gate
       && sw_hci->phy_reset && sw_hci->clk_is_open){

    	sw_hci->clk_is_open = 0;

        if(ohci && sw_hci->ohci_gate){
	        clk_disable(sw_hci->ohci_gate);
	    }

		clk_reset(sw_hci->phy_reset, 1);
	    clk_disable(sw_hci->phy_reset);
	    clk_disable(sw_hci->phy_gate);

	    clk_disable(sw_hci->sie_clk);
    }else{
		DMSG_PANIC("[%s]: wrn: open clock failed, (0x%p, 0x%p, 0x%p, %d, 0x%p)\n",
			      sw_hci->hci_name,
			      sw_hci->sie_clk, sw_hci->phy_gate,
			      sw_hci->phy_reset, sw_hci->clk_is_open,
			      sw_hci->ohci_gate);
	}

    DMSG_DEBUG("[%s]: close clock, 0x60(0x%x), 0xcc(0x%x)\n",
              sw_hci->hci_name,
              (u32)USBC_Readl(SW_VA_CCM_IO_BASE + 0x60),
              (u32)USBC_Readl(SW_VA_CCM_IO_BASE + 0xcc));

	return 0;
}

#else

static s32 clock_init(struct sw_hci_hcd *sw_hci, u32 ohci)

{
	return 0;
}

static s32 clock_exit(struct sw_hci_hcd *sw_hci, u32 ohci)

{
	return 0;
}

/*
*******************************************************************************
*                     open_usb_clock
*
* Description:
*
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
//static u32  open_usb_clock(sw_hcd_io_t *sw_hcd_io)
static int open_clock(struct sw_hci_hcd *sw_hci, u32 ohci)
{
	u32 reg_value = 0;
	u32 ccmu_base = SW_VA_CCM_IO_BASE;

 	//DMSG_INFO_HCD0("[%s]: open_usb_clock\n", sw_hcd_driver_name);

	//Gating AHB clock for USB_phy0
	reg_value = USBC_Readl(ccmu_base + 0x60);
	reg_value |= (1 << 2);
	reg_value |= (1 << 1);	            /* AHB clock gate usb1 */
	USBC_Writel(reg_value, (ccmu_base + 0x60));

	//delay to wati SIE stable
	reg_value = 10000;
	while(reg_value--);

	//Enable module clock for USB phy0
	reg_value = USBC_Readl(ccmu_base + 0xcc);
	reg_value |= (1 << 9);
	reg_value |= (1 << 8);
	reg_value |= (1 << 6);
	reg_value |= (1 << 1);
	reg_value |= (1 << 0);          //disable reset
	USBC_Writel(reg_value, (ccmu_base + 0xcc));

	//delay some time
	reg_value = 10000;
	while(reg_value--);

	sw_hci->clk_is_open = 1;

	DMSG_INFO("[hcd1]: open, 0x60(0x%x), 0xcc(0x%x)\n",
	      (u32)USBC_Readl(SW_VA_CCM_IO_BASE + 0x60),
	      (u32)USBC_Readl(SW_VA_CCM_IO_BASE + 0xcc));

	return 0;
}

/*
*******************************************************************************
*                     close_usb_clock
*
* Description:
*
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/

//static u32 close_usb_clock(sw_hcd_io_t *sw_hcd_io)
static int close_clock(struct sw_hci_hcd *sw_hci, u32 ohci)
{
#if 0
	u32 reg_value = 0;
	u32 ccmu_base = SW_VA_CCM_IO_BASE;

 	//DMSG_INFO_HCD0("[%s]: close_usb_clock\n", sw_hcd_driver_name);

	//Gating AHB clock for USB_phy0
	reg_value = USBC_Readl(ccmu_base + 0x60);
	reg_value &= ~(1 << 2);
	reg_value &= ~(1 << 1);	             /* AHB clock gate usb1 */
	USBC_Writel(reg_value, (ccmu_base + 0x60));

	//��sie��ʱ�ӱ���
	reg_value = 10000;
	while(reg_value--);

	//Enable module clock for USB phy0
	reg_value = USBC_Readl(ccmu_base + 0xcc);
	reg_value &= ~(1 << 9);
	reg_value &= ~(1 << 8);
	reg_value &= ~(1 << 6);
	reg_value &= ~(1 << 1);
	reg_value &= ~(1 << 0);          //disable reset
	USBC_Writel(reg_value, (ccmu_base + 0xcc));

	//��ʱ
	reg_value = 10000;
	while(reg_value--);

	sw_hci->clk_is_open = 0;


	DMSG_INFO("[hcd1]: close, 0x60(0x%x), 0xcc(0x%x)\n",
	      (u32)USBC_Readl(SW_VA_CCM_IO_BASE + 0x60),
	      (u32)USBC_Readl(SW_VA_CCM_IO_BASE + 0xcc));
#endif
	return 0;
}

#endif

/*
*******************************************************************************
*                     enable_usb_passby
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void usb_passby(struct sw_hci_hcd *sw_hci, u32 enable)
{
	unsigned long reg_value = 0;
	spinlock_t lock;
	unsigned long flags = 0;

	spin_lock_init(&lock);
	spin_lock_irqsave(&lock, flags);


	DMSG_INFO("usb_passby en=%d,passly=%d\n",enable, usb_enable_passly_cnt);

	/*enable passby*/
	if(enable && usb_enable_passly_cnt == 0){
    	reg_value = USBC_Readl(sw_hci->usb_vbase + SW_USB_PMU_IRQ_ENABLE);
    	reg_value |= (1 << 10);		/* AHB Master interface INCR8 enable */
    	reg_value |= (1 << 9);     	/* AHB Master interface burst type INCR4 enable */
    	reg_value |= (1 << 8);     	/* AHB Master interface INCRX align enable */
    	reg_value |= (1 << 0);     	/* ULPI bypass enable */
    	USBC_Writel(reg_value, (sw_hci->usb_vbase + SW_USB_PMU_IRQ_ENABLE));
	}else if(!enable && usb_enable_passly_cnt == 1){
    	reg_value = USBC_Readl(sw_hci->usb_vbase + SW_USB_PMU_IRQ_ENABLE);
    	reg_value &= ~(1 << 10);	/* AHB Master interface INCR8 disable */
    	reg_value &= ~(1 << 9);     /* AHB Master interface burst type INCR4 disable */
    	reg_value &= ~(1 << 8);     /* AHB Master interface INCRX align disable */
    	reg_value &= ~(1 << 0);     /* ULPI bypass disable */
    	USBC_Writel(reg_value, (sw_hci->usb_vbase + SW_USB_PMU_IRQ_ENABLE));
	}

  if(enable){
      usb_enable_passly_cnt++;
  }else{
      usb_enable_passly_cnt--;
  }

	spin_unlock_irqrestore(&lock, flags);

    return;
}

/*
*******************************************************************************
*                     hci_port_configure
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void hci_port_configure(struct sw_hci_hcd *sw_hci, u32 enable)
{
	unsigned long reg_value = 0;
	u32 usbc_sdram_hpcr = 0;

	DMSG_INFO("hci_port_configure en=%d,config_cnt=%d\n",enable, usb_enable_configure_cnt);

	if(sw_hci->usbc_no == 1){
		usbc_sdram_hpcr = SW_SDRAM_REG_HPCR_USB1;
	}else if(sw_hci->usbc_no == 2){
		usbc_sdram_hpcr = SW_SDRAM_REG_HPCR_USB2;
	}else{
		DMSG_PANIC("EER: unkown usbc_no(%d)\n", sw_hci->usbc_no);
		return;
	}

	reg_value = USBC_Readl(sw_hci->sdram_vbase + usbc_sdram_hpcr);
	if(enable && usb_enable_configure_cnt == 0){
		reg_value |= (1 << SW_SDRAM_BP_HPCR_ACCESS_EN);
	}else if(!enable && usb_enable_configure_cnt == 1){
		reg_value &= ~(1 << SW_SDRAM_BP_HPCR_ACCESS_EN);
	}

	if(enable){
		usb_enable_configure_cnt++;
  }else{
  	usb_enable_configure_cnt--;
  }

	USBC_Writel(reg_value, (sw_hci->sdram_vbase + usbc_sdram_hpcr));

	return;
}

/*
*******************************************************************************
*                     pin_init
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static u32 alloc_pin(struct sw_hci_hcd *sw_hci, user_gpio_set_t *gpio_list)
{
    u32 pin_handle = 0;
    char name[32];

    memset(name, 0, 32);

    if(gpio_list->port == 0xffff){  //axp
        if(gpio_list->port_num >= USB_EXTERN_PIN_LDO_MASK){
#ifdef  CONFIG_REGULATOR
            switch((gpio_list->port_num - USB_EXTERN_PIN_LDO_MASK)){
                case 1:
                    strcpy(name, "axp20_rtc");
                break;

                case 2:
                    strcpy(name, "axp20_analog/fm");
                break;

                case 3:
                    strcpy(name, "axp20_pll");
                break;

                case 4:
                    strcpy(name, "axp20_hdmi");
                break;

                default:
                    DMSG_PANIC("ERR: unkown gpio_list->port_num(%d)\n", gpio_list->port_num);
                    goto failed;
            }

            pin_handle = (u32)regulator_get(NULL, name);
            if(pin_handle == 0){
                DMSG_PANIC("ERR: regulator_get failed\n");
                return 0;
            }

            regulator_force_disable((struct regulator*)pin_handle);
#else
			DMSG_PANIC("CONFIG_REGULATOR is not define\n");
#endif
        }else{
            axp_gpio_set_io(gpio_list->port_num, gpio_list->mul_sel);
            axp_gpio_set_value(gpio_list->port_num, gpio_list->data);

            return (100 + gpio_list->port_num);
        }
	}else{  //gpio
        pin_handle = gpio_request(gpio_list, 1);
        if(pin_handle == 0){
            DMSG_PANIC("ERR: gpio_request failed\n");
            return 0;
        }

        /* set config, ouput */
        gpio_set_one_pin_io_status(pin_handle, 1, NULL);

        /* reserved is pull down */
        gpio_set_one_pin_pull(pin_handle, 2, NULL);
	}

failed:
	return pin_handle;
}

/*
*******************************************************************************
*                     pin_exit
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void free_pin(u32 pin_handle, user_gpio_set_t *gpio_list)
{
    if(pin_handle){
        if(gpio_list->port == 0xffff){ //axp
            if(gpio_list->port_num >= USB_EXTERN_PIN_LDO_MASK){
#ifdef  CONFIG_REGULATOR
                regulator_force_disable((struct regulator*)pin_handle);
#else
			DMSG_PANIC("CONFIG_REGULATOR is not define\n");
#endif
            }else{
                axp_gpio_set_io(gpio_list->port_num, gpio_list->mul_sel);
                axp_gpio_set_value(gpio_list->port_num, gpio_list->data);
            }
        }else{  //gpio
        	gpio_release(pin_handle, 0);
    	}
    }

	return;
}


/*
*******************************************************************************
*                     __sw_set_vbus_ex
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void __sw_set_vbus_ex(user_gpio_set_t *drv_vbus_gpio_set, u32 drv_vbus_Handle, int is_on)
{
    u32 on_off = 0;

    /* set power */
    if(drv_vbus_gpio_set->data == 0){
        on_off = is_on ? 1 : 0;
    }else{
        on_off = is_on ? 0 : 1;
    }

    if(drv_vbus_gpio_set->port == 0xffff){ //axp
        if(drv_vbus_gpio_set->port_num >= USB_EXTERN_PIN_LDO_MASK){
#ifdef  CONFIG_REGULATOR
            if(is_on){
                regulator_enable((struct regulator*)drv_vbus_Handle);
            }else{
                regulator_disable((struct regulator*)drv_vbus_Handle);
            }
#else
			DMSG_PANIC("CONFIG_REGULATOR is not define\n");
#endif
        }else{
            axp_gpio_set_value(drv_vbus_gpio_set->port_num, on_off);
        }
    }else{  //gpio
        gpio_write_one_pin_value(drv_vbus_Handle, on_off, NULL);
	}

	return;
}


/*
*******************************************************************************
*                     __sw_set_vbus
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void __sw_set_vbus(struct sw_hci_hcd *sw_hci, int is_on)
{
    u32 on_off = 0;

    if(sw_hci->drv_vbus_Handle == 0){
        DMSG_PANIC("wrn: sw_hci->drv_vbus_Handle is null\n");
        return;
    }

	DMSG_INFO("[%s]: Set USB Power %s\n", sw_hci->hci_name, (is_on ? "ON" : "OFF"));

    /* set power flag */
	sw_hci->power_flag = is_on;

	__sw_set_vbus_ex(&sw_hci->drv_vbus_gpio_set, sw_hci->drv_vbus_Handle, is_on);
	__sw_set_vbus_ex(&sw_hci->drv_vbus_1_gpio_set, sw_hci->drv_vbus_1_Handle, is_on);

	return;
}

/*
*******************************************************************************
*                     sw_set_vbus
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void sw_set_vbus(struct sw_hci_hcd *sw_hci, int is_on)
{
    if(sw_hci->usbc_no == 1){
        if(is_on)
            usb1_set_vbus_cnt++;
        else
            usb1_set_vbus_cnt--;
        
        printk("[%s]: sw_set_vbus cnt %d\n",
              sw_hci->hci_name, usb1_set_vbus_cnt);
        if(is_on && usb1_set_vbus_cnt == 1){
            __sw_set_vbus(sw_hci, 1);  /* power on */
        }else if(!is_on && usb1_set_vbus_cnt == 0){
            __sw_set_vbus(sw_hci, 0);  /* power off */
        }
    }else{
        if(is_on)
            usb2_set_vbus_cnt++;
        else
            usb2_set_vbus_cnt--;
            
        printk("[%s]: sw_set_vbus cnt %d\n",
              sw_hci->hci_name, usb2_set_vbus_cnt);
        if(is_on && usb2_set_vbus_cnt == 1){
            __sw_set_vbus(sw_hci, 1);  /* power on */
        }else if(!is_on && usb2_set_vbus_cnt == 0){
            __sw_set_vbus(sw_hci, 0);  /* power off */
        }
    }

	return;
}

int hci_get_vbus_status(int usbc_no)
{
    if(usbc_no == 1)
        return usb1_set_vbus_cnt > 0? 1:0;
    else if(usbc_no == 2)
        return usb2_set_vbus_cnt > 0? 1:0;
    else
        printk("invalid usbc_no : %d\n", usbc_no);

    return 0;
}
EXPORT_SYMBOL(hci_get_vbus_status);
//---------------------------------------------------------------
//  EHCI
//---------------------------------------------------------------

#define  SW_EHCI_NAME		"sw-ehci"
static const char ehci_name[] = SW_EHCI_NAME;

static struct sw_hci_hcd sw_ehci0;
static struct sw_hci_hcd sw_ehci1;

static u64 sw_ehci_dmamask = DMA_BIT_MASK(32);

static struct platform_device sw_usb_ehci_device[] = {
	[0] = {
		.name		= ehci_name,
		.id			= 1,
		.dev 		= {
			.dma_mask			= &sw_ehci_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &sw_ehci0,
		},
	},

	[1] = {
		.name		= ehci_name,
		.id			= 2,
		.dev 		= {
			.dma_mask			= &sw_ehci_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &sw_ehci1,
		},
	},
};

//---------------------------------------------------------------
//  OHCI
//---------------------------------------------------------------
#define  SW_OHCI_NAME		"sw-ohci"
static const char ohci_name[] = SW_OHCI_NAME;

static struct sw_hci_hcd sw_ohci0;
static struct sw_hci_hcd sw_ohci1;

static u64 sw_ohci_dmamask = DMA_BIT_MASK(32);

static struct platform_device sw_usb_ohci_device[] = {
	[0] = {
		.name		= ohci_name,
		.id			= 1,
		.dev 		= {
			.dma_mask			= &sw_ohci_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &sw_ohci0,
		},
	},

	[1] = {
		.name		= ohci_name,
		.id			= 2,
		.dev 		= {
			.dma_mask			= &sw_ohci_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &sw_ohci1,
		},
	},
};

/*
*******************************************************************************
*                     print_sw_hci
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void print_sw_hci(struct sw_hci_hcd *sw_hci)
{
	DMSG_DEBUG("\n------%s config------\n", sw_hci->hci_name);
	DMSG_DEBUG("hci_name             = %s\n", sw_hci->hci_name);
	DMSG_DEBUG("irq_no               = %d\n", sw_hci->irq_no);
	DMSG_DEBUG("usbc_no              = %d\n", sw_hci->usbc_no);

	DMSG_DEBUG("usb_vbase            = 0x%p\n", sw_hci->usb_vbase);
	DMSG_DEBUG("sram_vbase           = 0x%p\n", sw_hci->sram_vbase);
	DMSG_DEBUG("clock_vbase          = 0x%p\n", sw_hci->clock_vbase);
	DMSG_DEBUG("sdram_vbase          = 0x%p\n", sw_hci->sdram_vbase);

	DMSG_DEBUG("used                 = %d\n", sw_hci->used);
	DMSG_DEBUG("host_init_state      = %d\n", sw_hci->host_init_state);

	DMSG_DEBUG("drv_vbus_gpio_valid  = %d\n", sw_hci->drv_vbus_gpio_valid);
	DMSG_DEBUG("gpio_name            = %s\n", sw_hci->drv_vbus_gpio_set.gpio_name);
	DMSG_DEBUG("port                 = %d\n", sw_hci->drv_vbus_gpio_set.port);
	DMSG_DEBUG("port_num             = %d\n", sw_hci->drv_vbus_gpio_set.port_num);
	DMSG_DEBUG("mul_sel              = %d\n", sw_hci->drv_vbus_gpio_set.mul_sel);
	DMSG_DEBUG("pull                 = %d\n", sw_hci->drv_vbus_gpio_set.pull);
	DMSG_DEBUG("drv_level            = %d\n", sw_hci->drv_vbus_gpio_set.drv_level);
	DMSG_DEBUG("data                 = %d\n", sw_hci->drv_vbus_gpio_set.data);

	DMSG_DEBUG("\n--------------------------\n");

    return;
}

/*
*******************************************************************************
*                     init_sw_hci
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int init_sw_hci(struct sw_hci_hcd *sw_hci, u32 usbc_no, u32 ohci, const char *hci_name)
{
    s32 ret = 0;

    memset(sw_hci, 0, sizeof(struct sw_hci_hcd));

    sw_hci->usbc_no = usbc_no;

    if(ohci){
        sw_hci->irq_no = ohci_irq_no[sw_hci->usbc_no];
    }else{
        sw_hci->irq_no = ehci_irq_no[sw_hci->usbc_no];
    }

    sprintf(sw_hci->hci_name, "%s%d", hci_name, sw_hci->usbc_no);

	sw_hci->usb_vbase		= (void __iomem	*)usbc_base[sw_hci->usbc_no];
	sw_hci->sram_vbase		= (void __iomem	*)SW_VA_SRAM_IO_BASE;
	sw_hci->clock_vbase     = (void __iomem	*)SW_VA_CCM_IO_BASE;
	sw_hci->gpio_vbase		= (void __iomem	*)SW_VA_PORTC_IO_BASE;
	sw_hci->sdram_vbase     = (void __iomem	*)SW_VA_DRAM_IO_BASE;

	get_usb_cfg(sw_hci);
	sw_hci->open_clock          = open_clock;
	sw_hci->close_clock         = close_clock;
	sw_hci->set_power           = sw_set_vbus;
	sw_hci->usb_passby          = usb_passby;
    sw_hci->port_configure      = hci_port_configure;

    ret = clock_init(sw_hci, ohci);
    if(ret != 0){
        DMSG_PANIC("ERR: clock_init failed\n");
        goto failed1;
    }

    print_sw_hci(sw_hci);

    return 0;

failed1:

    return -1;
}

/*
*******************************************************************************
*                     exit_sw_hci
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int exit_sw_hci(struct sw_hci_hcd *sw_hci, u32 ohci)
{
    clock_exit(sw_hci, ohci);

    return 0;
}

/*
*******************************************************************************
*                     sw_hci_sun5i_init
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static int __init sw_hci_sun5i_init(void)
{
    /* USB1 */
    init_sw_hci(&sw_ehci0, 1, 0, ehci_name);
    init_sw_hci(&sw_ohci0, 1, 1, ohci_name);

    sw_ehci0.pdev = &sw_usb_ehci_device[0];
    sw_ohci0.pdev = &sw_usb_ohci_device[0];

    if(sw_ehci0.drv_vbus_gpio_valid){
        sw_ehci0.drv_vbus_Handle = alloc_pin(&sw_ehci0, &sw_ehci0.drv_vbus_gpio_set);
        if(sw_ehci0.drv_vbus_Handle == 0){
            DMSG_PANIC("ERR: drv_vbus_gpio_set alloc_pin failed\n");
            goto failed0;
        }

        sw_ohci0.drv_vbus_Handle = sw_ehci0.drv_vbus_Handle;
    }else{
        sw_ehci0.drv_vbus_Handle = 0;
        sw_ohci0.drv_vbus_Handle = 0;
    }

    if(sw_ehci0.drv_vbus_1_gpio_valid){
        sw_ehci0.drv_vbus_1_Handle = alloc_pin(&sw_ehci0, &sw_ehci0.drv_vbus_1_gpio_set);
        if(sw_ehci0.drv_vbus_1_Handle == 0){
            DMSG_PANIC("ERR: drv_vbus_1_gpio_set alloc_pin failed\n");
            goto failed0;
        }

        sw_ohci0.drv_vbus_1_Handle = sw_ehci0.drv_vbus_1_Handle;
    }else{
        sw_ehci0.drv_vbus_1_Handle = 0;
        sw_ohci0.drv_vbus_1_Handle = 0;
    }


#ifdef  CONFIG_USB_SW_SUN5I_EHCI0
    if(sw_ehci0.used){
		if(sw_ehci0.usb_controller_type != SW_USB_OHCI){
		#ifdef CONFIG_USB_PORT_POWER_MANAGEMENT
		    if(sw_ehci0.host_init_state)
		        usb1_set_vbus_cnt--;
		#endif
			platform_device_register(&sw_usb_ehci_device[0]);
		}
    }else{
		DMSG_PANIC("ERR: usb%d %s is disable\n", sw_ehci0.usbc_no, sw_ehci0.hci_name);
    }
#endif

#ifdef  CONFIG_USB_SW_SUN5I_OHCI0
    if(sw_ohci0.used){
		if(sw_ohci0.usb_controller_type != SW_USB_EHCI){
		#ifdef CONFIG_USB_PORT_POWER_MANAGEMENT
		    if(sw_ohci0.host_init_state)
		        usb1_set_vbus_cnt--;
		#endif
  	    	platform_device_register(&sw_usb_ohci_device[0]);
		}
    }else{
		DMSG_PANIC("ERR: usb%d %s is disable\n", sw_ohci0.usbc_no, sw_ohci0.hci_name);
    }
#endif

#ifdef  CONFIG_USB_SW_SUN5I_EHCI1
    if(sw_ehci1.used){
        #ifdef CONFIG_USB_PORT_POWER_MANAGEMENT
            if(sw_ehci1.host_init_state)
		        usb2_set_vbus_cnt--;
		#endif
     	platform_device_register(&sw_usb_ehci_device[1]);
    }else{
		DMSG_PANIC("ERR: usb%d %s is disable\n", sw_ehci1.usbc_no, sw_ehci1.hci_name);
    }
#endif

#ifdef  CONFIG_USB_SW_SUN5I_OHCI1
    if(sw_ohci1.used){
        #ifdef CONFIG_USB_PORT_POWER_MANAGEMENT
            if(sw_ohci1.host_init_state)
		        usb2_set_vbus_cnt--;
		#endif
     	platform_device_register(&sw_usb_ohci_device[1]);
    }else{
		DMSG_PANIC("ERR: usb%d %s is disable\n", sw_ohci1.usbc_no, sw_ohci1.hci_name);
    }
#endif

    return 0;

failed0:
    return -1;
}

/*
*******************************************************************************
*                     sw_hci_sun5i_exit
*
* Description:
*    void
*
* Parameters:
*    void
*
* Return value:
*    void
*
* note:
*    void
*
*******************************************************************************
*/
static void __exit sw_hci_sun5i_exit(void)
{
#ifdef  CONFIG_USB_SW_SUN5I_EHCI0
    if(sw_ehci0.used){
    	if(sw_ehci0.usb_controller_type != SW_USB_OHCI){
    	platform_device_unregister(&sw_usb_ehci_device[0]);
		}
    }else{
		DMSG_PANIC("ERR: usb%d %s is disable\n", sw_ehci0.usbc_no, sw_ehci0.hci_name);
    }
#endif

#ifdef  CONFIG_USB_SW_SUN5I_OHCI0
    if(sw_ohci0.used){
		if(sw_ohci0.usb_controller_type != SW_USB_EHCI){
			platform_device_unregister(&sw_usb_ohci_device[0]);
		}
    }else{
		DMSG_PANIC("ERR: usb%d %s is disable\n", sw_ohci0.usbc_no, sw_ohci0.hci_name);
    }
#endif

#ifdef  CONFIG_USB_SW_SUN5I_EHCI1
    if(sw_ehci1.used){
     	platform_device_unregister(&sw_usb_ehci_device[1]);
    }else{
		DMSG_PANIC("ERR: usb%d %s is disable\n", sw_ehci1.usbc_no, sw_ehci1.hci_name);
    }
#endif

#ifdef  CONFIG_USB_SW_SUN5I_OHCI1
    if(sw_ohci1.used){
     	platform_device_unregister(&sw_usb_ohci_device[1]);
    }else{
		DMSG_PANIC("ERR: usb%d %s is disable\n", sw_ohci1.usbc_no, sw_ohci1.hci_name);
    }
#endif

    /* USB1 */
    exit_sw_hci(&sw_ehci0, 0);
    exit_sw_hci(&sw_ohci0, 1);

    free_pin(sw_ehci0.drv_vbus_Handle, &sw_ehci0.drv_vbus_gpio_set);
    sw_ehci0.drv_vbus_Handle = 0;

    free_pin(sw_ehci0.drv_vbus_1_Handle, &sw_ehci0.drv_vbus_1_gpio_set);
    sw_ehci0.drv_vbus_1_Handle = 0;

    return ;
}

late_initcall(sw_hci_sun5i_init);
module_exit(sw_hci_sun5i_exit);

