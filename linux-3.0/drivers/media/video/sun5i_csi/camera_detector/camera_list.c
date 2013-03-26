#include "camera_list.h"

__hdle camera_gpio_request(user_gpio_set_t *gpio_list, __u32 group_count_max)
{    
    //__inf("camera_GPIO_Request, port:%d, port_num:%d, mul_sel:%d, pull:%d, drv_level:%d, data:%d\n", gpio_list->port, gpio_list->port_num, gpio_list->mul_sel, gpio_list->pull, gpio_list->drv_level, gpio_list->data);
    
    if(gpio_list->port == 0xffff)
    {
        if(gpio_list->mul_sel == 0 || gpio_list->mul_sel == 1)
        {
            axp_gpio_set_io(gpio_list->port_num, gpio_list->mul_sel);
            axp_gpio_set_value(gpio_list->port_num, gpio_list->data);
            return 100+gpio_list->port_num;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return gpio_request(gpio_list, group_count_max);
    }
}

//if_release_to_default_status:
    //如果是0或者1，表示释放后的GPIO处于输入状态，输入状状态不会导致外部电平的错误。
    //如果是2，表示释放后的GPIO状态不变，即释放的时候不管理当前GPIO的硬件寄存器。
__s32 camera_gpio_release(__hdle p_handler, __s32 if_release_to_default_status)
{
    //__inf("OSAL_GPIO_Release\n");
    if(p_handler < 200 && p_handler >=100)
    {
        return 0;
    }
    else
    {
        return gpio_release(p_handler, if_release_to_default_status);
    }
}

__s32 camera_gpio_write(user_gpio_set_t *gpio_list, __u32 gpio_value)
{
    user_gpio_set_t  gpio_info[1];
    __hdle hdl;
    
    if (gpio_list->port) {
        memcpy(gpio_info, gpio_list, sizeof(user_gpio_set_t));
        
        gpio_info->data = gpio_value;
        
        hdl = camera_gpio_request(gpio_info, 1);
        camera_gpio_release(hdl, 2);
    }

    return 0;
}

__s32 camera_gpio_set_status(user_gpio_set_t *gpio_list, __u32 gpio_status)
{
    user_gpio_set_t  gpio_info[1];
    __hdle hdl;
    
    if (gpio_list->port) {
        memcpy(gpio_info, gpio_list, sizeof(user_gpio_set_t));
        
        gpio_info->mul_sel = gpio_status;
        
        hdl = camera_gpio_request(gpio_info, 1);
        camera_gpio_release(hdl, 2);
    }
    
    return 0;
}

__u32 camera_i2c_read(struct i2c_adapter *i2c_adap, __u8 *reg, __u8 *value, 
                             __u32 i2c_addr, __u32 REG_ADDR_STEP, __u32 REG_DATA_STEP)
{
	__u8 data[4];
	struct i2c_msg msg;
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
		data[i] = reg[i];
	
	data[REG_ADDR_STEP] = 0xff;
	/*
	 * Send out the register address...
	 */
	msg.addr = i2c_addr>>1;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP;
	msg.buf = data;
	ret = i2c_transfer(i2c_adap, &msg, 1);
	if (ret < 0) {
		camera_err("Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	
	msg.flags = I2C_M_RD;
	msg.len = REG_DATA_STEP;
	msg.buf = &data[REG_ADDR_STEP];
	
	ret = i2c_transfer(i2c_adap, &msg, 1);
	if (ret >= 0) {
		for(i = 0; i < REG_DATA_STEP; i++)
			value[i] = data[i+REG_ADDR_STEP];
		ret = 0;
	}
	else {
		camera_err("Error %d on register read\n", ret);
	}
	return ret;
}

__u32 camera_i2c_write(struct i2c_adapter *i2c_adap, __u8 *reg, __u8 *value, 
                              __u32 i2c_addr, __u32 REG_ADDR_STEP, __u32 REG_DATA_STEP)
{
	struct i2c_msg msg;
	unsigned char data[4];
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++) {
	    data[i] = reg[i];
        camera_msg("reg[%d]:%d->data[%d]:%d\n", i, reg[i], i, data[i]);
	}
	for(i = REG_ADDR_STEP; i < (REG_ADDR_STEP + REG_DATA_STEP); i++) {
        camera_msg("value[%d]:%d->data[%d]:%d\n", 
                                i-REG_ADDR_STEP, value[i], i, data[i]);
		data[i] = value[i-REG_ADDR_STEP];
	}
	
	msg.addr = i2c_addr>>1;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP + REG_DATA_STEP;
	msg.buf = data;
    
	ret = i2c_transfer(i2c_adap, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}
	else if (ret < 0) {
		camera_err("sensor_write error!, error number: %d \n", ret);
	}
	return ret;
}

#define __OV7670__

static void camera_pwr_on_ov7670(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);    
}

static void camera_pwr_off_ov7670(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input    
}

static __s32 camera_detect_ov7670(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    char reg_num[4]        = {0x1c, 0x1d, 0x0a, 0x0b};
    char expect_value[4]   = {0x7f, 0xa2, 0x76, 0x73};
    char value;
    __u32 i;
    __s32 ret = 0;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect ov7670 ... \n");
    
    for (i = 0; i < 4; i++) {
    	ret = camera_i2c_read(i2c_adap, &reg_num[i], &value, i2c_addr, addr_step, data_step);
    	if (ret < 0) {
    		return ret;
    	}
        if (value != expect_value[i]) {
            return -ENODEV;
        }
    }
    
    camera_msg("detect ov7670 success!!\n");

    return 0;
}

#define __GC0308__

static void camera_pwr_on_gc0308(__u32 list_index, __camera_info_t *camera_info)
{    
    camera_gpio_set_status(camera_info->stby_pin, 1);
    camera_gpio_set_status(camera_info->reset_pin, 1);
    
    //power supply
    camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
    if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
	}
    
    mdelay(10);
    
    //standby off io
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
    mdelay(10);
    
    clk_enable(camera_info->module_clk);
    mdelay(10);

    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
    mdelay(30);

    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);    
}

static void camera_pwr_off_gc0308(__u32 list_index, __camera_info_t *camera_info)
{    
    //reset on io
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
    mdelay(10);
    //inactive mclk after power off
    clk_disable(camera_info->module_clk);
    //power supply off
    if(camera_info->iovdd) {
    	regulator_disable(camera_info->iovdd);
    }
    if(camera_info->avdd) {
    	regulator_disable(camera_info->avdd);
    }
    if(camera_info->dvdd) {
    	regulator_disable(camera_info->dvdd);
    }
    camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
    mdelay(10);
    //standby of io
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
    mdelay(10);
    //set the io to hi-z
    camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
    camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input
}

static __s32 camera_detect_gc0308(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    char reg_num[1], value[1];
    __s32 ret = 0;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect gc0308 ... \n");

    reg_num[0] = 0xfe;
	value[0] = 0x00; //PAGE 0x00
	ret = camera_i2c_write(i2c_adap, reg_num, value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
    	
	reg_num[0] = 0x00;
	ret = camera_i2c_read(i2c_adap, reg_num, value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
    
	if(value[0] != 0x9b)
		return -ENODEV;
	
	camera_msg("detect gc0308 success!!\n");
    
    return 0;
}

#define __GT2005__

static void camera_pwr_on_gt2005(__u32 list_index, __camera_info_t *camera_info)
{    
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin, 1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin, 1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(30);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
}

static void camera_pwr_off_gt2005(__u32 list_index, __camera_info_t *camera_info)
{   
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(10);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input
}

static __s32 camera_detect_gt2005(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect gt2005 ... \n");
    
	regs.reg_num[0] = 0x00;
	regs.reg_num[1] = 0x00;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
    
	if(regs.value[0] != 0x51)
		return -ENODEV;
    
    camera_msg("detect gt2005 success!!\n");
    
	return 0;           
}

#define __HI704__

static void camera_pwr_on_hi704(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);
}

static void camera_pwr_off_hi704(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input
}

static __s32 camera_detect_hi704(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect hi704 ... \n");
    
	regs.reg_num[0] = 0x03;
	regs.value[0] = 0x00; //PAGE 0x00
	ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	regs.reg_num[0] = 0x04;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

	if(regs.value[0] != 0x96)
		return -ENODEV;
    
    camera_msg("detect hi704 success!!\n");
    
	return 0;    
}

#define __SP0838__

static void camera_pwr_on_sp0838(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin, 1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin, 1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
}

static void camera_pwr_off_sp0838(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input
}

static __s32 camera_detect_sp0838(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;    
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect sp0838 ... \n");
    
	regs.reg_num[0] = 0xfd;
	regs.value[0] = 0x00; //PAGE 0x00
	ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	regs.reg_num[0] = 0x02;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

	if(regs.value[0] != 0x27)
		return -ENODEV;
    
    camera_msg("detect sp0838 success!!\n");
    
	return 0;
}

#define __MT9M112__

static void camera_pwr_on_mt9m112(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);    
}

static void camera_pwr_off_mt9m112(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input        
}

static __s32 camera_detect_mt9m112(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;
    
    camera_msg("try to detect mt9m112 ... \n");
    
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00; //PAGE 0x00
	regs.value[1] = 0x00;
	ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	regs.reg_num[0] = 0x00;
	regs.reg_num[1] = 0x00;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

	if(regs.value[0] != 0x14)
		return -ENODEV;

    camera_msg("detect mt9m112 success!!\n");
    
	return 0;
}

#define __MT9M113__

static void camera_pwr_on_mt9m113(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
    camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
    camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
    //reset on io
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
    mdelay(1);
    //active mclk before power on
    clk_enable(camera_info->module_clk);
    mdelay(10);
    //power supply
    camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
    mdelay(10);
    if(camera_info->dvdd) {
    	regulator_enable(camera_info->dvdd);
    	mdelay(10);
    }
    if(camera_info->avdd) {
    	regulator_enable(camera_info->avdd);
    	mdelay(10);
    }
    if(camera_info->iovdd) {
    	regulator_enable(camera_info->iovdd);
    	mdelay(10);
    }
    //standby off io
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
    mdelay(10);
    //reset after power on
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
    mdelay(10);
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
    mdelay(100);
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
    mdelay(100);   
}

static void camera_pwr_off_mt9m113(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
    mdelay(100);
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
    mdelay(100);
    //power supply off
    if(camera_info->iovdd) {
    	regulator_disable(camera_info->iovdd);
    	mdelay(10);
    }
    if(camera_info->avdd) {
    	regulator_disable(camera_info->avdd);
    	mdelay(10);
    }
    if(camera_info->dvdd) {
    	regulator_disable(camera_info->dvdd);
    	mdelay(10);	
    }
    camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
    mdelay(10);
    //inactive mclk after power off
    clk_disable(camera_info->module_clk);
    //set the io to hi-z
    camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
    camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input   
}

static __s32 camera_detect_mt9m113(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
    struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect mt9m113 ... \n");

    regs.reg_num[0] = 0x00;
    regs.reg_num[1] = 0x00;
    ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
    if (ret < 0) {
    	return ret;
    }

    if(regs.value[0] != 0x24)
    	return -ENODEV;
    
    camera_msg("detect mt9m113 success!!\n");
    
    return 0;   
}

#define __OV2655__

static void camera_pwr_on_ov2655(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin, 1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin, 1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);
}

static void camera_pwr_off_ov2655(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
    mdelay(100);
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
    mdelay(100);
    //power supply off
    if(camera_info->iovdd) {
    	regulator_disable(camera_info->iovdd);
    	mdelay(10);
    }
    if(camera_info->avdd) {
    	regulator_disable(camera_info->avdd);
    	mdelay(10);
    }
    if(camera_info->dvdd) {
    	regulator_disable(camera_info->dvdd);
    	mdelay(10);	
    }
    camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
    mdelay(10);
    //inactive mclk after power off
    clk_disable(camera_info->module_clk);
    //set the io to hi-z
    camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
    camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input    
}

static __s32 camera_detect_ov2655(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect ov2655 ... \n");
    
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x0A;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

	if(regs.value[0] != 0x26)
		return -ENODEV;
	
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x0B;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

	if(regs.value[0] != 0x56)
		return -ENODEV;
    
    camera_msg("detect ov2655 success!!\n");
    
	return 0;
}

#define __HI253__

static void camera_pwr_on_hi253(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);
}

static void camera_pwr_off_hi253(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input
}

static __s32 camera_detect_hi253(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect hi253 ... \n");
    
	regs.reg_num[0] = 0x03;
	regs.value[0] = 0x00; //PAGE 0x00
	ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	regs.reg_num[0] = 0x04;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

	if(regs.value[0] != 0x92)
		return -ENODEV;
    
    camera_msg("detect hi253 success!!\n");
    
	return 0;    
}

#define __GC0307__

static void camera_pwr_on_gc0307(__u32 list_index, __camera_info_t *camera_info)
{   
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin, 1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin, 1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);
}

static void camera_pwr_off_gc0307(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input    
}

static __s32 camera_detect_gc0307(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	char reg_num[1], value[1];
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect gc0307 ... \n");
    
	reg_num[0] = 0xfe;
	value[0] = 0x00; //PAGE 0x00
	ret = camera_i2c_write(i2c_adap, reg_num, value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	reg_num[0] = 0x00;
	ret = camera_i2c_read(i2c_adap, reg_num, value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	if(value[0] != 0x99)
		return -ENODEV;
	
	camera_msg("detect gc0307 success!!\n");
    
    return 0;
}

#define __MT9D112__

static void camera_pwr_on_mt9d112(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);
}

static void camera_pwr_off_mt9d112(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input    
}

static __s32 camera_detect_mt9d112(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect mt9d112 ... \n");
    
	regs.value[0] = 0x00; 
	regs.value[1] = 0x00;
	
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x00;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

	if(regs.value[0] != 0x15)//0x1580
		return -ENODEV;

    camera_msg("detect mt9d112 success!!\n");
    
	return 0;    
}

#define __OV5640__

static void camera_pwr_on_ov5640(__u32 list_index, __camera_info_t *camera_info)
{
    __u32 CSI_AF_PWR_ON = 1;
    
    //power on reset
    camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
    camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
    //reset on io
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
    mdelay(1);
    //active mclk before power on
    clk_enable(camera_info->module_clk);
    mdelay(10);
    //power supply
    camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
    if(camera_info->iovdd) {
        regulator_enable(camera_info->iovdd);
    }
    if(camera_info->dvdd) {
        regulator_enable(camera_info->dvdd);
    }
    if(camera_info->avdd) {
        regulator_enable(camera_info->avdd);
    }
    
    camera_gpio_write(camera_info->af_pwr_pin, CSI_AF_PWR_ON);
    mdelay(10);
    
    //standby off io
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
    mdelay(10);
    //reset after power on
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
    mdelay(30);
}

static void camera_pwr_off_ov5640(__u32 list_index, __camera_info_t *camera_info)
{
    __u32 CSI_AF_PWR_OFF = 0;
    
    //inactive mclk before power off
	clk_disable(camera_info->module_clk);
	//power supply off
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
    
	camera_gpio_write(camera_info->af_pwr_pin, CSI_AF_PWR_OFF);
    
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
	}
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
	}
    
	//standby and reset io
	mdelay(10);
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);

	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input
}

static __s32 camera_detect_ov5640(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;
    
    camera_msg("try to detect ov5640 ... \n");
    
    regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x0a;
    ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

    if (regs.value[0] != 0x56) {
        return -ENODEV;
    }
    
    regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x0b;
    ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
    
    if (regs.value[0] != 0x40) {
        return -ENODEV;
    }
    
    camera_msg("detect ov5640 success!!\n");

    return 0;
}

#define __GC2015__

static void camera_pwr_on_gc2015(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin, 1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin, 1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);
}

static void camera_pwr_off_gc2015(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input
}

static __s32 camera_detect_gc2015(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect gc2015 ... \n");
    
	regs.reg_num[0] = 0xfe;
	regs.value[0]   = 0x80;           //for GC2015 SOFT reset!  2012-3-15
	ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00; //PAGE 0x00
	ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	regs.reg_num[0] = 0x00;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	if(regs.value[0] != 0x20)
		return -ENODEV;
	
	regs.reg_num[0] = 0x01;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	if(regs.value[0] != 0x05)
		return -ENODEV;
    
    camera_msg("detect gc2015 success!!\n");
    
	return 0;
}

#define __OV2643__

static void camera_pwr_on_ov2643(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin, 1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin, 1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(10);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);    
}

static void camera_pwr_off_ov2643(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin, 0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin, 0);//set the gpio to input    
}

static __s32 camera_detect_ov2643(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect ov2643 ... \n");
    
	regs.reg_num[0] = 0x0a;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	if(regs.value[0] != 0x26) {
		return -ENODEV;
	}
	
	regs.reg_num[0] = 0x0b;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	if(regs.value[0] != 0x43) {
		return -ENODEV;
	}
    
    camera_msg("detect ov2643 success!!\n");
    
	return 0;
}

#define __GC0329__

static void camera_pwr_on_gc0329(__u32 list_index, __camera_info_t *camera_info)
{
    //inactive mclk before power on
	clk_disable(camera_info->module_clk);
	//power on reset
	camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(10);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);
}

static void camera_pwr_off_gc0329(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input    
}

static __s32 camera_detect_gc0329(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect gc0329 ... \n");
	
	regs.reg_num[0] = 0xfc;
	regs.value[0] = 0x16; //enable digital clock
	ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	regs.reg_num[0] = 0x00;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

	if(regs.value[0] != 0xc0)
		return -ENODEV;
    
    camera_msg("detect gc0329 success!!\n");
    
	return 0;
}

#define __GC0309__

static void camera_pwr_on_gc0309(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(10);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(100);    
}

static void camera_pwr_off_gc0309(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(100);
	//power supply off
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
		mdelay(10);	
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	mdelay(10);
	//inactive mclk after power off
	clk_disable(camera_info->module_clk);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input
}

static __s32 camera_detect_gc0309(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00; //PAGE 0x00
	ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	regs.reg_num[0] = 0x00;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	if(regs.value[0] != 0xa0)
		return -ENODEV;
	
	return 0;
}

#define __TVP5150__

static void camera_pwr_on_tvp5150(__u32 list_index, __camera_info_t *camera_info)
{
    //power on reset
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	mdelay(1);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	mdelay(10);
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	mdelay(10);
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
		mdelay(10);
	}
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
		mdelay(10);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
		mdelay(10);
	}
	//standby off io
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	mdelay(30);
}

static void camera_pwr_off_tvp5150(__u32 list_index, __camera_info_t *camera_info)
{
    //inactive mclk before power off
    clk_disable(camera_info->module_clk);
    //power supply off
    camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
    if(camera_info->dvdd) {
        regulator_disable(camera_info->dvdd);
    }
    if(camera_info->avdd) {
        regulator_disable(camera_info->avdd);
    }
    if(camera_info->iovdd) {
        regulator_disable(camera_info->iovdd);
    }
    
    //standby and reset io
    mdelay(10);
    camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
    camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
    
}

static __s32 camera_detect_tvp5150(__u32 list_index, struct i2c_adapter *i2c_adap)
{
	int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect tvp5150 ... \n");
	
	regs.reg_num[0] = 0x80;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	if(regs.value[0] != 0x51)
		return -ENODEV;
	
	regs.reg_num[0] = 0x81;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	
	if(regs.value[0] != 0x50)
		return -ENODEV;

    camera_msg("detect tvp5150 success!!\n");
	
	return 0;        
}

#define __S5K4EC__

static void camera_pwr_on_s5k4ec(__u32 list_index, __camera_info_t *camera_info)
{
    __u32 CSI_AF_PWR_ON = 0;
    
    camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
	//power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	if(camera_info->iovdd) {
		regulator_enable(camera_info->iovdd);
	}
	if(camera_info->dvdd) {
		regulator_enable(camera_info->dvdd);
	}
	if(camera_info->avdd) {
		regulator_enable(camera_info->avdd);
	}
    
	camera_gpio_write(camera_info->af_pwr_pin, CSI_AF_PWR_ON);
	mdelay(10);
	//active mclk power on reset
	clk_enable(camera_info->module_clk);
	//power on reset
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_OFF);
	udelay(20);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	udelay(100);
}

static void camera_pwr_off_s5k4ec(__u32 list_index, __camera_info_t *camera_info)
{
    __u32 CSI_AF_PWR_OFF = 1;
    
    //reset on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	udelay(100);
	//inactive mclk before power off
	clk_disable(camera_info->module_clk);
	udelay(10);
	//standy on
	camera_gpio_write(camera_info->stby_pin, camera_list[list_index].CSI_STBY_ON);	
    
	//power supply off
	camera_gpio_write(camera_info->af_pwr_pin, CSI_AF_PWR_OFF);
    
	if(camera_info->dvdd) {
		regulator_disable(camera_info->dvdd);
	}
	if(camera_info->avdd) {
		regulator_disable(camera_info->avdd);
	}
	if(camera_info->iovdd) {
		regulator_disable(camera_info->iovdd);
	}
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input
}

static __s32 camera_detect_s5k4ec(__u32 list_index, struct i2c_adapter *i2c_adap)
{
    int ret;
	struct regval_list regs;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;
    
    camera_msg("try to detect s5k4ec ... \n");
    
    regs.reg_num[0] = 0x00;
	regs.reg_num[1] = 0x2c;
	regs.value[0] = 0x70;
    regs.value[1] = 0x00;
    ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

    regs.reg_num[0] = 0x00;
	regs.reg_num[1] = 0x2e;
	regs.value[0] = 0x01;
    regs.value[1] = 0xa4;
    ret = camera_i2c_write(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}

    regs.reg_num[0] = 0x0f;
	regs.reg_num[1] = 0x12;
    ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
    
    if ((regs.value[0] != 0x4e) || (regs.value[1] != 0xc0)) {
        return -ENODEV;    
    }
    
    camera_msg("detect s5k4ec success!!\n");
    
    return 0;
}

#define __OV5650_MV9335__

static void camera_pwr_on_ov5650_mv9335(__u32 list_index, __camera_info_t *camera_info)
{
    //power supply
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_ON);
	msleep(10);
	//power on reset
	camera_gpio_set_status(camera_info->stby_pin,1);//set the gpio to output
	camera_gpio_set_status(camera_info->reset_pin,1);//set the gpio to output
    //camera_gpio_write(camera_info->stby_pin,CSI_STBY_ON);
	//reset on io
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	msleep(10);
	//active mclk before power on
	clk_enable(camera_info->module_clk);
	msleep(10);
    //if(camera_info->dvdd) {
    //	regulator_enable(camera_info->dvdd);
    //	mdelay(10);
    //}
    //if(camera_info->avdd) {
    //	regulator_enable(camera_info->avdd);
    //	mdelay(10);
    //}
    //if(camera_info->iovdd) {
    //	regulator_enable(camera_info->iovdd);
    //	mdelay(10);
    //}
    ////standby off io
    //camera_gpio_write(camera_info->stby_pin,CSI_STBY_OFF);
    //mdelay(10);
	//reset after power on
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_OFF);
	msleep(10);
    //camera_gpio_write(camera_info->reset_pin,CSI_RST_ON);
    //mdelay(100);
    //camera_gpio_write(camera_info->reset_pin,CSI_RST_OFF);
    //mdelay(100);
}

static void camera_pwr_off_ov5650_mv9335(__u32 list_index, __camera_info_t *camera_info)
{
    //standby and reset io
    //camera_gpio_write(camera_info->stby_pin,CSI_STBY_ON);
    //mdelay(100);
	camera_gpio_write(camera_info->reset_pin, camera_list[list_index].CSI_RST_ON);
	msleep(10);
	//power supply off
    //if(camera_info->iovdd) {
    //	regulator_disable(camera_info->iovdd);
    //	mdelay(10);
    //}
    //if(camera_info->avdd) {
    //	regulator_disable(camera_info->avdd);
    //	mdelay(10);
    //}
    //if(camera_info->dvdd) {
    //	regulator_disable(camera_info->dvdd);
    //	mdelay(10);	
    //}
	//inactive mclk before power off
	clk_disable(camera_info->module_clk);
	msleep(10);
	camera_gpio_write(camera_info->pwr_pin, camera_list[list_index].CSI_PWR_OFF);
	//set the io to hi-z
	camera_gpio_set_status(camera_info->reset_pin,0);//set the gpio to input
	camera_gpio_set_status(camera_info->stby_pin,0);//set the gpio to input    
}

static __s32 camera_detect_ov5650_mv9335(__u32 list_index, struct i2c_adapter *i2c_adap)
{
	int ret;
	struct regval_list regs;
	char c0,c1;
    __u32 i2c_addr  = camera_list[list_index].i2c_addr;
    __u32 addr_step = camera_list[list_index].REG_ADDR_STEP;
    __u32 data_step = camera_list[list_index].REG_DATA_STEP;

    camera_msg("try to detect ov5650_mv9335 ... \n");
    
	regs.value[0] = 0xff;
	regs.reg_num[0] = 0x00;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	c0=regs.value[0];
	
	regs.reg_num[0] = 0x01;
	ret = camera_i2c_read(i2c_adap, regs.reg_num, regs.value, i2c_addr, addr_step, data_step);
	if (ret < 0) {
		return ret;
	}
	c1=regs.value[0];
	
	if(c0 != 0x93 || c1 != 0x35) {
		return -ENODEV;
	}
    
    camera_msg("detect ov5650_mv9335 success!!\n");
    
	return 0;    
}

__camera_list_t camera_list[MAX_CAMERA_LIST_ITEM] = {
    CAMERA_LIST_ITEM_INIT(ov7670,        1, 1, 0x42, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(gc0308,        1, 1, 0x42, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(gt2005,        2, 1, 0x78, 0, 1, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(hi704,         1, 1, 0x60, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(sp0838,        1, 1, 0x30, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(mt9m112,       1, 2, 0xba, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(mt9m113,       2, 2, 0x78, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(ov2655,        2, 1, 0x60, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(hi253,         1, 1, 0x40, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(gc0307,        1, 1, 0x42, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(mt9d112,       2, 2, 0x78, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(ov5640,        2, 1, 0x78, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(gc2015,        1, 1, 0x60, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(ov2643,        1, 1, 0x60, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(gc0329,        1, 1, 0x62, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(gc0309,        1, 1, 0x42, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(tvp5150,       1, 1, 0xb8, 1, 0, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(s5k4ec,        2, 2, 0x5a, 0, 1, 0, 1, 1, 0),
    CAMERA_LIST_ITEM_INIT(ov5650_mv9335, 1, 1, 0x50, 1, 0, 0, 1, 1, 0),
};

