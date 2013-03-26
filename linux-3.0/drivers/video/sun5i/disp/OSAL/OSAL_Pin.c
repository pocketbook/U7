/*
*************************************************************************************
*                         			eBsp
*					   Operation System Adapter Layer
*
*				(c) Copyright 2006-2010, All winners Co,Ld.
*							All	Rights Reserved
*
* File Name 	: OSAL_Pin.h
*
* Author 		: javen
*
* Description 	: C�⺯��
*
* History 		:
*      <author>    		<time>       	<version >    		<desc>
*       javen     	   2010-09-07          1.0         create this word
*       holi     	   2010-12-02          1.1         ��Ӿ���Ľӿڣ�
*************************************************************************************
*/
#include "OSAL_Pin.h"
#include "../../../../power/axp_power/axp-gpio.h"

__hdle OSAL_GPIO_Request(user_gpio_set_t *gpio_list, __u32 group_count_max)
{    
    __inf("OSAL_GPIO_Request, port:%d, port_num:%d, mul_sel:%d, pull:%d, drv_level:%d, data:%d\n", gpio_list->port, gpio_list->port_num, gpio_list->mul_sel, gpio_list->pull, gpio_list->drv_level, gpio_list->data);

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

__hdle OSAL_GPIO_Request_Ex(char *main_name, const char *sub_name)
{
    return gpio_request_ex(main_name, sub_name);
}

//if_release_to_default_status:
    //�����0����1����ʾ�ͷź��GPIO��������״̬������״״̬���ᵼ���ⲿ��ƽ�Ĵ���
    //�����2����ʾ�ͷź��GPIO״̬���䣬���ͷŵ�ʱ�򲻹���ǰGPIO��Ӳ���Ĵ�����
__s32 OSAL_GPIO_Release(__hdle p_handler, __s32 if_release_to_default_status)
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

__s32 OSAL_GPIO_DevGetAllPins_Status(unsigned p_handler, user_gpio_set_t *gpio_status, unsigned gpio_count_max, unsigned if_get_from_hardware)
{
    return gpio_get_all_pin_status(p_handler, gpio_status, gpio_count_max, if_get_from_hardware);
}

__s32 OSAL_GPIO_DevGetONEPins_Status(unsigned p_handler, user_gpio_set_t *gpio_status,const char *gpio_name,unsigned if_get_from_hardware)
{
    return gpio_get_one_pin_status(p_handler, gpio_status,gpio_name,if_get_from_hardware);
}

__s32 OSAL_GPIO_DevSetONEPin_Status(u32 p_handler, user_gpio_set_t *gpio_status, const char *gpio_name, __u32 if_set_to_current_input_status)
{
    return gpio_set_one_pin_status(p_handler, gpio_status, gpio_name, if_set_to_current_input_status);
}

__s32 OSAL_GPIO_DevSetONEPIN_IO_STATUS(u32 p_handler, __u32 if_set_to_output_status, const char *gpio_name)
{
    if(p_handler < 200 && p_handler >=100)
    {
        return axp_gpio_set_io(p_handler-100, if_set_to_output_status);
    }
    else
    {
        return gpio_set_one_pin_io_status(p_handler, if_set_to_output_status, gpio_name);
    }
}

__s32 OSAL_GPIO_DevSetONEPIN_PULL_STATUS(u32 p_handler, __u32 set_pull_status, const char *gpio_name)
{
    return gpio_set_one_pin_pull(p_handler, set_pull_status, gpio_name);
}

__s32 OSAL_GPIO_DevREAD_ONEPIN_DATA(u32 p_handler, const char *gpio_name)
{
    if(p_handler < 200 && p_handler >=100)
    {
        int value;
        
        axp_gpio_get_value(p_handler-100, &value);
        return value;
    }
    else
    {
        return gpio_read_one_pin_value(p_handler, gpio_name);
    }
}

__s32 OSAL_GPIO_DevWRITE_ONEPIN_DATA(u32 p_handler, __u32 value_to_gpio, const char *gpio_name)
{
    if((p_handler<200) && (p_handler>=100))
    {        
        return axp_gpio_set_value(p_handler-100, value_to_gpio);
    }
    else
    {
        return gpio_write_one_pin_value(p_handler, value_to_gpio, gpio_name);
    }
}


