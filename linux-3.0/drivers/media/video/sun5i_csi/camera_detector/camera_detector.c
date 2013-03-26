#include "camera_list.h"
#include "camera_detector.h"

__camera_detector_t camera_detector[1];

__bool camera_sub_name_exist(char *main_name, char *sub_name);

static __s32 camera_request_clk(__u32 csi_index,
                                        struct clk **csi_module_clk, 
                                        struct clk **csi_clk_src, 
                                        __hdle *csi_pin_hd)
{
    char *csi_para[2] = {"csi0_para", "csi1_para"};
    char *csi[2] = {"csi0", "csi1"};
    __s32 ret = 0;
    
    *csi_pin_hd = gpio_request_ex(csi_para[csi_index], NULL);
    
    *csi_module_clk = clk_get(NULL, csi[csi_index]);
    if(*csi_module_clk == NULL) {
       	camera_err("get %s module clk error!\n", csi[csi_index]);
    	return -ECLKSRC;
    }
    
    *csi_clk_src = clk_get(NULL,"hosc");
    if (*csi_clk_src == NULL) {
        camera_err("get %s hosc source clk error!\n", csi[csi_index]);	
    	return -ECLKSRC;
    }
    
    ret = clk_set_parent(*csi_module_clk, *csi_clk_src);
	if (ret == -1) {
        camera_err(" csi set parent failed \n");
        return -ECLKSRC;
	}
    
    clk_put(*csi_clk_src);
	
	ret = clk_set_rate(*csi_module_clk, CSI_MCLK);
	if (ret == -1) {
		camera_err("set %s module clock error\n", csi[csi_index]);
		return -ECLKSRC;
	}
    
	ret = clk_enable(*csi_module_clk);
    if (ret == -1) {
        camera_err("enable module clock error\n");

        return -ECLKSRC;
    }
    
    return 0;
}

static void camera_release_clk(struct clk *csi_module_clk)
{
    clk_disable(csi_module_clk);
    clk_put(csi_module_clk);        
    csi_module_clk = NULL;    
}

static __s32 camera_mclk_open(__camera_detector_t *camera_detector)
{
    __u32 i, csi_cnt = 0;
    
    camera_inf("camera_mclk_open !!\n");
    
    if (camera_sub_name_exist("csi0_para", "csi_used")) {
        csi_cnt++;
    }
    
    if (camera_sub_name_exist("csi1_para", "csi_used")) {
        csi_cnt++;
    }
    
    for (i = 0; i < csi_cnt; i++) {
        camera_request_clk(i, &camera_detector->camera[i].module_clk, 
                                 &camera_detector->camera[i].clk_src, 
                                 &camera_detector->camera[i].clk_pin_hdle);
    }
    
    if ((csi_cnt == 1) && (camera_detector->num == 2)) {
        camera_detector->camera[1].module_clk = camera_detector->camera[0].module_clk;
    }
    
    return 0;
}

void camera_mclk_close(__camera_detector_t *camera_detector)
{
    camera_inf("camera_mclk_close !!\n");

    if (camera_detector->camera[0].module_clk != NULL) {
        camera_release_clk(camera_detector->camera[0].module_clk);
    }
    
    if (camera_detector->camera[1].module_clk != NULL) {
        camera_release_clk(camera_detector->camera[1].module_clk);
    }
}

__bool camera_sub_name_exist(char *main_name, char *sub_name)
{
    __s32 value;
    __s32 ret;
    
    ret = script_parser_fetch(main_name, sub_name, &value, 1);
    if ((ret >= 0) && (value == 1)) {
        return TRUE;
    }
    else {
        return FALSE;
    }
}

__s32 camera_get_sysconfig(char *main_name, char *sub_name, __s32 *buf, __u32 cnt)
{
    __s32 ret;
    
    ret = script_parser_fetch(main_name, sub_name, buf, cnt);
    if (ret < 0) {
        camera_err("[%s].%s not exist!!\n", main_name, sub_name);
    }
    
    return ret;
}

/*
use_b_para:
TRUE:  use para from [csi0_para] _b para
FALSE: NOT use para from [csi0_para] _b para
*/
static __s32 camera_get_para(__camera_detector_t *camera_detector,
                                    __u32 camera_index,
                                    char *main_name,
                                    __bool use_b_para)
{
    __u32 para_index;
    __u32 pin_struct_size;
    //char csi_drv_node[2][MAX_NODE_NAME_LEN] = {"/dev/video0", "/dev/video1"};  
    char csi_drv_node[2][MAX_NODE_NAME_LEN] = {"sun5i_csi0", "sun5i_csi1"};      
    
    pin_struct_size = sizeof(user_gpio_set_t)/sizeof(__s32);
    
    if (strcmp(main_name, "csi0_para") == 0) {
        para_index = 0;
    }
    else {
        para_index = 1;
    }
    
    memcpy(camera_detector->camera[camera_index].drv_node_name, csi_drv_node[para_index], MAX_NODE_NAME_LEN);
    
    if (use_b_para) {
        camera_get_sysconfig(main_name, "csi_twi_id_b",
                                &(camera_detector->camera[camera_index].i2c_id), 1);
        camera_get_sysconfig(main_name, "csi_facing_b",
                                (__s32 *)(&(camera_detector->camera[camera_index].facing)), 1);
        camera_get_sysconfig(main_name, "csi_reset_b",
                                (__s32 *)camera_detector->camera[camera_index].reset_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "csi_power_en_b",
                                (__s32 *)camera_detector->camera[camera_index].pwr_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "csi_stby_b",
                                (__s32 *)camera_detector->camera[camera_index].stby_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "csi_af_en_b",
                                (__s32 *)camera_detector->camera[camera_index].af_pwr_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "csi_iovdd_b",
                                (__s32 *)camera_detector->camera[camera_index].iovdd_str, 
                                MAX_VDD_STR_LEN);
        camera_get_sysconfig(main_name, "csi_avdd_b",
                                (__s32 *)camera_detector->camera[camera_index].avdd_str, 
                                MAX_VDD_STR_LEN);
        camera_get_sysconfig(main_name, "csi_dvdd_b",
                                (__s32 *)camera_detector->camera[camera_index].dvdd_str, 
                                MAX_VDD_STR_LEN);
    }
    else {
        camera_get_sysconfig(main_name, "csi_twi_id",
                                &(camera_detector->camera[camera_index].i2c_id), 1);
        camera_get_sysconfig(main_name, "csi_facing",
                                (__s32 *)(&(camera_detector->camera[camera_index].facing)), 1);
        camera_get_sysconfig(main_name, "csi_reset",
                                (__s32 *)camera_detector->camera[camera_index].reset_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "csi_power_en",
                                (__s32 *)camera_detector->camera[camera_index].pwr_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "csi_stby",
                                (__s32 *)camera_detector->camera[camera_index].stby_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "csi_af_en",
                                (__s32 *)camera_detector->camera[camera_index].af_pwr_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "csi_iovdd",
                                (__s32 *)camera_detector->camera[camera_index].iovdd_str, 
                                MAX_VDD_STR_LEN);
        camera_get_sysconfig(main_name, "csi_avdd",
                                (__s32 *)camera_detector->camera[camera_index].avdd_str, 
                                MAX_VDD_STR_LEN);
        camera_get_sysconfig(main_name, "csi_dvdd",
                                (__s32 *)camera_detector->camera[camera_index].dvdd_str, 
                                MAX_VDD_STR_LEN);
    }
    
    if (camera_detector->camera[camera_index].reset_pin->port != 0) {
        camera_detector->camera[camera_index].reset_pin_used = 1;
    }
    if (camera_detector->camera[camera_index].pwr_pin->port != 0) {
        camera_detector->camera[camera_index].pwr_pin_used = 1;
    }
    if (camera_detector->camera[camera_index].stby_pin->port != 0) {
        camera_detector->camera[camera_index].stby_pin_used = 1;
    }
    
    if (strcmp(camera_detector->camera[camera_index].iovdd_str, "")) {
        camera_detector->camera[camera_index].iovdd = 
                        regulator_get(NULL, camera_detector->camera[camera_index].iovdd_str);
        if (camera_detector->camera[camera_index].iovdd == NULL) {
            camera_err("get regulator csi_iovdd error!! \n");

            return -EPMUPIN;
       }
        
    }
    if (strcmp(camera_detector->camera[camera_index].avdd_str, "")) {
        camera_detector->camera[camera_index].avdd = 
                        regulator_get(NULL, camera_detector->camera[camera_index].avdd_str);
        if (camera_detector->camera[camera_index].avdd == NULL) {
            camera_err("get regulator csi_avdd error!! \n");

        return -EPMUPIN;
        }
    }
    if (strcmp(camera_detector->camera[camera_index].dvdd_str, "")) {
        camera_detector->camera[camera_index].dvdd = 
                        regulator_get(NULL, camera_detector->camera[camera_index].dvdd_str);
        if (camera_detector->camera[camera_index].dvdd == NULL) {
            camera_err("get regulator csi_dvdd error!! \n");

            return -EPMUPIN;
       }
    }
    
    return 0;
}

static __s32 camera_get_board_info(__camera_detector_t *camera_detector)
{
    char *csi[2] = {"csi0_para", "csi1_para"};
    __s32 value;
    __s32 ret;
    
    //get camera number
    if (camera_sub_name_exist(csi[0], "csi_used") 
        && camera_sub_name_exist(csi[1], "csi_used")) {
        camera_detector->num = 2;
    }
    else if (camera_sub_name_exist(csi[0], "csi_used")) {
        ret = camera_get_sysconfig(csi[0], "csi_dev_qty", &value, 1);
        if (ret < 0) {
            return -ECFGNOEX;
        }
        else {
            camera_detector->num = value;
            if ((camera_detector->num > 2) || (camera_detector->num <= 0)) {
                return -ECAMNUMERR;
            }
        }
    }
    else {
        return -ECFGERR;
    }
    
    camera_get_para(camera_detector, 0, csi[0], FALSE);
    if (camera_detector->num == 2) {
        if (camera_sub_name_exist(csi[1], "csi_used")) {
            camera_get_para(camera_detector, 1, csi[1], FALSE);
        }
        else {
            camera_get_para(camera_detector, 1, csi[0], TRUE);
        }
    }
    
    //get I2C adapter
    camera_detector->camera[0].i2c_adap = i2c_get_adapter(camera_detector->camera[0].i2c_id);
    if (camera_detector->num == 2) {
        if (camera_detector->camera[0].i2c_id != camera_detector->camera[1].i2c_id) {
            camera_detector->camera[1].i2c_adap = i2c_get_adapter(camera_detector->camera[1].i2c_id);
        }
        else {
            camera_detector->camera[1].i2c_adap = camera_detector->camera[0].i2c_adap;
        }
    }
    
    return 0;
}

static __s32 camera_init_module_list(__u32 camera_list_size)
{
    __u32 i;
    
    camera_inf("camera_list_size: %d \n", camera_list_size);
    
    for (i = 0; i < camera_list_size; i++) {
        if (camera_sub_name_exist("camera_list_para", camera_list[i].name)) {
            camera_list[i].need_detect = TRUE;
            camera_inf("modules: %s need detect!!\n", camera_list[i].name);
        }
    }

    return 0;
}

static __s32 camera_diff_i2c_id_detect(__camera_detector_t *camera_detector, 
                                                __camera_list_t *camera_list,
                                                __u32 camera_list_size)
{
    __u32 i, j;
    __u32 camera_detected = 0;
    __s32 ret = 0;
    
    camera_inf("camera_diff_i2c_id_detect!!\n");
    
    for (i = 0; i < camera_detector->num; i++) {
        for (j = 0; j < camera_list_size; j++) {
            if (camera_list[j].need_detect) {
                camera_list[j].pwr_on(j, &camera_detector->camera[i]);
                ret = camera_list[j].detect(j, camera_detector->camera[i].i2c_adap);
                if (ret == 0) {
                    camera_list[j].pwr_off(j, &camera_detector->camera[i]);
                    strcpy(camera_detector->camera[i].name, camera_list[j].name);
                    camera_detector->camera[i].i2c_addr = camera_list[j].i2c_addr;
                    camera_detected++;
                    break;
                }
            }
        }
    }    
    
    if (camera_detector->num != camera_detected) {
        camera_err("detect camera fail in func: camera_diff_i2c_id_detect !!\n");
        
        return -EDETECTFAIL;
    }
    
    return 0;
}

static __s32 camera_same_i2c_id_detect(__camera_detector_t *camera_detector, 
                                                  __camera_list_t *camera_list,
                                                  __u32 camera_list_size)
{
    __u32 i, j, reverse_i;
    __s32 ret = 0;
    __s32 camera_index[2][2] = {{-1, -1}, {-1, -1}};
    __u32 camera0_index, camera1_index;
    __u32 scan_cnt[2] = {0, 0};
    __u32 scan_index;
    
    camera_inf("camera_same_i2c_id_detect!!\n");
    
    for (i = 0; i < camera_detector->num; i++) {
        for (j = 0; j < camera_list_size; j++) {
            if (camera_list[j].need_detect) {
                camera_list[j].pwr_on(j, &camera_detector->camera[i]);
                ret = camera_list[j].detect(j, camera_detector->camera[i].i2c_adap);
                if (ret == 0) {
                    camera_index[i][scan_cnt[i]] = j;
                    scan_cnt[i]++;
                    camera_list[j].pwr_off(j, &camera_detector->camera[i]);
                }
            }
        }
    }
    
    if ((scan_cnt[0] == 2) || (scan_cnt[1] == 2)) {
        scan_index = (scan_cnt[0] == 2) ? 0 : 1;
        for (i = 0; i < 2; i++) {
            reverse_i = (i == 0) ? 1 : 0;
            camera0_index = camera_index[scan_index][i];
            camera1_index = camera_index[scan_index][reverse_i];
            camera_list[camera1_index].pwr_off(camera1_index, &camera_detector->camera[reverse_i]);
            camera_list[camera0_index].pwr_on(camera0_index, &camera_detector->camera[i]);
            ret = camera_list[camera0_index].detect(camera0_index, camera_detector->camera[i].i2c_adap);
            if (ret == 0) {
                strcpy(camera_detector->camera[i].name, camera_list[camera0_index].name);
                camera_detector->camera[i].i2c_addr = camera_list[camera0_index].i2c_addr;
                strcpy(camera_detector->camera[reverse_i].name, camera_list[camera1_index].name);
                camera_detector->camera[reverse_i].i2c_addr = camera_list[camera1_index].i2c_addr;
                camera_list[camera0_index].pwr_off(camera0_index, &camera_detector->camera[i]);
                if (i == 0) {
                    break;
                }
            }
        }
    }
    else if ((scan_cnt[0] == scan_cnt[1]) 
              && (scan_cnt[0] == 1)) {
        camera0_index = camera_index[0][0];
        camera1_index = camera_index[1][0];
        strcpy(camera_detector->camera[0].name, camera_list[camera0_index].name);
        strcpy(camera_detector->camera[1].name, camera_list[camera1_index].name);
        camera_detector->camera[0].i2c_addr = camera_list[camera0_index].i2c_addr;
        camera_detector->camera[1].i2c_addr = camera_list[camera1_index].i2c_addr;
    }
    else {
        camera_err("detect camera fail in func: camera_same_i2c_id_detect !!\n");
        
        return -EDETECTFAIL;
    }
    
    return 0;
}

static __u32 camera_get_facing_index(__camera_detector_t *camera_detector, 
                                               __camera_facing_e camera_facing)
{
    if (camera_detector->num == 1) {
        if (camera_facing == CAMERA_FACING_BACK) {
            return 0;
        }
        if (camera_facing == CAMERA_FACING_FRONT) {
            return 1;
        }
    }
    else {
        if (camera_detector->camera[0].facing == camera_facing) {
            return 0;
        }
        if (camera_detector->camera[1].facing == camera_facing) {
            return 1;
        }
    }
    
    return 0;
}

static ssize_t camera_num_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    __u32 camera_num;
    
    if ((camera_detector->num == 1) && strcmp(camera_detector->camera[0].name, "")) {
        camera_num = 1;
    }
    else if ((camera_detector->num == 2) 
            && strcmp(camera_detector->camera[0].name, "")
            && strcmp(camera_detector->camera[1].name, "")) {
        camera_num = 2;
    }
    else {
        camera_num = 0;
    }
    
    return sprintf(buf, "%u", camera_num);
}

static ssize_t camera_num_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
    return 0;
}

static ssize_t camera_front_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    __u32 index = camera_get_facing_index(camera_detector, CAMERA_FACING_FRONT);
    
    return sprintf(buf, camera_detector->camera[index].name);
}

static ssize_t camera_front_name_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
    return 0;
}

static ssize_t camera_back_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{   
    __u32 index = camera_get_facing_index(camera_detector, CAMERA_FACING_BACK);
    
    return sprintf(buf, camera_detector->camera[index].name);
}

static ssize_t camera_back_name_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
    return 0;
}

static ssize_t camera_front_node_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{   
    __u32 index = camera_get_facing_index(camera_detector, CAMERA_FACING_FRONT);
    
    if (strcmp(camera_detector->camera[index].name, "")) {
        return sprintf(buf, camera_detector->camera[index].drv_node_name);
    }
    else {
        return sprintf(buf, "");
    }
}

static ssize_t camera_front_node_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
    return 0;
}

static ssize_t camera_back_node_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{   
    __u32 index = camera_get_facing_index(camera_detector, CAMERA_FACING_BACK);
    
    if (strcmp(camera_detector->camera[index].name, "")) {
        return sprintf(buf, camera_detector->camera[index].drv_node_name);
    }
    else {
        return sprintf(buf, "");
    }
}

static ssize_t camera_back_node_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
    return 0;
}

static DEVICE_ATTR(num, S_IRUGO|S_IWUSR|S_IWGRP, camera_num_show, camera_num_store);
static DEVICE_ATTR(front_name, S_IRUGO|S_IWUSR|S_IWGRP, camera_front_name_show, camera_front_name_store);
static DEVICE_ATTR(back_name, S_IRUGO|S_IWUSR|S_IWGRP, camera_back_name_show, camera_back_name_store);
static DEVICE_ATTR(front_node, S_IRUGO|S_IWUSR|S_IWGRP, camera_front_node_show, camera_front_node_store);
static DEVICE_ATTR(back_node, S_IRUGO|S_IWUSR|S_IWGRP, camera_back_node_show, camera_back_node_store);

static struct attribute *camera_detector_attributes[] = {
	&dev_attr_num.attr,
    &dev_attr_front_name.attr,
    &dev_attr_back_name.attr,
    &dev_attr_front_node.attr,
    &dev_attr_back_node.attr,
	NULL
};

static struct attribute_group dev_attr_group = {
	.attrs = camera_detector_attributes,
};

static const struct attribute_group *dev_attr_groups[] = {
	&dev_attr_group,
	NULL,
};

static void camera_detector_release(struct device *dev)
{
    
}

static struct device camera_detector_device = {
        .init_name = "camera",
        .release = camera_detector_release,
};

static int __init camera_detector_init(void) {
	int err = 0;
    __u32 camera_list_size;
    
	camera_inf("camera detect driver init\n");
    
    camera_detector_device.groups = dev_attr_groups;
	err = device_register(&camera_detector_device);
	if(err) {
		camera_err("%s register camera detect driver as misc device error\n", __FUNCTION__);
		goto exit;
	}
    
    memset(camera_detector, 0, sizeof(__camera_detector_t));
    err = camera_get_board_info(camera_detector);
    if (err)
    {
        camera_err("camera_get_board_info fail !!\n");
        goto exit;
    }

    camera_list_size = sizeof(camera_list) / sizeof(__camera_list_t);
    camera_init_module_list(camera_list_size);
    
    camera_mclk_open(camera_detector);
    
    if ((camera_detector->camera[0].i2c_id == camera_detector->camera[1].i2c_id)
        && (camera_detector->num == 2)) {
        camera_same_i2c_id_detect(camera_detector, camera_list, camera_list_size);
    }
    else if ((camera_detector->camera[0].i2c_id != camera_detector->camera[1].i2c_id)
        || (camera_detector->num == 1)) {
        camera_diff_i2c_id_detect(camera_detector, camera_list, camera_list_size);
    }
    
    camera_mclk_close(camera_detector);
    
    return 0;
    
exit:
	return err;
}

static void __exit camera_detector_exit(void) {

	camera_inf("Bye, camera detect driver exit\n");
	device_unregister(&camera_detector_device);
}

void camera_export_info(char *module_name, int *i2c_addr, int index)
{
    strcpy(module_name, camera_detector->camera[index].name);
    *i2c_addr = camera_detector->camera[index].i2c_addr;
}

EXPORT_SYMBOL(camera_export_info);

late_initcall(camera_detector_init);
module_exit(camera_detector_exit);

MODULE_DESCRIPTION("camera detect driver");
MODULE_AUTHOR("heyihang");
MODULE_LICENSE("GPL");

