/*
*********************************************************************************************************
*                                                    LINUX-KERNEL
*                                        newbie Linux Platform Develop Kits
*                                                   Kernel Module
*
*                                    (c) Copyright 2006-2011, kevin.z China
*                                             All Rights Reserved
*
* File    : standby_i.h
* By      : kevin.z
* Version : v1.0
* Date    : 2011-5-30 17:21
* Descript:
* Update  : date                auther      ver     notes
*********************************************************************************************************
*/
#ifndef __STANDBY_I_H__
#define __STANDBY_I_H__

#include <linux/power/aw_pm.h>
#include <mach/platform.h>

#include "standby_cfg.h"
#include "common.h"
#include "standby_clock.h"
#include "standby_key.h"
#include "standby_power.h"
#include "standby_usb.h"
#include "standby_twi.h"
#include "standby_ir.h"
#include "standby_tmr.h"
#include "standby_int.h"
#include "../pm.h"

extern struct aw_pm_info  pm_info;


#endif  //__STANDBY_I_H__

