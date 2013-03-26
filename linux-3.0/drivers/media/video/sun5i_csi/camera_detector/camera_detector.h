#ifndef __CAMERA_DETECTOR_H__
#define __CAMERA_DETECTOR_H__

#include "camera_includes.h"

//error number
#define ECFGNOEX            1   //config not exit
#define ECFGERR             2   //config error
#define ECAMNUMERR          3   //camera number error
#define ECLKSRC             4   //clock source error
#define EDETECTFAIL         5   //detect camera fail
#define EPMUPIN             6   //get pmu pin fail

typedef struct {
    __u32                   num;
    __camera_csi_connect_e  csi_connect_type;
    __camera_info_t         camera[MAX_CAMERA_NUM];
}__camera_detector_t;

#endif
