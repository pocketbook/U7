#ifndef __CAMERA_DBG_H__
#define __CAMERA_DBG_H__

/*
0: no print
1: only inf && msg
2: inf && msg && err
*/

#define CAMERA_DBG_LEVEL 2

#if (CAMERA_DBG_LEVEL == 2)
#define camera_inf(x...) printk("[camera_print][INF][L%d]", __LINE__);printk(x)
#define camera_msg(x...) printk("[camera_print][MSG][L%d]", __LINE__);printk(x)
#define camera_err(x...) printk("[camera_print][ERR][L%d]", __LINE__);printk(x)
#elif (CAMERA_DBG_LEVEL == 1)
#define camera_inf(x...) printk("[camera_print][INF][L%d]", __LINE__);printk(x)
#define camera_msg(x...) printk("[camera_print][MSG][L%d]", __LINE__);printk(x)
#define camera_err(x...) 
#else
#define camera_inf(x...) 
#define camera_msg(x...) 
#define camera_err(x...) 
#endif

#endif
