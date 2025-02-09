/* drivers/input/touchscreen/gt9xx.c
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 2.0
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2013/04/25
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          3. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. send config after resume
 *                  By Meta, 2013/08/06
 */

#include <linux/irq.h>
#include "gt9xx.h"

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif
#if GTP_PROXIMITY
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>

#define PROXIMITY_DEVICE   "gtp_proximity"

/******************confirm the parameter that hal use**********************/

/*
#define GTP_IOCTL_MAGIC        	0X5D
#define GTP_IOCTL_PROX_ON		_IO(GTP_IOCTL_MAGIC, 7)
#define GTP_IOCTL_PROX_OFF		_IO(GTP_IOCTL_MAGIC, 8)
*/


#define GTP_IOCTL_MAGIC        	0X5D
#define GTP_IOCTL_PROX_ON		_IO(GTP_IOCTL_MAGIC, 7)
#define GTP_IOCTL_PROX_OFF		_IO(GTP_IOCTL_MAGIC, 8)

/****************************************/

static int misc_opened = 0;
static int gtp_proximity_start = 0;	/* 0 is stop, 1 is start */
static int suspend_entry_flag = 0;

#endif


#define  TPD_DEBUG_ON  0
///add  by jinq start for adb  open debug
extern int zyt_adb_debug;
#define ZYT_PRINTK(fmt,arg...)  do{\
								 if(zyt_adb_debug||TPD_DEBUG_ON)printk("<<ZYT_PRINTK>> [%d]"fmt"\n",__LINE__, ##arg);\
								}while(0)
 ///add by jinq end


#if  defined(ZCFG_MK_TP_GESTURE)
#define GTP_GESTURE_WAKEUP  
#endif




#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#endif
extern int tp_device_id(int id);
extern void ctp_lock_mutex(void);
extern void ctp_unlock_mutex(void);

static const char *goodix_ts_name = GTP_I2C_NAME;
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL;
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
= {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
static const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#if GTP_DEBUG_ON
static const int  key_codes[] = {KEY_HOME, KEY_BACK, KEY_APPSELECT, KEY_SEARCH};
static const char *key_names[] = {"Key_Home", "Key_Back", "Key_Menu", "Key_Search"};
#endif
#if TOUCH_VIRTUAL_KEYS
static struct kobject *properties_kobj;
#endif    
#endif

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(s32 ms);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);
#endif

//*********** For GT9XXF Start **********//
#if GTP_COMPATIBLE_MODE
extern s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 gup_clk_calibration(void);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
extern u8 gup_check_fs_mounted(char *path_name);

void gtp_recovery_reset(struct i2c_client *client);
static s32 gtp_esd_recovery(struct i2c_client *client);
s32 gtp_fw_startup(struct i2c_client *client);
static s32 gtp_main_clk_proc(struct goodix_ts_data *ts);
static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode);
#endif
//********** For GT9XXF End **********//

#ifdef GTP_GESTURE_WAKEUP

#include <linux/fs.h>

#define GESTURE_FUNCTION_SWITCH 	"/data/data/com.zyt.close_gesture_sttings/gesture_switch"
static int s_gesture_switch = 1;	// Defaultly, the macro is open


typedef enum {
        DOZE_DISABLED = 0,
        DOZE_ENABLED = 1,
        DOZE_WAKEUP = 2,
} DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);


#define GESTURE_LEFT		0xBB
#define GESTURE_RIGHT		0xAA
#define GESTURE_UP			0xBA
#define GESTURE_DOWN		0xAB
#define GESTURE_DOUBLECLICK 0xCC
#define GESTURE_O			'o'
#define GESTURE_W			'w'
#define GESTURE_M			'm'
#define GESTURE_E			'e'
#define GESTURE_C			'c'
#define GESTURE_S			's'
#define GESTURE_V			'v'
#define GESTURE_Z			'z'



#endif

static u8 chip_gt9xxs = 0;  // true if ic is gt9xxs, like gt915s
u8 grp_cfg_version = 0;

/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer:
      2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
        struct i2c_msg msgs[2];
        s32 ret=-1;
        s32 retries = 0;

        GTP_DEBUG_FUNC();

        msgs[0].flags = !I2C_M_RD;
        msgs[0].addr  = client->addr;
        msgs[0].len   = GTP_ADDR_LENGTH;
        msgs[0].buf   = &buf[0];
        //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

        msgs[1].flags = I2C_M_RD;
        msgs[1].addr  = client->addr;
        msgs[1].len   = len - GTP_ADDR_LENGTH;
        msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
        //msgs[1].scl_rate = 300 * 1000;

        while(retries < 5) {
                ret = i2c_transfer(client->adapter, msgs, 2);
                if(ret == 2)break;
                retries++;
        }
        if((retries >= 5)) {
#if GTP_COMPATIBLE_MODE
                struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

#ifdef  GTP_GESTURE_WAKEUP
                // reset chip would quit doze mode
                if (DOZE_ENABLED == doze_status) {
                        return ret;
                }

#endif
                GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
#if GTP_COMPATIBLE_MODE
                if (CHIP_TYPE_GT9F == ts->chip_type) {
                        gtp_recovery_reset(client);
                } else
#endif
                {
                        gtp_reset_guitar(client, 10);
                }
        }
        return ret;
}



/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer:
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
        struct i2c_msg msg;
        s32 ret = -1;
        s32 retries = 0;

        GTP_DEBUG_FUNC();

        msg.flags = !I2C_M_RD;
        msg.addr  = client->addr;
        msg.len   = len;
        msg.buf   = buf;
        //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

        while(retries < 5) {
                ret = i2c_transfer(client->adapter, &msg, 1);
                if (ret == 1)break;
                retries++;
        }
        if((retries >= 5)) {
#if GTP_COMPATIBLE_MODE
                struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

#ifdef GTP_GESTURE_WAKEUP
                if (DOZE_ENABLED == doze_status) {
                        return ret;
                }
#endif
                GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
#if GTP_COMPATIBLE_MODE
                if (CHIP_TYPE_GT9F == ts->chip_type) {
                        gtp_recovery_reset(client);
                } else
#endif
                {
                        gtp_reset_guitar(client, 10);
                }
        }
        return ret;
}

#if TOUCH_VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
#if defined(ZCFG_TP_GT9XX_KEY_MAP)
		u32 key_map_data[12]=ZCFG_TP_GT9XX_KEY_MAP;
		return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
			,__stringify(EV_KEY), __stringify(KEY_APPSELECT),key_map_data[0],key_map_data[1],key_map_data[2],key_map_data[3]
			,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),key_map_data[4],key_map_data[5],key_map_data[6],key_map_data[7]
			,__stringify(EV_KEY), __stringify(KEY_BACK),key_map_data[8],key_map_data[9],key_map_data[10],key_map_data[11]);
#else
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_APPSELECT) ":40:1380:50:30"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":120:1380:50:30"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":200:1380:50:30"
		":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":280:1380:50:30"
		"\n");
#endif
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.goodix_ts",//goodix_ts
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void virtual_keys_init(void)
{
	int ret;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,&properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");    
}

static void virtual_keys_destroy(void)
{
	kobject_del(properties_kobj);
}
#endif

#if GTP_PROXIMITY
static int gtp_proximity_open(void)
{
	u8 rbuffer[3] = {0x80, 0x42, 0x01};

	ZYT_PRINTK("%s\n", __func__);
	if (misc_opened)
		return -EBUSY;
	misc_opened = 1;
	gtp_proximity_start = 1;

	return gtp_i2c_write(i2c_connect_client, rbuffer, 3);
}

static int gtp_proximity_release(void)
{
	u8 rbuffer[3] = {0x80, 0x42, 0x00};

	ZYT_PRINTK("%s\n", __func__);
	misc_opened = 0;
	gtp_proximity_start = 0;

	return gtp_i2c_write(i2c_connect_client, rbuffer, 3);
}
//static int tp_ps_ioctl(struct i2c_client *client, unsigned int cmd, void *arg);

static long gtp_proximity_ioctl(struct i2c_client *client, unsigned int cmd, void *arg)
{

	ZYT_PRINTK("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case GTP_IOCTL_PROX_ON:
		gtp_proximity_open();
		break;
	case GTP_IOCTL_PROX_OFF:
		gtp_proximity_release();
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		return -EINVAL;
	}
	return 0;
}
/*
static struct file_operations tp_ps_fops = {
	.owner		= THIS_MODULE,
	.open		= NULL,
	.release	= NULL,
	.unlocked_ioctl		= tp_ps_ioctl,
};

static struct miscdevice tp_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TP_PS_DEVICE,
	.fops = &tp_ps_fops,
};
*/
static struct file_operations gtp_proximity_fops = {
	.owner = THIS_MODULE,
	.open = NULL,//gtp_proximity_open,
	.release = NULL,//gtp_proximity_release,
	.unlocked_ioctl = gtp_proximity_ioctl
};

struct miscdevice gtp_proximity_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = PROXIMITY_DEVICE,						//match the hal's name 
	.fops = &gtp_proximity_fops
};
#endif
/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
        u8 buf[16] = {0};
        u8 confirm_buf[16] = {0};
        u8 retry = 0;

        while (retry++ < 3) {
                memset(buf, 0xAA, 16);
                buf[0] = (u8)(addr >> 8);
                buf[1] = (u8)(addr & 0xFF);
                gtp_i2c_read(client, buf, len + 2);

                memset(confirm_buf, 0xAB, 16);
                confirm_buf[0] = (u8)(addr >> 8);
                confirm_buf[1] = (u8)(addr & 0xFF);
                gtp_i2c_read(client, confirm_buf, len + 2);

                if (!memcmp(buf, confirm_buf, len+2)) {
                        memcpy(rxbuf, confirm_buf+2, len);
                        return SUCCESS;
                }
        }
        GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
        return FAIL;
}

/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation.
        1: succeed, otherwise: failed
*********************************************************/

s32 gtp_send_cfg(struct i2c_client *client)
{
        s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
        s32 retry = 0;
        struct goodix_ts_data *ts = i2c_get_clientdata(client);

        if (ts->fixed_cfg) {
                GTP_INFO("Ic fixed config, no config sent!");
                return 0;
        } else if (ts->pnl_init_error) {
                GTP_INFO("Error occured in init_panel, no config sent");
                return 0;
        }

        GTP_INFO("Driver send config.");
        for (retry = 0; retry < 5; retry++) {
                ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
                if (ret > 0) {
                        break;
                }
        }
#endif
        return ret;
}
/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
        unsigned long irqflags;

        GTP_DEBUG_FUNC();

        spin_lock_irqsave(&ts->irq_lock, irqflags);
        if (!ts->irq_is_disable) {
                ts->irq_is_disable = 1;
                disable_irq_nosync(ts->client->irq);
        }
        spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
        unsigned long irqflags = 0;

        GTP_DEBUG_FUNC();

        spin_lock_irqsave(&ts->irq_lock, irqflags);
        if (ts->irq_is_disable) {
                enable_irq(ts->client->irq);
                ts->irq_is_disable = 0;
        }
        spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*******************************************************
Function:
    Report touch point event
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
        GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
        input_mt_slot(ts->input_dev, id);
        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
        input_mt_sync(ts->input_dev);
#endif

        ZYT_PRINTK("gtp_touch_down :%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
        input_mt_slot(ts->input_dev, id);
        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
        GTP_DEBUG("Touch id[%2d] release!", id);
#else
       //input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
       //input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
        input_mt_sync(ts->input_dev);
#endif
		ZYT_PRINTK("gtp_touch_up Touch id[%2d]", id);
}

#ifdef GTP_GESTURE_WAKEUP
////=================Customer ges code Start===============================////
#define GESTURE_FUNCTION_KEY_C_NEXT 	"/data/data/com.zyt.close_gesture_sttings/c_next"
#define GESTURE_FUNCTION_KEY_C_PRE 	"/data/data/com.zyt.close_gesture_sttings/c_pre"
#define GESTURE_FUNCTION_KEY_C_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/c_play_pause"

#define GESTURE_FUNCTION_KEY_W_NEXT 	"/data/data/com.zyt.close_gesture_sttings/w_next"
#define GESTURE_FUNCTION_KEY_W_PRE 	"/data/data/com.zyt.close_gesture_sttings/w_pre"
#define GESTURE_FUNCTION_KEY_W_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/w_play_pause"

#define GESTURE_FUNCTION_KEY_V_NEXT 	"/data/data/com.zyt.close_gesture_sttings/v_next"
#define GESTURE_FUNCTION_KEY_V_PRE 	"/data/data/com.zyt.close_gesture_sttings/v_pre"
#define GESTURE_FUNCTION_KEY_V_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/v_play_pause"

#define GESTURE_FUNCTION_KEY_M_NEXT 	"/data/data/com.zyt.close_gesture_sttings/m_next"
#define GESTURE_FUNCTION_KEY_M_PRE 	"/data/data/com.zyt.close_gesture_sttings/m_pre"
#define GESTURE_FUNCTION_KEY_M_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/m_play_pause"

#define GESTURE_FUNCTION_KEY_S_NEXT 	"/data/data/com.zyt.close_gesture_sttings/s_next"
#define GESTURE_FUNCTION_KEY_S_PRE 	"/data/data/com.zyt.close_gesture_sttings/s_pre"
#define GESTURE_FUNCTION_KEY_S_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/s_play_pause"

#define GESTURE_FUNCTION_KEY_Z_NEXT 	"/data/data/com.zyt.close_gesture_sttings/z_next"
#define GESTURE_FUNCTION_KEY_Z_PRE 	"/data/data/com.zyt.close_gesture_sttings/z_pre"
#define GESTURE_FUNCTION_KEY_Z_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/z_play_pause"

#define GESTURE_FUNCTION_KEY_O_NEXT 	"/data/data/com.zyt.close_gesture_sttings/o_next"
#define GESTURE_FUNCTION_KEY_O_PRE 	"/data/data/com.zyt.close_gesture_sttings/o_pre"
#define GESTURE_FUNCTION_KEY_O_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/o_play_pause"

#define GESTURE_FUNCTION_KEY_E_NEXT 	"/data/data/com.zyt.close_gesture_sttings/e_next"
#define GESTURE_FUNCTION_KEY_E_PRE 	"/data/data/com.zyt.close_gesture_sttings/e_pre"
#define GESTURE_FUNCTION_KEY_E_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/e_play_pause"

#define GESTURE_FUNCTION_KEY_RIGHT_NEXT 	"/data/data/com.zyt.close_gesture_sttings/right_next"
#define GESTURE_FUNCTION_KEY_RIGHT_PRE 	"/data/data/com.zyt.close_gesture_sttings/right_pre"
#define GESTURE_FUNCTION_KEY_RIGHT_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/right_play_pause"

#define GESTURE_FUNCTION_KEY_DOWN_NEXT 	"/data/data/com.zyt.close_gesture_sttings/down_next"
#define GESTURE_FUNCTION_KEY_DOWN_PRE 	"/data/data/com.zyt.close_gesture_sttings/down_pre"
#define GESTURE_FUNCTION_KEY_DOWN_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/down_play_pause"

#define GESTURE_FUNCTION_KEY_LEFT_NEXT 	"/data/data/com.zyt.close_gesture_sttings/left_next"
#define GESTURE_FUNCTION_KEY_LEFT_PRE 	"/data/data/com.zyt.close_gesture_sttings/left_pre"
#define GESTURE_FUNCTION_KEY_LEFT_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/left_play_pause"

#define GESTURE_FUNCTION_KEY_UP_NEXT 	"/data/data/com.zyt.close_gesture_sttings/up_next"
#define GESTURE_FUNCTION_KEY_UP_PRE 	"/data/data/com.zyt.close_gesture_sttings/up_pre"
#define GESTURE_FUNCTION_KEY_UP_PLAY_PAUSE	"/data/data/com.zyt.close_gesture_sttings/up_play_pause"

int Ges_trans_key(struct goodix_ts_data *info,unsigned int code)
{
	struct file *fp; 

	switch (code)
	{
		case KEY_C:
			fp = filp_open(GESTURE_FUNCTION_KEY_C_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_C_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_C_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_C_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_C_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_C_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_C_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_C_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_C_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}				
			break;
		case KEY_W:
			fp = filp_open(GESTURE_FUNCTION_KEY_W_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_W_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_W_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_W_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_W_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_W_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_W_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_W_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_W_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_V:
			fp = filp_open(GESTURE_FUNCTION_KEY_V_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_V_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_V_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_V_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_V_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_V_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_V_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_V_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_V_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_M:
			fp = filp_open(GESTURE_FUNCTION_KEY_M_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_M_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_M_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_M_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_M_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_M_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_M_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_M_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_M_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_S:
			fp = filp_open(GESTURE_FUNCTION_KEY_S_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_S_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_S_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_S_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_S_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_S_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_S_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_S_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_S_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_Z:
			fp = filp_open(GESTURE_FUNCTION_KEY_Z_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_Z_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_Z_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_Z_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_Z_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_Z_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_Z_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_Z_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_Z_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_O:
			fp = filp_open(GESTURE_FUNCTION_KEY_O_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_O_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_O_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_O_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_O_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_O_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_O_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_O_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_O_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_E:
			fp = filp_open(GESTURE_FUNCTION_KEY_E_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_E_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_E_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_E_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_E_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_E_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_E_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_E_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_E_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_RIGHT:
			fp = filp_open(GESTURE_FUNCTION_KEY_RIGHT_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_RIGHT_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_RIGHT_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_RIGHT_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_RIGHT_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_RIGHT_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_RIGHT_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_RIGHT_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_RIGHT_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_DOWN:
			fp = filp_open(GESTURE_FUNCTION_KEY_DOWN_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_DOWN_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_DOWN_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_DOWN_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_DOWN_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_DOWN_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_DOWN_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_DOWN_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_DOWN_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_LEFT:
			fp = filp_open(GESTURE_FUNCTION_KEY_LEFT_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_LEFT_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_LEFT_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			
			fp = filp_open(GESTURE_FUNCTION_KEY_LEFT_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_LEFT_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_LEFT_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_LEFT_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_LEFT_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_LEFT_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
		case KEY_UP:
			fp = filp_open(GESTURE_FUNCTION_KEY_UP_NEXT, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_UP_NEXT);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_UP_NEXT); 
				filp_close(fp, NULL);
				code = KEY_NEXTSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_UP_PRE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_UP_PRE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_UP_PRE); 
				filp_close(fp, NULL);
				code = KEY_PREVIOUSSONG;
				break;
			}
			fp = filp_open(GESTURE_FUNCTION_KEY_UP_PLAY_PAUSE, O_RDONLY , 0);
			if (IS_ERR(fp)) 
			{ 
				pr_info("open file %s error!\n", GESTURE_FUNCTION_KEY_UP_PLAY_PAUSE);
			} 
			else 
			{
				printk("open file %s success!\n", GESTURE_FUNCTION_KEY_UP_PLAY_PAUSE); 
				filp_close(fp, NULL);
				code = KEY_PLAYPAUSE;
				break;
			}	
			break;
	}
	input_report_key(info->input_dev, code, 1);
	input_sync(info->input_dev);
	input_report_key(info->input_dev, code, 0);
	input_sync(info->input_dev);
}
////=================Customer ges code End===============================////

static void check_gesture(struct goodix_ts_data *ts,int gesture_id)
{

	printk("GT9XX gesture_id==0x%x\n ",gesture_id);
	switch(gesture_id)
	{
		case GESTURE_LEFT:		
			Ges_trans_key(ts, KEY_LEFT);
			break;
		case GESTURE_RIGHT:
			Ges_trans_key(ts, KEY_RIGHT);
			break;
		case GESTURE_UP:
			Ges_trans_key(ts, KEY_UP);
			break;
		case GESTURE_DOWN:
			Ges_trans_key(ts, KEY_DOWN);
			break;
		case GESTURE_DOUBLECLICK:
			Ges_trans_key(ts, KEY_U);
			break;
		case GESTURE_O:
			Ges_trans_key(ts, KEY_O);
			break;
		case GESTURE_W:
			Ges_trans_key(ts, KEY_W);
			break;
		case GESTURE_M:
			Ges_trans_key(ts, KEY_M);
			break;
		case GESTURE_E:
			Ges_trans_key(ts, KEY_E);
			break;
		case GESTURE_C:
			Ges_trans_key(ts, KEY_C);
			break;		
		case GESTURE_S:
			Ges_trans_key(ts, KEY_S);
		break;
		case GESTURE_V:
			Ges_trans_key(ts, KEY_V);
		break;
		case GESTURE_Z:
			Ges_trans_key(ts, KEY_Z);
		break;		
		default:
			break;
	}
}
#endif

/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
        u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
        u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]= {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
        u8  touch_num = 0;
        u8  finger = 0;
        static u16 pre_touch = 0;
        static u8 pre_key = 0;
#if GTP_WITH_PEN
        static u8 pre_pen = 0;
#endif
        u8  key_value = 0;
        u8* coor_data = NULL;
        s32 input_x = 0;
        s32 input_y = 0;
        s32 input_w = 0;
        s32 id = 0;
        s32 i  = 0;
        s32 ret = -1;
        struct goodix_ts_data *ts = NULL;

#if GTP_COMPATIBLE_MODE
        u8 rqst_buf[3] = {0x80, 0x43};  // for GT9XXF
#endif

#ifdef GTP_GESTURE_WAKEUP
        u8 doze_buf[3] = {0x81, 0x4B};
#endif

        GTP_DEBUG_FUNC();
		ZYT_PRINTK("===========goodix_ts_work_func==========\n");
        ts = container_of(work, struct goodix_ts_data, work);
        if (ts->enter_update) {
                return;
        }
#ifdef GTP_GESTURE_WAKEUP
    if (DOZE_ENABLED == doze_status)
    {               
        ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
        GTP_INFO("0x814B = 0x%02X", doze_buf[2]);
        if (ret > 0)
        {     
            if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') || (doze_buf[2] == 'c') ||
                (doze_buf[2] == 'd') || (doze_buf[2] == 'e') || (doze_buf[2] == 'g') || 
                (doze_buf[2] == 'h') || (doze_buf[2] == 'm') || (doze_buf[2] == 'o') ||
                (doze_buf[2] == 'q') || (doze_buf[2] == 's') || (doze_buf[2] == 'v') || 
                (doze_buf[2] == 'w') || (doze_buf[2] == 'y') || (doze_buf[2] == 'z') ||
                (doze_buf[2] == 0x5E) || (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
                (doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) || (doze_buf[2] == 0xCC)/* ^ */
                )
           	 {
				check_gesture(ts,doze_buf[2]);
                doze_status = DOZE_WAKEUP;
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
				gtp_enter_doze(ts);
            }
			     else
            {
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
                gtp_enter_doze(ts);
            }
				#if 0
                if (doze_buf[2] != 0x5E)
                {
                    GTP_INFO("Wakeup by gesture(%c), light up the screen!", doze_buf[2]);
                }
                else
                {
                    GTP_INFO("Wakeup by gesture(^), light up the screen!");
                }
		
      
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
			
                          doze_status = DOZE_WAKEUP;
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
			}
			else if ( (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
				(doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) )
            {
                char *direction[4] = {"Right", "Down", "Up", "Left"};
                u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;
                
                GTP_INFO("%s slide to light up the screen!", direction[type]);
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
            }
            else if (0xCC == doze_buf[2])
            {
                GTP_INFO("Double click to light up the screen!");
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
            }
				#endif
       
        }
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }
#endif

        ret = gtp_i2c_read(ts->client, point_data, 12);
        if (ret < 0) {
                GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
                goto exit_work_func;
        }

        finger = point_data[GTP_ADDR_LENGTH];

#if GTP_COMPATIBLE_MODE
        // GT9XXF
        if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type)) {   // request arrived
                ret = gtp_i2c_read(ts->client, rqst_buf, 3);
                if (ret < 0) {
                        GTP_ERROR("Read request status error!");
                        goto exit_work_func;
                }

                switch (rqst_buf[2] & 0x0F) {
                case GTP_RQST_CONFIG:
                        GTP_INFO("Request for config.");
                        ret = gtp_send_cfg(ts->client);
                        if (ret < 0) {
                                GTP_ERROR("Request for config unresponded!");
                        } else {
                                rqst_buf[2] = GTP_RQST_RESPONDED;
                                gtp_i2c_write(ts->client, rqst_buf, 3);
                                GTP_INFO("Request for config responded!");
                        }
                        break;

                case GTP_RQST_BAK_REF:
                        GTP_INFO("Request for backup reference.");
                        ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_SEND);
                        if (SUCCESS == ret) {
                                rqst_buf[2] = GTP_RQST_RESPONDED;
                                gtp_i2c_write(ts->client, rqst_buf, 3);
                                GTP_INFO("Request for backup reference responded!");
                        } else {
                                GTP_ERROR("Requeset for backup reference unresponed!");
                        }
                        break;

                case GTP_RQST_RESET:
                        GTP_INFO("Request for reset.");
                        gtp_recovery_reset(ts->client);
                        break;

                case GTP_RQST_MAIN_CLOCK:
                        GTP_INFO("Request for main clock.");
                        ts->rqst_processing = 1;
                        ret = gtp_main_clk_proc(ts);
                        if (FAIL == ret) {
                                GTP_ERROR("Request for main clock unresponded!");
                        } else {
                                GTP_INFO("Request for main clock responded!");
                                rqst_buf[2] = GTP_RQST_RESPONDED;
                                gtp_i2c_write(ts->client, rqst_buf, 3);
                                ts->rqst_processing = 0;
                                ts->clk_chk_fs_times = 0;
                        }
                        break;

                case GTP_RQST_IDLE:
                default:
                        break;
                }
        }
#endif

        if((finger & 0x80) == 0) {
                goto exit_work_func;
        }
#if GTP_PROXIMITY
    if (gtp_proximity_start == 1)
    {
		if (point_data[GTP_ADDR_LENGTH] & 0x20)//if (point_data[GTP_ADDR_LENGTH] & 0x60)
		{
			input_report_abs(ts->input_dev, ABS_DISTANCE, 0);
			ZYT_PRINTK("proximity_data-ABS_DISTANCE-==00000000000000000\n");
			input_sync(ts->input_dev);
            goto exit_work_func;
		}
		else
		{
			input_report_abs(ts->input_dev, ABS_DISTANCE, 1);
			ZYT_PRINTK("proximity_data-ABS_DISTANCE-==11111111111111111\n");

		}
   		ZYT_PRINTK("proximity_data[2] = 0x%x\n", point_data[GTP_ADDR_LENGTH]);
   }   
#endif	
        touch_num = finger & 0x0f;
        if (touch_num > GTP_MAX_TOUCH) {
                goto exit_work_func;
        }

        if (touch_num > 1) {
                u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

                ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
                memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
        }

#if GTP_HAVE_TOUCH_KEY
        key_value = point_data[3 + 8 * touch_num];

        if(key_value || pre_key) {
	#if TOUCH_VIRTUAL_KEYS
		if ((1==key_value) || (2==key_value) || (4==key_value) || (8==key_value))
		{
			int index_value=0;
			int	x=40;

			if (1==key_value)
				index_value = 0;
			else if (2==key_value)
				index_value = 1;
			else if (4==key_value)
				index_value = 2;
			else if (8==key_value)
				index_value = 3;

			if (KEY_APPSELECT == touch_key_array[index_value])
			{
				x = 40;
			}
			else if (KEY_HOMEPAGE == touch_key_array[index_value])
			{
				x = 120;
			}
			else if (KEY_BACK == touch_key_array[index_value])
			{
				x = 200;
			}
			else if (KEY_SEARCH == touch_key_array[index_value])
			{
				x = 280;
			}
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 20);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 1380);
			input_mt_sync(ts->input_dev);
			input_sync(ts->input_dev);

			pre_touch = 1;	// alex.shi 这样才会发送弹起消息
	        goto exit_work_func;
		}
	#else
                for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
		#if GTP_DEBUG_ON
                        for (ret = 0; ret < 4; ++ret) {
                                if (key_codes[ret] == touch_key_array[i]) {
                                        GTP_DEBUG("Key: %s %s", key_names[ret], (key_value & (0x01 << i)) ? "Down" : "Up");
                                        break;
                                }
                        }
		#endif
                        input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));
                }
                touch_num = 0;
                pre_touch = 0;
	#endif
        }
#endif
        pre_key = key_value;

        GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT

#if GTP_WITH_PEN
        if (pre_pen && (touch_num == 0)) {
                GTP_DEBUG("Pen touch UP(Slot)!");
                input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
                input_mt_slot(ts->input_dev, 5);
                input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
                pre_pen = 0;
        }
#endif
        if (pre_touch || touch_num) {
                s32 pos = 0;
                u16 touch_index = 0;
                u8 report_num = 0;
                coor_data = &point_data[3];

                if(touch_num) {
                        id = coor_data[pos] & 0x0F;

#if GTP_WITH_PEN
                        id = coor_data[pos];
                        if ((id & 0x80)) {
                                GTP_DEBUG("Pen touch DOWN(Slot)!");
                                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

                                input_report_key(ts->input_dev, BTN_TOOL_PEN, 1);
                                input_mt_slot(ts->input_dev, 5);
                                input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 5);
                                input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
                                input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
                                input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
                                GTP_DEBUG("Pen/Stylus: (%d, %d)[%d]", input_x, input_y, input_w);
                                pre_pen = 1;
                                pre_touch = 0;
                        }
#endif

                        touch_index |= (0x01<<id);
                }

                GTP_DEBUG("id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",id, touch_index,pre_touch);
                for (i = 0; i < GTP_MAX_TOUCH; i++) {
#if GTP_WITH_PEN
                        if (pre_pen == 1) {
                                break;
                        }
#endif

                        if ((touch_index & (0x01<<i))) {
                                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

                                gtp_touch_down(ts, id, input_x, input_y, input_w);
                                pre_touch |= 0x01 << i;

                                report_num++;
                                if (report_num < touch_num) {
                                        pos += 8;
                                        id = coor_data[pos] & 0x0F;
                                        touch_index |= (0x01<<id);
                                }
                        } else {
                                gtp_touch_up(ts, i);
                                pre_touch &= ~(0x01 << i);
                        }
                }
        }
#else
        input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
        if (touch_num) {
                for (i = 0; i < touch_num; i++) {
                        coor_data = &point_data[i * 8 + 3];

                        id = coor_data[0] & 0x0F;
                        input_x  = coor_data[1] | (coor_data[2] << 8);
                        input_y  = coor_data[3] | (coor_data[4] << 8);
                        input_w  = coor_data[5] | (coor_data[6] << 8);

#if GTP_WITH_PEN
                        id = coor_data[0];
                        if (id & 0x80) {
                                GTP_DEBUG("Pen touch DOWN!");
                                input_report_key(ts->input_dev, BTN_TOOL_PEN, 1);
                                pre_pen = 1;
                                id = 0;
                        }
#endif

                        gtp_touch_down(ts, id, input_x, input_y, input_w);
                }
        } else if (pre_touch) {

#if GTP_WITH_PEN
                if (pre_pen == 1) {
                        GTP_DEBUG("Pen touch UP!");
                        input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
                        pre_pen = 0;
                }
#endif

                GTP_DEBUG("Touch Release!");
                gtp_touch_up(ts, 0);
        }

        pre_touch = touch_num;
#endif

        input_sync(ts->input_dev);

exit_work_func:
        if(!ts->gtp_rawdiff_mode) {
                ret = gtp_i2c_write(ts->client, end_cmd, 3);
                if (ret < 0) {
                        GTP_INFO("I2C write end_cmd error!");
                }
        }
        if (ts->use_irq) {
                gtp_irq_enable(ts);
        }
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode.
        HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
        struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

        GTP_DEBUG_FUNC();

        queue_work(goodix_wq, &ts->work);
        hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
        return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
        IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
        struct goodix_ts_data *ts = dev_id;

        GTP_DEBUG_FUNC();

        gtp_irq_disable(ts);

        queue_work(goodix_wq, &ts->work);

        return IRQ_HANDLED;
}
/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
void gtp_int_sync(s32 ms)
{
        GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
        msleep(ms);
        GTP_GPIO_AS_INT(GTP_INT_PORT);
}


/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
#if GTP_COMPATIBLE_MODE
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

	GTP_DEBUG_FUNC();
	GTP_INFO("Guitar reset");
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);   // begin select I2C slave addr
	msleep(ms);                         // T2: > 10ms
	// HIGH: 0x28/0x29, LOW: 0xBA/0xBB
	GTP_GPIO_OUTPUT(GTP_INT_PORT, client->addr == 0x14);

	msleep(2);                          // T3: > 100us
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

	msleep(6);                          // T4: > 5ms

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);    // end select I2C slave addr

#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		return;
	}
#endif

	gtp_int_sync(50);
#if GTP_ESD_PROTECT
	gtp_init_ext_watchdog(client);
#endif
}

#ifdef GTP_GESTURE_WAKEUP
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
        s8 ret = -1;
        s8 retry = 0;
        u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

        GTP_DEBUG_FUNC();

#if GTP_DBL_CLK_WAKEUP
        i2c_control_buf[2] = 0x09;
#endif

        gtp_irq_disable(ts);

        GTP_DEBUG("Entering doze mode.");
        while(retry++ < 5) {
                i2c_control_buf[0] = 0x80;
                i2c_control_buf[1] = 0x46;
                ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
                if (ret < 0) {
                        GTP_DEBUG("failed to set doze flag into 0x8046, %d", retry);
                        continue;
                }
                i2c_control_buf[0] = 0x80;
                i2c_control_buf[1] = 0x40;
                ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
                if (ret > 0) {
                        doze_status = DOZE_ENABLED;
                        GTP_INFO("GTP has been working in doze mode!");
						enable_irq_wake(ts->client->irq);
						irq_set_irq_type(ts->client->irq,IRQF_TRIGGER_HIGH|IRQF_NO_SUSPEND);
                        gtp_irq_enable(ts);
						GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
                        return ret;
                }
                msleep(10);
        }
        GTP_ERROR("GTP send doze cmd failed.");
        gtp_irq_enable(ts);
        return ret;
}
#else
/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
        s8 ret = -1;
        s8 retry = 0;
        u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_COMPATIBLE_MODE
        u8 status_buf[3] = {0x80, 0x44};
#endif

        GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type) {
                // GT9XXF: host interact with ic
                ret = gtp_i2c_read(ts->client, status_buf, 3);
                if (ret < 0) {
                        GTP_ERROR("failed to get backup-reference status");
                }

                if (status_buf[2] & 0x80) {
                        ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_STORE);
                        if (FAIL == ret) {
                                GTP_ERROR("failed to store bak_ref");
                        }
                }
        }
#endif

        GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
        msleep(5);

        while(retry++ < 5) {
                ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
                if (ret > 0) {
                        GTP_INFO("GTP enter sleep!");

                        return ret;
                }
                msleep(10);
        }
        GTP_ERROR("GTP send sleep cmd failed.");
        return ret;
}
#endif
/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
        u8 retry = 0;
        s8 ret = -1;

        GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type) {
                u8 opr_buf[3] = {0x41, 0x80};

                GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
                msleep(5);

                for (retry = 0; retry < 20; ++retry) {
                        // hold ss51 & dsp
                        opr_buf[2] = 0x0C;
                        ret = gtp_i2c_write(ts->client, opr_buf, 3);
                        if (FAIL == ret) {
                                GTP_ERROR("failed to hold ss51 & dsp!");
                                continue;
                        }
                        opr_buf[2] = 0x00;
                        ret = gtp_i2c_read(ts->client, opr_buf, 3);
                        if (FAIL == ret) {
                                GTP_ERROR("failed to get ss51 & dsp status!");
                                continue;
                        }
                        if (0x0C != opr_buf[2]) {
                                GTP_DEBUG("ss51 & dsp not been hold, %d", retry+1);
                                continue;
                        }
                        GTP_DEBUG("ss51 & dsp confirmed hold");

                        ret = gtp_fw_startup(ts->client);
                        if (FAIL == ret) {
                                GTP_ERROR("failed to startup GT9XXF, process recovery");
                                gtp_esd_recovery(ts->client);
                        }
                        break;
                }
                if (retry >= 10) {
                        GTP_ERROR("failed to wakeup, processing esd recovery");
                        gtp_esd_recovery(ts->client);
                } else {
                        GTP_INFO("GT9XXF gtp wakeup success");
                }
                return ret;
        }
#endif

#if GTP_POWER_CTRL_SLEEP
        while(retry++ < 5) {
                gtp_reset_guitar(ts->client, 20);

                GTP_INFO("GTP wakeup sleep.");
                return 1;
        }
#else
        while(retry++ < 10) {
#ifdef GTP_GESTURE_WAKEUP
                if (DOZE_WAKEUP != doze_status) {     // wakeup not by slide
                        GTP_DEBUG("wakeup by power, reset guitar");
                        doze_status = DOZE_DISABLED;
                        gtp_irq_disable(ts);
                        gtp_reset_guitar(ts->client, 10);
                        gtp_irq_enable(ts);
                } else {          // wakeup by slide
                        GTP_DEBUG("wakeup by slide/double-click, no reset guitar");
                        doze_status = DOZE_DISABLED;
#if GTP_ESD_PROTECT
                        gtp_init_ext_watchdog(ts->client);
#endif
                }

#else
                if (chip_gt9xxs == 1) {
                        gtp_reset_guitar(ts->client, 10);
                } else {
                        GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
                        msleep(5);
                }
#endif

                ret = gtp_i2c_test(ts->client);
                if (ret > 0) {
                        GTP_INFO("GTP wakeup sleep.");

#ifndef GTP_GESTURE_WAKEUP
                        if (chip_gt9xxs == 0) {
                                gtp_int_sync(25);
#if GTP_ESD_PROTECT
                                gtp_init_ext_watchdog(ts->client);
#endif
                        }
#endif

                        return ret;
                }
                gtp_reset_guitar(ts->client, 20);
        }
#endif

        GTP_ERROR("GTP wakeup sleep failed.");
        return ret;
}
#if GTP_DRIVER_SEND_CFG
static s32 gtp_get_info(struct goodix_ts_data *ts)
{
        u8 opr_buf[6] = {0};
        s32 ret = 0;

        opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+1) >> 8);
        opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+1) & 0xFF);

        ret = gtp_i2c_read(ts->client, opr_buf, 6);
        if (ret < 0) {
                return FAIL;
        }

        ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
        ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];

        opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+6) >> 8);
        opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+6) & 0xFF);

        ret = gtp_i2c_read(ts->client, opr_buf, 3);
        if (ret < 0) {
                return FAIL;
        }
        ts->int_trigger_type = opr_buf[2] & 0x03;

        GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
                 ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);

        return SUCCESS;
}
#endif

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
        s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
        s32 i = 0;
        u8 check_sum = 0;
        u8 opr_buf[16] = {0};
        u8 sensor_id = 0;

        u8 cfg_info_group1[] = CTP_CFG_GROUP1;
        u8 cfg_info_group2[] = CTP_CFG_GROUP2;
        u8 cfg_info_group3[] = CTP_CFG_GROUP3;
        u8 cfg_info_group4[] = CTP_CFG_GROUP4;
        u8 cfg_info_group5[] = CTP_CFG_GROUP5;
        u8 cfg_info_group6[] = CTP_CFG_GROUP6;
        u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
                              cfg_info_group4, cfg_info_group5, cfg_info_group6
                             };
        u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1),
                              CFG_GROUP_LEN(cfg_info_group2),
                              CFG_GROUP_LEN(cfg_info_group3),
                              CFG_GROUP_LEN(cfg_info_group4),
                              CFG_GROUP_LEN(cfg_info_group5),
                              CFG_GROUP_LEN(cfg_info_group6)
                            };

        GTP_DEBUG_FUNC();
        GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d",
                  cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
                  cfg_info_len[4], cfg_info_len[5]);


#if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type) {
                ts->fw_error = 0;
        } else
#endif
        {
                ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
                if (SUCCESS == ret) {
                        if (opr_buf[0] != 0xBE) {
                                ts->fw_error = 1;
                                GTP_ERROR("Firmware error, no config sent!");
                                return -1;
                        }
                }
        }

        if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
            (!cfg_info_len[3]) && (!cfg_info_len[4]) &&
            (!cfg_info_len[5])) {
                sensor_id = 0;
        } else {
#if GTP_COMPATIBLE_MODE
                msleep(50);
#endif
                ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
                if (SUCCESS == ret) {
                        if (sensor_id >= 0x06) {
                                GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
                                ts->pnl_init_error = 1;
#if GTP_COMPATIBLE_MODE
                                if (CHIP_TYPE_GT9F == ts->chip_type) {
                                        return -1;
                                } else
#endif
                                {
                                        gtp_get_info(ts);
                                }
                                return 0;
                        }
                } else {
                        GTP_ERROR("Failed to get sensor_id, No config sent!");
                        ts->pnl_init_error = 1;
                        return -1;
                }
                GTP_INFO("Sensor_ID: %d", sensor_id);
        }
        ts->gtp_cfg_len = cfg_info_len[sensor_id];
        GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id + 1, ts->gtp_cfg_len);

        if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH) {
                GTP_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id+1, ts->gtp_cfg_len);
                ts->pnl_init_error = 1;
                return -1;
        }

#if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type) {
                ts->fixed_cfg = 0;
        } else
#endif
        {
                ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);

                if (ret == SUCCESS) {
                        GTP_DEBUG("CFG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id+1,
                                  send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);

                        if (opr_buf[0] < 92) {
                                grp_cfg_version = send_cfg_buf[sensor_id][0];       // backup group config version
                                send_cfg_buf[sensor_id][0] = 0x00;
                                ts->fixed_cfg = 0;
                        } else {    // treated as fixed config, not send config
                                GTP_INFO("Ic fixed config with config version(%d, 0x%02X)", opr_buf[0], opr_buf[0]);
                                ts->fixed_cfg = 1;
                                gtp_get_info(ts);
                                return 0;
                        }
                } else {
                        GTP_ERROR("Failed to get ic config version!No config sent!");
                        return -1;
                }
        }

        memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
        memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);

#if GTP_CUSTOM_CFG
        config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
        config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
        config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
        config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);

        if (GTP_INT_TRIGGER == 0) { //RISING
                config[TRIGGER_LOC] &= 0xfe;
        } else if (GTP_INT_TRIGGER == 1) { //FALLING
                config[TRIGGER_LOC] |= 0x01;
        }
#endif  // GTP_CUSTOM_CFG

        check_sum = 0;
        for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
                check_sum += config[i];
        }
        config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else // driver not send config

        ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
        ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
        if (ret < 0) {
                GTP_ERROR("Read Config Failed, Using Default Resolution & INT Trigger!");
                ts->abs_x_max = GTP_MAX_WIDTH;
                ts->abs_y_max = GTP_MAX_HEIGHT;
                ts->int_trigger_type = GTP_INT_TRIGGER;
        }

#endif // GTP_DRIVER_SEND_CFG

        if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0)) {
                ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
                ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
                ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
        }

#if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type) {
                u8 sensor_num = 0;
                u8 driver_num = 0;
                u8 have_key = 0;

                have_key = (config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01);

                if (1 == ts->is_950) {
                        driver_num = config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
                        sensor_num = config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
                        if (have_key) {
                                driver_num--;
                        }
                        ts->bak_ref_len = (driver_num * (sensor_num - 1) + 2) * 2 * 6;
                } else {
                        driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
                        if (have_key) {
                                driver_num--;
                        }
                        sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F) + ((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
                        ts->bak_ref_len = (driver_num * (sensor_num - 2) + 2) * 2;
                }

                GTP_INFO("Drv * Sen: %d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
                         driver_num, sensor_num, have_key, ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
                return 0;
        } else
#endif
        {
#if GTP_DRIVER_SEND_CFG
                ret = gtp_send_cfg(ts->client);
                if (ret < 0) {
                        GTP_ERROR("Send config error.");
                }
                // set config version to CTP_CFG_GROUP, for resume to send config
                config[GTP_ADDR_LENGTH] = grp_cfg_version;
                check_sum = 0;
                for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
                        check_sum += config[i];
                }
                config[ts->gtp_cfg_len] = (~check_sum) + 1;
#endif
                GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
        }

        msleep(10);
        return 0;
}

/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
        2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
	s32 ret = -1;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version) {
		*version = (buf[7] << 8) | buf[6];
	}

	if (buf[5] == 0x00) {
		GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
	} else {
		if (buf[5] == 'S' || buf[5] == 's') {
			chip_gt9xxs = 1;
		}
		GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
	}
	return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
        2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

	while(retry++ < 5) {
		ret = gtp_i2c_read(client, test, 3);
		if (ret > 0) {
			return ret;
		}
		GTP_ERROR("GTP i2c test failed time %d.",retry);
		msleep(10);
	}
	return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	s32 ret = 0;
	struct regulator *reg_vdd;
	struct i2c_client *client = ts->client;

	GTP_DEBUG_FUNC();
	ret = GTP_GPIO_REQUEST(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32)GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		ts->client->irq = GTP_INT_IRQ;
	}

	ret = GTP_GPIO_REQUEST(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",(s32)GTP_RST_PORT,ret);
		ret = -ENODEV;
	}

	//GTP_GPIO_AS_INPUT(GTP_RST_PORT);

	reg_vdd = regulator_get(&client->dev, "vdd18"/*pdata->vdd_name*/);
	//if (!WARN(IS_ERR(reg_vdd), "[GTP] goodix_ts_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
		//if(!strcmp(pdata->vdd_name,"vdd18"))
			regulator_set_voltage(reg_vdd,1800000,1800000);
		//if(!strcmp(pdata->vdd_name,"vdd28"))
			//regulator_set_voltage(reg_vdd, 2800000, 2800000);
		regulator_enable(reg_vdd);
	//}
	msleep(100);

	gtp_reset_guitar(ts->client, 20);

	if(ret < 0) {
		GTP_GPIO_FREE(GTP_RST_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);
	}

	return ret;
}

/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);
#if defined(GTP_GESTURE_WAKEUP)
		ret  = request_irq(ts->client->irq,
			goodix_ts_irq_handler,
			irq_table[ts->int_trigger_type]| IRQF_ONESHOT | IRQF_NO_SUSPEND,
			ts->client->name,
			ts);
#else
	ret  = request_irq(ts->client->irq,
			goodix_ts_irq_handler,
			irq_table[ts->int_trigger_type],
			ts->client->name,
			ts);
#endif
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return -1;
	} else {
		gtp_irq_disable(ts);
		ts->use_irq = 1;
		return 0;
	}
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if GTP_ICS_SLOT_REPORT
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	input_mt_init_slots(ts->input_dev, 16);     // in case of "out of memory"
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[index]);
	}
#endif

#ifdef GTP_GESTURE_WAKEUP
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif

#if GTP_WITH_PEN
	// pen support
	__set_bit(BTN_TOOL_PEN, ts->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	//__set_bit(INPUT_PROP_POINTER, ts->input_dev->propbit);
#endif

#if GTP_CHANGE_X2Y
	GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

#if GTP_PROXIMITY
   input_set_abs_params(ts->input_dev, ABS_DISTANCE, 0, 1, 0, 0);
#endif
    printk(" ts->abs_x_max================%d\n", ts->abs_x_max);
    printk(" ts->abs_y_max================%d\n", ts->abs_y_max);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	sprintf(phys, "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", ts->input_dev->name);
		return -ENODEV;
	}

	return 0;
}

//************** For GT9XXF Start *************//
#if GTP_COMPATIBLE_MODE

s32 gtp_fw_startup(struct i2c_client *client)
{
	u8 opr_buf[4];
	s32 ret = 0;

	//init sw WDT
	opr_buf[0] = 0xAA;
	ret = i2c_write_bytes(client, 0x8041, opr_buf, 1);
	if (ret < 0) {
		return FAIL;
	}

	//release SS51 & DSP
	opr_buf[0] = 0x00;
	ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
	if (ret < 0) {
		return FAIL;
	}
	//int sync
	gtp_int_sync(25);

	//check fw run status
	ret = i2c_read_bytes(client, 0x8041, opr_buf, 1);
	if (ret < 0) {
		return FAIL;
	}
	if(0xAA == opr_buf[0]) {
		GTP_ERROR("IC works abnormally,startup failed.");
		return FAIL;
	} else {
		GTP_INFO("IC works normally, Startup success.");
		opr_buf[0] = 0xAA;
		i2c_write_bytes(client, 0x8041, opr_buf, 1);
		return SUCCESS;
	}
}

static s32 gtp_esd_recovery(struct i2c_client *client)
{
	s32 retry = 0;
	s32 ret = 0;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);

	gtp_irq_disable(ts);

	GTP_INFO("GT9XXF esd recovery mode");
	gtp_reset_guitar(client, 20);       // reset & select I2C addr
	for (retry = 0; retry < 5; ++retry) {
		ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY);
		if (FAIL == ret) {
			GTP_ERROR("esd recovery failed %d", retry+1);
			continue;
		}
		ret = gtp_fw_startup(ts->client);
		if (FAIL == ret) {
			GTP_ERROR("GT9XXF start up failed %d", retry+1);
			continue;
		}
		break;
	}
	gtp_irq_enable(ts);

	if (retry >= 5) {
		GTP_ERROR("failed to esd recovery");
		return FAIL;
	}

	GTP_INFO("Esd recovery successful");
	return SUCCESS;
}

void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_OFF);
#endif
	GTP_DEBUG_FUNC();

	gtp_esd_recovery(client);

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
}

static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode)
{
        s32 ret = 0;
        s32 i = 0;
        s32 j = 0;
        u16 ref_sum = 0;
        u16 learn_cnt = 0;
        u16 chksum = 0;
        s32 ref_seg_len = 0;
        s32 ref_grps = 0;
        struct file *ref_filp = NULL;
        u8 *p_bak_ref;

        ret = gup_check_fs_mounted("/data");
        if (FAIL == ret) {
                ts->ref_chk_fs_times++;
                GTP_DEBUG("Ref check /data times/MAX_TIMES: %d / %d", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
                if (ts->ref_chk_fs_times < GTP_CHK_FS_MNT_MAX) {
                        msleep(50);
                        GTP_INFO("/data not mounted.");
                        return FAIL;
                }
                GTP_INFO("check /data mount timeout...");
        } else {
                GTP_INFO("/data mounted!!!(%d/%d)", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
        }

        p_bak_ref = (u8 *)kzalloc(ts->bak_ref_len, GFP_KERNEL);

        if (NULL == p_bak_ref) {
                GTP_ERROR("Allocate memory for p_bak_ref failed!");
                return FAIL;
        }

        if (ts->is_950) {
                ref_seg_len = ts->bak_ref_len / 6;
                ref_grps = 6;
        } else {
                ref_seg_len = ts->bak_ref_len;
                ref_grps = 1;
        }
        ref_filp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
        if (IS_ERR(ref_filp)) {
                GTP_INFO("%s is unavailable, default backup-reference used", GTP_BAK_REF_PATH);
                goto bak_ref_default;
        }

        switch (mode) {
        case GTP_BAK_REF_SEND:
                GTP_INFO("Send backup-reference");
                ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
                ret = ref_filp->f_op->read(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
                if (ret < 0) {
                        GTP_ERROR("failed to read bak_ref info from file, sending defualt bak_ref");
                        goto bak_ref_default;
                }
                for (j = 0; j < ref_grps; ++j) {
                        ref_sum = 0;
                        for (i = 0; i < (ref_seg_len); i += 2) {
                                ref_sum += (p_bak_ref[i + j * ref_seg_len] << 8) + p_bak_ref[i+1 + j * ref_seg_len];
                        }
                        learn_cnt = (p_bak_ref[j * ref_seg_len + ref_seg_len -4] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -3]);
                        chksum = (p_bak_ref[j * ref_seg_len + ref_seg_len -2] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -1]);
                        GTP_DEBUG("learn count = %d", learn_cnt);
                        GTP_DEBUG("chksum = %d", chksum);
                        GTP_DEBUG("ref_sum = 0x%04X", ref_sum & 0xFFFF);
                        // Sum(1~ref_seg_len) == 1
                        if (1 != ref_sum) {
                                GTP_INFO("wrong chksum for bak_ref, reset to 0x00 bak_ref");
                                memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
                                p_bak_ref[ref_seg_len + j * ref_seg_len - 1] = 0x01;
                        } else {
                                if (j == (ref_grps - 1)) {
                                        GTP_INFO("backup-reference data in %s used", GTP_BAK_REF_PATH);
                                }
                        }
                }
                ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
                if (FAIL == ret) {
                        GTP_ERROR("failed to send bak_ref because of iic comm error");
                        filp_close(ref_filp, NULL);
                        return FAIL;
                }
                break;

        case GTP_BAK_REF_STORE:
                GTP_INFO("Store backup-reference");
                ret = i2c_read_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
                if (ret < 0) {
                        GTP_ERROR("failed to read bak_ref info, sending default back-reference");
                        goto bak_ref_default;
                }
                ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
                ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
                break;

        default:
                GTP_ERROR("invalid backup-reference request");
                break;
        }
        filp_close(ref_filp, NULL);
        return SUCCESS;

bak_ref_default:

        for (j = 0; j < ref_grps; ++j) {
                memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
                p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;  // checksum = 1
        }
        ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (!IS_ERR(ref_filp)) {
                GTP_INFO("write backup-reference data into %s", GTP_BAK_REF_PATH);
                ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
                ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
                filp_close(ref_filp, NULL);
        }
        if (ret == FAIL) {
                GTP_ERROR("failed to load the default backup reference");
                return FAIL;
        }
        return SUCCESS;
}


static s32 gtp_verify_main_clk(u8 *p_main_clk)
{
	u8 chksum = 0;
	u8 main_clock = p_main_clk[0];
	s32 i = 0;

	if (main_clock < 50 || main_clock > 120) {
		return FAIL;
	}

	for (i = 0; i < 5; ++i) {
		if (main_clock != p_main_clk[i]) {
			return FAIL;
		}
		chksum += p_main_clk[i];
	}
	chksum += p_main_clk[5];
	if ( (chksum) == 0) {
		return SUCCESS;
	} else {
		return FAIL;
	}
}

static s32 gtp_main_clk_proc(struct goodix_ts_data *ts)
{
	s32 ret = 0;
	s32 i = 0;
	s32 clk_chksum = 0;
	struct file *clk_filp = NULL;
	u8 p_main_clk[6] = {0};

	ret = gup_check_fs_mounted("/data");
	if (FAIL == ret) {
		ts->clk_chk_fs_times++;
		GTP_DEBUG("Clock check /data times/MAX_TIMES: %d / %d", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
		if (ts->clk_chk_fs_times < GTP_CHK_FS_MNT_MAX) {
			msleep(50);
			GTP_INFO("/data not mounted.");
			return FAIL;
		}
		GTP_INFO("Check /data mount timeout!");
	} else {
		GTP_INFO("/data mounted!!!(%d/%d)", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
	}

	clk_filp = filp_open(GTP_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(clk_filp)) {
		GTP_ERROR("%s is unavailable, calculate main clock", GTP_MAIN_CLK_PATH);
	} else {
		clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
		clk_filp->f_op->read(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);

		ret = gtp_verify_main_clk(p_main_clk);
		if (FAIL == ret) {
			// recalculate main clock & rewrite main clock data to file
			GTP_ERROR("main clock data in %s is wrong, recalculate main clock", GTP_MAIN_CLK_PATH);
		} else {
			GTP_INFO("main clock data in %s used, main clock freq: %d", GTP_MAIN_CLK_PATH, p_main_clk[0]);
			filp_close(clk_filp, NULL);
			goto update_main_clk;
		}
	}

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
	ret = gup_clk_calibration();
	gtp_esd_recovery(ts->client);

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif

	GTP_INFO("calibrate main clock: %d", ret);
	if (ret < 50 || ret > 120) {
		GTP_ERROR("wrong main clock: %d", ret);
		goto exit_main_clk;
	}

	// Sum{0x8020~0x8025} = 0
	for (i = 0; i < 5; ++i) {
		p_main_clk[i] = ret;
		clk_chksum += p_main_clk[i];
	}
	p_main_clk[5] = 0 - clk_chksum;

	if (!IS_ERR(clk_filp)) {
		GTP_DEBUG("write main clock data into %s", GTP_MAIN_CLK_PATH);
		clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
		clk_filp->f_op->write(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);
		filp_close(clk_filp, NULL);
	}

update_main_clk:
	ret = i2c_write_bytes(ts->client, GTP_REG_MAIN_CLK, p_main_clk, 6);
	if (FAIL == ret) {
		GTP_ERROR("update main clock failed!");
		return FAIL;
	}
	return SUCCESS;

exit_main_clk:
	if (!IS_ERR(clk_filp)) {
		filp_close(clk_filp, NULL);
	}
	return FAIL;
}

s32 gtp_gt9xxf_init(struct i2c_client *client)
{
	s32 ret = 0;

	ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN);
	if (FAIL == ret) {
		return FAIL;
	}

	ret = gtp_fw_startup(client);
	if (FAIL == ret) {
		return FAIL;
	}
	return SUCCESS;
}

void gtp_get_chip_type(struct goodix_ts_data *ts)
{
	u8 opr_buf[10] = {0x00};
	s32 ret = 0;

	msleep(10);

	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CHIP_TYPE, opr_buf, 10);

	if (FAIL == ret) {
		GTP_ERROR("Failed to get chip-type, set chip type default: GOODIX_GT9");
		ts->chip_type = CHIP_TYPE_GT9;
		return;
	}

	if (!memcmp(opr_buf, "GOODIX_GT9", 10)) {
		ts->chip_type = CHIP_TYPE_GT9;
	} else { // GT9XXF
		ts->chip_type = CHIP_TYPE_GT9F;
	}
	GTP_INFO("Chip Type: %s", (ts->chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}

#endif
//************* For GT9XXF End ************//

static int check_ctp_chip(void)
{
	ctp_lock_mutex();
	tp_device_id(0x0960);
	ctp_unlock_mutex();
	return 0;
}

static int remove_ctp_chip(void)
{
	ctp_lock_mutex();
	tp_device_id(0xFFFF);
	ctp_unlock_mutex();
	return 0;
}

/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes.
        0: succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_data *ts;
	u16 version_info;

	//do NOT remove these logs
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP Driver Built@%s, %s", __TIME__, __DATE__);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	if(tp_device_id(0)!=0)
	{
		printk("CTP(0x%x)Exist!", tp_device_id(0));
		return -ENODEV;
	}

	i2c_connect_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		GTP_ERROR("Creat workqueue failed.");
		ret = -ENOMEM;
		goto err_kzalloc;    
	}
#if GTP_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif

	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	spin_lock_init(&ts->irq_lock);          // 2.6.39 later
	// ts->irq_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before
#if GTP_ESD_PROTECT
	ts->clk_tick_cnt = 2 * HZ;      // HZ: clock ticks in 1 second generated by system
	GTP_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);
	spin_lock_init(&ts->esd_lock);
	// ts->esd_lock = SPIN_LOCK_UNLOCKED;
#endif
	i2c_set_clientdata(client, ts);

	ts->gtp_rawdiff_mode = 0;

	ret = gtp_request_io_port(ts);
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		goto err_irq;    
	}

#if GTP_COMPATIBLE_MODE
	gtp_get_chip_type(ts);

	if (CHIP_TYPE_GT9F == ts->chip_type) {
		ret = gtp_gt9xxf_init(ts->client);
		if (FAIL == ret) {
			GTP_INFO("Failed to init GT9XXF.");
			goto err_testi2c;    
		}
	}
#endif

	ret = gtp_i2c_test(client);
	if (ret < 0) {
		GTP_ERROR("I2C communication ERROR!");
		goto err_testi2c;    
	}

	ret = gtp_read_version(client, &version_info);
	if (ret < 0) {
		GTP_ERROR("Read version failed.");
		goto err_testi2c;    
	}

	ret = check_ctp_chip();
	if (ret<0) {
		GTP_ERROR("ft5x0x_ts_probe failed(%d): failed to check_ctp_chip\n", ret);
		goto exit_check_ctp_chip;
	}

	ret = gtp_init_panel(ts);
	if (ret < 0) {
		GTP_ERROR("GTP init panel failed.");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
		goto exit_check_ctp_chip;
	}

#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(ts);
	if (ret < 0) {
		GTP_ERROR("Create update thread error.");
		goto exit_check_ctp_chip;
	}
#endif

#if TOUCH_VIRTUAL_KEYS
	virtual_keys_init();
#endif
#if GTP_PROXIMITY	
	ret = misc_register(&gtp_proximity_misc);
	if (ret < 0)
	{
		pr_err("%s: could not register misc device\n", __func__);
		goto err_mis_reg;
	}
#endif
	ret = gtp_request_input_dev(ts);
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
		goto err_virtual_keys;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	ret = gtp_request_irq(ts);
	if (ret < 0) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
#ifdef GTP_GESTURE_WAKEUP

	input_set_capability(ts->input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(ts->input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(ts->input_dev, EV_KEY, KEY_UP);
	input_set_capability(ts->input_dev, EV_KEY, KEY_DOWN);
	input_set_capability(ts->input_dev, EV_KEY, KEY_U);
	input_set_capability(ts->input_dev, EV_KEY, KEY_O);
	input_set_capability(ts->input_dev, EV_KEY, KEY_W);
	input_set_capability(ts->input_dev, EV_KEY, KEY_M);
	input_set_capability(ts->input_dev, EV_KEY, KEY_E);
	input_set_capability(ts->input_dev, EV_KEY, KEY_C);
	input_set_capability(ts->input_dev, EV_KEY, KEY_V);
	input_set_capability(ts->input_dev, EV_KEY, KEY_Z);
	input_set_capability(ts->input_dev, EV_KEY, KEY_S);
	input_set_capability(ts->input_dev, EV_KEY, KEY_PREVIOUSSONG);
	input_set_capability(ts->input_dev, EV_KEY, KEY_NEXTSONG);
	input_set_capability(ts->input_dev, EV_KEY, KEY_PLAYPAUSE);
	enable_irq_wake(client->irq);
#endif
    }

#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
	{
		extern void zyt_info_s2(char* s1,char* s2);
		zyt_info_s2("[TP] : ","GT9XX");
	}
	return 0;

err_version:
	if (ts) 
	{
		if (ts->use_irq)
		{
			GTP_GPIO_AS_INPUT(GTP_INT_PORT);
			//GTP_GPIO_FREE(GTP_INT_PORT);
			free_irq(client->irq, ts);
		}
		else
		{
			hrtimer_cancel(&ts->timer);
		}
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
err_virtual_keys:
#if TOUCH_VIRTUAL_KEYS
	virtual_keys_destroy();
#endif
exit_check_ctp_chip:
	remove_ctp_chip();
#if GTP_PROXIMITY	
err_mis_reg:
	misc_deregister(&gtp_proximity_misc);
#endif
err_testi2c:
	if(ts->client->irq>0)
	{
		//sprd_free_gpio_irq(ts->client->irq);
		ts->client->irq=-1;
		GTP_GPIO_FREE(GTP_RST_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);
	}
err_irq:
	i2c_set_clientdata(client, NULL);
	ts->client = NULL;
	if (goodix_wq)
	{
		destroy_workqueue(goodix_wq);
	}
err_kzalloc:
	kfree(ts);
	return ret;
}


/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();
	if (ts) 
	{
		if (ts->use_irq)
		{
			GTP_GPIO_AS_INPUT(GTP_INT_PORT);
			//GTP_GPIO_FREE(GTP_INT_PORT);
			free_irq(client->irq, ts);
		}
		else
		{
			hrtimer_cancel(&ts->timer);
		}
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
#if TOUCH_VIRTUAL_KEYS
	virtual_keys_destroy();
#endif
#if GTP_PROXIMITY	
	misc_deregister(&gtp_proximity_misc);
#endif
	remove_ctp_chip();
	if(ts->client->irq>0)
	{
		//sprd_free_gpio_irq(ts->client->irq);
		ts->client->irq=-1;
		GTP_GPIO_FREE(GTP_RST_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);
	}
	i2c_set_clientdata(client, NULL);
	ts->client = NULL;
	if (goodix_wq)
	{
		destroy_workqueue(goodix_wq);
	}
	kfree(ts);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(h, struct goodix_ts_data, early_suspend);

	GTP_DEBUG_FUNC();
#if GTP_PROXIMITY
    if (gtp_proximity_start == 1)
	return;
	suspend_entry_flag = 1;
#endif	

#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
	ts->gtp_is_suspend = 1;

#ifdef GTP_GESTURE_WAKEUP
	struct file *fp; 
		
	fp = filp_open(GESTURE_FUNCTION_SWITCH, O_RDONLY , 0); 
	if (IS_ERR(fp)) 
	{ 
		pr_info("open file %s error!\n", GESTURE_FUNCTION_SWITCH);
		s_gesture_switch = 1;	
	} 
	else 
	{
		printk("open file %s success!\n", GESTURE_FUNCTION_SWITCH);
		s_gesture_switch = 0;	
		filp_close(fp, NULL); 
	}
	if (1==s_gesture_switch)
	{
		ret = gtp_enter_doze(ts);
	}
#else
	if (ts->use_irq) {
		gtp_irq_disable(ts);
	} else {
		hrtimer_cancel(&ts->timer);
	}
	ret = gtp_enter_sleep(ts);
#endif
	if (ret < 0) {
		GTP_ERROR("GTP early suspend failed.");
	}
	// to avoid waking up while not sleeping
	//  delay 48 + 10ms to ensure reliability
	msleep(58);
}




	
/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(h, struct goodix_ts_data, early_suspend);


	GTP_DEBUG_FUNC();
#ifdef GTP_GESTURE_WAKEUP
	const u8 irq_table[] = GTP_IRQ_TAB;
	disable_irq_wake(ts->client->irq);
	irq_set_irq_type(ts->client->irq,irq_table[ts->int_trigger_type]);
#endif
	

#if GTP_PROXIMITY
    if (gtp_proximity_start == 1)
{
		if(suspend_entry_flag == 0)
			{
	           return;
			}
}
	suspend_entry_flag = 0;
#endif
	ret = gtp_wakeup_sleep(ts);

#ifdef GTP_GESTURE_WAKEUP
	doze_status = DOZE_DISABLED;
#endif

	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
#if (GTP_COMPATIBLE_MODE)
	if (CHIP_TYPE_GT9F == ts->chip_type) {
		// do nothing
	} else
#endif
	{
		gtp_send_cfg(ts->client);
	}

	if (ts->use_irq) {
		gtp_irq_enable(ts);
	} else {
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
}
#endif

#if GTP_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret=-1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];
	//msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
	//msgs[1].scl_rate = 300 * 1000;

	while(retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	if ((retries >= 5)) {
		GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	}
	return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client,u8 *buf,s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;
	//msg.scl_rate = 300 * 1000;    // for Rockchip, etc

	while(retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)break;
		retries++;
	}
	if((retries >= 5)) {
		GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	}
	return ret;
}
/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);
	spin_lock(&ts->esd_lock);

	if (SWITCH_ON == on) {   // switch on esd
		if (!ts->esd_running) {
			ts->esd_running = 1;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd started");
			queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
		} else {
			spin_unlock(&ts->esd_lock);
		}
	} else { // switch off esd
		if (ts->esd_running) {
			ts->esd_running = 0;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd cancelled");
			cancel_delayed_work_sync(&gtp_esd_check_work);
		} else {
			spin_unlock(&ts->esd_lock);
		}
	}
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation.
        1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
	u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
	GTP_DEBUG("[Esd]Init external watchdog");
	return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
    Esd protect function.
    External watchdog added by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	s32 i;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;
	u8 esd_buf[4] = {0x80, 0x40};

	GTP_DEBUG_FUNC();

	ts = i2c_get_clientdata(i2c_connect_client);

	if (ts->gtp_is_suspend) {
		GTP_INFO("Esd suspended!");
		return;
	}

	for (i = 0; i < 3; i++) {
		ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);

		GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
		if ((ret < 0)) {
			// IIC communication problem
			continue;
		} else {
			if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA)) {
				// IC works abnormally..
				u8 chk_buf[4] = {0x80, 0x40};

				gtp_i2c_read_no_rst(ts->client, chk_buf, 4);

				GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);

				if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA)) {
					i = 3;
					break;
				} else {
					continue;
				}
			} else {
				// IC works normally, Write 0x8040 0xAA, feed the dog
				esd_buf[2] = 0xAA;
				gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
				break;
			}
		}
	}
	if (i >= 3) {
#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == ts->chip_type) {
			if (ts->rqst_processing) {
				GTP_INFO("Request processing, no esd recovery");
			} else {
				GTP_ERROR("IC working abnormally! Process esd recovery.");
				gtp_esd_recovery(ts->client);
			}
		} else
#endif
		{
			GTP_ERROR("IC working abnormally! Process reset guitar.");
			gtp_reset_guitar(ts->client, 50);
		}
	}

	if(!ts->gtp_is_suspend) {
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
	} else {
		GTP_INFO("Esd suspended!");
	}
	return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, goodix_ts_id);

static const struct of_device_id goodix_of_match[] = {
	{ .compatible = "Goodix,goodix_ts", },
	{ }
};


MODULE_DEVICE_TABLE(of, goodix_of_match);

static struct i2c_driver goodix_ts_driver = {
	.probe      = goodix_ts_probe,
	.remove     = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = goodix_ts_early_suspend,
	.resume     = goodix_ts_late_resume,
#endif
	.id_table   = goodix_ts_id,
	.driver = {
		.name     = GTP_I2C_NAME,
		.owner    = THIS_MODULE,
		.of_match_table = goodix_of_match,
	},
};

/*******************************************************
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int /*__devinit*/ goodix_ts_init(void)
{
	GTP_INFO("Func:%s@Line:%d\n",__func__,__LINE__);
	if(tp_device_id(0)!=0)
	{
		printk("CTP(0x%x)Exist!", tp_device_id(0));
		return -ENODEV;
	}

	return i2c_add_driver(&goodix_ts_driver);
}

/*******************************************************
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
	GTP_INFO("Func:%s@Line:%d\n",__func__,__LINE__);
	i2c_del_driver(&goodix_ts_driver);
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
