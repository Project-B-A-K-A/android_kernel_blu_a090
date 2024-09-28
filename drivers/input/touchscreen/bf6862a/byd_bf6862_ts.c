/*
 * File:         byd_bfxxxx_ts.c
 *
 * Created:	     2015-06-07
 * Depend on:    byd_bfxxxx_ts.h
 * Description:  BYD TouchScreen IC driver for Android
 *
 * Copyright (C) 2015 BYD Company Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* History
20150608,jiayuanyuan IC algorithm partially intruduced in driver
          function added: byd_ts_read_coor(), byd_ts_read_IC_para()
		  global variable: buf[50]
*/
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#include <soc/sprd/regulator.h>
#include <soc/sprd/i2c-sprd.h>
#include <asm/mach/irq.h>
#include <soc/sprd/board.h>
#include <linux/gpio.h>	
#include <linux/module.h>
#include <linux/platform_device.h> //module_init
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/slab.h>      // kzalloc()
//#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
//#endif
#include <linux/interrupt.h> // disable_irq(), disable_irq_nosync()
#include <linux/i2c.h>       // i2c_client, i2c_master_send(), i2c_transfer(), I2C_M_RD
#include <linux/delay.h>     // mdelay()
#include <linux/gpio.h>
#include "byd_bf6862_ts.h"
#include <linux/wakelock.h>
#include <soc/sprd/board.h>

#ifdef CONFIG_REPORT_PROTOCOL_B
#include <linux/input/mt.h> // Protocol B, for input_mt_slot(), input_mt_init_slot()
#endif

////////////////// extension for additional functionalities ///////////////////
// --------------- compile independently --------------------------------------
#include <linux/miscdevice.h> // misc_register(), misc_deregister()

// interface for accessing external functions and global variables
int byd_ts_download(struct i2c_client *client, int module_num);
int byd_ts_get_resolution(struct i2c_client *client, int *xy_res);
static int  byd_ctp_read_byte(struct i2c_client *client, uint8_t *address, char addresslen, uint8_t *comparedata, unsigned char* pbt_buf, char type);
int byd_ts_get_firmware_version(struct i2c_client *client, unsigned char* version);
int byd_ts_upgrade_firmware(struct i2c_client *client);
static int smbus_master_recv(struct i2c_client *client, char *buf, int len);
int byd_ts_read_IC_para(struct i2c_client *client);
int byd_ts_read_coor(struct i2c_client *client);
extern struct miscdevice byd_ts_device;
// ----------------------------------------------------------------------------

#define  TPD_PROXIMITY_DEBUG_ON  1

///add  by jinq start for adb  open debug
extern int zyt_adb_debug;
#define ZYT_PRINTK(fmt,arg...)  do{\
								if(zyt_adb_debug||TPD_PROXIMITY_DEBUG_ON)printk("<<ZYT_PRINTK>> [%d]"fmt"\n",__LINE__, ##arg);\
							     }while(0)

#if defined(ZCFG_MK_TP_PROXIMITY)
#define CONFIG_TOUCHSCREEN_WITH_PROXIMITY
#endif


#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
static atomic_t BF6862_PROXIMITY_ENABLE;
static atomic_t BF6862_PROXIMITY_STATE;
static int BF6862_tp_proximity_opened=0;
#define AL3006_PLS_DEVICE 		        "currency_tp"  //according to  the customer modify settings
#define AL3006_PLS_INPUT_DEV  	        "currency_tp" //according to  the customer modify settings

static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_flag_one = 0;//add by lc
static u8 tpd_proximity_detect = 1;  //0-->close ; 1--> far away       00
static u8 tpd_als_opened = 0;   
struct input_dev *pro_input_dev;
struct timer_list ps_timer;
static struct wake_lock pls_delayed_work_wake_lock;

#define PROXIMITY_DEVICE   "gtp_proximity"

//ioctl cmd
#define BF_IOCTL_MAGIC			0X5D
#define BF_IOCTL_PROX_ON		_IO(BF_IOCTL_MAGIC, 7)
#define BF_IOCTL_PROX_OFF		_IO(BF_IOCTL_MAGIC, 8)


#endif
//gesture function 

extern int tp_device_id(int id);
extern void ctp_lock_mutex(void);
extern void ctp_unlock_mutex(void);

#ifdef BYD_GESTURE
static int gesture_enable = 0;
struct wake_lock poll_wake_lock;
struct wake_lock suspend_gestrue_lock;
 static unsigned char Byd_Ts_Gesture_Keys[] = BYD_TS_GESTURE_KEYS;
#endif

//#include <linux/kernel.h> 4:KERN_WARNING 5:KERN_NOTICE 6:KERN_INFO 7:KERN_DEBUG
#ifdef TS_DEBUG_MSG
  #define TS_DBG(format, ...) \
	printk(format, ##__VA_ARGS__)
	//printk(KERN_INFO "BYD_TS *** " format "\n", ## __VA_ARGS__)
  #define PR_BUF(pr_msg, buf, len) \
  { \
	int _i_; \
	TS_DBG("%s: %s", __FUNCTION__, pr_msg); \
	for (_i_ = 0; _i_ < len; _i_++) { \
		if(_i_%16 == 0) TS_DBG("\n"); \
		/* if(_i_%16 == 0) TS_DBG("\n%02x ", _i_); */ \
		TS_DBG(" %02x", *(buf + _i_)); \
	} \
	TS_DBG("\n"); \
  }
#else
  #define TS_DBG(format, ...)
  #define PR_BUF(pr_msg, buf, len)
#endif

#ifndef CONFIG_DRIVER_BASIC_FUNCTION
	// Acquiring firmware version
	  #define CONFIG_SUPPORT_VERSION_ACQUISITION
	// Resolution acquiring (from chip)
	  #define CONFIG_SUPPORT_RESOLUTION_ACQUISITION
	  //#define CONFIG_SUPPORT_MODULE_ID
	#ifdef CONFIG_CTS02
	// On-line update for firmware and/or parameter (flash burning)
	  #define CONFIG_SUPPORT_FIRMWARE_UPG

	   #define CONFIG_TP_UPGRADE_SUPPORT
	  //for version compare
	  #define VERSION_IDENTIFY
	#endif
	// Raw data acquiring
	  #define CONFIG_SUPPORT_RAW_DATA_ACQUISITION
#endif

/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/

/*    The max number of points/fingers for muti-points TP */
	  #define BYD_TS_MT_NUMBER_POINTS	2

   /* The size of finger point buffer */
	  #define BYD_TS_MT_BUFFER_SIZE	BYD_TS_MT_NUMBER_POINTS * 4 + 1

   /* The size (and max size) of finger touching surface of touch panel */
	  #define BYD_TS_TOUCHSIZE	100

/*    对Android 2.3及以下版本，除了可以使用过去旧协议外，还可以选择触摸设备匿名
      协议（协议A，4.0版本的协议） */
#if !defined(CONFIG_ANDROID_4_ICS) && !defined(CONFIG_REPORT_PROTOCOL_B)
	//#define CONFIG_REPORT_PROTOCOL_A
#endif

   /* Touch Panel Default Resolution:
      The RESOLUTION VALUES MUST BE SAME as those in the chip's resolution of
      given TP. They are used when the TP's IC chip does not support the
      functionality of reading resolution from the chip.
      Since Linux/Adroid is able to automatically adapt TP's resolution to
      Screen's (LCD's), a constant resolution as 2048x2048 is STRONGLY
	  RECOMMENDED.*/
	  #define TS_X_RES 	2048 // 320
	  #define TS_Y_RES  2048 // 480

   /* to enable the functionality for resolution transformation in the driver
      (for some special case), the screen (LCD) resolution below should be
      presented */
	//#define REPORT_X_RES  320 //800
	//#define REPORT_Y_RES  480

   /* The swap of X Y is ONLY NEEDD when the resolution of X and Y is same
      (e.g. 2048x2048) and only if this swap is absolutely necessary. DO NOT
      try to use this as a way to adapt screen's orientation otherwise. */
	//#define REPORT_XY_SWAPPED

   /* Coordinate transformation, used only if the coordinates in the chip is not
      properly transformed. NOTE: Changing the chip's firmware is prefered way. */
	//#define REPORT_X_REVERSED
	//#define REPORT_Y_REVERSED

   /* About Touch Panel (TP) resolution and screen resolution transformation:
        The TP resolution values and its order MUST BE SAME AS the one in
      the chip's resolution registers (0x0c). The order of the resolution
      values determines the orientation of the screen, e.g. 480x320 indicates
      landscape (i.e. horizontal) while 320x480 represents portrait (vertical).
      Be aware that the order of X and Y is determined by the order of the
      presenting of the coordinate values in point registers, the 1st register
      is alwayse X and the 2nd is constantly Y.
        Although this TP driver is capable of transforming TP's resolution to LCD's,
      since Linux/Adroid's up layer is also able to automatically adapt TP's
      resolution to the LCD's (as long as the driver correctly reporting TP's
      resolution), the transformation in the driver is really not necessary,
      and presenting a constant TP resolution as 2048x2048 is STRONGLY
      RECOMMENDED. */

	// Acquiring firmware version
	  #define I2C_READ_FIRMWARE_VERSION {0x00} // {0x0c, 0x0d, 0x02}
	// resolution acquiring (from chip)
	  #define I2C_READ_RESOLUTION {0x08}
	// Raw data acquiring
	  #define I2C_READ_RAW_DATA {0x0c, 0x0d, 0x03}
	  
	//  #define I2C_READ_RAW_DATA {0x0c, 0x0d, 0x03}	  
	  #define I2C_READ_RAW_DATA_SINGLE_FRAME I2C_READ_RAW_DATA // *** not implimented yet ***
	  #define I2C_RESET {0x0c, 0x0d, 0x04}
	  #define I2C_EXIT_DEBUG_MODE I2C_RESET
	  #define CHANNELS_MAX 38
	  
#ifdef CONFIG_TP_UPGRADE_SUPPORT
struct input_dev *fwup_input_dev;
static struct kobject *fwup_kobj;
#endif


struct byd_ts_t {
	struct input_dev *input;
	struct i2c_client *client;
	struct work_struct ts_work;
	struct workqueue_struct *ts_work_queue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend ts_early_suspend;
#endif
};

/*** Global variables *********************************************************/

/* Variable   : buf - buffer for touch
 * Accessed by: byd_ts_work()
 */
unsigned char buf[50]; //buf[BYD_TS_MT_BUFFER_SIZE+9];

/* Variable   : This_Client - i2c client
 * Defined in : byd_ts_probe()
 * Accessed by: byd_ts_exit(), byd_bfxxxx_ts_extn.c: byd_ts_ioctl(), byd_ts_read()
 */
struct i2c_client *This_Client;
/* Variable   : Irq_Enabled - flag indecating interrupt enabled
 * Accessed by: byd_ts_work(), byd_ts_irq_handler(), some functions for raw data
 */
int Irq_Enabled = 1;

// ----------------------------------------------------------------------------

/* Variable   : XY_res_out - resolution (X*Y) reported to upper layer, literally
 *                  LCD resolution, but Linux/Android accept TP resolution
 *              XY_res_ts  - TP resolution (X*Y), used when REPORT_?_RES are
 *                  presented
 * Defined in : byd_ts_probe()
 * Accessed by: byd_ts_work()
 */
#if defined(REPORT_X_RES) && defined(REPORT_Y_RES)
static int XY_res_ts[2] = {TS_X_RES, TS_Y_RES};          // initialized to TP res
static int XY_res_out[2] = {REPORT_X_RES, REPORT_Y_RES}; // initialized to LCD res
#else
static int XY_res_out[2] = {TS_X_RES, TS_Y_RES};         // initialized to TP res
#endif

/* Variable   : Byd_Ts_Keys - key definition
 * Defined in : byd_ts_probe()
 * Accessed by: byd_ts_register_input_device()
 */
 static unsigned char Byd_Ts_Keys[] = BYD_TS_KEYS;


/*******************************************************************************
* Function    : i2c_read_bytes
* Description : read data to buffer form i2c bus:
*               send i2c commands/address and then read data
*               (i2c_master_send i2c_master_recv)
* Parameters  : input: client, send_buf, send_len
                output: recv_buf, recv_len
* Return      : i2c data transfer status
*******************************************************************************/
int i2c_read_bytes(struct i2c_client *client,
	char *send_buf, int send_len, char *recv_buf, int recv_len)
{
	int err = 0;

	err = i2c_master_send(client, send_buf, send_len);
	if (err < 0) {
	    pr_err("%s: err #%d by i2c_master_send(), size:%d, send[0]:0x%02x\n", __FUNCTION__, err, send_len, send_buf[0]);
	    return err;
	}

	err = i2c_master_recv(client, recv_buf, recv_len);
	if (err < 0) {
	    pr_err("%s: err #%d returned by i2c_master_recv()\n", __FUNCTION__, err);
	}
	return err;
}

/*******************************************************************************
* Function    : i2c_write_reg
* Description : i2c write a 8 bit data to a given register
* In          : client: i2c client
*               addr: register number
*               para: data to be writen
* Return      : i2c data send status
*******************************************************************************/
static int byd_ts_write_reg(struct i2c_client *client, unsigned char addr, unsigned char para)
{
	unsigned char buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = i2c_master_send(client, buf, 2);
	if (ret < 0) {
		pr_err("%s: write register 0x%x failed! value:%d, errno:%d\n",
			__FUNCTION__, buf[0], buf[1], ret);
		return ret;
	}

	return 1;
}

////////////////// extension for additional functionalities ///////////////////
/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/

#ifdef CONFIG_HAS_EARLYSUSPEND
#define BYD_TS_SLEEP_REG	0x07
/*******************************************************************************
* Function    :  byd_ts_early_suspend
* Description :  put ts to sleep or shutdown mode
* In          :  handler
* Return      :  none
*******************************************************************************/
static void byd_ts_early_suspend(struct early_suspend *handler)
{
	struct byd_ts_t *ts;
	TS_DBG("%s\n", __FUNCTION__);
    int ret = 0;
	#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	if(atomic_read(&BF6862_PROXIMITY_ENABLE) == 1)
	{
		TS_DBG("byd_ts_early_suspend: BF6862_PROXIMITY_ENABLE = 1\n");	
		return;
	}
	  TS_DBG("byd_ts_early_suspend: BF6862_PROXIMITY_ENABLE\n");	
    #endif
	ts =  container_of(handler, struct byd_ts_t, ts_early_suspend);
  #ifdef BYD_GESTURE
    		TS_DBG("set_irq_wake(1) \n");	
		//gsl_irq_mode_change(This_Client,GSL_IRQ_HIGH);
		//set_irq_type(ts->client->irq, IRQF_TRIGGER_FALLING); //set irq to falling IRQF_TRIGGER_FALLING
		ret = byd_ts_write_reg(ts->client, BYD_TS_SLEEP_REG, 0x21);
		gesture_enable = 1;
		TS_DBG("<><><gesture_enable><><>>%d\n",ret);
		return;
    #else
		byd_ts_write_reg(ts->client, BYD_TS_SLEEP_REG, 0x00);
		mdelay(5);
    #endif
}

/*******************************************************************************
* Function    :  byd_ts_late_resume
* Description :  ts re-entry the normal mode and schedule the work, there need
                 to be a little time for ts ready
* In          :  handler
* Return      :  none
*******************************************************************************/
static void byd_ts_late_resume(struct early_suspend *handler)
{
	struct byd_ts_t *ts;
	int res = 0;
	TS_DBG("%s\n", __FUNCTION__);
	#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	if(atomic_read(&BF6862_PROXIMITY_ENABLE) == 1)
	{
		TS_DBG("byd_ts_late_resume: BF6862_PROXIMITY_ENABLE = 1\n");	
		return;
	}
	TS_DBG("byd_ts_late_resume: BF6862_PROXIMITY_ENABLE = 0\n");	
    #endif
	ts =  container_of(handler, struct byd_ts_t, ts_early_suspend);
//////////////////////////////////////////////////////////////////////
  
#ifdef BYD_GESTURE
	  if(gesture_enable == 1){	     	  
		 res =  byd_ts_write_reg(ts->client, BYD_TS_SLEEP_REG, 0x20);
  		gesture_enable = 0;
		mdelay(2);
   		TS_DBG("set_irq_wake(0) result = %d\n",res);	
	  }
#else	  
	byd_ts_write_reg(ts->client, BYD_TS_SLEEP_REG, 0x01);
	mdelay(3);
	byd_ts_write_reg(ts->client, BYD_TS_SLEEP_REG, 0x01); // *** qi ximing, for IC bug
	mdelay(3);
#endif
/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
  #ifdef CONFIG_SUPPORT_FIRMWARE_UPG
	{	static uint8_t burnt_time = 0;
		int ret = 0;
		
		if(burnt_time == 0) {
			ret = byd_ts_upgrade_firmware(ts->client);
			burnt_time ++ ;
			if (ret < 0) {
				pr_err("%s: error in firmware update, errno: %d\n", __FUNCTION__, ret);
				burnt_time = 0;	
			}

			msleep(20);
		}
	}
  #endif

///////////////////////////////////////////////////////////////////////////////
#if defined(CONFIG_REPORT_PROTOCOL_B) //&& !defined(CONFIG_REMOVE_POINT_STAY) //&& !defined(TS_DEBUG_MSG)
	{	int i;
		for (i = 0; i < BYD_TS_MT_NUMBER_POINTS; i++) {
			input_mt_slot(ts->input, i + 1);
			//input_mt_slot(ts->input, 1);  //jyy test
			//input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		}
		input_sync(ts->input);
		TS_DBG("ALL UP\n");
	}
#endif
/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
}

#endif // CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
int BF6862_Get_Face_Mode_Enable(void)
{
     return atomic_read(&BF6862_PROXIMITY_ENABLE);
}

void BF6862_CTP_Face_Mode_Switch(int enable)
{
    uint8_t proximity_onoff_state;

	if(enable==1)
	{
		TS_DBG("BF6862_CTP_Face_Mode_Switch on\n");
		atomic_set(&BF6862_PROXIMITY_ENABLE, 1);
		
		proximity_onoff_state = 0x03;

		atomic_set(&BF6862_PROXIMITY_STATE, 0);
	}
	else
	{	
		TS_DBG("BF6862_CTP_Face_Mode_Switch off\n");
		atomic_set(&BF6862_PROXIMITY_ENABLE, 0);
		
		proximity_onoff_state = 0x02;
		
		atomic_set(&BF6862_PROXIMITY_STATE, 0);
	}
	byd_ts_write_reg(This_Client, 0x07, proximity_onoff_state);
}

int BF6862_Get_Ctp_Face_Mode_State(void)
{
	return atomic_read(&BF6862_PROXIMITY_STATE);
}


static ssize_t proximity_enable_show(struct device* cd, struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	if(tpd_proximity_flag==1)
		//sprintf(buf, "Proximity Start\n");
		sprintf(buf, "1\n");
	else
		//sprintf(buf, "Proximity Stop\n");
		sprintf(buf, "0\n");
	ret = strlen(buf) + 1;
	return ret;
}

//static ssize_t rohm_proximity_store_psenable(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
static ssize_t proximity_enable_store(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	printk("byd_bf6862: <><><><><>>rohm_proximity_store_psenable\n");
	printk("byd_bf6862: <><><><><>>rohm_proximity_store_psenable\n");
	printk("byd_bf6862: <><><><><>>rohm_proximity_store_psenable\n");
	printk("byd_bf6862: <><><><><>>rohm_proximity_store_psenable\n");
	printk("byd_bf6862: <><><><><>>rohm_proximity_store_psenable\n");
	printk("byd_bf6862: <><><><><>>rohm_proximity_store_psenable\n");
	tpd_proximity_flag = on_off;
	BF6862_CTP_Face_Mode_Switch(on_off);
	return len;
}



static ssize_t rohm_proximity_show_version(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	sprintf(buf, "ROHM");	
	ret = strlen(buf) + 1;

	return ret;
}
//proximity_enable_show
static DEVICE_ATTR(enable,0777 , proximity_enable_show, proximity_enable_store);
static DEVICE_ATTR(version, 0777, rohm_proximity_show_version, NULL);

//added by gary
static struct attribute *rohm_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_version.attr,
	NULL
};

static struct attribute_group rohm_attribute_group = {
	.attrs = rohm_attributes
};



static long gtp_proximity_ioctl(struct i2c_client *client, unsigned int cmd, void *arg)
{

	ZYT_PRINTK("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case BF_IOCTL_PROX_ON:
		BF6862_CTP_Face_Mode_Switch(1);
		break;
	case BF_IOCTL_PROX_OFF:
		BF6862_CTP_Face_Mode_Switch(0);
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


#endif // CONFIG_HAS_EARLYSUSPEND

#ifdef CONFIG_TP_UPGRADE_SUPPORT
static ssize_t bf6862a_fwup_version(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	
	unsigned char version[] = {0, 0, 0, 0, 0, 0};
	int ret=0;
	ret = byd_ts_get_firmware_version(This_Client, version);
	if(ret<0)
		return sprintf(buf, "read version error\n");
	else
		return sprintf(buf, " bf%02x%02x TP:  %02x%02x-%.2x.%.2x\n",version[0], version[1], version[2], version[3], version[4], version[5]);

}

static ssize_t bf6862a_fwup_store(struct device *dev,
				   struct device_attribute *attr,  char *buf)
{
	int error;
	//ssize_t  fwup_state = 0;
	#if 1
		error = byd_ts_upgrade_firmware(This_Client);
		if(error < 0)
			return sprintf(buf, "byd_ts_upgrade_firmware error\n");
		else
			return sprintf(buf, "upgrade_firmware action complate\n");
	#endif
	return  sprintf(buf, "action complate");

}

static ssize_t bf6862a_fwup_reset(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char cmd_reset[] = I2C_RESET;
	int ret;
	ret = i2c_master_send(This_Client, cmd_reset, sizeof(cmd_reset));	
	if (ret < 0) {
		pr_err("%s: i2c read VERSION# error, errno: %d \n", __FUNCTION__, ret);
	}
	return sprintf(buf, "reset successful");

}
static ssize_t bf6862a_diag_show_rowdata(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
   	ssize_t  num_read_chars = 0;
	int ret,len,len_all,i=0,row=0,diff=0;
	int16_t diffdata;
	uint8_t rawbuf[1 + 38 * 2 + 38 * 2] = {1, 0};  
	uint16_t baseline[38]={0},rawdata[38]={0};
	unsigned char cmd_rawdata[] = I2C_READ_RAW_DATA;
	ret = i2c_master_send(This_Client, cmd_rawdata, sizeof(cmd_rawdata));
	if (ret < 0) {
		msleep(5);
		ret = i2c_master_send(This_Client, cmd_rawdata, sizeof(cmd_rawdata));
		if (ret < 0) {
			pr_err("%s: i2c_master_send RAWDATA COMMAND error, errno: %d\n", __FUNCTION__, ret);
			return sprintf(buf,"%s: i2c_master_send RAWDATA COMMAND error, errno: %d\n", __FUNCTION__, ret);
		}
	}
	
	ret = smbus_master_recv(This_Client, (char *)&rawbuf,  sizeof(rawbuf));
	//ret = mtk_smbus_master_recv(This_Client, (char *)&rawbuf,  sizeof(rawbuf));
	TS_DBG("rawbuf[0]:   %d, rawbuf[1]: %d  rawbuf[2]:   %d\n");
	//ret = i2c_master_recv(This_Client, (char *)&rawbuf,  sizeof(rawbuf));
	if (ret < 0) {
		pr_err("%s: i2c_master_recv RAWDATA error, errno: %d\n", __FUNCTION__, ret);
		return sprintf(buf,"%s:i2c_master_recv RAWDATA error, errno: %d\n", __FUNCTION__, ret);
	}
	printk("\nbyd6862a channel %d\n",rawbuf[0]);
	printk(" rawdata\n");
		len = rawbuf[0];
		len_all = 1 + len * 2 + len * 2;
	num_read_chars = sprintf(buf,"byd6862a channel :%u\n",rawbuf[0]);
	num_read_chars += sprintf(&(buf[num_read_chars]),"CH_num row data:");

            for(i=1;i<(2*len);i=i+2)
            {
           	rawdata[row]=rawbuf[i+1]|(rawbuf[i]<<8);

		if(rawdata[row]==0xff)
			printk("rawdata read error %d\n",i);

		printk("  %d",rawdata[row]);
		//num_read_chars += sprintf(&(buf[num_read_chars])," %5d",rawdata[row]);

		row++;

			if(i==(2*len-1))
			{
				printk("\n");
				//num_read_chars += sprintf(&(buf[num_read_chars]),"\n");
			}
		}

	//num_read_chars += sprintf(&(buf[num_read_chars]),"\n");
	printk(" \nbaseline\n");

	for(i=(2*len+1);i<(len_all-1);i=i+2)
            {
            	baseline[diff]=rawbuf[i+1]|(rawbuf[i]<<8);
           	if(baseline[diff]==0xff)
			printk("rawdata read error %d\n",i);

		printk("  %d",baseline[diff]);
		if(i==(3*len-1))
			printk("\n");

		diff++;

		}
		
	printk("\n diff");
	num_read_chars += sprintf(&(buf[num_read_chars]),"    base data:");
	num_read_chars += sprintf(&(buf[num_read_chars]),"    diff data:\n");
	for(i=0;i<len;i++)
	{
		diffdata=baseline[i]-rawdata[i];

		printk("  %d",diffdata);
		num_read_chars += sprintf(&(buf[num_read_chars]),"CH%2d    %5d        %5d        %5d\n", i+1, rawdata[i], baseline[i], diffdata);

		//if(i==len)
		//	num_read_chars += sprintf(&(buf[num_read_chars]),"\n");
	}
	num_read_chars += sprintf(&(buf[num_read_chars]),"\n");
	return num_read_chars;
}

//static DEVICE_ATTR(fwupdate, (S_IWUSR|S_IRUGO), bf6862a_fwup_show, bf6862a_fwup_store);
static DEVICE_ATTR(fwversion, 0777, bf6862a_fwup_version, NULL);
static DEVICE_ATTR(update, 0777, bf6862a_fwup_store, NULL);
static DEVICE_ATTR(reset, 0777, bf6862a_fwup_reset, NULL);
static DEVICE_ATTR(rowdata, 0777, bf6862a_diag_show_rowdata, NULL);
//added by gary
static struct attribute *fwup_attributes[] = {
	&dev_attr_fwversion.attr,
	&dev_attr_rowdata.attr,		
	&dev_attr_update.attr,
	&dev_attr_reset.attr,	
	NULL
};

static struct attribute_group fwup_attribute_group = {
	.attrs = fwup_attributes
};

#endif // CONFIG_HAS_EARLYSUSPEND

/*******************************************************************************
* Function    : Linux callback function
*******************************************************************************/
//#define BYD_TS_DEVICE		BYD_TS_NAME
#define BYD_TS_REGISTER_BASE	0x5c
#define BYD_TS_MT_POINT_REG	0x5d
#define BYD_TS_MT_TOUCH_REG	0x5c
#define BYD_TS_MT_TOUCH_MASK	0xF0
#define BYD_TS_MT_TOUCH_FLAG	0x80
#define BYD_TS_MT_KEY_FLAG	0x90
#define BYD_TS_MT_NPOINTS_MASK	0x0f
static void byd_ts_work(struct work_struct *work)
{
	int x, y, ret = 0;
	struct byd_ts_t *ts = container_of(work, struct byd_ts_t, ts_work);
	char cmd[1] = {BYD_TS_REGISTER_BASE}; //buf[BYD_TS_MT_BUFFER_SIZE] to global
	uint8_t touch_type;   //screen touch or key touch
	uint8_t touch_numb;
	uint8_t id;
	uint8_t i;
	static int Touch_key_down = 0;
	uint8_t gesture_numb;
#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
	u8 proximity_buf=0;
	int proximity_state = 0;
	static int pre_proximity_state = 0;
#endif

	//byd_ts_read_IC_para(ts->client);
	//TS_DBG("*** ISR WORK entered ***\n");
#if !defined(I2C_PACKET_SIZE) || (I2C_PACKET_SIZE > 32)
	ret = i2c_read_bytes(ts->client, cmd, 1, buf, 9); //BYD_TS_MT_BUFFER_SIZE);
	if (ret <= 0) {
		pr_err("%s: i2c read register error, errno: %d\n", __FUNCTION__, ret);
		goto out;
	}
#else
	ret = i2c_smbus_read_i2c_block_data(ts->client, BYD_TS_REGISTER_BASE, 5, buf);
	if (ret <= 0) {
		pr_err("%s: i2c read register 0x%02x error, errno: %d. please check protocol.\n",
			  __FUNCTION__, BYD_TS_REGISTER_BASE, ret);
		goto out;
	}
	touch_numb = 2; //buf[BYD_TS_MT_TOUCH_REG - BYD_TS_REGISTER_BASE]
			& BYD_TS_MT_NPOINTS_MASK; // buf[0x5c - 0x5c] & 0x0f;
	//if (touch_numb > BYD_TS_MT_NUMBER_POINTS) {
	////	pr_err("%s: error number of points: %d, please check ptotocol. \n", __FUNCTION__, touch_numb);
		goto out;
	//}
	for (i = 1; i < touch_numb; i++) { // BYD_TS_MT_NUMBER_POINTS
		id = i * 4 + 1; // id means index here
		ret = i2c_smbus_read_i2c_block_data(ts->client, BYD_TS_REGISTER_BASE + id, 4, &buf[id]);
	} 
#endif
	TS_DBG("-----buf[0-8]:  %x %x %x %x %x %x %x %x %x \n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8]);
if((buf[0]&0xf0) == 0x80)
{
	TS_DBG("byd_ts_read_coor: buf[0-4]:  %x %x %x %x \n",
		buf[0], buf[1], buf[2], buf[3]);
	byd_ts_read_coor(ts->client);
}
	/*       "CS: 0.TOUCH 1.X1H+Y1H 2.X1L 3.Y1L ...\n"
	         "MC: 0.TOUCH 1.ID+X1H  2.X1L 3.ST+Y1H 4.Y1L ...\n" */
	TS_DBG("-----buf[0-8]:  %x %x %x %x %x %x %x %x %x \n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8]);
#ifdef BYD_GESTURE
	if(gesture_enable == 1)
	{
		if((buf[0] == (0xa0 | buf[1])) && (buf[2] == 0x00) && (buf[3] == 0x0a) && (buf[4] == 0x00)){
			if((0x12 >= buf[1])  && (buf[1] > 0))
				{
			       gesture_numb = buf[1];
				}
			else{
				printk(" gesture identify error \n");
				goto out;
				}
				printk("=key_data=====gesture KEY = %d =====\n",gesture_numb);
			}
		else{
			printk(" gesture identify error \n");
			goto out;
			}
			input_report_key(ts->input, Byd_Ts_Gesture_Keys[gesture_numb -1], 1);
			input_mt_sync(ts->input);
			input_report_key(ts->input, Byd_Ts_Gesture_Keys[gesture_numb -1], 0);
			input_mt_sync(ts->input);
			input_sync(ts->input);
			goto out;
	}
	else
#endif
	{
#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
			if(atomic_read(&BF6862_PROXIMITY_ENABLE) == 1)
			{
				TS_DBG("BYD_WORK TP_PROXIMITY:buf[0] = %d, buf[1] = %d ,buf[2] = %d,\n", buf[0],buf[1],buf[2]);
				if ((buf[0]==0x0a) && (buf[1]==0x03) && (buf[2]==0x38) 
					&& (buf[3]==0x07))
				{
					proximity_buf = buf[4];
					TS_DBG("reg0x=%x\n", proximity_buf);
		
					if (proximity_buf==0x01)
					{
						atomic_set(&BF6862_PROXIMITY_STATE, 1);
					}
					else if (proximity_buf==0xff)
					{
						atomic_set(&BF6862_PROXIMITY_STATE, 0);
					}
		
					proximity_state = BF6862_Get_Ctp_Face_Mode_State();
				    printk("pre_proximity_state %dd<><>proximity_state %d\n",pre_proximity_state,proximity_state);
					if (pre_proximity_state != proximity_state)
					{
						pre_proximity_state = proximity_state;
		
						if (proximity_state){
							printk("<><><><><><><>FAR\n");
							printk("The thing is Close TP\n");
							input_report_abs(ts->input, ABS_DISTANCE, 0);
							input_sync(ts->input);
							}
						else{
							printk("<><><><><><><>NEAR\n");
							printk("The thing is Near TP\n");
							input_report_abs(ts->input, ABS_DISTANCE, 1);
							input_sync(ts->input);
							}
		
						input_sync(ts->input);
							
					}
					goto out;
				}
		
				
			}	
#endif

		touch_type = buf[BYD_TS_MT_TOUCH_REG - BYD_TS_REGISTER_BASE]
				& BYD_TS_MT_TOUCH_MASK; // buf[0x5c - 0x5c] & 0xF0

		if(touch_type != BYD_TS_MT_TOUCH_FLAG && touch_type != BYD_TS_MT_KEY_FLAG) {
	///////////////// parameter download protocol /////////////////////////////////
#if defined(CONFIG_SUPPORT_PARAMETER_UPG) || defined(CONFIG_SUPPORT_PARAMETER_FILE)
			//if(touch_type != 0 && touch_type != 0xf0) {
			if(touch_type == 0x50  // 0x55 is protocol for parameter download
				&& (buf[BYD_TS_MT_TOUCH_REG - BYD_TS_REGISTER_BASE + 1]
				& BYD_TS_MT_TOUCH_MASK) == 0x50  // touch_id
				&& (buf[BYD_TS_MT_TOUCH_REG - BYD_TS_REGISTER_BASE + 3]
				& BYD_TS_MT_TOUCH_MASK) == 0x50) {// touch_state
				printk("%s: parameters lost! download again...", __FUNCTION__);
				printk("(register #0-#3: %x, %x, %x, %x)\n", buf[0], buf[1], buf[2], buf[3]);
				//disable_irq(ts->client->irq);
				byd_ts_download(ts->client, -1); // -1 unknown module number
				goto out;
			}
#endif
	/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
			pr_err("%s: data error in registers! register #0-#3: %x, %x, %x, %x\n",
				__FUNCTION__, buf[0], buf[1], buf[2], buf[3]);
			goto out;
		}

		touch_numb = buf[BYD_TS_MT_TOUCH_REG - BYD_TS_REGISTER_BASE]
				& BYD_TS_MT_NPOINTS_MASK;
		if (touch_numb == 0) {
			if (Touch_key_down) {
					TS_DBG(" key %d up\n", Touch_key_down);
					input_report_key(ts->input, Byd_Ts_Keys[Touch_key_down - 1], 0);
			}
			Touch_key_down = 0;

		} // end of if touch_numb==0

		if(touch_type == BYD_TS_MT_TOUCH_FLAG) {
		  TS_DBG("#P=%d", touch_numb);
		  if (touch_numb == 0) {
#if (defined(CONFIG_ANDROID_4_ICS) && !defined(CONFIG_REPORT_PROTOCOL_B)) \
	    || defined(CONFIG_REPORT_PROTOCOL_A)
			TS_DBG(" (ALL UP)");
			//input_report_key(tpd->dev, BTN_TOUCH, 0);
			input_mt_sync(ts->input); //input_report_key(ts->input, BTN_TOUCH, 0);
			input_sync(ts->input);
#elif defined(CONFIG_REMOVE_POINT_STAY)	&& defined(CONFIG_REPORT_PROTOCOL_B)
			TS_DBG(" ALL UP (CONFIG_REMOVE_POINT_STAY enabled)\n");
			for (i = 0; i < BYD_TS_MT_NUMBER_POINTS; i++) {
				input_mt_slot(ts->input, i + 1);
				//input_mt_slot(ts->input, 1);  //jyy test
				//input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
				input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
			}
			input_sync(ts->input);
#endif

			TS_DBG("\n*** Warning: The TP driver is working in DEBUG mode. Please switch off TS_DEBUG_MSG in driver's .h file ***\n");
#ifdef CONFIG_DRIVER_BASIC_FUNCTION
			TS_DBG("*** Warning: The TP driver is basic version. Please switch off CONFIG_DRIVER_BASIC_FUNCTION in driver's .h file ***\n");
#endif
			goto out;
		  } // end of if touch_numb==0

			for (i = 0; i < touch_numb && i < BYD_TS_MT_NUMBER_POINTS; i++) { // i <= touch_numb &&
				uint8_t finger_size;
#ifdef CONFIG_FIRMWARE_PRE_VERSION_2_0
				x = ( ((buf[i * 3 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE] & 0xf0) << 4)
					   | buf[i * 3 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE + 1] );
				y = ( ((buf[i * 3 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE] & 0x0f) << 8)
					   | buf[i * 3 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE + 2] );
				finger_size = ((x == 0 && y == 0) /* || touch_numb == 0 */) ? 0 : BYD_TS_TOUCHSIZE;
#else
				uint8_t touch_state;
				id = (buf[i * 4 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE] & 0xf0) >> 4;
				touch_state = buf[i * 4 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE + 2] & 0xf0;
				finger_size = (touch_state == 0xC0 /* || touch_numb == 0 */) ? 0 : BYD_TS_TOUCHSIZE; // 0xC0 represents touch up
				//finger_size = ((x == 0 && y == 0) /* || touch_numb == 0 */) ? 0 : BYD_TS_TOUCHSIZE;
				x = ( ((buf[i * 4 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE] & 0x0f) << 8)
					   | buf[i * 4 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE + 1] );
				y = ( ((buf[i * 4 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE + 2] & 0x0f) << 8)
					   | buf[i * 4 + BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE + 3] );
				// dealing with error from IC
				if (id <= 0 || id > BYD_TS_MT_NUMBER_POINTS || (x == 0 && y == 0)) {
					pr_err(" %d[%u,%u]%d err", i+1, x, y, id);
					continue;
				}
#endif
				//TS_DBG(" %d[%u,%u]", i+1, x, y);
#if defined(CONFIG_ANDROID_4_ICS) || defined(CONFIG_REPORT_PROTOCOL_A)
				//if (i >= touch_numb) break;
				if (finger_size <= 0) {
		#ifdef CONFIG_REPORT_PROTOCOL_B
					TS_DBG(" %d(up)%d", i + 1, id);
					//input_mt_slot(ts->input, 1);  //jyy test
					input_mt_slot(ts->input, id);
					//input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
					input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		#else
					input_mt_sync(ts->input); //input_report_key(ts->input, BTN_TOUCH, 0);
		#endif
					continue;
				}
#else // for Android 2.3
				//if (i >= touch_numb) finger_size = 0;
#endif

#ifdef REPORT_XY_SWAPPED
				int tmp = y;
				y = x;
				x = tmp;
			#if defined(REPORT_X_RES) && defined(REPORT_Y_RES)
				// the precision of all variables must be greater than 3 bytes here
				x = (x * (XY_res_out[0] - 1)) / (XY_res_ts[1] - 1);
				y = (y * (XY_res_out[1] - 1)) / (XY_res_ts[0] - 1);
			#endif
#else
			#if defined(REPORT_X_RES) && defined(REPORT_Y_RES)
				x = (x * (XY_res_out[0] - 1)) / (XY_res_ts[0] - 1);
				y = (y * (XY_res_out[1] - 1)) / (XY_res_ts[1] - 1);
			#endif
#endif
#ifdef REPORT_X_REVERSED
				x = (XY_res_out[0] - 1) - x;
#endif
#ifdef REPORT_Y_REVERSED
				y = (XY_res_out[1] - 1) - y;
#endif
				// dealing with error from IC
				if (x < 0 || y < 0 || x >= XY_res_out[0] || y >= XY_res_out[1]) {
					TS_DBG(" %d[%u,%u] err", i+1, x, y);
					continue;
				}
				TS_DBG(" %d(%u,%u)", i + 1, x, y);
#ifdef CONFIG_ANDROID_4_ICS
		#ifdef CONFIG_REPORT_PROTOCOL_B
				TS_DBG("%d", id);
				input_mt_slot(ts->input, id);
				//input_mt_slot(ts->input, 1);  //jyy test
				//input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
				input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		#endif
#elif !defined(CONFIG_ANDROID_4_ICS) && !defined(CONFIG_REPORT_PROTOCOL_A)
				input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, finger_size);
			#if defined(TS_DEBUG_MSG)
				if (finger_size <= 0) TS_DBG("up");
			#endif
#endif
				input_report_abs(ts->input, ABS_MT_POSITION_X, x);
				input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
#ifndef CONFIG_REPORT_PROTOCOL_B
				input_mt_sync(ts->input);
#endif
			} // end of for touch_numb

			TS_DBG(" end\n");
		} else if(touch_type == BYD_TS_MT_KEY_FLAG) { // key tapped

	 		if (sizeof(Byd_Ts_Keys) > 0) {
				if(buf[BYD_TS_MT_POINT_REG - BYD_TS_REGISTER_BASE] == touch_numb * 2 - 1) {
					Touch_key_down = touch_numb;
				}
				if (Touch_key_down) 
					{
					TS_DBG("Key %d down,", touch_numb);
					input_report_key(ts->input, Byd_Ts_Keys[touch_numb - 1], 1);
				}
#ifndef CONFIG_REPORT_PROTOCOL_B
				input_mt_sync(ts->input);
#endif
			} else {
				pr_err("%s: the key 0x%x is not defined!\n", __FUNCTION__,
				  buf[BYD_TS_MT_TOUCH_REG - BYD_TS_REGISTER_BASE]);
				goto out;
			}
		}

		input_sync(ts->input);
	}
out:
	enable_irq(ts->client->irq);
	Irq_Enabled = 1;
}

/*******************************************************************************
* Function    : Linux callback function
*******************************************************************************/
static irqreturn_t byd_ts_irq_handler(int irq, void *dev_id)
{
	struct byd_ts_t *ts = (struct byd_ts_t *)dev_id;
	//TS_DBG("@@@@ %s entered, irq=%d, @@@@\n", __FUNCTION__, irq);
	Irq_Enabled = 0;
	disable_irq_nosync(ts->client->irq); // should NOT use disable_irq()
	if (!work_pending(&ts->ts_work)) {
		queue_work(ts->ts_work_queue,&ts->ts_work);
	} else { // qi ximing: this may cause dada lost??
		printk("*** BYD TS QUEUE: touch data pending - queue is full! ***\n");
	}

	return IRQ_HANDLED;
}


int baseline[22]={0};
int rawdata[22]={0};
int aver_baseline[22]={0};
int aver_rawdata[22]={0};
static ssize_t bf6862a_diag_show(void)	
{
	int ret,i=0,row=0,diff=0;
	int16_t diffdata;
	uint8_t rawbuf[1 + 22 * 2 + 22 * 2] = {1, 0};  
	
	unsigned char cmd_rawdata_1[3] = {0x0c, 0x0d, 0x06};
	unsigned char cmd_rawdata_2[3] = {0x0c, 0x0d, 0x014};
	unsigned char cmd_rawdata_3[3] = {0x0c, 0x0d, 0x017};	
	unsigned char cmd_rawdata[] = {0x0c, 0x0d, 0x03};
	TS_DBG("I2C_READ_RAW_DATA:  %x   %x   %x \n",cmd_rawdata[0], cmd_rawdata[1], cmd_rawdata[2]);
	//ret = i2c_master_write(This_Client, cmd_rawdata_3, 3);
	ret = i2c_master_send(This_Client, cmd_rawdata_3, 3);
	
	if (ret < 0) {
		msleep(5);
		ret = i2c_master_send(This_Client, cmd_rawdata_3, sizeof(cmd_rawdata_3));
		if (ret < 0) {
			pr_err("%s: i2c_master_send RAWDATA COMMAND error, errno: %d\n", __FUNCTION__, ret);
		}
	}	
	//msleep(10);
	//ret = i2c_master_write(This_Client, cmd_rawdata_1, 3);
	//msleep(10);
	//ret = i2c_master_write(This_Client, cmd_rawdata_2, 3);
	//msleep(100);
	//ret = i2c_master_write(This_Client, cmd_rawdata, sizeof(cmd_rawdata));

	ret = i2c_master_send(This_Client, cmd_rawdata_1, 3);
	msleep(10);
	ret = i2c_master_send(This_Client, cmd_rawdata_2, 3);
	msleep(100);
	ret = i2c_master_send(This_Client, cmd_rawdata, sizeof(cmd_rawdata));

	
	if (ret < 0) {
		msleep(5);
		ret = i2c_master_send(This_Client, cmd_rawdata, sizeof(cmd_rawdata));
		if (ret < 0) {
			pr_err("%s: i2c_master_send RAWDATA COMMAND error, errno: %d\n", __FUNCTION__, ret);
		}
	}

	//smbus_read_bytes(client, cmd, sizeof(cmd), buf, sizeof(buf));
	ret = smbus_master_recv(This_Client, (char *)&rawbuf, sizeof(rawbuf));
	if (ret < 0) {
		pr_err("%s: i2c_master_recv RAWDATA error, errno: %d\n", __FUNCTION__, ret);
	}
	TS_DBG("byd6862a channel %d\n",rawbuf[0]);
	TS_DBG(" rawdata\n");

            for(i=1;i<44;i=i+2)
            {
           	aver_rawdata[row] += rawbuf[i+1]|(rawbuf[i]<<8);
		rawdata[row]=rawbuf[i+1]|(rawbuf[i]<<8);
		if((rawbuf[i+1]|(rawbuf[i]<<8))==0xff)
			printk("rawdata read error %d\n",i);

		printk("  %d",rawdata[row]);

		row++;
		}

	printk(" \nbaseline\n");

	for(i=45;i<88;i=i+2)
            {
            	aver_baseline[diff] += rawbuf[i+1]|(rawbuf[i]<<8);
		baseline[diff]=rawbuf[i+1]|(rawbuf[i]<<8);
           	if((rawbuf[i+1]|(rawbuf[i]<<8))==0xff)
			printk("rawdata read error %d\n",i);

		printk("  %d",baseline[diff]);
		if(i==65)
			printk("\n");

		diff++;

		}
		
	printk("\n diff \n");
}

//int rawdata_average(struct device *dev, struct device_attribute *attr, char *buf)
int byd_rawdata_test(void)
{
	int i;
	unsigned char cmd_reset[3] = I2C_RESET;
	bf6862a_diag_show();
	bf6862a_diag_show();
	bf6862a_diag_show();	
        for(i=0;i<22;i++)
        {
       	aver_rawdata[i] = 0; //time;
		aver_baseline[i] = 0; //time;;
	}	
	for(i = 0; i < 8; i++)
	{
	TS_DBG("rawdata read time == %d\n",i);
	bf6862a_diag_show();
	}
	i2c_master_send(This_Client, cmd_reset, 3);
	//i2c_master_write(This_Client, cmd_reset, 3);
	msleep(10);
		TS_DBG("aver_rawdata :\n");
		TS_DBG("aver_rawdata    aver_baseline \n");
        for(i=0;i<22;i++)
        {
               aver_rawdata[i] = aver_rawdata[i]/8; //time;
		aver_baseline[i] = aver_baseline[i]/8; //time;;
		TS_DBG("     %d         %d    \n",aver_rawdata[i], aver_baseline[i]);
	}
		TS_DBG("\n");
        for(i=0;i<22;i++)
        { 
       	if(aver_rawdata[i] <5000)
       		{
       		pr_err("TP modules short");
			return -1;
       		}
		else if(aver_rawdata[i] > 15000)
		{
       		pr_err("TP modules open \n ");
			return -2;
		}
		else{
			printk("TP modules ok \n ");
			return 1;
		}
	}	
}

/*******************************************************************************
* Function    : byd_ts_register_input_device
* Description : register tp chip as a input device of Linux
* In          : client, i2c client
                input_dev, struct of Linux input device
* Return      : status
*******************************************************************************/
static int byd_ts_register_input_device(struct i2c_client *client, struct input_dev *input_dev)
{
	int i, err;

//////////////////// read screen parameters from chip /////////////////////////
#if defined(CONFIG_SUPPORT_RESOLUTION_ACQUISITION)
	int xy_res_out[2];
	err = byd_ts_get_resolution(client, xy_res_out);
	if (xy_res_out[0] < 1 || xy_res_out[1] < 1
	 || xy_res_out[0] > 2048 || xy_res_out[1] > 2048) {
		pr_err("%s: error resolution range: %d x %d !\n", __FUNCTION__,
			xy_res_out[0], xy_res_out[1]);
		err = -1;
	}

	if (err < 0) {
		pr_err("%s: CONFIG_SUPPORT_RESOLUTION_ACQUISITION may not supported, "
				"default to %d x %d.\n", __FUNCTION__, TS_X_RES, TS_Y_RES);
	} else {
		TS_DBG("%s: resolution get from TP: %d x %d (ret=%d)\n", __FUNCTION__,
			xy_res_out[0], xy_res_out[1], err);
	#if defined(REPORT_X_RES) && defined(REPORT_Y_RES)
		XY_res_ts[0] = xy_res_out[0];   // tp res = tp res
		XY_res_ts[1] = xy_res_out[1];
	#else
		XY_res_out[0] =  xy_res_out[0]; // report res = tp res
		XY_res_out[1] =  xy_res_out[1];
	#endif
	}
#endif
/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/

#ifdef REPORT_XY_SWAPPED
	i = XY_res_out[1];
	XY_res_out[1] = XY_res_out[0];
	XY_res_out[0] = i;
#endif
	printk("%s: resolution reported: %d x %d\n", __FUNCTION__,
			XY_res_out[0], XY_res_out[1]);

	input_dev->name = BYD_TS_NAME;
	//input_dev->phys  = BYD_TS_DEVICE;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = BYD_TS_DRIVER_VERSION_CODE;

	set_bit(EV_SYN, input_dev->evbit); // can be omitted?
	set_bit(EV_KEY, input_dev->evbit); // for button only?
	set_bit(EV_ABS, input_dev->evbit); // must
	//for proximity
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	//for lightsensor
	input_set_abs_params(input_dev, ABS_MISC, 0, 100001, 0, 0);
#ifdef CONFIG_ANDROID_4_ICS
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit); // tells the device is a touchscreen
	#ifdef CONFIG_REPORT_PROTOCOL_B
		//set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);
		input_mt_init_slots(input_dev, BYD_TS_MT_NUMBER_POINTS + 1); // 16
		//input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, BYD_TS_MT_NUMBER_POINTS, 0, 0); // 16
	#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, XY_res_out[0] - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, XY_res_out[1] - 1, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, XY_res_out[0], 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, XY_res_out[1], 0, 0);
  #ifndef CONFIG_REPORT_PROTOCOL_A
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, BYD_TS_TOUCHSIZE, 0, 0);
  #endif
#endif

	for (i = 0; i < sizeof(Byd_Ts_Keys); i++) {
		set_bit(Byd_Ts_Keys[i],  input_dev->keybit);
		input_set_capability(input_dev, EV_KEY, Byd_Ts_Keys[i]);
		}
#ifdef BYD_GESTURE
	//for (i = 0; i < sizeof(Byd_Ts_Gesture_Keys); i++) {
	//	input_set_capability(input_dev, EV_KEY, Byd_Ts_Gesture_Keys[i]);
	//	}
		__set_bit(KEY_POWER,  input_dev->keybit);
		input_set_capability(input_dev, EV_KEY, KEY_L);
		input_set_capability(input_dev, EV_KEY, KEY_R);
		input_set_capability(input_dev, EV_KEY, KEY_U);
		input_set_capability(input_dev, EV_KEY, KEY_D);
		input_set_capability(input_dev, EV_KEY, KEY_O);
		input_set_capability(input_dev, EV_KEY, KEY_W);
		input_set_capability(input_dev, EV_KEY, KEY_M);
		input_set_capability(input_dev, EV_KEY, KEY_C);
		input_set_capability(input_dev, EV_KEY, KEY_Z);
		input_set_capability(input_dev, EV_KEY, KEY_POWER);	
#endif	
	
	return input_register_device(input_dev);
}

static int check_ctp_chip(void)
{
	ctp_lock_mutex();
	tp_device_id(0x6862);
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

#ifdef CONFIG_OF
static struct bf6862a_ts_platform_data *bf6862a_ts_parse_dt(struct device *dev)
{
	struct bf6862a_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct ft5x0x_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", &pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
#ifdef ZCFG_LCM_WIDTH
	pdata->TP_MAX_X = ZCFG_LCM_WIDTH;
#else
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
#endif
#ifdef ZCFG_LCM_HEIGHT
	pdata->TP_MAX_Y = ZCFG_LCM_HEIGHT;
#else
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}
#endif

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

/*******************************************************************************
* Include     : module file for platform specific settings
* Interface   : int byd_ts_platform_init(void)
*               in:  none
*               out: status or irq number if device is added on the fly
* Description : the function is usually used for interrupt pin or I2C pin
                configration during initialization of TP driver.
*******************************************************************************/
//#include "byd_ts_platform_init.c" // *** it is MOVED INTO byd_bfxxxx_ts.h ***

/*******************************************************************************
* Function    : Linux callback function
*******************************************************************************/
static int byd_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i, ret, err = 0;
	struct input_dev *input_dev;
	struct byd_ts_t *byd_ts = NULL;
	struct bf6862a_ts_platform_data	*pdata = client->dev.platform_data;
#ifdef CONFIG_OF
	struct device_node		*np = client->dev.of_node;
#endif
#ifndef BUS_NUM
  #ifndef CONFIG_NON_BYD_DRIVER
	ret = byd_ts_platform_init(client);
	if (ret < 0) err = ret;
  #endif
  #ifdef BYD_TS_SLAVE_ADDRESS
 // (Re)defining I2C address if not given or given in the "client->addr" is not correct
	client->addr = BYD_TS_SLAVE_ADDRESS; // |= I2C_ENEXT_FLAG; // |= I2C_HS_FLAG;
  #endif
#endif

	TS_DBG("%s: start. found device: name:%s, deviceId:%s, IRQ:%d, addr:0x%x\n",
		   __FUNCTION__, client->name, id->name, client->irq, client->addr);
     if(tp_device_id(0)!=0)
	{
		printk("CTP(0x%x)Exist!", tp_device_id(0));
		return -ENODEV;
	}
	#ifdef CONFIG_OF
	if (np && !pdata){
		pdata = bf6862a_ts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: functionality check failed\n", __FUNCTION__);
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

////////////////////// firmware downloading ///////////////////////////////////
//  chip code downloading (available only if rootfs is started at this moment)
#if defined(CONFIG_SUPPORT_FIRMWARE_UPG)
	byd_ts_upgrade_firmware(client);
#endif
/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
//////////////////// read firmware version from chip //////////////////////////
#ifdef CONFIG_SUPPORT_VERSION_ACQUISITION
	{
	unsigned char version[] = {0, 0, 0, 0, 0, 0};
	for(i = 0; i < 3; i++)
	        {
		ret = byd_ts_get_firmware_version(client, version);
		mdelay(2);
		}
	if (ret < 0) err = ret;
	printk("%s: TP firmware version: %02x%02x-%02x%02x-%.2x%.2x\n", __FUNCTION__, // ##-PROJ.CO.PA
		version[0], version[1], version[2], version[3], version[4], version[5]);
		
	if(!((version[0]==0x68)&&((version[1]==0x63)||(version[1]==0x62))))
		goto exit_check_functionality_failed;		
	}
#endif
/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/

///////////////////// proximity sensor support ///////////////////////////////////

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
		// register input device 
		pro_input_dev = input_allocate_device();
		if (!pro_input_dev) 
		{
			printk("%s: input allocate device failed\n", __func__);
		}	
		pro_input_dev->name = AL3006_PLS_INPUT_DEV;
		pro_input_dev->phys  = AL3006_PLS_INPUT_DEV;
		pro_input_dev->id.bustype = BUS_I2C;
		pro_input_dev->dev.parent = &client->dev;
		pro_input_dev->id.vendor = 0x0001;
		pro_input_dev->id.product = 0x0001;
		pro_input_dev->id.version = 0x0010;
	
		__set_bit(EV_ABS, pro_input_dev->evbit);	
		//for proximity
		input_set_abs_params(pro_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
		//for lightsensor
		input_set_abs_params(pro_input_dev, ABS_MISC, 0, 100001, 0, 0);
	
		err = input_register_device(pro_input_dev);
		if (err < 0)
		{
			printk("%s: input device regist failed\n", __func__);
		}
		//setup_timer(&ps_timer, tpd_ps_handler, 0 );
		//wake_lock_init(&pls_delayed_work_wake_lock, WAKE_LOCK_SUSPEND, "prox_delayed_work");
	//	ret = sysfs_create_group(&pro_input_dev->dev.kobj, &rohm_attribute_group);//added by gary

		ret = misc_register(&gtp_proximity_misc);
		if (ret < 0)
		{
		pr_err("%s: could not register misc device\n", __func__);
		goto exit_request_memory_failed;
		}
#endif

#if defined(CONFIG_TP_UPGRADE_SUPPORT)
		// register input device 
		fwup_input_dev = input_allocate_device();
		if (!fwup_input_dev) 
		{
			printk("%s: input allocate device failed\n", __func__);
		}	
		fwup_input_dev->name ="bfxxxx_fwup";  // BYD_TS_NAME;
		fwup_input_dev->phys  = "bfxxxx_fwup"; //BYD_TS_NAME;  
		fwup_input_dev->id.bustype = BUS_I2C;
		fwup_input_dev->dev.parent = &client->dev;
		fwup_input_dev->id.vendor = 0x0001;
		fwup_input_dev->id.product = 0x0001;
		fwup_input_dev->id.version = 0x0010;
		//__set_bit(EV_ABS, fwup_input_dev->evbit);	
		//for proximity
		//input_set_abs_params(fwup_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
		//for lightsensor
		//input_set_abs_params(fwup_input_dev, ABS_MISC, 0, 100001, 0, 0);
	
		err = input_register_device(fwup_input_dev);
		if (err < 0)
		{
			printk("%s: input device regist failed\n", __func__);
		}

		fwup_kobj = kobject_create_and_add("bfxxxx_ts_fwup", NULL);
		if (fwup_kobj)

		ret = sysfs_create_group(fwup_kobj, &fwup_attribute_group);//added by gary
			if (ret < 0) 
		{
			printk("%s:sysfs_create_group  failed\n", __func__);
		}	



#endif


/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
	byd_ts = kzalloc(sizeof(struct byd_ts_t), GFP_KERNEL);
	if (!byd_ts) {
		pr_err("%s: request memory failed\n", __FUNCTION__);
		ret= -ENOMEM;
		goto exit_request_memory_failed;
	}

	This_Client = client;
	byd_ts->client = client;
	i2c_set_clientdata(client, byd_ts);

	// input device
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: input allocate device failed\n", __FUNCTION__);
		ret = -ENOMEM;
		goto exit_input_dev_allocate_failed;
	}

	ret = byd_ts_register_input_device(client, input_dev);
	if (ret < 0) {
		pr_err("%s: input device regist failed\n", __FUNCTION__);
		goto exit_input_register_failed;
	}
	byd_ts_read_IC_para(client);
	byd_ts->input = input_dev;

	err = check_ctp_chip();
	if (err<0) {
		pr_err("bf6862a_ts_probe failed(%d): failed to check_ctp_chip\n", err);
		goto exit_check_ctp_chip;
	}
	  
	//create work queue
	INIT_WORK(&byd_ts->ts_work, byd_ts_work);
	byd_ts->ts_work_queue= create_singlethread_workqueue("byd_ts");

//#ifdef CONFIG_HAS_EARLYSUSPEND
	//register early suspend
	byd_ts->ts_early_suspend.suspend = byd_ts_early_suspend;
	byd_ts->ts_early_suspend.resume = byd_ts_late_resume;
	register_early_suspend(&byd_ts->ts_early_suspend);
//#endif

//////////////////// raw data can be passed up for deguging ///////////////////
#if defined(CONFIG_SUPPORT_RAW_DATA_ACQUISITION) \
 || defined(CONFIG_SUPPORT_PARAMETER_FILE)
	ret = misc_register(&byd_ts_device);
	if (ret) {
		pr_err("%s: misc device register (for tools) failed! \n", __FUNCTION__);
		err = ret; //		goto irq_request_err;
	}
#endif
/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/

	//register irq
	if(client->irq > 0) {
		ret =  request_irq(client->irq, byd_ts_irq_handler, EINT_TRIGGER_TYPE | IRQF_NO_SUSPEND, client->name, byd_ts); // | IRQF_SHARED
		if (ret < 0) {
			pr_err("%s: IRQ setup failed! errno: %d\n", __FUNCTION__, ret);
			goto irq_request_err;
		}
	} else {
		pr_err("%s: IRQ setup failed! client-irq = %d\n", __FUNCTION__, client->irq);
		goto irq_request_err;
	}
	if (err >= 0) {
		printk("%s: *** Probe Success! ***\n%s\n", __FUNCTION__,
			   BYD_TS_DRIVER_DESCRIPTION);
	} else {
		printk("%s: *** Probe finished with some errors! ***\n%s\n", __FUNCTION__,
			   BYD_TS_DRIVER_DESCRIPTION);
	}
	{
		extern void zyt_info_s2(char* s1,char* s2);
		zyt_info_s2("[TP] : ","BF6862A");
      }
	 printk("bf6862 has run overed!!");
	return 0;

irq_request_err:
	destroy_workqueue(byd_ts->ts_work_queue);
exit_input_register_failed:
	input_free_device(input_dev);
exit_input_dev_allocate_failed:
	kfree(byd_ts);
exit_request_memory_failed:
exit_check_ctp_chip:
	remove_ctp_chip();
exit_chip_check_failed:
	gpio_free(pdata->irq_gpio_number);
	gpio_free(pdata->reset_gpio_number);
	kfree(byd_ts);
	byd_ts = NULL;
	i2c_set_clientdata(client, NULL);
exit_check_functionality_failed:
	gpio_free(GPIO_TOUCH_IRQ);
	gpio_free(GPIO_TOUCH_RESET);
	printk("%s: Probe Fail!\n",__FUNCTION__);
	return ret;
exit_alloc_platform_data_failed:
	return err;
}


#ifdef BUS_NUM

/*******************************************************************************
* Function    :  byd_ts_i2c_attach_device
* Description :  add a i2c device to i2c bus (instantiate the i2c device)
* In          :  bus_num, the number of the I2C bus segment, on which the device
                 is attached, on the target board
                 irq, interrupt request number
* Return      :  status
*******************************************************************************/
int byd_ts_i2c_attach_device(int bus_num, int irq)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int err;
	struct i2c_board_info board_info = {
		I2C_BOARD_INFO(BYD_TS_NAME, BYD_TS_SLAVE_ADDRESS),   /*** slave addr, 0x0a/0x14 BF6862, 0x2c/0x58 BF6711a, 0x34 BF6712b ***/
		.irq = irq
	};

	TS_DBG("%s: %s, i2c_bus_num:%d, IRQ:%d, addr:0x%x\n",
		     __FUNCTION__, board_info.type, bus_num, irq, board_info.addr);

	//i2c_register_board_info(bus_num, &board_info, 1);

	adapter = i2c_get_adapter(bus_num);
	if (!adapter) {
		printk("%s: can't get i2c adapter, bus number: %d\n", __FUNCTION__, bus_num);
		err = -ENODEV;
		goto err_driver;
	}

	client = i2c_new_device(adapter, &board_info);
	if (!client) {
		printk("%s:  can't add i2c device at 0x%x\n",
			__FUNCTION__, (unsigned int)board_info.addr);
		err = -ENODEV;
		goto err_driver;
	}

	i2c_put_adapter(adapter);

	return 0;

err_driver:
	return err;
}
#endif // ifdef BUS_NUM


/*******************************************************************************
* Function    : Linux callback function
*******************************************************************************/
static int byd_ts_remove(struct i2c_client *client)
{
	struct byd_ts_t *byd_ts = i2c_get_clientdata(client);

	TS_DBG("%s\n", __FUNCTION__);

	//remove queue
	flush_workqueue(byd_ts->ts_work_queue); // cancel_work_sync(&byd_ts->work);
	destroy_workqueue(byd_ts->ts_work_queue);
	remove_ctp_chip();
	i2c_set_clientdata(client, NULL);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&byd_ts->ts_early_suspend);
#endif
	input_unregister_device(byd_ts->input);
	input_free_device(byd_ts->input);
	free_irq(client->irq, byd_ts);
//////////////////// raw data functionality removing //////////////////////////
#if defined(CONFIG_SUPPORT_RAW_DATA_ACQUISITION) \
 || defined(CONFIG_SUPPORT_PARAMETER_FILE)
	misc_deregister(&byd_ts_device);
#endif
#ifdef CONFIG_TP_UPGRADE_SUPPORT	
	sysfs_remove_group(fwup_kobj, &fwup_attribute_group);//remove "/sys/bfxxxx_ts_fwup"
#endif

/*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
	kfree(byd_ts);

	return 0;
}

/*******************************************************************************
* Global Variable    : TS driver object
*******************************************************************************/
static const struct i2c_device_id byd_ts_id[] = { // devices supported by this driver
	{ BYD_TS_NAME, 0 }, // i2c_divice_id->name is same as i2c_client->name, must match i2c_board_info->type
	{ }
};
MODULE_DEVICE_TABLE(i2c, byd_ts_id);

static const struct of_device_id byd_of_match[] = {
	{ .compatible = "Bf6862a,bf6862a_ts", },
	{ }
};
MODULE_DEVICE_TABLE(of, byd_of_match);

#define BYD_TS_DRIVER_NAME BYD_TS_NAME
static struct i2c_driver byd_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = BYD_TS_DRIVER_NAME, // driver name can be found under /sys/bus/i2c/drivers/
		.of_match_table = byd_of_match,
	},

	.probe      = byd_ts_probe,
	.remove     = byd_ts_remove,
	.id_table = byd_ts_id, // one driver can support many devices (clients)
};

/*******************************************************************************
* Function    : Linux callback function
*******************************************************************************/
static int __init byd_ts_init(void)
{
	int ret;
	printk("********************************\n");
	printk("* init BYD touch screen driver  \n");
#if defined(CONFIG_CTS01)
	printk("*     %s (BF685X)\n", BYD_TS_NAME);
#elif defined(CONFIG_CTS02)
	printk("*     %s (BF686X)\n", BYD_TS_NAME);
#endif
	printk("********************************\n");

       if(tp_device_id(0)!=0)
	{
		printk("CTP(0x%x)Exist!", tp_device_id(0));
		return -ENODEV;
	}
	   
#ifdef BUS_NUM
	ret = byd_ts_platform_init(NULL);
	if (ret <= 0) { // ret is irq here
		pr_err("%s: can't add dirver %s, irq <= 0\n", __FUNCTION__, BYD_TS_NAME);
		return -ENODEV;
	}
	ret = byd_ts_i2c_attach_device(BUS_NUM, ret);
	if (ret != 0) {
		//pr_err("%s: can't attach i2c device to bus %d\n", __FUNCTION__, BUS_NUM);
		return -ENODEV;
	}
#endif

     ret = i2c_add_driver(&byd_ts_driver);
	if (ret != 0) {
		printk("<><><><><><><>%s: can't add i2c driver\n", __FUNCTION__);
		return -ENODEV;
	}else
	printk("<><><><<<><><><><%s:add i2c driver OK \n", __FUNCTION__);
	printk("The byd_tp has run overed!!");
	return 0;
}

/*******************************************************************************
* Function    : Linux callback function
*******************************************************************************/
static void __exit byd_ts_exit(void)
{
	TS_DBG("%s: addr=0x%x; i2c_name=%s",__FUNCTION__, This_Client->addr, This_Client->name);
#ifdef BUS_NUM
	i2c_unregister_device(This_Client);
#endif
	i2c_del_driver(&byd_ts_driver);
}

module_init(byd_ts_init);
module_exit(byd_ts_exit);

MODULE_AUTHOR("Simon Chee <qi.ximing@byd.com>");
MODULE_DESCRIPTION(BYD_TS_DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");

/*
 * part 2:
 * externt code
 * Created:	     2014-11-19
 * Depend on:    extern.c
 * Description:  BYD TouchScreen IC driver for Android
*/
//##############################################################################

#include <linux/gpio.h>       // gpio_get_value()
#include <linux/fs.h>
#include <asm/uaccess.h>      // get_fs(), set_fs()
#include <linux/miscdevice.h> // miscdevice, MISC_DYNAMIC_MINOR

// --------------- compile independently --------------------------------------
#include <linux/module.h>
#include <linux/interrupt.h> // disable_irq(), disable_irq_nosync()
#include <linux/i2c.h>       // i2c_client, i2c_master_send(), i2c_transfer(), I2C_M_RD
#include <linux/delay.h>     // mdelay()
#include <linux/slab.h>      // kmalloc()

//#include "byd_bfxxxx_ts.h"

#ifdef CONFIG_CTS02
int byd_ts_upgrade_firmware(struct i2c_client *client);
#endif
extern struct i2c_client *This_Client;

#ifdef CONFIG_NON_BYD_DRIVER

#ifdef TS_DEBUG_MSG
  #define TS_DBG(format, ...) \
	printk(format, ##__VA_ARGS__)
  #define PR_BUF(pr_msg, buf, len) \
  { \
	int _i_; \
	TS_DBG("%s: %s", __FUNCTION__, pr_msg); \
	for (_i_ = 0; _i_ < len; _i_++) { \
		if(_i_%16 == 0) TS_DBG("\n"); \
		/* if(_i_%16 == 0) TS_DBG("\n%02x ", _i_); */ \
		TS_DBG(" %02x", *(buf + _i_)); \
	} \
	TS_DBG("\n"); \
  }
#else
  #define TS_DBG(format, ...)
  #define PR_BUF(pr_msg, buf, len)
#endif

#ifndef CONFIG_DRIVER_BASIC_FUNCTION
	// Acquiring firmware version
	  #define CONFIG_SUPPORT_VERSION_ACQUISITION
	// Resolution acquiring (from chip)
	  #define CONFIG_SUPPORT_RESOLUTION_ACQUISITION
	  //#define CONFIG_SUPPORT_MODULE_ID /*** software module ID ***/
	#ifdef CONFIG_CTS02
	// On-line update for firmware and/or parameter (flash burning)
	  #define CONFIG_SUPPORT_FIRMWARE_UPG
	#endif
	// Raw data acquiring
	  #define CONFIG_SUPPORT_RAW_DATA_ACQUISITION
#endif

#endif // CONFIG_NON_BYD_DRIVER
// ------------------------------------------------------------------------- //

/*******************************************************************************
* Function    : smbus_master_recv
* Description : receive a buffer of data from i2c bus: -- FOR PARA OR RAW DATA
*               encapsulation of i2c_smbus_read_i2c_block_data for size limited
*               receiving
*               the data may be break into none-addressable packets and received
*               one by one 
* Parameters  : input: client
*               output: buf, len
* Return      : the length of bytes received or transmit status
*******************************************************************************/
#define START_ADDR	0
static int smbus_master_recv(struct i2c_client *client, char *buf, int len)
{
#if !defined(I2C_PACKET_SIZE) || (I2C_PACKET_SIZE > 32)
	return i2c_master_recv(client, buf, len);
#else
	int j, packet_number, leftover, start = 0, i = 0;
	unsigned char packet[I2C_PACKET_SIZE]= {0};

	/////////////////////////////////////////////
	if (len <= I2C_PACKET_SIZE) {
		return i2c_master_recv(client, buf, len);
	}
	/////////////////////////////////////////////
	leftover = len % I2C_PACKET_SIZE;
	packet_number = len / I2C_PACKET_SIZE + (leftover > 0 ? 1 : 0);
	for (j = 0; j < packet_number; j++) {
		int packet_len, ret;

	    start = j * I2C_PACKET_SIZE;
		packet_len = (leftover > 0 && j == packet_number - 1) ? leftover : I2C_PACKET_SIZE;
		TS_DBG("offset=0x%02x, packet_size=%d\n", start + START_ADDR, packet_len);
		// Note: 0xda is supposed to be the offset of parameter data, but currently this constant should be given instead
		//ret = gsl_read_interface(client, 0xda, packet, packet_len);
		ret = i2c_smbus_read_i2c_block_data(client, 0xda, packet_len, packet);
		if (ret < 0) {
			pr_err("%s: err #%d returned by i2c_smbus_read_i2c_block_data()\n", __FUNCTION__, ret);
			return ret;
		}

		for (i = 0; i < ret; i++) { // i < packet_len
			buf[start + i] = packet[i];
        }
	}
	TS_DBG("%s: I2C_PACKET_SIZE:%d, total_packets:%d, last_packet_size:%d\n", __FUNCTION__, I2C_PACKET_SIZE, packet_number, i);
	PR_BUF("last packet:", packet, i);
	return start + i;
#endif
}

static int mtk_smbus_master_recv(struct i2c_client *client, char *buf, int len)
{
	int j, packet_number, leftover, start = 0, i = 0;
	unsigned char packet[8]= {0};

	/////////////////////////////////////////////
	if (len <= 8) {
		return i2c_master_recv(client, buf, len);
	}
	/////////////////////////////////////////////
	leftover = len % 8;
	packet_number = len / 8 + (leftover > 0 ? 1 : 0);
	for (j = 0; j < packet_number; j++) {
		int packet_len, ret;

	    start = j * 8;
		packet_len = (leftover > 0 && j == packet_number - 1) ? leftover : 8;
		TS_DBG("offset=0x%02x, packet_size=%d\n", start + 0, packet_len);
		// Note: 0xda is supposed to be the offset of parameter data, but currently this constant should be given instead
		//ret = gsl_read_interface(client, 0xda, packet, packet_len);
		ret = i2c_smbus_read_i2c_block_data(client, 0xda, packet_len, packet);
		//ret = i2c_master_recv(client, packet, packet_len);
		if (ret < 0) {
			pr_err("%s: err #%d returned by i2c_smbus_read_i2c_block_data()\n", __FUNCTION__, ret);
			return ret;
		}

		for (i = 0; i < ret; i++) { // i < packet_len
			buf[start + i] = packet[i];
        }
	}
	TS_DBG("%s: total_packets:%d, last_packet_size:%d\n", __FUNCTION__, packet_number, i);
	PR_BUF("last packet:", packet, i);
	return start + i;

}

/*******************************************************************************
* Function    : smbus_read_bytes
* Description : read data to buffer form i2c bus:
*               send i2c commands/address and then read data
*               (i2c_master_send i2c_master_recv)
* Parameters  : input: client, send_buf, send_len
                output: recv_buf, recv_len
* Return      : length of bytes read or i2c data transfer status
*******************************************************************************/
static int smbus_read_bytes(struct i2c_client *client,
	char *send_buf, int send_len, char *recv_buf, int recv_len)
{
	int ret;

	PR_BUF("i2c send:", send_buf, send_len);
	ret = i2c_smbus_write_i2c_block_data(client, send_buf[0], send_len - 1, send_buf + 1);
	if (ret < 0) {
	    pr_err("%s: err #%d returned by i2c_smbus_write_i2c_block_data()\n", __FUNCTION__, ret);
	    return ret;
	}

	ret = smbus_master_recv(client, recv_buf, recv_len); // SMBUS, recv_len <= 32
	if (ret < 0) {
	    pr_err("%s: err #%d returned by i2c_master_recv()\n", __FUNCTION__, ret);
	}
	TS_DBG("size required/received: %d/%d\n", recv_len, ret);
	return ret;
}

#ifdef CONFIG_SUPPORT_MODULE_ID
/*******************************************************************************
* Function    :  byd_ts_get_module_number
* Description :  get module id for a perticular TP.
* Parameters  :  client, - i2c client
* Return      :  module number or status
*******************************************************************************/
int byd_ts_get_module_number(struct i2c_client *client)
{
	int i, ret, module_num = 0;
	unsigned char module = 0x00, module1 = 0xff;
	for (i = 0; i < 3; i++) { // try 3 times
		unsigned char version[] = {0, 0, 0, 0, 0, 0};
		ret = byd_ts_get_firmware_version(client, version);
		module = version[3];
		if (ret <= 0) {
			pr_err("%s: i2c read MODULE ID error, errno: %d\n", __FUNCTION__, ret);
		} else if (module == module1) {
			module_num = (int)((module >> 4) | ((module & 0x0C) << 2));
			break;
		} else {
			module1 = module;
		}
	}
	if (ret <= 0 || i > 3) {
		pr_err("%s: module_id may not correct\n", __FUNCTION__);
		return ret;
	}
	if (module_num < 1 || module_num > 99) {
		printk("%s: MODULE ID 0x%x is not correct.\n", __FUNCTION__, module);
		//printk("module number default to 1.\n");
		module_num = 0;
	}
	if((module & 0x03) == 0x03) {
		printk("%s: the parameters already in IC's flash, they may be overrided. \n", __FUNCTION__);
	/* Deleted for COF, in order to enable parameter download after mass production
		printk("%s: parameter download aborted! \n", __FUNCTION__);
		return -EFAULT;
	*/
	}
	return module_num;
}
#endif // CONFIG_SUPPORT_MODULE_ID


#if defined(CONFIG_SUPPORT_FIRMWARE_UPG) || defined(CONFIG_SUPPORT_PARAMETER_FILE)

/*******************************************************************************
* Function    : byd_ts_read_file
* Description : read data to buffer form file
* In          : filepath, buffer, len, offset
* Return      : length of data read or negative value in the case of error
                -EIO:    I/O error
                -ENOENT: No file found
*******************************************************************************/
int byd_ts_read_file(char *p_path, char *buf, ssize_t len, int offset)
{
	struct file *fd;
	int ret_len = -EIO;
	char file_path[64];
	char *p_file;

	p_file = strrchr(p_path, '/');  // file name without directory
	if (!p_file) {
		p_file = p_path;
	} else {
		p_file++;
	}

	strcpy(file_path, "/mnt/sdcard/");
	strcat(file_path, p_file);
	fd = filp_open(file_path, O_RDONLY, 0);
	if (IS_ERR(fd)) {
		TS_DBG("BYD TS: Warning: %s is not found or SD card not exist. \n", file_path);
		fd = filp_open(p_path, O_RDONLY, 0);
		if (!IS_ERR(fd)) {
			strcpy(file_path, p_path);
			goto alternate;
		}
		TS_DBG("BYD TS: Warning: %s is not found. \n", p_path);

#ifdef PARA_ALTERNATE_DIR
		strcpy(file_path, PARA_ALTERNATE_DIR);
		strcat(file_path, "/");
		strcat(file_path, p_file);
		fd = filp_open(file_path, O_RDONLY, 0);
		if (!IS_ERR(fd)) {
			goto alternate;
		}
		TS_DBG("BYD TS: Warning: %s is not found. \n", file_path);
#endif
		printk("%s: Warning: file %s is not found in all specified locations, FILE READ ABORTED!! \n", __FUNCTION__, p_file);
		return -ENOENT;
	}

alternate:
	do {
		mm_segment_t old_fs = get_fs();
		set_fs(KERNEL_DS);

		if (fd->f_op == NULL || fd->f_op->read == NULL) {
			pr_err("BYD TS: Error in set_fs(KERNEL_DS)!! ");
			break;
		}

		if (fd->f_pos != offset) {
			if (fd->f_op->llseek) {
				if (fd->f_op->llseek(fd, offset, 0) != offset) {
					pr_err("BYD TS: Error in llseek()!! ");
					break;
				}
			} else {
				fd->f_pos = offset;
			}
		}

		ret_len = fd->f_op->read(fd, buf, len, &fd->f_pos);
		 //fd->f_op->write(fd, buf, len, &fd->f_pos);
		printk("%s: %d bytes of data read from file %s. \n", __FUNCTION__, ret_len, file_path);

		set_fs(old_fs);

	} while (false);

	if (ret_len == -EIO) pr_err("FILE READ ABORTED!! \n");

	filp_close(fd, NULL);

	return ret_len;
}

/*******************************************************************************
* Function    : byd_ts_write_file 
* Description : write data to file 
* Parameters  : filepath, buffer, len, offset
* Return      : file len
*******************************************************************************/
static int byd_ts_write_file(char *filename, char *buf, ssize_t len, int offset)
{	
    struct file *fd;
    int retLen = -1;

    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

    fd = filp_open(filename, O_WRONLY|O_CREAT, 0666);
    // fd = filp_open(filename, O_WRONLY, 0666);   
    if (IS_ERR(fd)) {
        TS_DBG("[BFxxxx_ts][flie_write] : failed to open!!\n");
        return -1;
    }
 //   fd ->f_op->llseek(fd, 0, 2);	// 2: file end  0:file start
    do {
        if ((fd->f_op == NULL) || (fd->f_op->write == NULL)) {
            printk("[BFxxxx_ts][flie_write] : file can not be write!!\n");
            break;
        } // End of if 

     //   if (fd->f_pos != offset) {
            if (fd->f_op->llseek) {
               // if(fd->f_op->llseek(fd, offset, 2) != offset) {  // 2: file end  0:file start
                if(fd->f_op->llseek(fd, offset, 2) < 0) {  // 2: file end  0:file start
                    printk("[BFxxxx_ts][flie_write] : failed to seek!!\n");
                    break;
                }
            } else {
                fd->f_pos = offset;
            }
       // }       		

        retLen = fd->f_op->write(fd, buf, len, &fd->f_pos);

    } while (false);

    filp_close(fd, NULL);

    set_fs(old_fs);

    return retLen;
}

/*******************************************************************************
* Function    : byd_ts_read_file_with_id
* Description : filling data to a buffer form file
* In          : data_file, module_num, buffer, len
* Return      : length of data read or negative value in the case of error
                -EIO:    I/O error
                -ENOENT: No file found
*******************************************************************************/
int byd_ts_read_file_with_id(unsigned char const *data_file_c, int module_num, char *buf, ssize_t len, int offset)
{
	int ret;
	unsigned char *data_file_p, data_file[127];

	strcpy(data_file, data_file_c);
	data_file_p = strstr(data_file, "#."); // data_file_p = strrchr(data_file, '.'); 
	if (data_file_p != NULL) {
		if (module_num < 10) {
			*data_file_p = (unsigned char)(module_num + 0x30);
		} else {
			*data_file_p = (unsigned char)(module_num / 10 + 0x30);
			*(data_file_p + 6) = *(data_file_p + 5);
			*(data_file_p + 5) = *(data_file_p + 4);
			*(data_file_p + 4) = *(data_file_p + 3);
			*(data_file_p + 3) = *(data_file_p + 2);
			*(data_file_p + 2) = *(data_file_p + 1);
			*(data_file_p + 1) = (unsigned char)(module_num % 10 + 0x30);
		}
	}
	data_file_p = data_file;

	ret = byd_ts_read_file(data_file_p, buf, len, offset);

	return ret;
}

#endif // ..._FIRMWARE_UPG || ..._PARAMETER_FILE

///////////////////////////////////////////////////////////////////////////////
/*
* Module      : Touchscreen working parameter updating
* Description : tuning touchscreen by writing parameters to its registers
* Interface   : int byd_ts_download(i2c_client, id)
*               in:  i2c_client
*                    id - module number, -1 unknown, 0 firmware download
*               out: status of parameter updating
* Return      : status (error if less than zero)
*/
#if defined(CONFIG_SUPPORT_PARAMETER_UPG) || defined(CONFIG_SUPPORT_PARAMETER_FILE) // Module range

	  #define I2C_WRITE_PARAMETER 0x20
	  #define LEN_HEAD 1
	  #define PARA_BUFFER_SIZE 65

	  #define MAX_DOWNLOAD_BUFFER_SIZE	256 //PARA_BUFFER_SIZE
	  #define PARA_FILE_OFFSET 0

/*******************************************************************************
* Function    : smbus_master_send
* Description : send a buffer of data over i2c bus: - FOR cts12/13 PARA DOWNLOAD
*               encapsulation of i2c_smbus_write_i2c_block_data for size limited
*               transmiting
*               the data may be break into none-addressable packets and send
*               over one by one 
* Parameters  : input: client, buf, len
*               output: none
* Return      : the length of bytes sent or transmit status
*******************************************************************************/
#ifdef I2C_PACKET_SIZE
  #if (I2C_PACKET_SIZE == 8) // MTK specific
	#define I2C_SEND_MAX_SIZE	((I2C_PACKET_SIZE - 1)  / 4 * 4)  // 1 means device_addr
  #else
	#define I2C_SEND_MAX_SIZE	((I2C_PACKET_SIZE + 1) / 4 * 4) // 1 means reg_addr
  #endif
#endif
static int smbus_master_send(struct i2c_client *client, char *buf, int len)
{
#if !defined(I2C_PACKET_SIZE) || (I2C_PACKET_SIZE > 32)
	return i2c_master_send(client, buf, len);
#else
	int j, packet_number, leftover, start = 0, i = 0;
	unsigned char packet[I2C_SEND_MAX_SIZE];

	leftover = len % I2C_SEND_MAX_SIZE;
	packet_number = len / I2C_SEND_MAX_SIZE + (leftover > 0 ? 1 : 0);
	for (j = 0; j < packet_number; j++) {
		int packet_len, ret;

	    start = j * I2C_SEND_MAX_SIZE;
		packet_len = (leftover > 0 && j == packet_number - 1) ? leftover : I2C_SEND_MAX_SIZE;

		for (i = 0; i < packet_len; i++) {
			packet[i] = buf[start + i];
        }

		ret = i2c_smbus_write_i2c_block_data(client, packet[0], packet_len - 1, packet + 1);
		if (ret < 0) {
			pr_err("%s: err #%d returned by i2c_smbus_write_i2c_block_data()\n", __FUNCTION__, ret);
			return ret;
		}
	}
	TS_DBG("%s: I2C_SEND_MAX_SIZE:%d, total_packets:%d, last_packet_size:%d\n", __FUNCTION__, I2C_SEND_MAX_SIZE, packet_number, i);
	PR_BUF("last packet:", packet, i);
	return start + i;
#endif
}

// CRC is used only for parameter tune
static unsigned char calc_crc(unsigned char* buf, int len)
{
	unsigned int i;
	unsigned char crc = 0;

	for(i = 0; i < (len - 1); i++) {
		crc += buf[i];
	}
	buf[len - 1] = crc;

	return buf[len - 1];
}


/*******************************************************************************
 * Function    : byd_ts_download
 * Description : download parameters or firmware to TP IC via I2C
 * In          : client, i2c client
 *               nodule_number, -1 unknown, 0 firmware download
 * Out         : none
 *              -EIO:    I/O error
 *              -ENOENT: No module number found
 *              -ENODATA: No parameter data found
 * Return      : status
*******************************************************************************/
int byd_ts_download(struct i2c_client *client, int module_num)
{
	int ret = 0, tmp, err = 0;
	unsigned char *buf = NULL;
	int len = 0;

#ifdef CONFIG_SUPPORT_PARAMETER_UPG // ..._UPG
	unsigned char buf1[] = BYD_CTS0X_TS1;
	unsigned char buf2[] = BYD_CTS0X_TS2;
	unsigned char buf3[] = BYD_CTS0X_TS3;
	unsigned char buf4[] = BYD_CTS0X_TS4;
	unsigned char buf5[] = BYD_CTS0X_TS5;
	unsigned char buf6[] = BYD_CTS0X_TS6;
	unsigned char buf7[] = BYD_CTS0X_TS7;
	unsigned char buf8[] = BYD_CTS0X_TS8;
	unsigned char buf9[] = BYD_CTS0X_TS9;
#endif // ..._UPG

#ifdef CONFIG_SUPPORT_PARAMETER_FILE
	unsigned char buff[MAX_DOWNLOAD_BUFFER_SIZE + LEN_HEAD] = {I2C_WRITE_PARAMETER};
	int len_f = MAX_DOWNLOAD_BUFFER_SIZE + LEN_HEAD;
#endif

	TS_DBG("%s entered \n", __func__);

	disable_irq(client->irq);
#ifdef CONFIG_SUPPORT_MODULE_ID
	if (module_num < 0) {
		module_num = byd_ts_get_module_number(client);
		if (module_num <= 0) {
			pr_err("%s: module number is not correct, default to 1.\n",
				__FUNCTION__);
			err = -ENOENT; // err = module_num;
			module_num = 1;
		}
	} else if (module_num == 0) {
		// TO DO:
	}
#else
	if (module_num < 0) module_num = 1;
#endif

#ifdef CONFIG_SUPPORT_PARAMETER_UPG
	switch (module_num) {
		case 1:
			buf = buf1;
			len = (int)sizeof(buf1);
			break;
		case 2:
			buf = buf2;
			len = (int)sizeof(buf2);
			break;
		case 3:
			buf = buf3;
			len = (int)sizeof(buf3);
			break;
		case 4:
			buf = buf4;
			len = (int)sizeof(buf4);
			break;
		case 5:
			buf = buf5;
			len = (int)sizeof(buf5);
			break;
		case 6:
			buf = buf6;
			len = (int)sizeof(buf6);
			break;
		case 7:
			buf = buf7;
			len = (int)sizeof(buf7);
			break;
		case 8:
			buf = buf8;
			len = (int)sizeof(buf8);
			break;
		case 9:
			buf = buf9;
			len = (int)sizeof(buf9);
			break;
		default:
			//buf = buf1;
			//len = sizeof(buf1); // error: sizeof(*buf)
			break;
	};
	//if (len - LEN_HEAD > 0) {
	//	printk("%s: %d bytes of default data is found in driver. module number:%d \n", __FUNCTION__, len - LEN_HEAD, module_num);
	//}
#endif // ...UPG

#if defined(CONFIG_SUPPORT_PARAMETER_FILE) || defined(FIRM_DOWNLOAD_FILE)
	ret = byd_ts_read_file_with_id(PARA_DOWNLOAD_FILE, module_num, buff + LEN_HEAD, len_f - LEN_HEAD, PARA_FILE_OFFSET);
	if (ret > 0) {
		buf = buff;
		if (len > 0) {
			printk("%s: the driver's default data will be overrided by the data in external file. \n", __FUNCTION__);
		}
		len = ret + LEN_HEAD; // the last byte for CRC included
	} else if (len - LEN_HEAD > 0) { // parameter from driver's parameter array
		printk("%s: %d bytes of default data is found in driver. module number:%d \n", __FUNCTION__, len - LEN_HEAD, module_num);
	}
#endif // ...PARAMETER_FILE

	if (len <= LEN_HEAD) {
		printk("%s: data not found, download aborted, errno:%d\n", __FUNCTION__, ret);
		printk("%s: the parameter in chip's flash will be used!\n", __FUNCTION__);
		enable_irq(client->irq);
		return -ENODATA; // return to use chip's defaults
	} else if (len != PARA_BUFFER_SIZE + LEN_HEAD && len != MAX_DOWNLOAD_BUFFER_SIZE) {
		printk("%s: Warning - the number of data may be incorrect.\n", __FUNCTION__);
	}
	printk("%s: sending data: %x, %x, %x, %x, ... %x, %x, len = %d \n",
		__FUNCTION__, buf[0], buf[1], buf[2], buf[3], buf[len-2], buf[len-1], len);

	// CRC is used only for parameter tune
	tmp = buf[len - 1];
	buf[len - 1] = calc_crc(buf + LEN_HEAD, len - LEN_HEAD);
	//ret = file_write(CALI_FILENAME, buf, len, 0);

	ret = smbus_master_send(client, buf, len);
	if (ret <= 0) {
		pr_err("%s: smbus_master_send PARAMETER failed, errno: %d\n", __FUNCTION__, ret);
		enable_irq(client->irq);
		return ret;
	}

	// wait until chip finished writing and restart
	msleep(250);

  #if defined(TS_DEBUG_MSG) // this is for test only
	{
		unsigned char crc_r;
		//ret = i2c_master_send(client, buf, 2);
		//if (ret < 0) {
		//	pr_err("%s write register failed! errno: %d\n", __FUNCTION__, ret);
		//}

		ret = smbus_master_recv(client, &crc_r, 1);
		if (ret <= 0) {
			pr_err("%s: i2c read CRC error, errno: %d\n", __FUNCTION__, ret);
		}
		TS_DBG("%s: CRC test: original/writen/read = 0x%x / 0x%x / 0x%x\n",
			__FUNCTION__, tmp, buf[len - 1], crc_r);
		if (buf[len - 1] != crc_r) {
			pr_err("%s: check sum (CRC) failed! \n", __FUNCTION__);
			enable_irq(client->irq);
			return -EINVAL;
		}
	}
  #endif

	enable_irq(client->irq);
	return err;
}

#endif // ..._PARAMETER_UPG || ..._PARAMETER_FILE // Module range

///////////////////////////////////////////////////////////////////////////////

#if defined(CONFIG_SUPPORT_RESOLUTION_ACQUISITION)

/*******************************************************************************
* Function    : byd_ts_get_resolution
* Description : read screen resolution from chip via I2c
* In          : i2c_client
* Out         : xy_res, screen resolution (X first and then Y, <=2048)
* Return      : status (error if less than zero)
*******************************************************************************/
// #define BYD_TS_RESOLUTION_REG	70  //0x46-0x49
#define RESO_BUF_SIZE 4
int byd_ts_get_resolution(struct i2c_client *client, int *xy_res)
{
	int ret;
	unsigned char cmd[] = I2C_READ_RESOLUTION; // I2C_READ_PARAMETERS
	unsigned char buf[RESO_BUF_SIZE]; // buf[PARA_BUFFER_SIZE]

	ret = smbus_read_bytes(client, cmd, sizeof(cmd), buf, sizeof(buf));
	if (ret <= 0) {
		pr_err("%s: i2c read RESOLUTION error, errno: %d\n", __FUNCTION__, ret);
		return ret;
	}

	xy_res[0] = (buf[0] << 8) | buf[1];
	xy_res[1] = (buf[2] << 8) | buf[3];

	return ret;
}

#endif //CONFIG_SUPPORT_RESOLUTION_ACQUISITION

#ifdef CONFIG_SUPPORT_VERSION_ACQUISITION
/*******************************************************************************
* Function    : byd_ts_get_firmware_version
* Description : get firmware's version from chip
* In          : i2c_client
* Return      : status (error if less than zero)
*******************************************************************************/
#define VERSION_BUF_SIZE	6
int byd_ts_get_firmware_version(struct i2c_client *client, unsigned char* version)
{
	unsigned char version_id[] = I2C_READ_FIRMWARE_VERSION; // must single byte
	int ret;
	//err = smbus_read_bytes(client, version_id, sizeof(version_id), version, VERSION_SIZE);
	ret = i2c_smbus_read_i2c_block_data(client, version_id[0], VERSION_BUF_SIZE, version);
	if (ret < 0) {
		pr_err("%s: i2c read VERSION# error, errno: %d \n", __FUNCTION__, ret);
	}
	return ret;
}
#endif

///////////////////////////////////////////////////////////////////////////////


#if defined(CONFIG_SUPPORT_RAW_DATA_ACQUISITION) || defined(CONFIG_SUPPORT_PARAMETER_FILE)

int byd_ts_get_channel_number(struct i2c_client *client)
{
	int ret;
	unsigned char cmd[] = I2C_READ_RAW_DATA;
	char n_channel;

	ret = smbus_read_bytes(client, cmd, sizeof(cmd), &n_channel, 1);
	if (ret < 0) {
		pr_err("%s: error in smbus_read_bytes, errno: %d\n", __FUNCTION__, ret);
		//*n_channel = CHANNELS_MAX + 1;
		return ret;
	}

	return (int)n_channel;
}

static int byd_ts_open(struct inode *inode, struct file *file) {
 	TS_DBG("%s: major=%d, minor=%d\n", __FUNCTION__, imajor(inode), iminor(inode));
	return 0;
}

static int byd_ts_release(struct inode *inode, struct file *file) {
	return 0;
}

static unsigned char n_tx_rx[2] = {CHANNELS_MAX + 1, 0}; // {1, 0};

static int byd_ts_read(struct file *file, char __user *user_buf,
	size_t const count, loff_t *offset)
{
	int ret,i;
	static int len = 2; // = count

	if (count <= 2) {
		if (n_tx_rx[0] <= 1 || n_tx_rx[0] > CHANNELS_MAX) {
			ret = byd_ts_get_channel_number(This_Client);
			if (ret < 0) {
				/* in the case of sleeping, try again */
				msleep(5);
				ret = byd_ts_get_channel_number(This_Client);
			}
			if (ret <= 1 || ret > CHANNELS_MAX) {
				pr_err("%s: error in getting the total number of channels ! errno: %d\n", __FUNCTION__, ret);
				len = CHANNELS_MAX + 1;
				//return ret;
			} else {
				len = ret;
				printk("%s: number of channels: %d\n", __FUNCTION__, len);
			}
			n_tx_rx[0] = len;
			n_tx_rx[1] = 0; // ensure self-capacitance
			len = 1 + len * 2 + len * 2;
		}
		ret = copy_to_user(user_buf, n_tx_rx, count);

	} else 	{ // count > 2
		char buf[1 + (CHANNELS_MAX + 1) * 2 + (CHANNELS_MAX + 1) * 2] = {CHANNELS_MAX + 1, 0}; //{1, 0};
		TS_DBG("%s: size of raw data: %d\n", __FUNCTION__, len);
		ret = smbus_master_recv(This_Client, (char *)&buf, len);
		//ret = mtk_smbus_master_recv(This_Client, (char *)&buf, len);
	//	for(i =0; i<len;i++)
	//		{
	//		TS_DBG("%s:buf[%d] = %x \n", __FUNCTION__, i,  buf[i]);
	//		}
		if (ret < 0) {
			pr_err("%s: i2c_master_recv RAWDATA error, errno: %d\n", __FUNCTION__, ret);
		}
		if (ret < 0) {
			return ret;
		}
		ret = copy_to_user(user_buf, buf, count);

	}

	if (ret < 0) {
		pr_err("%s: copy_to_user() error, errno: %d\n", __FUNCTION__, ret);
		return ret;
	}

	return 0;
}

static int byd_ts_write(struct file *file, /*char __user *user_buf,*/
	size_t const count, loff_t *offset)
{
	int ret;
	int len1 = 1+n_tx_rx[0]*4; // = count
	
	if (count <= 2) {
	

	} else 	{ // count > 2
		char buf[1 + (CHANNELS_MAX + 1) * 2 + (CHANNELS_MAX + 1) * 2] = {CHANNELS_MAX + 1, 0}; //{1, 0};
		TS_DBG("%s: size of raw data: %d\n", __FUNCTION__, len1);
		//ret = smbus_master_recv(This_Client, (char *)&buf, len);
		ret = mtk_smbus_master_recv(This_Client, (char *)&buf, len1);
		if (ret < 0) {
			pr_err("%s: i2c_master_recv RAWDATA error, errno: %d\n", __FUNCTION__, ret);
		}
		if (ret < 0) {
			return ret;
		}
		
		ret = byd_ts_write_file(RAWDATA_FILE, &buf[1], len1 * 2, 0);
		if (ret < 0) {
			pr_err("%s: write rawdata file error, errno: %d\n", __FUNCTION__, ret);
		}
		ret = byd_ts_write_file(BASELINE_FILE, &buf[1]+len1 * 2, len1 * 2, 0);
		if (ret < 0) {
			pr_err("%s: write rawdata file error, errno: %d\n", __FUNCTION__, ret);
		}
	}

	if (ret < 0) {
		pr_err("%s: copy_to_user() error, errno: %d\n", __FUNCTION__, ret);
		return ret;
	}

	return 0;
}

static int byd_ts_ioctl(struct file *file, unsigned int cmd, unsigned long arg) 
{
	int ret, len_cmd;
	unsigned char *command = NULL;
	unsigned char cmd_rawdata[] = I2C_READ_RAW_DATA;
	unsigned char cmd_rawdata_sf[] = I2C_READ_RAW_DATA_SINGLE_FRAME;
	unsigned char cmd_exit_debug[] = I2C_EXIT_DEBUG_MODE;
	unsigned char cmd_reset[] = I2C_RESET;

	//TS_DBG("%s: line=%d, cmd=%d, arg=%ld\n", __FUNCTION__, __LINE__, cmd, arg);
	switch (cmd) {

		case 1:
		case 3: // *** cmd in ioctl() cannot be 2 in Linux ***
			if (cmd == 1) {
				command = cmd_rawdata;
			} else if (cmd == 3) {
				command = cmd_rawdata_sf;
			}
			len_cmd = sizeof(cmd_rawdata);
			//Raw_Data_Mode = 1;
			printk("%s: i2c send command: rawdata\n", __FUNCTION__);
		break;

		case 4: // on line parameter download
#if defined(CONFIG_SUPPORT_PARAMETER_UPG) || defined(CONFIG_SUPPORT_PARAMETER_FILE)
			printk("%s: i2c send command: download parameter\n", __FUNCTION__);
			ret = byd_ts_download(This_Client, -1); // -1 unknown module number
			if (ret == -ENOENT) { // no module number found, use default
				ret = 0;
			} else if (ret < 0) {
				pr_err("%s: error in parameter download, errno: %d\n", __FUNCTION__, ret);
			}
			n_tx_rx[0] = CHANNELS_MAX + 1; // 1;  // signal re-acquire number of channels
			n_tx_rx[1] = 0; // 1;
#else
			printk("%s: parameter download is not supported!\n", __FUNCTION__);
			ret = 0;
#endif
		return ret;

		case 5: // on line firmware update
#ifdef CONFIG_SUPPORT_FIRMWARE_UPG
			disable_irq(This_Client->irq);
			printk("%s: i2c send command: firmware update\n", __FUNCTION__);
			ret = byd_ts_upgrade_firmware(This_Client);
			if (ret < 0) {
				pr_err("%s: error in firmware update, errno: %d\n", __FUNCTION__, ret);
			}
			n_tx_rx[0] = CHANNELS_MAX + 1; // 1;  // signal re-acquire number of channels
			n_tx_rx[1] = 0; // 1;
			enable_irq(This_Client->irq);
#else
			printk("%s: firmware update is not supported!\n", __FUNCTION__);
			ret = 0;
#endif
		return ret;

		case 6:
			
		//disable_irq(This_Client->irq);
		//byd_rawdata_test();
		//enable_irq(This_Client->irq);
		//return ret;
		#if 1
			command = cmd_exit_debug;
			len_cmd = sizeof(cmd_exit_debug);
			printk("%s: i2c send command: exit_debug_mode\n", __FUNCTION__);
		#endif
		break;

		case 7:
			disable_irq(This_Client->irq); 
			command = cmd_reset;
			len_cmd = sizeof(cmd_reset);
			//Raw_Data_Mode = 0;
			printk("%s: i2c send command: reset\n", __FUNCTION__);
		break;
		case 9:		
		case 10:
			if (cmd == 9) {
				command = cmd_rawdata;
			} else if (cmd == 10) {
				command = cmd_rawdata_sf;
			}
			len_cmd = sizeof(cmd_rawdata);
			//Raw_Data_Mode = 1;
			printk("%s: i2c write rawdata to file\n", __FUNCTION__);
			break;
		default:
			TS_DBG("%s: unknown i2c command\n", __FUNCTION__);
		return 0;
	}
     
	ret = i2c_smbus_write_i2c_block_data(This_Client, command[0], len_cmd - 1, command + 1);
	if (ret < 0) {
		/* in the case of sleeping, try again */
		printk("%s: warnning: error %d in i2c send command, will try again after 5 ms!\n", __FUNCTION__, ret);
		msleep(5);
		ret = i2c_smbus_write_i2c_block_data(This_Client, command[0], len_cmd - 1, command + 1);
		if (ret < 0) {
			pr_err("%s: i2c_master_send COMMAND error, errno: %d\n", __FUNCTION__, ret);
			return ret;
		}
	}

	if (cmd == 7) {
		n_tx_rx[0] = CHANNELS_MAX + 1; // 1;  // signal re-acquire number of channels
		n_tx_rx[1] = 0; // 1;
		enable_irq(This_Client->irq);
	}
	return 0;
}

static struct file_operations byd_ts_fops = {
	.owner   = THIS_MODULE,
	.open    = byd_ts_open,
	.release = byd_ts_release,
	.read    = byd_ts_read,
	.write   = byd_ts_write,
	.unlocked_ioctl = byd_ts_ioctl,
};

// ******* defining a device for raw data output ********
/*  Variable   : byd_ts_device - misc device for raw data
 *  Accessed by: byd_ts_probe(), byd_ts_remove()
 */
struct miscdevice byd_ts_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = BYD_TS_NAME,
	.fops  = &byd_ts_fops,
};

#endif // CONFIG_SUPPORT_RAW_DATA_ACQUISITION || CONFIG_SUPPORT_PARAMETER_FILE




//##############################################################################

#if defined(CONFIG_SUPPORT_FIRMWARE_UPG)

/*
 * File:         byd_cts02_12_13_ts.c
 *
 * Created:	     2012-10-07
 * Depend on:    byd_bfxxxx_ts.c
 * Description:  Firmware burnning for BYD TouchScreen IC
 *
 * Copyright (C) 2012 BYD Company Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* /////////////// compile independently //////////////////////////////////////
#include <linux/i2c.h>       // struct i2c_client
#include <linux/delay.h>     // msleep(), udelay()

#include "byd_bfxxxx_ts.h"

#ifdef TS_DEBUG_MSG
#define TS_DBG(format, ...)	\
	printk(format, ##__VA_ARGS__)
#else
#define TS_DBG(format, ...)
#endif

#define BYD_TS_ALTERNATE_ADDRESS	0x43 // 0x43/0x86, old 0x5d/0xba

int byd_ts_read_file(char *p_path, char *buf, ssize_t len, int offset);
int byd_ts_get_module_number(struct i2c_client *client);
int byd_ts_read_file_with_id(unsigned char const *data_file_c, int module_num, char *buf, ssize_t len, int offset);
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */

typedef unsigned char   FTS_BYTE;
#define FTS_PACKET_LENGTH       128
#define bin     0
#define para    1

/* only presented file(s) in SD card /mnt/sdcard or following position will be
   writen to chip's flash memory */
  	#define FIRM_BURNING_FILE	"/data/local/tmp/byd_cts02_ts#.bin"
	#define PARA_BURNING_FILE	"/data/local/tmp/byd_cts02_ts#.dat"
// I2C address for burning, possibly 0xfc/7e, 0x1b, 0x3a, 0x1a or 0x70
#define   READ_CONFIG0  0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
unsigned char   I2C_ONLINE_MODE_COMMAND[] = {0xbb};
unsigned char   ONLINE_HANDSHAKE[10] = {0xa1, 0x1a, 0xc3, 0x3c, 0xe7, 0x7e, 0xa5, 0x5a, 0x24, 0x42};  //ONLINE_HANDSHAKE
unsigned char   I2C_ONLINE_BEGIN[10] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa}; //ONLINE_BEGIN
unsigned char   I2C_WRITE_FLASH[] = {0x81, 0x00, 0x00, 0x5a, 0x22, 0xc6, 0x66};  //write flash
//unsigned char   I2C_FLASH_END_ADDR[] = {0x81, 0x00, 0x00, 0xa5, 0x68, 0xf5, 0x86};  //flash end address
unsigned char   I2C_ADDR_SUB_1[] = {0x81, 0x00, 0x00, 0xa5, 0xbc, 0xbc, 0x9a};  //address subtract-1
unsigned char   I2C_ERASE_FULL_FLASH[] = {0x81, 0x00, 0x00, 0x5a, 0xc3, 0x58, 0xa4};  //erase full flash
unsigned char   I2C_IN_READ_FLASH_1[] = {0x81, 0x00, 0x00, 0xb6, 0x3b, 0x20, 0x6e};  //entry read flash
unsigned char   I2C_IN_READ_FLASH_2[] = {0x81, 0x00, 0x00, 0xb6, 0x7b, 0xc7, 0xe4};  //entry read flash
unsigned char   I2C_OUT_READ_FLASH_1[] = {0x81, 0x00, 0x00, 0xb6, 0x76, 0x15, 0x46};  //end read flash
unsigned char   I2C_OUT_READ_FLASH_2[] = {0x81, 0x00, 0x00, 0xb6, 0x36, 0xf2, 0xcc};  //end read flash
unsigned char   CTS12_CONFIG0_COMMAND[] = {0x80, 0x00, 0x00, 0x03, 0x33}; //
unsigned char   I2C_RDN1_START_ADDR[] = {0x81, 0x00, 0x00, 0xa5, 0x05, 0x76, 0xf2}; //RDN1区首地址
unsigned char   CTS13_CONFIG0_COMMAND[] = {0x80, 0x00, 0x00, 0x00, 0x1b};  //
unsigned char   I2C_BUFFER_DATA[4] = {0xff, 0xff, 0xff, 0xff};

//CTS02 command
unsigned char   I2C_ONLINE_STATE_ADDR[1] = {0xaf};    //ONLINE_BEG_CHECK
unsigned char   I2C_ONLINE_STATE_DATA[1] = {0x55};  //exit online mode check
unsigned char   I2C_FLASH_START_ADDR[1] = {0x80};  //flash start address
unsigned char   I2C_FLASH_DATA_ADDR[1] = {0x84};  //entry read flash
unsigned char   I2C_FLASH_PAGE2_ADDR[4] = {0xb4, 0x02, 0xb5, 0x00}; //page 2 address
unsigned char   I2C_ERASE_FLASH_PAGE[2] = {0x81, 0x83};
unsigned char   I2C_ADDR_ADD_1[1] = {0x86};  //address add+1
unsigned char   I2C_ONLINE_EXIT_DATA[10] = {0xb3, 0xb6, 0xb3, 0xb6, 0xb3, 0xb6, 0xb3, 0xb6, 0xb3, 0xb6};  //exit online mode
unsigned char   I2C_FLASH_PAGE31_ADDR[4] = {0xb4, 0x3c, 0xb5, 0x00}; //page 2 address
unsigned char   I2C_FLASH_END_ADDR[4] = {0xb4, 0x3c, 0xb5, 0x7c}; //page 2 address
unsigned char   I2C_ERASE_OVER[2] = {0x90, 0x97};


int byd_ts_rename_file(struct i2c_client *client, char *file_path, int module_num)
{
	int  ret = 0;
	char cmd_path[] = "/system/bin/mv";
	char s_file[127], d_file[127];
	char *s_file_p = s_file, *d_file_p = d_file;
	char* cmd_argv[] = {cmd_path, s_file_p, d_file_p, NULL};
	//char* cmdEnvp[] = {"HOME=/", "PATH=/sbin:/bin:/usr/bin", NULL};

	strcpy(s_file, file_path);
	s_file_p = strstr(s_file, "#.");
	if (s_file_p != NULL) {
		*s_file_p = (unsigned char)(module_num + 0x30);
	}
	//s_file_p = s_file;
	strcpy(d_file, s_file);
	strcat(d_file, ".bak");
	//cmd_argv[1] = s_file_p;
	printk("%s: %s %s %s \n", __FUNCTION__, cmd_argv[0], cmd_argv[1], cmd_argv[2]);
	ret = call_usermodehelper(cmd_path, cmd_argv, NULL, UMH_WAIT_PROC);
	if (ret < 0) {
		pr_err("%s: could not remove file %s, errno: %d\n", __FUNCTION__, s_file_p, ret);
	}
	
	return ret;
}


/*******************************************************************************
* Function    :  CRC 校验
* Description : 对32 位数据进行数据效验
* Parameters  :  datahigh--地址(0x80/0x81)，datalow--32位数据
* Return      :    crc 值
*******************************************************************************/
static unsigned int calc32bit_crc(unsigned char datahigh, unsigned int datalow)
{
	unsigned int crc = 0;
	unsigned int tempdata = 0;
	unsigned int tempcrc = 0;
	unsigned char i = 0;

	for (i = 0; i < 8; i++)
	{
		tempdata = datahigh << 7;
		if ((tempdata & 0x4000) != (crc & 0x4000))
		{
			crc = ((crc << 1) & 0x7fff) ^ 0x4599;
		}
		else
		{
			crc = (crc << 1) & 0x7fff;
		}
		datahigh = datahigh << 1;
	}

	for (i = 0; i < 32; i++)
	{
		tempcrc = (unsigned int)crc << 17;
		if ((tempcrc & 0x80000000) != (datalow & 0x80000000))
		{
			crc = ((crc << 1) & 0x7fff) ^ 0x4599;
		}
		else
		{
			crc = (crc << 1) & 0x7fff;
		}
		datalow = datalow << 1;
	}
   	return crc;
}

/*******************************************************************************
* Function    :  byd_ctpm_crc_get
* Description : get crc data to buffer
* Parameters  :  datahigh--地址(0x80/0x81)，datalow--32位数据
* Return      :    crc 值
*******************************************************************************/
static FTS_BYTE* byd_ctpm_crc_get(u8* pbt_buf)
{
	int IICBufOnlineCRC;
	unsigned int tempcrc = 0;

	IICBufOnlineCRC = (pbt_buf[1] << 24) | (pbt_buf[2] << 16) | (pbt_buf[3] << 8) | pbt_buf[4];
	tempcrc = calc32bit_crc(pbt_buf[0], IICBufOnlineCRC);
	tempcrc = (tempcrc << 1);
	pbt_buf[5] = tempcrc >> 8;
	pbt_buf[6] = tempcrc;
	return pbt_buf;
}

//read program form flash
/*******************************************************************************
* Function    : static int  byd_ctp_read_byte(struct i2c_client *client,uint8_t *address, char addresslen,  FTS_BYTE* pbt_buf,char type)
* Description : CTS12/13
*               read program form flash
* Parameters  : address : start address
*               direction : +1/-1
*               len:  length
*               pbt_buf: data save in buffer
*               type:  para or bin
* Return      : 0: successful ; error: -1
*******************************************************************************/
static int  byd_ctp_read_byte(struct i2c_client *client, uint8_t *address, char addresslen, uint8_t *comparedata, unsigned char* pbt_buf, char type)
{
	int  i = 0;
	unsigned char IICBufOnlineR[4];
	printk("%s: PROG_READ_START  ok \n",__FUNCTION__);
	i2c_master_send(client, address,addresslen);

 	for(i = 0; i < 4; i++)			//xwc/ 读取前4个字节, 然后比较
	{
		i2c_master_send(client, I2C_FLASH_DATA_ADDR, sizeof(I2C_FLASH_DATA_ADDR));		//xwc/读数据, 地址?????
		i2c_master_recv(client, &pbt_buf[i], 1);
		i2c_master_send(client,  I2C_ADDR_ADD_1, sizeof(I2C_ADDR_ADD_1));
	}
	TS_DBG("IICBufOnlineR = %x, %x, %x, %x \n",  pbt_buf[0], pbt_buf[1], pbt_buf[2], pbt_buf[3]);
	TS_DBG("comparedata = %x, %x, %x, %x \n",  comparedata[0], comparedata[1], comparedata[2], comparedata[3]);
	 if ((pbt_buf[0] == comparedata[0]) && (pbt_buf[1] == comparedata[1]) && 
	 	(pbt_buf[2] ==comparedata[2]) && (pbt_buf[3] == comparedata[3]))
	{ //如果读到的数不是 0xab9d573b
		printk("flash upgrade ok\n");
		return 1;
	}
	else
	{
		printk("cts02 flash upgrade  error\n");
		return -1;
	}
}


//read program form flash
/*******************************************************************************
* Function    : static FTS_BYTE*  byd_ctp_read_flash(struct i2c_client *client,uint8_t *address, char addresslen,  FTS_BYTE* pbt_buf,char type)
* Description : CTS12/13
*               read program form flash
* Parameters  : address : start address
*               direction : +1/-1
*               len:  length
*               pbt_buf: data save in buffer
*               type:  para or bin
* Return      : 0: successful ; error: -1
*******************************************************************************/
static int  byd_ctp_read_flash(struct i2c_client *client, uint8_t *address, char addresslen, uint8_t *comparedata, FTS_BYTE* pbt_buf, char type)
{
	int  i = 0;
	unsigned char IICBufOnlineR[4];
	printk("%s: PROG_READ_START  ok \n",__FUNCTION__);
	i2c_master_send(client, address,addresslen);

 	for(i = 0; i < 4; i++)			//xwc/ 读取前4个字节, 然后比较
	{
		i2c_master_send(client, I2C_FLASH_DATA_ADDR, sizeof(I2C_FLASH_DATA_ADDR));		//xwc/读数据, 地址?????
		i2c_master_recv(client, &IICBufOnlineR[i], 1);
		i2c_master_send(client,  I2C_ADDR_ADD_1, sizeof(I2C_ADDR_ADD_1));
	}
	TS_DBG("IICBufOnlineR = %x, %x, %x, %x \n",  IICBufOnlineR[0], IICBufOnlineR[1], IICBufOnlineR[2], IICBufOnlineR[3]);
	TS_DBG("comparedata = %x, %x, %x, %x \n",  comparedata[0], comparedata[1], comparedata[2], comparedata[3]);
	 if ((IICBufOnlineR[0] == comparedata[0]) && (IICBufOnlineR[1] == comparedata[1]) && 
	 	(IICBufOnlineR[2] ==comparedata[2]) && (IICBufOnlineR[3] == comparedata[3]))
	{ //如果读到的数不是 0xab9d573b
		printk("cts02_ctpm_read_Result ok\n");
		return 1;
	}
	else
	{
		printk("cts02_ctpm_read_Result error\n");
		return -1;
	}
}

//进入在线编程模式
/*******************************************************************************
* Function    :  byd_ctpm_fw_online
* Description :  CTS02/12/13
*                entry online  mode and check
* Parameters  :  none
* Return      :  0: successful ; error: -1
*******************************************************************************/
static int byd_ctpm_fw_online(struct i2c_client *client)
{
	int i;
	unsigned char IICBufOnlineR[10];
	unsigned char IICBufOnlineR1[2];	
	printk("%s:\n",__FUNCTION__);
	//由正常工作切换到在线编程模式
	i2c_master_send(client, I2C_ONLINE_MODE_COMMAND, 1);
	//进入在线编程的握手指令
	i2c_master_send(client, ONLINE_HANDSHAKE, 10);  //4
      // i2c_master_recv(client, IICBufOnlineR, 10);
	 i2c_master_recv(client, IICBufOnlineR, 8);
	 i2c_master_recv(client, IICBufOnlineR1, 1);	  
	for (i = 0; i < 8; i++)
	{
		if (IICBufOnlineR[i] != ONLINE_HANDSHAKE[i])
        {
	    	printk("%s: ONLINE_HANDLE error! \n", __FUNCTION__);
			return -1;
        }
		printk("%s: ONLINE_HANDLE ok! \n", __FUNCTION__);

        }

	//开始在线编程的命令
	i2c_master_send(client, I2C_ONLINE_BEGIN, 10);  //4
	TS_DBG("%s:ONLINE_BEGIN data \n", __FUNCTION__);
	msleep(30);
	//判断是否已进入在线编程
	i2c_master_send(client,  I2C_ONLINE_STATE_ADDR, sizeof(I2C_ONLINE_STATE_ADDR));
	i2c_master_recv(client, IICBufOnlineR, sizeof(I2C_ONLINE_STATE_DATA));
	//pr_info("IICBufOnlineR = %x, %x, %x, %x \n", IICBufOnlineR[0], IICBufOnlineR[1], IICBufOnlineR[2], IICBufOnlineR[3]);
	 if (IICBufOnlineR[0] == I2C_ONLINE_STATE_DATA[0])
	{ //如果读到的数不是 0xab9d573b
		printk("%s: ONLINE_BEG_CHECK ok \n", __FUNCTION__);
	}
	else
	{
		printk("%s:ONLINE_BEG_CHECK error \n", __FUNCTION__);
		return -1;
	}
	return 1;
}

//写数据到flash.
/*******************************************************************************
* Function    : byd_ctpm_fw_upgrade
* Description : CTS02
*               entry online  burst
* Parameters  : none
* Return      : 0: successful ; error: -1
*******************************************************************************/
static unsigned int  byd_ctpm_fw_upgrade(struct i2c_client *client, uint8_t *address, char addresslen, u8* pbt_buf, u16 dw_lenth)
{
	u16  i, j, temp, lenght, packet_number, ret;
	unsigned char IICBufOnlineW[7];
	u8  packet_buf[FTS_PACKET_LENGTH + 4];
	unsigned char CTPM_FLASHE[10];

	printk("%s: PROG_WRIT_start  ok \n", __FUNCTION__);
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;		// 要发送的packet数量
	//0x810000a502d108     //指向Flash起始地址
	//i2c_master_send(client,  I2C_FLASH_START_ADDR, sizeof(I2C_FLASH_START_ADDR));
	i2c_master_send(client,  address, addresslen);			//xwc/ 设置要写入数据的起始地址
	for (j = 0; j < packet_number; j++)
	{
	        temp = j * FTS_PACKET_LENGTH;
	        packet_buf[0] = (FTS_BYTE)(temp >> 8);
	        packet_buf[1] = (FTS_BYTE)temp;
	        lenght = FTS_PACKET_LENGTH;
	        packet_buf[2] = (FTS_BYTE)(lenght >> 8);
	        packet_buf[3] = (FTS_BYTE)lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++)
		{//xwc/ 将一个packet放入packet_buf[]
			packet_buf[4 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
           	}
		//xwc/ 写入一个packet
		for (i = 0; i < 128; i++)  ///FTS_PACKET_LENGTH/4
		{//xwc/ 写入一个byte，并地址+1
			IICBufOnlineW[0] =  0x82;
			IICBufOnlineW[1] =  packet_buf[4 + i];
			IICBufOnlineW[2] =  0x83;
			IICBufOnlineW[3] =  0x98;
			IICBufOnlineW[4] =  0x99;
			i2c_master_send(client, IICBufOnlineW, 5);
			udelay(1);
			////地址+1  0x810000a5b379b7
			i2c_master_send(client,  I2C_ADDR_ADD_1, sizeof(I2C_ADDR_ADD_1));
	  	}
	#if 0
		if ((j * FTS_PACKET_LENGTH % 4096) == 0)
		{
			pr_info("upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
			printk("upgrade the 0x%d th byte.\n", j * FTS_PACKET_LENGTH);

		}
	#endif
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)		//xwc/ 处理不足一个packet的数据。
	{
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp >> 8);
		packet_buf[3] = (FTS_BYTE)temp;

		for (i = 0; i < temp; i++)					//xwc/ 数据放入packet_buf[]中
		{
			packet_buf[4 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
		}
		for (i = 0; i < temp; i++)  ///FTS_PACKET_LENGTH/4	//xwc/ 数据写入底层设备
		{
			IICBufOnlineW[0] =  0x82;
			IICBufOnlineW[1] =  packet_buf[4 + i];
			IICBufOnlineW[2] =  0x83;
			IICBufOnlineW[3] =  0x98;
			IICBufOnlineW[4] =  0x99;
			i2c_master_send(client, IICBufOnlineW, 5);
			udelay(1);
			////地址+1  0x810000a5b379b7
			i2c_master_send(client,  I2C_ADDR_ADD_1, sizeof(I2C_ADDR_ADD_1));
	  	}
	}
	ret = byd_ctp_read_flash(client, address, addresslen, pbt_buf, CTPM_FLASHE, bin);  //19
	if(ret > 0)
		printk("%s: over \n", __FUNCTION__);
	else
		printk("%s: error \n", __FUNCTION__);
	return ret;
}

/*******************************************************************************
* 函数名称static int byd_ctpm_main_erase(struct i2c_client *client)
* 功能描述：擦除整片FLASH
* 输入参数：
* 返回值   ：ret
* 其它说明：CTS02/12/13
*******************************************************************************/
static int byd_ctpm_main_erase(struct i2c_client *client)
{
	int i, ret;
	unsigned char CTPM_FLASH[10];

	printk("%s: start \n", __FUNCTION__);
	I2C_FLASH_PAGE2_ADDR[1] = 0x02; // 20140605 added
	i2c_master_send(client, I2C_FLASH_START_ADDR, sizeof(I2C_FLASH_START_ADDR));
	i2c_master_send(client, I2C_ERASE_FLASH_PAGE, sizeof(I2C_ERASE_FLASH_PAGE));
	msleep(30);

	for(i = 0; i < 30; i++)  //29
	{
		i2c_master_send(client, I2C_FLASH_PAGE2_ADDR, sizeof(I2C_FLASH_PAGE2_ADDR));
		i2c_master_send(client, I2C_ERASE_FLASH_PAGE, sizeof(I2C_ERASE_FLASH_PAGE));
		msleep(30);
		I2C_FLASH_PAGE2_ADDR[1] += 2;
	}
	i2c_master_send(client, I2C_ERASE_OVER, 2);
	msleep(10);

	ret = byd_ctp_read_flash(client, I2C_FLASH_START_ADDR, sizeof(I2C_FLASH_START_ADDR), I2C_BUFFER_DATA, CTPM_FLASH, bin);  //19
	if(ret > 0)
	printk("%s: over \n", __FUNCTION__);
	else
	printk("%s: error \n", __FUNCTION__);
	return ret;

}
/*******************************************************************************
* 函数名称static int byd_ctpm_last_page_erase(struct i2c_client *client)
* 功能描述：CTS02 擦除FLASH 最后页
* 输入参数：
* 返回值   ：ret
* 其它说明：CTS02
*******************************************************************************/
static int byd_ctpm_last_page_erase(struct i2c_client *client)
{
	int ret;
	unsigned char CTPM_FLASH[10];
	i2c_master_send(client, I2C_FLASH_PAGE31_ADDR, sizeof(I2C_FLASH_PAGE31_ADDR));
	i2c_master_send(client, I2C_ERASE_FLASH_PAGE, sizeof(I2C_ERASE_FLASH_PAGE));
	msleep(30);
	i2c_master_send(client, I2C_ERASE_OVER, 2);
	msleep(10);
	ret = byd_ctp_read_flash(client, I2C_FLASH_PAGE31_ADDR, sizeof(I2C_FLASH_PAGE31_ADDR), I2C_BUFFER_DATA, CTPM_FLASH, bin);  //19
	if(ret > 0)
		printk("%s:  over \n", __FUNCTION__);
	else
		printk("%s:  error \n", __FUNCTION__);
	return ret;
}
/*******************************************************************************
* 函数名称：static void byd_ctpm_fw_online_exit(struct i2c_client *client)
* 功能描述：在线编程退出判断
* 输入参数：client
* 返回值   ：ret
* 其它说明：CTS02
*******************************************************************************/
static int byd_ctpm_fw_online_exit(struct i2c_client *client)
{
	unsigned char IICBufOnlineR[10];

	//在线编程结束指令
	i2c_master_send(client,I2C_ONLINE_EXIT_DATA, 10);
	printk("%s: \n", __FUNCTION__);
	//判断在线编程结束指令
	msleep(20);
	//msleep(1);
	i2c_master_send(client, I2C_ONLINE_STATE_ADDR, sizeof(I2C_ONLINE_STATE_ADDR));
	i2c_master_recv(client, IICBufOnlineR, sizeof(I2C_ONLINE_STATE_DATA));
	if (IICBufOnlineR[0] == I2C_ONLINE_STATE_DATA[0])
	{ //如果读到的数是 0xab9d573b ,则未退出，需重新退出
		printk("IN ONLINE MODE, out error \n");
		return -1;
	}
	else if(IICBufOnlineR[0] == 0x00)
	{//读到的是0x0a7558zz则已经退出
		printk("%s: ok\n", __FUNCTION__);
		return 1;
	}
	else
	{
		printk("%s: error\n", __FUNCTION__);
		return -1;
	}
}

/*******************************************************************************
* 函数名称：static int byd_ctpm_page_erase(struct i2c_client *client,uint8_t *address)
* 功能描述：页擦除函数
* 输入参数：address: 擦除的起始页地址
* 返回值   ：ret
* 其它说明：CTS02
*******************************************************************************/
static int byd_ctpm_page_erase(struct i2c_client *client, uint8_t *address)
{
	int  i, ret;
	 unsigned char CTPM_FLASH[10];
	//0x810000a505xxxx
	printk("%s: start \n", __FUNCTION__);
	byd_ctpm_crc_get(address);
	i2c_master_send(client,  address, 7);
	//0x8100005ac1xxxx
	i2c_master_send(client, I2C_ERASE_FLASH_PAGE, 7);
	printk("PROG_ERASE_CODE ok\n");
	msleep(40);
	ret = byd_ctp_read_flash(client,address, 7, I2C_BUFFER_DATA, CTPM_FLASH, para);  //19
	if(ret > 0)
		printk("byd_ctpm_page_erase  over\n");
	else
		printk("byd_ctpm_page_erase  error\n");
	return ret;
}

/*******************************************************************************
* 函数名称：unsigned int  byd_ctpm_write_para(struct i2c_client *client,uint8_t *address,u8* pbt_buf, u16 dw_lenth)
* 功能描述：参数写入函数
* 输入参数：address: 写数据输入地址
				   pbt_buf:写入的数据buffer
				   dw_lenth: 数据的长度
* 返回值   ：ret
* 其它说明：CTS102
*******************************************************************************/
static unsigned int byd_ctpm_write_para(struct i2c_client *client, uint8_t *address, u8* pbt_buf, u16 dw_lenth)
{
	int  i, ret;
	unsigned char IICBufOnlineW[10];
	unsigned char CTPM_FLASH[10];

	printk("%s:  start \n", __FUNCTION__);
	//0x810000a568xxxx    //指向Flash 最后配置字 地址
	i2c_master_send(client, address, 7);
	for( i = 0; i < dw_lenth; i++ )
	{
		IICBufOnlineW[0] = 0x80;
		IICBufOnlineW[1] = pbt_buf[4 * i];//写数据
		IICBufOnlineW[2] = pbt_buf[4 * i + 1];
		IICBufOnlineW[3] = pbt_buf[4 * i + 2];
		IICBufOnlineW[4] = pbt_buf[4 * i + 3];
		byd_ctpm_crc_get(IICBufOnlineW);
        	i2c_master_send(client, IICBufOnlineW, 7);
		i2c_master_send(client, I2C_WRITE_FLASH, 7);
		udelay(100);
		i2c_master_send(client, I2C_ADDR_ADD_1, 7);
	}
	// 写配置字00
	ret = byd_ctp_read_flash(client,address, 7, pbt_buf, CTPM_FLASH, para);  //19
	if(ret > 0)
	{
		printk("byd_ctpm_write_para  over \n");
	}
	return ret;
}

/*******************************************************************************
* 函数名称tatic int byd_ctpm_main_erase_write(struct i2c_client *client,u8* pbt_buf,u16 dw_lenth,int ret)
* 功能描述：all flash  erase and write
* 输入参数：
* 返回值   ：ret
* 其它说明：CTS02
*******************************************************************************/
static int byd_ctpm_main_erase_write(struct i2c_client *client, u8* pbt_buf, u16 dw_lenth)
{
	int ret;		//all FLASH
	ret = byd_ctpm_main_erase(client);
	if (ret < 0)
	{
		return ret;
	}
	ret = byd_ctpm_fw_upgrade(client, I2C_FLASH_START_ADDR, sizeof(I2C_FLASH_START_ADDR), pbt_buf, dw_lenth);
	return ret;
}
int read_flash_code_mark(struct i2c_client *client, uint8_t *comparedata)
{
	unsigned char code_mark[4] = {0};
	int ret;	
	ret = byd_ctp_read_byte(client, I2C_FLASH_END_ADDR, sizeof(I2C_FLASH_END_ADDR), comparedata, code_mark, 0);
}

/*******************************************************************************
* 函数名称byd_ts_upgrade_firmware
* 功能描述:II2C烧录入口函数
* 输入参数：
* 返回值   ：ret
* 其它说明：无
*******************************************************************************/
int byd_ts_upgrade_firmware(struct i2c_client *client)
{
	int i, ret = -1;
	static unsigned char CTPM_FW[16 * 1024] = {0}; // reverse count 7: 1e
	static unsigned char CTPM_PARA[512];  //参数个数大于512时需修改
	//unsigned char CONFIG0[4] = {0x01,0x02, 0x03, 0x04};
	int para_sz = 0;
	int firm_sz = 0;
	int ui_sz = 15360; //32K //15872 = 15*1024+512;
	unsigned char flag = 0x00;
	int module_num = 0;
	unsigned char version[] = {0, 0, 0, 0, 0, 0};
	pr_info("*** %s ***\n", __func__);
		
#ifdef CONFIG_SUPPORT_MODULE_ID
	module_num = byd_ts_get_module_number(client);
	if (module_num <= 0) {
		printk("%s: module number default to 1. \n", __FUNCTION__);
		module_num = 1;
	}
#else
	module_num = 1;
#endif

#ifdef PARA_BURNING_FILE
	para_sz = byd_ts_read_file_with_id(PARA_BURNING_FILE, module_num, CTPM_PARA, 512, 0);
	if (para_sz > 0) {
		flag = flag | 0x02;
	}
#endif
#ifdef FIRM_BURNING_FILE
	//firm_sz = byd_ts_read_file(FIRM_BURNING_FILE, &CTPM_FW[0], ui_sz, 0);
	firm_sz = byd_ts_read_file_with_id(FIRM_BURNING_FILE , module_num, CTPM_FW, ui_sz, 0);
	//pr_info("FIRM_BURNING_FILE = %x, %x, %x, %x, %x \n", CTPM_FW[0], CTPM_FW[1], CTPM_FW[2], CTPM_FW[3], CTPM_FW[4]);
	if (firm_sz < 0) {
		TS_DBG("FIRM_BURNING_FILE %s opening failed!\n", FIRM_BURNING_FILE);
	} else {
		flag = flag | 0x01;
	}
#endif

#if 0 //def VERSION_IDENTIFY	
	//=============FW upgrade====================*/
	ret = byd_ts_get_firmware_version(client, version);
	if (ret <= 0) {
			ret = byd_ts_get_firmware_version(client, version);		
	} 

	printk("%s: TP firmware version: %02x%02x-%02x%02x-%.2x%.2x, ret = %d \n", __FUNCTION__, // ##-PROJ.CO.PA
	version[0], version[1], version[2], version[3], version[4], version[5], ret);	
	if (ret <= 0) 
	{
		pr_err("%s: i2c read MODULE ID error, errno: %d\n", __FUNCTION__, ret);
	}
	else
	{
		if((version[4] >= CTPM_PARA[6] ) && (version[5] >= CTPM_PARA[7]))
		{
			TS_DBG(" TP firmware is the latest version \n");
			ret = -2;
			goto output;
		}
		else{
			TS_DBG("TP version: %x %x, TP file version: %x, %x \n", version[4], version[5], CTPM_PARA[6], CTPM_PARA[7]);
		}
		
	}
#endif


	if (flag == 0x00) {
		return ret;
	}

	client->addr = I2C_CTPM_ADDRESS;

//version update to upgrade firmware	
	//进入在线编程
	//ret = ctpm_fw_online(client, ret);
	ret = byd_ctpm_fw_online(client);
	if(ret < 0)
	{
		goto output;
	}
	//读config0
//	ret = read_flash_code_mark(client, CONFIG0);

	//烧录全部flash
	if((flag & 0x01) == 1)
	{
		ret = byd_ctpm_main_erase_write(client, CTPM_FW, firm_sz);
		if(ret < 0)
		{
			goto output;
		}
		byd_ts_rename_file(client, FIRM_BURNING_FILE, module_num);
	}

	if((flag & 0x02) == 2)
	{
		ret = byd_ctpm_last_page_erase(client);
		if(ret < 0)
		{
			goto output;
		}

		ret = byd_ctpm_fw_upgrade(client, I2C_FLASH_PAGE31_ADDR, sizeof(I2C_FLASH_PAGE31_ADDR), CTPM_PARA, para_sz);
		if(ret < 0)
		{
			goto output;
		}
		byd_ts_rename_file(client, PARA_BURNING_FILE, module_num); 
	}

	//byd_ctpm_fw_upgrade(client, I2C_FLASH_END_ADDR, sizeof(I2C_FLASH_END_ADDR), CONFIG0, 4);
	//read_flash_code_mark(client, CONFIG0);
	ret = byd_ctpm_fw_online_exit(client);

output:
	client->addr =BYD_TS_SLAVE_ADDRESS; // i2c_addr;
	msleep(250);
	return ret;
}

#endif // ..._FIRMWARE_UPG
/*static const struct i2c_device_id byd_ts_id[] = {
	{ BYD_TS_NAME, 0 },
	{ }
};*/




