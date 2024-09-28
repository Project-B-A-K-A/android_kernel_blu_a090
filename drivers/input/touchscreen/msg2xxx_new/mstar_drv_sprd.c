////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_sprd.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif //CONFIG_HAS_EARLYSUSPEND
#include <linux/i2c.h>
#include <linux/kobject.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "mstar_drv_platform_interface.h"

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <mach/regulator.h>
//#include <soc/sprd/regulator.h>
#include <linux/regulator/consumer.h>
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

#ifdef CONFIG_I2C_SPRD
//#include <mach/i2c-sprd.h>
#endif


#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

extern int tp_device_id(int id);
extern void ctp_lock_mutex(void);
extern void ctp_unlock_mutex(void);
/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "msg2xxx" //"msg21xxA" or "msg22xx" or "msg26xxM" or "msg28xx" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */

/*=============================================================*/
// VARIABLE DEFINITION
/*=============================================================*/

struct i2c_client *g_I2cClient = NULL;

struct msg2xxx_ts_platform_data *g_msg2xxx_ts = NULL;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
struct regulator *g_ReguVdd = NULL;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

#ifdef CONFIG_OF
static struct msg2xxx_ts_platform_data *msg2xxx_ts_parse_dt(struct device *dev)
{

	struct msg2xxx_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Could not allocate struct msg2133_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		pr_err("fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		pr_err("fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		pr_err("fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", &pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
	}
#endif

/* probe function is used for matching and initializing input device */
static int /*__devinit*/ touch_driver_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
//    const char *vdd_name = "vdd";
	int err = 0;

#ifdef CONFIG_OF
	struct device_node		*np = client->dev.of_node;
#endif
	struct msg2xxx_ts_platform_data *pdata = client->dev.platform_data;
  //  g_msg2xxx_ts = client->dev.platform_data;
		
    printk("*** %s ***\n", __FUNCTION__);
    
    if (client == NULL)
    {
        printk("i2c client is NULL\n");
        return -1;
    }
    g_I2cClient = client;

#ifdef CONFIG_OF
	if (np && !pdata){
		pdata = msg2xxx_ts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
			g_msg2xxx_ts = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif


#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    g_ReguVdd = regulator_get(&g_I2cClient->dev, g_I2cClient->dev.platform_data->vdd_name);
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    return MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);
#ifdef CONFIG_OF
exit_alloc_platform_data_failed:
#endif
	printk("%s: Probe Fail!\n",__func__);
    return err;
}

/* remove function is triggered when the input device is removed from input sub-system */
static int /*__devexit*/ touch_driver_remove(struct i2c_client *client)
{
    printk("*** %s ***\n", __FUNCTION__);

    return MsDrvInterfaceTouchDeviceRemove(client);
}

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id touch_device_id[] =
{
    {MSG_TP_IC_NAME, 0},
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, touch_device_id);

static  struct of_device_id touch_dt_match_table[] = {
    { .compatible = "Mstar,msg2xxx",},
    {},
};

MODULE_DEVICE_TABLE(of, touch_dt_match_table);

static struct i2c_driver touch_device_driver =
{
    .driver = {
        .name = MSG_TP_IC_NAME,
        .owner = THIS_MODULE,
        .of_match_table = touch_dt_match_table,
    },
    .probe = touch_driver_probe,
    .remove = touch_driver_remove,
    .id_table = touch_device_id,
};

static int /*__init*/ touch_driver_init(void)
{
    int ret;

	if(tp_device_id(0)!=0)
	{
		printk("msg2xxx CTP(0x%x)Exist!", tp_device_id(0));
		return -ENODEV;
	}

    /* register driver */
    ret = i2c_add_driver(&touch_device_driver);
    if (ret < 0)
    {
        printk("add MStar touch device driver i2c driver failed.\n");
        return -ENODEV;
    }
    printk("add MStar touch device driver i2c driver.\n");

    return ret;
}

static void /*__exit*/ touch_driver_exit(void)
{
    printk("remove MStar touch device driver i2c driver.\n");

    i2c_del_driver(&touch_device_driver);
}

module_init(touch_driver_init);
module_exit(touch_driver_exit);
MODULE_LICENSE("GPL");