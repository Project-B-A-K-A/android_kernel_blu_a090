/*
 * aw3215a.c  --  AW3215A Switching Charger driver
 *
 * Copyright (C) 2012 Fairchild semiconductor Co.Ltd
 * Author: Bright Yang <bright.yang@fairchildsemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
//#include <mach/hardware.h>
//#include <mach/adi.h>
//#include <mach/adc.h>
//#include <mach/gpio.h>
#include <soc/sprd/board.h>
#include <linux/device.h>

#include <linux/slab.h>
#include <linux/jiffies.h>
#include "sprd_battery.h"
//#include <mach/usb.h>


#if (defined(GPIO_EXT_CHG_EN)&& defined(GPIO_EXT_CHG_STAT))
struct sprdchg_drivier_data_ext {
	uint32_t gpio_vchg_ovi;
	uint32_t irq_vchg_ovi;
	uint32_t gpio_charge_en;
	uint32_t gpio_charge_done;
	struct work_struct ovi_irq_work;
};

static struct sprdchg_drivier_data_ext sprdchg_ext_ic;
extern int in_calibration(void);


int sprdchg_is_chg_done_ext(void)
{
	return gpio_get_value(sprdchg_ext_ic.gpio_charge_done);
}

void sprdchg_start_charge_ext(void)
{
	if (in_calibration()) 
	gpio_set_value(sprdchg_ext_ic.gpio_charge_en, 1);
	else
	gpio_set_value(sprdchg_ext_ic.gpio_charge_en, 0);
	printk("sprdchg: aw3215a sprdchg_start_charge_ext\n");
	
}
void sprdchg_stop_charge_ext(void)
{
	gpio_set_value(sprdchg_ext_ic.gpio_charge_en, 1);
	printk("sprdchg: aw3215a sprdchg_stop_charge_ext\n");
	
}

void sprdchg_open_ovi_fun_ext(void)
{
	irq_set_irq_type(sprdchg_ext_ic.irq_vchg_ovi, IRQ_TYPE_LEVEL_HIGH);
	enable_irq(sprdchg_ext_ic.irq_vchg_ovi);
}

void sprdchg_close_ovi_fun_ext(void)
{
	disable_irq_nosync(sprdchg_ext_ic.irq_vchg_ovi);
}

static void sprdchg_ovi_irq_works_ext(struct work_struct *work)
{
	int value;
	value = gpio_get_value(sprdchg_ext_ic.gpio_vchg_ovi);
	if (value) {
		sprdbat_charge_event_ext(SPRDBAT_CHG_EVENT_EXT_OVI);
	} else {
		sprdbat_charge_event_ext(SPRDBAT_CHG_EVENT_EXT_OVI_RESTART);
	}
}

static __used irqreturn_t sprdchg_vchg_ovi_irq_ext(int irq, void *dev_id)
{
	int value;
	value = gpio_get_value(sprdchg_ext_ic.gpio_vchg_ovi);
	if (value) {
		printk("sprdchg_vchg_ovi_irq_ext ovi high\n");
		irq_set_irq_type(sprdchg_ext_ic.irq_vchg_ovi, IRQ_TYPE_LEVEL_LOW);
	} else {
		printk("sprdchg_vchg_ovi_irq_ext ovi low\n");
		irq_set_irq_type(sprdchg_ext_ic.irq_vchg_ovi, IRQ_TYPE_LEVEL_HIGH);
	}
	schedule_work(&sprdchg_ext_ic.ovi_irq_work);
	return IRQ_HANDLED;
}

int sprdchg_charge_init_ext(struct platform_device *pdev)
{
	int ret = -ENODEV;
	int i=0;
	if (in_calibration()) 
	{
	printk("calibration mode, quit...\n");
	gpio_request(GPIO_EXT_CHG_EN, "charge_en_ext");
	gpio_direction_output(GPIO_EXT_CHG_EN, 1);
	gpio_set_value(GPIO_EXT_CHG_EN, 1);
	return ret;
	}


	sprdchg_ext_ic.gpio_charge_en = GPIO_EXT_CHG_EN;
	sprdchg_ext_ic.gpio_charge_done = GPIO_EXT_CHG_STAT;
/*
	sprdchg_ext_ic.gpio_vchg_ovi = GPIO_EXT_CHG_STAT;
	gpio_request(sprdchg_ext_ic.gpio_vchg_ovi, "vchg_ovi_ext");
	gpio_direction_input(sprdchg_ext_ic.gpio_vchg_ovi);
	sprdchg_ext_ic.irq_vchg_ovi = gpio_to_irq(sprdchg_ext_ic.gpio_vchg_ovi);
	set_irq_flags(sprdchg_ext_ic.irq_vchg_ovi, IRQF_VALID | IRQF_NOAUTOEN);
	ret = request_irq(sprdchg_ext_ic.irq_vchg_ovi, sprdchg_vchg_ovi_irq_ext,
			IRQF_NO_SUSPEND, "sprdbat_vchg_ovi_ext", NULL);
	INIT_WORK(&sprdchg_ext_ic.ovi_irq_work, sprdchg_ovi_irq_works_ext);
*/
	gpio_request(sprdchg_ext_ic.gpio_charge_done, "chg_done_ext");
	gpio_direction_input(sprdchg_ext_ic.gpio_charge_done);
	gpio_request(sprdchg_ext_ic.gpio_charge_en, "charge_en_ext");
	gpio_direction_output(sprdchg_ext_ic.gpio_charge_en, 1);
	gpio_set_value(sprdchg_ext_ic.gpio_charge_en, 0);
	return ret;
}

void sprdchg_chg_monitor_cb_ext(void *data)
{
	return;
}
#endif

