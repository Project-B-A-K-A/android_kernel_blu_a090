/*
* Copyright (C) 2012 Spreadtrum Communications Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
	
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#ifndef CONFIG_64BIT
#include <soc/sprd/hardware.h>
#include <soc/sprd/board.h>
#endif
#include <soc/sprd/adi.h>
#include "../common/parse_hwinfo.h"


#define KPLED_CTL               (ANA_REGS_GLB_BASE + 0xf4)

#define KPLED_V_SHIFT           12
#define KPLED_V_MSK             (0x0F << KPLED_V_SHIFT)
#define LDO_KPLED_PD			(1 << 8)
#define LDO_KPLED_V_SHIFT		(0x00)
#define LDO_KPLED_V_MSK			(0xff)
#define KPLED_PD            	(1 << 11)
#define KPLED_PULLDOWN_EN		(1 << 10)


#define brightness_level  0x06    //30mA

int sprd_front_flash_on(void)
{
	printk("sprd_kpled_flash_on \n");
	sci_adi_clr(KPLED_CTL, KPLED_PD);
	sci_adi_write(KPLED_CTL, ((brightness_level << KPLED_V_SHIFT) & KPLED_V_MSK), KPLED_V_MSK);
	return 0;
}

int sprd_front_flash_high_light(void)
{
	printk("sprd_kpled_flash_high_light \n");
	sci_adi_clr(KPLED_CTL, KPLED_PD);
    sci_adi_write(KPLED_CTL, ((brightness_level << KPLED_V_SHIFT) & KPLED_V_MSK), KPLED_V_MSK);
	return 0;
}

int sprd_front_flash_close(void)
{
	printk("sprd_kpled_flash_close \n");
//	sci_adi_set(KPLED_CTL, KPLED_PULLDOWN_EN);
	//sci_adi_set(KPLED_CTL, LDO_KPLED_PD);
	sci_adi_set(KPLED_CTL, KPLED_PD);
	return 0;
}

int sprd_flash_cfg(struct sprd_flash_cfg_param *param, void *arg)
{
	return 0;
}
