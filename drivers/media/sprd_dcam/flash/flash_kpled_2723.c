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

#define high_brightness_level  0x0F    //80mA      min : 5mA          max  80mA  step 5mA


int sprd_flash_on(void)
{
	printk("sprd_kpled_flash_on \n");
	sci_adi_clr(KPLED_CTL, KPLED_PD);
	sci_adi_write(KPLED_CTL, ((brightness_level << KPLED_V_SHIFT) & KPLED_V_MSK), KPLED_V_MSK);
	return 0;
}

int sprd_flash_high_light(void)
{
	printk("sprd_kpled_flash_high_light \n");
	sci_adi_clr(KPLED_CTL, KPLED_PD);


	/*由于FS072的灯是贴片的，规格书上面写的普亮30mA，所以默认不改，如果有
	客户纠结太暗，一定要改的话，需要和硬件确认
	*/
	#if defined(ZCFG_FLASH_KELED_2723_HIGH_BRIGHTNESS_LEVEL)
	 sci_adi_write(KPLED_CTL, ((ZCFG_FLASH_KELED_2723_HIGH_BRIGHTNESS_LEVEL << KPLED_V_SHIFT) & KPLED_V_MSK), KPLED_V_MSK);
	#else
	    sci_adi_write(KPLED_CTL, ((brightness_level << KPLED_V_SHIFT) & KPLED_V_MSK), KPLED_V_MSK);
    #endif
	return 0;
}

int sprd_flash_close(void)
{
	printk("sprd_kpled_flash_close \n");
//	sci_adi_set(KPLED_CTL, KPLED_PULLDOWN_EN);
	//sci_adi_set(KPLED_CTL, LDO_KPLED_PD);
	sci_adi_set(KPLED_CTL, KPLED_PD);
	return 0;
}

//int sprd_flash_cfg(struct sprd_flash_cfg_param *param, void *arg)
//{
//	return 0;
//}
