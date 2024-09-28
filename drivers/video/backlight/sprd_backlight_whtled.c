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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/earlysuspend.h>
#include <soc/sprd/sci_glb_regs.h>
#include <soc/sprd/adi.h>
#ifndef CONFIG_64BIT
#include <soc/sprd/hardware.h>
#endif
#include <linux/gpio.h>
#include <linux/err.h>

#include <linux/of.h>
#include <linux/delay.h>
#include <soc/sprd/board.h>


//#define SPRD_BACKLIGHT_DBG
#ifdef SPRD_BACKLIGHT_DBG
#define ENTER printk(KERN_INFO "[SPRD_BACKLIGHT_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_BACKLIGHT_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_BACKLIGHT_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif


#define SPRD_PWM0_CTRL_OFST                       0xEC
#define SPRD_PWM0_PATTERN_HIGHT_OFST              0x10
#define SPRD_PWM0_PATTERT_LOW_OFST                0xC
#define SPRD_PWM0_TONE_OFST                       0x8
#define SPRD_PWM0_RATION_OFST                     0x4
#define SPRD_WHTLED_CTRL_OFST                     0xF0

struct sprd_bl_devdata {
	int suspend;
	int brightness_max;
	int brightness_min;
	unsigned int led_level;
	unsigned int ib_trim_cal_data;
	struct clk *clk;
	struct backlight_device *bldev;
	struct early_suspend sprd_early_suspend_desc;
};

static struct sprd_bl_devdata sprdbl = {
        .brightness_max = 0,
        .brightness_min = 0,
	};

static void sprd_white_led_off(void)
{
	/*DISABLE WHTLED*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x01FE);
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x1);

	/*DISABLE THE PWM0 CONTROLLTER: RTC_PWM0_EN=0 & PWM0_EN=0*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xC000);

	/*ENABLE PWM0 OUTPUT: PWM0_EN=0*/
	sci_adi_clr(ANA_PWM_BASE, 0x100);
}

	extern int sci_efuse_ib_trim_get(unsigned int *p_cal_data);
  static unsigned int ib_trim_cal_data = 0;
  static int init_flag = 1;

static void sprd_white_led_set_brightness(u32 level)
{

        if (init_flag && sci_efuse_ib_trim_get(&ib_trim_cal_data)) {
                sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x7F << 9);
                sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, (ib_trim_cal_data & 0x7F) << 9);
                sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, (0x1 << 11));
                init_flag = 0;
                printk("parallel sprd_flash_on trim = %d\n", ib_trim_cal_data);
        }

	
	/*ENABLE THE PWM0 CONTROLLTER: RTC_PWM0_EN=1 & PWM0_EN=1*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xFFFF);
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xC005);

	/*ENABLE PWM0 OUTPUT: PWM0_EN=1*/
	sci_adi_set(ANA_PWM_BASE, 0x100);

	/*SET LOW LIGHT */
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x07E); // modify by jinq for not clear calibration bit [15~9] form SC273G
#if defined(ZCFG_BACKLIGHT_WHTLED_LEVEL)
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, ((level*ZCFG_BACKLIGHT_WHTLED_LEVEL/0xff)<<1));	//(n*0.625+5)*4, 0xc: 50mA
#else
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, ((level*0x2c/0xff)<<1));	//(n*0.625+5)*4,   0x3f *0.7 =0x2c  0x3f is maximum
#endif
	/*ENABLE WHTLED*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x1);
}

static int sprd_bl_whiteled_update_status(struct backlight_device *bldev)
{
        u32 led_level;

        if ((bldev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) ||
            bldev->props.power != FB_BLANK_UNBLANK ||
            sprdbl.suspend ||
            bldev->props.brightness == 0) {
                /* disable backlight */
		sprd_white_led_off();
                PRINT_INFO("disabled\n");
        } else {
			led_level = bldev->props.brightness & 0xff;
			sprdbl.led_level = led_level;
			sprd_white_led_set_brightness(led_level);
			PRINT_INFO("brightness = %d, led_level = %d\n", bldev->props.brightness, led_level);
        }
        return 0;
}

static int sprd_bl_whiteled_get_brightness(struct backlight_device *bldev)
{
	return sprdbl.led_level;
}


static const struct backlight_ops sprd_backlight_whiteled_ops = {
       .update_status = sprd_bl_whiteled_update_status,
       .get_brightness = sprd_bl_whiteled_get_brightness,
};

#ifdef CONFIG_EARLYSUSPEND

static void sprd_backlight_earlysuspend(struct early_suspend *h)
{

	sprdbl.suspend = 1;
	sprdbl.bldev->ops->update_status(sprdbl.bldev);
	PRINT_INFO("early suspend\n");
}

static void sprd_backlight_lateresume(struct early_suspend *h)
{
	mdelay(100);
	sprdbl.suspend = 0;
	sprdbl.bldev->ops->update_status(sprdbl.bldev);
	PRINT_INFO("late resume\n");
}
#endif
extern int sci_efuse_ib_trim_get(unsigned int *p_cal_data);
static int sprd_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev;
	int use_pwm = 0;
	
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 0xff;
	props.type = BACKLIGHT_RAW;
	/*the default brightness = 1/2 max brightness */
	props.brightness = 0xff >> 1;
	props.power = FB_BLANK_UNBLANK;

	bldev = backlight_device_register(
			pdev->name, &pdev->dev,
			&sprdbl, &sprd_backlight_whiteled_ops, &props);
	if (IS_ERR(bldev)) {
		printk(KERN_ERR "Failed to register backlight device\n");
		return -ENOMEM;
	}

	sprdbl.bldev = bldev;
	platform_set_drvdata(pdev, bldev);

	if(sci_efuse_ib_trim_get(&sprdbl.ib_trim_cal_data))
	{
		sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x7F << 9);
		sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, (sprdbl.ib_trim_cal_data & 0x7F) << 9);
		sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, (0x1 << 11));
	}

		/*ENABLE THE PWM0 CONTROLLTER: RTC_PWM0_EN=1 & PWM0_EN=1*/
	//sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xFFFF);
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xC005);


	/*SET PWM0 PATTERN HIGH*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_PATTERN_HIGHT_OFST, 0xFFFF);

	/*SET PWM0 PATTERN LOW*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_PATTERT_LOW_OFST, 0xFFFF);

	/*TONE DIV USE DEFAULT VALUE*/
	sci_adi_clr(ANA_PWM_BASE + SPRD_PWM0_TONE_OFST, 0x0100);

	/*SET PWM0 DUTY RATIO = 100%: MOD=FF & DUTY=FF*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_RATION_OFST, 0x0100);

	//

	bldev->ops->update_status(bldev);

#ifdef CONFIG_EARLYSUSPEND
	sprdbl.sprd_early_suspend_desc.level	= EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	sprdbl.sprd_early_suspend_desc.suspend	= sprd_backlight_earlysuspend;
	sprdbl.sprd_early_suspend_desc.resume	= sprd_backlight_lateresume;
	register_early_suspend(&sprdbl.sprd_early_suspend_desc);
#endif

	return 0;
}

static int sprd_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bldev;

	bldev = platform_get_drvdata(pdev);
	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.brightness = 0xff;

	backlight_update_status(bldev);

	backlight_device_unregister(bldev);

	platform_set_drvdata(pdev, NULL);

#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&sprdbl.sprd_early_suspend_desc);
#endif
	return 0;
}

static void sprd_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bldev;

	bldev = platform_get_drvdata(pdev);
	bldev->props.brightness = 0;
	backlight_update_status(bldev);
}


#ifdef CONFIG_PM
static int sprd_backlight_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int sprd_backlight_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define sprd_backlight_suspend NULL
#define sprd_backlight_resume NULL
#endif

static const struct of_device_id backlight_of_match[] = {
	{ .compatible = "sprd,sprd_backlight", },
	{ }
};
static struct platform_driver sprd_backlight_driver = {
	.probe = sprd_backlight_probe,
	.remove = sprd_backlight_remove,
	.suspend = sprd_backlight_suspend,
	.resume = sprd_backlight_resume,
	.shutdown = sprd_backlight_shutdown,
	.driver = {
		.name = "sprd_backlight",
		.owner = THIS_MODULE,
		.of_match_table = backlight_of_match,
	},
};

static int __init sprd_backlight_init(void)
{
	return platform_driver_register(&sprd_backlight_driver);
}

static void __exit sprd_backlight_exit(void)
{
	platform_driver_unregister(&sprd_backlight_driver);
}

module_init(sprd_backlight_init);
module_exit(sprd_backlight_exit);

MODULE_DESCRIPTION("Spreadtrum backlight Driver");
