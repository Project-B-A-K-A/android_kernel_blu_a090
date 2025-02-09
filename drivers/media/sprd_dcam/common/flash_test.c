/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
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
/* notes : note location /sys/class/flash_test/flash_test/
 *            echo 0x11>flash_value
 * 		  cat flash_value
 */
#include "flash_test.h"
#include "parse_hwinfo.h"
#include <../flash/flash.h>
#include <linux/gpio.h>
#include <linux/gpio.h>

#define SENSOR_K_SUCCESS                  0
#define SENSOR_K_FAIL                     (-1)
#define SENSOR_K_FALSE                    0
#define SENSOR_K_TRUE                     1

static struct class* flash_test_class = NULL;
static uint32_t g_flash_mode=0;
static uint32_t g_front_flash_mode = 0;
#if defined(CONFIG_ZYT_FLASH_CTRL_BY_GPIO)
int setflash(uint32_t flash_mode, uint32_t flash_index)
{
//	printk("setflash ======================%d\n",flash_mode);
	switch (flash_mode) {
	case FLASH_OPEN:				 /*flash on */
	case FLASH_TORCH:				  /*for torch */
		/*low light */
		if (0 == flash_index) {
			gpio_direction_output(GPIO_SPRD_FLASH_LOW, SPRD_FLASH_ON);
			gpio_set_value(GPIO_SPRD_FLASH_LOW, SPRD_FLASH_ON);
			gpio_direction_output(GPIO_SPRD_FLASH_HIGH, SPRD_FLASH_OFF);
			gpio_set_value(GPIO_SPRD_FLASH_HIGH, SPRD_FLASH_OFF);
		} else {
			sprd_front_flash_on();
		}
		break;
	case FLASH_HIGH_LIGHT:
		/*high light */
		if (0 == flash_index) {
			gpio_direction_output(GPIO_SPRD_FLASH_LOW, SPRD_FLASH_ON);
			gpio_set_value(GPIO_SPRD_FLASH_LOW, SPRD_FLASH_ON);
			gpio_direction_output(GPIO_SPRD_FLASH_HIGH, SPRD_FLASH_ON);
			gpio_set_value(GPIO_SPRD_FLASH_HIGH, SPRD_FLASH_ON);
		} else {
			sprd_front_flash_high_light();
		}
		break;
	case FLASH_CLOSE_AFTER_OPEN:			  /*close flash */
	case FLASH_CLOSE_AFTER_AUTOFOCUS:
	case FLASH_CLOSE:
		/*close the light */
		if (0 == flash_index) {
			gpio_direction_output(GPIO_SPRD_FLASH_LOW, SPRD_FLASH_OFF);
			gpio_set_value(GPIO_SPRD_FLASH_LOW, SPRD_FLASH_OFF);
			gpio_direction_output(GPIO_SPRD_FLASH_HIGH, SPRD_FLASH_OFF);
			gpio_set_value(GPIO_SPRD_FLASH_HIGH, SPRD_FLASH_OFF);
		} else {
			sprd_front_flash_close();
		}
		break;
	default:
		printk("sensor set flash unknown mode:%d \n", flash_mode);
		return 0;
	}
//	printk("setflash: GPIO_flash_mode 0x%x  \n", flash_mode);
	return 0;

}

#else

int setflash(uint32_t flash_mode, uint32_t flash_index)
{
	switch (flash_mode) {
	case FLASH_OPEN:        /*flash on */
	case FLASH_TORCH:        /*for torch low light */
		if (0 == flash_index) {
			sprd_flash_on();
		} else {
			sprd_front_flash_on();
		}
		break;
	case FLASH_HIGH_LIGHT: /*high light */
		if (0 == flash_index) {
			sprd_flash_on();
		} else {
			sprd_front_flash_high_light();
		}
		break;
	case FLASH_CLOSE_AFTER_OPEN:     /*close flash */
	case FLASH_CLOSE_AFTER_AUTOFOCUS:
	case FLASH_CLOSE:
		if (0 == flash_index) {
			sprd_flash_close();
		} else {
			sprd_front_flash_close();
		}
		break;
	default:
		printk("sprd_v4l2_setflash unknow mode:flash_mode 0x%x \n", flash_mode);
		break;
	}
	return 0;
}
#endif
static ssize_t flash_test_show(struct device* dev,
			   struct device_attribute* attr,  char* buf)
{
    return snprintf(buf, PAGE_SIZE, "flash_mode is :0x%x; front_flash_mode is: 0x%xd\n", g_flash_mode, g_front_flash_mode);
}

static ssize_t flash_test_store(struct device *dev,
				struct device_attribute *attr, const char *buf,size_t c)
{
	int ret;
	unsigned long flash_mode;
	printk(KERN_ALERT"flash_test_store  buf=%s  line %d .\n",buf,__LINE__);
	ret = kstrtoul(buf, 16, &flash_mode);
	if (ret)
		return ret;
	if (0 == (flash_mode>>16)) {
		g_flash_mode=flash_mode;
	} else {
		g_front_flash_mode = flash_mode & 0xFFFF;
	}
	setflash((flash_mode&0xFFFF), (flash_mode >> 16));
	return c;
}
static DEVICE_ATTR(flash_value, 0644, flash_test_show, flash_test_store);

static int __init flash_test_init(void){
	int err = -1;
	dev_t dev_num = 0;
	int flash_test_major = 0;
	int flash_test_minor = 0;
	struct device* flash_test_dev = NULL;
	err = alloc_chrdev_region(&dev_num, 0, 1, FLASH_TEST_DEVICE_NODE_NAME);
	if (err < 0) {
	    printk(KERN_ALERT"Failed to alloc char dev region.\n");
	    goto fail;
	}
	flash_test_major = MAJOR(dev_num);
	flash_test_minor = MINOR(dev_num);
	flash_test_class = class_create(THIS_MODULE, FLASH_TEST_DEVICE_CLASS_NAME);
	if (IS_ERR(flash_test_class)) {
	    err = PTR_ERR(flash_test_class);
	    printk(KERN_ALERT"Failed to create flash_test class.\n");
	    goto destroy_class;
	}
	flash_test_dev = device_create(flash_test_class, NULL, dev_num, "%s", FLASH_TEST_DEVICE_FILE_NAME);
	if (IS_ERR(flash_test_dev)) {
	    err = PTR_ERR(flash_test_dev);
	    printk(KERN_ALERT"Failed to create flash_test device.");
	    goto destroy_class;
	}
	err = device_create_file(flash_test_dev, &dev_attr_flash_value);
	if (err < 0) {
	    printk(KERN_ALERT"Failed to create attribute val.");
	    goto destroy_device;
	}
	return 0;
destroy_device:
    device_destroy(flash_test_class, dev_num);
destroy_class:
    class_destroy(flash_test_class);
    unregister_chrdev_region(MKDEV(flash_test_major, flash_test_minor), 1);
fail:
    return err;
}
module_init(flash_test_init);
MODULE_AUTHOR("freed wang, spreadtrum");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Flash test Interface");
