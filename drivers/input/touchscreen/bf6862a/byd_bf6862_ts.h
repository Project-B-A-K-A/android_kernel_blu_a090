/*
 * File:         byd_bfxxxx_ts.h
 *
 * Created:	     2012-10-07
 * Depend on:    byd_bfxxxx_ts.c
 * Description:  BYD TouchScreen IC driver for Android
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
#ifndef __BFXXXX_TS_H__
#define __BFXXXX_TS_H__


/*******************************************************************************
*Part 1: base setting
*******************************************************************************/
/* 1.debug control*/
	  #define TS_DEBUG_MSG

/* 2.  */
	//#define CONFIG_DRIVER_BASIC_FUNCTION 

/* 3.  IC choose */
	//#define CONFIG_CTS01  //CTS01: BF6852A1, BF6852A2, BF6852C, BF6853A1. include CTS11C
	  #define CONFIG_CTS02  //CTS02: BF6862A1, BF6863A1

/* 4. */
	//#define CONFIG_FIRMWARE_PRE_VERSION_2_0

/* 5. Android I2C Protocol selection, Protocol A */
	  #define CONFIG_ANDROID_4_ICS
    /* Protocol B with ID */
	 // #define CONFIG_REPORT_PROTOCOL_B // 
    /*  */

	//#define CONFIG_REMOVE_POINT_STAY // byd driver


/* 6. PROXIMITY support     */
	//#define CONFIG_TOUCHSCREEN_WITH_PROXIMITY

/* 7. gesture function  */
	//#define BYD_GESTURE 1

	 #define BYD_TS_GESTURE_KEYS { \ 
	 	KEY_U,\
		KEY_D,\
		KEY_L,\
		KEY_R,\
		KEY_W,\
		KEY_M,\
		KEY_C,\
		KEY_O,\	
		KEY_Z,\	
		NULL,\
		NULL,\
		NULL,\
		NULL,\
		NULL,\
		NULL,\
		NULL,\
		NULL,\
		NULL,\
		KEY_POWER,\			
	  }

/* 8. keys*/
	  #define BYD_TS_KEYS { \ 
			KEY_BACK, \
			KEY_HOMEPAGE, \
			KEY_MENU, \
	/*    KEY_SEARCH */ \
	  }
	 
	  
/* 9. version */
	  #define BYD_TS_DRIVER_DESCRIPTION "BYD Touchscreen Driver, Version: " \
	      "CTS01-BF6862A1-COF2.0-DRV2.10-CLIENT0000-PROJ00-PARA1.0"
	  #define BYD_TS_DRIVER_VERSION_CODE	0x0210 // version  2.10

struct bf6862a_ts_platform_data{
	int irq_gpio_number;
	int reset_gpio_number;
	const char *vdd_name;
	int virtualkeys[12];
	int TP_MAX_X;
	int TP_MAX_Y;
};
/*******************************************************************************
* 
*******************************************************************************/
#ifndef CONFIG_DRIVER_BASIC_FUNCTION

/*version compare control*/
 //#define VERSION_IDENTIFY
	#define RAWDATA_FILE	"/data/local/tmp/rawdata_file.bin"
	#define BASELINE_FILE	"/data/local/tmp/baseline_file.bin"
#endif // ndef CONFIG_DRIVER_BASIC_FUNCTION

/*******************************************************************************
*part 3： platform setting 
*******************************************************************************/

/* 1. 配置中断 */

   /* interrupt gpio number */
	  #define GPIO_TS_EINT  GPIO_TOUCH_IRQ //S5PV210_GPH0(1) //S5PV210_GPH1(7) // GPIO_TS_WAKE port is used instead // S5PV210_GPH0(5) 

   /* IRQ number */
	#define EINT_NUM	gpio_to_irq(GPIO_TS_EINT)  // 也可以是个常数

   /* TRIGGER	 TYPE, example:    IRQF_TRIGGER_FALLING, IRQF_TRIGGER_LOW,
      IRQF_TRIGGER_RISING, IRQF_TRIGGER_HIGH */
	  #define EINT_TRIGGER_TYPE  IRQF_TRIGGER_FALLING   //IRQF_TRIGGER_RISING //IRQF_TRIGGER_FALLING

/* 2. I2C address */
	#define BYD_TS_SLAVE_ADDRESS	0x2c  //0x2c // 0x2c/0x58, 0x43/0x86(backup), 0x5d/0xba(old mc)
	#define   I2C_CTPM_ADDRESS		0x7e
/* 3. driver name  */
	  #define BYD_TS_NAME		"bfxxxx_ts"

/* 4. 4*x */
	//#define I2C_PACKET_SIZE  8 // 32

#ifndef CONFIG_NON_BYD_DRIVER // 

#include <linux/gpio.h>         // gpio_request(), gpio_to_irq()
//#include <linux/irq.h>          // irq_set_irq_type(), IRQ_TYPE_...

/*******************************************************************************
* Function    :  byd_ts_platform_init
* Description :  platform specfic configration.
*                This function is usually used for interrupt pin or I2C pin
                 configration during initialization of TP driver.
* Parameters  :  client, - i2c client or NULL if not appropriate
* Return      :  Optional IRQ number or status if <= 0
*******************************************************************************/
static int byd_ts_platform_init(struct i2c_client *client) {
    int irq = 0;

	printk("%s: entered \n", __FUNCTION__);

    /* config port for IRQ */
    if(gpio_request(GPIO_TS_EINT, "ts-interrupt") != 0) {
		printk("%s:GPIO_TS_EINT gpio_request() error\n", __FUNCTION__);
		return -EIO;
	}
    gpio_direction_input(GPIO_TS_EINT);     // IO configer as input port

#ifdef EINT_NUM
    /* (Re)define IRQ if not given or value of "client->irq" is not correct */
    irq = EINT_NUM;
    /* if the IRQ number is already given in the "i2c_board_info" at
     platform end, this will cause it being overrided. */
    if (client != NULL) {
        client->irq = irq;
    }
#endif

    /** warning: irq_set_irq_type() call may cause an unexpected interrupt **/
    //irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);   // IRQ_TYPE_LEVEL_LOW

    /* Returned IRQ is used only for adding device to the bus by the driver.
       otherwise, the irq would not necessarilly to be returned here (or,
       a status zero can be returned instead) */
    return irq;
}


/*******************************************************************************

      two type:

      1.maser type
       "i2c_board_info"=
	example:
         {
          I2C_BOARD_INFO(BYD_TS_NAME, BYD_TS_SLAVE_ADDRESS),
          .irq = EINT_NUM // optional
         },

	or:

          struct platform_data {
              u16  irq;
          };
          
	2.   BUS_NUM is I2C bus number of platform,it's slave type.  */
	#define BUS_NUM		1

#endif // ifndef CONFIG_NON_BYD_DRIVER

#endif
