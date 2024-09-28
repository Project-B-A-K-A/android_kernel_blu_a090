/*
 * mtv_spi.c
 *
 * RAONTECH MTV spi driver.
 *
 * Copyright (C) (2011, RAONTECH)
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>



#include "raontv.h"
#include "raontv_internal.h"

#include "mtv.h"
#include "mtv_gpio.h"

#ifdef RTV_IF_SPI

#define SPI_DEV_NAME	"mtvspi"

#define MTV222_SPI_CMD_SIZE		3

#if defined(CONFIG_ARCH_MTKXXXX)
#include <mach/mt_spi.h>
#include <mach/mt_gpio.h>
static struct mt_chip_conf spi_conf;
struct spi_device	*mtv_spi;

int MTV_SPI_Mode(int mode)
{
	struct mt_chip_conf* spi_par;	
	spi_par =&spi_conf;
		if(!spi_par){
			printk("spi config fail\n");
			return -1;
		}
	if(1 == mode)
	{
		spi_par->com_mod = DMA_TRANSFER;
		printk("mtv change to dma mode\n");
	}
	else
	{
		spi_par->com_mod = FIFO_TRANSFER;
		printk("mtv change to fifo mode\n");
	}	
	if(spi_setup(mtv_spi)){
				printk("spi_setup fail\n");
				return -1;
	}	
	return 0;
}


#endif
#define SPI_RW_BUF (188*20)
static u8 out_buf[SPI_RW_BUF] = {0};
static u8 in_buf[SPI_RW_BUF] = {0};
static u8 out_tmp[SPI_RW_BUF] = {0};

void mtv_spi_read_burst(unsigned char page, unsigned char reg,
			unsigned char *buf, int size)
{
	int ret;
//	u8 out_buf[MTV222_SPI_CMD_SIZE];
	struct spi_message msg;
	struct spi_transfer xfer0 = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = MTV222_SPI_CMD_SIZE + size,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	struct spi_transfer xfer1 = {
		.tx_buf = out_tmp,//in_buf,
		.rx_buf = in_buf,
		.len = size,
		.cs_change = 0,
		.delay_usecs = 0,
	};
	
	memset(in_buf,0xff,size);
       memset(out_buf,0xff,size);
	out_buf[0] = 0xA0; /* Memory read */
	out_buf[1] = 0x00;
	out_buf[2] = 188; /* Fix */


	   
	spi_message_init(&msg);
	spi_message_add_tail(&xfer0, &msg);

//DMBERR("[mtv_spi_read_burst] out_buf %p in_buf %p \n",out_buf, in_buf);
	
#if defined(CONFIG_ARCH_MTKXXXX)
	ret = spi_sync(mtv_cb_ptr->spi_ptr, &msg);
	if (ret)
	{
		DMBERR("[mtv_spi_read_burst] error: %d\n", ret);	
	}
	
/*****************************************************************************/
   
/****然后修改SPI的片选进入GPIO模式，将GPIO拉低	****/
	mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SPI_CS_PIN, GPIO_OUT_ZERO);
/******************************************************************************/	
 	MTV_SPI_Mode(1);
	spi_message_init(&msg);
#endif	
	
	
	//spi_message_add_tail(&xfer1, &msg);

	ret = spi_sync(mtv_cb_ptr->spi_ptr, &msg);
	if (ret)
		DMBERR("error: %d\n", ret);	

      memcpy(buf, &in_buf[3],size);
		
#if defined(CONFIG_ARCH_MTKXXXX)
/*****************************************************************************/
/*****然后修改SPI的片选引脚进入SPI模式，SPI的片选引脚将拉高****/
	mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_SPI_CSN);
	mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SPI_CS_PIN, GPIO_OUT_ONE);
    MTV_SPI_Mode(0);
/******************************************************************************/	
#endif		

//DMBERR("[mtv_spi_read_burst out] out_buf %p in_buf %p data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",out_buf, in_buf, buf[0],buf[1],buf[2],buf[3],buf[4]);		
}

unsigned char mtv_spi_read(unsigned char page, unsigned char reg)
{
	int ret;
//	u8 out_buf[4], in_buf[4];
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = 4,
		.cs_change = 0,
		.delay_usecs = 0
	};

	out_buf[0] = 0x90 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* Read size */
       out_buf[3] = 0xff;
	
	//memset(in_buf,0xff,4);
	
	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv_cb_ptr->spi_ptr, &msg);
	if (ret) {
		DMBERR("error: %d\n", ret);  
		return 0xFF;
	}
//DMBERR("mtv_spi_read: page %d, reg 0x%x, value 0x%x\n", page, reg, in_buf[MTV222_SPI_CMD_SIZE]);  
	return in_buf[MTV222_SPI_CMD_SIZE];
}


void mtv_spi_write(unsigned char page, unsigned char reg, unsigned char val)
{
//	u8 out_buf[4];
//	u8 in_buf[4];
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = 4,
		.cs_change = 0,
		.delay_usecs = 0
	};
	int ret;

	out_buf[0] = 0x80 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* size */
	out_buf[3] = val;
	
//DMBERR("mtv_spi_write: page %d, reg 0x%x, val 0x%x\n", page, reg, val);  

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv_cb_ptr->spi_ptr, &msg);
	if (ret)
		DMBERR("error: %d\n", ret);  
}


static int mtv_spi_probe(struct spi_device *spi)
{
	int ret;
	
	DMBMSG("ENTERED!!!!!!!!!!!!!!\n");
	
#if defined(CONFIG_ARCH_MTKXXXX)

		mtv_spi = spi;
		struct mt_chip_conf* spi_par;		
		mtv_spi->controller_data =(void*)&spi_conf; 
	

        spi_par =&spi_conf;
		if(!spi_par){
			printk("spi config fail");
			return -1;
		}
		spi_par->setuptime = 10;//15
		spi_par->holdtime = 10;//15
		spi_par->high_time = 10;       //10--6m   15--4m   20--3m  30--2m  [ 60--1m 120--0.5m  300--0.2m]
		spi_par->low_time = 10;
		spi_par->cs_idletime = 20;//20

		spi_par->rx_mlsb = 1; 
		spi_par->tx_mlsb = 1;		 
		spi_par->tx_endian = 0;
		spi_par->rx_endian = 0;

		spi_par->cpol = 0;
		spi_par->cpha = 0;
		spi_par->com_mod = FIFO_TRANSFER;

		spi_par->pause = 0;
		spi_par->finish_intr = 1;
		spi_par->deassert = 0;
		if(spi_setup(mtv_spi)){
			printk("spi_setup fail");
			return -1;
		}
#endif	

	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;

	ret = spi_setup(spi);
	if (ret < 0)
	       return ret;

	mtv_cb_ptr->spi_ptr = spi;

	return 0;
}


static int mtv_spi_remove(struct spi_device *spi)
{
	return 0;
}


struct spi_driver mtv_spi_driver = {
	.driver = {
		.name = SPI_DEV_NAME,
		.owner = THIS_MODULE,
	},

	.probe    = mtv_spi_probe,
	.suspend	= NULL,
	.resume 	= NULL,
	.remove	= __devexit_p(mtv_spi_remove),
};
#endif /* #ifdef RTV_IF_SPI */

