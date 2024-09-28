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



#include <src/raontv.h>
#include <src/raontv_internal.h>

#include "mtv.h"
#include "mtv_gpio.h"

#ifdef RTV_IF_SPI

#define SPI_DEV_NAME	"mtvspi"

#define MTV222_SPI_CMD_SIZE		3

#if defined(CONFIG_ARCH_SPRDXXXX)


static u8 wdata_buf[128] __cacheline_aligned;
static u8 rdata_buf[65536] __cacheline_aligned;;


static int mtv_spi_write_then_read(struct spi_device *spi
	, u8 *txbuf, int tx_length, u8 *rxbuf, int rx_length)
{
	int ret = 0;

	struct spi_message	message;
	struct spi_transfer	x;

	if (spi == NULL) {
		DMBERR("[ERROR] MTV23X_SPI Handle Fail...........\n");
		return 0xFF;
	}

	spi_message_init(&message);
	memset(&x, 0, sizeof x);
	
	memcpy(&wdata_buf[0], txbuf, tx_length);	
	x.tx_buf = &wdata_buf[0];
	x.rx_buf = &rdata_buf[0];
	x.len = tx_length + rx_length;
	x.cs_change = 0;
	x.bits_per_word = 8;
	
	spi_message_add_tail(&x, &message);	
	ret = spi_sync(spi, &message);

	memcpy(rxbuf, x.rx_buf + tx_length, rx_length);
#if 0
	DMBMSG("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			wdata_buf[0], wdata_buf[1], wdata_buf[2], wdata_buf[3],wdata_buf[4] );
	DMBMSG("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			rdata_buf[tx_length], rdata_buf[tx_length+1], rdata_buf[tx_length+2], rdata_buf[tx_length+3],rdata_buf[tx_length+4] );
#endif
	return ret;
}

unsigned char mtv_spi_read(unsigned char page, unsigned char reg)
{
	int ret;
	u8 out_buf[4], in_buf[4];

	out_buf[0] = 0x90 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* Read size */

	ret = mtv_spi_write_then_read(mtv_cb_ptr->spi_ptr, &out_buf[0], 3, &in_buf[0], 1);

	if (ret) {
		DMBERR( "isdbt_spi_read fail : %d\n", ret);
		return 0xFF;
	}
	return in_buf[0];
}

void mtv_spi_read_burst(unsigned char page, unsigned char reg,
			unsigned char *buf, int size)
{
	int ret;
	u8 out_buf[256];
    memset(out_buf,0,128);

	out_buf[81] = 0xA0; /* Memory read */
	out_buf[82] = 0x00;
	out_buf[83] = 188; /* Fix */

#if 0	
	ret = mtv_spi_write_then_read(mtv_cb_ptr->spi_ptr, &out_buf[0], 3, buf, 0);
	if (ret) {
		DMBERR( "isdbt_spi_read_burst fail : %d\n", ret);
		return ;
	}	
#endif	
	ret = mtv_spi_write_then_read(mtv_cb_ptr->spi_ptr, &out_buf[0], 84, buf, size);
	if (ret) {
		DMBERR( "isdbt_spi_read_burst fail : %d\n", ret);
		return ;
	}		
	//mtv_mtk_spi_cspin(1);

}

void mtv_spi_write(unsigned char page, unsigned char reg, unsigned char val)
{
	u8 out_buf[4];
	u8 in_buf[4];
	int ret;

	out_buf[0] = 0x80 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* size */
	out_buf[3] = val;

	ret = mtv_spi_write_then_read(mtv_cb_ptr->spi_ptr, &out_buf[0], 4, &in_buf[0], 0);
	if (ret) {
		DMBERR( "isdbt_spi_write fail : %d\n", ret);
		return ;
	}			
		
}

#else

void mtv_spi_read_burst(unsigned char page, unsigned char reg,
			unsigned char *buf, int size)
{
	int ret;
	static u8 out_buf[MTV222_SPI_CMD_SIZE] __attribute__((aligned(8)));
	struct spi_message msg;
	struct spi_transfer xfer0 = {
		.tx_buf = out_buf,
		.rx_buf = buf,
		.len = MTV222_SPI_CMD_SIZE,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	struct spi_transfer xfer1 = {
		.tx_buf = NULL,
		.rx_buf = buf,
		.len = size,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	out_buf[0] = 0xA0; /* Memory read */
	out_buf[1] = 0x00;
	out_buf[2] = 188; /* Fix */

	spi_message_init(&msg);
	spi_message_add_tail(&xfer0, &msg);
	
#if 0
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
	
	
	spi_message_add_tail(&xfer1, &msg);

	ret = spi_sync(mtv_cb_ptr->spi_ptr, &msg);
	if (ret)
		DMBERR("error: %d\n", ret);	
		
#if 0
/*****************************************************************************/
/*****然后修改SPI的片选引脚进入SPI模式，SPI的片选引脚将拉高****/
	mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_SPI_CSN);
	mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SPI_CS_PIN, GPIO_OUT_ONE);
    MTV_SPI_Mode(0);
/******************************************************************************/	
#endif		
		
}

unsigned char mtv_spi_read(unsigned char page, unsigned char reg)
{
	int ret;
	static u8 out_buf[4] __attribute__((aligned(8)));
	static u8 in_buf[4] __attribute__((aligned(8)));
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
    out_buf[3] = 0;
	
	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv_cb_ptr->spi_ptr, &msg);
	if (ret) {
		DMBERR("error: %d\n", ret);  
		return 0xFF;
	}

	return in_buf[MTV222_SPI_CMD_SIZE];
}


void mtv_spi_write(unsigned char page, unsigned char reg, unsigned char val)
{
	static u8 out_buf[4] __attribute__((aligned(8)));
	static u8 in_buf[4] __attribute__((aligned(8)));
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

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv_cb_ptr->spi_ptr, &msg);
	if (ret)
		DMBERR("error: %d\n", ret);  
}
#endif

#if defined(CONFIG_ARCH_MTKXXXX)
#include <mach/mt_spi.h>
static struct mt_chip_conf spi_conf;
struct spi_device	*mtv_spi=NULL;
#endif

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
		spi_par->com_mod = DMA_TRANSFER;

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
	.remove	= (mtv_spi_remove),
};
#endif /* #ifdef RTV_IF_SPI */

