/*****************************************************************************
	Copyright(c) 2013 FCI Inc. All Rights Reserved

	File name : fc8300.c

	Description : Driver source file

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
/******************************************************************************
********************Alex.shi
Focus on:GPIO_ISDBT_IRQ/GPIO_ISDBT_PWR_EN/GPIO_ISDBT_RST/LDO
*******************************************************************************/
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/io.h>

#include "fc8300.h"
#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8300_regs.h"
#include "fc8300_isr.h"
#include "fci_hal.h"
#include <linux/regulator/consumer.h>
#include <soc/sprd/board.h>

//#define FC8300_DEBUG		// alex.shi add 20150924

struct ISDBT_INIT_INFO_T *hInit;
#define FEATURE_THREADED_IRQ

#define RING_BUFFER_SIZE	(188 * 321 * 8)

/* GPIO(RESET & INTRRUPT) Setting */
#define FC8300_NAME		"isdbt"

#ifndef GPIO_ISDBT_IRQ
#define GPIO_ISDBT_IRQ 		73
#define GPIO_ISDBT_PWR_EN	217
#define GPIO_ISDBT_RST		218
#endif

struct ISDBT_OPEN_INFO_T hOpen_Val;
u8 static_ringbuffer[RING_BUFFER_SIZE];

enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;
static DEFINE_MUTEX(ringbuffer_lock);
static DEFINE_MUTEX(driver_mode_lock);
static DECLARE_WAIT_QUEUE_HEAD(isdbt_isr_wait);
#ifdef FEATURE_TS_CHECK
u32 check_cnt_size;

#define MAX_DEMUX           2

/*
 * Sync Byte 0xb8
 */
#define SYNC_BYTE_INVERSION

struct pid_info {
	unsigned long count;
	unsigned long discontinuity;
	unsigned long continuity;
};

struct demux_info {
	struct pid_info  pids[8192];

	unsigned long    ts_packet_c;
	unsigned long    malformed_packet_c;
	unsigned long    tot_scraped_sz;
	unsigned long    packet_no;
	unsigned long    sync_err;
	unsigned long 	   sync_err_set;
};

static int is_sync(unsigned char *p)
{
	int syncword = p[0];
#ifdef SYNC_BYTE_INVERSION
	if (0x47 == syncword || 0xb8 == syncword)
		return 1;
#else
	if (0x47 == syncword)
		return 1;
#endif
	return 0;
}
static struct demux_info demux[MAX_DEMUX];

int print_pkt_log(void)
{
	unsigned long i = 0;

	print_log(NULL, "\nPKT_TOT : %d, SYNC_ERR : %d, SYNC_ERR_BIT : %d \
		, ERR_PKT : %d \n"
		, demux[0].ts_packet_c, demux[0].sync_err
		, demux[0].sync_err_set, demux[0].malformed_packet_c);

	for (i = 0; i < 8192; i++) {
		if (demux[0].pids[i].count > 0)
			print_log(NULL, "PID : %d, TOT_PKT : %d\
							, DISCONTINUITY : %d \n"
			, i, demux[0].pids[i].count
			, demux[0].pids[i].discontinuity);
	}
	return 0;
}

int put_ts_packet(int no, unsigned char *packet, int sz)
{
	unsigned char *p;
	int transport_error_indicator, pid, payload_unit_start_indicator;
	int continuity_counter, last_continuity_counter;
	int i;
	if ((sz % 188)) {
		print_log(NULL, "L : %d", sz);
	} else {
		for (i = 0; i < sz; i += 188) {
			p = packet + i;

			pid = ((p[1] & 0x1f) << 8) + p[2];

			demux[no].ts_packet_c++;
			if (!is_sync(packet + i)) {
				print_log(NULL, "S     ");
				demux[no].sync_err++;
				if (0x80 == (p[1] & 0x80))
					demux[no].sync_err_set++;
				print_log(NULL, "0x%x, 0x%x, 0x%x, 0x%x \n"
					, *p, *(p+1),  *(p+2), *(p+3));
				continue;
			}

			transport_error_indicator = (p[1] & 0x80) >> 7;
			if (1 == transport_error_indicator) {
				demux[no].malformed_packet_c++;
				continue;
			}

			payload_unit_start_indicator = (p[1] & 0x40) >> 6;

			demux[no].pids[pid].count++;

			continuity_counter = p[3] & 0x0f;

			if (demux[no].pids[pid].continuity == -1) {
				demux[no].pids[pid].continuity
					= continuity_counter;
			} else {
				last_continuity_counter
					= demux[no].pids[pid].continuity;

				demux[no].pids[pid].continuity
					= continuity_counter;

				if (((last_continuity_counter + 1) & 0x0f)
					!= continuity_counter) {
					demux[no].pids[pid].discontinuity++;
				}
			}
		}
	}
	return 0;
}


void create_tspacket_anal(void)
{
	int n, i;

	for (n = 0; n < MAX_DEMUX; n++) {
		memset((void *)&demux[n], 0, sizeof(demux[n]));

		for (i = 0; i < 8192; i++)
			demux[n].pids[i].continuity = -1;
	}

}
#endif

#ifndef BBM_I2C_TSIF
static u8 isdbt_isr_sig;
#ifndef FEATURE_THREADED_IRQ
static struct task_struct *isdbt_kthread;
#endif
static int		s_isdbt_irq=0;

#ifdef FEATURE_THREADED_IRQ
static irqreturn_t isdbt_threaded_irq(int irq, void *dev_id)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)dev_id;

	mutex_lock(&driver_mode_lock);
	if (driver_mode == ISDBT_POWERON)
		bbm_com_isr(hInit);
	mutex_unlock(&driver_mode_lock);

	return IRQ_HANDLED;
}

static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}
#else
static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
#ifdef FC8300_DEBUG
	//printk("%s\n", __FUNCTION__);
#endif
	isdbt_isr_sig = 1;
	wake_up_interruptible(&isdbt_isr_wait);
	return IRQ_HANDLED;
}
#endif
#endif

#ifdef FEATURE_THREADED_IRQ
int isdbt_hw_setting(HANDLE hDevice)
#else
int isdbt_hw_setting(void)
#endif
{
	int err;
	#ifdef FEATURE_THREADED_IRQ
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;
#endif
	struct regulator *reg_vdd = NULL;


	print_log(0, "isdbt_hw_setting\n");

	reg_vdd = regulator_get(NULL, "vddsim2");
	if (!WARN(IS_ERR(reg_vdd), "[FC8300] %s: failed to get %s.\n", __FUNCTION__, "vddsim2")) {
		regulator_set_voltage(reg_vdd,1800000,1800000);
		regulator_enable(reg_vdd);
	}
	print_log(0, "%s: success to set vddsim2\n", __FUNCTION__);
	msleep(10);
	err = gpio_request(GPIO_ISDBT_PWR_EN, "isdbt_en");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_en\n");
		goto gpio_isdbt_en;
	}
	gpio_direction_output(GPIO_ISDBT_PWR_EN, 0);

	err = gpio_request(GPIO_ISDBT_RST, "isdbt_rst");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_rst\n");
		goto gpio_isdbt_rst;
	}
	gpio_direction_output(GPIO_ISDBT_RST, 0);

#ifndef BBM_I2C_TSIF
	err = gpio_request(GPIO_ISDBT_IRQ, "isdbt_irq");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_irq\n");
		goto gpio_isdbt_rst;
	}

	gpio_direction_input(GPIO_ISDBT_IRQ);
	s_isdbt_irq = gpio_to_irq(GPIO_ISDBT_IRQ);
#ifdef FEATURE_THREADED_IRQ
	err = request_threaded_irq(s_isdbt_irq, isdbt_irq
	, isdbt_threaded_irq, IRQF_DISABLED | IRQF_TRIGGER_FALLING
	, FC8300_NAME, hInit);
#else
	err = request_irq(s_isdbt_irq, isdbt_irq
		, IRQF_DISABLED | IRQF_TRIGGER_FALLING, FC8300_NAME, NULL);
#endif
	if (err < 0) {
		print_log(0,
			"isdbt_hw_setting: couldn't request gpio interrupt %d reason(%d)\n"
			, GPIO_ISDBT_IRQ, err);
		goto request_isdbt_irq;
	}
#endif

	return 0;
#ifndef BBM_I2C_TSIF
request_isdbt_irq:
	gpio_free(GPIO_ISDBT_IRQ);
#endif
gpio_isdbt_rst:
	gpio_free(GPIO_ISDBT_PWR_EN);
gpio_isdbt_en:
	return err;
}

/*POWER_ON & HW_RESET & INTERRUPT_CLEAR */
void isdbt_hw_init(void)
{
	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_init_jerry\n");

	gpio_set_value(GPIO_ISDBT_RST, 0);
	gpio_set_value(GPIO_ISDBT_PWR_EN, 1);
	mdelay(5);
	gpio_set_value(GPIO_ISDBT_RST, 1);
	mdelay(5);
	driver_mode = ISDBT_POWERON;
	mutex_unlock(&driver_mode_lock);
}

/*POWER_OFF */
void isdbt_hw_deinit(void)
{
	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_deinit\n");
	gpio_set_value(GPIO_ISDBT_PWR_EN, 0);
	gpio_set_value(GPIO_ISDBT_RST, 0);
	driver_mode = ISDBT_POWEROFF;
	mutex_unlock(&driver_mode_lock);
}

int data_callback(ulong hDevice, u8 bufid, u8 *data, int len)
{
	struct ISDBT_INIT_INFO_T *hInit;
	struct list_head *temp;
	hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	list_for_each(temp, &(hInit->hHead))
	{
		struct ISDBT_OPEN_INFO_T *hOpen;

		hOpen = list_entry(temp, struct ISDBT_OPEN_INFO_T, hList);

		if (hOpen->isdbttype == TS_TYPE) {
			if (fci_ringbuffer_free(&hOpen->RingBuffer) < len) {
				/*print_log(hDevice, "f"); */
				/* return 0 */;
				FCI_RINGBUFFER_SKIP(&hOpen->RingBuffer, len);
			}

			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_write(&hOpen->RingBuffer, data, len);
			mutex_unlock(&ringbuffer_lock);
#ifdef FEATURE_TS_CHECK
				if (!(len%188)) {
					put_ts_packet(0, data, len);
					check_cnt_size += len;
	
					if (check_cnt_size > 188*320*40) {
						print_pkt_log();
						check_cnt_size = 0;
					}
				}
#endif
			
		}
	}

	return 0;
}


#ifndef BBM_I2C_TSIF
#ifndef FEATURE_THREADED_IRQ
static int isdbt_thread(void *hDevice)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	set_user_nice(current, -20);

	print_log(hInit, "isdbt_kthread enter\n");

	bbm_com_ts_callback_register((ulong)hInit, data_callback);

	while (1) {
		wait_event_interruptible(isdbt_isr_wait,
			isdbt_isr_sig || kthread_should_stop());

		mutex_lock(&driver_mode_lock);
		if (driver_mode == ISDBT_POWERON)
			bbm_com_isr(hInit);
		mutex_unlock(&driver_mode_lock);

		isdbt_isr_sig = 0;

		if (kthread_should_stop())
			break;
	}

	bbm_com_ts_callback_deregister();

	print_log(hInit, "isdbt_kthread exit\n");

	return 0;
}
#endif
#endif

const struct file_operations isdbt_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= isdbt_ioctl,
	.open		= isdbt_open,
	.read		= isdbt_read,
	.release	= isdbt_release,
};

static struct miscdevice fc8300_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FC8300_NAME,
	.fops = &isdbt_fops,
};

int isdbt_open(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt open\n");

	hOpen = &hOpen_Val;
	hOpen->buf = &static_ringbuffer[0];
	hOpen->isdbttype = 0;

	if (list_empty(&(hInit->hHead)))
		list_add(&(hOpen->hList), &(hInit->hHead));

	hOpen->hInit = (HANDLE *)hInit;

	if (hOpen->buf == NULL) {
		print_log(hInit, "ring buffer malloc error\n");
		return -ENOMEM;
	}

	fci_ringbuffer_init(&hOpen->RingBuffer, hOpen->buf, RING_BUFFER_SIZE);

	filp->private_data = hOpen;

	return 0;
}

ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s32 avail;
	s32 non_blocking = filp->f_flags & O_NONBLOCK;
	struct ISDBT_OPEN_INFO_T *hOpen
		= (struct ISDBT_OPEN_INFO_T *)filp->private_data;
	struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
	ssize_t len, read_len = 0;

	if (!cibuf->data || !count)	{
		/*print_log(hInit, " return 0\n"); */
		return 0;
	}

	if (non_blocking && (fci_ringbuffer_empty(cibuf)))	{
		/*print_log(hInit, "return EWOULDBLOCK\n"); */
		return -EWOULDBLOCK;
	}

	if (wait_event_interruptible(cibuf->queue,
		!fci_ringbuffer_empty(cibuf))) {
		print_log(hInit, "return ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	mutex_lock(&ringbuffer_lock);
	avail = fci_ringbuffer_avail(cibuf);
	mutex_unlock(&ringbuffer_lock);

	if (count >= avail)
		len = avail;
	else
		len = count - (count % 188);

	read_len = fci_ringbuffer_read_user(cibuf, buf, len);


	return read_len;
}

int isdbt_release(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt_release\n");
	isdbt_hw_deinit();

	hOpen = filp->private_data;

	hOpen->isdbttype = 0;
	if (!list_empty(&(hInit->hHead)))
		list_del(&(hOpen->hList));
	return 0;
}


#ifndef BBM_I2C_TSIF
void isdbt_isr_check(HANDLE hDevice)
{
	u8 isr_time = 0;

	bbm_com_write(hDevice, DIV_BROADCAST, BBM_BUF_ENABLE, 0x00);

	while (isr_time < 10) {
		if (!isdbt_isr_sig)
			break;

		msWait(10);
		isr_time++;
	}

}
#endif

long isdbt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	s32 res = BBM_NOK;
	s32 err = 0;
	s32 size = 0;
	struct ISDBT_OPEN_INFO_T *hOpen;

	struct ioctl_info info;

	if (_IOC_TYPE(cmd) != IOCTL_MAGIC)
		return -EINVAL;
	if (_IOC_NR(cmd) >= IOCTL_MAXNR)
		return -EINVAL;

	hOpen = filp->private_data;

	size = _IOC_SIZE(cmd);

	switch (cmd) {
	case IOCTL_ISDBT_RESET:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_reset(hInit, (u16)info.buff[0]);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_RESET\n");
#endif
		break;
	case IOCTL_ISDBT_INIT:
		res = bbm_com_i2c_init(hInit, FCI_HPI_TYPE);
		res |= bbm_com_probe(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_INIT %d\n", BBM_XTAL_FREQ);
#endif
		if (res) {
			print_log(hInit, "FC8300 Initialize Fail\n");
			break;
		}
		res |= bbm_com_init(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_INIT res %d\n", res);
#endif
		break;
	case IOCTL_ISDBT_BYTE_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u8 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BYTE_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u8)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_WORD_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u16 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_WORD_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u16)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_LONG_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u32 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_LONG_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u32)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BULK_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_bulk_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u8 *)(&info.buff[3])
			, info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BULK_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BYTE_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u8)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BYTE_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_WORD_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u16)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_WORD_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_LONG_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0]
			, (u32)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_LONG_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_BULK_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_bulk_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u8 *)(&info.buff[3])
			, info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BULK_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_read(hInit, (u16)info.buff[3]
			, (u8)info.buff[0], (u8)info.buff[1]
			,  (u8 *)(&info.buff[4]), (u8)info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[3], (u16)info.buff[0], info.buff[4]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_TUNER_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_write(hInit, (u16)info.buff[3]
			, (u8)info.buff[0], (u8)info.buff[1]
			, (u8 *)(&info.buff[4]), (u8)info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[3], (u16)info.buff[0], info.buff[4]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_SET_FREQ:
		{
			u32 f_rf;
			u8 subch;
			err = copy_from_user((void *)&info, (void *)arg, size);

			f_rf = (u32)info.buff[0];
			subch = (u8)info.buff[1];
#ifndef BBM_I2C_TSIF
			isdbt_isr_check(hInit);
#endif
			res = bbm_com_tuner_set_freq(hInit
				, (u16)info.buff[2], f_rf, subch);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_SET_FREQ [0x%x][%d][0x%x]\n"
		, (u16)info.buff[2], f_rf, subch);
#endif
#ifndef BBM_I2C_TSIF
			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_flush(&hOpen->RingBuffer);
			mutex_unlock(&ringbuffer_lock);
			bbm_com_write(hInit
				, DIV_BROADCAST, BBM_BUF_ENABLE, 0x01);
#endif
		}
		break;
	case IOCTL_ISDBT_TUNER_SELECT:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_select(hInit, (u16)info.buff[2]
			, (u32)info.buff[0], (u32)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_SELECT [0x%x][%d][0x%x]\n"
		, (u16)info.buff[2], (u32)info.buff[0], (u32)info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_TS_START:
#ifdef FEATURE_TS_CHECK
				create_tspacket_anal();
				check_cnt_size = 0;
#endif
		
		hOpen->isdbttype = TS_TYPE;
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TS_START\n");
#endif
		break;
	case IOCTL_ISDBT_TS_STOP:
		hOpen->isdbttype = 0;
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TS_STOP\n");
#endif
		break;
	case IOCTL_ISDBT_POWER_ON:
		isdbt_hw_init();
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_POWER_ON\n");
#endif
		break;
	case IOCTL_ISDBT_POWER_OFF:
		isdbt_hw_deinit();
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_POWER_OFF\n");
#endif
		break;
	case IOCTL_ISDBT_SCAN_STATUS:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_scan_status(hInit, (u16)info.buff[0]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_SCAN_STATUS [0x%x]\n"
		, (u16)info.buff[0]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_GET_RSSI:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_get_rssi(hInit
			, (u16)info.buff[0], (s32 *)&info.buff[1]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_GET_RSSI [0x%x][%d]\n"
		, (u16)info.buff[0], info.buff[1]);
#endif
		break;

	case IOCTL_ISDBT_DEINIT:
		res = bbm_com_deinit(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_DEINIT\n");
#endif
		break;

	default:
		print_log(hInit, "isdbt ioctl error!\n");
		res = BBM_NOK;
		break;
	}

	if (err < 0) {
		print_log(hInit, "copy to/from user fail : %d", err);
		res = BBM_NOK;
	}
	return res;
}

int isdbt_init(void)
{
	s32 res;

	print_log(hInit, "isdbt_init 20160314\n");

	res = misc_register(&fc8300_misc_device);

	if (res < 0) {
		print_log(hInit, "isdbt init fail : %d\n", res);
		return res;
	}

	hInit = kmalloc(sizeof(struct ISDBT_INIT_INFO_T), GFP_KERNEL);
#ifdef FEATURE_THREADED_IRQ
	isdbt_hw_setting(hInit);
	bbm_com_ts_callback_register((ulong)hInit, data_callback);
#else
	isdbt_hw_setting();
#endif

#if defined(BBM_I2C_TSIF) || defined(BBM_I2C_SPI)
	res = bbm_com_hostif_select(hInit, BBM_I2C);
#else
	res = bbm_com_hostif_select(hInit, BBM_SPI);
#endif

	if (res)
		print_log(hInit, "isdbt host interface select fail!\n");

#ifndef BBM_I2C_TSIF
#ifndef FEATURE_THREADED_IRQ
	if (!isdbt_kthread)	{
		print_log(hInit, "kthread run\n");
		isdbt_kthread = kthread_run(isdbt_thread
			, (void *)hInit, "isdbt_thread");
	}
#endif
#endif

	INIT_LIST_HEAD(&(hInit->hHead));

	return 0;
}

void isdbt_exit(void)
{
	print_log(hInit, "isdbt isdbt_exit\n");

	isdbt_hw_deinit();

#ifndef BBM_I2C_TSIF
	free_irq((s_isdbt_irq), NULL);
	gpio_free(GPIO_ISDBT_IRQ);
#endif

	gpio_free(GPIO_ISDBT_RST);
	gpio_free(GPIO_ISDBT_PWR_EN);

#ifndef BBM_I2C_TSIF
#ifdef FEATURE_THREADED_IRQ
	bbm_com_ts_callback_deregister();
#else
	if (isdbt_kthread)
		kthread_stop(isdbt_kthread);

	isdbt_kthread = NULL;
#endif
#endif

	bbm_com_hostif_deselect(hInit);

	if (hInit != NULL)
		kfree(hInit);

	misc_deregister(&fc8300_misc_device);

}

module_init(isdbt_init);
module_exit(isdbt_exit);

MODULE_LICENSE("Dual BSD/GPL");

