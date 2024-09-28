/*
 * File name: mtv_isr.c
 *
 * Description: MTV ISR driver.
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
 */

#include <src/raontv.h>
#include <src/raontv_internal.h>

#include "mtv.h"

#if defined(RTV_IF_SPI) ||defined(RTV_IF_EBI2)
static void isr_print_tsp(U8 *tspb, UINT size)
{
#if 0
	unsigned int i, cnt = 4;
	const U8 *tsp_buf_ptr = (const U8 *)tspb;

	for (i = 0; i < cnt; i++, tsp_buf_ptr += 188)
	{
		DMBMSG("[%d] 0x%02X 0x%02X 0x%02X 0x%02X | 0x%02X\n",
			i, tsp_buf_ptr[0], tsp_buf_ptr[1],
			tsp_buf_ptr[2], tsp_buf_ptr[3],
			tsp_buf_ptr[187]);
	}
#endif
}

static inline void enqueue_wakeup(MTV_TS_PKT_INFO *tspb)
{
	/* Enqueue a TSP buffer into ts queue. */
	mtv_put_tsp(tspb);

	//printk("[enqueue_wakeup_0] get_tsp_queue_count(%u)\n", get_tsp_queue_count());
	
	/* Wake up threads blocked on read() function
	if the file was opened as the blocking mode. */
	if ((mtv_cb_ptr->f_flags & O_NONBLOCK) == 0) {
		if (!mtv_cb_ptr->first_interrupt) {
			if ((get_tsp_queue_count() >= mtv_cb_ptr->wakeup_tspq_thres_cnt))
				wake_up_interruptible(&mtv_cb_ptr->read_wq);
		} else {
			mtv_cb_ptr->first_interrupt = false;
			wake_up_interruptible(&mtv_cb_ptr->read_wq);
		}
	}

	//printk("[enqueue_wakeup_1] get_tsp_queue_count(%u)\n", get_tsp_queue_count());
}

void mtv_isr_handler(void)
{
	MTV_TS_PKT_INFO *tspb = NULL; /* reset */
	U8 *tspb_ptr = NULL;
	U8 istatus;
	UINT intr_size;

	RTV_GUARD_LOCK;

	intr_size = mtv_cb_ptr->msc1_thres_size;

	rtv_UpdateAdj();

	RTV_REG_MAP_SEL(SPI_CTRL_PAGE);

	istatus = RTV_REG_GET(0x10);
	//DMBMSG("$$$$$$$$ istatus(0x%02X)\n", istatus);
	
	if (istatus & (U8)(~SPI_INTR_BITS)) {
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		RTV_GUARD_FREE;
		DMBMSG("Interface error (0x%02X)\n", istatus);
		return;
	}

	if (istatus & SPI_UNDERFLOW_INTR) {
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		RTV_GUARD_FREE;
		DMBMSG("UDF: 0x%02X\n", istatus);
		return;
	}

	if (istatus & (SPI_THRESHOLD_INTR|SPI_OVERFLOW_INTR)) {
		/* Allocate a TS packet from TSP pool. */
		tspb = mtv_alloc_tsp();
		if (tspb) {
			tspb_ptr = tspb->msc1_buf;

			RTV_REG_MAP_SEL(SPI_MEM_PAGE);
			RTV_REG_BURST_GET(0x10, tspb_ptr, intr_size);

			tspb->msc1_size = intr_size;

			isr_print_tsp(tspb_ptr, intr_size); /* To debug */

			//DMBMSG("Read TS data\n");
			//RTV_REG_BURST_GET(0x10, tspb_ptr, 1);

		#ifdef DEBUG_MTV_IF_MEMORY
			mtv_cb_ptr->msc1_ts_intr_cnt++;
		#endif

			/* To debug */
			if (istatus & SPI_OVERFLOW_INTR) {
				mtv_cb_ptr->msc1_ovf_intr_cnt++;
				DMBMSG("OVF: 0x%02X\n", istatus);
			}

			//DMB_LEVEL_INTR_INC;
	
	#if 0 ////#### delay
			if ((total_int_cnt%20) == 0) {
				RTV_DELAY_MS(220);
				printk("[tdmb_isr_handler] Delayed!\n");
			}
	#endif			
		} else {
			RTV_REG_MAP_SEL(SPI_CTRL_PAGE);
			RTV_REG_SET(0x2A, 1); /* SRAM init */
			RTV_REG_SET(0x2A, 0);
			//mtv_cb_ptr->alloc_tspb_err_cnt++;
			DMBERR("No more TSP buffer from pool.\n");
		}
	}
	else {
		DMBMSG("No data interrupt (0x%02X)\n", istatus);
	}

	RTV_GUARD_FREE;

	if (tspb) /* Check if a tspb was exist to be enqueud? */
		enqueue_wakeup(tspb);
}

void mtv_sram_init(void)
{
	//U8 istatus;
	//U8 istatus2;
	RTV_GUARD_LOCK;

	DMBMSG("mtv_sram_init\n");
	RTV_REG_MAP_SEL(SPI_CTRL_PAGE);

	//istatus = RTV_REG_GET(0x10);
	//istatus2 = RTV_REG_GET(0x24);
	//DMBMSG("$$$$$$$$1 istatus(0x%02X) istatus2(0x%02X)\n", istatus, istatus2);

	RTV_REG_SET(0x2A, 1); /* SRAM init */
	RTV_REG_SET(0x2A, 0);
	//istatus = RTV_REG_GET(0x10);
	//istatus2 = RTV_REG_GET(0x24);
	//DMBMSG("$$$$$$$$2 istatus(0x%02X) istatus2(0x%02X)\n", istatus, istatus2);	
	RTV_GUARD_FREE;
}

int mtv_isr_thread(void *data)
{
	long ret;
	DMBMSG("Start...\n");

	set_user_nice(current, -1);
	
	while (!kthread_should_stop()) {
		ret = wait_event_interruptible_timeout(mtv_cb_ptr->isr_wq,
				kthread_should_stop() || (mtv_cb_ptr->isr_cnt > 0),2*HZ);

		if (kthread_should_stop())
			break;

			
		if (mtv_cb_ptr->is_power_on == TRUE) {

		if(ret ==0)
			{
			     if (mtv_cb_ptr->tsout_enabled==TRUE)
             		     {
                  		mtv_sram_init(); 
			     	}

			}
			else
			{
			
			mtv_cb_ptr->isr_cnt--;
			mtv_isr_handler();
			}
		}
		

			
	}

	DMBMSG("Exit.\n");

	return 0;
}

irqreturn_t mtv_isr(int irq, void *param)
{
	if(mtv_cb_ptr->is_power_on == TRUE) {
		mtv_cb_ptr->isr_cnt++;
		wake_up_interruptible(&mtv_cb_ptr->isr_wq);
	}

	return IRQ_HANDLED;
}
#endif /* #if defined(RTV_IF_SPI) ||defined(RTV_IF_EBI2) */

