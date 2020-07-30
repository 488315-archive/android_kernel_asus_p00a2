/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#ifdef MUSB_QMU_SUPPORT

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/stat.h>

#include "musb_core.h"
#include "musb_host.h"
#include "musbhsdma.h"
#include "mtk_musb.h"
#include "musb_qmu.h"
#if 1
//#ifdef MUSB_QMU_SUPPORT_HOST
#include "mtk_qmu.h"
#endif

void __iomem *qmu_base;
/* debug variable to check qmu_base issue */
void __iomem *qmu_base_2;

int musb_qmu_init(struct musb *musb)
{
	/* set DMA channel 0 burst mode to boost QMU speed */
	musb_writel(musb->mregs, 0x204, musb_readl(musb->mregs, 0x204) | 0x600);

#ifdef CONFIG_OF
	qmu_base = (void __iomem *)(mtk_musb->mregs + MUSB_QMUBASE);
	/* debug variable to check qmu_base issue */
	qmu_base_2 = (void __iomem *)(mtk_musb->mregs + MUSB_QMUBASE);
#else
	qmu_base = (void __iomem *)(USB_BASE + MUSB_QMUBASE);
	/* debug variable to check qmu_base issue */
	qmu_base_2 = (void __iomem *)(mtk_musb->mregs + MUSB_QMUBASE);
#if 1
//#ifdef MUSB_QMU_SUPPORT_HOST
	musb_writel((void __iomem *)(mtk_musb->mregs + MUSB_QISAR), 0x30, 0);
	musb_writel((void __iomem *)(USB_BASE + MUSB_QISAR), 0x30, 0);
	musb_writew((void __iomem *)mtk_musb->mregs, MUSB_QIMR, 0);
#endif
#endif
	mb();

	if (qmu_init_gpd_pool(musb->controller)) {
		QMU_ERR("[QMU]qmu_init_gpd_pool fail\n");
		return -1;
	}

	return 0;
}

void musb_qmu_exit(struct musb *musb)
{
	qmu_destroy_gpd_pool(musb->controller);
}

void musb_disable_q_all(struct musb *musb)
{
	u32 ep_num;

	QMU_WARN("disable_q_all\n");

	for (ep_num = 1; ep_num <= RXQ_NUM; ep_num++) {
		if (mtk_is_qmu_enabled(ep_num, RXQ))
			mtk_disable_q(musb, ep_num, 1);
	}
	for (ep_num = 1; ep_num <= TXQ_NUM; ep_num++) {
		if (mtk_is_qmu_enabled(ep_num, TXQ))
			mtk_disable_q(musb, ep_num, 0);
	}
}

void musb_kick_D_CmdQ(struct musb *musb, struct musb_request *request)
{
	int isRx;

	isRx = request->tx ? 0 : 1;

	/* enable qmu at musb_gadget_eanble */
#if 0
	if (!mtk_is_qmu_enabled(request->epnum, isRx)) {
		/* enable qmu */
		mtk_qmu_enable(musb, request->epnum, isRx);
	}
#endif

#if 1
	if (request->request.number_of_packets == 0) {
		/* note tx needed additional zlp field */
	    mtk_qmu_insert_task(request->epnum,
							isRx,
							(u8 *)request->request.dma,
							request->request.length, ((request->request.zero == 1)?1:0));
		mtk_qmu_resume(request->epnum, isRx);
	} else {
		int i = 0;
		u8 isioc = 0;
		u8 *pBuffer = (uint8_t *) request->request.dma;
		uint32_t offset, dwlength;

		for (i = 0; i < request->request.number_of_packets; i++) {
			offset = request->request.iso_frame_desc[i].offset;
			dwlength = request->request.iso_frame_desc[i].length;
			isioc = (i == (request->request.number_of_packets - 1)) ? 1 : 0;
			mtk_qmu_insert_task_ioc(request->epnum,
								isRx,
								pBuffer+offset,
								dwlength, ((request->request.zero == 1)?1:0), isioc);
			mtk_qmu_resume(request->epnum, isRx);
		}
	}
#endif
}

irqreturn_t musb_q_irq(struct musb *musb)
{

	irqreturn_t retval = IRQ_NONE;
	u32 wQmuVal = musb->int_queue;
#ifndef QMU_TASKLET
	int i;
#endif

	QMU_INFO("wQmuVal:%d\n", wQmuVal);
	QMU_INFO("queue csr:0x%x mask:0x%x tx:0x%x\n", 
						musb_readl(musb->mregs, MUSB_QISAR),
						musb_readl(musb->mregs, MUSB_QIMR),
						musb_readw(musb->mregs, MUSB_INTRTX));
#ifdef QMU_TASKLET
	if (musb->qmu_done_intr != 0) {
		musb->qmu_done_intr = wQmuVal | musb->qmu_done_intr;
		QMU_WARN("Has not handle yet %x\n", musb->qmu_done_intr);
	} else
		musb->qmu_done_intr = wQmuVal;
	tasklet_schedule(&musb->qmu_done);
#else
	for (i = 1; i <= MAX_QMU_EP; i++) {
#if 1
		if (wQmuVal & DQMU_M_RX_DONE(i)){
			#ifdef MUSB_QMU_SUPPORT_HOST
			if (!musb->is_host)
				qmu_done_rx(musb, i);
			else
				h_qmu_done_rx(musb, i);
			#else
			qmu_done_rx(musb, i);
			#endif
		}
		if (wQmuVal & DQMU_M_TX_DONE(i)) {
			#ifdef MUSB_QMU_SUPPORT_HOST
			if (!musb->is_host)
				qmu_done_tx(musb, i);
			else
				h_qmu_done_tx(musb, i);
			#else
			qmu_done_tx(musb, i);
			#endif
		}
#endif
	}
#endif
	mtk_qmu_irq_err(musb, wQmuVal);

	return retval;
}

void musb_flush_qmu(u32 ep_num, u8 isRx)
{
	QMU_DBG("flush %s(%d)\n", isRx ? "RQ" : "TQ", ep_num);
	mtk_qmu_stop(ep_num, isRx);
	qmu_reset_gpd_pool(ep_num, isRx);
}

void musb_restart_qmu(struct musb *musb, u32 ep_num, u8 isRx)
{
	QMU_DBG("restart %s(%d)\n", isRx ? "RQ" : "TQ", ep_num);
	flush_ep_csr(musb, ep_num, isRx);
	mtk_qmu_enable(musb, ep_num, isRx);
}

bool musb_is_qmu_stop(u32 ep_num, u8 isRx)
{
	void __iomem *base = qmu_base;

	/* debug variable to check qmu_base issue */
	if (qmu_base != qmu_base_2) {
		QMU_WARN("qmu_base != qmu_base_2");
		QMU_WARN("qmu_base = %p, qmu_base_2=%p", qmu_base, qmu_base_2);
	}

	if (!isRx) {
		if (MGC_ReadQMU16(base, MGC_O_QMU_TQCSR(ep_num)) & DQMU_QUE_ACTIVE)
			return false;
		else
			return true;
	} else {
		if (MGC_ReadQMU16(base, MGC_O_QMU_RQCSR(ep_num)) & DQMU_QUE_ACTIVE)
			return false;
		else
			return true;
	}
}

void musb_tx_zlp_qmu(struct musb *musb, u32 ep_num)
{
	/* sent ZLP through PIO */
	void __iomem *epio = musb->endpoints[ep_num].regs;
	void __iomem *mbase = musb->mregs;
	unsigned long timeout = jiffies + HZ;
	int is_timeout = 1;
	u16 csr;

	QMU_WARN("TX ZLP direct sent\n");
	musb_ep_select(mbase, ep_num);

	/* disable dma for pio */
	csr = musb_readw(epio, MUSB_TXCSR);
	csr &= ~MUSB_TXCSR_DMAENAB;
	musb_writew(epio, MUSB_TXCSR, csr);

	/* TXPKTRDY */
	csr = musb_readw(epio, MUSB_TXCSR);
	csr |= MUSB_TXCSR_TXPKTRDY;
	musb_writew(epio, MUSB_TXCSR, csr);

	/* wait ZLP sent */
	while (time_before_eq(jiffies, timeout)) {
		csr = musb_readw(epio, MUSB_TXCSR);
		if (!(csr & MUSB_TXCSR_TXPKTRDY)) {
			is_timeout = 0;
			break;
		}
	}

	/* re-enable dma for qmu */
	csr = musb_readw(epio, MUSB_TXCSR);
	csr |= MUSB_TXCSR_DMAENAB;
	musb_writew(epio, MUSB_TXCSR, csr);

	if (is_timeout)
		QMU_ERR("TX ZLP sent fail???\n");
	QMU_WARN("TX ZLP sent done\n");
}
#endif
