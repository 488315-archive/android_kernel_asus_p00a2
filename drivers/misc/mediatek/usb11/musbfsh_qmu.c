/*
* Copyright (C) 2011-2015 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/stat.h>

#include "mtkfsh_qmu.h"
#include "musbfsh_core.h"
#include "musbfsh_host.h"
#include "musbfsh_qmu.h"
//#include "musbhsdma.h"
//#include "mtk_musb.h"
#include "musbfsh_io.h"

void __iomem* usb11_qmu_base;
/* debug variable to check qmu_base issue */
void __iomem* usb11_qmu_base_2;
extern void mtkfsh_disable_q(struct musbfsh* musbfsh, u8 ep_num, u8 isRx);

int musbfsh_qmu_init(struct musbfsh *musbfsh)
{
	//printk("[QMU]%s:%d init\n", __FUNCTION__, __LINE__);
	/* set DMA channel 0 burst mode to boost QMU speed */
	musbfsh_writel(musbfsh->mregs, 0x204 , musbfsh_readl(musbfsh->mregs, 0x204) | 0x600 ) ;

#ifdef CONFIG_OF
	usb11_qmu_base = (void __iomem*)(musbfsh->mregs + MUSBFSH_QMUBASE);
	/* debug variable to check qmu_base issue */
	usb11_qmu_base_2 = (void __iomem*)(musbfsh->mregs + MUSBFSH_QMUBASE);
#else
	usb11_qmu_base = (void __iomem*)(USB_BASE + MUSBFSH_QMUBASE);
	/* debug variable to check qmu_base issue */
	usb11_qmu_base_2 = (void __iomem*)(musbfsh->mregs + MUSBFSH_QMUBASE);
	musbfsh_writel((void __iomem*)(musbfsh->mregs + MUSBFSH_QISAR),0x30,0);
	musbfsh_writel((void __iomem*)(USB_BASE + MUSBFSH_QISAR),0x30,0);
	musbfsh_writew((void __iomem *)musbfsh->mregs, MUSBFSH_QIMR, 0);
#endif
	mb();

	if(mtkfsh_qmu_init_gpd_pool(musbfsh->controller)){
		QMU_ERR("[QMU]mtkfsh_qmu_init_gpd_pool fail\n");
		return -1 ;
	}
	//printk("[QMU]%s:%d init ok\n", __FUNCTION__, __LINE__);
    return 0;
}

void musbfsh_qmu_exit(struct musbfsh *musbfsh)
{
	//printk("[QMU]%s:%d\n", __FUNCTION__, __LINE__);
	mtkfsh_qmu_destroy_gpd_pool(musbfsh->controller);
	//printk("[QMU]%s:%d ok\n", __FUNCTION__, __LINE__);
}

void musbfsh_disable_q_all(struct musbfsh *musbfsh)
{
    u32 ep_num;
	QMU_WARN("disable_q_all\n");
	//printk("[QMU]%s:%d\n", __FUNCTION__, __LINE__);
    for(ep_num = 1; ep_num <= RXQ_NUM; ep_num++){
        if(mtkfsh_is_qmu_enabled(ep_num, RXQ)){
            mtkfsh_disable_q(musbfsh, ep_num, 1);
		}
    }
    for(ep_num = 1; ep_num <= TXQ_NUM; ep_num++){
        if(mtkfsh_is_qmu_enabled(ep_num, TXQ)){
            mtkfsh_disable_q(musbfsh, ep_num, 0);
		}
    }
	//printk("[QMU]%s:%d ok\n", __FUNCTION__, __LINE__);
}

//jingao:add tmp
extern void mtkfsh_qmu_insert_task_ioc(u8 ep_num, u8 isRx, u8* buf, u32 length, u8 zlp,u8 ioc);

#if 0
void musbfsh_kick_D_CmdQ(struct musbfsh *musbfsh, struct musbfsh_request *request)
{
    int isRx;

    isRx = request->tx ? 0 : 1;

	/* enable qmu at musbfsh_gadget_eanble */
#if 0
    if(!mtkfsh_is_qmu_enabled(request->epnum,isRx)){
		/* enable qmu */
        mtkfsh_qmu_enable(musbfsh, request->epnum, isRx);
    }
#endif

	if(request->request.number_of_packets ==0){
	/* note tx needed additional zlp field */
    mtkfsh_qmu_insert_task(request->epnum,
						isRx,
						(u8*)request->request.dma,
						request->request.length, ((request->request.zero==1)?1:0));
		//musbfsh_writel(musbfsh->mregs,USB_L1INTM, musbfsh_readl(musbfsh->mregs,USB_L1INTM)| QINT_STATUS);
		mtkfsh_qmu_resume(request->epnum, isRx);
	}else{	//jingao:add for sony tinycap device.
		int i =0; 
		u8 isioc =0;
		u8 * pBuffer = (uint8_t*)request->request.dma;
		uint32_t offset,dwlength;
				
		for(i =0;i<request->request.number_of_packets;i++){
			offset = request->request.iso_frame_desc[i].offset;
			dwlength = request->request.iso_frame_desc[i].length;
			isioc = (i ==(request->request.number_of_packets -1)) ? 1 : 0;			
			////printk("jingao:insert_gpd%d offset = %d,dwlength =%d \n",i,offset,dwlength);
			mtkfsh_qmu_insert_task_ioc(request->epnum,
								isRx,
								pBuffer+offset,
								dwlength, ((request->request.zero==1)?1:0),isioc);
	//musbfsh_writel(musbfsh->mregs,USB_L1INTM, musbfsh_readl(musbfsh->mregs,USB_L1INTM)| QINT_STATUS);
	mtkfsh_qmu_resume(request->epnum, isRx);	
		}		
	}	
}
#endif

extern void mtkfsh_qmu_irq_err(struct musbfsh *musbfsh, u32 qisar);
extern void mtkfsh_qmu_done_rx(struct musbfsh *musbfsh, u8 ep_num);
extern void mtkfsh_qmu_done_tx(struct musbfsh *musbfsh, u8 ep_num);
irqreturn_t musbfsh_q_irq(struct musbfsh *musbfsh){

	irqreturn_t retval = IRQ_NONE;
	u32 wQmuVal = musbfsh->int_queue;
#ifndef QMU_TASKLET
	int i;
#endif
	QMU_INFO("wQmuVal:%d\n", wQmuVal);
#ifdef QMU_TASKLET
	if (musbfsh->qmu_done_intr != 0) {
		musbfsh->qmu_done_intr = wQmuVal | musbfsh->qmu_done_intr;
		QMU_WARN("Has not handle yet %x\n", musbfsh->qmu_done_intr);
	} else {
		musbfsh->qmu_done_intr = wQmuVal;
	}
	tasklet_schedule(&musbfsh->qmu_done);
#else
	for(i = 1; i<= MAX_QMU_EP; i++) {
		if (wQmuVal & DQMU_M_RX_DONE(i)){
			mtkfsh_qmu_done_rx(musbfsh, i);		
		}
		if (wQmuVal & DQMU_M_TX_DONE(i)){
			mtkfsh_qmu_done_tx(musbfsh, i);
		}
	}
#endif
	mtkfsh_qmu_irq_err(musbfsh, wQmuVal);
	//printk("[QMU]%s:%d ret:%d\n", __FUNCTION__, __LINE__, retval);
	return retval;
}

void musbfsh_flush_qmu(u32 ep_num, u8 isRx)
{
	//printk("[QMU]%s:%d\n", __FUNCTION__, __LINE__);
	QMU_WARN("flush %s(%d)\n", isRx?"RQ":"TQ", ep_num);
	mtkfsh_qmu_stop(ep_num, isRx);
	mtkfsh_qmu_reset_gpd_pool(ep_num, isRx);
	//printk("[QMU]%s:%d ok\n", __FUNCTION__, __LINE__);
}

#if 0 //gadget
void musbfsh_restart_qmu(struct musbfsh* musbfsh, u32 ep_num, u8 isRx)
{
	QMU_WARN("restart %s(%d)\n", isRx?"RQ":"TQ", ep_num);
	mtkfsh_flush_ep_csr(musbfsh, ep_num, isRx);
	mtkfsh_qmu_enable(musbfsh, ep_num, isRx);
}
#endif

bool musbfsh_is_qmu_stop(u32 ep_num, u8 isRx){
    void __iomem* base = usb11_qmu_base;
	//printk("[QMU]%s:%d\n", __FUNCTION__, __LINE__);
	/* debug variable to check qmu_base issue */
	if (usb11_qmu_base != usb11_qmu_base_2) {
		QMU_WARN("qmu_base != qmu_base_2");
		QMU_WARN("qmu_base = %p, qmu_base_2=%p",usb11_qmu_base, usb11_qmu_base_2);
	}

	if(!isRx){
		if(MGC_ReadQMU16(base, MGC_O_QMU_TQCSR(ep_num)) & DQMU_QUE_ACTIVE){
			//printk("[QMU]%s:%d no\n", __FUNCTION__, __LINE__);
			return false;
		}else{
			//printk("[QMU]%s:%d yes\n", __FUNCTION__, __LINE__);
			return true;
		}
	} else {
		if(MGC_ReadQMU16(base, MGC_O_QMU_RQCSR(ep_num)) & DQMU_QUE_ACTIVE){
			//printk("[QMU]%s:%d no\n", __FUNCTION__, __LINE__);
			return false;
		}else{
			//printk("[QMU]%s:%d yes\n", __FUNCTION__, __LINE__);
			return true;
		}
	}
}

void musbfsh_tx_zlp_qmu(struct musbfsh *musbfsh, u32 ep_num)
{
	/* sent ZLP through PIO */
	void __iomem        *epio = musbfsh->endpoints[ep_num].regs;
	void __iomem		*mbase =  musbfsh->mregs;
	unsigned long timeout = jiffies + HZ;
	int is_timeout = 1;
	u16			csr;
	//printk("[QMU]%s:%d\n", __FUNCTION__, __LINE__);
	QMU_WARN("TX ZLP direct sent\n");
	musbfsh_ep_select(mbase, ep_num);

	/* disable dma for pio */
	csr = musbfsh_readw(epio, MUSBFSH_TXCSR);
	csr &= ~MUSBFSH_TXCSR_DMAENAB;
	musbfsh_writew(epio, MUSBFSH_TXCSR, csr);

	/* TXPKTRDY */
	csr = musbfsh_readw(epio, MUSBFSH_TXCSR);
	csr |= MUSBFSH_TXCSR_TXPKTRDY;
	musbfsh_writew(epio, MUSBFSH_TXCSR, csr);

	/* wait ZLP sent */
	while(time_before_eq(jiffies, timeout)){
		csr = musbfsh_readw(epio, MUSBFSH_TXCSR);
		if(!(csr & MUSBFSH_TXCSR_TXPKTRDY)){
			is_timeout = 0;
			break;
		}
	}

	/* re-enable dma for qmu */
	csr = musbfsh_readw(epio, MUSBFSH_TXCSR);
	csr |= MUSBFSH_TXCSR_DMAENAB;
	musbfsh_writew(epio, MUSBFSH_TXCSR, csr);

	if(is_timeout){
		QMU_ERR("TX ZLP sent fail???\n");
	}
	QMU_WARN("TX ZLP sent done\n");
	//printk("[QMU]%s:%d ok\n", __FUNCTION__, __LINE__);
}

