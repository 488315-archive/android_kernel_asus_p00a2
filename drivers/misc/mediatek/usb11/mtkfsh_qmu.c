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
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/list.h>
#include "mtkfsh_qmu.h"
#include "musbfsh_qmu.h"
#include "musbfsh_host.h"
#include "musbfsh_dma.h"

#define DEBUG_KICK_URB_TIMING 0
#define ENALBE_MUSBFSH_FORCE_DEBUG 1

#if DEBUG_KICK_URB_TIMING
#include <linux/time.h>
#endif
volatile static PGPD Rx_gpd_head[15];
volatile static PGPD Tx_gpd_head[15];
volatile static PGPD Rx_gpd_end[15];
volatile static PGPD Tx_gpd_end[15];
volatile static PGPD Rx_gpd_last[15];
volatile static PGPD Tx_gpd_last[15];
volatile static GPD_R Rx_gpd_List[15];
volatile static GPD_R Tx_gpd_List[15];
volatile static u64 Rx_gpd_Offset[15];
volatile static u64 Tx_gpd_Offset[15];

//spinlock_t Tx_gpd_lock;
//spinlock_t Rx_gpd_lock;

//jingao: add
#define QMU_ONE_BY_ONE  0
//jingao: add

extern void __iomem* usb11_qmu_base;
#define DEBUG_NEW_QMU_METHOD 0
#define MUSB_QMU_HOST_BULK_RX_AUTO_SPLIT 1

#ifdef MUSB_QMU_HOST_BULK_RX_AUTO_SPLIT 
#define QMU_RX_SPLIT_BLOCK_SIZE (32*1024)
#define QMU_RX_SPLIT_THRE    (64*1024) //(64*1024)
#endif
#if DEBUG_KICK_URB_TIMING
static struct timeval last_tv = {0};
static struct timeval current_tv = {0};
static bool first_transfer = true;
static unsigned int max_idle_time = 0;
static unsigned long ave_idle_time = 0;
static unsigned int transfer_count = 0;
static unsigned int over_125us_count = 0;
static unsigned int idle_time(struct timeval *current_tv, struct timeval *last_tv)
{
	return (current_tv->tv_sec - last_tv->tv_sec)*1000000 +(current_tv->tv_usec - last_tv->tv_usec);
}
#endif

static int usb_hcd_link_urb_to_queue(struct urb *urb, struct musbfsh_qh *qh)
{
	int		rc = 0;

	spin_lock(&hcd_urb_list_lock);
	list_del_init(&urb->urb_list);
	urb->unlinked = 0;
	list_add_tail(&urb->urb_list, &qh->queue_urb_list);
	spin_unlock(&hcd_urb_list_lock);
	return rc;
}

static u32 qmu_free_gpd_count(u8 isRx, u32 num)
{
	u32 free_count = 0;
	//u32 used_count = 0;
	//u32 offset = 0;
	if(isRx){
		if(Rx_gpd_end[num] > Rx_gpd_last[num]){
			free_count = MAX_GPD_NUM - ((Rx_gpd_end[num] - Rx_gpd_last[num])/4) -1;
		}else if(Rx_gpd_end[num] < Rx_gpd_last[num]){
			free_count = (Rx_gpd_last[num] - Rx_gpd_end[num])/4 -1;
		}else{
			free_count = MAX_GPD_NUM -1;
		}
	}else{

		QMU_INFO("end:%p last:%p\n", Tx_gpd_end[num], Tx_gpd_last[num]);
		if(Tx_gpd_end[num] > Tx_gpd_last[num]){
			QMU_INFO("offset:%ld\n", (Tx_gpd_end[num] - Tx_gpd_last[num]));
			free_count = MAX_GPD_NUM - ((Tx_gpd_end[num] - Tx_gpd_last[num])/4) -1;
			QMU_INFO("free:%d\n", free_count);
		}else if(Tx_gpd_end[num] < Tx_gpd_last[num]){
			QMU_INFO("offset:%ld\n", (Tx_gpd_last[num] - Tx_gpd_end[num]));
			free_count = (Tx_gpd_last[num] - Tx_gpd_end[num])/4 -1;
			QMU_INFO("free:%d\n", free_count);
		}else{
			free_count = MAX_GPD_NUM -1;
			QMU_INFO("free:%d\n",free_count);
		}
	}

	return free_count;
}

extern void musbfsh_advance_schedule(struct musbfsh *musbfsh, struct urb *urb,
				  struct musbfsh_hw_ep *hw_ep, int is_in);

u8 musbfsh_PDU_calcCksum(u8 *data, int len)
{
	u8 *uDataPtr, ckSum;
	int i;

	*(data + 1) = 0x0;
	uDataPtr = data;
	ckSum = 0;
	for (i = 0; i < len; i++)
		ckSum += *(uDataPtr + i);

	return 0xFF - ckSum;
}

static PGPD get_gpd(u8 isRx, u32 num){
	PGPD ptr;
	if(isRx) {
		ptr = Rx_gpd_List[num].pNext;
		Rx_gpd_List[num].pNext = (PGPD)((u8 *)(Rx_gpd_List[num].pNext) + GPD_LEN_ALIGNED);

		if ( Rx_gpd_List[num].pNext >= Rx_gpd_List[num].pEnd ) {
			Rx_gpd_List[num].pNext = Rx_gpd_List[num].pStart;
		}
	} else {
		ptr = Tx_gpd_List[num].pNext;
		Tx_gpd_List[num].pNext = (PGPD)((u8 *)(Tx_gpd_List[num].pNext) + GPD_LEN_ALIGNED);

		if ( Tx_gpd_List[num].pNext >= Tx_gpd_List[num].pEnd ) {
			Tx_gpd_List[num].pNext = Tx_gpd_List[num].pStart;
		}
	}
	return ptr;
}

static void gpd_ptr_align(u8 isRx, u32 num, PGPD ptr)
{
	if(isRx)
		Rx_gpd_List[num].pNext = (PGPD)((u8 *)(ptr) + GPD_LEN_ALIGNED);
	else
		Tx_gpd_List[num].pNext = (PGPD)((u8 *)(ptr) + GPD_LEN_ALIGNED);
}

static dma_addr_t gpd_virt_to_phys(void *vaddr, u8 isRx, u32 num)
{
	dma_addr_t paddr;

	if (isRx) {
		paddr = (dma_addr_t)((u64)(unsigned long)vaddr - Rx_gpd_Offset[num]);
	} else {
		paddr = (dma_addr_t)((u64)(unsigned long)vaddr - Tx_gpd_Offset[num]);
	}

	QMU_INFO("%s[%d]phys=%p<->virt=%p\n",
			((isRx == RXQ)?"RQ":"TQ"), num, (void *)paddr, vaddr);

	return paddr;
}

static void *gpd_phys_to_virt(dma_addr_t paddr, u8 isRx, u32 num)
{
	void *vaddr;


	if (isRx) {
		vaddr = (void *)(unsigned long)((u64)paddr + Rx_gpd_Offset[num]);
	} else {
		vaddr = (void *)(unsigned long)((u64)paddr + Tx_gpd_Offset[num]);
	}
	QMU_INFO("%s[%d]phys=%p<->virt=%p\n",
			((isRx == RXQ)?"RQ":"TQ"), num , (void *)paddr, vaddr);

	return vaddr;
}

static void init_gpd_list(u8 isRx, int num, PGPD ptr, PGPD io_ptr, u32 size)
{
	if (isRx) {
		Rx_gpd_List[num].pStart = ptr;
		Rx_gpd_List[num].pEnd = (PGPD)( (u8*)(ptr + size) + (GPD_EXT_LEN*size) );
		Rx_gpd_Offset[num]=(u64)(unsigned long)ptr - (u64)(unsigned long)io_ptr;
		ptr++;
		Rx_gpd_List[num].pNext = (PGPD)((u8*)ptr + GPD_EXT_LEN);

		QMU_INFO("Rx_gpd_List[%d].pStart=%p, pNext=%p, pEnd=%p\n", \
				num, Rx_gpd_List[num].pStart, Rx_gpd_List[num].pNext, Rx_gpd_List[num].pEnd);
		QMU_INFO("Rx_gpd_Offset[%d]=%p\n", num, (void *)(unsigned long)Rx_gpd_Offset[num]);
	} else {
		Tx_gpd_List[num].pStart = ptr;
		Tx_gpd_List[num].pEnd = (PGPD)( (u8*)(ptr + size) + (GPD_EXT_LEN*size) );
		Tx_gpd_Offset[num]=(u64)(unsigned long)ptr - (u64)(unsigned long)io_ptr;
		ptr++;
		Tx_gpd_List[num].pNext = (PGPD)((u8*)ptr + GPD_EXT_LEN);

		QMU_INFO("Tx_gpd_List[%d].pStart=%p, pNext=%p, pEnd=%p\n", \
				num, Tx_gpd_List[num].pStart, Tx_gpd_List[num].pNext, Tx_gpd_List[num].pEnd);
		QMU_INFO("Tx_gpd_Offset[%d]=%p\n", num, (void *)(unsigned long)Tx_gpd_Offset[num]);
	}
}

int mtkfsh_qmu_init_gpd_pool(struct device *dev){
	u32 i, size;
	TGPD *ptr,*io_ptr;
	dma_addr_t  dma_handle;
	u32 gpd_sz;

	gpd_sz = (u32)(u64)sizeof(TGPD);
	QMU_INFO("sizeof(TGPD):%d\n", gpd_sz);
	if(gpd_sz != GPD_SZ){
		QMU_ERR("ERR!!!, GPD SIZE != %d\n", GPD_SZ);
	}

	for ( i = 1; i<= RXQ_NUM; i++) {
		/* Allocate Rx GPD */

        //printk("qmu_init_gpd_pool RXQ_NUM 11 i = %d\n",i);
		size = GPD_LEN_ALIGNED * MAX_GPD_NUM;
		ptr = (TGPD*)dma_alloc_coherent(dev, size, &dma_handle, GFP_KERNEL);
		if(!ptr){
			return -ENOMEM ;
		}
		memset(ptr, 0 , size);
		io_ptr = (TGPD *)(dma_handle);

		init_gpd_list(RXQ, i, ptr, io_ptr, MAX_GPD_NUM);
		Rx_gpd_head[i]= ptr;
		QMU_INFO("ALLOC RX GPD Head [%d] Virtual Mem=%p, DMA addr=%p\n", i, Rx_gpd_head[i], io_ptr);
		Rx_gpd_end[i] = Rx_gpd_last[i] = Rx_gpd_head[i];
		TGPD_CLR_FLAGS_HWO(Rx_gpd_end[i]);
		gpd_ptr_align(RXQ, i, Rx_gpd_end[i]);
		QMU_INFO("RQSAR[%d]=%p\n", i, (void *)gpd_virt_to_phys(Rx_gpd_end[i],RXQ,i));
	}

	for ( i = 1; i<= TXQ_NUM; i++) {
        //printk("qmu_init_gpd_pool TXQ_NUM 11 i = %d\n",i);
		/* Allocate Tx GPD */
		size = GPD_LEN_ALIGNED * MAX_GPD_NUM;
		ptr = (TGPD*)dma_alloc_coherent(dev, size, &dma_handle, GFP_KERNEL);
		if(!ptr){
			return -ENOMEM ;
		}
		memset(ptr, 0 , size);
		io_ptr = (TGPD *)(dma_handle);

		init_gpd_list(TXQ, i, ptr, io_ptr, MAX_GPD_NUM);
		Tx_gpd_head[i]= ptr;
		QMU_INFO("ALLOC TX GPD Head [%d] Virtual Mem=%p, DMA addr=%p\n", i, Tx_gpd_head[i], io_ptr);
		Tx_gpd_end[i] = Tx_gpd_last[i] = Tx_gpd_head[i];
		TGPD_CLR_FLAGS_HWO(Tx_gpd_end[i]);
		gpd_ptr_align(TXQ, i, Tx_gpd_end[i]);
		QMU_INFO("TQSAR[%d]=%p\n", i, (void *)gpd_virt_to_phys(Tx_gpd_end[i],TXQ,i));
	}

	return 0;
}

void mtkfsh_qmu_reset_gpd_pool(u32 ep_num, u8 isRx)
{
	u32 size = GPD_LEN_ALIGNED * MAX_GPD_NUM;

	/* SW reset */
	if(isRx){
		memset(Rx_gpd_head[ep_num], 0 , size);
		Rx_gpd_end[ep_num] = Rx_gpd_last[ep_num] = Rx_gpd_head[ep_num];
		TGPD_CLR_FLAGS_HWO(Rx_gpd_end[ep_num]);
		gpd_ptr_align(isRx, ep_num, Rx_gpd_end[ep_num]);

	}else{
		memset(Tx_gpd_head[ep_num], 0 , size);
		Tx_gpd_end[ep_num] = Tx_gpd_last[ep_num] = Tx_gpd_head[ep_num];
		TGPD_CLR_FLAGS_HWO(Tx_gpd_end[ep_num]);
		gpd_ptr_align(isRx, ep_num, Tx_gpd_end[ep_num]);
	}
}

void mtkfsh_qmu_destroy_gpd_pool(struct device *dev){

	int i;
	u32 size = GPD_LEN_ALIGNED * MAX_GPD_NUM;

	for ( i = 1; i<= RXQ_NUM; i++) {
		dma_free_coherent(dev, size, Rx_gpd_head[i], gpd_virt_to_phys(Rx_gpd_head[i], RXQ, i));
	}

	for ( i = 1; i<= TXQ_NUM; i++) {
		dma_free_coherent(dev, size, Tx_gpd_head[i], gpd_virt_to_phys(Tx_gpd_head[i], TXQ, i));
	}
}
static void prepare_rx_gpd_ioc(u8 *pBuf, u32 data_len, u8 ep_num,u8 isioc)
{
	TGPD* gpd;

	/* get gpd from tail */
	gpd = Rx_gpd_end[ep_num];

	TGPD_SET_DATA(gpd, pBuf);
	TGPD_CLR_FORMAT_BDP(gpd);

	TGPD_SET_DataBUF_LEN(gpd, data_len);
	TGPD_SET_BUF_LEN(gpd, 0);

//	TGPD_CLR_FORMAT_BPS(gpd);
	if(isioc){
	TGPD_SET_IOC(gpd);
	}

	
	/* update gpd tail */
	Rx_gpd_end[ep_num] = get_gpd(RXQ ,ep_num);
	QMU_INFO("[RX]""Rx_gpd_end[%d]=%p gpd=%p\n", ep_num, Rx_gpd_end[ep_num], gpd);
	memset(Rx_gpd_end[ep_num], 0 , GPD_LEN_ALIGNED);
	TGPD_CLR_FLAGS_HWO(Rx_gpd_end[ep_num]);

	/* make sure struct ready before set to next*/
	mb();
	TGPD_SET_NEXT(gpd, gpd_virt_to_phys(Rx_gpd_end[ep_num], RXQ, ep_num));

	TGPD_SET_CHKSUM_HWO(gpd, 16);

	/* make sure struct ready before HWO */
	mb();
	TGPD_SET_FLAGS_HWO(gpd);
}

static void prepare_rx_gpd(u8 *pBuf, u32 data_len, u8 ep_num)
{
	TGPD* gpd;

	/* get gpd from tail */
	gpd = Rx_gpd_end[ep_num];

	TGPD_SET_DATA(gpd, pBuf);
	TGPD_CLR_FORMAT_BDP(gpd);

	TGPD_SET_DataBUF_LEN(gpd, data_len);
	TGPD_SET_BUF_LEN(gpd, 0);

//	TGPD_CLR_FORMAT_BPS(gpd);
	TGPD_SET_IOC(gpd);
	
	/* update gpd tail */
	Rx_gpd_end[ep_num] = get_gpd(RXQ ,ep_num);
	QMU_INFO("[RX]""Rx_gpd_end[%d]=%p gpd=%p\n", ep_num, Rx_gpd_end[ep_num], gpd);
	memset(Rx_gpd_end[ep_num], 0 , GPD_LEN_ALIGNED);
	TGPD_CLR_FLAGS_HWO(Rx_gpd_end[ep_num]);

	/* make sure struct ready before set to next*/
	mb();
	TGPD_SET_NEXT(gpd, gpd_virt_to_phys(Rx_gpd_end[ep_num], RXQ, ep_num));

	TGPD_SET_CHKSUM_HWO(gpd, 16);

	/* make sure struct ready before HWO */
	mb();
	TGPD_SET_FLAGS_HWO(gpd);
}

static void prepare_tx_gpd_ioc(u8 *pBuf, u32 data_len, u8 ep_num, u8 zlp,u8 isioc)
{
	TGPD* gpd;

	/* get gpd from tail */
	gpd = Tx_gpd_end[ep_num];
	TGPD_SET_DATA(gpd, pBuf);
	TGPD_CLR_FORMAT_BDP(gpd);

	TGPD_SET_BUF_LEN(gpd, data_len);
	TGPD_SET_EXT_LEN(gpd, 0);

	if (zlp)
		TGPD_SET_FORMAT_ZLP(gpd);
	else
		TGPD_CLR_FORMAT_ZLP(gpd);

	//TGPD_CLR_FORMAT_BPS(gpd);

	if(isioc){
		TGPD_SET_IOC(gpd);
	}
	

	/* update gpd tail */
	Tx_gpd_end[ep_num] = get_gpd(TXQ ,ep_num);
	QMU_INFO("[TX]""Tx_gpd_end[%d]=%p gpd=%p\n", ep_num, Tx_gpd_end[ep_num], gpd);
	memset(Tx_gpd_end[ep_num], 0 , GPD_LEN_ALIGNED);
	TGPD_CLR_FLAGS_HWO(Tx_gpd_end[ep_num]);


	/* make sure struct ready before set to next*/
	mb();
	TGPD_SET_NEXT(gpd, gpd_virt_to_phys(Tx_gpd_end[ep_num], TXQ, ep_num));

	TGPD_SET_CHKSUM_HWO(gpd, 16);

	/* make sure struct ready before HWO */
	mb();
	TGPD_SET_FLAGS_HWO(gpd);
}

static void prepare_tx_gpd(u8 *pBuf, u32 data_len, u8 ep_num, u8 zlp)
{
	TGPD* gpd;

	/* get gpd from tail */
	gpd = Tx_gpd_end[ep_num];

	TGPD_SET_DATA(gpd, pBuf);
	TGPD_CLR_FORMAT_BDP(gpd);

	TGPD_SET_BUF_LEN(gpd, data_len);
	TGPD_SET_EXT_LEN(gpd, 0);

	if (zlp)
		TGPD_SET_FORMAT_ZLP(gpd);
	else
		TGPD_CLR_FORMAT_ZLP(gpd);

	//TGPD_CLR_FORMAT_BPS(gpd);

	TGPD_SET_IOC(gpd);

	/* update gpd tail */
	Tx_gpd_end[ep_num] = get_gpd(TXQ ,ep_num);
	QMU_INFO("[TX]""Tx_gpd_end[%d]=%p gpd=%p\n", ep_num, Tx_gpd_end[ep_num], gpd);
	memset(Tx_gpd_end[ep_num], 0 , GPD_LEN_ALIGNED);
	TGPD_CLR_FLAGS_HWO(Tx_gpd_end[ep_num]);


	/* make sure struct ready before set to next*/
	mb();
	TGPD_SET_NEXT(gpd, gpd_virt_to_phys(Tx_gpd_end[ep_num], TXQ, ep_num));

	TGPD_SET_CHKSUM_HWO(gpd, 16);

	/* make sure struct ready before HWO */
	mb();
	TGPD_SET_FLAGS_HWO(gpd);

}

void mtkfsh_qmu_resume(u8 ep_num, u8 isRx)
{
	void __iomem* base = usb11_qmu_base;
	
	if (!isRx){
		MGC_WriteQMU32(base,  MGC_O_QMU_TQCSR(ep_num), DQMU_QUE_RESUME);
		if(!MGC_ReadQMU32(base, MGC_O_QMU_TQCSR(ep_num))){

			MGC_WriteQMU32(base,  MGC_O_QMU_TQCSR(ep_num), DQMU_QUE_RESUME);
			//QMU_ERR("TQCSR[%d]=%x\n", ep_num, MGC_ReadQMU32(base, MGC_O_QMU_TQCSR(ep_num)));
			//MGC_WriteQMU32(base,  MGC_O_QMU_TQCSR(ep_num), DQMU_QUE_RESUME);
			QMU_ERR("TQCSR[%d]=%x\n", ep_num, MGC_ReadQMU32(base, MGC_O_QMU_TQCSR(ep_num)));
		}
	}else{
		MGC_WriteQMU32(base,  MGC_O_QMU_RQCSR(ep_num), DQMU_QUE_RESUME);
		if(!MGC_ReadQMU32(base, MGC_O_QMU_RQCSR(ep_num))){
			QMU_ERR("RQCSR[%d]=%x\n", ep_num, MGC_ReadQMU32(base, MGC_O_QMU_RQCSR(ep_num)));
			MGC_WriteQMU32(base,  MGC_O_QMU_RQCSR(ep_num), DQMU_QUE_RESUME);
			QMU_ERR("RQCSR[%d]=%x\n", ep_num, MGC_ReadQMU32(base, MGC_O_QMU_RQCSR(ep_num)));
		}
	}	
	
}

bool mtkfsh_is_qmu_enabled(u8 ep_num, u8 isRx)
{
	void __iomem* base = usb11_qmu_base;
	if(isRx){
		if(MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR)&(USB_QMU_Rx_EN(ep_num))){
			return true;
		}
	}
	else{
		if(MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR)&(USB_QMU_Tx_EN(ep_num))){
			return true;
		}
	}
	return false;
}

void mtkfsh_qmu_enable(struct musbfsh* musbfsh, u8 ep_num, u8 isRx)
{
	struct musbfsh_qh		 *qh = NULL;
	u32 QCR;
	void __iomem* base = usb11_qmu_base;
	void __iomem        *mbase = musbfsh->mregs;
    struct musbfsh_hw_ep    *hw_ep = musbfsh->endpoints + ep_num;
    void __iomem        *epio = hw_ep->regs;
    u16    csr = 0;
    u16 intr_e = 0;

	qh = musbfsh_ep_get_qh(hw_ep, isRx);

	musbfsh_ep_select(mbase, ep_num);
	if (isRx){
		QMU_WARN("enable RQ(%d)\n", ep_num);

		/* enable dma */
		csr |= MUSBFSH_RXCSR_DMAENAB;

		/* check ISOC */
		if (qh->type == USB_ENDPOINT_XFER_ISOC)
			csr |= MUSBFSH_RXCSR_P_ISO;
		musbfsh_writew(epio, MUSBFSH_RXCSR, csr);

		/* turn off intrRx */
		intr_e = musbfsh_readw(mbase, MUSBFSH_INTRRXE);
		intr_e = intr_e & (~(1<<(ep_num)));
		musbfsh_writew(mbase, MUSBFSH_INTRRXE, intr_e);

		/* set 1st gpd and enable */
		MGC_WriteQMU32(base, MGC_O_QMU_RQSAR(ep_num), gpd_virt_to_phys(Rx_gpd_end[ep_num], RXQ, ep_num));
		MGC_WriteQUCS32(base, MGC_O_QUCS_USBGCSR,  MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR)|(USB_QMU_Rx_EN(ep_num)));

#ifdef CFG_CS_CHECK
		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR0);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR0, QCR | DQMU_RQCS_EN(ep_num));
#endif

#ifdef CFG_RX_ZLP_EN
		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR3);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR3, QCR | DQMU_RX_ZLP(ep_num));
#endif

#ifdef CFG_RX_COZ_EN
		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR3);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR3, QCR | DQMU_RX_COZ(ep_num));
#endif

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_QIMCR, DQMU_M_RX_DONE(ep_num)|DQMU_M_RQ_EMPTY|DQMU_M_RXQ_ERR|DQMU_M_RXEP_ERR);


#ifdef CFG_EMPTY_CHECK
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_REPEMPMCR, DQMU_M_RX_EMPTY(ep_num));
#else
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_QIMSR, DQMU_M_RQ_EMPTY);
#endif

		QCR = DQMU_M_RX_LEN_ERR(ep_num);
#ifdef CFG_CS_CHECK
		QCR |= DQMU_M_RX_GPDCS_ERR(ep_num);
#endif

#ifdef CFG_RX_ZLP_EN
		QCR |= DQMU_M_RX_ZLP_ERR(ep_num);
#endif
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_RQEIMCR, QCR);


		MGC_WriteQIRQ32(base, MGC_O_QIRQ_REPEIMCR, DQMU_M_RX_EP_ERR(ep_num));

		mb();
		/* qmu start */
		MGC_WriteQMU32(base, MGC_O_QMU_RQCSR(ep_num), DQMU_QUE_START);

	}else{
		QMU_WARN("enable TQ(%d)\n", ep_num);
		#if DEBUG_KICK_URB_TIMING
		first_transfer = true;
		#endif
		/* enable dma */
		csr |= MUSBFSH_TXCSR_DMAENAB;

		/* check ISOC */
		if (qh->type==USB_ENDPOINT_XFER_ISOC)
			csr |= MUSBFSH_TXCSR_P_ISO;
		musbfsh_writew(epio, MUSBFSH_TXCSR, csr);

		/* turn off intrTx */
		QMU_INFO("Turn off intrTX epnum:%d\n", ep_num);

		intr_e = musbfsh_readw(mbase, MUSBFSH_INTRTXE);
		intr_e = intr_e & (~(1<< ep_num));
		musbfsh_writew(mbase, MUSBFSH_INTRTXE, intr_e);
		mb();
		intr_e = musbfsh_readw(mbase, MUSBFSH_INTRTXE);
		QMU_INFO("After turn off intrTX:0x%x\n", intr_e);
		/* set 1st gpd and enable */
		MGC_WriteQMU32(base, MGC_O_QMU_TQSAR(ep_num), gpd_virt_to_phys(Tx_gpd_end[ep_num], TXQ, ep_num));
		MGC_WriteQUCS32(base, MGC_O_QUCS_USBGCSR,  MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR)|(USB_QMU_Tx_EN(ep_num)));

#ifdef CFG_CS_CHECK
		QCR= MGC_ReadQMU32(base, MGC_O_QMU_QCR0);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR0, QCR|DQMU_TQCS_EN(ep_num));
#endif

#if (TXZLP==HW_MODE)
		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR2);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR2, QCR|DQMU_TX_ZLP(ep_num));
#elif (TXZLP==GPD_MODE)
		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR2);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR2, QCR|DQMU_TX_MULTIPLE(ep_num));
#endif

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_QIMCR, DQMU_M_TX_DONE(ep_num)|DQMU_M_TQ_EMPTY|DQMU_M_TXQ_ERR|DQMU_M_TXEP_ERR);

#ifdef CFG_EMPTY_CHECK
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TEPEMPMCR, DQMU_M_TX_EMPTY(ep_num));
#else
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_QIMSR, DQMU_M_TQ_EMPTY);
#endif

		QCR = DQMU_M_TX_LEN_ERR(ep_num);
#ifdef CFG_CS_CHECK
		QCR |= DQMU_M_TX_GPDCS_ERR(ep_num) | DQMU_M_TX_BDCS_ERR(ep_num);
#endif
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TQEIMCR, QCR);

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TEPEIMCR, DQMU_M_TX_EP_ERR(ep_num));

		mb();
		/* qmu start */
		MGC_WriteQMU32(base, MGC_O_QMU_TQCSR(ep_num), DQMU_QUE_START);
	}
}

void mtkfsh_qmu_stop(u8 ep_num, u8 isRx)
{
    void __iomem* base = usb11_qmu_base;
	if(!isRx){
		if(MGC_ReadQMU16(base, MGC_O_QMU_TQCSR(ep_num)) & DQMU_QUE_ACTIVE){
			MGC_WriteQMU32(base,  MGC_O_QMU_TQCSR(ep_num), DQMU_QUE_STOP);
			QMU_WARN("Stop TQ %d\n", ep_num);
		}else{
			QMU_WARN("TQ %d already inactive\n", ep_num);
		}
	} else {
		if(MGC_ReadQMU16(base, MGC_O_QMU_RQCSR(ep_num)) & DQMU_QUE_ACTIVE){
			MGC_WriteQMU32(base,  MGC_O_QMU_RQCSR(ep_num), DQMU_QUE_STOP);
			QMU_WARN("Stop RQ %d\n", ep_num);
		}else{
			QMU_WARN("RQ %d already inactive\n", ep_num);
		}
	}
}

static void mtkfsh_qmu_disable(u8 ep_num, u8 isRx)
{
	u32 QCR;
    void __iomem* base = usb11_qmu_base;

	QMU_WARN("disable %s(%d)\n", isRx?"RQ":"TQ", ep_num);
	mtkfsh_qmu_stop(ep_num, isRx);
	if(isRx){
		/// clear Queue start address
		MGC_WriteQMU32(base, MGC_O_QMU_RQSAR(ep_num), 0);

		// KOBE, in denali, different EP QMU EN is separated in MGC_O_QUCS_USBGCSR ??
		MGC_WriteQUCS32(base, MGC_O_QUCS_USBGCSR,  MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR)&(~(USB_QMU_Rx_EN(ep_num))));

		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR0);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR0, QCR&(~(DQMU_RQCS_EN(ep_num))));
		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR3);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR3, QCR&(~(DQMU_RX_ZLP(ep_num))));

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_QIMSR, DQMU_M_RX_DONE(ep_num));
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_REPEMPMSR, DQMU_M_RX_EMPTY(ep_num));
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_RQEIMSR, DQMU_M_RX_LEN_ERR(ep_num)|DQMU_M_RX_GPDCS_ERR(ep_num)|DQMU_M_RX_ZLP_ERR(ep_num));
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_REPEIMSR, DQMU_M_RX_EP_ERR(ep_num));
	}else{
		/// clear Queue start address
		MGC_WriteQMU32(base, MGC_O_QMU_TQSAR(ep_num), 0);

		// KOBE, in denali, different EP QMU EN is separated in MGC_O_QUCS_USBGCSR ??
		MGC_WriteQUCS32(base, MGC_O_QUCS_USBGCSR,  MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR)&(~(USB_QMU_Tx_EN(ep_num))));

		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR0);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR0, QCR&(~(DQMU_TQCS_EN(ep_num))));
		QCR = MGC_ReadQMU32(base, MGC_O_QMU_QCR2);
		MGC_WriteQMU32(base, MGC_O_QMU_QCR2, QCR&(~(DQMU_TX_ZLP(ep_num))));

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_QIMSR, DQMU_M_TX_DONE(ep_num));
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TEPEMPMSR, DQMU_M_TX_EMPTY(ep_num));
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TQEIMSR, DQMU_M_TX_LEN_ERR(ep_num)|DQMU_M_TX_GPDCS_ERR(ep_num)|DQMU_M_TX_BDCS_ERR(ep_num));
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TEPEIMSR, DQMU_M_TX_EP_ERR(ep_num));
	}
}

void mtkfsh_qmu_insert_task_ioc(u8 ep_num, u8 isRx, u8* buf, u32 length, u8 zlp,u8 isioc)
{
	QMU_INFO("mtkfsh_qmu_insert_task_ioc ep_num: %d, isRx: %d, buf: %p, length: %d zlp: %d isioc: %d\n",
			ep_num, isRx, buf, length,zlp,isioc);
	if (isRx){
		/* rx don't care zlp input */		
		prepare_rx_gpd_ioc(buf, length, ep_num,isioc);		
	}
	else{
		prepare_tx_gpd_ioc(buf, length, ep_num,zlp,isioc);
	}
}

void mtkfsh_qmu_insert_task(u8 ep_num, u8 isRx, u8* buf, u32 length, u8 zlp)
{
	QMU_INFO("mtkfsh_qmu_insert_task ep_num: %d, isRx: %d, buf: %p, length: %d\n",
			ep_num, isRx, buf, length);
	if (isRx){
		/* rx don't care zlp input */		
		prepare_rx_gpd(buf, length, ep_num);		
	}
	else{
		prepare_tx_gpd(buf, length, ep_num, zlp);
	}
}

#if 0 //gadget
void qmu_done_rx(struct musbfsh *musbfsh, u8 ep_num)
{
	void __iomem* base = qmu_base;

	TGPD* gpd = Rx_gpd_last[ep_num];
	TGPD* gpd_current = (TGPD*)(unsigned long)MGC_ReadQMU32(base, MGC_O_QMU_RQCPR(ep_num));
	struct musbfsh_ep		*musbfsh_ep = &musbfsh->endpoints[ep_num].ep_out;
	struct usb_request	*request = NULL;
	struct musbfsh_request	*req;
	uint32_t done = 1;

	//jingao:add 	
	//void __iomem		*epio = musbfsh->endpoints[ep_num].regs;	
	//u16 		   csr;
	//csr = musbfsh_readw(epio, MUSBFSH_RXCSR);
	//jingao:add
#if 1
	/*Transfer PHY addr got from QMU register to VIR addr*/
	gpd_current = (TGPD*)gpd_phys_to_virt((dma_addr_t)gpd_current, RXQ, ep_num);	
	if (gpd == gpd_current) {		
		//printk("jingao:gpd-------------------------------\n");
		return;
	}
#endif
	//printk("jingao:in -->");
	//trying to give_back the request to gadget driver.
	req = next_request(musbfsh_ep);
	if (!req) {
		QMU_ERR("[RXD]""%s Cannot get next request of %d, "
			"but QMU has done.\n", __func__, ep_num);
		return;
	} else {
		request = &req->request;
	}

	if(unlikely(!gpd || !gpd_current)) {
		QMU_ERR("[RXD][ERROR] EP%d, gpd=%p, gpd_current=%p, ishwo=%d, rx_gpd_last=%p, 	RQCPR=0x%x\n",
							ep_num, gpd, gpd_current,
							((gpd == NULL) ? 999 : TGPD_IS_FLAGS_HWO(gpd)),
							Rx_gpd_last[ep_num],
							MGC_ReadQMU32(base, MGC_O_QMU_RQCPR(ep_num)));
		return;
	}

	if(TGPD_IS_FLAGS_HWO(gpd)) {
		QMU_ERR("[RXD][ERROR]""HWO=1!!\n");
		QMU_ERR("[RXD][ERROR]""HWO=1!!\n");
		QMU_ERR("[RXD][ERROR]""HWO=1!!\n");
		QMU_ERR("[RXD][ERROR]""HWO=1!!\n");
		QMU_ERR("[RXD][ERROR]""HWO=1!!\n");
		//BUG_ON(1);
		return;
	}

	/* NORMAL EXEC FLOW */
	while(gpd != gpd_current && !TGPD_IS_FLAGS_HWO(gpd)) {
		u32 rcv_len = (u32)TGPD_GET_BUF_LEN(gpd);
		u32 buf_len  = (u32)TGPD_GET_DataBUF_LEN(gpd);

		if(rcv_len > buf_len){
			QMU_ERR("[RXD][ERROR] rcv(%d) > buf(%d) AUK!?\n", rcv_len, buf_len);
		}

		QMU_INFO("[RXD]""gpd=%p ->HWO=%d, Next_GPD=%p, RcvLen=%d, BufLen=%d, pBuf=%p\n",
				gpd, TGPD_GET_FLAG(gpd), TGPD_GET_NEXT(gpd), rcv_len, buf_len, TGPD_GET_DATA(gpd));

		request->actual += rcv_len;

		if(unlikely (!TGPD_GET_NEXT(gpd) || !TGPD_GET_DATA(gpd))) {
			QMU_ERR("[RXD][ERROR] EP%d ,gpd=%p\n", ep_num, gpd);
			QMU_ERR("[RXD][ERROR] EP%d ,gpd=%p\n", ep_num, gpd);
			QMU_ERR("[RXD][ERROR] EP%d ,gpd=%p\n", ep_num, gpd);
			QMU_ERR("[RXD][ERROR] EP%d ,gpd=%p\n", ep_num, gpd);
			QMU_ERR("[RXD][ERROR] EP%d ,gpd=%p\n", ep_num, gpd);
			//BUG_ON(1);
			break;
		}

		gpd = TGPD_GET_NEXT(gpd);

		gpd = gpd_phys_to_virt((dma_addr_t)gpd, RXQ, ep_num);

		if(!gpd) {
			QMU_ERR("[RXD][ERROR] !gpd, EP%d ,gpd=%p\n", ep_num, gpd);
			QMU_ERR("[RXD][ERROR] !gpd, EP%d ,gpd=%p\n", ep_num, gpd);
			QMU_ERR("[RXD][ERROR] !gpd, EP%d ,gpd=%p\n", ep_num, gpd);
			QMU_ERR("[RXD][ERROR] !gpd, EP%d ,gpd=%p\n", ep_num, gpd);
			QMU_ERR("[RXD][ERROR] !gpd, EP%d ,gpd=%p\n", ep_num, gpd);
			//BUG_ON(1);
			break;
		}

		Rx_gpd_last[ep_num] = gpd;

#if 1 //jingao:add
		//printk("jingao request->number_of_packets %d \n",request->number_of_packets);
		if(request->number_of_packets >0){
			struct usb_iso_packet_descriptor *d;
			d = request->iso_frame_desc + request->iso_index;
			d->actual_length = rcv_len;			
			//req->request.actual += TGPD_GET_BUF_LEN(gpd);
			request->iso_index++;
			done = (request->iso_index == request->number_of_packets) ? true : false ;
		}else{
			done = true;
		}
#endif
		if(done){
#if 0			
			csr = musbfsh_readw(epio, MUSBFSH_RXCSR);
			if (csr & MUSBFSH_RXCSR_P_ISO) {
				if (csr & MUSBFSH_RXCSR_P_OVERRUN) {
					csr &= ~MUSBFSH_RXCSR_P_OVERRUN;
					musbfsh_writew(epio, MUSBFSH_RXCSR, csr);			
					printk("%s iso overrun on %p\n", musbfsh_ep->name, request);
					if (request )
						request->status = -EOVERFLOW;
				}
			}			
#endif			
			musbfsh_g_giveback(musbfsh_ep, request, 0);
		}
#if 1		
		req = next_request(musbfsh_ep);
		if(!req){
			return;
		}else{  	
			request = &req->request;				
		}	
#endif		
	} //end while
	//printk("out while :request %p, request ->index = %d\n",request,request->iso_index);

	/* QMU should keep take HWO gpd , so there is error*/
	if(gpd != gpd_current && TGPD_IS_FLAGS_HWO(gpd)) {
		QMU_ERR("[RXD][ERROR]""gpd=%p\n", gpd);

		QMU_ERR("[RXD][ERROR]""EP%d RQCSR=%x, RQSAR=%x, RQCPR=%x, RQLDPR=%x\n",
				ep_num,
				MGC_ReadQMU32(base, MGC_O_QMU_RQCSR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_RQSAR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_RQCPR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_RQLDPR(ep_num)));

		QMU_ERR("[RXD][ERROR]""QCR0=%x, QCR2=%x, QCR3=%x, QGCSR=%x\n",
				MGC_ReadQMU32(base, MGC_O_QMU_QCR0),
				MGC_ReadQMU32(base, MGC_O_QMU_QCR2),
				MGC_ReadQMU32(base, MGC_O_QMU_QCR3),
				MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR));

		QMU_ERR("[RXD][ERROR]""HWO=%d, Next_GPD=%p ,DataBufLen=%d, "
			"DataBuf=%p, RecvLen=%d, Endpoint=%d\n",
			(u32)TGPD_GET_FLAG(gpd), TGPD_GET_NEXT(gpd),
			(u32)TGPD_GET_DataBUF_LEN(gpd), TGPD_GET_DATA(gpd),
			(u32)TGPD_GET_BUF_LEN(gpd), (u32)TGPD_GET_EPaddr(gpd));
	}

	QMU_INFO("[RXD]""%s EP%d, Last=%p, End=%p, complete\n", __func__,
		ep_num, Rx_gpd_last[ep_num], Rx_gpd_end[ep_num]);
}

void qmu_done_tx(struct musbfsh *musbfsh, u8 ep_num)
{
	void __iomem* base = qmu_base;
	TGPD* gpd = Tx_gpd_last[ep_num];
	TGPD* gpd_current = (TGPD*)(unsigned long)MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num));
	struct musbfsh_ep		*musbfsh_ep = &musbfsh->endpoints[ep_num].ep_in;
	struct usb_request	*request = NULL;
	struct musbfsh_request	*req = NULL;

	/*Transfer PHY addr got from QMU register to VIR addr*/
	gpd_current = gpd_phys_to_virt((dma_addr_t)gpd_current, TXQ, ep_num);

	/*
                      gpd or Last       gdp_current
                           |                  |
            |->  GPD1 --> GPD2 --> GPD3 --> GPD4 --> GPD5 -|
            |----------------------------------------------|
	*/

	QMU_INFO("[TXD]""%s EP%d, Last=%p, Current=%p, End=%p\n",
		__func__, ep_num, gpd, gpd_current, Tx_gpd_end[ep_num]);

	/*gpd_current should at least point to the next GPD to the previous last one.*/
	if (gpd == gpd_current) {
		QMU_ERR("[TXD] gpd(%p) == gpd_current(%p)\n",
				gpd, gpd_current);
		return;
	}

	if(TGPD_IS_FLAGS_HWO(gpd)) {
		QMU_ERR("[TXD] HWO=1, CPR=%x\n", MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num)));
		QMU_ERR("[TXD] HWO=1, CPR=%x\n", MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num)));
		QMU_ERR("[TXD] HWO=1, CPR=%x\n", MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num)));
		QMU_ERR("[TXD] HWO=1, CPR=%x\n", MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num)));
		QMU_ERR("[TXD] HWO=1, CPR=%x\n", MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num)));
		//BUG_ON(1);
		return;
	}

	/* NORMAL EXEC FLOW */
	while (gpd != gpd_current && !TGPD_IS_FLAGS_HWO(gpd)) {

		QMU_INFO("[TXD]""gpd=%p ->HWO=%d, BPD=%d, Next_GPD=%p, DataBuffer=%p, "
			"BufferLen=%d request=%p\n",
			gpd, (u32)TGPD_GET_FLAG(gpd), (u32)TGPD_GET_FORMAT(gpd),
			TGPD_GET_NEXT(gpd), TGPD_GET_DATA(gpd), (u32)TGPD_GET_BUF_LEN(gpd), req);

		if(!TGPD_GET_NEXT(gpd)) {
			QMU_ERR("[TXD][ERROR]""Next GPD is null!!\n");
			QMU_ERR("[TXD][ERROR]""Next GPD is null!!\n");
			QMU_ERR("[TXD][ERROR]""Next GPD is null!!\n");
			QMU_ERR("[TXD][ERROR]""Next GPD is null!!\n");
			QMU_ERR("[TXD][ERROR]""Next GPD is null!!\n");
			//BUG_ON(1);
			break;
		}

		gpd = TGPD_GET_NEXT(gpd);

		gpd = gpd_phys_to_virt((dma_addr_t)gpd, TXQ, ep_num);

		/* trying to give_back the request to gadget driver. */
		req = next_request(musbfsh_ep);
		if (!req) {
			QMU_ERR("[TXD]""%s Cannot get next request of %d, "
				"but QMU has done.\n", __func__, ep_num);
			return;
		} else {
			request = &req->request;
		}

		Tx_gpd_last[ep_num] = gpd;
		musbfsh_g_giveback(musbfsh_ep, request, 0);
		req = next_request(musbfsh_ep);
		if (req != NULL) {
			request = &req->request;
		}
	}

	if(gpd!=gpd_current && TGPD_IS_FLAGS_HWO(gpd)) {

		QMU_ERR("[TXD][ERROR]""EP%d TQCSR=%x, TQSAR=%x, TQCPR=%x\n",
				ep_num,
				MGC_ReadQMU32(base, MGC_O_QMU_TQCSR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_TQSAR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num)));

		QMU_ERR("[RXD][ERROR]""QCR0=%x, QCR2=%x, QCR3=%x, QGCSR=%x\n",
				MGC_ReadQMU32(base, MGC_O_QMU_QCR0),
				MGC_ReadQMU32(base, MGC_O_QMU_QCR2),
				MGC_ReadQMU32(base, MGC_O_QMU_QCR3),
				MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR));

		QMU_ERR("[TXD][ERROR]""HWO=%d, BPD=%d, Next_GPD=%p, DataBuffer=%p, "
							"BufferLen=%d, Endpoint=%d\n",
							(u32)TGPD_GET_FLAG(gpd), (u32)TGPD_GET_FORMAT(gpd),
							TGPD_GET_NEXT(gpd), TGPD_GET_DATA(gpd),
							(u32)TGPD_GET_BUF_LEN(gpd), (u32)TGPD_GET_EPaddr(gpd));
	}

	QMU_INFO("[TXD]""%s EP%d, Last=%p, End=%p, complete\n", __func__,
		ep_num, Tx_gpd_last[ep_num], Tx_gpd_end[ep_num]);

	req = next_request(musbfsh_ep);
	if (!req) {
		return;
	}

	/* special case handle for zero request , only solve 1 zlp case*/
	if (req != NULL) {
		if (request->length == 0) {

			QMU_WARN("[TXD]""==Send ZLP== %p\n", req);
			musbfsh_tx_zlp_qmu(musbfsh, req->epnum);

			QMU_WARN("[TXD]""Giveback ZLP of EP%d, actual:%d, length:%d %p\n",
				req->epnum, request->actual, request->length, request);
			musbfsh_g_giveback(musbfsh_ep, request, 0);
			//jingao:add			
			req = next_request(musbfsh_ep);
			//jingao:add
		}
	}
#if 1 //jingao:add
	//jingao:add
	if(req!=NULL){
		//musbfsh_kick_D_CmdQ(musb, req);
	}else{
		//printk("jingao:no req!\n");
	}
	//jingao:add
#endif	//jingao:add
	
}
#endif

void mtkfsh_flush_ep_csr(struct musbfsh* musbfsh, u8 ep_num, u8 isRx)
{
    void __iomem        *mbase = musbfsh->mregs;
    struct musbfsh_hw_ep    *hw_ep = musbfsh->endpoints + ep_num;
    void __iomem        *epio = hw_ep->regs;
    u16 csr, wCsr;

    if (epio == NULL)
        QMU_ERR("epio == NULL\n");
    if (hw_ep == NULL)
        QMU_ERR("hw_ep == NULL\n");

    if (isRx)
    {
        csr = musbfsh_readw(epio, MUSBFSH_RXCSR);
        csr |= MUSBFSH_RXCSR_FLUSHFIFO | MUSBFSH_RXCSR_RXPKTRDY;
        if (musbfsh->is_host)
            csr &= ~MUSBFSH_RXCSR_H_REQPKT;

        /* write 2x to allow double buffering */
        //CC: see if some check is necessary
        musbfsh_writew(epio, MUSBFSH_RXCSR, csr);
        musbfsh_writew(epio, MUSBFSH_RXCSR, csr | MUSBFSH_RXCSR_CLRDATATOG);
    }
    else
    {
        csr = musbfsh_readw(epio, MUSBFSH_TXCSR);
        if (csr&MUSBFSH_TXCSR_TXPKTRDY)
        {
            wCsr = csr | MUSBFSH_TXCSR_FLUSHFIFO | MUSBFSH_TXCSR_TXPKTRDY;
            musbfsh_writew(epio, MUSBFSH_TXCSR, wCsr);
        }

        csr |= MUSBFSH_TXCSR_FLUSHFIFO&~MUSBFSH_TXCSR_TXPKTRDY;
        musbfsh_writew(epio, MUSBFSH_TXCSR, csr);
        musbfsh_writew(epio, MUSBFSH_TXCSR, csr | MUSBFSH_TXCSR_CLRDATATOG);
        //CC: why is this special?
        musbfsh_writew(mbase, MUSBFSH_INTRTX, 1<<ep_num);
    }
}

void mtkfsh_disable_q(struct musbfsh* musbfsh, u8 ep_num, u8 isRx)
{
    void __iomem        *mbase = musbfsh->mregs;
    struct musbfsh_hw_ep    *hw_ep = musbfsh->endpoints + ep_num;
    void __iomem        *epio = hw_ep->regs;
    u16    csr;

    mtkfsh_qmu_disable(ep_num, isRx);
	mtkfsh_qmu_reset_gpd_pool(ep_num, isRx);
    musbfsh_ep_select(mbase, ep_num);
    if(isRx){
        csr = musbfsh_readw(epio, MUSBFSH_RXCSR);
        csr &= ~MUSBFSH_RXCSR_DMAENAB;
        musbfsh_writew(epio, MUSBFSH_RXCSR, csr);
        mtkfsh_flush_ep_csr(musbfsh, ep_num,  isRx);
    }else{
        csr = musbfsh_readw(epio, MUSBFSH_TXCSR);
        csr &= ~MUSBFSH_TXCSR_DMAENAB;
        musbfsh_writew(epio, MUSBFSH_TXCSR, csr);
        mtkfsh_flush_ep_csr(musbfsh, ep_num,  isRx);
    }
}

#if 0 //gadget
void mtkfsh_qmu_err_recover(struct musbfsh *musbfsh, u8 ep_num, u8 isRx, bool is_len_err)
{
	struct musbfsh_ep  *musbfsh_ep;
	struct musbfsh_request *request;

	/* same action as musbfsh_flush_qmu */
	mtkfsh_qmu_stop(ep_num, isRx);
	qmu_reset_gpd_pool(ep_num, isRx);

	/* same action as musbfsh_restart_qmu */
	mtkfsh_flush_ep_csr(musbfsh, ep_num, isRx);
	mtkfsh_qmu_enable(musbfsh, ep_num, isRx);

	if(isRx){
		musbfsh_ep = &musbfsh->endpoints[ep_num].ep_out;
	}else{
		musbfsh_ep = &musbfsh->endpoints[ep_num].ep_in;
	}

	/* requeue all req , basically the same as musbfsh_kick_D_CmdQ */
	list_for_each_entry(request, &musbfsh_ep->req_list, list) {
		QMU_ERR("request 0x%p length(0x%d) len_err(%d)\n", request, request->request.length, is_len_err);

		if(request->request.dma != DMA_ADDR_INVALID)
		{
			if(request->tx)
			{
				QMU_ERR("[TX] gpd=%p, epnum=%d, len=%d\n", Tx_gpd_end[ep_num], ep_num, request->request.length);
				request->request.actual = request->request.length;
				if(request->request.length > 0) {
					QMU_ERR("[TX]""Send non-ZLP cases\n");
					mtkfsh_qmu_insert_task(request->epnum,
							isRx,
							(u8*)request->request.dma,
							request->request.length, ((request->request.zero==1)?1:0));

				} else if(request->request.length == 0) {
					/* this case may be a problem */
					QMU_ERR("[TX]""Send ZLP cases, may be a problem!!!\n");
					musbfsh_tx_zlp_qmu(musbfsh, request->epnum);
					musbfsh_g_giveback(musbfsh_ep, &(request->request), 0);
				}else{
					QMU_ERR("ERR, TX, request->request.length(%d)\n", request->request.length);
				}
			} else {
				QMU_ERR("[RX] gpd=%p, epnum=%d, len=%d\n",
						Rx_gpd_end[ep_num], ep_num, request->request.length);
				mtkfsh_qmu_insert_task(request->epnum,
						isRx,
						(u8*)request->request.dma,
						request->request.length, ((request->request.zero==1)?1:0));
			}
		}
	}
   	QMU_ERR("RESUME QMU\n");
	/* RESUME QMU */
	mtkfsh_qmu_resume(ep_num, isRx);
}
#endif

void mtkfsh_qmu_irq_err(struct musbfsh *musbfsh, u32 qisar)
{
	u8 i;
	u32 wQmuVal;
	u32 wRetVal;
	void __iomem* base = usb11_qmu_base;
   	u8 err_ep_num = 0;
	bool is_len_err = false;
	u8 isRx;

	wQmuVal = qisar;

	//RXQ ERROR
	if (wQmuVal & DQMU_M_RXQ_ERR)
	{
		wRetVal = MGC_ReadQIRQ32(base, MGC_O_QIRQ_RQEIR) & (~(MGC_ReadQIRQ32(base, MGC_O_QIRQ_RQEIMR)));
		QMU_ERR("RQ error in QMU mode![0x%x]\n", wRetVal);

		isRx = RXQ;
		for (i = 1; i <= RXQ_NUM; i++)
		{
			if (wRetVal & DQMU_M_RX_GPDCS_ERR(i))
			{
				QMU_ERR("RQ %d GPD checksum error!\n", i);
				err_ep_num = i;
			}
			if (wRetVal & DQMU_M_RX_LEN_ERR(i))
			{
				QMU_ERR("RQ %d recieve length error!\n", i);				
				err_ep_num = i;
				is_len_err = true;
			}
			if (wRetVal & DQMU_M_RX_ZLP_ERR(i))
			{
				QMU_ERR("RQ %d recieve an zlp packet!\n", i);
			}
		}
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_RQEIR, wRetVal);
	}

	//TXQ ERROR
	if (wQmuVal & DQMU_M_TXQ_ERR)
	{
		isRx = TXQ;
		wRetVal = MGC_ReadQIRQ32(base, MGC_O_QIRQ_TQEIR) & (~(MGC_ReadQIRQ32(base, MGC_O_QIRQ_TQEIMR)));
		QMU_ERR("TQ error in QMU mode![0x%x]\n", wRetVal);

		for (i=1; i<=RXQ_NUM; i++)
		{
			if (wRetVal & DQMU_M_TX_BDCS_ERR(i))
			{
				QMU_ERR("TQ %d BD checksum error!\n", i);
				err_ep_num = i;
			}
			if (wRetVal & DQMU_M_TX_GPDCS_ERR(i))
			{
				QMU_ERR("TQ %d GPD checksum error!\n", i);
				err_ep_num = i;
			}
			if (wRetVal & DQMU_M_TX_LEN_ERR(i))
			{
				QMU_ERR("TQ %d buffer length error!\n", i);
				err_ep_num = i;
				is_len_err = true;
			}
		}
		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TQEIR, wRetVal);
	}

	//RX EP ERROR
	if (wQmuVal & DQMU_M_RXEP_ERR)
	{
		isRx = RXQ;
		wRetVal = MGC_ReadQIRQ32(base, MGC_O_QIRQ_REPEIR) & (~(MGC_ReadQIRQ32(base, MGC_O_QIRQ_REPEIMR)));
		QMU_ERR("Rx endpoint error in QMU mode![0x%x]\n", wRetVal);

		for (i=1; i<=RXQ_NUM; i++)
		{
			if (wRetVal & DQMU_M_RX_EP_ERR(i))
			{
				QMU_ERR("RX EP %d ERR\n", i);
				err_ep_num = i;
			}
		}

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_REPEIR, wRetVal);
	}

	//TX EP ERROR
	if(wQmuVal & DQMU_M_TXEP_ERR)
	{
		isRx = TXQ;
		wRetVal = MGC_ReadQIRQ32(base, MGC_O_QIRQ_TEPEIR)& (~(MGC_ReadQIRQ32(base, MGC_O_QIRQ_TEPEIMR)));
		QMU_ERR("Tx endpoint error in QMU mode![0x%x]\n", wRetVal);

		for (i=1; i<=TXQ_NUM; i++){
			if (wRetVal & DQMU_M_TX_EP_ERR(i))
			{
				QMU_ERR("TX EP %d ERR\n", i);
				err_ep_num = i;
			}
		}

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TEPEIR, wRetVal);
	}

	//RXQ EMPTY
	if (wQmuVal & DQMU_M_RQ_EMPTY)
	{
		wRetVal = MGC_ReadQIRQ32(base, MGC_O_QIRQ_REPEMPR)
			& (~(MGC_ReadQIRQ32(base, MGC_O_QIRQ_REPEMPMR)));
		QMU_ERR("RQ Empty in QMU mode![0x%x]\n", wRetVal);

		for (i=1; i<=RXQ_NUM; i++)
		{
			if (wRetVal & DQMU_M_RX_EMPTY(i))
			{
				QMU_ERR("RQ %d Empty!\n", i);
			}
		}

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_REPEMPR, wRetVal);
	}

	//TXQ EMPTY
	if (wQmuVal & DQMU_M_TQ_EMPTY)
	{
		wRetVal = MGC_ReadQIRQ32(base, MGC_O_QIRQ_TEPEMPR)
			& (~(MGC_ReadQIRQ32(base, MGC_O_QIRQ_TEPEMPMR)));
		QMU_ERR("TQ Empty in QMU mode![0x%x]\n", wRetVal);

		for (i=1; i<=TXQ_NUM; i++)
		{
			if (wRetVal & DQMU_M_TX_EMPTY(i))
			{
				QMU_ERR("TQ %d Empty!\n", i);
			}
		}

		MGC_WriteQIRQ32(base, MGC_O_QIRQ_TEPEMPR, wRetVal);
	}

	/* QMU ERR RECOVER , only servie one ep error ?*/
	if(err_ep_num){
#if 0 //gadget
		mtkfsh_qmu_err_recover(musbfsh, err_ep_num, isRx, is_len_err);
#endif
	}
}

int mtkfsh_kick_CmdQ(struct musbfsh *musbfsh, int isRx, struct musbfsh_qh *qh)
{
	 void __iomem        *mbase = musbfsh->mregs;
	 //u16    csr = 0;
	 u16 intr_e = 0;
	 struct urb		*urb = next_urb(qh);
	 struct musbfsh_hw_ep	*hw_ep = qh->hw_ep;
	 void __iomem		*epio = hw_ep->regs;
	 unsigned int offset = 0;
	 u8 bIsIoc;
	 u8 *pBuffer;
	 u32 dwLength;
	 u16 i;
	 u32 gdp_free_count = 0;

	if(!urb) {
		#if ENALBE_MUSBFSH_FORCE_DEBUG
	      printk("[USB] NO urb, urb_list:%d urb_going_list:%d\n",
		  			list_empty(&qh->hep->urb_list), 
		  			list_empty(&qh->queue_urb_list));
		#endif
		  return -1; // KOBE : should we return a value 
	}

	 musbfsh_ep_set_qh(hw_ep, isRx, qh);

	 if(!mtkfsh_is_qmu_enabled(hw_ep->epnum,isRx))
	 {
	     QMU_DBG("! mtkfsh_is_qmu_enabled\n");
	     // musbfsh_ep_set_qh(hw_ep, isRx, qh);
	     musbfsh_ep_select(mbase, hw_ep->epnum);
	     mtkfsh_flush_ep_csr(musbfsh, hw_ep->epnum,  isRx);

	     if (isRx)
	     {
	          QMU_DBG("isRX = 1\n");

	         if (qh->type == USB_ENDPOINT_XFER_ISOC)
	         {
	              QMU_DBG("USB_ENDPOINT_XFER_ISOC\n");
	             if(qh->hb_mult== 3)
	                 musbfsh_writew(epio, MUSBFSH_RXMAXP, qh->maxpacket|0x1000);
	             else if(qh->hb_mult == 2)
	                 musbfsh_writew(epio, MUSBFSH_RXMAXP, qh->maxpacket|0x800);
	             else
	                 musbfsh_writew(epio, MUSBFSH_RXMAXP, qh->maxpacket);
	         }
	         else {
	              QMU_DBG("!! USB_ENDPOINT_XFER_ISOC\n");
	             musbfsh_writew(epio, MUSBFSH_RXMAXP, qh->maxpacket);
	         }

             QMU_DBG("isHOST\n");
             musbfsh_writew(epio, MUSBFSH_RXCSR, MUSBFSH_RXCSR_DMAENAB);
             //CC: speed?
             musbfsh_writeb(epio, MUSBFSH_RXTYPE, qh->type_reg);
             musbfsh_writeb(epio, MUSBFSH_RXINTERVAL, qh->intv_reg);

	         if (musbfsh->is_multipoint) {
	                  QMU_DBG("is_multipoint\n");
	             musbfsh_write_rxfunaddr(musbfsh->mregs, hw_ep->epnum, qh->addr_reg);
	             musbfsh_write_rxhubaddr(musbfsh->mregs, hw_ep->epnum, qh->h_addr_reg);
	             musbfsh_write_rxhubport(musbfsh->mregs, hw_ep->epnum, qh->h_port_reg);

	         } else{
	                  QMU_DBG("!! is_multipoint\n");
	             musbfsh_writeb(musbfsh->mregs, MUSBFSH_FADDR, qh->addr_reg);
	             }

	         //turn off intrRx
	         intr_e = musbfsh_readw(musbfsh->mregs, MUSBFSH_INTRRXE);
	         intr_e = intr_e & (~(1<<(hw_ep->epnum)));
	         musbfsh_writew(musbfsh->mregs, MUSBFSH_INTRRXE, intr_e);
	     }else {
	     	 QMU_DBG("isHOST\n");
	         musbfsh_writew(epio, MUSBFSH_TXMAXP, qh->maxpacket);
             musbfsh_writew(epio, MUSBFSH_TXCSR, MUSBFSH_TXCSR_DMAENAB);
             musbfsh_writeb(epio, MUSBFSH_TXTYPE, qh->type_reg);
             musbfsh_writeb(epio, MUSBFSH_TXINTERVAL, qh->intv_reg);

	         if (musbfsh->is_multipoint) {
	             QMU_DBG("is_multipoint\n");
	             musbfsh_write_txfunaddr(mbase, hw_ep->epnum, qh->addr_reg);
	             musbfsh_write_txhubaddr(mbase, hw_ep->epnum, qh->h_addr_reg);
	             musbfsh_write_txhubport(mbase, hw_ep->epnum, qh->h_port_reg);
	             /* FIXME if !epnum, do the same for RX ... */
	         } else {
	             QMU_DBG("!! is_multipoint\n");
	             musbfsh_writeb(mbase, MUSBFSH_FADDR, qh->addr_reg);//set the address of the device,very important!!
	        }
	     }
	     QMU_DBG("mtkfsh_qmu_enable\n");
	     mtkfsh_qmu_enable(musbfsh, hw_ep->epnum, isRx); //JEREMY
	 }
	 gdp_free_count = qmu_free_gpd_count(isRx, hw_ep->epnum);
	 while(urb && (gdp_free_count >= urb->number_of_packets)){
	 	 usb_hcd_link_urb_to_queue(urb, qh);
		 if (qh->type == USB_ENDPOINT_XFER_ISOC){
		     QMU_DBG("USB_ENDPOINT_XFER_ISOC\n");
		     pBuffer = (uint8_t *)urb->transfer_dma;

		     for(i=0; i<urb->number_of_packets; i++)
		     {
		         offset = urb->iso_frame_desc[i].offset;
		         dwLength = urb->iso_frame_desc[i].length;
				 urb->iso_frame_desc[i].status = 0;
		         /* If interrupt on complete ? */
		        bIsIoc = (i == (urb->number_of_packets-1)) ? true : false;
		        QMU_DBG("mtkfsh_qmu_insert_task\n");
				if(i== (urb->number_of_packets-1)){
		        	mtkfsh_qmu_insert_task_ioc(hw_ep->epnum, isRx, pBuffer+offset, dwLength, 0,1);						
					mtkfsh_qmu_resume(hw_ep->epnum, isRx);
				}else{
					mtkfsh_qmu_insert_task_ioc(hw_ep->epnum, isRx, pBuffer+offset, dwLength, 0,0);
				}
		     }
		 } else {
#ifndef MUSB_QMU_HOST_BULK_RX_AUTO_SPLIT
		     /* Must be the bulk transfer type */
		     pBuffer = (uint8_t *)urb->transfer_dma;

		     /*
		     Note current GPD only support 16 bits transferred data length.
		     Currently no software workaround this problem.
		     */        
		     if (urb->transfer_buffer_length >= 65536) {
		          QMU_DBG("[USB] Insert Task LEN Error !\n");

		     }
		     dwLength = urb->transfer_buffer_length;
		     bIsIoc = 1;

#else
			/* Must be the bulk transfer type */
			pBuffer = (uint8_t *)urb->transfer_dma;
			if(urb->transfer_buffer_length < QMU_RX_SPLIT_THRE){
			 	 QMU_DBG("urb->transfer_buffer_length : %d\n",urb->transfer_buffer_length);
				 dwLength = urb->transfer_buffer_length;
				 bIsIoc = 1; 
				 if(isRx){
		        	mtkfsh_qmu_insert_task_ioc(hw_ep->epnum, isRx, pBuffer+offset, dwLength, 0,bIsIoc);
					mtkfsh_qmu_resume(hw_ep->epnum, isRx);
				}else{
					mtkfsh_qmu_insert_task_ioc(hw_ep->epnum, isRx, pBuffer+offset, dwLength,0,bIsIoc);
					mtkfsh_qmu_resume(hw_ep->epnum, isRx);
				}
			 }else{
			    urb->number_of_packets = ((urb->transfer_buffer_length) +QMU_RX_SPLIT_BLOCK_SIZE-1)/(QMU_RX_SPLIT_BLOCK_SIZE);
				for(i=0; i<urb->number_of_packets; i++)
		         {
		             offset = QMU_RX_SPLIT_BLOCK_SIZE*i;
		             dwLength = QMU_RX_SPLIT_BLOCK_SIZE;
		             /* If interrupt on complete ? */
		             bIsIoc = (i == (urb->number_of_packets-1)) ? true : false;
					 dwLength = (i == (urb->number_of_packets-1)) ? ((urb->transfer_buffer_length) %QMU_RX_SPLIT_BLOCK_SIZE) : dwLength;
					 if(dwLength==0) 
					 	  dwLength = QMU_RX_SPLIT_BLOCK_SIZE;
		             if(isRx){
			        	mtkfsh_qmu_insert_task_ioc(hw_ep->epnum, isRx, pBuffer+offset, dwLength, 0,bIsIoc);
					}else{
						mtkfsh_qmu_insert_task_ioc(hw_ep->epnum, isRx, pBuffer+offset, dwLength,0,bIsIoc);
					}
					mtkfsh_qmu_resume(hw_ep->epnum, isRx);
		         }						 
			 }
#endif
		 }
		 mb();
		 urb = next_urb(qh);
		 gdp_free_count = qmu_free_gpd_count(isRx, hw_ep->epnum);
	 }

	 QMU_DBG("\n");
	 return 0;
}

extern void musbfsh_giveback_stage1(struct musbfsh *musbfsh, struct urb *urb, int status);
extern void musbfsh_giveback_stage2(struct musbfsh *musbfsh, struct urb *urb, int status);
void mtkfsh_q_advance_schedule(struct musbfsh *musbfsh, struct urb *urb, struct musbfsh_hw_ep *hw_ep, int is_in)
{
#if 0
    struct musbfsh_qh        *qh = musbfsh_ep_get_qh(hw_ep, is_in);
    struct musbfsh_hw_ep    *ep = qh->hw_ep;
    int            ready = qh->is_ready;
    int            status = 0;
#endif
	struct musbfsh_qh        *qh = NULL;
    struct musbfsh_hw_ep    *ep = NULL;
    int            ready = 0;
    int            status = 0;
	//#ifdef CONFIG_MT7118_HOST
#if 1
    struct dma_controller *dma_controller;
    struct dma_channel *dma_channel;
#endif
    //struct urb			*nexturb;

	QMU_DBG("hw_ep = %p\n",hw_ep);
	qh = musbfsh_ep_get_qh(hw_ep, is_in);
	if(NULL == qh){
		printk("[USB](NULL == qh) \n");
		return;
	}
	QMU_DBG("qh = %p qh->ready = %d\n",qh,qh->is_ready);
	ep = qh->hw_ep;
	if(NULL == ep){
		QMU_DBG("(NULL == ep) \n");
		return;
	}
	QMU_DBG("ep = %p\n",ep);
	ready = qh->is_ready;
	QMU_DBG("\n");

    status = (urb->status == -EINPROGRESS) ? 0 : urb->status;
	QMU_DBG("\n");

    /* save toggle eagerly, for paranoia */
    switch (qh->type) {
        //CC: do we need to save toggle for Q handled bulk transfer?
    case USB_ENDPOINT_XFER_BULK:
		QMU_DBG("\n");
        musbfsh_save_toggle(qh, is_in, urb);
		QMU_DBG("\n");
        break;
    case USB_ENDPOINT_XFER_ISOC:
        if (status == 0 && urb->error_count) {
			QMU_DBG("\n");
            status = -EXDEV;
        	}
        break;
    }

//#ifdef CONFIG_MT7118_HOST
#if 1
    /* candidate for DMA? */
    dma_controller = musbfsh->dma_controller;
    dma_channel = is_in ? ep->rx_channel : ep->tx_channel;
    if(dma_channel)
    {
        dma_controller->channel_release(dma_channel);
        dma_channel=NULL;
        if(is_in)
            ep->rx_channel=NULL;
        else
            ep->tx_channel=NULL;
    }
	QMU_DBG("\n");
#endif
	QMU_DBG("\n");

    qh->is_ready = 0;
    //musbfsh_giveback(musbfsh, urb, status);	
    musbfsh_giveback_stage1(musbfsh, urb, status);
    qh->is_ready = ready;
	QMU_DBG("\n");
    /* reclaim resources (and bandwidth) ASAP; deschedule it, and
     * invalidate qh as soon as list_empty(&hep->urb_list)
     */
#if 1 
    if (list_empty(&qh->hep->urb_list)) {
        struct list_head	*head;
        if (is_in)
            ep->rx_reinit = 1;
        else
            ep->tx_reinit = 1;
		//printk("jg:ep%d!!\n",qh->epnum);//jingao:add
        /* Clobber old pointers to this qh */
		QMU_DBG("\n");
        musbfsh_ep_set_qh(ep, is_in, NULL);
        qh->hep->hcpriv = NULL;
		QMU_DBG("\n");

        switch (qh->type) {
        case USB_ENDPOINT_XFER_BULK:
            /* fifo policy for these lists, except that NAKing
             * should rotate a qh to the end (for fairness).
             */
            //   kfree(qh);
            //  qh = NULL;
            //We don't use this list, just free qh
            QMU_DBG("\n");
            if (qh->mux == 1) {
				QMU_DBG("\n");
                head = qh->ring.prev;
                list_del(&qh->ring);
                kfree(qh);
                qh = first_qh(head);
				QMU_DBG("\n");
                break;
            }

            break;
        case USB_ENDPOINT_XFER_ISOC:
			QMU_DBG("\n");
            kfree(qh);
            qh = NULL;
			QMU_DBG("\n");
            break;
        }
    }
#endif
    //if (qh != NULL && qh->is_ready && next_urb(qh)!= NULL) {

    if (qh != NULL && qh->is_ready) {
	QMU_DBG("\n");
        mtkfsh_kick_CmdQ(musbfsh, is_in, qh);
	QMU_DBG("\n");
    }
#if 0	
	musbfsh_giveback_stage2(musbfsh, urb, status);
#endif
}

void mtkfsh_qmu_done_rx(struct musbfsh *musbfsh, u8 ep_num)
{
	void __iomem* base = usb11_qmu_base;

	TGPD* gpd = Rx_gpd_last[ep_num];
	TGPD* gpd_current = (TGPD*)(unsigned long)MGC_ReadQMU32(base, MGC_O_QMU_RQCPR(ep_num));
	//struct musbfsh_ep		*musbfsh_ep = &musbfsh->endpoints[ep_num].ep_out;
	//struct usb_request	*request = NULL;
	//struct musbfsh_request	*req;
	struct musbfsh_hw_ep	 *hw_ep = musbfsh->endpoints + ep_num;				
	struct musbfsh_qh 	   *qh = hw_ep->in_qh;
	struct urb		  *urb=NULL;
	int is_in = qh->is_in;

	bool done = true;
	//trying to give_back the request to gadget driver.
	urb = next_queue_urb(qh);
    if (unlikely(!urb)) {
        printk(KERN_ALERT "BOGUS RX%d ready,qh%p\n", ep_num,hw_ep->in_qh);            
    	mtkfsh_qmu_stop(ep_num, USB_DIR_IN);			
        return;
    }
	QMU_DBG("\n");

	
	/*Transfer PHY addr got from QMU register to VIR addr*/
	gpd_current = (TGPD*)gpd_phys_to_virt((dma_addr_t)gpd_current, RXQ, ep_num);
	QMU_DBG("\n");

	QMU_INFO("[RXD]""%s EP%d, Last=%p, Current=%p, End=%p\n",
		__func__, ep_num, gpd, gpd_current, Rx_gpd_end[ep_num]);

	/* gpd_current should at least point to the next GPD to the previous last one */
	if (gpd == gpd_current) {

		QMU_ERR("[RXD][ERROR] gpd(%p) == gpd_current(%p)\n",
				gpd, gpd_current);

		QMU_ERR("[RXD][ERROR]""EP%d RQCSR=%x, RQSAR=%x, RQCPR=%x, RQLDPR=%x\n",
				ep_num, 
				MGC_ReadQMU32(base, MGC_O_QMU_RQCSR(ep_num)), 
				MGC_ReadQMU32(base, MGC_O_QMU_RQSAR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_RQCPR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_RQLDPR(ep_num))); 

		QMU_ERR("[RXD][ERROR]""QCR0=%x, QCR2=%x, QCR3=%x, QGCSR=%x\n", 
				MGC_ReadQMU32(base, MGC_O_QMU_QCR0), 
				MGC_ReadQMU32(base, MGC_O_QMU_QCR2), 
				MGC_ReadQMU32(base, MGC_O_QMU_QCR3), 
				MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR));

		QMU_ERR("[RXD][ERROR]""HWO=%d, Next_GPD=%p ,DataBufLen=%d, "
				"DataBuf=%p, RecvLen=%d, Endpoint=%d\n",
				(u32)TGPD_GET_FLAG(gpd), TGPD_GET_NEXT(gpd),
				(u32)TGPD_GET_DataBUF_LEN(gpd), TGPD_GET_DATA(gpd),
				(u32)TGPD_GET_BUF_LEN(gpd), (u32)TGPD_GET_EPaddr(gpd));

		return;
	}

	if(!gpd || !gpd_current) {

		QMU_ERR("[RXD][ERROR] EP%d, gpd=%p, gpd_current=%p, ishwo=%d, rx_gpd_last=%p, 	RQCPR=0x%x\n",
							ep_num, gpd, gpd_current, 
							((gpd == NULL) ? 999 : TGPD_IS_FLAGS_HWO(gpd)),
							Rx_gpd_last[ep_num], 
							MGC_ReadQMU32(base, MGC_O_QMU_RQCPR(ep_num)));
		return;
	}

	if(TGPD_IS_FLAGS_HWO(gpd)) {
		QMU_ERR("[RXD][ERROR]""HWO=1!!\n");
		BUG_ON(1);
	}

	/* NORMAL EXEC FLOW */
	while(gpd != gpd_current && !TGPD_IS_FLAGS_HWO(gpd)) {
		u32 rcv_len = (u32)TGPD_GET_BUF_LEN(gpd);

		
#if 1
		
		urb = next_queue_urb(qh);
				
		//QMU_DBG("\n");
		if (!urb) {
			QMU_DBG("extra RX%d ready\n", ep_num);
			 mtkfsh_qmu_stop(ep_num, USB_DIR_IN);
			return;
		}
#endif

		if (!TGPD_GET_NEXT(gpd) || !TGPD_GET_DATA(gpd)) {
			QMU_ERR("[RXD][ERROR] EP%d ,gpd=%p\n", ep_num, gpd);
			BUG_ON(1);
		}
		if(usb_pipebulk(urb->pipe) && urb->transfer_buffer_length >= QMU_RX_SPLIT_THRE && usb_pipein(urb->pipe)){
			urb->actual_length += TGPD_GET_BUF_LEN(gpd);
		    qh->offset += TGPD_GET_BUF_LEN(gpd);
			qh->iso_idx++;
			done = (qh->iso_idx == urb->number_of_packets) ? true : false ;
		}else if(usb_pipeisoc(urb->pipe)){
			struct usb_iso_packet_descriptor	*d;
			d = urb->iso_frame_desc + qh->iso_idx;
			d->actual_length = rcv_len;
			urb->actual_length += rcv_len;
			qh->offset += TGPD_GET_BUF_LEN(gpd);
			qh->iso_idx++;
			done = (qh->iso_idx == urb->number_of_packets) ? true : false ;
		}else{
			urb->actual_length = TGPD_GET_BUF_LEN(gpd);
			qh->offset = TGPD_GET_BUF_LEN(gpd);
			done = true;
		}
		
		gpd = TGPD_GET_NEXT(gpd);
		//QMU_DBG("gpd = %p ep_num = %d\n",gpd,ep_num);
		gpd = gpd_phys_to_virt((dma_addr_t)gpd, RXQ, ep_num);
		QMU_DBG("gpd = %p ep_num = %d\n",gpd,ep_num);
		if(!gpd) {
			QMU_ERR("[RXD][ERROR]""%s EP%d ,gpd=%p\n", __func__, ep_num, gpd);
			BUG_ON(1);
		}
		QMU_DBG("gpd = %p ep_num = %d\n",gpd,ep_num);
		Rx_gpd_last[ep_num] = gpd;
		QMU_DBG("gpd = %p ep_num = %d\n",gpd,ep_num);
		QMU_DBG("hw_ep = %p\n",hw_ep);



		QMU_DBG("done:%d\n", done);
		if(done){
			musbfsh_advance_schedule(musbfsh, urb, hw_ep, USB_DIR_IN);			
			if(musbfsh_ep_get_qh(hw_ep, is_in))
				qh->iso_idx = 0;
			//break;
		}
	}
	/* QMU should keep take HWO gpd , so there is error*/
	if(gpd != gpd_current && TGPD_IS_FLAGS_HWO(gpd)) {
		QMU_ERR("[RXD][ERROR]""gpd=%p\n", gpd);

		QMU_ERR("[RXD][ERROR]""EP%d RQCSR=%x, RQSAR=%x, RQCPR=%x, RQLDPR=%x\n",
				ep_num, 
				MGC_ReadQMU32(base, MGC_O_QMU_RQCSR(ep_num)), 
				MGC_ReadQMU32(base, MGC_O_QMU_RQSAR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_RQCPR(ep_num)),
				MGC_ReadQMU32(base, MGC_O_QMU_RQLDPR(ep_num))); 

		QMU_ERR("[RXD][ERROR]""QCR0=%x, QCR2=%x, QCR3=%x, QGCSR=%x\n", 
				MGC_ReadQMU32(base, MGC_O_QMU_QCR0), 
				MGC_ReadQMU32(base, MGC_O_QMU_QCR2), 
				MGC_ReadQMU32(base, MGC_O_QMU_QCR3), 
				MGC_ReadQUCS32(base, MGC_O_QUCS_USBGCSR));

		QMU_ERR("[RXD][ERROR]""HWO=%d, Next_GPD=%p ,DataBufLen=%d, "
			"DataBuf=%p, RecvLen=%d, Endpoint=%d\n",
			(u32)TGPD_GET_FLAG(gpd), TGPD_GET_NEXT(gpd),
			(u32)TGPD_GET_DataBUF_LEN(gpd), TGPD_GET_DATA(gpd),
			(u32)TGPD_GET_BUF_LEN(gpd), (u32)TGPD_GET_EPaddr(gpd));
	}

	QMU_INFO("[RXD]""%s EP%d, Last=%p, End=%p, complete\n", __func__,
		ep_num, Rx_gpd_last[ep_num], Rx_gpd_end[ep_num]);
	QMU_DBG("\n");
}

extern void musbfsh_advance_schedule(struct musbfsh *musbfsh, struct urb *urb,
				  struct musbfsh_hw_ep *hw_ep, int is_in);
void mtkfsh_qmu_done_tx(struct musbfsh *musbfsh, u8 ep_num)
{
	void __iomem* base = usb11_qmu_base;
	TGPD* gpd = Tx_gpd_last[ep_num];
	TGPD* gpd_current = (TGPD*)(unsigned long)MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num));
	TGPD* real_gpd_current;
	//struct musbfsh_ep		*musbfsh_ep = &musbfsh->endpoints[ep_num].ep_in;
	struct musbfsh_hw_ep    *hw_ep = musbfsh->endpoints + ep_num;         
    struct musbfsh_qh        *qh = hw_ep->out_qh;
	struct urb    *urb = NULL;	
	bool done = true;
	int is_in = qh->is_in;
	urb = next_queue_urb(qh);
	if (!urb) {
		QMU_DBG("extra TX%d ready\n", ep_num);
		 mtkfsh_qmu_stop(ep_num, USB_DIR_OUT);
		return;
	}
	/*Transfer PHY addr got from QMU register to VIR addr*/
	gpd_current = gpd_phys_to_virt((dma_addr_t)gpd_current, TXQ, ep_num);

	/*
                      gpd or Last       gdp_current
                           |                  |
            |->  GPD1 --> GPD2 --> GPD3 --> GPD4 --> GPD5 -|
            |----------------------------------------------|
	*/

	QMU_INFO("[TXD]""%s EP%d, Last=%p, Current=%p, End=%p\n",
		__func__, ep_num, gpd, gpd_current, Tx_gpd_end[ep_num]);

	/*gpd_current should at least point to the next GPD to the previous last one.*/
	if (gpd == gpd_current) {
		QMU_ERR("[TXD] gpd(%p) == gpd_current(%p)\n", 
				gpd, gpd_current);
		return;
	}

	if(TGPD_IS_FLAGS_HWO(gpd)) {
		QMU_ERR("[TXD] HWO=1, CPR=%x\n", MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num)));
		BUG_ON(1);
	}
	
	/* NORMAL EXEC FLOW */
	while (gpd != gpd_current && !TGPD_IS_FLAGS_HWO(gpd)) {
		
		QMU_INFO("[TXD]""gpd=%p ->HWO=%d, BPD=%d, Next_GPD=%p, DataBuffer=%p, "
			"BufferLen=%d \n", 
			gpd, (u32)TGPD_GET_FLAG(gpd), (u32)TGPD_GET_FORMAT(gpd),
			TGPD_GET_NEXT(gpd), TGPD_GET_DATA(gpd), (u32)TGPD_GET_BUF_LEN(gpd));

		if(!TGPD_GET_NEXT(gpd)) {
			QMU_ERR("[TXD][ERROR]""Next GPD is null!!\n");
			//BUG_ON(1);
			break;
		}
		urb = next_queue_urb(qh);			
		//QMU_DBG("\n");
		if (!urb) {
			QMU_ERR("extra TX%d ready\n", ep_num);
			 mtkfsh_qmu_stop(ep_num, USB_DIR_OUT);
			return;
		}

		if (!TGPD_GET_NEXT(gpd) || !TGPD_GET_DATA(gpd)) {
			QMU_ERR("[RXD][ERROR] EP%d ,gpd=%p\n", ep_num, gpd);
			BUG_ON(1);
		}
		
		if(usb_pipebulk(urb->pipe) && urb->transfer_buffer_length >= QMU_RX_SPLIT_THRE && usb_pipeout(urb->pipe)){
			urb->actual_length += TGPD_GET_BUF_LEN(gpd);
			qh->offset += TGPD_GET_BUF_LEN(gpd);
			qh->iso_idx++;
			done = (qh->iso_idx == urb->number_of_packets) ? true : false ;
		}else if(usb_pipeisoc(urb->pipe)){
			struct usb_iso_packet_descriptor	*d;
			d = urb->iso_frame_desc + qh->iso_idx;
			d->actual_length = TGPD_GET_BUF_LEN(gpd);
			urb->actual_length += TGPD_GET_BUF_LEN(gpd);
			qh->offset += TGPD_GET_BUF_LEN(gpd);
			qh->iso_idx++;
			done = (qh->iso_idx == urb->number_of_packets) ? true : false ;
		}else{
			urb->actual_length = TGPD_GET_BUF_LEN(gpd);
			qh->offset = TGPD_GET_BUF_LEN(gpd);
			done = true;
		}
		
		gpd = TGPD_GET_NEXT(gpd);
		gpd = gpd_phys_to_virt((dma_addr_t)gpd, TXQ, ep_num);
		Tx_gpd_last[ep_num] = gpd;
		
		if(done){
			musbfsh_advance_schedule(musbfsh, urb, hw_ep, USB_DIR_OUT);
			if(musbfsh_ep_get_qh(hw_ep, is_in))
				qh->iso_idx = 0;
			
			real_gpd_current = (TGPD*)(unsigned long)MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num));
			real_gpd_current = gpd_phys_to_virt((dma_addr_t)real_gpd_current, TXQ, ep_num);
			QMU_INFO("[TXD]""gpd=%p gpd_current=%p record gpd=%p is_hwo:0x%x ->HWO=%d, BPD=%d, Next_GPD=%p, DataBuffer=%p, "
					"BufferLen=%d \n", 
					gpd, real_gpd_current, gpd_current, TGPD_IS_FLAGS_HWO(gpd), (u32)TGPD_GET_FLAG(gpd), (u32)TGPD_GET_FORMAT(gpd),
					TGPD_GET_NEXT(gpd), TGPD_GET_DATA(gpd), (u32)TGPD_GET_BUF_LEN(gpd));
			QMU_INFO("queue csr:0x%x mask:0x%x tx:0x%x\n", 
						musbfsh_readl(musbfsh->mregs, MUSBFSH_QISAR),
						musbfsh_readl(musbfsh->mregs, MUSBFSH_QIMR),
						musbfsh_readw(musbfsh->mregs, MUSBFSH_INTRTX));
			return;
		}

	real_gpd_current = (TGPD*)(unsigned long)MGC_ReadQMU32(base, MGC_O_QMU_TQCPR(ep_num));
	real_gpd_current = gpd_phys_to_virt((dma_addr_t)real_gpd_current, TXQ, ep_num);
	QMU_INFO("[TXD]""gpd=%p gpd_current=%p record gpd=%p is_hwo:0x%x ->HWO=%d, BPD=%d, Next_GPD=%p, DataBuffer=%p, "
					"BufferLen=%d \n", 
					gpd, real_gpd_current, gpd_current, TGPD_IS_FLAGS_HWO(gpd), (u32)TGPD_GET_FLAG(gpd), (u32)TGPD_GET_FORMAT(gpd),
					TGPD_GET_NEXT(gpd), TGPD_GET_DATA(gpd), (u32)TGPD_GET_BUF_LEN(gpd));
	QMU_INFO("queue csr:0x%x mask:0x%x tx:0x%x\n", 
						musbfsh_readl(musbfsh->mregs, MUSBFSH_QISAR),
						musbfsh_readl(musbfsh->mregs, MUSBFSH_QIMR),
						musbfsh_readw(musbfsh->mregs, MUSBFSH_INTRTX));
	}
}

int mtkfsh_q_schedule(struct musbfsh *musbfsh, struct musbfsh_qh *qh, int  isRx)
{
	int			idle;
	int			best_diff;
	int			best_end, epnum;
	struct musbfsh_hw_ep	*hw_ep = NULL;
	struct list_head	*head = NULL;
	
	if (!musbfsh->is_active)
		return -ENODEV;

	/* use fixed hardware for control and bulk */
	if (qh->type == USB_ENDPOINT_XFER_CONTROL) {
		head = &musbfsh->control;
		hw_ep = musbfsh->control_ep;
		goto success;
	}

	/* else, periodic transfers get muxed to other endpoints */

	/*
	 * We know this qh hasn't been scheduled, so all we need to do
	 * is choose which hardware endpoint to put it on ...
	 *
	 * REVISIT what we really want here is a regular schedule tree
	 * like e.g. OHCI uses.
	 */
	best_diff = 4096;
	best_end = -1;

	for (epnum = 1, hw_ep = musbfsh->endpoints + 1;
			epnum < musbfsh->nr_endpoints;
			epnum++, hw_ep++) {
		//int	diff;

		if (musbfsh_ep_get_qh(hw_ep, isRx) != NULL)
			continue;

		//if (hw_ep == musbfsh->bulk_ep)
		//	continue;


		hw_ep = musbfsh->endpoints + epnum;//got the right ep
		QMU_DBG("musbfsh_schedule:: find a hw_ep%d\n",hw_ep->epnum);
		break;
	}

	if(!hw_ep){
        QMU_DBG("musbfsh::error!not find a ep for the urb\r\n");
        return -1;
     }

	idle = 1;
	qh->mux = 0;
	//hw_ep = musbfsh->endpoints + best_end;
	QMU_DBG("qh %p periodic slot %d\n", qh, best_end);
success:
	if (head) {
		idle = list_empty(head);
		list_add_tail(&qh->ring, head);
		qh->mux = 1;
	}
	qh->hw_ep = hw_ep;
	qh->hep->hcpriv = qh;
	
	if(qh->type != USB_ENDPOINT_XFER_CONTROL)
		musbfsh_allocate_ep_fifo(musbfsh, qh);

	if (idle) {
		mtkfsh_kick_CmdQ(musbfsh, isRx, qh);
                QMU_DBG("mtkfsh_kick_CmdQ!!!!!!!!\n");
    }
	return 0;
 }

