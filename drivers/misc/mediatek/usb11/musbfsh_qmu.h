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
#ifndef _MUSBFSH_QMU_H_
#define _MUSBFSH_QMU_H_

#include "musbfsh_core.h"		/* for struct musbfsh */

extern int musbfsh_qmu_init(struct musbfsh *musbfsh);
extern void musbfsh_qmu_exit(struct musbfsh *musbfsh);
extern void musbfsh_disable_q_all(struct musbfsh *musbfsh);
extern irqreturn_t musbfsh_q_irq(struct musbfsh *musbfsh);
extern void musbfsh_flush_qmu(u32 ep_num, u8 isRx);
extern bool musbfsh_is_qmu_stop(u32 ep_num, u8 isRx);
extern void musbfsh_tx_zlp_qmu(struct musbfsh *musbfsh, u32 ep_num);

/*FIXME, not good layer present */
extern void mtk_qmu_enable(struct musbfsh *musbfsh, u8 EP_Num, u8 isRx);
extern void __iomem *usb11_qmu_base;

#endif
