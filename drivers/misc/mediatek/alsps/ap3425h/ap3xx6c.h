/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3xx6.h
 *
 * Summary:
 *	AP3xx6 sensor dirver header file.
 *
 * Modification History:
 * Date	By		Summary
 * -------- -------- -------------------------------------------------------
 * 05/11/12 YC		Original Creation (Test version:1.0)
 */

/*
 * Definitions for AP3xx6 als/ps sensor chip.
 */
#ifndef __AP3xx6_H__
#define __AP3xx6_H__

#include <linux/ioctl.h>

#define AP3xx6_ENABLE						0X00
#define AP3xx6_INT_STATUS					0x01
#define AP3xx6_CONYROL_INT					0x02
#define AP3xx6_GAIN							0x10
#define AP3xx6_ADATA_L						0X0C
#define AP3xx6_ADATA_H						0X0D
#define AP3xx6_PDATA_L						0X0E
#define AP3xx6_PDATA_H						0X0F
#define AP3xx6_INT_LOW_THD_LOW			0X2A
#define AP3xx6_INT_LOW_THD_HIGH			0X2B
#define AP3xx6_INT_HIGH_THD_LOW			0X2C
#define AP3xx6_INT_HIGH_THD_HIGH			0X2D

#define AP3425_REG_SYS_CONF        0x00
#define AP3425_REG_SYS_CONF_SHIFT	(0)
#define AP3425_REG_SYS_CONF_MASK	0x07

#define AP3425_REG_SYS_INTSTATUS   0x01
#define AP3425_REG_SYS_INT_SHIFT   (0)
#define AP3425_REG_SYS_INT_PS_SHIFT	(1)
#define AP3425_REG_SYS_INT_LS_SHIFT	(0)
#define AP3425_REG_SYS_INT_MASK		0x03
#define AP3425_REG_SYS_INT_PMASK		0x02
#define AP3425_REG_SYS_INT_AMASK		0x01



#define AP3425_REG_ALS_THDL_L      0x1A
#define AP3425_REG_ALS_THDL_L_SHIFT	(0)
#define AP3425_REG_ALS_THDL_L_MASK	0xFF

#define AP3425_REG_ALS_THDL_H      0x1B
#define AP3425_REG_ALS_THDL_H_SHIFT	(0)
#define AP3425_REG_ALS_THDL_H_MASK	0xFF

#define AP3425_REG_ALS_THDH_L      0x1C
#define AP3425_REG_ALS_THDH_L_SHIFT	(0)
#define AP3425_REG_ALS_THDH_L_MASK	0xFF

#define AP3425_REG_ALS_THDH_H      0x1D
#define AP3425_REG_ALS_THDH_H_SHIFT	(0)
#define AP3425_REG_ALS_THDH_H_MASK	0xFF
//SYSTEM MODE (AP3425_REG_SYS_CONF)
#define	AP3425_SYS_DEV_DOWN        0x00
#define	AP3425_SYS_ALS_ENABLE      0x01
#define	AP3425_SYS_PS_ENABLE       0x02
#define	AP3425_SYS_ALS_PS_ENABLE   0x03
#define	AP3425_SYS_DEV_RESET       0x04

#define AP3xx6_SUCCESS						0
#define AP3xx6_ERR_I2C						-1
#define AP3xx6_ERR_STATUS					-3
#define AP3xx6_ERR_SETUP_FAILURE			-4
#define AP3xx6_ERR_GETGSENSORDATA			-5
#define AP3xx6_ERR_IDENTIFICATION			-6

#endif
