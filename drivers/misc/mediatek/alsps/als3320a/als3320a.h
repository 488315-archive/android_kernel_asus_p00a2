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
 * Filename: als3320a.h
 *
 * Summary:
 *	ALS3320A sensor dirver header file.
 *
 * Modification History:
 * Date	By		Summary
 * -------- -------- -------------------------------------------------------
 * 05/11/12 YC		Original Creation (Test version:1.0)
 */

/*
 * Definitions for ALS3320A als/ps sensor chip.
 */
#ifndef __ALS3320A_H__
#define __ALS3320A_H__

#include <linux/ioctl.h>

#define ALS3320A_ENABLE				0X00
#define ALS3320A_INT_STATUS			0x01
#define ALS3320A_CONTROL_INT			0x02
#define ALS3320A_ALS_WAITING			0x06
#define ALS3320A_GAIN					0x07
#define ALS3320A_ALS_PERSIST			0x08
#define ALS3320A_ALS_MEAN_TIME		0x09
#define ALS3320A_ADATA_L				0x22
#define ALS3320A_ADATA_H				0x23
#define ALS3320A_INT_LOW_THD_LOW		0x30
#define ALS3320A_INT_LOW_THD_HIGH	0x31
#define ALS3320A_INT_HIGH_THD_LOW	0x32
#define ALS3320A_INT_HIGH_THD_HIGH	0x33
#define ALS3320A_ALS_CALIBRATIONH		0x34

#define ALS3320A_SUCCESS						0
#define ALS3320A_ERR_I2C						-1
#define ALS3320A_ERR_STATUS					-3
#define ALS3320A_ERR_SETUP_FAILURE			-4
#define ALS3320A_ERR_GETGSENSORDATA			-5
#define ALS3320A_ERR_IDENTIFICATION			-6

//SYSTEM MODE (ALS3320A_REG_SYS_CONF)
#define	ALS3320A_SYS_DEV_DOWN			0x00
#define	ALS3320A_SYS_ALS_ENABLE			0x01
#define	ALS3320A_SYS_PS_ENABLE			0x02
#define	ALS3320A_SYS_ALS_PS_ENABLE		0x03
#define	ALS3320A_SYS_DEV_RESET			0x04

//	define for al3320a address ++
#define INDEX_ALS_SYS_CONFIG		0
#define INDEX_ALS_FLAG_STATUS		1
#define INDEX_ALS_INT_CONFIG		2
#define INDEX_ALS_WAITING			3
#define INDEX_ALS_GAIN				4
#define INDEX_ALS_PERSIST			5
#define INDEX_ALS_MEAN_TIME		6
#define INDEX_ALS_DATA_LOW		7
#define INDEX_ALS_DATA_HIGH		8
#define INDEX_ALS_THRES_LOW_L		9
#define INDEX_ALS_THRES_LOW_H		10
#define INDEX_ALS_THRES_HIGH_L	11
#define INDEX_ALS_THRES_HIGH_H	12
#define INDEX_ALS_CALIBRATION		13
//	define for al3320a address --



#endif
