/*
 *
 * FocalTech fts TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /*******************************************************************************
*
* File Name: Focaltech_ex_fun.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*   
* Modify by mshl on 2015-07-06
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include <linux/kernel.h>

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER	2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA		6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG				8
#define PROC_NAME	"ftxxxx-debug"

#define WRITE_BUF_SIZE		512
#define READ_BUF_SIZE		512


// add by leo ++
#define boolean unsigned char

/* Add by Tom for Create Node in sys/kernel/android_touch */
struct kobject *android_touch_kobj = NULL;

extern struct fts_ts_data *private_ts;
extern int ft5726_touch_status;

extern int rawdata_test_result;
extern int uniformity_test_result;
extern int scap_cb_test_result;
extern int scap_rawdata_test_result;
extern int FT5726_TP_ID;

extern boolean start_test_tp(void);
extern void ft5726_irq_enable(void);
extern void ft5726_irq_disable(void);
extern void free_test_param_data(void);
extern void fts_update_fw_vendor_id(struct fts_ts_data *data);
extern int get_test_data(char *pTestData);
extern int set_param_data(char * TestParamData);
extern int fts_ctpm_fw_download_with_i_file(struct i2c_client *client, char *firmware_name);


/* Add by Tom Cheng for Test USB Status Notify */
extern int usb_state;
extern int ft5726_cable_status_handler(int state);
#define TYPE_NONE	0
#define AC_IN	1
#define PC_IN	2
#define POWERBANK_IN	3

// add by leo --
/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
#if GTP_ESD_PROTECT
int apk_debug_flag = 0;
#endif
/*******************************************************************************
* Static function prototypes
*******************************************************************************/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		LDBG("copy from user error\n");
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			//disable_irq(fts_i2c_client->irq);
			ft5726_irq_disable();
			#if GTP_ESD_PROTECT
			apk_debug_flag = 1;
			#endif
			
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			#if GTP_ESD_PROTECT
			apk_debug_flag = 0;
			#endif
			//enable_irq(fts_i2c_client->irq);
			ft5726_irq_enable();
			if (ret < 0) {
				LDBG("upgrade failed.\n");
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			LDBG("write iic error\n");
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			LDBG("write iic error\n");
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("autoclb\n");
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			LDBG("write iic error\n");
			return ret;
		}
		break;
	default:
		break;
	}
	

	return count;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			LDBG("read iic error\n");
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			LDBG("read iic error\n");
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	if (copy_to_user(buff, buf, num_read_chars)) {
		LDBG("copy to user error\n");
		return -EFAULT;
	}

	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
		.owner = THIS_MODULE,
		.read = fts_debug_read,
		.write = fts_debug_write,
		
};
#else
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		LDBG("copy from user error\n");
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			//disable_irq(fts_i2c_client->irq);
			ft5726_irq_disable();
			#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
			#endif
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
			#endif
			//enable_irq(fts_i2c_client->irq);
			ft5726_irq_enable();
			if (ret < 0) {
				LDBG("upgrade failed.\n");
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			LDBG("write iic error\n");
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			LDBG("write iic error\n");
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("autoclb\n");

		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			LDBG("write iic error\n");
			return ret;
		}
		break;
	default:
		break;
	}
	

	return len;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	int ret = 0;
	unsigned char buf[READ_BUF_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			LDBG("read iic error\n");
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			LDBG("read iic error\n");
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);
	return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(struct i2c_client * client)
{	
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
		fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);		
	#else
		fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	#endif
	if (NULL == fts_proc_entry) 
	{
		LDBG("Couldn't create proc entry!\n");
		
		return -ENOMEM;
	} 
	else 
	{
		LDBG("Create proc entry success!\n");
		
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
			fts_proc_entry->write_proc = fts_debug_write;
			fts_proc_entry->read_proc = fts_debug_read;
		#endif
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{
	
	if (fts_proc_entry)
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
			proc_remove(fts_proc_entry);
		#else
			remove_proc_entry(NULL, fts_proc_entry);
		#endif
}

/************************************************************************
* Name: fts_tpfwver_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	mutex_lock(&fts_input_dev->mutex);
	if (fts_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0)
		return -1;
	
	
	if (fwver == 255)
		num_read_chars = snprintf(buf, 128,"get tp fw version fail!\n");
	else
	{
		num_read_chars = snprintf(buf, 128, "%02X\n", fwver);
	}
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpfwver_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tpdriver_version_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpdriver_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	
	mutex_lock(&fts_input_dev->mutex);
	
	num_read_chars = snprintf(buf, 128,"%s \n", FTS_DRIVER_INFO);
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpdriver_version_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpdriver_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;
	if (num_read_chars != 2) 
	{
		if (num_read_chars != 4) 
		{
			LDBG("please input 2 or 4 character\n");
			goto error_return;
		}
	}
	memcpy(valbuf, buf, num_read_chars);
	retval = kstrtoul(valbuf, 16, &wmreg);
	if (0 != retval) 
	{
		LDBG("ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", buf);
		goto error_return;
	}
	if (2 == num_read_chars) 
	{
		/*read register*/
		regaddr = wmreg;
		LDBG("[test](0x%02x)\n", regaddr);
		if (fts_read_reg(client, regaddr, &regvalue) < 0) {
			LDBG("Could not read the register(0x%02x)\n", regaddr);
		} else {
			LDBG("the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
		}
	} 
	else 
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if (fts_write_reg(client, regaddr, regvalue)<0) {
			LDBG("Could not write the register(0x%02x)\n", regaddr);
		} else {
			LDBG("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
		}
	}
	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupdate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupdate_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	mutex_lock(&fts_input_dev->mutex);
	
	//disable_irq(client->irq);
	ft5726_irq_disable();
	#if GTP_ESD_PROTECT
		apk_debug_flag = 1;
	#endif
	
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		LDBG("upgrade to new version 0x%x\n", uc_host_fm_ver);
	}
	else
	{
		LDBG("ERROR : upgrade failed ret=%d.\n", i_ret);
	}
	
	#if GTP_ESD_PROTECT
		apk_debug_flag = 0;
	#endif
	//enable_irq(client->irq);
	ft5726_irq_enable();
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
// add by leo ++
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwdownload_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/*
	if (gpio_is_valid(private_ts->pdata->reset_gpio)) {
		gpio_set_value_cansleep(private_ts->pdata->reset_gpio, 0);
		mdelay(10);
		gpio_set_value_cansleep(private_ts->pdata->reset_gpio, 1);
	}
	*/
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_tpfwdownload_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	LDBG("START ... fwname: %s\n", fwname); // add by leo for testtest

	mutex_lock(&fts_input_dev->mutex);
	
	//disable_irq(client->irq);
	ft5726_irq_disable();
	#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
	#endif
	//fts_ctpm_fw_upgrade_with_app_file(client, fwname);
	fts_ctpm_fw_download_with_i_file(client, fwname); // add by leo for testtest
	#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
	#endif
	//enable_irq(client->irq);
	ft5726_irq_enable();
	
	mutex_unlock(&fts_input_dev->mutex);
	return count;
}
// add by leo --
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupgradeapp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	LDBG("START ... fwname: %s\n", fwname); // add by leo for testtest

	mutex_lock(&fts_input_dev->mutex);
	
	//disable_irq(client->irq);
	ft5726_irq_disable();
	#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
	#endif
	fts_ctpm_fw_upgrade_with_app_file(client, fwname);
	#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
	#endif
	//enable_irq(client->irq);
	ft5726_irq_enable();
	
	mutex_unlock(&fts_input_dev->mutex);
	return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	return -EPERM;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

// add by leo ++
static ssize_t ft5726_touch_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ft5726_touch_status);
}

static ssize_t ft5726_touch_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return -EPERM;
}

static ssize_t ft5726_disable_irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	LDBG("disable_irq\n");
	ft5726_irq_disable();

	return sprintf(buf, "Disable IRQ \n");
}

static ssize_t ft5726_disable_irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return -EPERM;
}

static ssize_t ft5726_enable_irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	LDBG("enable_irq\n");
	ft5726_irq_enable();

	return sprintf(buf, "Enable IRQ \n");
}

static ssize_t ft5726_enable_irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return -EPERM;
}
static ssize_t ft5726_reset_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(private_ts->pdata->reset_gpio));
}

static ssize_t ft5726_reset_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rst_pin = 0;
	sscanf(buf, "%d\n", &rst_pin);

	gpio_set_value(private_ts->pdata->reset_gpio, (rst_pin > 0 ? 1 : 0));
	LDBG("set reset_gpio = %s\n", (rst_pin > 0 ? "High" : "Low"));
	LDBG("reset_gpio value= %d\n", gpio_get_value(private_ts->pdata->reset_gpio));
        return count;
}
static ssize_t ft5726_tpid_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", FT5726_TP_ID);
}

static ssize_t ft5726_tpid_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return count;
}

static ssize_t ft5726_fw_vendor_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	fts_update_fw_vendor_id(private_ts);

	return sprintf(buf, "0x%02X\n", private_ts->fw_vendor_id);
}

static ssize_t ft5726_fw_vendor_id_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return count;
}
// add by leo --

#define FT5X0X_CFG_FILEPATH "/data/"
int selft_test_result = 0;

static int ft5x0x_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_CFG_FILEPATH, config_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		LDBG("error occured while opening file %s. \n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadInIData(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_CFG_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		LDBG("error occured while opening file %s. \n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	config_buf[fsize] = '\0';
	return 0;
}

static int ft5x0x_get_testparam_from_ini(char *config_name)
{
	char *config_data = NULL;
	int file_size;

	file_size = ft5x0x_GetInISize(config_name);

	LDBG("inisize = %d \n ", file_size);
	if (file_size <= 0) {
		LDBG("ERROR : Get firmware size failed \n");
		return -EIO;
	}

	config_data = kmalloc(file_size + 1, GFP_ATOMIC);

	if (ft5x0x_ReadInIData(config_name, config_data)) {
		LDBG("ERROR: request_firmware failed \n");
		kfree(config_data);
		return -EIO;
	}
	else {
		LDBG("ft5x0x_ReadInIData successful \n");
	}

	set_param_data(config_data);

	return 0;
}

static mm_segment_t oldfs;

static struct file *fts_selftest_file_open(void)
{

	struct file* filp = NULL;
	char filepath[128];
	int err = 0;

	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s%s", FT5X0X_CFG_FILEPATH, "touch_selftest.csv");

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filepath, O_WRONLY|O_CREAT, 0644);
	if(IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}

	return filp;
}

int fts_selftest_file_write(struct file* file, unsigned char *data, int len)
{

	int ret;

	ret = file->f_op->write(file, data, len, &file->f_pos);

	return ret;
}

void fts_selftest_file_close(struct file* file)
{
	set_fs(oldfs);
	filp_close(file, NULL);
}

static ssize_t ft5726_test_tp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	LDBG("rawdata_test_result = %d uniformity_test_result = %d, scap_cb_test_result = %d, scap_rawdata_test_result = %d",
		rawdata_test_result, uniformity_test_result, scap_cb_test_result, scap_rawdata_test_result);

	return sprintf(buf, "%d, %d, %d, %d\n", rawdata_test_result, uniformity_test_result, scap_cb_test_result, scap_rawdata_test_result);
}

static ssize_t ft5726_test_tp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	char config_file[128];
	char *w_buf;
	struct file *w_file;
	int w_len, i = 0;

	mutex_lock(&fts_input_dev->mutex);
	//isable_irq(client->irq);
	ft5726_irq_disable();

	selft_test_result = 0;

	memset(config_file, 0, sizeof(config_file));
	sprintf(config_file, "%s", buf);
	config_file[count-1] = '\0';

	if (ft5x0x_get_testparam_from_ini(config_file) < 0) {
		LDBG("get testparam from ini failure \n");
	}
	else {
		LDBG("tp test Start... \n");

		if (start_test_tp()) {
			LDBG("tp test pass \n");
			selft_test_result = 0;
		}
		else {
			LDBG("tp test failure \n");
			selft_test_result = 1;
		}

		for (i = 0; i < 3; i++) {
			if (fts_write_reg(client, 0x00, 0x00) >= 0)
				break;
			else
				msleep(200);
		}

		w_file = fts_selftest_file_open();
		if(!w_file) {
			LDBG("Open log file fail !\n");
		} else {
			w_buf = kmalloc(80*1024, GFP_KERNEL);
			if(!w_buf) {
				LDBG("allocate memory fail !\n");
			} else {
				w_len = get_test_data(w_buf);
				fts_selftest_file_write(w_file, w_buf, w_len);
				kfree(w_buf);
			}
			fts_selftest_file_close(w_file);
		}

		free_test_param_data();
	}

	//enable_irq(client->irq);
	ft5726_irq_enable();
	mutex_unlock(&fts_input_dev->mutex);

        return count;
}
static ssize_t ft5726_usb_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	switch(usb_state)
 	{
  		case TYPE_NONE:
   			LDBG("TYPE_NONE\n");
			return sprintf(buf, "TYPE_NONE\n");

    		case AC_IN:
    			LDBG("AC_IN\n");
			return sprintf(buf, "AC_IN\n");

    		case PC_IN:
     			LDBG("PC_IN\n");
			return sprintf(buf, "PC_IN\n");

     		case POWERBANK_IN:
    			LDBG("POWERBANK_IN\n");
			return sprintf(buf, "POWERBANK_IN\n");

 		default:
			LDBG("wrong cable type ..\n");
			return sprintf(buf, "wrong cable type\n");
	}
}

static ssize_t ft5726_usb_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	int temp_status = 0;
	sscanf(buf, "%d\n", &temp_status);
	switch(temp_status)
 	{
  		case TYPE_NONE:
   			LDBG("TYPE_NONE\n");
			break;
    		case AC_IN:
    			LDBG("AC_IN\n");
			break;

    		case PC_IN:
     			LDBG("PC_IN\n");
			break;

     		case POWERBANK_IN:
    			LDBG("POWERBANK_IN\n");
			break;

 		default:
			LDBG("wrong cable type ..\n");
			break;
	}
	ft5726_cable_status_handler(temp_status);

        return count;
}
// add by leo --

/****************************************/
/* sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

static DEVICE_ATTR(ftstpdriverver, S_IRUGO|S_IWUSR, fts_tpdriver_version_show, fts_tpdriver_version_store);
/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, fts_fwupdate_show, fts_fwupdate_store);
/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, fts_fwupgradeapp_show, fts_fwupgradeapp_store);
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, fts_getprojectcode_show, fts_getprojectcode_store);

// add by leo ++
static DEVICE_ATTR(ftstpfwdownload, S_IRUGO|S_IWUSR, fts_tpfwdownload_show, fts_tpfwdownload_store);
static DEVICE_ATTR(touch_status, S_IRUGO|S_IWUSR, ft5726_touch_status_show, ft5726_touch_status_store);
static DEVICE_ATTR(disable_irq, S_IWUSR | S_IRUGO, ft5726_disable_irq_show, ft5726_disable_irq_store);
static DEVICE_ATTR(enable_irq, S_IWUSR | S_IRUGO, ft5726_enable_irq_show, ft5726_enable_irq_store);
static DEVICE_ATTR(reset_pin, S_IWUSR | S_IRUGO, ft5726_reset_pin_show, ft5726_reset_pin_store);
static DEVICE_ATTR(tp_id, S_IWUSR | S_IRUGO, ft5726_tpid_pin_show, ft5726_tpid_pin_store);
static DEVICE_ATTR(fw_vendor_id, S_IWUSR | S_IRUGO, ft5726_fw_vendor_id_show, ft5726_fw_vendor_id_store);
static DEVICE_ATTR(test_tp, S_IRUGO|S_IWUSR, ft5726_test_tp_show, ft5726_test_tp_store);
static DEVICE_ATTR(usb_status, S_IRUGO|S_IWUSR, ft5726_usb_status_show, ft5726_usb_status_store);
// add by leo --

/*add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftstpdriverver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,
	// add by leo ++
	&dev_attr_ftstpfwdownload.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_disable_irq.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_reset_pin.attr,
	&dev_attr_tp_id.attr,
	&dev_attr_fw_vendor_id.attr,
	&dev_attr_test_tp.attr,
	&dev_attr_usb_status.attr,
	// add by leo --
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: fts_touch_sysfs_init
* Brief:  create sysfs for debug in sys/kernel/android_touch
* Input: no
* Output: no
* Return: success = 0
***********************************************************************/
/* Add by Tom for Create Node in sys/kernel/android_touch */
int fts_touch_sysfs_init(void)
{
	int ret;
	
	android_touch_kobj = kobject_create_and_add("android_touch", kernel_kobj);
	if (android_touch_kobj == NULL) {
		LDBG("subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
	
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ftstpfwver.attr);
	if (ret) {
		LDBG(" &dev_attr_ftstpfwver.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ftstpdriverver.attr);
	if (ret) {
		LDBG(" &dev_attr_ftstpdriverver.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ftsfwupdate.attr);
	if (ret) {
		LDBG(" &dev_attr_ftsfwupdate.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ftstprwreg.attr);
	if (ret) {
		LDBG(" &dev_attr_ftstprwreg.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ftsfwupgradeapp.attr);
	if (ret) {
		LDBG(" &dev_attr_ftsfwupgradeapp.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ftsgetprojectcode.attr);
	if (ret) {
		LDBG(" &dev_attr_ftsgetprojectcode.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ftstpfwdownload.attr);
	if (ret) {
		LDBG(" &dev_attr_ftstpfwdownload.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_status.attr);
	if (ret) {
		LDBG("&dev_attr_touch_status.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, 	&dev_attr_disable_irq.attr);
	if (ret) {
		LDBG("	&dev_attr_disable_irq.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_enable_irq.attr);
	if (ret) {
		LDBG("&dev_attr_enable_irq.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_reset_pin.attr);
	if (ret) {
		LDBG("&dev_attr_reset_pin.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_id.attr);
	if (ret) {
		LDBG("&dev_attr_tp_id.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj,&dev_attr_fw_vendor_id.attr );
	if (ret) {
		LDBG("&dev_attr_fw_vendor_id.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj,&dev_attr_test_tp.attr );
	if (ret) {
		LDBG("&dev_attr_test_tp.attr failed\n");
		return ret;
	}
	return 0;
}

/************************************************************************
* Name: fts_create_sysfs
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_sysfs(struct i2c_client * client)
{
	int err;
	
	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err) 
	{
		LDBG("ERROR: sysfs_create_group() failed.\n");
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	} 
	else 
	{
		LDBG("sysfs_create_group() succeeded.\n");
	}
	return err;
}
/************************************************************************
* Name: fts_remove_sysfs
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_remove_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	return 0;
}
