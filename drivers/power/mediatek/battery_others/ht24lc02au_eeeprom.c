/*
 * Copyright (c) 2015, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include "asus_battery.h"
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/kthread.h>

/*qcom*/
#include <linux/uaccess.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>

#define HT24LC02AU_DEV_NAME "ht24lc02au"
#define I2C_RETRY_COUNT 2
#define I2C_RETRY_DELAY 5
#define I2C_WRITE_DELAY 1

/*
ISN: isn string
SSN: ssn string
MOD: model name string
COV: cover type string
ACT: activated flag string
*/

/* max number of bytes (each register in EEPROM can store 1 byte) */
#define NUM_BYTE_ISN    32
#define NUM_BYTE_SSN    20
#define NUM_BYTE_MOD     5
#define NUM_BYTE_RSV     3
#define NUM_BYTE_TIN     1
#define NUM_BYTE_COV     2
#define NUM_BYTE_ACT     1

/* start register address where to stored data (0x00 ~ 0xFF) */
#define ISN_START_REG    0
#define SSN_START_REG    (ISN_START_REG + NUM_BYTE_ISN)
#define MOD_START_REG    (SSN_START_REG + NUM_BYTE_SSN)
#define RSV_START_REG    (MOD_START_REG + NUM_BYTE_MOD)
#define TIN_START_REG    (RSV_START_REG + NUM_BYTE_RSV)
#define COV_START_REG    (TIN_START_REG + NUM_BYTE_TIN)
#define ACT_START_REG    (COV_START_REG + NUM_BYTE_COV)

static struct workqueue_struct *eeprom_wq;
static struct workqueue_struct *smb345c_charging_toggle_wq;
static struct workqueue_struct *cover_wq;
static struct workqueue_struct *cvbus_wq;
static DEFINE_SPINLOCK(cvbus_lock);
//CHARGING_CONTROL battery_charging_control = chr_control_interface;
DEFINE_MUTEX(cover_type_lock);
/*
    0: unknown
    1: CB81
    2: CA81
*/
static DECLARE_WAIT_QUEUE_HEAD(cover_gauge_thread_wq);
int g_cover_gauge_init = 0;
EXPORT_SYMBOL_GPL(g_cover_gauge_init);
int g_cover_low_bat_gauge_check = 0;
EXPORT_SYMBOL_GPL(g_cover_low_bat_gauge_check);
int irq_cvbus_count = 0;
static int cover_type;
int usb_in_int = 0;
EXPORT_SYMBOL_GPL(usb_in_int);
int g_cb81_int = 0;
EXPORT_SYMBOL_GPL(g_cb81_int);
int g_sdp_cb81_padcharging = 0;
EXPORT_SYMBOL_GPL(g_sdp_cb81_padcharging);
int g_call_cover_int = 0;
int g_Flag1 = 0;
int g_Flag2 = 0;
EXPORT_SYMBOL_GPL(g_Flag2);
int g_Flag4 = 0;
extern int cover_button(bool state);
int g_cvbus_in_int = 0;
int g_cinit = 0;
static ktime_t last_read_time;
extern int  smb345c_charging_toggle(charging_toggle_level_t level, bool on);
//static bool cover_init=false;
//extern bool COVER_ATTACHED_UPI(void);

struct ht24lc02au_eeprom {
    struct mutex        lock;
    struct i2c_client    *client;
    struct device        *dev;
    struct dentry        *dentry;
    int cover_i2c_enable_gpio;
    int cover_n1_np_det;
    int n1_vbus_in_det;
    int dcin_vbus_in_det_n;
    int irq;
    int cvbus_irq;
    int n1_vbus_in_det_irq;
    struct delayed_work eeprom_work;
    struct delayed_work eeprom_work_usb;
    struct delayed_work cover_work;
    struct delayed_work cover_ac_work;
    struct delayed_work cover_sdp_work;
    struct delayed_work cvbus_work;
    struct delayed_work cvbus_timer_work;
    struct delayed_work smb345c_charging_toggle_work;
};
struct ht24lc02au_eeprom *ht24lc02au;
bool COVER_ATTACHED(void)
{
#ifdef CONFIG_Z380M
	BAT_DBG("%s:%d\n", __func__, gpio_get_value(COVER_ATTACH_GPIO));
	return gpio_get_value(COVER_ATTACH_GPIO);
#else
	return false;
#endif
}
EXPORT_SYMBOL_GPL(COVER_ATTACHED);

bool COVER_ATTACHED_UPI(void)
{
  //  pr_err("%s, cover-i2c-enable-gpio = %d.\n", __func__, ht24lc02au->cover_i2c_enable_gpio);
    return gpio_get_value(COVER_ATTACH_GPIO);
}
EXPORT_SYMBOL_GPL(COVER_ATTACHED_UPI);

static int ht24lc02au_read(struct ht24lc02au_eeprom *ht24, u8 reg)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    if (!COVER_ATTACHED_UPI())
        return -EINVAL;
    do
    {
        ret = i2c_smbus_read_byte_data(ht24->client, reg);
        if (ret < 0) {
            retry_count--;
            dev_warn(&ht24->client->dev, "fail to read reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    return ret;
}

static int ht24lc02au_write(struct ht24lc02au_eeprom *ht24, u8 reg, u8 val)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    if (!COVER_ATTACHED_UPI())
        return -EINVAL;

    do
    {
        ret = i2c_smbus_write_byte_data(ht24->client, reg, val);
        if (ret < 0) {
            retry_count--;
            dev_warn(&ht24->client->dev, "fail to write reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
        else msleep(I2C_WRITE_DELAY);
    } while (ret < 0 && retry_count > 0);

    return ret;
}

/*
return:
    0      means success
    others means fail
*/
int WRITE_EEPROM(u8 address, u8 value)
{
    int ret;

    if (!ht24lc02au)
        return -ENODEV;

    ret = ht24lc02au_write(ht24lc02au, address, value);
    if (ret < 0)
        return ret;

    return 0;
}
EXPORT_SYMBOL_GPL(WRITE_EEPROM);

/*
return:
    Value read from EEPROM.
    Negative value means fail to read.
*/
int READ_EEPROM(u8 address)
{
    if (!ht24lc02au)
        return -ENODEV;

    return ht24lc02au_read(ht24lc02au, address);
}
EXPORT_SYMBOL_GPL(READ_EEPROM);

bool VBUS_IN(void)
{
//	pr_err("%s:%d\n", __func__, gpio_get_value(ht24lc02au->n1_vbus_in_det)?false:true);
	if(gpio_get_value(ht24lc02au->n1_vbus_in_det))
		return false;
	return true;
}
EXPORT_SYMBOL_GPL(VBUS_IN);

bool IS_CA81(void)
{
    int ctype;

    mutex_lock(&cover_type_lock);
    ctype = cover_type;
    mutex_unlock(&cover_type_lock);

    return (ctype == 2);
}
EXPORT_SYMBOL_GPL(IS_CA81);
bool _IS_CA81_(void)
{
    /* CB81 is '1' */
    char CA81_TYPE = '2';

    if (!ht24lc02au)
        return false;

    if (ht24lc02au_read(ht24lc02au, 0x3D) == (int)CA81_TYPE)
    {
        mutex_lock(&cover_type_lock);
        cover_type = 2;
        mutex_unlock(&cover_type_lock);
        return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(_IS_CA81_);

bool IS_CB81(void)
{
    int ctype;

    mutex_lock(&cover_type_lock);
    ctype = cover_type;
    mutex_unlock(&cover_type_lock);

    return (ctype == 1);
}
EXPORT_SYMBOL_GPL(IS_CB81);
bool _IS_CB81_(void)
{
    /* CB81 is '1' */
    char CB81_TYPE = '1';

    if (!ht24lc02au)
        return false;

    if (ht24lc02au_read(ht24lc02au, 0x3D) == (int)CB81_TYPE)
    {
        mutex_lock(&cover_type_lock);
        cover_type = 1;
        mutex_unlock(&cover_type_lock);
        return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(_IS_CB81_);
void SET_COVER_DETACHED(void)
{
    mutex_lock(&cover_type_lock);
    cover_type = 0;
    mutex_unlock(&cover_type_lock);
}
EXPORT_SYMBOL_GPL(SET_COVER_DETACHED);

static bool CA81_WITH_TIN_PLATE(void)
{
    int temp = -1;

    temp = ht24lc02au_read(ht24lc02au, 0x3C);
    BAT_DBG_E(" temp = %d\n", temp);
    if (temp == (int)0x31)
    //if (ht24lc02au_read(ht24lc02au, 0x3C) == 0x31);
        return true;
    return false;
}
bool IS_FAIL_CA81(void)
{
    if (IS_CA81()) {
        if (CA81_WITH_TIN_PLATE())
            return false;
        else
            return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(IS_FAIL_CA81);

int ht24lc02au_write_all_registers(struct seq_file *s, u8 val)
{
    struct ht24lc02au_eeprom *ht24;
    u16 reg, regl, regh;
    int ret;
    u16 array[16] = {0};

    ht24 = ht24lc02au;

    for (regh = 0x00; regh <= 0xF0; regh+=0x10) {
        for (regl = 0x00; regl <= 0x0F; regl+=0x01) {
            reg = regl + regh;
            if (reg >= 0xff) break; // FIXME: cannot read reg0xff
            ret = ht24lc02au_write(ht24, reg, val);
        }
        memset(array, 0, sizeof(array));
    }

    pr_info("\n");
    if (s)
        seq_printf(s, "\n");

    return 0;
}

int ht24lc02au_dump_registers(struct seq_file *s)
{
    struct ht24lc02au_eeprom *ht24;
    u16 reg, regl, regh;
    u16 array[16] = {0};
    ht24 = ht24lc02au;

    if (!COVER_ATTACHED_UPI()) {
        pr_info("* Cover is dettached *\n");
        return 0;
    }
    /* config output high to enable COVER I2C*/
    gpio_set_value(COVER_I2C_ENABLE_GPIO, 1);

    pr_err("========================================== EEPROM registers ==========================================\n");
    pr_err("Array | 0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F\n");
    pr_err("------|-----------------------------------------------------------------------------------------------\n");
    if (s) {
        seq_printf(s, "========================================== EEPROM registers ==========================================\n");
        seq_printf(s, "Array | 0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F\n");
        seq_printf(s, "------|-----------------------------------------------------------------------------------------------\n");
    }
    for (regh = 0x00; regh <= 0xF0; regh+=0x10) {
        for (regl = 0x00; regl <= 0x0F; regl+=0x01) {
            reg = regl + regh;
            if (reg >= 0xff) break; // FIXME: cannot read reg0xff
            array[regl] = ht24lc02au_read(ht24, reg);
        }

        pr_err(" 0x%02x | 0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x\n", regh, array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8], array[9], array[10], array[11], array[12], array[13], array[14], array[15]);
        if (s)
            seq_printf(s, " 0x%02x | 0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x\n", regh, array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8], array[9], array[10], array[11], array[12], array[13], array[14], array[15]);
        memset(array, 0, sizeof(array));
    }

    pr_err("\n");
    if (s)
        seq_printf(s, "\n");

    return 0;
}

#ifdef CHRIS

static int ht24lc02au_read_reg(struct i2c_client *client, int reg,
                u8 *val, int ifDebug)
{
    s32 ret; 
    struct ht24lc02au_eeprom *ht24lc02au_chg;

    ht24lc02au_chg = i2c_get_clientdata(client);
    ret = i2c_smbus_read_byte_data(ht24lc02au_chg->client, reg);
    if (ret < 0) {
        //dev_err(&ht24lc02au_chg->client->dev,
        dev_warn(&ht24lc02au_chg->client->dev,
            "i2c read fail: can't read from Reg%02Xh: %d\n", reg, ret);
        return ret;
    } else {
        *val = ret;
    }
    if (ifDebug)
        pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
            "\n", reg, BYTETOBINARY(*val));

    return 0;
}

static int ht24lc02au_write_reg(struct i2c_client *client, int reg,
                        u8 val)
{
    s32 ret;
    struct ht24lc02au_eeprom *ht24lc02au_chg;

    ht24lc02au_chg = i2c_get_clientdata(client);

    ret = i2c_smbus_write_byte_data(ht24lc02au_chg->client, reg, val);
    if (ret < 0) {
        //dev_err(&ht24lc02au_chg->client->dev,
        dev_warn(&ht24lc02au_chg->client->dev,
            "i2c write fail: can't write %02X to %02X: %d\n",
            val, reg, ret);
        return ret;
    }
    return 0;
}

static int ht24lc02au_masked_read(struct i2c_client *client, int reg, u8 mask)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        rc = ht24lc02au_read_reg(client, reg, &temp, 0);
        if (rc) {
            retry_count--;
            BAT_DBG("*ht24lc02au_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        BAT_DBG("ht24lc02au_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return -1;
    }

    temp &= mask;
    return temp;
}

static int ht24lc02au_masked_write(struct i2c_client *client, int reg,
        u8 mask, u8 val)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
    rc = ht24lc02au_read_reg(client, reg, &temp, 0);
    if (rc) {
        retry_count--;
        BAT_DBG("*ht24lc02au_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        msleep(I2C_RETRY_DELAY);
    }
    } while (rc && retry_count > 0);
    if (rc) {
        BAT_DBG("ht24lc02au_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    temp &= ~mask;
    temp |= val & mask;

    retry_count = I2C_RETRY_COUNT;
    do
    {
    rc = ht24lc02au_write_reg(client, reg, temp);
    if (rc) {
        retry_count--;
        BAT_DBG("*ht24lc02au_write failed: reg=%03X, rc=%d\n", reg, rc);
        msleep(I2C_RETRY_DELAY);
    }
    } while (rc && retry_count > 0);
    if (rc) {
        BAT_DBG("ht24lc02au_write failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    return 0;
}
#endif

static int ht24lc02au_debugfs_show(struct seq_file *s, void *data)
{
    ht24lc02au_dump_registers(s);

    return 0;
}

static int ht24lc02au_debugfs_open(struct inode *inode, struct file *file)
{
    return single_open(file, ht24lc02au_debugfs_show, inode->i_private);
}

static const struct file_operations ht24lc02au_debugfs_fops = {
    .open        = ht24lc02au_debugfs_open,
    .read        = seq_read,
    .llseek        = seq_lseek,
    .release    = single_release,
};

static ssize_t asus_proc_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    char proc_buf[5] = {'\0'};
    int val;

    if (count > sizeof(proc_buf)) {
        BAT_DBG_E("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buf, count)) {
        BAT_DBG_E("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    sscanf(proc_buf, "%X", &val);
    ht24lc02au_write_all_registers(NULL, val);

    return count;
}
static int asus_proc_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s: \n", __func__);
    return 0;
}
static int asus_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, asus_proc_read, NULL);
}
int init_asus_proc_toggle(void)
{
    struct proc_dir_entry *entry;

    static const struct file_operations asus_proc_fops = {
        .owner = THIS_MODULE,
        .open = asus_proc_open,
        .read = seq_read,
        .write = asus_proc_write,
        .llseek = seq_lseek,
        .release = single_release,
    };

    entry = proc_create("driver/eepromw", 0666, NULL,
        &asus_proc_fops);
    if (!entry) {
        pr_info("Unable to create proc/driver/eepromw\n");
        return -EINVAL;
    }

    return 0;
}

/* ============================ Device Attribute ============================ */

/* Acquire cover attached/dettached status */
static ssize_t get_cover_status(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret;

    ret = COVER_ATTACHED();
    return sprintf(buf, "%d\n", ret);
}


static ssize_t get_isn_info(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[64] = {'\0'}; /* 32 byte isn */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = ISN_START_REG; reg < NUM_BYTE_ISN; reg++)
        array[reg] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_isn_info(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 32 byte isn from 0 - 31 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("buf size: %lu, count: %zu\n", sizeof(buf), count);
    for (reg = ISN_START_REG; reg < NUM_BYTE_ISN; reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + reg));
    }

    return count;
}
static ssize_t get_ssn_info(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[32] = {'\0'}; /* 20 byte ssn */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = SSN_START_REG; reg < (SSN_START_REG + NUM_BYTE_SSN); reg++)
        array[reg - SSN_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_ssn_info(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 20 byte ssn from 32 - 51 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %zu\n", count);
    for (reg = SSN_START_REG; reg < (SSN_START_REG + NUM_BYTE_SSN); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - SSN_START_REG)));
    }

    return count;
}
static ssize_t get_model_name(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[8] = {'\0'}; /* 5 byte model name */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = MOD_START_REG; reg < (MOD_START_REG + NUM_BYTE_MOD); reg++)
        array[reg - MOD_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_model_name(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 9 byte model name from 52 - 60 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %zu\n", count);
    for (reg = MOD_START_REG; reg < (MOD_START_REG + NUM_BYTE_MOD); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - MOD_START_REG)));
    }

    return count;
}
static ssize_t get_cover_type(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[8] = {'\0'}; /* 2 byte cover type */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = COV_START_REG; reg < (COV_START_REG + NUM_BYTE_COV); reg++)
        array[reg - COV_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_cover_type(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 2 byte cover type from 61 - 62 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %zu\n", count);
    for (reg = COV_START_REG; reg < (COV_START_REG + NUM_BYTE_COV); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - COV_START_REG)));
    }

    return count;
}
static ssize_t get_activ_flag(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[2] = {'\0'}; /* 1 byte activated flag */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = ACT_START_REG; reg < (ACT_START_REG + NUM_BYTE_ACT); reg++)
        array[reg - ACT_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_activ_flag(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 1 byte activated flag from 63 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %zu\n", count);
    for (reg = ACT_START_REG; reg < (ACT_START_REG + NUM_BYTE_ACT); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - ACT_START_REG)));
    }

    return count;
}
static ssize_t get_tin_plate(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[2] = {'\0'}; /* 1 byte activated flag */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = TIN_START_REG; reg < (TIN_START_REG + NUM_BYTE_TIN); reg++)
        array[reg - TIN_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_tin_plate(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 1 byte activated flag from 63 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %zu\n", count);
    for (reg = TIN_START_REG; reg < (TIN_START_REG + NUM_BYTE_TIN); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - TIN_START_REG)));
    }

    return count;
}
static ssize_t cover_activate(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u8 active;
    int ret;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    active = 0x31;
    ret = ht24lc02au_write(ht24, ACT_START_REG, active);
    if (!ret)
        return snprintf(buf, PAGE_SIZE, "PASS");
    return snprintf(buf, PAGE_SIZE, "FAIL");
}
static DEVICE_ATTR(isn_info, S_IRUGO | S_IWUSR, get_isn_info, set_isn_info);
static DEVICE_ATTR(ssn_info, S_IRUGO | S_IWUSR, get_ssn_info, set_ssn_info);
static DEVICE_ATTR(model_name, S_IRUGO | S_IWUSR, get_model_name, set_model_name);
static DEVICE_ATTR(tin_plate, S_IRUGO | S_IWUSR, get_tin_plate, set_tin_plate);
static DEVICE_ATTR(cover_type, S_IRUGO | S_IWUSR, get_cover_type, set_cover_type);
static DEVICE_ATTR(activ_flag, S_IRUGO | S_IWUSR, get_activ_flag, set_activ_flag);
static DEVICE_ATTR(activate, S_IRUGO | S_IWUSR, cover_activate, set_activ_flag);
static DEVICE_ATTR(cover_status, S_IRUGO, get_cover_status, NULL);

static struct attribute *dev_attrs[] = {
    &dev_attr_isn_info.attr,
    &dev_attr_ssn_info.attr,
    &dev_attr_model_name.attr,
    &dev_attr_tin_plate.attr,
    &dev_attr_cover_type.attr,
    &dev_attr_activ_flag.attr,
    &dev_attr_activate.attr,
    &dev_attr_cover_status.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};
/* ============================ Device Attribute ============================ */
static irqreturn_t cvbus_interrupt_handler(int irq, void *dev_id)
{
	if(BMT_status.old_charging_mode == COVER_AC ||
		BMT_status.old_charging_mode == COVER_SDP ||
		BMT_status.old_charging_mode == COVER_CDP)
	{
		BAT_DBG("[%s] cover_vbus_interrupt = %d, g_cvbus_in_int = %d\n", "cvbus_irq", irq, g_cvbus_in_int);
		/* Begin to deal with interrupt bottom half */
		g_cvbus_in_int = g_cvbus_in_int + 1;
		queue_delayed_work(cvbus_wq, &ht24lc02au->cvbus_work, msecs_to_jiffies(0));
	}
	return IRQ_HANDLED;
}

static irqreturn_t eeprom_interrupt_handler(int irq, void *dev_id)
{
        /* Begin to deal with interrupt bottom half */
	g_Flag1 = 0;
	g_Flag2 = 0;
	g_Flag4 = 0;
	/*cover in*/
        BAT_DBG("[%s] cover attached = %d\n",HT24LC02AU_DEV_NAME ,ht24lc02au->irq);
	if(gpio_get_value(COVER_ATTACH_GPIO))
	{
		if (g_call_cover_int)
			BAT_DBG("--------------call_cover_interupt----------COVER_IN\n");	
		else
			BAT_DBG("-----------------------------------------------COVER_IN\n");	
		cancel_delayed_work(&ht24lc02au->cover_ac_work);
		cancel_delayed_work(&ht24lc02au->cover_sdp_work);
		cancel_delayed_work(&ht24lc02au->cover_work);
		cancel_delayed_work(&ht24lc02au->eeprom_work);
		queue_delayed_work(eeprom_wq, &ht24lc02au->eeprom_work,msecs_to_jiffies(1000));
	}
	/*cover out*/
	else
	{
		BAT_DBG("-----------------------------------------------COVER_OUT\n");
		cancel_delayed_work(&ht24lc02au->cover_ac_work);
		cancel_delayed_work(&ht24lc02au->cover_sdp_work);
		cancel_delayed_work(&ht24lc02au->cover_work);
		cancel_delayed_work(&ht24lc02au->eeprom_work);
		queue_delayed_work(smb345c_charging_toggle_wq, &ht24lc02au->smb345c_charging_toggle_work,msecs_to_jiffies(0));
	        queue_delayed_work(eeprom_wq, &ht24lc02au->eeprom_work,msecs_to_jiffies(1000));
	}
        /* Create the wakelock to ensure work can be finished before system suspend */
        return IRQ_HANDLED;
}

void call_cover_interupt(void)
{
	BAT_DBG("[%s]:\n",HT24LC02AU_DEV_NAME);
	g_call_cover_int = 1;
	eeprom_interrupt_handler(ht24lc02au->irq, ht24lc02au->dev);
}
EXPORT_SYMBOL_GPL(call_cover_interupt);

static irqreturn_t eeprom_interrupt_handler_usb(int irq, void *dev_id)
{
        BAT_DBG("[%s] eeprom_interrupt_usb = %d\n",HT24LC02AU_DEV_NAME ,ht24lc02au->n1_vbus_in_det_irq);
	usb_in_int = 1;
	g_Flag1 = 0;
	g_Flag2 = 0;
	g_Flag4 = 0;
	/* Begin to deal with interrupt bottom half */
	cancel_delayed_work(&ht24lc02au->eeprom_work_usb);
	queue_delayed_work(eeprom_wq, &ht24lc02au->eeprom_work_usb, msecs_to_jiffies(1000));//HZ);
	/* Create the wakelock to ensure work can be finished before system suspend */
	return IRQ_HANDLED;
}


static int set_irq_eeprom(struct ht24lc02au_eeprom *chip)
{
        int rc = 0 ;
        int irq_flags;

        /* Accroding to irq domain mappping GPIO number to IRQ number */
        ht24lc02au->irq = gpio_to_irq(COVER_ATTACH_GPIO);
        ht24lc02au->cvbus_irq = gpio_to_irq(ht24lc02au->dcin_vbus_in_det_n);
        BAT_DBG("[%s] eeprom irq = %d\n",HT24LC02AU_DEV_NAME ,ht24lc02au->irq);
        BAT_DBG("[%s] cover vbus irq = %d\n",HT24LC02AU_DEV_NAME ,ht24lc02au->cvbus_irq);

        irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        /* IRQs requested with this function will be automatically freed on driver detach */
        rc = devm_request_irq(ht24lc02au->dev ,ht24lc02au->irq ,eeprom_interrupt_handler,
                              irq_flags ,"eeprom_irq" ,chip);
        if (rc < 0) {
            BAT_DBG("[%s]Couldn't register for eeprom interrupt,irq = %d, rc = %d\n",HT24LC02AU_DEV_NAME ,ht24lc02au->irq ,rc);
            rc = -EIO;
            return rc ;
        }
        rc = devm_request_irq(ht24lc02au->dev ,ht24lc02au->cvbus_irq ,cvbus_interrupt_handler,
                              irq_flags ,"cvbus_irq" ,chip);
        if (rc < 0) {
            BAT_DBG("[%s]Couldn't register for cover vbus interrupt,irq = %d, rc = %d\n",HT24LC02AU_DEV_NAME ,ht24lc02au->cvbus_irq ,rc);
            rc = -EIO;
            return rc ;
        }

        /* Enable this irq line */
        enable_irq_wake(ht24lc02au->irq);
        enable_irq_wake(ht24lc02au->cvbus_irq);
	irq_cvbus_count = 1;
///***************************** USB ****************************************/
//        /* Accroding to irq domain mappping GPIO number to IRQ number */
        ht24lc02au->n1_vbus_in_det_irq = gpio_to_irq(ht24lc02au->n1_vbus_in_det);
        BAT_DBG("[%s] eeprom irq = %d\n",HT24LC02AU_DEV_NAME ,ht24lc02au->n1_vbus_in_det_irq);

        irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
        /* IRQs requested with this function will be automatically freed on driver detach */
        rc = devm_request_irq(ht24lc02au->dev ,ht24lc02au->n1_vbus_in_det_irq ,eeprom_interrupt_handler_usb, irq_flags ,"eeprom_usb_irq" ,chip);
       if (rc < 0) {
           BAT_DBG("[%s]Couldn't register for eeprom interrupt,irq = %d, rc = %d\n",HT24LC02AU_DEV_NAME ,ht24lc02au->n1_vbus_in_det_irq ,rc);
           rc = -EIO;
          return rc ;
      }

        /* Enable this irq line */
	enable_irq_wake(ht24lc02au->n1_vbus_in_det_irq);

        return 0;
}

static int ht24lc02au_parse_dt(struct ht24lc02au_eeprom *chip)
{
    int ret = 0;
    struct device_node *node = chip->client->dev.of_node;
    if (!node) {
        BAT_DBG("No DT data Failing Probe\n");
        return -EINVAL;
    }
/*-------------------------- ENABLE COVER I2C ---------------------------*/
    chip->cover_i2c_enable_gpio =
        of_get_named_gpio(node, "qcom,cover-i2c-enable-gpio", 0);
    BAT_DBG("qcom,cover-i2c-enable-gpio = %d.\n", chip->cover_i2c_enable_gpio);
    if (!gpio_is_valid(ht24lc02au->cover_i2c_enable_gpio)) {
       BAT_DBG("gpio is not valid: cover_i2c_enable_gpio\n");
        return -EINVAL;
    }
    /* config default value to output high : force enable */
    gpio_set_value(ht24lc02au->cover_i2c_enable_gpio, 1);
    ret = gpio_request(chip->cover_i2c_enable_gpio,"N1-I2C-EN");
    if (ret) {
        dev_err(&chip->client->dev, "unable to request gpio %d\n", chip->cover_i2c_enable_gpio);
	return 0;
    }

/*-------------------------- USB detect ---------------------------*/
    chip->n1_vbus_in_det =
        of_get_named_gpio(node, "qcom,n1_vbus_in_det", 0);
    BAT_DBG("qcom,n1_vbus_in_det = %d.\n", chip->n1_vbus_in_det);
    if (!gpio_is_valid(ht24lc02au->n1_vbus_in_det)){
        BAT_DBG("gpio is not valid: n1_vbus_in_det\n");
        return -EINVAL;
    }

/*-------------------------- COVER detect ---------------------------*/
    chip->cover_n1_np_det =
        of_get_named_gpio(node, "qcom,cover-n1_np_det", 0);
    BAT_DBG("qcom,cover-n1_np_det = %d.\n", chip->cover_n1_np_det);
    if (!gpio_is_valid(ht24lc02au->cover_n1_np_det)){
        BAT_DBG("gpio is not valid: cover_n1_np_det\n");
        return -EINVAL;
    }
    
    node = of_find_compatible_node(NULL, NULL, "ti,bq25896");
    chip->dcin_vbus_in_det_n = of_get_named_gpio(node, "dcin_vbus_in_det_n", 0);
    BAT_DBG(" dcin_vbus_in_det_n = %d.\n", chip->dcin_vbus_in_det_n);
    if (!gpio_is_valid(ht24lc02au->cover_n1_np_det)){
        BAT_DBG("gpio is not valid: dcin_vbus_in_det_n\n");
        return -EINVAL;
    }
    return 0;
}

extern void charger_set_gpio(int gpio_usbbusonnum, int gpio_coverbuson, int gpio_coverbusoff, int gpio_padqbdet);
extern void do_chrdet_int_task(void);
extern void upi_ug31xx_attach(bool attach);
extern int smb3xxc_pre_config(bool initial, bool cover_changed_cable_changed);
extern void upi_ug31xx_alarm_callback_for_asus(void);
extern void upi_ug31xx_alarm_callback(void);
extern void cover_otg_current(int curr);
extern void cover_otg(int on);
extern CHARGER_TYPE mt_get_charger_type(void);
extern int g_otg_enable;
extern void tbl_charger_otg_vbus(int mode);
extern void bq25896_set_wd_rst(unsigned int val);
extern bool CHARGING_FULL(void);

static void back_to_a(void) // cover with ac porting
{
	cancel_delayed_work(&ht24lc02au->cover_ac_work);
	if(VBUS_IN() && COVER_ATTACHED_UPI()){
		BAT_DBG("[%s]\n", __func__);
		queue_delayed_work(cover_wq, &ht24lc02au->cover_ac_work, msecs_to_jiffies(60000));
	}
	//queue_delayed_work(eeprom_wq, &ht24lc02au->cover_ac_work, msecs_to_jiffies(60000));
}

static void back_to_b(void) // cover only porting
{
	bq25896_set_wd_rst(0x1);
	cancel_delayed_work(&ht24lc02au->cover_work);
	if(_IS_CA81_()){
		queue_delayed_work(cover_wq, &ht24lc02au->cover_work, msecs_to_jiffies(60000));
	}
	else{
		if(!VBUS_IN() && COVER_ATTACHED_UPI()){
			BAT_DBG("[%s]\n", __func__);
			queue_delayed_work(cover_wq, &ht24lc02au->cover_work, msecs_to_jiffies(60000));
		}
	}
	//queue_delayed_work(eeprom_wq, &ht24lc02au->cover_work, msecs_to_jiffies(60000));
}

static void back_to_c(void) // cover with sdp porting
{
	cancel_delayed_work(&ht24lc02au->cover_sdp_work);
	if(VBUS_IN() && COVER_ATTACHED_UPI()){
		BAT_DBG("[%s]\n", __func__);
		queue_delayed_work(cover_wq, &ht24lc02au->cover_sdp_work, msecs_to_jiffies(60000));
	}
	//queue_delayed_work(eeprom_wq, &ht24lc02au->cover_sdp_work, msecs_to_jiffies(60000));
}

inline struct power_supply *get_psy_battery(void)
{
    struct class_dev_iter iter;
    struct device *dev;
    static struct power_supply *pst;
    class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        pst = (struct power_supply *)dev_get_drvdata(dev);
        if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
            class_dev_iter_exit(&iter);
            return pst;
        }
    }
    class_dev_iter_exit(&iter);

    return NULL;
}

inline int get_battery_rsoc(int *rsoc)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}

inline struct power_supply *get_psy_pack_bat(void)
{
    struct class_dev_iter iter;
    struct device *dev;
    static struct power_supply *pst;

    class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        pst = (struct power_supply *)dev_get_drvdata(dev);
        if (pst->type == POWER_SUPPLY_TYPE_PACK_BATTERY) {
            class_dev_iter_exit(&iter);
            return pst;
        }
    }
    class_dev_iter_exit(&iter);

    return NULL;
}

int get_pack_bat_vol(int *vol)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_pack_bat();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
    if (!ret)
        *vol = val.intval;

    return ret;
}
EXPORT_SYMBOL_GPL(get_pack_bat_vol);

inline int get_pack_bat_rsoc(int *rsoc)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_pack_bat();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
    BAT_DBG("pack status = %d\n", val.intval);

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}
int get_cover_soc(void)
{
	int soc=-99;
	get_pack_bat_rsoc(&soc);

	return soc;
}
EXPORT_SYMBOL_GPL(get_cover_soc);
void get_soc_p_c(int *pad_soc, int *cover_soc)
{
	get_battery_rsoc(pad_soc);
	get_pack_bat_rsoc(cover_soc);

	BAT_DBG("pad = %d, cover = %d\n", *pad_soc, *cover_soc);	
}
EXPORT_SYMBOL_GPL(get_soc_p_c);

extern void bq25896_set_en_hiz(unsigned int val);
extern void bq25896_set_iinlim(unsigned int val);
extern int cover_usbin(bool turn_on);
extern void cover_aicl(int I_USB_IN);

static void charging_ac_balance(int mode)
{
	int ret;
	BAT_DBG("%s: balance %d\n", __func__, mode);	
	if(irq_cvbus_count == 0){
		enable_irq_wake(ht24lc02au->cvbus_irq);
		irq_cvbus_count = 1;
	}
	switch(mode)
	{
		case 0:
			charger_set_gpio(0,1,1,0);
			smb3xxc_pre_config(true, true);
			g_sdp_cb81_padcharging = 1;
			bq25896_set_en_hiz(0x0);	// 1. Disable PAD Charger HIZ mode
			ret = cover_usbin(true);	// 2. Enable Cover Charger USBIN
			bq25896_set_iinlim(0x6);	// 3. Set PAD Input Current = 300 mA
			cover_aicl(700);		// 4. Set Cover Charger IUSB_IN = 700mA
			break;
		case 1:
			charger_set_gpio(0,0,1,0);
			g_sdp_cb81_padcharging = 1;
			bq25896_set_en_hiz(0x0);	// 1. Disable PAD Charger HIZ mode
			ret = cover_usbin(false);	// 2. Suspend Cover Charger USBIN
			bq25896_set_iinlim(0x12);	// 3. Set PAD Input Current = 1000 mA 
			break;
		case 2:
			charger_set_gpio(0,1,1,0);
			smb3xxc_pre_config(true, true);
			g_sdp_cb81_padcharging = 1;
			bq25896_set_en_hiz(0x0);	// 1. Disable PAD Charger HIZ mode
			ret = cover_usbin(true);	// 2. Enable Cover Charger USBIN
			bq25896_set_iinlim(0x8);	// 3. Set PAD Input Current = 500 mA
			cover_aicl(500);		// 4. Set Cover Charger IUSB_IN = 500mA
			break;
	}
}

static void charging_sdp_balance(int mode)
{
	int ret;
	if(BMT_status.charger_type == STANDARD_HOST)
		BAT_DBG("[%s]: balance %d\n", __func__, mode);
	else
		BAT_DBG("[charging_cdp_balance]: balance %d\n", mode);

	switch(mode)
	{
		case 0:
			g_Flag2 = 1;
			g_sdp_cb81_padcharging = 1;
			bq25896_set_en_hiz(0x0);	// 1. Disable PAD Charger HIZ mod
			cover_usbin(false);		// 2. Suspend Cover Charger USBIN
			break;
		case 1:
			g_Flag2 = 0;
			g_sdp_cb81_padcharging = 0;
			bq25896_set_en_hiz(0x1);	// 1. Enable PAD Charger HIZ mode
			ret = cover_usbin(true);	// 2. Enable Cover Charger USBIN
			if (BMT_status.charger_type == STANDARD_HOST)
				cover_aicl(500);	// 3. Set Cover Charger IUSB_IN = 500mA
			else
				cover_aicl(1500);	// 3. Set Cover Charger IUSB_IN = 1500mA
			break;
	}
}

void cover_ac_function_for_alarm(void){
	int c_rsoc = 101;
        int rsoc = 101;
        int status;
	if(BMT_status.old_charging_mode == COVER_AC)
	{
	        printk("[%s]Charging mode : [PAD + COVER + AC]\n",__func__);

	        get_soc_p_c( &rsoc, &c_rsoc);
	        battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &status);
	        if(status == KAL_TRUE) // pad = 100%
	                charging_ac_balance(0);
	        else if (CHARGING_FULL()) // cover = 100%
	                charging_ac_balance(1);
	        else if (rsoc > 30) // pad > 30%
	                charging_ac_balance(2);
	        else
	                charging_ac_balance(1);
	}
	else if (BMT_status.old_charging_mode == COVER && _IS_CB81_())
	{
		printk("[%s]Charging mode : [PAD + CB81]\n",__func__);
		
		get_soc_p_c( &rsoc, &c_rsoc);
		if (c_rsoc<=5 && c_rsoc >= 0){ // cover < 5%
			if(irq_cvbus_count == 1){
				disable_irq_wake(ht24lc02au->cvbus_irq);
				irq_cvbus_count = 0;
			}
			pr_err("Disable ht24lc02au->cvbus_irq\n");
			cover_otg(0);
			g_cb81_int = 0;
		}
		else if ( rsoc >= 70 ) // pad >= 70%
		{
			if(g_Flag1) //cover to pad
			{
				if(rsoc >= 90)
				{
					g_Flag1=0;
					if(irq_cvbus_count == 1){
						disable_irq_wake(ht24lc02au->cvbus_irq);
						irq_cvbus_count = 0;
					}
					pr_err("Disable ht24lc02au->cvbus_irq\n");
					cover_otg(0);
					g_Flag4=1;
				}
			}
			else // cover not to pad
			{
				if(g_Flag4 == 0 && rsoc < 90)
				{
					cover_otg_current(900);
					bq25896_set_iinlim(0x10);	// 3. Set Input Current Limit = 900 mA
					cover_otg(1);
					g_Flag1=1;
					g_Flag4=1;
				}
				else
				{
					g_Flag1=0;
					if(irq_cvbus_count == 1){
						disable_irq_wake(ht24lc02au->cvbus_irq);
						irq_cvbus_count = 0;
					}
					pr_err("Disable ht24lc02au->cvbus_irq\n");
					cover_otg(0);
					g_Flag4=1;
				}
			}
		}
		else // cover not to pad
		{
			cover_otg_current(900);
			bq25896_set_iinlim(0x10);	// 3. Set Input Current Limit = 900 mA
			cover_otg(1);
			g_Flag1=1;
			g_Flag4=1;
		}
	}
}
EXPORT_SYMBOL_GPL(cover_ac_function_for_alarm);

static void cover_ac_function(struct work_struct *work)
{
	int c_rsoc = 101;
	int rsoc = 101;
	int status;
	printk("Charging mode : [PAD + COVER + AC]\n");
	BAT_DBG("%s\n", __func__);

	get_soc_p_c( &rsoc, &c_rsoc);
	if(_IS_CA81_()){
		if (c_rsoc == 0)
			cover_button(false);
		else if(c_rsoc == 1)
			cover_button(true);
	}
	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &status);
	if(status == KAL_TRUE) // pad = 100%
		charging_ac_balance(0);	
	else if (CHARGING_FULL()) // cover = 100%
		charging_ac_balance(1);
	else if (rsoc > 30) // pad > 30%
		charging_ac_balance(2);
	else
		charging_ac_balance(1);
	BAT_thread();
	back_to_a();
}

static void cover_sdp_function(struct work_struct *work)
{
	int c_rsoc = 101;
	int rsoc = 101;
	if (BMT_status.charger_type == STANDARD_HOST)
		BAT_DBG("[%s]\n", __func__);
	else
		BAT_DBG("cover_cdp_function\n");

	get_soc_p_c( &rsoc, &c_rsoc);

	if(_IS_CA81_()){
		if (c_rsoc == 0)
			cover_button(false);
		else if(c_rsoc == 1)
			cover_button(true);
	}
	if (rsoc >= 70)
	{
		if(g_Flag2 == 1)		
		{
			if(rsoc >= 90)
			{
				if(CHARGING_FULL())
					charging_sdp_balance(0);
				else
					charging_sdp_balance(1);
			}			
		}
		else
		{
			if(CHARGING_FULL())
				charging_sdp_balance(0);
			else
				charging_sdp_balance(1);
		}
	}
	else
	{
		if(g_Flag2 == 0)
			charging_sdp_balance(0);
		else
		{
			if(rsoc >= 90)
			{
				if(CHARGING_FULL())
					charging_sdp_balance(0);
				else
					charging_sdp_balance(1);
			}
		}
	
	}
	BAT_thread();
	back_to_c();
}

static void cover_function(struct work_struct *work)
{
	int c_rsoc = 101;
	int rsoc = 101;
	
	BAT_DBG("[%s]\n", __func__);

	get_soc_p_c( &rsoc, &c_rsoc);
	if(_IS_CA81_()){
		if (c_rsoc == 0)
			cover_button(false);
		else if(c_rsoc == 1)
			cover_button(true);
		back_to_b();
		return;
	}
	if (c_rsoc<=5 && c_rsoc >= 0){ // cover < 5%
		if(irq_cvbus_count == 1){
                        disable_irq_wake(ht24lc02au->cvbus_irq);
			irq_cvbus_count = 0;
		}
		pr_err("Disable ht24lc02au->cvbus_irq\n");
		cover_otg(0);
		g_cb81_int = 0;
	}
	else if ( rsoc >= 70 ) // pad >= 70% 
	{
		if(g_Flag1) //cover to pad
		{
			if(rsoc >= 90)
			{
				g_Flag1=0;
				if(irq_cvbus_count == 1){
	                                disable_irq_wake(ht24lc02au->cvbus_irq);
                                	irq_cvbus_count = 0;
				}
				pr_err("Disable ht24lc02au->cvbus_irq\n");
				cover_otg(0);
				g_Flag4=1;
			}
		}
		else // cover not to pad 
		{
			if(g_Flag4 == 0 && rsoc < 90)
			{
				cover_otg_current(900);
				bq25896_set_iinlim(0x10); 	// 3. Set Input Current Limit = 900 mA
				cover_otg(1);
				g_Flag1=1;
				g_Flag4=1;
			}	
			else
			{
				g_Flag1=0;
				if(irq_cvbus_count == 1){
                	                disable_irq_wake(ht24lc02au->cvbus_irq);
        	                        irq_cvbus_count = 0;
	                        }
				pr_err("Disable ht24lc02au->cvbus_irq\n");
                                cover_otg(0);
                                g_Flag4=1;
			}
		}
	}
	else  // pad < 70%
	{
		if(g_Flag1)
		{
			if(rsoc >= 90) // pad >90%
			{
				g_Flag1=0;
				if(irq_cvbus_count == 1){
                	                disable_irq_wake(ht24lc02au->cvbus_irq);
        	                        irq_cvbus_count = 0;
	                        }
				pr_err("Disable ht24lc02au->cvbus_irq\n");
				cover_otg(0);
				g_Flag4=1;
			}
		} 
		else // cover not to pad
		{
			cover_otg_current(900);
			bq25896_set_iinlim(0x10); 	// 3. Set Input Current Limit = 900 mA
			cover_otg(1);
			g_Flag1=1;
			g_Flag4=1;
		}
	}
	back_to_b();
}

extern void otg_workaround(void);
static void cvbus_timer_function(struct work_struct *work)
{
	g_cvbus_in_int = 0;
	g_cinit = 0;
}
static void cvbus_report_function(struct work_struct *work)
{
	unsigned long flags;
	unsigned long delta;
	BAT_DBG("[%s]\n", __func__);

	spin_lock_irqsave(&cvbus_lock, flags);
	delta = ktime_to_ms(ktime_sub(ktime_get(), last_read_time));
	last_read_time = ktime_get();
	spin_unlock_irqrestore(&cvbus_lock, flags);
	BAT_DBG("[%s] delta time = %ld\n", __func__, delta);

	g_cinit += 1;
	if (delta < 80 && !gpio_get_value(ht24lc02au->dcin_vbus_in_det_n)){
		if(g_cinit!=3){
			BAT_DBG("[%s] workaround: plug out usb when using cb81. g_cinit=%d \n", __func__,g_cinit);
			charger_set_gpio(0,0,1,0);
			call_cover_interupt();
		}
		g_cinit = 0;
	}
	queue_delayed_work(eeprom_wq, &ht24lc02au->cvbus_timer_work,msecs_to_jiffies(80));
}

extern int g_projector_on;
static void usb_report_function(struct work_struct *work)
{
	if (VBUS_IN())
		BAT_DBG("%s: cable plug in\n", __func__);
	else
		BAT_DBG("%s: cable plug out\n", __func__);

	if (g_projector_on){
		if(VBUS_IN()){
			BAT_DBG("%s: VBUS = hi, Charging mode: [PAD + AC]\n", __func__);
			BMT_status.charger_type = STANDARD_CHARGER;
			BMT_status.old_charging_mode = PAD;
			do_chrdet_int_task();
			//chr_control_interface( CHARGING_CMD_AC, NULL);
		}
		else{
			BAT_DBG("%s: VBUS = lo, Charging mode: [PAD only]\n", __func__);
			BMT_status.old_charging_mode = AC;
			do_chrdet_int_task();
			//battery_charging_control( CHARGING_CMD_INIT, NULL);
		}
	}
	else{
		if(_IS_CB81_())
		{
			BMT_status.charger_type = CHARGER_UNKNOWN;
			do_chrdet_int_task();
		}
	}
	usb_in_int = 0;
}

int cover_gauge_thread(void *x){
	while(1){
		if(VBUS_IN()){
			BAT_DBG("[%s] Before wait(%d)\n",__func__,g_cover_gauge_init);
			wait_event_interruptible(cover_gauge_thread_wq,(g_cover_gauge_init == 1));
			BAT_DBG("[%s] After wait(%d)\n",__func__,g_cover_gauge_init);
			do_chrdet_int_task();
		}
		break;
	}
	return 0;
}

static void eeprom_report_function(struct work_struct *work)
{
	int charger_type;
	unsigned int delay_time = 5000;
	BAT_DBG("[%s]\n",__func__);

	cancel_delayed_work(&ht24lc02au->cover_sdp_work);
	cancel_delayed_work(&ht24lc02au->cover_ac_work);
	cancel_delayed_work(&ht24lc02au->cover_work);
	
	if (COVER_ATTACHED()){ //hi active
		if(VBUS_IN()){
			chr_control_interface( CHARGING_CMD_COVER_DISABLEOTG, NULL);
			charger_set_gpio(0,1,1,0);
			msleep_interruptible(1000);
		}
		if(_IS_CB81_()) {
			if (!g_call_cover_int || (BMT_status.old_charging_mode == CINIT))
				upi_ug31xx_attach(true);
			if(g_otg_enable){
				msleep_interruptible(5000);
				tbl_charger_otg_vbus(1);
			}
			if(BMT_status.old_charging_mode == PAD ||
					BMT_status.old_charging_mode == COVER_SDP ||
					BMT_status.old_charging_mode == COVER_CDP ||
					(BMT_status.old_charging_mode == COVER_AC && !VBUS_IN())||
					BMT_status.old_charging_mode == COVER_OTG ){
				BAT_DBG("[%s]Charging mode : [PAD + CB81]\n", __func__);
				if(g_call_cover_int)
					delay_time = 0;
				g_cb81_int = 1;
				chr_control_interface( CHARGING_CMD_COVER, NULL);
				queue_delayed_work(cover_wq, &ht24lc02au->cover_work, msecs_to_jiffies(delay_time));
			}
			else if(BMT_status.old_charging_mode == CINIT){
				BAT_DBG("[%s]Charging mode : [INIT] , BMT_status.charger_type= %d\n", __func__, BMT_status.charger_type);
				g_cb81_int = 1;
				g_cinit = 1;
				if(VBUS_IN()){
					if (BMT_status.charger_type == STANDARD_HOST){
						battery_log(BAT_LOG_CRTI,"[%s]Charging mode: [PAD + COVER + SDP] \n", __func__);
						battery_charging_control(CHARGING_CMD_COVER_SDP, NULL);
						queue_delayed_work(cover_wq, &ht24lc02au->cover_sdp_work, msecs_to_jiffies(delay_time));
					}
					else if (BMT_status.charger_type == CHARGING_HOST){
						battery_log(BAT_LOG_CRTI,"[%s]Charging mode: [PAD + COVER + CDP] \n", __func__);
						battery_charging_control(CHARGING_CMD_COVER_CDP, NULL);
						queue_delayed_work(cover_wq, &ht24lc02au->cover_sdp_work, msecs_to_jiffies(delay_time));
					}
					else if ((BMT_status.charger_type == STANDARD_CHARGER) || (BMT_status.charger_type == NONSTANDARD_CHARGER)){
						battery_log(BAT_LOG_CRTI,"[%s]Charging mode: [PAD + COVER + AC] (%d) \n", __func__, BMT_status.charger_type);
						battery_charging_control(CHARGING_CMD_COVER_AC, NULL);
						queue_delayed_work(cover_wq, &ht24lc02au->cover_ac_work, msecs_to_jiffies(delay_time));
					}
					else
					{
						BAT_DBG("[%s]VBUS_IN, Charging mode : [PAD + CB81]\n", __func__);
						chr_control_interface( CHARGING_CMD_COVER, NULL);
						queue_delayed_work(cover_wq, &ht24lc02au->cover_work, msecs_to_jiffies(delay_time));
					}
				}
				else
				{
					BAT_DBG("[%s]Charging mode : [PAD + CB81]\n", __func__);
					chr_control_interface( CHARGING_CMD_COVER, NULL);
					queue_delayed_work(cover_wq, &ht24lc02au->cover_work, msecs_to_jiffies(delay_time));
				}			
			}
			else
			{
				if(g_call_cover_int)
					delay_time = 0;
				charger_type = mt_get_charger_type();
				BAT_DBG("[%s]:charger_type = %d\n",__func__,charger_type);
				
				if (charger_type == 1 )
				{
					BAT_DBG("[%s]BMT_status.old_charging_mode == SDP\n",__func__);
					chr_control_interface( CHARGING_CMD_COVER_SDP, NULL);
					queue_delayed_work(cover_wq, &ht24lc02au->cover_sdp_work, msecs_to_jiffies(delay_time));
				}
				if ( charger_type == 2 )
				{
					BAT_DBG("[%s]BMT_status.old_charging_mode == CDP\n",__func__);
					chr_control_interface( CHARGING_CMD_COVER_CDP, NULL);
					queue_delayed_work(cover_wq, &ht24lc02au->cover_sdp_work, msecs_to_jiffies(delay_time));
				}
				else if (charger_type == 3 || charger_type == 4 )// ac + cover
				{
					BAT_DBG("[%s]BMT_status.old_charging_mode == AC\n",__func__);
					chr_control_interface( CHARGING_CMD_COVER_AC, NULL);
					queue_delayed_work(cover_wq, &ht24lc02au->cover_ac_work, msecs_to_jiffies(delay_time));
				}
				else if (BMT_status.old_charging_mode == OTG)
					tbl_charger_otg_vbus(1);
				else
					BAT_DBG("[%s]there is some thing wrong! cover detect but no match mode.\n",__func__);
			}
			cover_button(false);
		}
		else{
			if (!g_call_cover_int || BMT_status.old_charging_mode == CINIT)
				upi_ug31xx_attach(true);
			if(g_otg_enable){
			tbl_charger_otg_vbus(1);
			}
			else{
				BAT_DBG("[%s]:CA81 notify USB Port1 open\n",__func__);
				BAT_DBG("[%s]Charging mode : [PAD + CA81]\n", __func__);
				charger_type = mt_get_charger_type();
				BAT_DBG("[%s]:charger_type = %d\n",__func__,charger_type);
	
				if (charger_type == 1 || charger_type == 2 ) // usb + cover
				{
					chr_control_interface( CHARGING_CMD_COVER_SDP, NULL);
					queue_delayed_work(eeprom_wq, &ht24lc02au->cover_sdp_work, msecs_to_jiffies(delay_time));
				}
				else if (charger_type == 3 || charger_type == 4 ) // usb + cover
				{
					chr_control_interface( CHARGING_CMD_COVER_AC, NULL);
					queue_delayed_work(eeprom_wq, &ht24lc02au->cover_ac_work, msecs_to_jiffies(delay_time));
				}
				else{
					chr_control_interface( CHARGING_CMD_COVER, NULL);
					queue_delayed_work(eeprom_wq, &ht24lc02au->cover_work, msecs_to_jiffies(delay_time));
				}
				if(_IS_CA81_())
					cover_button(true);
			}
		}
		BAT_DBG("[%s] g_cover_gauge_init = %d \n", __func__,g_cover_gauge_init);
		if(VBUS_IN() && (g_cover_gauge_init == 0)){
			kthread_run(cover_gauge_thread, NULL, "cover_gauge_thread");
		}
	}
	
	else{
		BAT_DBG("[%s] cover out \n", __func__);
		g_cover_gauge_init = 0;
		g_cover_low_bat_gauge_check = 0;
		upi_ug31xx_attach(false);
		cover_button(false);
		if(g_otg_enable){
			BAT_DBG("[%s] otg mode, cover out \n", __func__);
			tbl_charger_otg_vbus(1);
		}
		else
			do_chrdet_int_task();
	}
	g_call_cover_int = 0;
	usb_in_int = 0;
	g_cvbus_in_int = 0;	
}

static void smb345c_charging_toggle_function(struct work_struct *work){
	smb345c_charging_toggle(FLAGS, true);
}

static int ht24lc02au_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct ht24lc02au_eeprom *ht24;
    unsigned long flags;
    int ret = 0;

    BAT_DBG(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);

    ht24 = devm_kzalloc(dev, sizeof(*ht24), GFP_KERNEL);
    if (!ht24)
        return -ENOMEM;

    i2c_set_clientdata(client, ht24);
    ht24->client = client;
    ht24->dev = dev;
    ht24lc02au = ht24;

 /* Parsing eeprom resource from device tree */
    ret = ht24lc02au_parse_dt(ht24lc02au);
    if (ret < 0) {
        dev_err(&client->dev, "%s, Couldn't to parse dt ret = %d\n", __func__, ret);
        goto error;
    }

/* Checking GPIO pin which requested by driver can be used or not */
    ret = gpio_request(COVER_ATTACH_GPIO,"N1_NP_DET");
    if (ret) {
        dev_err(&client->dev, "unable to request gpio %d\n", COVER_ATTACH_GPIO);
        goto error;
    }
    gpio_direction_input(COVER_ATTACH_GPIO);

/* Checking GPIO pin which requested by driver can be used or not */
    ret = gpio_request(ht24lc02au->n1_vbus_in_det,"N1_VBUS_IN_DET");
    if (ret) {
        dev_err(&client->dev, "unable to request gpio %d\n", ht24lc02au->n1_vbus_in_det);
        goto error;
    }
    gpio_direction_input(ht24lc02au->n1_vbus_in_det);

/* Checking GPIO pin which requested by driver can be used or not */
    ret = gpio_request(ht24lc02au->dcin_vbus_in_det_n,"DCIN_VBUS_IN_DET_N");
    if (ret) {
        dev_err(&client->dev, "unable to request gpio %d\n", ht24lc02au->dcin_vbus_in_det_n);
        goto error;
    }
    gpio_direction_input(ht24lc02au->dcin_vbus_in_det_n);

 /* Initialize workqueue and assign lid_report_function to work */
    eeprom_wq = create_workqueue("eeprom_wq");
    //eeprom_wq = create_singlethread_workqueue("eeprom_wq");
    cover_wq = create_workqueue("cover_wq");
    cvbus_wq = create_workqueue("cvbus_wq");
    smb345c_charging_toggle_wq = create_workqueue("smb345c_charging_toggle_wq");
    INIT_DEFERRABLE_WORK(&ht24lc02au->eeprom_work, eeprom_report_function);
    INIT_DEFERRABLE_WORK(&ht24lc02au->eeprom_work_usb, usb_report_function);
    INIT_DEFERRABLE_WORK(&ht24lc02au->cover_work, cover_function);
    INIT_DEFERRABLE_WORK(&ht24lc02au->cover_ac_work, cover_ac_function);
    INIT_DEFERRABLE_WORK(&ht24lc02au->cover_sdp_work, cover_sdp_function);
    INIT_DEFERRABLE_WORK(&ht24lc02au->cvbus_work, cvbus_report_function);
    INIT_DEFERRABLE_WORK(&ht24lc02au->cvbus_timer_work, cvbus_timer_function);
    INIT_DEFERRABLE_WORK(&ht24lc02au->smb345c_charging_toggle_work,smb345c_charging_toggle_function);

    ret = set_irq_eeprom(ht24lc02au); 
    if (ret < 0){
        dev_err(&client->dev,"fail to set irq\n");
        gpio_free(COVER_ATTACH_GPIO);
        goto error;
    }
    ht24lc02au_dump_registers(NULL);

#if 0
    if (COVER_ATTACHED_UPI()) {
    if (!ht24lc02au_write_all_registers(NULL, 0xCC))
        ht24lc02au_dump_registers(NULL);
    }
#endif

    ht24->dentry = debugfs_create_file("eeprom", S_IRUGO, NULL, ht24,
                      &ht24lc02au_debugfs_fops);
    ret = init_asus_proc_toggle();
    ret = sysfs_create_group(&client->dev.kobj, &dev_attr_grp);
    BAT_DBG(" %s: CA81: %s\n", __func__, _IS_CA81_() ? "Yes" : "No");
    BAT_DBG(" %s: CB81: %s\n", __func__, _IS_CB81_() ? "Yes" : "No");
    BAT_DBG(" ++++++++++++++++ %s done ++++++++++++++++\n", __func__);
    g_cvbus_in_int = 0;

    spin_lock_irqsave(&cvbus_lock, flags);
    last_read_time = ktime_get();
    spin_unlock_irqrestore(&cvbus_lock, flags);

    if (IS_FAIL_CA81())
        BAT_DBG(" **************************************************************** fail CA81(No Tin Plate)\n");

error:
    return ret;
}

static int ht24lc02au_remove(struct i2c_client *client)
{
    //struct ht24lc02au_eeprom *ht24 = i2c_get_clientdata(client);

    return 0;
}

static void ht24lc02au_shutdown(struct i2c_client *client)
{
    dev_info(&client->dev, "%s\n", __func__);

}

#ifdef CONFIG_PM
static int ht24lc02au_prepare(struct device *dev)
{
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    dev_info(&ht24->client->dev, "ht24lc02au suspend\n");
    return 0;
}

static void ht24lc02au_complete(struct device *dev)
{
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    dev_info(&ht24->client->dev, "ht24lc02au resume\n");

}
#else
#define ht24lc02au_prepare NULL
#define ht24lc02au_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int ht24lc02au_runtime_suspend(struct device *dev)
{
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int ht24lc02au_runtime_resume(struct device *dev)
{
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int ht24lc02au_runtime_idle(struct device *dev)
{

    dev_info(dev, "%s called\n", __func__);
    return 0;
}
#else
#define ht24lc02au_runtime_suspend    NULL
#define ht24lc02au_runtime_resume    NULL
#define ht24lc02au_runtime_idle    NULL
#endif

static const struct of_device_id ht24lc02au_match[] = {
    { .compatible = "holtek,ht24lc02au" },
    { },
};

static const struct i2c_device_id ht24lc02au_id[] = {
    {HT24LC02AU_DEV_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, ht24lc02au_id);

static const struct dev_pm_ops ht24lc02au_pm_ops = {
    .prepare        = ht24lc02au_prepare,
    .complete        = ht24lc02au_complete,
    .runtime_suspend    = ht24lc02au_runtime_suspend,
    .runtime_resume        = ht24lc02au_runtime_resume,
    .runtime_idle        = ht24lc02au_runtime_idle,
};

static struct i2c_driver ht24lc02au_driver = {
    .driver = {
        .name    = HT24LC02AU_DEV_NAME,
        .owner    = THIS_MODULE,
        .pm    = &ht24lc02au_pm_ops,
        .of_match_table = of_match_ptr(ht24lc02au_match),
    },
    .probe        = ht24lc02au_probe,
    .remove        = ht24lc02au_remove,
    .shutdown    = ht24lc02au_shutdown,
    .id_table    = ht24lc02au_id,
};

static int __init ht24lc02au_init(void)
{
    BAT_DBG(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);
    return i2c_add_driver(&ht24lc02au_driver);
}
module_init(ht24lc02au_init);

static void __exit ht24lc02au_exit(void)
{
    i2c_del_driver(&ht24lc02au_driver);
}
module_exit(ht24lc02au_exit);

module_param(cover_type, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(COVER_TYPE, "Cover Type Id");

MODULE_AUTHOR("Chris Chang <chris1_chang@asus.com>");
MODULE_AUTHOR("Shilun Huang <shilun_huang@asus.com>");
MODULE_DESCRIPTION("HOLTEK EEPROM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:ht24lc02au");
