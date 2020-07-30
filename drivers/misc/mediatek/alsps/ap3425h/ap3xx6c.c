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
 * Filename: ap3xx6.c
 *
 * Summary:
 *	ap3xx6 sensor dirver.
 *
 * Modification History:
 * Date	By		Summary
 * -------- -------- -------------------------------------------------------
 * 05/11/12 YC		Original Creation (Test version:1.0)
 * 05/30/12 YC		Modify AP3216C_check_and_clear_intr return value and exchange
 *					AP3216C_get_ps_value return value to meet our spec.
 * 05/30/12 YC		Correct shift number in AP3216C_read_ps.
 * 05/30/12 YC		Correct ps data formula.
 * 05/31/12 YC		1. Change the reg in clear int function from low byte to high byte
 *						and modify the return value.
 *					2. Modify the eint_work function to filter als int.
 * 06/04/12 YC		Add PS high/low threshold instead of using the same value.
 * 07/12/12 YC		Add wakelock to prevent entering suspend when early suspending.
 *
 *
 *29/5/14	ansun modify code to mt6582 add ap3425
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/wakelock.h>

#include <mt-plat/mt_gpio.h>

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <linux/io.h>
#include <cust_alsps.h>
#include "ap3xx6c.h"
#include <alsps.h>
#include <batch.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>


/*-------------------------------flag---------------------------------------------*/
#define APS_TAG					"alp.D :"
#define APS_FUN(f)			pr_err(APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)	printk(KERN_ERR APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)	pr_err(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)	pr_err(APS_TAG fmt, ##args)

//#define AP3425H_NUM_CACHABLE_REGS	12
//static u8 *ap3425h_reg[AP3425H_NUM_CACHABLE_REGS]={0x01, 0x02, 0x06, 0x0C, 0x0D, 0x10, 0x01, 0x14, 0x1A, 0x1B, 0x1C, 0x1D};
//static char *ap3425h_reg_name[AP3425H_NUM_CACHABLE_REGS]={"SYS_CONFIG","INTERRUPT_FLAG","INT_CONTROL","WAITING","ALS_DATA_L","ALS_DATA_H","GAIN","Persistence","ALS_THRES_LOW_L","ALS_THRES_LOW_H","ALS_THRES_HIGH_L","ALS_THRES_HIGH_H"};

/*---------------------------user define-------------------------------------------------*/

#define DELAYED_WORK 0	// default -->0 ; test -->1
#define ap3425
//#ifdef ap3425
//#define DI_AUTO_CAL
//#ifdef DI_AUTO_CAL
//		#define DI_PS_CAL_THR 255
//#endif
	#define AP3XX6_DEV_NAME	"ap3425h"
//#else
//	#define AP3XX6_DEV_NAME	"AP3216"
//#endif

#define AP3425H_INPUT_NAME		"ap3425h_input"

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ap3xx6_i2c_id[] = {{AP3XX6_DEV_NAME, 0}, {} };
static int of_get_ap3425h_platform_data(struct device *dev);
static struct kobject *android_light_kobj = NULL;	// Sys kobject variable

// Wait 1ms for i2x retry
#define I2C_RETRY_DELAY()		usleep_range(1000, 2000)
// Wait 2ms for calibration ready
#define WAIT_CAL_READY()		usleep_range(2000, 2500)
// >3ms wait device ready
#define WAIT_DEVICE_READY()	usleep_range(3000, 5000)
// >5ms for device reset 
#define RESET_DELAY()			usleep_range(5000, 10000)
// Wait 10ms for self test done
#define SELF_TEST_DELAY()		usleep_range(10000, 15000)
// Wait 100ms for calibration test 
#define CALI_TEST_DELAY()		usleep_range(100000, 150000)


/*----------------------------------------------------------------------------*/
//configuration
#define ALS_CALI_PATH "/persist/als_cali.ini"
//#define ALS_CALI_PATH "/data/misc/sensor/als_cali.ini"	// for test

// For delay calibration
#define AP3425H_DELAY_CALITIME		20000
#define AP3425H_DELAY_RETRYCALITIME	3000
#define AP3425H_DELAY_CALIBRATION		1

#define AP3425H_CALI_SAMPLE_TIME		4
#define AP3425H_CALI_FILE_LENGTH		16

/*----------------------------------------------------------------------------*/
static int ap3xx6_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ap3xx6_i2c_remove(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static int ap3xx6_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ap3xx6_i2c_resume(struct i2c_client *client);
#ifdef SENSOR_PS
static int ap3xx6_ps_get_data(int *value, int *status);
#endif
static int ap3xx6_als_get_data(int *value, int *status);
//static int ap3425h_input_init(void);
/*----------------------------------------------------------------------------*/

static struct wake_lock chrg_lock;

/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS	= 1,
	CMC_BIT_PS	= 2,

} CMC_BIT;
typedef enum {
	TRACE_DEBUG = 0x1,
	TRACE_ALS = 0x2,
	TRACE_PS = 0x4,
} TRACE_BIT;
/*----------------------------------------------------------------------------*/
struct ap3xx6_i2c_addr {	/*define a series of i2c slave address*/
	u8 write_addr;
	u8 ps_thd;	/*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct ap3xx6_priv {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct input_dev *input_dev;
#if DELAYED_WORK
	struct delayed_work eint_work;
#else
	struct work_struct eint_work;
#endif
	struct mutex lock;
	/*i2c address group*/
	struct ap3xx6_i2c_addr addr;
	struct device_node *irq_node;
#ifdef AP3425H_DELAY_CALIBRATION
	struct delayed_work delayworkcalibration;
	atomic_t delaycalibration;
#endif
	int irq;
	/*misc*/
	u16		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on;		/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t	trace;

	/*data*/
	u16		als;
	u16		ps;
	u8		_align;
	u16		als_level_num;
	u16		als_value_num;
	u32		als_level[C_CUST_ALS_LEVEL-1];
	u32		als_value[C_CUST_ALS_LEVEL];
	atomic_t delay;

	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val;		/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_h;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_l;	/*the cmd value can't be read, stored in ram*/

	ulong		enable;			/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/

	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
#endif
};

struct als_cali {
	int als_cali_base;
	int als_cali_adc;
	int cali_state;
	int cal_threshold_low;
	int cal_threshold_high;
	int do_calibrating;
};

static struct als_cali ap3425h_cali;

static struct i2c_client *ap3xx6_i2c_client;
static struct ap3xx6_priv *ap3xx6_obj;


/* Add  for Interrupt mode */
#define AP3425H_TABLE_LENGTH		20
static uint16_t defalut_table[AP3425H_TABLE_LENGTH] = {0x32, 0x64, 0xC8, 0x12C, 0x190, 0x1F4, 0x28A, 0x320, 0x3E8, 0x5DC, 0x7D0, 0xBB8, 0xFA0, 0x1388, 0x1B58, 0x2710, 0x30D4, 0x3A98, 0x445C, 0x4E20};
//static uint16_t cali_table[AP3425H_TABLE_LENGTH] = {0x0C, 0x19, 0x32, 0x4B, 0x64, 0x7D, 0xA2, 0xC8, 0xFA, 0x177, 0x1F4, 0x2EE, 0x3E8, 0x4E2, 0x6D6, 0x9C4, 0xC35, 0xEA6, 0x1117, 0x1388};
static uint16_t cali_table[AP3425H_TABLE_LENGTH] = {0x32, 0x64, 0xC8, 0x12C, 0x190, 0x1F4, 0x28A, 0x320, 0x3E8, 0x5DC, 0x7D0, 0xBB8, 0xFA0, 0x1388, 0x1B58, 0x2710, 0x30D4, 0x3A98, 0x445C, 0x4E20};

#define AP3425H_GOLDEN_VALUE_Z380M	150

#define LIGHT_STABLE_LIMIT		2	// 20% ; use 10+2/10=1.2 to do it
#define LIGHT_STABLE_COUNTER	10
static int g_temp_als_lux=-1;
static int g_als_lux_up_value=0;
static int g_als_lux_up_count=0;
static int g_als_lux_down_value=0;
static int g_als_lux_down_count=0;

/*----------------------------------------------------------------------------*/
static const struct of_device_id als_of_match[] = {
	{.compatible = "mediatek,ap3425h"},
	{},
};

static struct i2c_driver ap3xx6_i2c_driver = {
	.probe		= ap3xx6_i2c_probe,
	.remove	= ap3xx6_i2c_remove,
	.suspend	= ap3xx6_i2c_suspend,
	.resume	= ap3xx6_i2c_resume,
	.id_table	= ap3xx6_i2c_id,
	.driver = {
/* .owner			= THIS_MODULE, */
		.name			= AP3XX6_DEV_NAME,
		.of_match_table = als_of_match,
	},
};

static int ap3xx6_local_init(void);
static int ap3xx6_remove(void);

static int ap3xx6_init_flag = -1; // 0<==>OK -1 <==> fail 
static int ap3xx6_openlog = 0; // 0<==>close ;  1<==>open
unsigned int alsps_int_gpio_number = 0;
static unsigned int alsps_irq;

static struct alsps_init_info ap3xx6_init_info = {
	.name = AP3XX6_DEV_NAME,
	.init = ap3xx6_local_init,
	.uninit = ap3xx6_remove,
};

static struct alsps_hw cust_alsps_hw = {
  .i2c_num    = 2,
  .polling_mode_ps =0,
  .polling_mode_als =0,
//  .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
//  .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
  .i2c_addr   = {0x3C, 0x38, 0x3A, 0x00}, //
  .als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
  .als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
  .ps_threshold_high = 300,
  .ps_threshold_low = 100,
};

//static int ap3xx6_get_als_value(struct ap3xx6_priv *obj, u16 als);

static int ap3425_write_to_califile(int base,int calilux);
#ifdef AP3425H_DELAY_CALIBRATION
static int ap3425_read_from_califile(void);
#endif
mm_segment_t old_als_fs;


#ifdef AP3425H_DELAY_CALIBRATION
static void ap3425h_delay_calibration_func(struct work_struct *work)
{
//	unsigned long delay = msecs_to_jiffies(AP3425H_DELAY_RETRYCALITIME);
	int status=0;
	printk("alp.D : ap3425h_delay_calibration_func !!\n");
	status = ap3425_read_from_califile();
	if(status<0){
		printk("alp.D : read calibration fail , need to Retry !!\n");
//		schedule_delayed_work(&light_sensor_data->delayworkcalibration, delay);
		return ;
	}		
}
#endif

static int ap3xx6_read_reg(struct i2c_client *client,
		char reg, u8 mask, u8 shift)
{
	int ret = 0;
	char tmp[1];
	tmp[0] = reg;
	mutex_lock(&ap3xx6_obj->lock);

	ret = i2c_master_send(client, tmp, 0x01);
	if (ret <= 0) {
		APS_ERR("ap3xx6_read_reg 1 ret=%x\n", ret);
		goto EXIT_ERR;
	}
	ret = i2c_master_recv(client, tmp, 0x01);
	if (ret <= 0) {
		APS_ERR("ap3xx6_read_reg 2 ret=%d\n", ret);
		goto EXIT_ERR;
	}

	mutex_unlock(&ap3xx6_obj->lock);
	return (tmp[0] & mask) >> shift;

EXIT_ERR:
		APS_ERR("ap3xx6_read_reg fail\n");
	mutex_unlock(&ap3xx6_obj->lock);
		return ret;
}

static int ap3xx6_write_reg(struct i2c_client *client,
		char reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0x00;
	char tmp[2];

	mutex_lock(&ap3xx6_obj->lock);

	tmp[0] = reg;
	tmp[1] = val;
	ret = i2c_master_send(client, tmp, 0x02);
	if (ret <= 0) {
		APS_ERR("ap3xx6_write_reg ret=%d\n", ret);
		goto EXIT_ERR;
	}

	mutex_unlock(&ap3xx6_obj->lock);
	return ret;

EXIT_ERR:
		APS_ERR("ap3xx6_write_reg fail\n");
	mutex_unlock(&ap3xx6_obj->lock);
		return ret;
}
/*----------------------------------------------------------------------------*/
int ap3xx6_get_addr(struct alsps_hw *hw, struct ap3xx6_i2c_addr *addr)
{
	if (!hw || !addr) {
		return -EFAULT;
	}
	addr->write_addr = hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
//static void ap3xx6_power(struct alsps_hw *hw, unsigned int on)
//{
//	static unsigned int power_on;
//
//#ifdef __USE_LINUX_REGULATOR_FRAMEWORK__
//
//#else
//	if (hw->power_id != POWER_NONE_MACRO) {
//		if (power_on == on) {
//			APS_LOG("ignore power control: %d\n", on);
//		} else if (on) {
//			if (!hwPowerOn(hw->power_id, hw->power_vol, AP3XX6_DEV_NAME)) {
//				APS_ERR("power on fails!!\n");
//			}
//		} else{
//			if (!hwPowerDown(hw->power_id, AP3XX6_DEV_NAME)) {
//				APS_ERR("power off fail!!\n");
//			}
//		}
//	}
//#endif
//	power_on = on;
//}
/*----------------------------------------------------------------------------*/
static int ap3xx6_enable_als(struct i2c_client *client, int enable)
{
//		struct ap3xx6_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];
		int res = 0;
		u8 buffer[1];
		int reg_value[1];

		if (client == NULL) {
			APS_DBG("CLIENT CANN'T EQUAL NULL\n");
			return -1;
		}

		buffer[0] = AP3xx6_ENABLE;
		reg_value[0] = ap3xx6_read_reg(client, buffer[0], 0xFF, 0x00);
		if (res < 0) {
			goto EXIT_ERR;
		}

		if (enable) {
			databuf[0] = AP3xx6_ENABLE;
			databuf[1] = reg_value[0] | 0x01;
			res = ap3xx6_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
			if (res <= 0) {
				goto EXIT_ERR;
			}
			atomic_set(&ap3xx6_obj->als_deb_on, 1);
			atomic_set(&ap3xx6_obj->als_deb_end, jiffies+atomic_read(&ap3xx6_obj->als_debounce)/(1000/HZ));
			APS_DBG("ap3xx6_ ALS enable\n");
#if DELAYED_WORK
			schedule_delayed_work(&ap3xx6_obj->eint_work, 1100*HZ/1000);
#endif
		} else{
			res = ap3xx6_write_reg(ap3xx6_obj->client, AP3425_REG_SYS_CONF, AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, AP3425_SYS_DEV_RESET);
			databuf[0] = AP3xx6_ENABLE;
			databuf[1] = reg_value[0] & 0xFE;
			res = ap3xx6_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
			if (res <= 0) {
				goto EXIT_ERR;
			}
			atomic_set(&ap3xx6_obj->als_deb_on, 0);
#if DELAYED_WORK
			cancel_delayed_work_sync(&ap3xx6_obj->eint_work);
#endif
			APS_DBG("ap3xx6_ ALS disable\n");
		}
		return 0;

EXIT_ERR:
		APS_ERR("ap3xx6__enable_als fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
#ifdef SENSOR_PS
static int ap3xx6_enable_ps(struct i2c_client *client, int enable)
{
//	struct ap3xx6_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}


	buffer[0] = AP3xx6_ENABLE;
	reg_value[0] = ap3xx6_read_reg(client, buffer[0], 0xFF, 0x00);
	if (res < 0) {
		goto EXIT_ERR;
	}

	if (enable) {
		databuf[0] = AP3xx6_ENABLE;
		databuf[1] = reg_value[0] | 0x02;
		res = ap3xx6_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		if (0 == ap3xx6_obj->hw->polling_mode_ps) {
			enable_irq(alsps_irq);
		} else{
			wake_lock(&chrg_lock);
			atomic_set(&ap3xx6_obj->ps_deb_on, 1);
			atomic_set(&ap3xx6_obj->ps_deb_end, jiffies+atomic_read(&ap3xx6_obj->ps_debounce)/(1000/HZ));
		}

	#if DELAYED_WORK
		schedule_delayed_work(&ap3xx6_obj->eint_work, 110*HZ/1000);
	#endif
		APS_DBG("ap3xx6_ PS enable\n");
	} else{
		databuf[0] = AP3xx6_ENABLE;
		databuf[1] = reg_value[0] & 0xfd;
		res = ap3xx6_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		atomic_set(&ap3xx6_obj->ps_deb_on, 0);
		APS_DBG("ap3xx6_ PS disable\n");

		if (0 == ap3xx6_obj->hw->polling_mode_ps) {
	#if (!DELAYED_WORK)
			cancel_delayed_work_sync(&ap3xx6_obj->eint_work);
	#endif
			disable_irq_nosync(alsps_irq);
		} else{
			wake_unlock(&chrg_lock);
		}
	}
	return 0;

EXIT_ERR:
	APS_ERR("ap3xx6__enable_ps fail\n");
	return res;
}
#endif

static int ap3425_get_intstat(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3425_REG_SYS_INTSTATUS);
    val &= AP3425_REG_SYS_INT_MASK;

    return val >> AP3425_REG_SYS_INT_SHIFT;
}

/*----------------------------------------------------------------------------*/
#if 0
static int ap3xx6_check_and_clear_intr(struct i2c_client *client)
{
	int res;
	u8 ints[1];

	/* Get Int status */
	ints[0] = ap3xx6_read_reg(client, AP3xx6_INT_STATUS, 0xFF, 0x00);
	if (ints[0] < 0) {
		goto EXIT_ERR;
	}

	/* Clear ALS int flag */
	res = ap3xx6_read_reg(client, AP3xx6_ADATA_H, 0xFF, 0x00);
	if (res < 0) {
		goto EXIT_ERR;
	}

	/* Clear PS int flag */
	res = ap3xx6_read_reg(client, AP3xx6_PDATA_H, 0xFF, 0x00);
	if (res < 0) {
		goto EXIT_ERR;
	}

	return ints[0];

EXIT_ERR:
	APS_ERR("ap3xx6_check_and_clear_intr fail\n");
	return -1;
}
#endif

//for als
static int ap3xx6_set_ALSGain(struct i2c_client *client, int val)
{
	int re_val, err;

#ifdef ap3425
	/*val=0x00, 0x10, 0x20, 0x30*/
//	re_val = val << 4;
	re_val = 0x00;
	err = ap3xx6_write_reg(client, 0x10, 0xFF, 0x00, re_val);
#else
/*val=0x00~0xF*/
	re_val = ap3xx6_read_reg(client, 0x20, 0xFF, 0x00);
	re_val = (re_val&0xF)|(val << 4);
#endif

	return err;
}

/*
static int ap3425_get_ahthres(struct i2c_client *client)
{
	int lsb, msb, value;
	lsb = ap3xx6_read_reg(client, 0x1C, 0xFF, 0x00);
	msb = ap3xx6_read_reg(client, 0x1D, 0xFF, 0x00);
	value = ((msb << 8) | lsb);
	if(ap3xx6_openlog) APS_LOG("ap3425_get_ahthres  value=%d\n", value);
	return value;
}
static int ap3425_get_althres(struct i2c_client *client)
{
	int lsb, msb, value;
	lsb = ap3xx6_read_reg(client, 0x1A, 0xFF, 0x00);
	msb = ap3xx6_read_reg(client, 0x1B, 0xFF, 0x00);
	value = ((msb << 8) | lsb);
	if(ap3xx6_openlog) APS_LOG("ap3425_get_althres  value=%d\n", value);
	return value;
}
*/
// ALS low threshold
static int ap3425_set_althres(struct i2c_client *client, int val)
{
//	int lsb, msb;
	int ret=0;
	u8 databuf_L, databuf_H;
	databuf_H = val >> 8;
	databuf_L = val & 0xFF;

//	ret = ap3xx6_write_reg(client, 0x1A, 0xFF, 0x00, lsb);
//	if (ret)
//		return ret;
//	ret = ap3xx6_write_reg(client, 0x1B, 0xFF, 0x00, msb);

	if(ap3xx6_openlog) APS_LOG(" set L threshold val = %d !!\n",val);
	ret += ap3xx6_write_reg(ap3xx6_obj->client, 0x1A, 0xFF, 0x00, databuf_L);
	ret += ap3xx6_write_reg(ap3xx6_obj->client, 0x1B, 0xFF, 0x00, databuf_H);
	if(ap3xx6_openlog) APS_LOG(" set L threshold ret = %d !!\n",ret);
	if(ap3xx6_openlog) APS_LOG("ap3425_set_althres  msb=%d ; lsb=%d\n",databuf_H ,databuf_L);
	return ret;
}

// ALS high threshold
static int ap3425_set_ahthres(struct i2c_client *client, int val)
{
//	int lsb, msb;
	int ret=0;
	u8 databuf_L, databuf_H;
	databuf_H = val >> 8;
	databuf_L = val & 0xFF;

//	ret = ap3xx6_write_reg(client, 0x1C, 0xFF, 0x00, lsb);
//	if (ret)
//		return ret;
//	ret = ap3xx6_write_reg(client, 0x1D, 0xFF, 0x00, msb);
	if(ap3xx6_openlog) APS_LOG(" set H threshold val = %d !!\n",val);
	ret += ap3xx6_write_reg(ap3xx6_obj->client, 0x1C, 0xFF, 0x00, databuf_L);
	ret += ap3xx6_write_reg(ap3xx6_obj->client, 0x1D, 0xFF, 0x00, databuf_H);
	if(ap3xx6_openlog) APS_LOG(" set H threshold ret = %d !!\n",ret);
	if(ap3xx6_openlog) APS_LOG("ap3425_set_ahthres  msb=%d ; lsb=%d\n",databuf_H ,databuf_L);
	return ret;
}


#ifdef SENSOR_PS
// for ps 
static int ap3xx6_set_plthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

#ifdef ap3425
	msb = val >> 8;
	lsb = val & 0xFF;
#else
	msb = val >> 2;
	lsb = val & 0x03;
#endif

	err = ap3xx6_write_reg(client, 0x2A,
			0xFF, 0x00, lsb);
	if (err <= 0)
		return err;

	err = ap3xx6_write_reg(client, 0x2B,
			0xFF, 0x00, msb);

	return err;
}

static int ap3xx6_set_phthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

#ifdef ap3425
	msb = val >> 8;
	lsb = val & 0xFF;
#else
	msb = val >> 2;
	lsb = val & 0x03;
#endif

	err = ap3xx6_write_reg(client, 0x2C,
			0xFF, 0x00, lsb);
	if (err <= 0)
		return err;

	err = ap3xx6_write_reg(client, 0x2D,
			0xFF, 0x00, msb);

	return err;
}

static int ap3xx6_set_pcrosstalk(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	#ifdef ap3425
	msb = val >> 8;
	lsb = val & 0xFF;
	#else
	msb = val >> 1;
	lsb = val & 0x01;
	#endif
	err = ap3xx6_write_reg(client, 0x28,
		0xFF, 0x00, lsb);
	if (err <= 0)
		return err;
	err = ap3xx6_write_reg(client, 0x29,
		0xFF, 0x00, msb);

	return err;
}
#endif

#ifdef SENSOR_PS
static int ap3xx6_set_PSTtime(struct i2c_client *client, int val)
{
	int re_val, err;
	#ifdef ap3425
	/*val=0x00~0x3F*/
	re_val = val&0x3F;
	err = ap3xx6_write_reg(client, 0x25, 0xFF, 0x00, re_val);
	#else
	/*val=0x00~0xF*/
	re_val = ap3xx6_read_reg(client, 0x20, 0xFF, 0x00);
	re_val = (re_val&0xF)|(val << 4);
	#endif

	return err;
}

/*val=0x00~0x03*/
static int ap3xx6_set_PSgain(struct i2c_client *client, int val)
{
	int re_val, err;

	#ifdef ap3425
	re_val = val << 2;
	err = ap3xx6_write_reg(client, 0x20, 0xFF, 0x00, re_val);
	#else
	re_val = ap3xx6_read_reg(client, 0x20, 0xFF, 0x00);
	re_val = (re_val&0xF3)|(val << 2);
	#endif

	return err;
}

static int ap3xx6_set_PSpulse(struct i2c_client *client, int val)
{
	int re_val, err;

	#ifdef ap3425
	re_val = ap3xx6_read_reg(client, 0x21, 0xFF, 0x00);
	re_val = ((re_val&0xFC)|val);
	err = ap3xx6_write_reg(client, 0x21,0xFF, 0x00, re_val);
	#else
	/*val=0x00~0x03*/
	re_val = ap3xx6_read_reg(client, 0x21, 0xFF, 0x00);
	re_val = (re_val&0xCF)|(val<<4);
	err = ap3xx6_write_reg(client, 0x21, 0xFF, 0x00, re_val);
	#endif

	return err;
}

static int ap3xx6_set_meantime(struct i2c_client *client, int val)
{
	int re_val, err;
	/*val=0x00~0x03*/
	re_val = val&0x3;
	err = ap3xx6_write_reg(client, 0x23, 0xFF, 0x00, re_val);

	return err;
}
#endif

/*----------------------------------------------------------------------------*/

//void ap3xx6_eint_func(void)
//{
////	struct ap3xx6_priv *obj = ap3xx6_obj;
//	APS_FUN();
//	if (unlikely(ap3xx6_obj == NULL)) {
//		APS_ERR("%s--%d ap3xx6_obj is NULL!\n", __func__, __LINE__);
//		return;
//	}
//
//	if (atomic_read(&ap3xx6_obj->trace) & TRACE_DEBUG) {
//		if(ap3xx6_openlog) APS_LOG("%s--%d\n", __func__, __LINE__);
//	}
//
//	enable_irq(alsps_irq);
//
////#if DELAYED_WORK
////	schedule_delayed_work(&ap3xx6_obj->eint_work, 0);
////#else
////	schedule_work(&ap3xx6_obj->eint_work);
////#endif
//}

static irqreturn_t ap3xx6_eint_handler(int irq, void *desc)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	if (unlikely(ap3xx6_obj == NULL)) {
		APS_ERR("%s--%d ap3xx6_obj is NULL!\n", __func__, __LINE__);
		return IRQ_HANDLED;
	}

	if (atomic_read(&ap3xx6_obj->trace) & TRACE_DEBUG) {
		APS_LOG("%s--%d\n", __func__, __LINE__);
	}

	disable_irq_nosync(alsps_irq);
//	ap3xx6_eint_func();
	if(ap3xx6_openlog) APS_LOG("%s--%d\n", __func__, __LINE__);
#if DELAYED_WORK
#else
	schedule_work(&ap3xx6_obj->eint_work);
#endif
	return IRQ_HANDLED;
}


/*----------------------------------------------------------------------------*/
/* This function depends the real hw setting, customers should modify it. 2012/5/10 YC. */
int ap3xx6_setup_eint(struct i2c_client *client)
{
	int err = 0;
	gpio_direction_input(alsps_int_gpio_number);
	APS_LOG("ap3xx6 probe alsps_irq = %d\n", alsps_irq);
	err = request_irq(alsps_irq, ap3xx6_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL);
	if (err != 0) {
		APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
		return -EINVAL;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ap3xx6_init_client(struct i2c_client *client)
{
//	struct ap3xx6_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = AP3xx6_ENABLE;
	databuf[1] = 0x00;
	res = ap3xx6_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	ap3425_set_althres(client, 0xFFFF);
	ap3425_set_ahthres(client, 0x00);

#ifdef SENSOR_PS
	ap3xx6_set_plthres(client, atomic_read(&ap3xx6_obj->ps_thd_val_l));
	ap3xx6_set_phthres(client, atomic_read(&ap3xx6_obj->ps_thd_val_h));
#endif

	ap3xx6_set_ALSGain(client, 0x00);

#ifdef SENSOR_PS
	ap3xx6_set_PSTtime(client, 0x00);
	ap3xx6_set_PSgain(client, 0x03);
	ap3xx6_set_PSpulse(client, 0x03);
	ap3xx6_set_meantime(client, 0x02);
	ap3xx6_set_pcrosstalk(client,50); //set crosstalk 100
#endif

#ifndef SENSOR_PS
	// enable ALS interrupt
	databuf[0] = AP3xx6_CONYROL_INT;
	databuf[1] = 0x08;
	res = ap3xx6_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
	if (res <= 0) {
		goto EXIT_ERR;
	}
#endif

#ifdef SENSOR_PS
	// enable ALS & PS interrupt
	databuf[0] = AP3xx6_CONYROL_INT;
	databuf[1] = 0x88;
	res = ap3xx6_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
	if (res <= 0) {
		goto EXIT_ERR;
	}
#endif


	return AP3xx6_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
int ap3xx6_read_als(struct i2c_client *client, u16 *data)
{
	/*struct ap3xx6_priv *obj = i2c_get_clientdata(client);	*/
	u8 als_value_low[1], als_value_high[1];
	int value=0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	/* get ALS adc count */
	als_value_low[0] = ap3xx6_read_reg(client, AP3xx6_ADATA_L, 0xFF, 0x00);
	if (als_value_low[0] < 0) {
		goto EXIT_ERR;
	}

	als_value_high[0] = ap3xx6_read_reg(client, AP3xx6_ADATA_H, 0xFF, 0x00);
	if (als_value_high[0] < 0) {
		goto EXIT_ERR;
	}

	value = als_value_low[0] | (als_value_high[0]<<8);
	if (atomic_read(&ap3xx6_obj->trace) & TRACE_DEBUG){
		if(ap3xx6_openlog) APS_LOG("ap3xx6_read_als  adc=%d\n", value);
	}
	if (value < 0) {
		*data = 0;
		APS_DBG("als_value is invalid!!\n");
		goto EXIT_ERR;
	}

	if(ap3425h_cali.do_calibrating==0){
		*data = (u16) value * (ap3425h_cali.als_cali_base)/ap3425h_cali.als_cali_adc;
		if (atomic_read(&ap3xx6_obj->trace) & TRACE_DEBUG){
			if(ap3xx6_openlog) APS_LOG("calibration  adc=%d\n", *data);
		}
	}else{
		*data = (u16) value ;
		if(ap3xx6_openlog) APS_LOG("orig-adc=%d\n", *data);
	}
//		*data = (*data / 8)*8;
	return 0;

EXIT_ERR:
	APS_ERR("ap3xx6__read_als fail\n");
	return -1;
}
/*----------------------------------------------------------------------------*/

//  need to modify the L-sensor mapping table ; marked by alp 2015.11.19 ++
//static int ap3xx6_get_als_value(struct ap3xx6_priv *obj, u16 als)
//{
//	int idx;
//	int invalid = 0;
//	for (idx = 0; idx < ap3xx6_obj->als_level_num; idx++) {
//		if (als < ap3xx6_obj->hw->als_level[idx]) {
//			break;
//		}
//	}
//
//	if (idx >= ap3xx6_obj->als_value_num) {
//		APS_ERR("exceed range\n");
//		idx = obj->als_value_num - 1;
//	}
//
//	if (1 == atomic_read(&ap3xx6_obj->als_deb_on)) {
//		unsigned long endt = atomic_read(&ap3xx6_obj->als_deb_end);
//		if (time_after(jiffies, endt)) {
//			atomic_set(&ap3xx6_obj->als_deb_on, 0);
//		}
//
//		if (1 == atomic_read(&ap3xx6_obj->als_deb_on)) {
//			invalid = 1;
//		}
//	}
//
//	if (!invalid) {
//		if (atomic_read(&ap3xx6_obj->trace) & TRACE_DEBUG) {
//			APS_DBG("ALS: %05d => %05d\n", als, ap3xx6_obj->hw->als_value[idx]);
//		}
//		return ap3xx6_obj->hw->als_value[idx];
//	} else{
//		APS_ERR("ALS: %05d => %05d (-1)\n", als, ap3xx6_obj->hw->als_value[idx]);
//		return -1;
//	}
//}
//  need to modify the L-sensor mapping table ; marked by alp 2015.11.19 --

/*----------------------------------------------------------------------------*/
int ap3xx6_read_ps(struct i2c_client *client, u16 *data)
{
	/*struct ap3xx6_priv *obj = i2c_get_clientdata(client); */
	u8 ps_value_low[1], ps_value_high[1];

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}
	ps_value_low[0] = ap3xx6_read_reg(client, AP3xx6_PDATA_L, 0xFF, 0x00);
	if (ps_value_low[0] < 0) {
		goto EXIT_ERR;
	}

	ps_value_high[0] = ap3xx6_read_reg(client, AP3xx6_PDATA_H, 0xFF, 0x00);
	if (ps_value_high[0] < 0) {
		goto EXIT_ERR;
	}

#ifdef ap3425
	*data = (ps_value_low[0] & 0xFF) | ((ps_value_high[0] & 0x03) << 8);
#else
	*data = (ps_value_low[0] & 0x0f) | ((ps_value_high[0] & 0x3f) << 4);
#endif

	return 0;

EXIT_ERR:
	APS_ERR("ap3xx6_read_ps fail\n");
	return -1;
}
/*----------------------------------------------------------------------------*/
/*
	for ap3xx6_get_ps_value:
	return 1 = object close,
	return 0 = object far away. 2012/5/10 YC	// exchange 0 and 1 2012/5/30 YC
*/
static int ap3xx6_get_ps_value(struct ap3xx6_priv *obj, u16 ps)
{
	int val=0;
	int invalid = 0;

	if (ps > atomic_read(&ap3xx6_obj->ps_thd_val_h))
		val = 0;	/*close*/
	else if (ps < atomic_read(&ap3xx6_obj->ps_thd_val_l))
		val = 1;	/*far away*/

	if (atomic_read(&ap3xx6_obj->ps_suspend)) {
		invalid = 1;
	} else if (1 == atomic_read(&ap3xx6_obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&ap3xx6_obj->ps_deb_end);
		if (time_after(jiffies, endt)) {
			atomic_set(&ap3xx6_obj->ps_deb_on, 0);
		}

		if (1 == atomic_read(&ap3xx6_obj->ps_deb_on)) {
			invalid = 1;
		}
	}

	if (!invalid) {
		if (atomic_read(&ap3xx6_obj->trace) & TRACE_DEBUG) {
			APS_DBG("PS: %05d => %05d\n", ps, val);
		}
		return val;
	} else{
		return -1;
	}
}

//static int ap3xx6_get_OBJ(struct i2c_client *client)
//{
//
//	u8 ps_value_high[1];
//
//	if (client == NULL) {
//		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
//		return -1;
//	}
//#ifdef ap3425
//	ps_value_high[0] = ap3xx6_read_reg(client, AP3xx6_INT_STATUS, 0xFF, 0x00);
//	if (ps_value_high[0] < 0) {
//		goto EXIT_ERR;
//	}
//	return !((ps_value_high[0]&0x10)>>4);
//#else
//	ps_value_high[0] = ap3xx6_read_reg(client, AP3xx6_PDATA_H, 0xFF, 0x00);
//	if (ps_value_high[0] < 0) {
//		goto EXIT_ERR;
//	}
//	/* APS_LOG("the ps_value_h>>7 is %d\n",ps_value_high[0]>>7); */
//	return !(ps_value_high[0]>>7);
//#endif
//
//EXIT_ERR:
//	APS_ERR("ap3xx6_get_obj fail\n");
//	return 0;
//}

#ifdef DI_AUTO_CAL
u8 Calibration_Flag = 0;
int ap3xx6_Calibration(struct i2c_client *client)
{
	int err;
	int i = 0;
	u16 ps_data = 0;
	/* struct i2c_client *client = (struct i2c_client*)file->private_data; */
//	struct ap3xx6_priv *obj = i2c_get_clientdata(client);
	if(ap3xx6_openlog) APS_LOG("ap3425 C_F =%d\n", Calibration_Flag);
	if (Calibration_Flag == 0) {
		for (i = 0; i < 4; i++) {
			err = ap3xx6_read_ps(ap3xx6_obj->client, &ap3xx6_obj->ps);
			if (err != 0) {
				goto err_out;
			}
			if(ap3xx6_openlog)APS_LOG("ap3425 ps =%d\n", ap3xx6_obj->ps);
			if ((obj->ps) > DI_PS_CAL_THR) {
				Calibration_Flag = 0;
				goto err_out;
			} else{
				ps_data += ap3xx6_obj->ps;
			}
			msleep(100);
		}
		Calibration_Flag = 1;
		if(ap3xx6_openlog) APS_LOG("ap3425 ps_data1 =%d\n", ps_data);

		ps_data = ps_data/4;

		if (atomic_read(&ap3xx6_obj->trace) & TRACE_DEBUG) {
			APS_LOG("ap3425 ps_data2 =%d\n", ps_data);
		}
		ap3xx6_set_pcrosstalk(ap3xx6_obj->client, ps_data);
	}
	return 1;
err_out:
	APS_ERR("ap3xx6_read_ps fail\n");
	return -1;
}
#endif
/*----------------------------------------------------------------------------*/
static void ap3xx6_eint_work(struct work_struct *work)
{
	int err=0;
	int value = -1;
	int status = -1;
#if DELAYED_WORK
#else
	int index = 0;
#endif
//	if (atomic_read(&ap3xx6_obj->trace) & TRACE_DEBUG) {
//		APS_LOG("%s--%d\n", __func__, __LINE__);
//	}

	err = ap3xx6_als_get_data(&value, &status);
	if (err < 0) {
		APS_ERR("ap3xx6_als_get_data failed\n");
	}else{
		if(ap3xx6_openlog) APS_LOG("ap3xx6_eint_work lux = %d\n",value);
	}
#if DELAYED_WORK
	schedule_delayed_work(&ap3xx6_obj->eint_work, msecs_to_jiffies(atomic_read(&ap3xx6_obj->delay)));
#else
	for(index = 0 ; index < AP3425H_TABLE_LENGTH ; index++) {
		if( value <= defalut_table[index] ) {
			ap3425_set_ahthres( ap3xx6_obj->client, (cali_table[index]+1) );
			ap3425_set_althres( ap3xx6_obj->client, (index ==0) ? 0 : cali_table[index-1] );
			if(ap3xx6_openlog) APS_LOG("set_thres index[%d] => [ %d , %d ]\n", index, (cali_table[index]+1), (index ==0) ? 0 : cali_table[index-1] );
			break;
		}
	}
	enable_irq(alsps_irq);
#endif

}
/*----------------------------------------------------------------------------*/

#ifdef SENSOR_PS
static ssize_t ap3xx6_ps_rawdata_show(struct device_driver *ddri, char *buf)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	int err = 0;
	u16 ps = -1;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}

	err = ap3xx6_read_ps(ap3xx6_obj->client, &ps);
	if (err < 0) {
		APS_ERR("ap3xx6_read_ps failed\n");
		return -1;
	}
	return sprintf(buf, "ps raw dat= %d\n", ps);
}
static ssize_t ap3xx6_ps_data_show(struct device_driver *ddri, char *buf)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	int err = 0;
	int value = -1;
	int status = -1;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}

	err = ap3xx6_ps_get_data(&value, &status);
	if (err < 0) {
		APS_ERR("ap3xx6_ps_get_data failed\n");
		return -1;
	}
	return sprintf(buf, "ps dat= %d,,status=%d\n", value, status);
}
#endif
static ssize_t ap3xx6_als_rawdata_show(struct device_driver *ddri, char *buf)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	int err = 0;
	u16 als = -1;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}

	err = ap3xx6_read_als(ap3xx6_obj->client, &als);
	if (err < 0) {
		APS_ERR("ap3xx6_read_als failed\n");
		return -1;
	}
	return sprintf(buf, "als raw dat= %d\n", als);
}
static ssize_t ap3xx6_als_data_show(struct device_driver *ddri, char *buf)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	int err = 0;
	int value = -1;
	int status = -1;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}

	err = ap3xx6_als_get_data(&value, &status);
	if (err < 0) {
		APS_ERR("ap3xx6_als_get_data failed\n");
		return -1;
	}
	return sprintf(buf, "als dat= %d,,status=%d\n", value, status);
}

static ssize_t ap3xx6_trace_show(struct device_driver *ddri, char *buf)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	int ret = 0;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}
	ret = sprintf(buf, "ap3xx6_obj->trace = %d,\n", atomic_read(&ap3xx6_obj->trace));

	return ret;
}
static ssize_t ap3xx6_trace_store(struct device_driver *ddri, const char *buf, size_t count)
{

	int val = 0;
	int ret = 0;

	if (!ap3xx6_obj) {
		APS_ERR("ap3xx6_obj is null!!\n");
		return -1;
	}
	ret = sscanf(buf, "0x%x", &val);
	if (ret == 1) {
		atomic_set(&ap3xx6_obj->trace, val);
	}

	return count;
}

static ssize_t ap3xx6_status_show(struct device_driver *ddri, char *buf)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	int ret = 0;
//	int value = -1;
//	int status = -1;
	int i = 0;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}
	ret += sprintf(buf+ret, "obj->irq = %d,\n", alsps_irq);
#ifdef CUST_EINT_ALS_NUM
	ret += sprintf(buf+ret, "CUST_EINT_ALS_NUM = %d,\n", CUST_EINT_ALS_NUM);
#endif
	ret += sprintf(buf+ret, "als_level:");
	for (i = 0; i < sizeof(ap3xx6_obj->hw->als_level)/sizeof(ap3xx6_obj->hw->als_level[0]); i++) {
		ret += sprintf(buf+ret, "%d, ", ap3xx6_obj->hw->als_level[i]);
	}
	ret += sprintf(buf+ret, "\n als_value:");
	for (i = 0; i < sizeof(ap3xx6_obj->hw->als_value)/sizeof(ap3xx6_obj->hw->als_value[0]); i++) {
		ret += sprintf(buf+ret, "%d, ", ap3xx6_obj->hw->als_value[i]);
	}

	ret += sprintf(buf+ret, "\n ps_thd_val_h= %d,,ps_thd_val_l=%d, i2c_num = %d\n",
		atomic_read(&ap3xx6_obj->ps_thd_val_h), atomic_read(&ap3xx6_obj->ps_thd_val_l), ap3xx6_obj->hw->i2c_num);
	return ret;
}

static ssize_t ap3xx6_em_read(struct device_driver *ddri, char *buf)
{
	u16 idx = 0;
	int count = 0;
	int reg_value[1];
#ifdef ap3425
		#define ap3xx6_NUM_CACHABLE_REGS	29
		u8 ap3xx6_reg[ap3xx6_NUM_CACHABLE_REGS] = {
		0x00, 0x01, 0x02, 0x06, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x14, 0x1a, 0x1b, 0x1c, 0x1d,
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d};
#else
		#define ap3xx6_NUM_CACHABLE_REGS	26
		u8 ap3xx6_reg[ap3xx6_NUM_CACHABLE_REGS] = {
		0x00, 0x01, 0x02, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
		0x20, 0x21, 0x22, 0x23, 0x24, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d};
#endif
	if (!ap3xx6_obj) {
		APS_ERR("ap3xx6_obj is null!!\n");
		count += sprintf(buf+count, "ap3xx6_obj is null!!\n");
		return count;
	}
	for (idx = 0; idx < ap3xx6_NUM_CACHABLE_REGS; idx++) {

			reg_value[0] = ap3xx6_read_reg(ap3xx6_obj->client, ap3xx6_reg[idx], 0xFF, 0x00);
		if (reg_value[0] < 0) {
			count += sprintf(buf+count, "i2c read_reg err\n");
			return count;
		}
		count += sprintf(buf+count, "[%x]=0x%x\n", ap3xx6_reg[idx], reg_value[0]);
	}
	ap3xx6_read_ps(ap3xx6_obj->client, &idx);
	count += sprintf(buf+count, "[ps]=%d\n", idx);
	return count;
}

static ssize_t ap3xx6_em_write(struct device_driver *ddri, const char *buf, size_t count)
{

	int addr, val;
	int ret = 0;

	if (!ap3xx6_obj) {
		APS_ERR("ap3xx6_obj is null!!\n");
		return -1;
	}

	ret = sscanf(buf, "%x %x", &addr, &val);

	APS_LOG("Reg[%x].Write [%x]..\n", addr, val);

	ret = ap3xx6_write_reg(ap3xx6_obj->client, addr, 0xFF, 0x00, val);

	return count;
}
static DRIVER_ATTR(em, 0660, ap3xx6_em_read, ap3xx6_em_write);
#ifdef SENSOR_PS
static DRIVER_ATTR(ps_rawdata, 0660, ap3xx6_ps_rawdata_show, NULL);
static DRIVER_ATTR(ps_data, 0660, ap3xx6_ps_data_show, NULL);
#endif
static DRIVER_ATTR(als_rawdata, 0660, ap3xx6_als_rawdata_show, NULL);
static DRIVER_ATTR(als_data, 0600, ap3xx6_als_data_show, NULL);
static DRIVER_ATTR(state, 0660, ap3xx6_status_show, NULL);
static DRIVER_ATTR(trace, 0660, ap3xx6_trace_show, ap3xx6_trace_store);



static struct driver_attribute *ap3xx6_attr_list[] = {
	&driver_attr_em,
#ifdef SENSOR_PS
	&driver_attr_ps_rawdata,
	&driver_attr_ps_data,
#endif
	&driver_attr_als_rawdata,
	&driver_attr_als_data,
	&driver_attr_state,
	&driver_attr_trace,
};

static int ap3xx6_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ap3xx6_attr_list)/sizeof(ap3xx6_attr_list[0]));
	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, ap3xx6_attr_list[idx]);
		if (err != 0) {
			APS_ERR("driver_create_file (%s) = %d\n", ap3xx6_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int ap3xx6_delete_attr(struct device_driver *driver)
{
	int idx , err = 0;
	int num = (int)(sizeof(ap3xx6_attr_list)/sizeof(ap3xx6_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, ap3xx6_attr_list[idx]);
	}

	return err;
}

static int ls_update_table(void)
{
	uint32_t tmp_data[AP3425H_TABLE_LENGTH];
	int i;
//	if(ap3425h_cali.cali_state==1){
		for (i = 0; i < AP3425H_TABLE_LENGTH; i++) {
			tmp_data[i] = (uint32_t)(*(defalut_table + i)) * (ap3425h_cali.als_cali_adc)/ap3425h_cali.als_cali_base;
			if (tmp_data[i] <= 0xFFFF)
				cali_table[i] = (uint16_t) tmp_data[i];
			else
				cali_table[i] = 0xFFFF;
			APS_LOG("Table[%d],%d -> %d\n", i ,defalut_table[i], cali_table[i]);
		}
//	}else{
//		APS_LOG( "alp.D : no need to update !!\n");
//	}
	return 0;
}

#ifdef AP3425H_DELAY_CALIBRATION
static int ap3425_read_from_califile(void) 
{
	int ilen = 0;
	char tmp_data[32] = {0};
	int base=0, calilux=0;
	struct file *fp = NULL;

	if(ap3425h_cali.cali_state==1) {
		APS_LOG("ALS Calibration already loaded !\n");
		return 0;
	}
		
	old_als_fs = get_fs();
	set_fs(KERNEL_DS);
//	fp=filp_open(ALS_CALI_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	fp=filp_open(ALS_CALI_PATH,O_RDWR,00770);
	if(IS_ERR(fp)){
		APS_LOG("ALS filp_open fail [%s] \n", ALS_CALI_PATH);
		return 0;
	}
	ilen = fp->f_op->read(fp,tmp_data,AP3425H_CALI_FILE_LENGTH,&fp->f_pos);
	if(ilen == AP3425H_CALI_FILE_LENGTH) {
		sscanf(tmp_data, "%04d,%04d\n", &base, &calilux);
		ap3425h_cali.als_cali_base= base;
		ap3425h_cali.als_cali_adc= calilux;
		APS_LOG("base [%d], calilux[%d]\n", base, calilux);
		APS_LOG("Load Calibration base [%d] , adc[%d] \n", ap3425h_cali.als_cali_base, ap3425h_cali.als_cali_adc);
		ap3425h_cali.cali_state=1;
	} else {
		ap3425h_cali.als_cali_base= 1000;
		ap3425h_cali.als_cali_adc= AP3425H_GOLDEN_VALUE_Z380M;
		APS_LOG("ilen[%d] != 32 \n", ilen);	
		APS_LOG("Use Defalut Calibration base [%d] , adc[%d] \n", ap3425h_cali.als_cali_base, ap3425h_cali.als_cali_adc);
		ap3425h_cali.cali_state=0;
	}
	set_fs(old_als_fs);
	filp_close(fp,NULL);

//	if(ap3425h_cali.cali_state==1) 
		ls_update_table();

	return ap3425h_cali.cali_state;
}
#endif

static int ap3425_write_to_califile(int base,int calilux)
{
	int retval = 1, ilen=0;
	char tmp_data[32]={0};
	struct file *fp=NULL;
	
	old_als_fs = get_fs();
	set_fs(KERNEL_DS);
	fp=filp_open(ALS_CALI_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		retval = 0;
		APS_LOG("alp.D : als filp_open fail\n");
		return retval;
	}
	sprintf(tmp_data, "%04d,%04d\n", base, calilux);
	APS_LOG( "%s \n",tmp_data);

	ilen = fp->f_op->write(fp,tmp_data,AP3425H_CALI_FILE_LENGTH,&fp->f_pos);
	APS_LOG( "alp.D : als write file len : %d\n",ilen);
	set_fs(old_als_fs);
	filp_close(fp,NULL);
	
	return retval;
}

static int ap3425_get_mode(struct i2c_client *client)
{
    int ret;

    ret = ap3xx6_read_reg(client, AP3425_REG_SYS_CONF,
	    AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT);
    return ret;
}

//----------------------------- enable --------------------------------------
static ssize_t ap3425h_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/*int als_data = 0;;
	als_data = ap3xx6_read_reg(ap3xx6_obj->client, AP3xx6_ENABLE, 0xFF, 0x00);
	if (als_data < 0) {
		als_data = 0;
		APS_LOG("read thres L_L fail !!\n");
	}
	return sprintf(buf, "%d\n", als_data);*/
	
	return sprintf(buf, "%d\n", ap3425_get_mode(ap3xx6_obj->client));
}
static ssize_t ap3425h_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long mode = AP3425_SYS_ALS_ENABLE;
	int ret;

	if(buf[0]=='1'){
		mode = AP3425_SYS_ALS_ENABLE;
		APS_LOG("alp.D : Power set_mode 1 !!\n");
	}else{
		mode = AP3425_SYS_DEV_DOWN;
		APS_LOG("alp.D : Power set_mode 0 !!\n");
	}

	if( (mode == AP3425_SYS_ALS_ENABLE) && (ap3425_get_mode(ap3xx6_obj->client) != AP3425_SYS_ALS_ENABLE) ) {
#if DELAYED_WORK
		schedule_delayed_work(&ap3xx6_obj->eint_work, 1100*HZ/1000);
#else
		//ap3425_set_ahthres(data->client, 0);
		//ap3425_set_althres(data->client, 65535);
		ret = ap3xx6_write_reg(ap3xx6_obj->client, AP3425_REG_ALS_THDH_L, AP3425_REG_ALS_THDH_L_MASK, AP3425_REG_ALS_THDH_L_SHIFT, 0x0);
		if (ret < 0)
			return ret;
		ret = ap3xx6_write_reg(ap3xx6_obj->client, AP3425_REG_ALS_THDH_H, AP3425_REG_ALS_THDH_H_MASK, AP3425_REG_ALS_THDH_H_SHIFT, 0x0);
		if (ret < 0)
			return ret;
		ret = ap3xx6_write_reg(ap3xx6_obj->client, AP3425_REG_ALS_THDL_L, AP3425_REG_ALS_THDL_L_MASK, AP3425_REG_ALS_THDL_L_SHIFT, 0xFF);
		if (ret < 0)
			return ret;
		ret = ap3xx6_write_reg(ap3xx6_obj->client, AP3425_REG_ALS_THDL_H, AP3425_REG_ALS_THDL_H_MASK, AP3425_REG_ALS_THDL_H_SHIFT, 0xFF);
		if (ret < 0)
			return ret;
#endif
		APS_LOG("alp.D : open als \n");
		ret = ap3xx6_write_reg(ap3xx6_obj->client, AP3425_REG_SYS_CONF, AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, AP3425_SYS_ALS_ENABLE);
		if (ret < 0)
			return ret;
		APS_LOG("alp.D : open success \n");
	} else if(mode == AP3425_SYS_DEV_DOWN) {
#if DELAYED_WORK
		cancel_delayed_work_sync(&ap3xx6_obj->eint_work);
#else
		ret = ap3xx6_write_reg(ap3xx6_obj->client, AP3425_REG_SYS_CONF, AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, AP3425_SYS_DEV_RESET);
#endif
		ret = ap3xx6_write_reg(ap3xx6_obj->client, AP3425_REG_SYS_CONF, AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, AP3425_SYS_DEV_DOWN);
	}
	
//	int ret = -1;
//	u8 databuf[2];
//	databuf[0] = AP3xx6_ENABLE;
//	
//	if(buf[0]=='1'){
//		databuf[1] = 0x01;
//		ret = ap3xx6_write_reg(ap3xx6_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
//		if(ap3xx6_openlog) APS_LOG("Power set_mode 1 !!\n");
//#if DELAYED_WORK
//		if (ret > 0) 
//			schedule_delayed_work(&ap3xx6_obj->eint_work, 1100*HZ/1000);
//#endif
//	}else{
//		databuf[1] = 0x00;
//		ret = ap3xx6_write_reg(ap3xx6_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
//		if(ap3xx6_openlog) APS_LOG("Power set_mode 0 !!\n");
//#if DELAYED_WORK
//		if (ret > 0) 
//			cancel_delayed_work_sync(&ap3xx6_obj->eint_work);
//#endif
//	}
//	if(ap3xx6_openlog) APS_LOG(" set Power %c ret=%d !!\n",buf[0] ,ret);
	
	return count;
}

//----------------------------- delay --------------------------------------
static ssize_t ap3425h_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_delay = 0;;
	als_delay = atomic_read(&ap3xx6_obj->delay);
	if (als_delay < 0) {
		als_delay = 0;
		APS_ERR("read delay fail !!\n");
	}else{
		if(ap3xx6_openlog)APS_LOG("show delay = %d\n",als_delay);	
	}
	return sprintf(buf, "%d\n", als_delay);
}
static ssize_t ap3425h_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms)){
		APS_ERR(" get interval_ms = %ld  fail!!\n",interval_ms);
	}

	if ((interval_ms < 1) || (interval_ms > 1000)){
		APS_ERR(" overflow interval_ms !!\n");
		interval_ms = 1000 ; // default 1000 ms
	}

	atomic_set(&ap3xx6_obj->delay, interval_ms);
	if(ap3xx6_openlog) APS_LOG(" set delay = %ld !!\n",interval_ms);
	
	return count;
}

//----------------------------- gain --------------------------------------
static ssize_t ap3425h_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_gain=0;

	als_gain = ap3xx6_read_reg(ap3xx6_obj->client, AP3xx6_GAIN, 0xFF, 0x00);
	switch(als_gain){
		case 0 :
			APS_LOG("gain 34304 lux!!\n");
		break ;

		case 1:
			APS_LOG("gain 8576 lux!!\n");
		break ;

		case 2:
			APS_LOG("gain 2144 lux!!\n");
		break ;

		case 3:
			APS_LOG("gain 536 lux!!\n");
		break ;

		default :
			APS_LOG("read gain fail !!\n");
		break;
	}
	return sprintf(buf, "%d\n", als_gain);
}
static ssize_t ap3425h_gain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	u8 databuf[2];

	databuf[0] = AP3xx6_GAIN;
	if(buf[0]=='0'){
		databuf[1] = 0x00;
		APS_LOG("Power gain 0 !!\n");
	}else if(buf[0]=='1'){
		databuf[1] = 0x01;
		APS_LOG("Power gain 1 !!\n");
	}else if(buf[0]=='2'){
		databuf[1] = 0x02;
		APS_LOG("Power gain 2 !!\n");
	}else{
		databuf[1] = 0x03;
		APS_LOG("Power gain 3 !!\n");
	}
	ret = ap3xx6_write_reg(ap3xx6_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
	APS_LOG(" set gain = %c , ret = %d !!\n",buf[0] ,ret);
	
	return count;
}

//----------------------------- lux --------------------------------------
static ssize_t ap3425h_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err = 0;
	err = ap3xx6_read_als(ap3xx6_obj->client, &ap3xx6_obj->als);
	if(ap3xx6_openlog) APS_LOG("(%d) ap3425 ALS adc=%d\n",err ,ap3xx6_obj->als);

	return sprintf(buf, "%d\n", ap3xx6_obj->als);
}

//----------------------------- threshold low --------------------------------------
static ssize_t ap3425h_thres_low_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_value_low[2];
	int als_thres_L=0;
	/* get ALS adc count */
	als_value_low[0] = ap3xx6_read_reg(ap3xx6_obj->client, AP3425_REG_ALS_THDL_L, AP3425_REG_ALS_THDL_L_MASK, AP3425_REG_ALS_THDL_L_SHIFT);
	if (als_value_low[0] < 0) {
		als_value_low[0] = 0;
		APS_LOG("read thres L_L fail !!\n");
	}
	als_value_low[1] = ap3xx6_read_reg(ap3xx6_obj->client, AP3425_REG_ALS_THDL_H, AP3425_REG_ALS_THDL_H_MASK, AP3425_REG_ALS_THDL_H_SHIFT);
	if (als_value_low[1] < 0) {
		als_value_low[1] = 0;
		APS_LOG("read thres L_H fail !!\n");
	}
	als_thres_L = als_value_low[0]+ (als_value_low[1] *256);

	return sprintf(buf, "%d\n", als_thres_L);
}
static ssize_t ap3425h_thres_low_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	u8 databuf_L[2];
	u8 databuf_H[2];

	databuf_L[0] = AP3425_REG_ALS_THDL_L;
	databuf_H[0] = AP3425_REG_ALS_THDL_H;
	if(buf[0]=='0'){
		databuf_L[1] = 0xFF;
		databuf_H[1] = 0xFF;
		APS_LOG("L threshold_set_mode 0--> FFFF !!\n");
	}else if(buf[0]=='1'){
		databuf_L[1] = 0x64;
		databuf_H[1] = 0x00;
		APS_LOG("L threshold_set_mode 1--> 100 !!\n");
	}else if(buf[0]=='2'){
		databuf_L[1] = 0xF4;
		databuf_H[1] = 0x01;
		APS_LOG("L threshold_set_mode 2--> 500 !!\n");
	}else if(buf[0]=='3'){
		databuf_L[1] = 0xE8;
		databuf_H[1] = 0x03;
		APS_LOG("L threshold_set_mode 3--> 1000 !!\n");
	}else{
		databuf_L[1] = 0x00;
		databuf_H[1] = 0x00;
		APS_LOG("L default !! -> 0  !!\n");
	}
	ret += ap3xx6_write_reg(ap3xx6_obj->client, databuf_L[0], AP3425_REG_ALS_THDL_L_MASK, AP3425_REG_ALS_THDL_L_SHIFT, databuf_L[1]);
	ret += ap3xx6_write_reg(ap3xx6_obj->client, databuf_H[0], AP3425_REG_ALS_THDL_H_MASK, AP3425_REG_ALS_THDL_H_SHIFT, databuf_H[1]);
	APS_LOG(" set L threshold ret = %d !!\n",ret);

	return count;
}

//----------------------------- threshold high --------------------------------------
static ssize_t ap3425h_thres_high_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_value_high[2];
	int als_thres_H=0;
	/* get ALS adc count */
	als_value_high[0] = ap3xx6_read_reg(ap3xx6_obj->client, AP3425_REG_ALS_THDH_L, AP3425_REG_ALS_THDH_L_MASK, AP3425_REG_ALS_THDH_L_SHIFT);
	if (als_value_high[0] < 0) {
		als_value_high[0] = 0;
		APS_LOG("read thres H_L fail !!\n");
	}
	als_value_high[1] = ap3xx6_read_reg(ap3xx6_obj->client, AP3425_REG_ALS_THDH_H, AP3425_REG_ALS_THDH_H_MASK, AP3425_REG_ALS_THDH_H_SHIFT);
	if (als_value_high[1] < 0) {
		als_value_high[1] = 0;
		APS_LOG("read thres H_H fail !!\n");
	}
	als_thres_H = als_value_high[0]+ (als_value_high[1] *256);

	return sprintf(buf, "%d\n", als_thres_H);
}
static ssize_t ap3425h_thres_high_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	u8 databuf_L[2];
	u8 databuf_H[2];

	databuf_L[0] = AP3425_REG_ALS_THDH_L;
	databuf_H[0] = AP3425_REG_ALS_THDH_H;
	if(buf[0]=='0'){
		databuf_L[1] = 0x00;
		databuf_H[1] = 0x00;
		APS_LOG("H threshold_set_mode 0--> 0 !!\n");
	}else if(buf[0]=='1'){
		databuf_L[1] = 0x2C;
		databuf_H[1] = 0x01;
		APS_LOG("H threshold_set_mode 1--> 300 !!\n");
	}else if(buf[0]=='2'){
		databuf_L[1] = 0xE8;
		databuf_H[1] = 0x03;
		APS_LOG("H threshold_set_mode 2--> 1000 !!\n");
	}else if(buf[0]=='3'){
		databuf_L[1] = 0xDC;
		databuf_H[1] = 0x05;
		APS_LOG("H threshold_set_mode 3--> 1500 !!\n");
	}else{
		databuf_L[1] = 0xFF;
		databuf_H[1] = 0xFF;
		APS_LOG("H default !! -> FFFF  !!\n");
	}
	ret += ap3xx6_write_reg(ap3xx6_obj->client, databuf_L[0], AP3425_REG_ALS_THDH_L_MASK, AP3425_REG_ALS_THDH_L_SHIFT, databuf_L[1]);
	ret += ap3xx6_write_reg(ap3xx6_obj->client, databuf_H[0], AP3425_REG_ALS_THDH_H_MASK, AP3425_REG_ALS_THDH_H_SHIFT, databuf_H[1]);
	APS_LOG(" set H threshold ret = %d !!\n",ret);

	return count;
}

//-----------------------------  Interrupt PIN status --------------------------------------
static ssize_t ap3425h_pin_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	ret = ap3xx6_read_reg(ap3xx6_obj->client, AP3xx6_INT_STATUS, 0xFF, 0x00);
	APS_LOG("ap3425 Interrput status %d   --(ALS:B0 ; PS:B1)-- \n",ret);

	return sprintf(buf, "PIN 0x%x\n", ret);
}

//-----------------------------  Control Pin --------------------------------------
static ssize_t ap3425h_control_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	ret = ap3xx6_read_reg(ap3xx6_obj->client, AP3xx6_CONYROL_INT, 0xFF, 0x00);
	APS_LOG("ap3425 AP3xx6_CONYROL_INT %d   --(ALS:B4 ; PS:B8)-- \n",ret);

	return sprintf(buf, "Control PIN 0x%x\n", ret);
}

static ssize_t ap3425h_control_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	u8 databuf[2];
	databuf[0] = AP3xx6_CONYROL_INT;

	if(buf[0]=='0'){
		databuf[1] = 0x01;
		APS_LOG("control_PIN_set_mode 0--> clear INT !!\n");
	}else if(buf[0]=='1'){
		databuf[1] = 0x08;
		APS_LOG("control_PIN_set_mode 1--> PS:0, ALS:1 !!\n");
	}else if(buf[0]=='2'){
		databuf[1] = 0x88;
		APS_LOG("control_PIN_set_mode 2--> PS:1, ALS:1 !!\n");
	}else{
		databuf[1] = 0x08;
		APS_LOG("control_PIN_set_mode ?--> PS:0, ALS:1 !!\n");
	}

	ret = ap3xx6_write_reg(ap3xx6_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);

	return count;
}

//-----------------------------  do calibration by owner --------------------------------------
static ssize_t ap3425h_do_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	int power_state=0, do_cali=0, iloop=0, lux=0, ret=0;
	u8 databuf[2];

	power_state = ap3xx6_read_reg(ap3xx6_obj->client, AP3xx6_ENABLE, 0xFF, 0x00);
	if (power_state == AP3425_SYS_DEV_DOWN){
		databuf[0] = AP3xx6_ENABLE;
		databuf[1] = 0x01;
		ret = ap3xx6_write_reg(ap3xx6_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
	}
	CALI_TEST_DELAY();
	ap3425h_cali.cali_state = 0;
	ap3425h_cali.do_calibrating = 1;
	for(iloop=0;iloop<AP3425H_CALI_SAMPLE_TIME;iloop++){
		ap3xx6_read_als(ap3xx6_obj->client, &ap3xx6_obj->als);
		lux = lux + ap3xx6_obj->als;
		CALI_TEST_DELAY();
		if(ap3xx6_openlog) APS_LOG("Total calibration lux = %d\n",lux);
	}
	ap3425h_cali.als_cali_adc = lux/AP3425H_CALI_SAMPLE_TIME;

	if(ap3425h_cali.als_cali_adc<ap3425h_cali.cal_threshold_low){
		do_cali = 0;
		return sprintf(buf, "%d\n", do_cali);
	}else if(ap3425h_cali.als_cali_adc>ap3425h_cali.cal_threshold_high){
		do_cali = 0;
		return sprintf(buf, "%d\n", do_cali);
	}

	if(power_state!=1){
		databuf[0] = AP3xx6_ENABLE;
		databuf[1] = AP3425_SYS_DEV_DOWN;
		ret = ap3xx6_write_reg(ap3xx6_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
	}
	ret = ap3425_write_to_califile(ap3425h_cali.als_cali_base, ap3425h_cali.als_cali_adc);
	if(ret!=1){
		ap3425h_cali.als_cali_adc = AP3425H_GOLDEN_VALUE_Z380M;
		ap3425h_cali.als_cali_base=1000;
		ap3425h_cali.cali_state = 0;
		if(ap3xx6_openlog) APS_LOG("do calibration write file fail\n");
	}else{
		if(ap3xx6_openlog) APS_LOG("Need to update table after calibration\n");
		ls_update_table();
		ap3425h_cali.cali_state = 1;
		do_cali = 1;
	}
	ap3425h_cali.do_calibrating = 0;
	return sprintf(buf, "%d\n", do_cali);
}

static ssize_t ap3425h_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//	struct ap3425_data *data = ap3425_data_g;
	int power_state=0, do_cali=0, iloop=0, lux=0, ret=0;
	u8 databuf[2];

	APS_LOG("ap3425_store_calibration_state..\n");
	if(buf[0]=='0'){
		ap3425h_cali.cali_state = 0;
		ap3425h_cali.als_cali_base = 1000;
		ap3425h_cali.als_cali_adc = AP3425H_GOLDEN_VALUE_Z380M;
		if(ap3xx6_openlog) APS_LOG(" calibration reset!!\n");
	}else if(buf[0]=='1'){
		do_cali = 1;
		if(ap3xx6_openlog) APS_LOG(" calibration lux 1000!!\n");
	}else if(buf[0]=='2'){
		do_cali = 1;
		if(ap3xx6_openlog) APS_LOG(" calibration lux 200!!\n");
	}else{
		do_cali = 0;
		ap3425h_cali.als_cali_base = 200;
		if(ap3xx6_openlog) APS_LOG("error input !! none done\n");
	}
	ap3425h_cali.do_calibrating = 1;
	power_state = ap3xx6_read_reg(ap3xx6_obj->client, AP3xx6_ENABLE, 0xFF, 0x00);
	if(do_cali==1){
		ap3425h_cali.cali_state = 0;
		if (power_state == AP3425_SYS_DEV_DOWN){
			databuf[0] = AP3xx6_ENABLE;
			databuf[1] = 0x01;
			ret = ap3xx6_write_reg(ap3xx6_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
		}
		CALI_TEST_DELAY();
		for(iloop=0;iloop<AP3425H_CALI_SAMPLE_TIME;iloop++){
			ap3xx6_read_als(ap3xx6_obj->client, &ap3xx6_obj->als);
			lux = lux + ap3xx6_obj->als;
			CALI_TEST_DELAY();
			if(ap3xx6_openlog) APS_LOG("Total calibration lux = %d\n",lux);
		}
		ap3425h_cali.als_cali_adc = lux/AP3425H_CALI_SAMPLE_TIME;
		if(power_state!=1){
			databuf[0] = AP3xx6_ENABLE;
			databuf[1] = AP3425_SYS_DEV_DOWN;
			ret = ap3xx6_write_reg(ap3xx6_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
		}
		do_cali = ap3425_write_to_califile(ap3425h_cali.als_cali_base, ap3425h_cali.als_cali_adc);
		if(do_cali!=1){
			ap3425h_cali.als_cali_adc = AP3425H_GOLDEN_VALUE_Z380M;
			ap3425h_cali.als_cali_base=1000;
			ap3425h_cali.cali_state = 0;
		}else{
			if(ap3xx6_openlog) APS_LOG("Need to update table after calibration\n");
			ls_update_table();
			ap3425h_cali.cali_state = 1;
		}
	}
	printk("%d\n", do_cali);
	ap3425h_cali.do_calibrating = 0;
	return count;
}

//-----------------------------  cal_threshold_low --------------------------------------
static ssize_t cal_threshold_low_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ap3425h_cali.cal_threshold_low);
}
static ssize_t cal_threshold_low_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(buf[0]=='-'){
		ap3425h_cali.cal_threshold_low = ap3425h_cali.cal_threshold_low-5;
		APS_LOG("cal_threshold_low - 5\n");
	}else{
		ap3425h_cali.cal_threshold_low = ap3425h_cali.cal_threshold_low+5;
		APS_LOG("cal_threshold_low + 5\n");
	}
	return count;
}

//-----------------------------  cal_threshold_high --------------------------------------
static ssize_t cal_threshold_high_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ap3425h_cali.cal_threshold_high);
}
static ssize_t cal_threshold_high_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(buf[0]=='-'){
		ap3425h_cali.cal_threshold_high = ap3425h_cali.cal_threshold_high-50;
		APS_LOG("cal_threshold_high - 50\n");
	}else{
		ap3425h_cali.cal_threshold_high = ap3425h_cali.cal_threshold_high+50;
		APS_LOG("cal_threshold_high + 50\n");
	}
	return count;
}

//-----------------------------  factory calibration --------------------------------------
static ssize_t ap3425h_factory_cal_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	return count;
}

//-----------------------------  status --------------------------------------
static ssize_t ap3425h_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ap3xx6_init_flag);
}

//-----------------------------  ATD cali-als --------------------------------------
static ssize_t ap3425h_atd_calials_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(ap3425h_cali.cali_state==1)
		return sprintf(buf, "%d\n", ap3425h_cali.als_cali_adc);
	else{
		APS_LOG("Non-calibration value use default !!\n");
	}
	return sprintf(buf, "%d\n", 0);
}

//-----------------------------  openlog --------------------------------------
static ssize_t ap3425h_openlog_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ap3xx6_openlog);
}
static ssize_t ap3425h_openlog_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(buf[0]=='1'){
		ap3xx6_openlog = 1;
		APS_LOG("open debug log\n");
	}else{
		ap3xx6_openlog = 0;
		APS_LOG("close debug log\n");
	}
	return count;
}

static DEVICE_ATTR(enable, 0660, ap3425h_enable_show, ap3425h_enable_store);
static DEVICE_ATTR(delay, 0660, ap3425h_delay_show, ap3425h_delay_store);
static DEVICE_ATTR(gain, 0660, ap3425h_gain_show, ap3425h_gain_store);
static DEVICE_ATTR(lux, 0660, ap3425h_lux_show, NULL);
static DEVICE_ATTR(als_thres_low, 0660, ap3425h_thres_low_show, ap3425h_thres_low_store);
static DEVICE_ATTR(als_thres_high, 0660, ap3425h_thres_high_show, ap3425h_thres_high_store);
static DEVICE_ATTR(pin_status, 0660, ap3425h_pin_status_show, NULL);
static DEVICE_ATTR(control_pin, 0660, ap3425h_control_pin_show, ap3425h_control_pin_store);
static DEVICE_ATTR(calibration, 0660, ap3425h_do_calibration, ap3425h_calibration_store);
static DEVICE_ATTR(cal_threshold_low, 0660, cal_threshold_low_show, cal_threshold_low_store);
static DEVICE_ATTR(cal_threshold_high, 0660, cal_threshold_high_show, cal_threshold_high_store);
static DEVICE_ATTR(factory_cal, 0660, NULL, ap3425h_factory_cal_store);
static DEVICE_ATTR(state, 0660, ap3425h_status_show, NULL);
static DEVICE_ATTR(atd_calials, 0660, ap3425h_atd_calials_show, NULL);
static DEVICE_ATTR(openlog, 0660, ap3425h_openlog_show, ap3425h_openlog_store);

static struct attribute *ap3425h_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_gain.attr,
	&dev_attr_lux.attr,
	&dev_attr_als_thres_low.attr,
	&dev_attr_als_thres_high.attr,
	&dev_attr_pin_status.attr,
	&dev_attr_control_pin.attr,
	&dev_attr_calibration.attr,
	&dev_attr_cal_threshold_low.attr,
	&dev_attr_cal_threshold_high.attr,
	&dev_attr_factory_cal.attr,
	&dev_attr_state.attr,
	&dev_attr_atd_calials.attr,
	&dev_attr_openlog.attr,
	NULL
};

static struct attribute_group ap3425h_attribute_group = {
	.attrs = ap3425h_attributes
};


/******************************************************************************
 * Function Configuration
******************************************************************************/
static int ap3xx6_open(struct inode *inode, struct file *file)
{
	file->private_data = ap3xx6_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ap3xx6_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long ap3xx6_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
//	struct i2c_client *client = (struct i2c_client *)file->private_data;
//	struct ap3xx6_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user *) arg;
	int dat;
	uint32_t enable;

	switch (cmd) {
#ifdef SENSOR_PS
	case ALSPS_SET_PS_MODE:
			if (copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			if (enable) {
				err = ap3xx6_enable_ps(ap3xx6_obj->client, 1);
				if (err != 0) {
					APS_ERR("enable ps fail: %ld\n", err);
					goto err_out;
				}
				msleep(100);

				set_bit(CMC_BIT_PS, &ap3xx6_obj->enable);

			} else{
				err = ap3xx6_enable_ps(ap3xx6_obj->client, 0);
				if (err != 0) {
					APS_ERR("disable ps fail: %ld\n", err);
					goto err_out;
				}


				clear_bit(CMC_BIT_PS, &ap3xx6_obj->enable);

			}
			break;
#endif

	case ALSPS_GET_PS_MODE:

			enable = test_bit(CMC_BIT_PS, &ap3xx6_obj->enable) ? (1) : (0);

			if (copy_to_user(ptr, &enable, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

	case ALSPS_GET_PS_DATA:
			err = ap3xx6_read_ps(ap3xx6_obj->client, &ap3xx6_obj->ps);
			if (err != 0) {
				goto err_out;
			}

			dat = ap3xx6_get_ps_value(ap3xx6_obj, ap3xx6_obj->ps);
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

	case ALSPS_GET_PS_RAW_DATA:
			err = ap3xx6_read_ps(ap3xx6_obj->client, &ap3xx6_obj->ps);
			if (err != 0) {
				goto err_out;
			}

			dat = ap3xx6_obj->ps;
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

	case ALSPS_SET_ALS_MODE:
			if (copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			if (enable) {
				err = ap3xx6_enable_als(ap3xx6_obj->client, 1);
				if (err != 0) {
					APS_ERR("enable als fail: %ld\n", err);
					goto err_out;
				}

				set_bit(CMC_BIT_ALS, &ap3xx6_obj->enable);

			} else{
				err = ap3xx6_enable_als(ap3xx6_obj->client, 0);
				if (err != 0) {
					APS_ERR("disable als fail: %ld\n", err);
					goto err_out;
				}

				clear_bit(CMC_BIT_ALS, &ap3xx6_obj->enable);

			}
			break;

	case ALSPS_GET_ALS_MODE:

			enable = test_bit(CMC_BIT_ALS, &ap3xx6_obj->enable) ? (1) : (0);

			if (copy_to_user(ptr, &enable, sizeof(enable)))	{
				err = -EFAULT;
				goto err_out;
			}
			break;

	case ALSPS_GET_ALS_DATA:
			err = ap3xx6_read_als(ap3xx6_obj->client, &ap3xx6_obj->als);
			if (err != 0) {
				goto err_out;
			}

			dat = ap3xx6_obj->als;//ap3xx6_get_als_value(ap3xx6_obj, ap3xx6_obj->als);
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

	case ALSPS_GET_ALS_RAW_DATA:
			err = ap3xx6_read_als(ap3xx6_obj->client, &ap3xx6_obj->als);
			if (err != 0) {
				goto err_out;
			}

			dat = ap3xx6_obj->als;
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

	default:
			APS_ERR("%s not supported = 0x%04x", __func__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

err_out:
	return err;
}
#if 0
#ifdef CONFIG_COMPAT
static long ap3xx6_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_ALSPS_SET_PS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_MODE, (unsigned long)arg32);
		if (err) {
			APS_ERR("ALSPS_SET_PS_MODE unlocked_ioctl failed.");
		}

		break;

	case COMPAT_ALSPS_GET_PS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_MODE, (unsigned long)arg32);
		if (err) {
			APS_ERR("ALSPS_GET_PS_MODE unlocked_ioctl failed.");
		}

		break;

	case COMPAT_ALSPS_GET_PS_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_DATA, (unsigned long)arg32);
		if (err) {
			APS_ERR("ALSPS_GET_PS_DATA unlocked_ioctl failed.");
		}

		break;

	case COMPAT_ALSPS_GET_PS_RAW_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_RAW_DATA, (unsigned long)arg32);
		if (err) {
			APS_ERR("ALSPS_GET_PS_RAW_DATA unlocked_ioctl failed.");
		}

		break;

	case COMPAT_ALSPS_SET_ALS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_SET_ALS_MODE, (unsigned long)arg32);
		if (err) {
			APS_ERR("ALSPS_SET_ALS_MODE unlocked_ioctl failed.");
		}

		break;

	case COMPAT_ALSPS_GET_ALS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_MODE, (unsigned long)arg32);
		if (err) {
			APS_ERR("ALSPS_GET_ALS_MODE unlocked_ioctl failed.");
		}

		break;

	case COMPAT_ALSPS_GET_ALS_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_DATA, (unsigned long)arg32);
		if (err) {
			APS_ERR("ALSPS_GET_ALS_DATA unlocked_ioctl failed.");
		}

		break;

	case COMPAT_ALSPS_GET_ALS_RAW_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_RAW_DATA, (unsigned long)arg32);
		if (err) {
			APS_ERR("ALSPS_GET_ALS_RAW_DATA unlocked_ioctl failed.");
		}

		break;

	default:
		APS_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;

	}
	return err;
}
#endif
#endif
/*----------------------------------------------------------------------------*/
static const struct file_operations ap3xx6_fops = {
/* .owner = THIS_MODULE, */
	.open = ap3xx6_open,
	.release = ap3xx6_release,
	.unlocked_ioctl = ap3xx6_ioctl,
#if 0
#ifdef CONFIG_COMPAT
	.compat_ioctl = ap3xx6_compat_ioctl,
#endif
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ap3xx6_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ap3xx6_fops,
};
/*----------------------------------------------------------------------------*/
static int ap3xx6_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ap3xx6_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ap3xx6_early_suspend(struct early_suspend *h)
{
//	struct ap3xx6_priv *obj = container_of(h, struct ap3xx6_priv, early_drv);
	int err;
	APS_FUN();

	if (!ap3xx6_obj) {
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&ap3xx6_obj->als_suspend, 1);

	if (test_bit(CMC_BIT_ALS, &ap3xx6_obj->enable)) {
		err = ap3xx6_enable_als(ap3xx6_obj->client, 0);
		if (err != 0) {
			APS_ERR("disable als fail: %d\n", err);
		}
	}

}
/*----------------------------------------------------------------------------*/
static void ap3xx6_late_resume(struct early_suspend *h)
{
//	struct ap3xx6_priv *obj = container_of(h, struct ap3xx6_priv, early_drv);
	int err;
	APS_FUN();

	if (!ap3xx6_obj) {
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&ap3xx6_obj->als_suspend, 0);
	if (test_bit(CMC_BIT_ALS, &ap3xx6_obj->enable)) {
		err = ap3xx6_enable_als(ap3xx6_obj->client, 1);
		if (err != 0) {
			APS_ERR("enable als fail: %d\n", err);

		}
	}

}
#endif

static int ap3xx6_als_open_report_data(int open)
{

	return 0;
}

static int ap3xx6_als_enable_nodata(int en)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	int value = 0;
	int err = 0;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}

	value = en;
	if (value) {
		err = ap3xx6_enable_als(ap3xx6_obj->client, 1);
		if (err != 0) {
			APS_ERR("enable als fail: %d\n", err);
			return -1;
		}
		set_bit(CMC_BIT_ALS, &ap3xx6_obj->enable);
	} else{
		err = ap3xx6_enable_als(ap3xx6_obj->client, 0);
		if (err != 0) {
			APS_ERR("disable als fail: %d\n", err);
			return -1;
		}
		clear_bit(CMC_BIT_ALS, &ap3xx6_obj->enable);
	}
	return 0;
}

static int ap3xx6_als_set_delay(u64 ns)
{
	return 0;
}

static int ap3xx6_als_get_data(int *value, int *status)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
//	static int temp_als;
	int return_lux = 0;
	int err = 0;
//	u16 b[2];
//	int i;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}

	ap3xx6_read_als(ap3xx6_obj->client, &ap3xx6_obj->als);
	if(g_temp_als_lux<0){
		g_temp_als_lux = ap3xx6_obj->als;
		if(ap3xx6_openlog) APS_LOG("alp.D : set temp for init!!\n");
	}

	if (ap3xx6_obj->als < 10) {
		return_lux = 0;
	} else{
		if(ap3xx6_obj->als >= g_als_lux_up_value){
			g_als_lux_up_count ++ ;
			if(ap3xx6_openlog) APS_LOG("alp.D : up counter +1 => %d\n", g_als_lux_up_count);
		}else if(ap3xx6_obj->als <= g_als_lux_down_value){
			g_als_lux_down_count ++ ;
			if(ap3xx6_openlog) APS_LOG("alp.D : down counter +1 => %d\n", g_als_lux_down_count);
		}else{
			if(ap3xx6_openlog) APS_LOG("alp.D : counter to zero !!\n");
			g_als_lux_up_count = 0;
			g_als_lux_down_count = 0;
		}
		if( (g_als_lux_up_count>=LIGHT_STABLE_COUNTER)||(g_als_lux_down_count>=LIGHT_STABLE_COUNTER) ){
			g_als_lux_up_count = 0;
			g_als_lux_down_count = 0;
			return_lux = ap3xx6_obj->als;
			g_als_lux_up_value = return_lux * (10+LIGHT_STABLE_LIMIT)/10;
			g_als_lux_down_value = return_lux * (10-LIGHT_STABLE_LIMIT)/10;
			g_temp_als_lux = ap3xx6_obj->als;//ap3xx6_get_als_value(ap3xx6_obj, ap3xx6_obj->als);
			if(ap3xx6_openlog) APS_LOG("alp.D : update return_lux=%d\n", return_lux);
			if(ap3xx6_openlog) APS_LOG("alp.D : limit_up_lux=%d , limit_down_lux=%d \n", g_als_lux_up_value, g_als_lux_down_value);
			if(ap3xx6_openlog) APS_LOG("alp.D : g_temp_als_lux=%d\n", g_temp_als_lux);
		}else{
			return_lux = g_temp_als_lux;
			if(ap3xx6_openlog) APS_LOG("alp.D : fixed_lux=%d\n", return_lux);
		}
	}

	*value = return_lux;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return err;
}

//static int ap3xx6_ps_open_report_data(int open)
//{
//	return 0;
//}

//static int ap3xx6_ps_enable_nodata(int en)
//{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
//	int value = 0;
//	int err = 0;
//	int ps_value = -1;
//	int ps_status = 0;
//
//	if (ap3xx6_obj == NULL) {
//		APS_ERR("ap3xx6_obj is null\n");
//		return -1;
//	}
//	value = en;
//	if (value) {
//		err = ap3xx6_enable_ps(ap3xx6_obj->client, 1);
//		if (err != 0) {
//			APS_ERR("enable ps fail: %d\n", err);
//			return -1;
//		}
//		msleep(100);
//		set_bit(CMC_BIT_PS, &ap3xx6_obj->enable);
//
//#ifdef DI_AUTO_CAL
//		ap3xx6_Calibration(ap3xx6_obj->client);
//#endif
//		err = ap3xx6_ps_get_data(&ps_value, &ps_status);
//		if ((err == 0) && (ps_value > 0)) {
//			ps_report_interrupt_data(ps_value);
//		}
//	} else{
//		err = ap3xx6_enable_ps(ap3xx6_obj->client, 0);
//		if (err != 0) {
//			APS_ERR("disable ps fail: %d\n", err);
//			return -1;
//		}
//
//		clear_bit(CMC_BIT_PS, &ap3xx6_obj->enable);
//
//	}
//
//	return 0;
//}

//static int ap3xx6_ps_set_delay(u64 ns)
//{
//	return 0;
//}

#ifdef SENSOR_PS
static int ap3xx6_ps_get_data(int *value, int *status)
{
//	struct ap3xx6_priv *obj = ap3xx6_obj;
	int err = 0;

	if (ap3xx6_obj == NULL) {
		APS_ERR("ap3xx6_obj is null\n");
		return -1;
	}

	err = ap3xx6_read_ps(ap3xx6_obj->client, &ap3xx6_obj->ps);
	if (err != 0) {
		APS_ERR("ap3xx6_read_ps failed err=%d\n", err);
		*value = -1;
		return err;
	}
	*value = ap3xx6_get_ps_value(ap3xx6_obj, ap3xx6_obj->ps);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
#endif
/*
static int ap3425h_input_init(void)
{
	int ret;
	// allocate light input_device 
	ap3xx6_obj->input_dev = input_allocate_device();
	if (!ap3xx6_obj->input_dev) {
		APS_ERR("ap3425h could not allocate input device\n");
		goto err_light_all;
	}

	ap3xx6_obj->input_dev->name = AP3425H_INPUT_NAME;
	input_set_capability(ap3xx6_obj->input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(ap3xx6_obj->input_dev, ABS_MISC, 0, 1, 0, 0);

	APS_LOG("ap3425h registering light sensor input device\n");
	ret = input_register_device(ap3xx6_obj->input_dev);
	if (ret < 0) {
		APS_ERR("could not register input device\n");
		goto err_light_reg;
	}
	return 0;

err_light_reg:
	input_free_device(ap3xx6_obj->input_dev);
err_light_all:
	return (-1);   
}*/

static int of_get_ap3425h_platform_data(struct device *dev)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,ap3425h");
	if (node) {
		alsps_int_gpio_number = of_get_named_gpio(node, "int-gpio", 0);
		alsps_irq = irq_of_parse_and_map(node, 0);
		if (alsps_irq < 0) {
			APS_ERR("alsps request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		APS_LOG("alsps_int_gpio_number %d; alsps_irq : %d\n", alsps_int_gpio_number, alsps_irq);
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ap3xx6_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
//	struct ap3xx6_priv *obj = NULL;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
#ifdef SENSOR_PS
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};
#endif
	int err = 0;
	u8 int_stat = 0;

	APS_FUN();
	ap3xx6_obj = kzalloc(sizeof(*ap3xx6_obj), GFP_KERNEL);
	if (ap3xx6_obj == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	memset(ap3xx6_obj, 0, sizeof(*ap3xx6_obj));
//	ap3xx6_obj = obj;

	mutex_init(&ap3xx6_obj->lock);

	ap3xx6_obj->hw = &cust_alsps_hw;

	ap3xx6_get_addr(ap3xx6_obj->hw, &ap3xx6_obj->addr);

#if DELAYED_WORK
	INIT_DELAYED_WORK(&ap3xx6_obj->eint_work, ap3xx6_eint_work);
#else
	INIT_WORK(&ap3xx6_obj->eint_work, ap3xx6_eint_work);
#endif
	ap3xx6_obj->client = client;
	i2c_set_clientdata(client, ap3xx6_obj);
	atomic_set(&ap3xx6_obj->als_debounce, 300);
	atomic_set(&ap3xx6_obj->als_deb_on, 0);
	atomic_set(&ap3xx6_obj->als_deb_end, 0);
	atomic_set(&ap3xx6_obj->ps_debounce, 300);
	atomic_set(&ap3xx6_obj->ps_deb_on, 0);
	atomic_set(&ap3xx6_obj->ps_deb_end, 0);
	atomic_set(&ap3xx6_obj->ps_mask, 0);
	atomic_set(&ap3xx6_obj->als_suspend, 0);
	atomic_set(&ap3xx6_obj->als_cmd_val, 0xDF);
	atomic_set(&ap3xx6_obj->ps_cmd_val, 0xC1);
	atomic_set(&ap3xx6_obj->ps_thd_val_h, ap3xx6_obj->hw->ps_threshold_high);
	atomic_set(&ap3xx6_obj->ps_thd_val_l, ap3xx6_obj->hw->ps_threshold_low);
	atomic_set(&ap3xx6_obj->trace, 0);

	ap3xx6_obj->enable = 0;
	ap3xx6_obj->pending_intr = 0;
	ap3xx6_obj->als_level_num = sizeof(ap3xx6_obj->hw->als_level)/sizeof(ap3xx6_obj->hw->als_level[0]);
	ap3xx6_obj->als_value_num = sizeof(ap3xx6_obj->hw->als_value)/sizeof(ap3xx6_obj->hw->als_value[0]);
	ap3xx6_obj->als_modulus = (400*100*40)/(1*1500);

	BUG_ON(sizeof(ap3xx6_obj->als_level) != sizeof(ap3xx6_obj->hw->als_level));
	memcpy(ap3xx6_obj->als_level, ap3xx6_obj->hw->als_level, sizeof(ap3xx6_obj->als_level));
	BUG_ON(sizeof(ap3xx6_obj->als_value) != sizeof(ap3xx6_obj->hw->als_value));
	memcpy(ap3xx6_obj->als_value, ap3xx6_obj->hw->als_value, sizeof(ap3xx6_obj->als_value));
	atomic_set(&ap3xx6_obj->i2c_retry, 3);
	
	APS_LOG("ap3xx6 probe client->addr = %x\n", client->addr);

	ap3xx6_i2c_client = client;
	err = ap3xx6_init_client(client);
	if (err != 0) {
		APS_ERR("ap3xx6_init_client() ERROR!\n");
		goto exit_init_failed;
	}
	APS_LOG("ap3xx6_init_client() OK!\n");

    /* reset Interrupt pin */
    int_stat = ap3425_get_intstat(client);
    APS_LOG("Interrupt Status [0x%2X]\n", int_stat);

	of_get_ap3425h_platform_data(&client->dev);
	gpio_request_one(alsps_int_gpio_number, GPIOF_IN, "alsps_int");

#ifdef DI_AUTO_CAL
	#ifdef SENSOR_PS
	ap3xx6_enable_ps(client, 1);
	msleep(100);
	ap3xx6_Calibration(client);
	ap3xx6_enable_ps(client, 0);
	#endif
#endif

	err = misc_register(&ap3xx6_device);
	if (err != 0) {
		APS_ERR("ap3xx6_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = ap3xx6_create_attr(&ap3xx6_init_info.platform_diver_addr->driver);
	if (err != 0) {
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	// init input subsystem	
//	APS_LOG("ap3425h_input_init start!!\n");
//	err = ap3425h_input_init();
//	if (err) {
//		APS_ERR("ap3425h_input_init failed.\n");
//		goto exit_create_attr_failed;
//	}

	// sys init ++
	err = 0;
	android_light_kobj = kobject_create_and_add("android_lsensor", NULL);	
	err += sysfs_create_file(android_light_kobj, &dev_attr_enable.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_delay.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_gain.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_lux.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_als_thres_low.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_als_thres_high.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_pin_status.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_control_pin.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_calibration.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_cal_threshold_low.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_cal_threshold_high.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_factory_cal.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_state.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_atd_calials.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_openlog.attr);
	if (err != 0) {
		APS_ERR("create light sensor attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}	
	err = sysfs_create_group(&client->dev.kobj, &ap3425h_attribute_group);
	// sys init --

#ifdef AP3425H_DELAY_CALIBRATION
	atomic_set(&ap3xx6_obj->delaycalibration, AP3425H_DELAY_CALITIME);  // 12000 ms
	INIT_DELAYED_WORK(&ap3xx6_obj->delayworkcalibration, ap3425h_delay_calibration_func);
#endif

#ifdef SENSOR_PS
	if (1 == ap3xx6_obj->hw->polling_mode_ps) {
		ps_ctl.is_report_input_direct = false;
		ps_ctl.is_polling_mode = true;
		wake_lock_init(&chrg_lock, WAKE_LOCK_SUSPEND, "ap3xx6_wake_lock");
	} else{
		ps_ctl.is_report_input_direct = true;
		ps_ctl.is_polling_mode = false;
	}
#endif

	err = ap3xx6_setup_eint(client);
	if (err != 0) {
		APS_ERR("setup eint: %d\n", err);
		goto exit_create_attr_failed;
	}

#ifdef SENSOR_PS
	ps_ctl.open_report_data = ap3xx6_ps_open_report_data;
	ps_ctl.enable_nodata = ap3xx6_ps_enable_nodata;
	ps_ctl.set_delay = ap3xx6_ps_set_delay;
	ps_ctl.is_support_batch = ap3xx6_obj->hw->is_batch_supported_ps;
	ps_ctl.is_use_common_factory = false;

	err = ps_register_control_path(&ps_ctl);
	if (err != 0) {
		APS_ERR("ps_register_control_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_data.get_data = ap3xx6_ps_get_data;
	ps_data.vender_div = 1;

	err = ps_register_data_path(&ps_data);
	if (err != 0) {
		APS_ERR("ps_register_data_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}
#endif

	als_ctl.open_report_data = ap3xx6_als_open_report_data;
	als_ctl.enable_nodata = ap3xx6_als_enable_nodata;
	als_ctl.set_delay = ap3xx6_als_set_delay;
	als_ctl.is_support_batch = ap3xx6_obj->hw->is_batch_supported_als;
	als_ctl.is_use_common_factory = false;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_polling_mode = false;

	err = als_register_control_path(&als_ctl);
	if (err != 0) {
		APS_ERR("als_register_control_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_data.get_data = ap3xx6_als_get_data;
	als_data.vender_div = 1;

	err = als_register_data_path(&als_data);

	if (err != 0) {
		APS_ERR("als_register_data_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	APS_LOG("als_register_data_path OK.%s:\n", __func__);

	ap3425h_cali.cali_state=0;
	ap3425h_cali.do_calibrating = 0;
	ap3425h_cali.als_cali_base=1000;
	ap3425h_cali.als_cali_adc=AP3425H_GOLDEN_VALUE_Z380M;
	ap3425h_cali.cal_threshold_low=50;
	ap3425h_cali.cal_threshold_high=700;

	atomic_set(&ap3xx6_obj->delay, 200);

#ifdef AP3425H_DELAY_CALIBRATION
	schedule_delayed_work(&ap3xx6_obj->delayworkcalibration, msecs_to_jiffies(atomic_read(&ap3xx6_obj->delaycalibration)));
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	ap3xx6_obj->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	ap3xx6_obj->early_drv.suspend = ap3xx6_early_suspend,
	ap3xx6_obj->early_drv.resume	= ap3xx6_late_resume,
	register_early_suspend(&ap3xx6_obj->early_drv);
#endif

	ap3xx6_init_flag = 1;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&ap3xx6_device);
exit_misc_device_register_failed:
exit_init_failed:
	mutex_destroy(&ap3xx6_obj->lock);
//exit_kfree:
	kfree(ap3xx6_obj);
exit:
	ap3xx6_i2c_client = NULL;
	ap3xx6_init_flag = -1;
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int ap3xx6_i2c_remove(struct i2c_client *client)
{
	int err;

	ap3xx6_delete_attr(&ap3xx6_init_info.platform_diver_addr->driver);
	err = misc_deregister(&ap3xx6_device);
	if (err != 0) {
		APS_ERR("misc_deregister fail: %d\n", err);
	}

	if (1 == ap3xx6_obj->hw->polling_mode_ps) {
		wake_lock_destroy(&chrg_lock);
	}

	ap3xx6_i2c_client = NULL;
	i2c_unregister_device(client);
	mutex_destroy(&ap3xx6_obj->lock);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int ap3xx6_local_init(void)
{
//	struct alsps_hw *hw = &cust_alsps_hw;
//	ap3xx6_power(hw, 1);
	if (i2c_add_driver(&ap3xx6_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
	if (-1 == ap3xx6_init_flag) {
		APS_ERR("add driver--ap3xx6_init_flag check error\n");
		return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int ap3xx6_remove(void)
{
//	struct alsps_hw *hw = &cust_alsps_hw;
	APS_FUN();
//	ap3xx6_power(hw, 0);
	i2c_del_driver(&ap3xx6_i2c_driver);
	ap3xx6_init_flag = -1;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init ap3xx6_init(void)
{
	struct alsps_hw *hw = &cust_alsps_hw;
	APS_FUN();
	APS_LOG("%s: i2c_number=%d\n", __func__, hw->i2c_num);

	alsps_driver_add(&ap3xx6_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ap3xx6_exit(void)
{
	APS_FUN();
}


module_init(ap3xx6_init);
module_exit(ap3xx6_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("YC Hou");
MODULE_DESCRIPTION("ap3xx6 driver");
MODULE_LICENSE("GPL");
