
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
#include "als3320a.h"
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

//#define ALS3320A_NUM_CACHABLE_REGS	12
//static u8 *als3320a_reg[ALS3320A_NUM_CACHABLE_REGS]={0x01, 0x02, 0x06, 0x0C, 0x0D, 0x10, 0x01, 0x14, 0x1A, 0x1B, 0x1C, 0x1D};
//static char *als3320a_reg_name[ALS3320A_NUM_CACHABLE_REGS]={"SYS_CONFIG","INTERRUPT_FLAG","INT_CONTROL","WAITING","ALS_DATA_L","ALS_DATA_H","GAIN","Persistence","ALS_THRES_LOW_L","ALS_THRES_LOW_H","ALS_THRES_HIGH_L","ALS_THRES_HIGH_H"};

/*---------------------------user define-------------------------------------------------*/

#define DELAYED_WORK	0	// default -->0 ; test -->1
#define als3320a

#define ALS3320A_DEV_NAME		"als3320a"

#define ALS3320A_INPUT_NAME	"als3320a_input"

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id als3320a_i2c_id[] = {{ALS3320A_DEV_NAME, 0}, {} };
static int of_get_als3320a_platform_data(struct device *dev);
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
#define ALS3320A_DELAY_CALITIME		20000
#define ALS3320A_DELAY_RETRYCALITIME	3000
#define ALS3320A_DELAY_CALIBRATION	1

#define ALS3320A_CALI_SAMPLE_TIME		3
#define ALS3320A_CALI_FILE_LENGTH		16

#define ALS3320A_USE_GAIN				0x03
#define ALS3320A_USE_RESOLUTION		38	// 0.38 , so need to div 100

#define ALS3320A_GOLDEN_VALUE_Z300M	480

/*----------------------------------------------------------------------------*/
static int als3320a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int als3320a_i2c_remove(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static int als3320a_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int als3320a_i2c_resume(struct i2c_client *client);
static int als3320a_als_get_data(int *value, int *status);
static int als3320a_input_init(void);

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
struct als3320a_i2c_addr {	/*define a series of i2c slave address*/
	u8 write_addr;
};
/*----------------------------------------------------------------------------*/
struct als3320a_priv {
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
	struct als3320a_i2c_addr addr;
	struct device_node *irq_node;
#ifdef ALS3320A_DELAY_CALIBRATION
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
	atomic_t	trace;

	/*data*/
	u16		als;
	u8		_align;
	u16		als_level_num;
	u16		als_value_num;
	u32		als_level[C_CUST_ALS_LEVEL-1];
	u32		als_value[C_CUST_ALS_LEVEL];
	atomic_t delay;

	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/

	ulong		enable;			/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/

	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
#endif
};

struct als_cali {
	int als_cali_base;
	int als_cali_lux;
	int cali_state;
	int do_calibrating;
};

static struct als_cali als3320a_cali;

static struct i2c_client *als3320a_i2c_client;
static struct als3320a_priv *als3320a_obj;


/* Add  for Interrupt mode */
#define ALS3320A_TABLE_LENGTH		20
static uint16_t defalut_table[ALS3320A_TABLE_LENGTH] = {0x32, 0x64, 0xC8, 0x12C, 0x190, 0x1F4, 0x28A, 0x320, 0x3E8, 0x5DC, 0x7D0, 0xBB8, 0xFA0, 0x1388, 0x1B58, 0x2710, 0x30D4, 0x3A98, 0x445C, 0x4E20};
//static uint16_t cali_table[ALS3320A_TABLE_LENGTH] = {0x0C, 0x19, 0x32, 0x4B, 0x64, 0x7D, 0xA2, 0xC8, 0xFA, 0x177, 0x1F4, 0x2EE, 0x3E8, 0x4E2, 0x6D6, 0x9C4, 0xC35, 0xEA6, 0x1117, 0x1388};
static uint16_t cali_table[ALS3320A_TABLE_LENGTH] = {0x32, 0x64, 0xC8, 0x12C, 0x190, 0x1F4, 0x28A, 0x320, 0x3E8, 0x5DC, 0x7D0, 0xBB8, 0xFA0, 0x1388, 0x1B58, 0x2710, 0x30D4, 0x3A98, 0x445C, 0x4E20};

/*----------------------------------------------------------------------------*/
static const struct of_device_id als_of_match[] = {
	{.compatible = "mediatek,als3320a"},
	{},
};

static struct i2c_driver als3320a_i2c_driver = {
	.probe		= als3320a_i2c_probe,
	.remove	= als3320a_i2c_remove,
	.suspend	= als3320a_i2c_suspend,
	.resume	= als3320a_i2c_resume,
	.id_table	= als3320a_i2c_id,
	.driver = {
/* .owner			= THIS_MODULE, */
		.name			= ALS3320A_DEV_NAME,
		.of_match_table = als_of_match,
	},
};

static int als3320a_local_init(void);
static int als3320a_remove(void);

static int als3320a_init_flag = -1; // 0<==>OK -1 <==> fail 
static int als3320a_openlog = 0; // 0<==>close ;  1<==>open
unsigned int alsps_int_gpio_number = 0;
static unsigned int alsps_irq;

static struct alsps_init_info als3320a_init_info = {
	.name = ALS3320A_DEV_NAME,
	.init = als3320a_local_init,
	.uninit = als3320a_remove,
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

static int als3320a_write_to_califile(int base,int calilux);
#ifdef ALS3320A_DELAY_CALIBRATION
static int als3320a_read_from_califile(void);
#endif
static int als3320a_init_client(struct i2c_client *client);
mm_segment_t old_als_fs;


#ifdef ALS3320A_DELAY_CALIBRATION
static void als3320a_delay_calibration_func(struct work_struct *work)
{
//	unsigned long delay = msecs_to_jiffies(ALS3320A_DELAY_RETRYCALITIME);
	int status=0;
	printk("alp.D : als3320a_delay_calibration_func !!\n");
	status = als3320a_read_from_califile();
	if(status<0){
		printk("alp.D : read calibration fail , need to Retry !!\n");
//		schedule_delayed_work(&light_sensor_data->delayworkcalibration, delay);
		return ;
	}		
}
#endif

static int als3320a_read_reg(struct i2c_client *client,
		char reg, u8 mask, u8 shift)
{
	int ret = 0;
	char tmp[1];
	tmp[0] = reg;
	mutex_lock(&als3320a_obj->lock);

	ret = i2c_master_send(client, tmp, 0x01);
	if (ret <= 0) {
		APS_ERR("als3320a_read_reg 1 ret=%x\n", ret);
		goto EXIT_ERR;
	}
	ret = i2c_master_recv(client, tmp, 0x01);
	if (ret <= 0) {
		APS_ERR("als3320a_read_reg 2 ret=%d\n", ret);
		goto EXIT_ERR;
	}

	mutex_unlock(&als3320a_obj->lock);
	return (tmp[0] & mask) >> shift;

EXIT_ERR:
		APS_ERR("als3320a_read_reg fail\n");
	mutex_unlock(&als3320a_obj->lock);
		return ret;
}

static int als3320a_write_reg(struct i2c_client *client,
		char reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0x00;
	char tmp[2];

	mutex_lock(&als3320a_obj->lock);

	tmp[0] = reg;
	tmp[1] = val;
	ret = i2c_master_send(client, tmp, 0x02);
	if (ret <= 0) {
		APS_ERR("als3320a_write_reg ret=%d\n", ret);
		goto EXIT_ERR;
	}

	mutex_unlock(&als3320a_obj->lock);
	return ret;

EXIT_ERR:
		APS_ERR("als3320a_write_reg fail\n");
	mutex_unlock(&als3320a_obj->lock);
		return ret;
}
/*----------------------------------------------------------------------------*/
int als3320a_get_addr(struct alsps_hw *hw, struct als3320a_i2c_addr *addr)
{
	if (!hw || !addr) {
		return -EFAULT;
	}
	addr->write_addr = hw->i2c_addr[0];
	return 0;
}

/*----------------------------------------------------------------------------*/
static int als3320a_enable_als(struct i2c_client *client, int enable)
{
//		struct als3320a_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];
		int res = 0;
		u8 buffer[1];
		int reg_value[1];

		if (client == NULL) {
			APS_DBG("CLIENT CANN'T EQUAL NULL\n");
			return -1;
		}

		buffer[0] = ALS3320A_ENABLE;
		reg_value[0] = als3320a_read_reg(client, buffer[0], 0xFF, 0x00);
		if (res < 0) {
			goto EXIT_ERR;
		}

		if (enable) {			
			databuf[0] = ALS3320A_ENABLE;
			databuf[1] = 0x01;
			res = als3320a_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
			if (res <= 0) {
				goto EXIT_ERR;
			}
			atomic_set(&als3320a_obj->als_deb_on, 1);
			atomic_set(&als3320a_obj->als_deb_end, jiffies+atomic_read(&als3320a_obj->als_debounce)/(1000/HZ));
			APS_DBG("als3320a_ ALS enable\n");
#if DELAYED_WORK
			schedule_delayed_work(&als3320a_obj->eint_work, 1100*HZ/1000);
#endif
		} else{
//			databuf[0] = ALS3320A_ENABLE;
//			databuf[1] = 0;
//			res = als3320a_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
//			if (res <= 0) {
//				goto EXIT_ERR;
//			}
			als3320a_init_client(client);

			atomic_set(&als3320a_obj->als_deb_on, 0);
#if DELAYED_WORK
			cancel_delayed_work_sync(&als3320a_obj->eint_work);
#endif

			APS_DBG("als3320a_ ALS disable\n");
		}
		return 0;

EXIT_ERR:
		APS_ERR("als3320a__enable_als fail\n");
		return res;
}

//for als
static int als3320a_set_ALSWaiting(struct i2c_client *client, int val)
{
	int err;
	err = als3320a_write_reg(client, ALS3320A_ALS_WAITING, 0xFF, 0x00, val);
	return err;
}

static int als3320a_set_ALSGain(struct i2c_client *client, int val)
{
	int err;
	err = als3320a_write_reg(client, ALS3320A_GAIN, 0xFF, 0x00, val);
	return err;
}

static int als3320a_set_ALSPersist(struct i2c_client *client, int val)
{
	int err;
	err = als3320a_write_reg(client, ALS3320A_ALS_PERSIST, 0xFF, 0x00, val);
	return err;
}

static int als3320a_set_ALSMean(struct i2c_client *client, int val)
{
	int err;
	err = als3320a_write_reg(client, ALS3320A_ALS_MEAN_TIME, 0xFF, 0x00, val);
	return err;
}

// ALS low threshold
static int als3320a_set_althres(struct i2c_client *client, int val)
{
//	int lsb, msb;
	int ret=0;
	u8 databuf_L, databuf_H;
	databuf_H = val >> 8;
	databuf_L = val & 0xFF;

	if(als3320a_openlog) APS_LOG(" set L threshold val = %d !!\n",val);
	ret += als3320a_write_reg(als3320a_obj->client, 0x30, 0xFF, 0x00, databuf_L);
	ret += als3320a_write_reg(als3320a_obj->client, 0x31, 0xFF, 0x00, databuf_H);
	if(als3320a_openlog) APS_LOG(" set L threshold ret = %d !!\n",ret);
	if(als3320a_openlog) APS_LOG("als3320a_set_althres  msb=%d ; lsb=%d\n",databuf_H ,databuf_L);
	return ret;
}

// ALS high threshold
static int als3320s_set_ahthres(struct i2c_client *client, int val)
{
//	int lsb, msb;
	int ret=0;
	u8 databuf_L, databuf_H;
	databuf_H = val >> 8;
	databuf_L = val & 0xFF;

	if(als3320a_openlog) APS_LOG(" set H threshold val = %d !!\n",val);
	ret += als3320a_write_reg(als3320a_obj->client, 0x32, 0xFF, 0x00, databuf_L);
	ret += als3320a_write_reg(als3320a_obj->client, 0x33, 0xFF, 0x00, databuf_H);
	if(als3320a_openlog) APS_LOG(" set H threshold ret = %d !!\n",ret);
	if(als3320a_openlog) APS_LOG("als3320s_set_ahthres  msb=%d ; lsb=%d\n",databuf_H ,databuf_L);
	return ret;
}

/*----------------------------------------------------------------------------*/

void als3320a_eint_func(void)
{
//	struct als3320a_priv *obj = als3320a_obj;
	APS_FUN();
	if (unlikely(als3320a_obj == NULL)) {
		APS_ERR("%s--%d als3320a_obj is NULL!\n", __func__, __LINE__);
		return;
	}

	if (atomic_read(&als3320a_obj->trace) & TRACE_DEBUG) {
		if(als3320a_openlog) APS_LOG("%s--%d\n", __func__, __LINE__);
	}

	enable_irq(alsps_irq);
}

static irqreturn_t als3320a_eint_handler(int irq, void *desc)
{
//	struct als3320a_priv *obj = als3320a_obj;
	APS_FUN();
	if (unlikely(als3320a_obj == NULL)) {
		APS_ERR("%s--%d als3320a_obj is NULL!\n", __func__, __LINE__);
		return IRQ_HANDLED;
	}

	if (atomic_read(&als3320a_obj->trace) & TRACE_DEBUG) {
		APS_LOG("%s--%d\n", __func__, __LINE__);
	}

	disable_irq_nosync(alsps_irq);
	als3320a_eint_func();

	return IRQ_HANDLED;
}


/*----------------------------------------------------------------------------*/
/* This function depends the real hw setting, customers should modify it. 2012/5/10 YC. */
int als3320a_setup_eint(struct i2c_client *client)
{
	int err = 0;
	gpio_direction_input(alsps_int_gpio_number);
	APS_LOG("als3320a probe alsps_irq = %d\n", alsps_irq);
	err = request_irq(alsps_irq, als3320a_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL);
	if (err != 0) {
		APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
		return -EINVAL;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int als3320a_init_client(struct i2c_client *client)
{
//	struct als3320a_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res=0, ret=0;

	databuf[0] = ALS3320A_ENABLE;
	databuf[1] = 0x00;
	res = als3320a_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	// do software-reset
	databuf[1] = 0x04;		
	res = als3320a_write_reg(client, databuf[0], 0xFF, 0x00, databuf[1]);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	ret = als3320a_read_reg(als3320a_obj->client, ALS3320A_CONTROL_INT, 0xFF, 0x00);
	APS_LOG("als3320a init Configuration = %d\n", ret);

	als3320a_set_ALSWaiting(client, 0x01);
	als3320a_set_ALSGain(client, ALS3320A_USE_GAIN);
	als3320a_set_ALSPersist(client, 0x03);
	als3320a_set_ALSMean(client, 0x02);

	als3320a_set_althres(client, 0xFFF);
	als3320s_set_ahthres(client, 0x00); 

	return ALS3320A_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
int als3320a_read_als(struct i2c_client *client, u16 *data)
{
	/*struct als3320a_priv *obj = i2c_get_clientdata(client);	*/
	u8 als_value_low[1], als_value_high[1];
	int value=0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	/* get ALS adc count */
	als_value_low[0] = als3320a_read_reg(client, ALS3320A_ADATA_L, 0xFF, 0x00);
	if (als_value_low[0] < 0) {
		goto EXIT_ERR;
	}

	als_value_high[0] = als3320a_read_reg(client, ALS3320A_ADATA_H, 0xFF, 0x00);
	if (als_value_high[0] < 0) {
		goto EXIT_ERR;
	}

	value = als_value_low[0] | (als_value_high[0]<<8);
	if(als3320a_openlog) APS_LOG("als3320a_read_als  adc=%d\n", value);
	if (value < 0) {
		*data = 0;
		APS_DBG("als_value is invalid!!\n");
		goto EXIT_ERR;
	}

	if(als3320a_cali.do_calibrating==0){
		*data = (u16) value * (als3320a_cali.als_cali_base)/als3320a_cali.als_cali_lux;
		if(als3320a_openlog) APS_LOG("calibration  adc=%d\n", *data);
	}else{
		*data = (u16) value ;
		if(als3320a_openlog) APS_LOG("orig-adc=%d\n", *data);
	}
//		*data = (*data / 8)*8;
	return 0;

EXIT_ERR:
	APS_ERR("als3320a__read_als fail\n");
	return -1;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void als3320a_eint_work(struct work_struct *work)
{
	int err=0;
	int value = -1;
	int status = -1;
#if DELAYED_WORK
#else
	int index = 0;
#endif

	err = als3320a_als_get_data(&value, &status);
	if (err < 0) {
		APS_ERR("als3320a_als_get_data failed\n");
	}else{
		if(als3320a_openlog) APS_LOG("als3320a_eint_work lux = %d\n",value);
	}

#if DELAYED_WORK
	schedule_delayed_work(&als3320a_obj->eint_work, msecs_to_jiffies(atomic_read(&als3320a_obj->delay)));
#else
	for(index = 0 ; index < ALS3320A_TABLE_LENGTH ; index++) {
		if( value <= defalut_table[index] ) {
			als3320s_set_ahthres(als3320a_obj->client, cali_table[index] );
			als3320a_set_althres(als3320a_obj->client, (index ==0) ? 0 : (cali_table[index-1]+1)  );
			break;
		}
	}
	enable_irq(alsps_irq);
#endif

}
/*----------------------------------------------------------------------------*/
static ssize_t als3320a_als_rawdata_show(struct device_driver *ddri, char *buf)
{
//	struct als3320a_priv *obj = als3320a_obj;
	int err = 0;
	u16 als = -1;

	if (als3320a_obj == NULL) {
		APS_ERR("als3320a_obj is null\n");
		return -1;
	}

	err = als3320a_read_als(als3320a_obj->client, &als);
	if (err < 0) {
		APS_ERR("als3320a_read_als failed\n");
		return -1;
	}
	return sprintf(buf, "als raw dat= %d\n", als);
}
static ssize_t als3320a_als_data_show(struct device_driver *ddri, char *buf)
{
//	struct als3320a_priv *obj = als3320a_obj;
	int err = 0;
	int value = -1;
	int status = -1;

	if (als3320a_obj == NULL) {
		APS_ERR("als3320a_obj is null\n");
		return -1;
	}

	err = als3320a_als_get_data(&value, &status);
	if (err < 0) {
		APS_ERR("als3320a_als_get_data failed\n");
		return -1;
	}
	return sprintf(buf, "als dat= %d,,status=%d\n", value, status);
}

static ssize_t als3320a_trace_show(struct device_driver *ddri, char *buf)
{
//	struct als3320a_priv *obj = als3320a_obj;
	int ret = 0;

	if (als3320a_obj == NULL) {
		APS_ERR("als3320a_obj is null\n");
		return -1;
	}
	ret = sprintf(buf, "als3320a_obj->trace = %d,\n", atomic_read(&als3320a_obj->trace));

	return ret;
}
static ssize_t als3320a_trace_store(struct device_driver *ddri, const char *buf, size_t count)
{

	int val = 0;
	int ret = 0;

	if (!als3320a_obj) {
		APS_ERR("als3320a_obj is null!!\n");
		return -1;
	}
	ret = sscanf(buf, "0x%x", &val);
	if (ret == 1) {
		atomic_set(&als3320a_obj->trace, val);
	}

	return count;
}

static ssize_t als3320a_status_show(struct device_driver *ddri, char *buf)
{
//	struct als3320a_priv *obj = als3320a_obj;
	int ret = 0;
//	int value = -1;
//	int status = -1;
	int i = 0;

	if (als3320a_obj == NULL) {
		APS_ERR("als3320a_obj is null\n");
		return -1;
	}
	ret += sprintf(buf+ret, "obj->irq = %d,\n", alsps_irq);
#ifdef CUST_EINT_ALS_NUM
	ret += sprintf(buf+ret, "CUST_EINT_ALS_NUM = %d,\n", CUST_EINT_ALS_NUM);
#endif
	ret += sprintf(buf+ret, "als_level:");
	for (i = 0; i < sizeof(als3320a_obj->hw->als_level)/sizeof(als3320a_obj->hw->als_level[0]); i++) {
		ret += sprintf(buf+ret, "%d, ", als3320a_obj->hw->als_level[i]);
	}
	ret += sprintf(buf+ret, "\n als_value:");
	for (i = 0; i < sizeof(als3320a_obj->hw->als_value)/sizeof(als3320a_obj->hw->als_value[0]); i++) {
		ret += sprintf(buf+ret, "%d, ", als3320a_obj->hw->als_value[i]);
	}
	return ret;
}

static ssize_t als3320a_em_read(struct device_driver *ddri, char *buf)
{
	u16 idx = 0;
	int count = 0;
	int reg_value[1];

	#define ALS3320A_NUM_CACHABLE_REGS	29
	u8 als3320a_reg[ALS3320A_NUM_CACHABLE_REGS] = {
	0x00, 0x01, 0x02, 0x06, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x14, 0x1a, 0x1b, 0x1c, 0x1d,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d};

	if (!als3320a_obj) {
		APS_ERR("als3320a_obj is null!!\n");
		count += sprintf(buf+count, "als3320a_obj is null!!\n");
		return count;
	}
	for (idx = 0; idx < ALS3320A_NUM_CACHABLE_REGS; idx++) {

			reg_value[0] = als3320a_read_reg(als3320a_obj->client, als3320a_reg[idx], 0xFF, 0x00);
		if (reg_value[0] < 0) {
			count += sprintf(buf+count, "i2c read_reg err\n");
			return count;
		}
		count += sprintf(buf+count, "[%x]=0x%x\n", als3320a_reg[idx], reg_value[0]);
	}
	return count;
}

static ssize_t als3320a_em_write(struct device_driver *ddri, const char *buf, size_t count)
{

	int addr, val;
	int ret = 0;

	if (!als3320a_obj) {
		APS_ERR("als3320a_obj is null!!\n");
		return -1;
	}

	ret = sscanf(buf, "%x %x", &addr, &val);

	APS_LOG("Reg[%x].Write [%x]..\n", addr, val);

	ret = als3320a_write_reg(als3320a_obj->client, addr, 0xFF, 0x00, val);

	return count;
}
static DRIVER_ATTR(em, 0660, als3320a_em_read, als3320a_em_write);
static DRIVER_ATTR(als_rawdata, 0660, als3320a_als_rawdata_show, NULL);
static DRIVER_ATTR(als_data, 0600, als3320a_als_data_show, NULL);
static DRIVER_ATTR(state, 0660, als3320a_status_show, NULL);
static DRIVER_ATTR(trace, 0660, als3320a_trace_show, als3320a_trace_store);



static struct driver_attribute *als3320a_attr_list[] = {
	&driver_attr_em,
	&driver_attr_als_rawdata,
	&driver_attr_als_data,
	&driver_attr_state,
	&driver_attr_trace,
};

static int als3320a_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(als3320a_attr_list)/sizeof(als3320a_attr_list[0]));
	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, als3320a_attr_list[idx]);
		if (err != 0) {
			APS_ERR("driver_create_file (%s) = %d\n", als3320a_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int als3320a_delete_attr(struct device_driver *driver)
{
	int idx , err = 0;
	int num = (int)(sizeof(als3320a_attr_list)/sizeof(als3320a_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, als3320a_attr_list[idx]);
	}

	return err;
}

static int ls_update_table(void)
{
	uint32_t tmp_data[ALS3320A_TABLE_LENGTH];
	int i;
//	if(als3320a_cali.cali_state==1){
		for (i = 0; i < ALS3320A_TABLE_LENGTH; i++) {
			tmp_data[i] = (uint32_t)(*(defalut_table + i)) * (als3320a_cali.als_cali_lux)/als3320a_cali.als_cali_base;
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

#ifdef ALS3320A_DELAY_CALIBRATION
static int als3320a_read_from_califile(void) 
{
	int ilen = 0;
	char tmp_data[32] = {0};
	int base=0, calilux=0;
	struct file *fp = NULL;

	if(als3320a_cali.cali_state==1) {
		APS_LOG("ALS Calibration already loaded !\n");
		return 0;
	}
		
	old_als_fs = get_fs();
	set_fs(KERNEL_DS);
//	fp=filp_open(ALS_CALI_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	fp=filp_open(ALS_CALI_PATH,O_RDWR,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		APS_LOG("ALS filp_open fail [%s] \n", ALS_CALI_PATH);
		return 0;
	}
	ilen = fp->f_op->read(fp,tmp_data,ALS3320A_CALI_FILE_LENGTH,&fp->f_pos);
	if(ilen == ALS3320A_CALI_FILE_LENGTH) {
		sscanf(tmp_data, "%04d,%04d\n", &base, &calilux);
		als3320a_cali.als_cali_base= base;
		als3320a_cali.als_cali_lux= calilux;
		APS_LOG("base [%d], calilux[%d]\n", base, calilux);
		APS_LOG("Load Calibration base [%d] , adc[%d] \n", als3320a_cali.als_cali_base, als3320a_cali.als_cali_lux);
		als3320a_cali.cali_state=1;
	} else {
		als3320a_cali.als_cali_base= 1000;
		als3320a_cali.als_cali_lux= ALS3320A_GOLDEN_VALUE_Z300M;
		APS_LOG("ilen[%d] != 32 \n", ilen);	
		APS_LOG("Use Defalut Calibration base [%d] , adc[%d] \n", als3320a_cali.als_cali_base, als3320a_cali.als_cali_lux);
		als3320a_cali.cali_state=0;
	}
	set_fs(old_als_fs);
	filp_close(fp,NULL);

//	if(als3320a_cali.cali_state==1) 
		ls_update_table();

	return als3320a_cali.cali_state;
}
#endif

static int als3320a_write_to_califile(int base,int calilux)
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

	ilen = fp->f_op->write(fp,tmp_data,ALS3320A_CALI_FILE_LENGTH,&fp->f_pos);
	APS_LOG( "alp.D : als write file len : %d\n",ilen);
	set_fs(old_als_fs);
	filp_close(fp,NULL);
	
	return retval;
}


//----------------------------- enable --------------------------------------
static ssize_t als3320a_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_data = 0;;
	als_data = als3320a_read_reg(als3320a_obj->client, ALS3320A_ENABLE, 0xFF, 0x00);
	if (als_data < 0) {
		als_data = 0;
		APS_LOG("read als3320a_enable_show fail !!\n");
	}
	return sprintf(buf, "%d\n", als_data);
}
static ssize_t als3320a_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = -1;
	u8 databuf[2];
	databuf[0] = ALS3320A_ENABLE;

	if(buf[0]=='1'){
		databuf[1] = 0x01;
		ret = als3320a_write_reg(als3320a_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
		if(als3320a_openlog) APS_LOG("Power set_mode 1 !!\n");
#if DELAYED_WORK
		if (ret > 0) 
			schedule_delayed_work(&als3320a_obj->eint_work, 1100*HZ/1000);
#endif
	}else{
//		databuf[1] = 0x00;
//		ret = als3320a_write_reg(als3320a_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
		if(als3320a_openlog) APS_LOG("Power set_mode 0 !!\n");
#if DELAYED_WORK
		if (ret > 0) 
			cancel_delayed_work_sync(&als3320a_obj->eint_work);
#endif
		als3320a_init_client(als3320a_obj->client);
	}
	if(als3320a_openlog) APS_LOG(" set Power %c ret=%d !!\n",buf[0] ,ret);
	
	return count;
}

//----------------------------- delay --------------------------------------
static ssize_t als3320a_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_delay = 0;;
	als_delay = atomic_read(&als3320a_obj->delay);
	if (als_delay < 0) {
		als_delay = 0;
		APS_ERR("read delay fail !!\n");
	}else{
		if(als3320a_openlog)APS_LOG("show delay = %d\n",als_delay);	
	}
	return sprintf(buf, "%d\n", als_delay);
}
static ssize_t als3320a_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms)){
		APS_ERR(" get interval_ms = %ld  fail!!\n",interval_ms);
	}

	if ((interval_ms < 1) || (interval_ms > 1000)){
		APS_ERR(" overflow interval_ms !!\n");
		interval_ms = 1000 ; // default 1000 ms
	}

	atomic_set(&als3320a_obj->delay, interval_ms);
	if(als3320a_openlog) APS_LOG(" set delay = %ld !!\n",interval_ms);
	
	return count;
}

//----------------------------- gain --------------------------------------
static ssize_t als3320a_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_gain=0;

	als_gain = als3320a_read_reg(als3320a_obj->client, ALS3320A_GAIN, 0xFF, 0x00);
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
static ssize_t als3320a_gain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	u8 databuf[2];

	databuf[0] = ALS3320A_GAIN;
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
	ret = als3320a_write_reg(als3320a_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
	APS_LOG(" set gain = %c , ret = %d !!\n",buf[0] ,ret);
	
	return count;
}

//----------------------------- lux --------------------------------------
static ssize_t als3320a_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err = 0, lux=0;
	err = als3320a_read_als(als3320a_obj->client, &als3320a_obj->als);
	lux = als3320a_obj->als * ALS3320A_USE_RESOLUTION/100;
	if(als3320a_openlog) APS_LOG("(%d) als3320a ALS adc=%d\n",err ,als3320a_obj->als);

	return sprintf(buf, "%d\n", lux);
}

//----------------------------- threshold low --------------------------------------
static ssize_t als3320a_thres_low_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_value_low[2];
	int als_thres_L=0;
	/* get ALS adc count */
	als_value_low[0] = als3320a_read_reg(als3320a_obj->client, ALS3320A_INT_LOW_THD_LOW, 0xFF, 0x00);
	if (als_value_low[0] < 0) {
		als_value_low[0] = 0;
		APS_LOG("read thres L_L fail !!\n");
	}
	als_value_low[1] = als3320a_read_reg(als3320a_obj->client, ALS3320A_INT_LOW_THD_HIGH, 0xFF, 0x00);
	if (als_value_low[1] < 0) {
		als_value_low[1] = 0;
		APS_LOG("read thres L_H fail !!\n");
	}
	als_thres_L = als_value_low[0]+ (als_value_low[1] *256);

	return sprintf(buf, "%d\n", als_thres_L);
}
static ssize_t als3320a_thres_low_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	u8 databuf_L[2];
	u8 databuf_H[2];

	databuf_L[0] = ALS3320A_INT_LOW_THD_LOW;
	databuf_H[0] = ALS3320A_INT_LOW_THD_HIGH;
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
	ret += als3320a_write_reg(als3320a_obj->client, databuf_L[0], 0xFF, 0x00, databuf_L[1]);
	ret += als3320a_write_reg(als3320a_obj->client, databuf_H[0], 0xFF, 0x00, databuf_H[1]);
	APS_LOG(" set L threshold ret = %d !!\n",ret);

	return count;
}

//----------------------------- threshold high --------------------------------------
static ssize_t als3320a_thres_high_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int als_value_high[2];
	int als_thres_H=0;
	/* get ALS adc count */
	als_value_high[0] = als3320a_read_reg(als3320a_obj->client, ALS3320A_INT_HIGH_THD_LOW, 0xFF, 0x00);
	if (als_value_high[0] < 0) {
		als_value_high[0] = 0;
		APS_LOG("read thres H_L fail !!\n");
	}
	als_value_high[1] = als3320a_read_reg(als3320a_obj->client, ALS3320A_INT_HIGH_THD_HIGH, 0xFF, 0x00);
	if (als_value_high[1] < 0) {
		als_value_high[1] = 0;
		APS_LOG("read thres H_H fail !!\n");
	}
	als_thres_H = als_value_high[0]+ (als_value_high[1] *256);

	return sprintf(buf, "%d\n", als_thres_H);
}
static ssize_t als3320a_thres_high_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	u8 databuf_L[2];
	u8 databuf_H[2];

	databuf_L[0] = ALS3320A_INT_HIGH_THD_LOW;
	databuf_H[0] = ALS3320A_INT_HIGH_THD_HIGH;
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
	ret += als3320a_write_reg(als3320a_obj->client, databuf_L[0], 0xFF, 0x00, databuf_L[1]);
	ret += als3320a_write_reg(als3320a_obj->client, databuf_H[0], 0xFF, 0x00, databuf_H[1]);
	APS_LOG(" set H threshold ret = %d !!\n",ret);

	return count;
}

//-----------------------------  Interrupt PIN status --------------------------------------
static ssize_t als3320a_pin_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	ret = als3320a_read_reg(als3320a_obj->client, ALS3320A_INT_STATUS, 0xFF, 0x00);
	APS_LOG("als3320a Interrput status %d   --(ALS:B0 ; PS:B1)-- \n",ret);

	return sprintf(buf, "PIN 0x%x\n", ret);
}

//-----------------------------  Control Pin --------------------------------------
static ssize_t als3320a_control_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	ret = als3320a_read_reg(als3320a_obj->client, ALS3320A_CONTROL_INT, 0xFF, 0x00);
	APS_LOG("als3320a ALS3320A_CONYROL_INT %d   --(ALS:B4 ; PS:B8)-- \n",ret);

	return sprintf(buf, "Control PIN 0x%x\n", ret);
}

static ssize_t als3320a_control_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	u8 databuf[2];
	databuf[0] = ALS3320A_CONTROL_INT;

	if(buf[0]=='0'){
		databuf[1] = 0x00;
		APS_LOG("control_PIN_set_mode 0--> clear INT !!\n");
	}else{
		databuf[1] = 0x08;
		APS_LOG("control_PIN_set_mode ?--> ALS:1 !!\n");
	}

	ret = als3320a_write_reg(als3320a_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);

	return count;
}

//-----------------------------  do calibration by owner --------------------------------------
static ssize_t als3320a_do_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	int power_state=0, do_cali=0, iloop=0, lux=0, ret=0;
	u8 databuf[2];

	power_state = als3320a_read_reg(als3320a_obj->client, ALS3320A_ENABLE, 0xFF, 0x00);
	if (power_state == ALS3320A_SYS_DEV_DOWN){
		databuf[0] = ALS3320A_ENABLE;
		databuf[1] = 0x01;
		ret = als3320a_write_reg(als3320a_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
	}
	CALI_TEST_DELAY();
	als3320a_cali.cali_state = 0;
	als3320a_cali.do_calibrating = 1;
	for(iloop=0;iloop<ALS3320A_CALI_SAMPLE_TIME;iloop++){
		als3320a_read_als(als3320a_obj->client, &als3320a_obj->als);
		lux = lux + als3320a_obj->als* ALS3320A_USE_RESOLUTION/100;
		CALI_TEST_DELAY();
		if(als3320a_openlog) APS_LOG("Total calibration lux = %d\n",lux);
	}
	als3320a_cali.als_cali_lux = lux/ALS3320A_CALI_SAMPLE_TIME;
	if(power_state!=1){
		databuf[0] = ALS3320A_ENABLE;
		databuf[1] = ALS3320A_SYS_DEV_DOWN;
		ret = als3320a_write_reg(als3320a_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
	}
	ret = als3320a_write_to_califile(als3320a_cali.als_cali_base, als3320a_cali.als_cali_lux);
	if(ret!=1){
		als3320a_cali.als_cali_lux = ALS3320A_GOLDEN_VALUE_Z300M;
		als3320a_cali.als_cali_base=1000;
		als3320a_cali.cali_state = 0;
		if(als3320a_openlog) APS_LOG("do calibration write file fail\n");
	}else{
		if(als3320a_openlog) APS_LOG("Need to update table after calibration\n");
		ls_update_table();
		als3320a_cali.cali_state = 1;
		do_cali = 1;
	}
	als3320a_cali.do_calibrating = 0;
	return sprintf(buf, "%d\n", do_cali);
}

static ssize_t als3320a_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//	struct als3320a_data *data = als3320a_data_g;
	int power_state=0, do_cali=0, iloop=0, lux=0, ret=0;
	u8 databuf[2];

	APS_LOG("als3320a_store_calibration_state..\n");
	als3320a_cali.do_calibrating = 1;
	if(buf[0]=='0'){
		als3320a_cali.cali_state = 0;
		als3320a_cali.als_cali_base = 1000;
		als3320a_cali.als_cali_lux = ALS3320A_GOLDEN_VALUE_Z300M;
		if(als3320a_openlog) APS_LOG(" calibration reset!!\n");
	}else if(buf[0]=='1'){
		do_cali = 1;
		if(als3320a_openlog) APS_LOG(" calibration lux 1000!!\n");
	}else if(buf[0]=='2'){
		do_cali = 1;
		if(als3320a_openlog) APS_LOG(" calibration lux 200!!\n");
	}else{
		do_cali = 0;
		als3320a_cali.als_cali_base = 200;
		if(als3320a_openlog) APS_LOG("error input !! none done\n");
	}
	power_state = als3320a_read_reg(als3320a_obj->client, ALS3320A_ENABLE, 0xFF, 0x00);
	if(do_cali==1){
		als3320a_cali.cali_state = 0;
		if (power_state == ALS3320A_SYS_DEV_DOWN){
			databuf[0] = ALS3320A_ENABLE;
			databuf[1] = 0x01;
			ret = als3320a_write_reg(als3320a_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
		}
		CALI_TEST_DELAY();
		for(iloop=0;iloop<ALS3320A_CALI_SAMPLE_TIME;iloop++){
			als3320a_read_als(als3320a_obj->client, &als3320a_obj->als);
			lux = lux + als3320a_obj->als* ALS3320A_USE_RESOLUTION/100;
			CALI_TEST_DELAY();
			if(als3320a_openlog) APS_LOG("Total calibration lux = %d\n",lux);
		}
		als3320a_cali.als_cali_lux = lux/ALS3320A_CALI_SAMPLE_TIME;
		if(power_state!=1){
			databuf[0] = ALS3320A_ENABLE;
			databuf[1] = ALS3320A_SYS_DEV_DOWN;
			ret = als3320a_write_reg(als3320a_obj->client, databuf[0], 0xFF, 0x00, databuf[1]);
		}
		do_cali = als3320a_write_to_califile(als3320a_cali.als_cali_base, als3320a_cali.als_cali_lux);
		if(do_cali!=1){
			als3320a_cali.als_cali_lux = ALS3320A_GOLDEN_VALUE_Z300M;
			als3320a_cali.als_cali_base=1000;
			als3320a_cali.cali_state = 0;
		}else{
			if(als3320a_openlog) APS_LOG("Need to update table after calibration\n");
			ls_update_table();
			als3320a_cali.cali_state = 1;
		}
	}
	printk("%d\n", do_cali);
	als3320a_cali.do_calibrating = 0;
	return count;
}

//-----------------------------  factory calibration --------------------------------------
static ssize_t als3320a_factory_cal_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	return count;
}

//-----------------------------  state --------------------------------------
static ssize_t als3320a_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", als3320a_init_flag);
}

//-----------------------------  openlog --------------------------------------
static ssize_t als3320a_openlog_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", als3320a_openlog);
}
static ssize_t als3320a_openlog_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(buf[0]=='1'){
		als3320a_openlog = 1;
		APS_LOG("open debug log\n");
	}else{
		als3320a_openlog = 0;
		APS_LOG("close debug log\n");
	}
	return count;
}


static DEVICE_ATTR(enable, 0660, als3320a_enable_show, als3320a_enable_store);
static DEVICE_ATTR(delay, 0660, als3320a_delay_show, als3320a_delay_store);
static DEVICE_ATTR(gain, 0660, als3320a_gain_show, als3320a_gain_store);
static DEVICE_ATTR(lux, 0660, als3320a_lux_show, NULL);
static DEVICE_ATTR(als_thres_low, 0660, als3320a_thres_low_show, als3320a_thres_low_store);
static DEVICE_ATTR(als_thres_high, 0660, als3320a_thres_high_show, als3320a_thres_high_store);
static DEVICE_ATTR(pin_status, 0660, als3320a_pin_status_show, NULL);
static DEVICE_ATTR(control_pin, 0660, als3320a_control_pin_show, als3320a_control_pin_store);
static DEVICE_ATTR(calibration, 0660, als3320a_do_calibration, als3320a_calibration_store);
static DEVICE_ATTR(factory_cal, 0660, NULL, als3320a_factory_cal_store);
static DEVICE_ATTR(state, 0660, als3320a_state_show, NULL);
static DEVICE_ATTR(openlog, 0660, als3320a_openlog_show, als3320a_openlog_store);

static struct attribute *als3320a_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_gain.attr,
	&dev_attr_lux.attr,
	&dev_attr_als_thres_low.attr,
	&dev_attr_als_thres_high.attr,
	&dev_attr_pin_status.attr,
	&dev_attr_control_pin.attr,
	&dev_attr_calibration.attr,
	&dev_attr_factory_cal.attr,
	&dev_attr_state.attr,
	&dev_attr_openlog.attr,
	NULL
};

static struct attribute_group als3320a_attribute_group = {
	.attrs = als3320a_attributes
};


/******************************************************************************
 * Function Configuration
******************************************************************************/
static int als3320a_open(struct inode *inode, struct file *file)
{
	file->private_data = als3320a_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int als3320a_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long als3320a_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
//	struct i2c_client *client = (struct i2c_client *)file->private_data;
//	struct als3320a_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user *) arg;
	int dat;
	uint32_t enable;

	switch (cmd) {
	case ALSPS_SET_ALS_MODE:
			if (copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			if (enable) {
				err = als3320a_enable_als(als3320a_obj->client, 1);
				if (err != 0) {
					APS_ERR("enable als fail: %ld\n", err);
					goto err_out;
				}

				set_bit(CMC_BIT_ALS, &als3320a_obj->enable);

			} else{
				err = als3320a_enable_als(als3320a_obj->client, 0);
				if (err != 0) {
					APS_ERR("disable als fail: %ld\n", err);
					goto err_out;
				}

				clear_bit(CMC_BIT_ALS, &als3320a_obj->enable);

			}
			break;

	case ALSPS_GET_ALS_MODE:

			enable = test_bit(CMC_BIT_ALS, &als3320a_obj->enable) ? (1) : (0);

			if (copy_to_user(ptr, &enable, sizeof(enable)))	{
				err = -EFAULT;
				goto err_out;
			}
			break;

	case ALSPS_GET_ALS_DATA:
			err = als3320a_read_als(als3320a_obj->client, &als3320a_obj->als);
			if (err != 0) {
				goto err_out;
			}

			dat = als3320a_obj->als* ALS3320A_USE_RESOLUTION/100;//ap3xx6_get_als_value(ap3xx6_obj, ap3xx6_obj->als);
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

	case ALSPS_GET_ALS_RAW_DATA:
			err = als3320a_read_als(als3320a_obj->client, &als3320a_obj->als);
			if (err != 0) {
				goto err_out;
			}

			dat = als3320a_obj->als;
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

/*----------------------------------------------------------------------------*/
static const struct file_operations als3320a_fops = {
/* .owner = THIS_MODULE, */
	.open = als3320a_open,
	.release = als3320a_release,
	.unlocked_ioctl = als3320a_ioctl,
#if 0
#ifdef CONFIG_COMPAT
	.compat_ioctl = als3320a_compat_ioctl,
#endif
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice als3320a_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &als3320a_fops,
};
/*----------------------------------------------------------------------------*/
static int als3320a_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static int als3320a_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void als3320a_early_suspend(struct early_suspend *h)
{
//	struct als3320a_priv *obj = container_of(h, struct als3320a_priv, early_drv);
	int err;
	APS_FUN();

	if (!als3320a_obj) {
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&als3320a_obj->als_suspend, 1);

	if (test_bit(CMC_BIT_ALS, &als3320a_obj->enable)) {
		err = als3320a_enable_als(als3320a_obj->client, 0);
		if (err != 0) {
			APS_ERR("disable als fail: %d\n", err);
		}
	}

}
/*----------------------------------------------------------------------------*/
static void als3320a_late_resume(struct early_suspend *h)
{
//	struct als3320a_priv *obj = container_of(h, struct als3320a_priv, early_drv);
	int err;
	APS_FUN();

	if (!als3320a_obj) {
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&als3320a_obj->als_suspend, 0);
	if (test_bit(CMC_BIT_ALS, &als3320a_obj->enable)) {
		err = als3320a_enable_als(als3320a_obj->client, 1);
		if (err != 0) {
			APS_ERR("enable als fail: %d\n", err);

		}
	}

}
#endif

static int als3320a_als_open_report_data(int open)
{

	return 0;
}

static int als3320a_als_enable_nodata(int en)
{
//	struct als3320a_priv *obj = als3320a_obj;
	int value = 0;
	int err = 0;

	if (als3320a_obj == NULL) {
		APS_ERR("als3320a_obj is null\n");
		return -1;
	}

	value = en;
	if (value) {
		err = als3320a_enable_als(als3320a_obj->client, 1);
		if (err != 0) {
			APS_ERR("enable als fail: %d\n", err);
			return -1;
		}
		set_bit(CMC_BIT_ALS, &als3320a_obj->enable);
	} else{
		err = als3320a_enable_als(als3320a_obj->client, 0);
		if (err != 0) {
			APS_ERR("disable als fail: %d\n", err);
			return -1;
		}
		clear_bit(CMC_BIT_ALS, &als3320a_obj->enable);
	}
	return 0;
}

static int als3320a_als_set_delay(u64 ns)
{
	return 0;
}

static int als3320a_als_get_data(int *value, int *status)
{
//	struct als3320a_priv *obj = als3320a_obj;
	static int temp_als;
	int temp_value = -1;
	int err = 0;

	if (als3320a_obj == NULL) {
		APS_ERR("als3320a_obj is null\n");
		return -1;
	}

	als3320a_read_als(als3320a_obj->client, &als3320a_obj->als);
	if (atomic_read(&als3320a_obj->trace) & TRACE_DEBUG) {
		if(als3320a_openlog) APS_LOG("als3320a ALS level=%d\n", als3320a_obj->als);
	}

	if (als3320a_obj->als == 0) {
		temp_value = temp_als* ALS3320A_USE_RESOLUTION/100;
	} else{
		u16 b[2];
		int i;
		for (i = 0; i < 2; i++) {
			als3320a_read_als(als3320a_obj->client, &als3320a_obj->als);
			b[i] = als3320a_obj->als;
		}
		(b[1] > b[0])?(als3320a_obj->als = b[0]):(als3320a_obj->als = b[1]);
		temp_value = als3320a_obj->als* ALS3320A_USE_RESOLUTION/100;//ap3xx6_get_als_value(ap3xx6_obj, ap3xx6_obj->als);
		temp_als = temp_value;
	}

	*value = temp_value;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return err;
}

static int als3320a_input_init(void)
{
	int ret;
	// allocate light input_device 
	als3320a_obj->input_dev = input_allocate_device();
	if (!als3320a_obj->input_dev) {
		APS_ERR("als3320a could not allocate input device\n");
		goto err_light_all;
	}

	als3320a_obj->input_dev->name = ALS3320A_INPUT_NAME;
	input_set_capability(als3320a_obj->input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(als3320a_obj->input_dev, ABS_MISC, 0, 1, 0, 0);

	APS_LOG("als3320a registering light sensor input device\n");
	ret = input_register_device(als3320a_obj->input_dev);
	if (ret < 0) {
		APS_ERR("could not register input device\n");
		goto err_light_reg;
	}
	return 0;

err_light_reg:
	input_free_device(als3320a_obj->input_dev);
err_light_all:
	return (-1);   
}

static int of_get_als3320a_platform_data(struct device *dev)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,als3320a");
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
static int als3320a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
//	struct als3320a_priv *obj = NULL;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	int err = 0;

	APS_FUN();
	als3320a_obj = kzalloc(sizeof(*als3320a_obj), GFP_KERNEL);
	if (als3320a_obj == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	memset(als3320a_obj, 0, sizeof(*als3320a_obj));
//	als3320a_obj = obj;

	mutex_init(&als3320a_obj->lock);

	als3320a_obj->hw = &cust_alsps_hw;

	als3320a_get_addr(als3320a_obj->hw, &als3320a_obj->addr);

#if DELAYED_WORK
	INIT_DELAYED_WORK(&als3320a_obj->eint_work, als3320a_eint_work);
#else
	INIT_WORK(&als3320a_obj->eint_work, als3320a_eint_work);
#endif
	als3320a_obj->client = client;
	i2c_set_clientdata(client, als3320a_obj);
	atomic_set(&als3320a_obj->als_debounce, 300);
	atomic_set(&als3320a_obj->als_deb_on, 0);
	atomic_set(&als3320a_obj->als_deb_end, 0);
	atomic_set(&als3320a_obj->als_suspend, 0);
	atomic_set(&als3320a_obj->als_cmd_val, 0xDF);
	atomic_set(&als3320a_obj->trace, 0);

	als3320a_obj->enable = 0;
	als3320a_obj->pending_intr = 0;
	als3320a_obj->als_level_num = sizeof(als3320a_obj->hw->als_level)/sizeof(als3320a_obj->hw->als_level[0]);
	als3320a_obj->als_value_num = sizeof(als3320a_obj->hw->als_value)/sizeof(als3320a_obj->hw->als_value[0]);
	als3320a_obj->als_modulus = (400*100*40)/(1*1500);

	BUG_ON(sizeof(als3320a_obj->als_level) != sizeof(als3320a_obj->hw->als_level));
	memcpy(als3320a_obj->als_level, als3320a_obj->hw->als_level, sizeof(als3320a_obj->als_level));
	BUG_ON(sizeof(als3320a_obj->als_value) != sizeof(als3320a_obj->hw->als_value));
	memcpy(als3320a_obj->als_value, als3320a_obj->hw->als_value, sizeof(als3320a_obj->als_value));
	atomic_set(&als3320a_obj->i2c_retry, 3);
	
	APS_LOG("als3320a probe client->addr = %x\n", client->addr);

	als3320a_i2c_client = client;
	err = als3320a_init_client(client);
	if (err != 0) {
		APS_ERR("als3320a_init_client() ERROR!\n");
		goto exit_init_failed;
	}
	APS_LOG("als3320a_init_client() OK!\n");

	of_get_als3320a_platform_data(&client->dev);
	gpio_request_one(alsps_int_gpio_number, GPIOF_IN, "alsps_int");

	err = misc_register(&als3320a_device);
	if (err != 0) {
		APS_ERR("als3320a_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = als3320a_create_attr(&als3320a_init_info.platform_diver_addr->driver);
	if (err != 0) {
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	// init input subsystem	
	APS_LOG("als3320a_input_init start!!\n");
	err = als3320a_input_init();
	if (err) {
		APS_ERR("als3320a_input_init failed.\n");
		goto exit_create_attr_failed;
	}

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
	err += sysfs_create_file(android_light_kobj, &dev_attr_factory_cal.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_state.attr);
	err += sysfs_create_file(android_light_kobj, &dev_attr_openlog.attr);
	if (err != 0) {
		APS_ERR("create light sensor attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}	
	err = sysfs_create_group(&client->dev.kobj, &als3320a_attribute_group);
	// sys init --

#ifdef ALS3320A_DELAY_CALIBRATION
	atomic_set(&als3320a_obj->delaycalibration, ALS3320A_DELAY_CALITIME);  // 12000 ms
	INIT_DELAYED_WORK(&als3320a_obj->delayworkcalibration, als3320a_delay_calibration_func);
#endif


	err = als3320a_setup_eint(client);
	if (err != 0) {
		APS_ERR("setup eint: %d\n", err);
		goto exit_create_attr_failed;
	}

	als_ctl.open_report_data = als3320a_als_open_report_data;
	als_ctl.enable_nodata = als3320a_als_enable_nodata;
	als_ctl.set_delay = als3320a_als_set_delay;
	als_ctl.is_support_batch = als3320a_obj->hw->is_batch_supported_als;
	als_ctl.is_use_common_factory = false;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_polling_mode = false;

	err = als_register_control_path(&als_ctl);
	if (err != 0) {
		APS_ERR("als_register_control_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_data.get_data = als3320a_als_get_data;
	als_data.vender_div = 1;

	err = als_register_data_path(&als_data);

	if (err != 0) {
		APS_ERR("als_register_data_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	APS_LOG("als_register_data_path OK.%s:\n", __func__);

	als3320a_cali.do_calibrating=0;
	als3320a_cali.cali_state=0;
	als3320a_cali.als_cali_base=1000;
	als3320a_cali.als_cali_lux=ALS3320A_GOLDEN_VALUE_Z300M;

	atomic_set(&als3320a_obj->delay, 200);

#ifdef ALS3320A_DELAY_CALIBRATION
	schedule_delayed_work(&als3320a_obj->delayworkcalibration, msecs_to_jiffies(atomic_read(&als3320a_obj->delaycalibration)));
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	als3320a_obj->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	als3320a_obj->early_drv.suspend = als3320a_early_suspend,
	als3320a_obj->early_drv.resume	= als3320a_late_resume,
	register_early_suspend(&als3320a_obj->early_drv);
#endif

	als3320a_init_flag = 1;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&als3320a_device);
exit_misc_device_register_failed:
exit_init_failed:
	mutex_destroy(&als3320a_obj->lock);
//exit_kfree:
	kfree(als3320a_obj);
exit:
	als3320a_i2c_client = NULL;
	als3320a_init_flag = -1;
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int als3320a_i2c_remove(struct i2c_client *client)
{
	int err;

	als3320a_delete_attr(&als3320a_init_info.platform_diver_addr->driver);
	err = misc_deregister(&als3320a_device);
	if (err != 0) {
		APS_ERR("misc_deregister fail: %d\n", err);
	}

	if (1 == als3320a_obj->hw->polling_mode_ps) {
		wake_lock_destroy(&chrg_lock);
	}

	als3320a_i2c_client = NULL;
	i2c_unregister_device(client);
	mutex_destroy(&als3320a_obj->lock);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int als3320a_local_init(void)
{
	if (i2c_add_driver(&als3320a_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
	if (-1 == als3320a_init_flag) {
		APS_ERR("add driver--als3320a_init_flag check error\n");
		return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int als3320a_remove(void)
{
	APS_FUN();
	i2c_del_driver(&als3320a_i2c_driver);
	als3320a_init_flag = -1;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init als3320a_init(void)
{
	struct alsps_hw *hw = &cust_alsps_hw;
	APS_FUN();
	APS_LOG("%s: i2c_number=%d\n", __func__, hw->i2c_num);

	alsps_driver_add(&als3320a_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit als3320a_exit(void)
{
	APS_FUN();
}


module_init(als3320a_init);
module_exit(als3320a_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("alp");
MODULE_DESCRIPTION("als3320a driver");
MODULE_LICENSE("GPL");
