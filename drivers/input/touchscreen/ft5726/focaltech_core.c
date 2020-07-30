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
 * VERSION      	DATE			AUTHOR
 *    1.0		       2014-09			mshl
 *
 */

 /*******************************************************************************
*
* File Name: focaltech.c
*
* Author: mshl
*
* Created: 2014-09
*
* Modify by mshl on 2015-07-06
*
* Abstract:
*
* Reference:
*
*******************************************************************************/
/*******************************************************************************
* Included header files
*******************************************************************************/
//user defined include header files
#include "focaltech_core.h"
#include <linux/wakelock.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FTS_SUSPEND_LEVEL 1
#endif
#define CONFIG_OF 1


/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_META_REGS		3
#define FTS_ONE_TCH_LEN		6
#define FTS_TCH_LEN(x)		(FTS_META_REGS + FTS_ONE_TCH_LEN * x)

#define FTS_PRESS		0x7F
#define FTS_MAX_ID		0x0F
#define FTS_TOUCH_X_H_POS	3
#define FTS_TOUCH_X_L_POS	4
#define FTS_TOUCH_Y_H_POS	5
#define FTS_TOUCH_Y_L_POS	6
#define FTS_TOUCH_PRE_POS	7
#define FTS_TOUCH_AREA_POS	8
#define FTS_TOUCH_POINT_NUM		2
#define FTS_TOUCH_EVENT_POS	3
#define FTS_TOUCH_ID_POS		5

#define FTS_TOUCH_DOWN		0
#define FTS_TOUCH_UP		1
#define FTS_TOUCH_CONTACT	2

#define POINT_READ_BUF	(3 + FTS_ONE_TCH_LEN * FTS_MAX_POINTS)

/*register address*/
#define FTS_REG_DEV_MODE		0x00
#define FTS_DEV_MODE_REG_CAL	0x02

#define FTS_REG_PMODE		0xA5

#define FTS_REG_POINT_RATE	0x88
#define FTS_REG_THGROUP		0x80

/* power register bits*/
#define FTS_PMODE_ACTIVE		0x00
#define FTS_PMODE_MONITOR	0x01
#define FTS_PMODE_STANDBY	0x02
#define FTS_PMODE_HIBERNATE	0x03

#define FTS_STATUS_NUM_TP_MASK	0x0F

#define FTS_VTG_MIN_UV		2600000
#define FTS_VTG_MAX_UV		3300000
#define FTS_I2C_VTG_MIN_UV		1800000
#define FTS_I2C_VTG_MAX_UV		1800000

#define FTS_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FTS_8BIT_SHIFT		8
#define FTS_4BIT_SHIFT		4

/* psensor register address*/
#define FTS_REG_PSENSOR_ENABLE	0xB0
#define FTS_REG_PSENSOR_STATUS	0x01

/* psensor register bits*/
#define FTS_PSENSOR_ENABLE_MASK	0x01
#define FTS_PSENSOR_STATUS_NEAR	0xC0
#define FTS_PSENSOR_STATUS_FAR	0xE0
#define FTS_PSENSOR_FAR_TO_NEAR	0
#define FTS_PSENSOR_NEAR_TO_FAR	1
#define FTS_PSENSOR_ORIGINAL_STATE_FAR	1
#define FTS_PSENSOR_WAKEUP_TIMEOUT	500

#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"

//add by leo ++
#define FTS_USB_PLUG_IN	0x8B

#define TOUCH_SDEV_NAME "touch"
#define boolean unsigned char
#define FOCAL_BIEL_OGS_BLACK_FW_PATH "FOCAL_BIEL_OGS_BLACK_FW.bin"
#define FOCAL_BIEL_GFF_WHITE_FW_PATH "FOCAL_BIEL_GFF_WHITE_FW.bin"
#define FOCAL_BIEL_OGS_WHITE_FW_PATH "FOCAL_BIEL_OGS_WHITE_FW.bin"
#define FOCAL_LAIB_OGS_BLACK_FW_PATH "FOCAL_LAIB_OGS_BLACK_FW.bin"
/* 
 * 0 : 00 BIEL   OGS BLACK
 * 1 : 01 BIEL   GFF WHITE
 * 2 : 10 BIEL   OGS WHITE
 * 3 : 11 LAIBAO OGS BLACK
 */
#define BIEL_OGS_BLACK 0
#define BIEL_GFF_WHITE 1
#define BIEL_OGS_WHITE 2
#define LAIB_OGS_BLACK 3

static struct proc_dir_entry *ft5726_proc_gesture_file = NULL;
static struct proc_dir_entry *ft5726_proc_tp_debug_file = NULL;
static struct proc_dir_entry *ft5726_proc_tp_test_file = NULL;

struct fts_ts_data *private_ts;
struct wake_lock int_wakelock;

static int ft5726_debug = 0;
static int irq_is_disable = 0;

/* Add by Tom Cheng for USB Status Notify */
int usb_state = 0;
int is_probe_success = 0;
int ft5726_cable_status_handler(int state);

int FT5726_HW_ID = 0;
int FT5726_TP_ID = -1;
// int FW_VERSION = 0; // Mark by Tom for MTK defined but not used 
int ft5726_touch_status = 0;
/* Add by Tom Cheng for Check BOE Panel */
static int is_obe_lcm = 0;

void ft5726_irq_enable(void);
void ft5726_irq_disable(void);

extern int build_version;
extern int Read_HW_ID(void);
extern boolean start_test_tp(void);
extern int set_param_data(char * TestParamData);
extern int entry_mode; 
//add by leo --


/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/


/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
struct i2c_client *fts_i2c_client;
struct fts_ts_data *fts_wq_data;
struct input_dev *fts_input_dev;

static unsigned int buf_count_add=0;
static unsigned int buf_count_neg=0;

u8 buf_touch_data[30*POINT_READ_BUF] = { 0 };

#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
static struct sensors_classdev __maybe_unused sensors_proximity_cdev = {
	.name = "fts-proximity",
	.vendor = "FocalTech",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int fts_ts_start(struct device *dev);
static int fts_ts_stop(struct device *dev);

#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
/*******************************************************************************
*  Name: fts_psensor_support_enabled
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static inline bool fts_psensor_support_enabled(void)
{
	return config_enabled(CONFIG_TOUCHSCREEN_FTS_PSENSOR);
}
#endif

/*******************************************************************************
*  Name: fts_i2c_read
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			LDBG(" i2c read error.\n");
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			LDBG("i2c read error.\n");
	}

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

/*******************************************************************************
*  Name: fts_i2c_write
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	mutex_lock(&i2c_rw_access);
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		LDBG(" i2c write error.\n");

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

/*******************************************************************************
*  Name: fts_write_reg
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return fts_i2c_write(client, buf, sizeof(buf));
}

/*******************************************************************************
*  Name: fts_read_reg
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return fts_i2c_read(client, &addr, 1, val, 1);
}

#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
/*******************************************************************************
*  Name: fts_psensor_enable
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_psensor_enable(struct fts_ts_data *data, int enable)
{
	u8 state;
	int ret = -1;

	if (data->client == NULL)
		return;

	fts_read_reg(data->client, FTS_REG_PSENSOR_ENABLE, &state);
	if (enable)
		state |= FTS_PSENSOR_ENABLE_MASK;
	else
		state &= ~FTS_PSENSOR_ENABLE_MASK;

	ret = fts_write_reg(data->client, FTS_REG_PSENSOR_ENABLE, state);
	if (ret < 0)
		LDBG("write psensor switch command failed\n");
	return;
}

/*******************************************************************************
*  Name: fts_psensor_enable_set
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_psensor_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct fts_psensor_platform_data *psensor_pdata =
		container_of(sensors_cdev,
			struct fts_psensor_platform_data, ps_cdev);
	struct fts_ts_data *data = psensor_pdata->data;
	struct input_dev *input_dev = data->psensor_pdata->input_psensor_dev;

	mutex_lock(&input_dev->mutex);
	fts_psensor_enable(data, enable);
	psensor_pdata->tp_psensor_data = FTS_PSENSOR_ORIGINAL_STATE_FAR;
	if (enable)
		psensor_pdata->tp_psensor_opened = 1;
	else
		psensor_pdata->tp_psensor_opened = 0;
	mutex_unlock(&input_dev->mutex);
	return enable;
}

/*******************************************************************************
*  Name: fts_read_tp_psensor_data
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_read_tp_psensor_data(struct fts_ts_data *data)
{
	u8 psensor_status;
	char tmp;
	int ret = 1;

	fts_read_reg(data->client,
			FTS_REG_PSENSOR_STATUS, &psensor_status);

	tmp = data->psensor_pdata->tp_psensor_data;
	if (psensor_status == FTS_PSENSOR_STATUS_NEAR)
		data->psensor_pdata->tp_psensor_data =
						FTS_PSENSOR_FAR_TO_NEAR;
	else if (psensor_status == FTS_PSENSOR_STATUS_FAR)
		data->psensor_pdata->tp_psensor_data =
						FTS_PSENSOR_NEAR_TO_FAR;

	if (tmp != data->psensor_pdata->tp_psensor_data) {
		LDBG("sensor data changed\n");
		ret = 0;
	}
	return ret;
}
#else
/*******************************************************************************
*  Name: fts_psensor_enable_set
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
/*
static int fts_psensor_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	return enable;
}
*/

/*******************************************************************************
*  Name: fts_read_tp_psensor_data
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
/*
static int fts_read_tp_psensor_data(struct fts_ts_data *data)
{
	return 0;
}
*/
#endif

/*******************************************************************************
*  Name: fts_ts_interrupt
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static irqreturn_t fts_ts_interrupt(int irq, void *dev_id)
{
	struct fts_ts_data *fts_ts = dev_id;

	// add by leo for suspend issue ++
	if((!(private_ts->gesture & _GESTURE_ENABLE))&&(private_ts->suspended))
	{
		printk(KERN_ERR "[ft5726] %s: skip touch interrupts during sleep mode\n", __func__);
		return IRQ_HANDLED;
	}
	// add by leo for suspend issue --
	
	/* Add by Tom Cheng For Delay 30ms for Gesture I2C fail when resume time */
	if((private_ts->gesture & _GESTURE_ENABLE) && (private_ts->suspended)) {
		wake_lock(&int_wakelock);
		LDBG("In Touch Gesture Mode\n");
		msleep(30);
	}

	ft5726_irq_disable();

	if (!fts_ts) {
		LDBG("Invalid fts_ts\n");
		return IRQ_HANDLED;
	}

	queue_work(fts_ts->ts_workqueue, &fts_ts->touch_event_work);

	/* Add by Tom Cheng For Delay 10ms for Gesture I2C fail when resume time */
	if((private_ts->gesture & _GESTURE_ENABLE) && (private_ts->suspended)) 
		wake_unlock(&int_wakelock);

	return IRQ_HANDLED;
}
/*******************************************************************************
*  Name: fts_read_Touchdata
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_read_Touchdata(struct fts_ts_data *data)
{
#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
	int rc = 0;
#endif

	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	u8 state;


	#if FTS_GESTRUE_EN
	//if(data->suspended)
	if((private_ts->gesture & _GESTURE_ENABLE)&&(data->suspended)) // add by leo for suspend issue
	{
		fts_read_reg(data->client, 0xd0, &state);
	       if(state ==1)
	       {
	          fts_read_Gestruedata();
		   return 1;
	      }
	}
      #endif
	// add by leo for suspend issue ++
	if((!(private_ts->gesture & _GESTURE_ENABLE))&&(private_ts->suspended))
	{
		printk(KERN_ERR "[ft5726] %s: skip touch interrupts during sleep mode\n", __func__);
		return IRQ_HANDLED;
	}
	// add by leo for suspend issue --

	#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
	if (fts_psensor_support_enabled() && data->pdata->psensor_support &&
		data->psensor_pdata->tp_psensor_opened) {
		rc = fts_read_tp_psensor_data(data);
		if (!rc) {
			if (data->suspended)
				pm_wakeup_event(&data->client->dev,
					FTS_PSENSOR_WAKEUP_TIMEOUT);
			input_report_abs(data->psensor_pdata->input_psensor_dev,
					ABS_DISTANCE,
					data->psensor_pdata->tp_psensor_data);
			input_sync(data->psensor_pdata->input_psensor_dev);
			if (data->suspended)
				return 1;
		}
		if (data->suspended)
			return 1;
	}
	#endif
	
	ret = fts_i2c_read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		LDBG("read touchdata failed.\n");
		return ret;
	}

	buf_count_add++;
	memcpy( buf_touch_data+(((buf_count_add-1)%30)*POINT_READ_BUF), buf, sizeof(u8)*POINT_READ_BUF );

	return 0;
}

/*******************************************************************************
*  Name: fts_report_value
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_report_value(struct fts_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;
	int touchs = 0;
	u8 pointid = FTS_MAX_ID;
	
	u8 buf[POINT_READ_BUF] = { 0 };

	buf_count_neg++;
	
	memcpy( buf,buf_touch_data+(((buf_count_neg-1)%30)*POINT_READ_BUF), sizeof(u8)*POINT_READ_BUF );


	memset(event, 0, sizeof(struct ts_event));

	event->point_num=buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	event->touch_point = 0;
	for (i = 0; i < FTS_MAX_POINTS; i++) {
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			event->touch_point++;
		
		event->au16_x[i] =
		    (s16) (buf[FTS_TOUCH_X_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_ONE_TCH_LEN * i];
		event->au16_y[i] =
		    (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_ONE_TCH_LEN * i];
		event->au8_touch_event[i] =
		    buf[FTS_TOUCH_EVENT_POS + FTS_ONE_TCH_LEN * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		event->area[i] =
			(buf[FTS_TOUCH_AREA_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		event->pressure[i] =
			(s16) buf[FTS_TOUCH_PRE_POS + FTS_ONE_TCH_LEN * i];

		if(0 == event->area[i])
			event->area[i] = 0x09;

		if(0 == event->pressure[i])
			event->pressure[i] = 0x3f;

		if((event->au8_touch_event[i]==0 || event->au8_touch_event[i]==2)&&(event->point_num==0))
			return;
	}
	
	for (i = 0; i < event->touch_point; i++)
	{
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);
		
		if (event->au8_touch_event[i] == FTS_TOUCH_DOWN || event->au8_touch_event[i] == FTS_TOUCH_CONTACT)
		{
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
			/* Add by Tom Cheng for recover x y */
			event->au16_x[i] = private_ts->pdata->x_max - event->au16_x[i];
			event->au16_y[i] = private_ts->pdata->y_max - event->au16_y[i];
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			touchs |= BIT(event->au8_finger_id[i]);
			data->touchs |= BIT(event->au8_finger_id[i]);
		}
		else
		{
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(event->au8_finger_id[i]);
		}
	}

	if(unlikely(data->touchs ^ touchs))
	{
		for(i = 0; i < FTS_MAX_POINTS; i++)
		{
			if(BIT(i) & (data->touchs ^ touchs))
			{
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}
	data->touchs = touchs;
	if(event->touch_point == uppoint)
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	}
	else
	{
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}

	input_sync(data->input_dev);
}

/*******************************************************************************
*  Name: fts_touch_irq_work
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_touch_irq_work(struct work_struct *work)
{
	int ret = -1;
	
	ret = fts_read_Touchdata(fts_wq_data);
	if (ret == 0)
		fts_report_value(fts_wq_data);

	//enable_irq(fts_wq_data->client->irq);
	ft5726_irq_enable();
}

/*******************************************************************************
*  Name: fts_gpio_configure
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_gpio_configure(struct fts_ts_data *data, bool on)
{
	int err = 0;

	if (on) {
		if (gpio_is_valid(data->pdata->irq_gpio)) {
			err = gpio_request(data->pdata->irq_gpio,
						"focaltech-interrupt");
			if (err) {
				LDBG("irq gpio request failed");
				goto err_irq_gpio_req;
			}

			err = gpio_direction_input(data->pdata->irq_gpio);
			if (err) {
				LDBG("set_direction for irq gpio failed\n");
				goto err_irq_gpio_dir;
			}
		}

		if (gpio_is_valid(data->pdata->reset_gpio)) {
#if 0
			err = gpio_request(data->pdata->reset_gpio,
						"fts_reset_gpio");
			if (err) {
				LDBG("reset gpio request failed");
				goto err_irq_gpio_dir;
			}
#endif
			err = gpio_direction_output(data->pdata->reset_gpio, 0);
			if (err) {
				LDBG(
				"set_direction for reset gpio failed\n");
				goto err_reset_gpio_dir;
			}
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}

		return 0;
	} else {
		if (gpio_is_valid(data->pdata->irq_gpio))
			gpio_free(data->pdata->irq_gpio);
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			/*
			 * This is intended to save leakage current
			 * only. Even if the call(gpio_direction_input)
			 * fails, only leakage current will be more but
			 * functionality will not be affected.
			 */
			err = gpio_direction_input(data->pdata->reset_gpio);
			if (err) {
				LDBG("unable to set direction for gpio "
					"[%d]\n", data->pdata->irq_gpio);
			}
			gpio_free(data->pdata->reset_gpio);
		}

		return 0;
	}

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

err_irq_gpio_req:
	return err;
}

/*******************************************************************************
*  Name: fts_power_on
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
/*
static int fts_power_on(struct fts_ts_data *data, bool on)
{
	int rc = 0;

	return rc; // add by leo for testtest

	if (!on)
	{
		FTS_DBG("Enter fts_power_on false ! \n");
		goto power_off;
	}

	rc = regulator_enable(data->vdd);
	if (rc) {
		LDBG( "Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}
#if 1
	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		LDBG( "Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}
#endif
	return rc;


power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		LDBG( "Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}
#if 1
	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		LDBG( "Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			LDBG(
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}
#endif
	return rc;
}
*/
/*******************************************************************************
*  Name: fts_power_init
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_power_init(struct fts_ts_data *data, bool on)
{
	int rc;

	if (!on)
	{
		LDBG( "fts_power_init false \n");
		goto pwr_deinit;
	}

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		LDBG( "Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FTS_VTG_MIN_UV, FTS_VTG_MAX_UV);
		if (rc) {
			LDBG( "Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
		rc = regulator_enable(data->vdd);
	}
#if 1
	data->vcc_i2c = regulator_get(&data->client->dev, "iovcc");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		LDBG( "Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FTS_I2C_VTG_MIN_UV, FTS_I2C_VTG_MAX_UV);
		if (rc) {
			LDBG( "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
		rc = regulator_enable(data->vcc_i2c);
	}
#endif
	return 0;
#if 1
reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FTS_VTG_MAX_UV);
#endif
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FTS_VTG_MAX_UV);

	regulator_put(data->vdd);
#if 1
	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FTS_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
#endif
	return 0;
}

/*******************************************************************************
*  Name: fts_ts_pinctrl_init
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
#ifdef MSM_NEW_VER
static int fts_ts_pinctrl_init(struct fts_ts_data *fts_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	fts_data->ts_pinctrl = devm_pinctrl_get(&(fts_data->client->dev));
	if (IS_ERR_OR_NULL(fts_data->ts_pinctrl)) {
		retval = PTR_ERR(fts_data->ts_pinctrl);
		dev_dbg(&fts_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	fts_data->pinctrl_state_active
		= pinctrl_lookup_state(fts_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(fts_data->pinctrl_state_active)) {
		retval = PTR_ERR(fts_data->pinctrl_state_active);
		dev_err(&fts_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	fts_data->pinctrl_state_suspend
		= pinctrl_lookup_state(fts_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(fts_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(fts_data->pinctrl_state_suspend);
		dev_err(&fts_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	fts_data->pinctrl_state_release
		= pinctrl_lookup_state(fts_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(fts_data->pinctrl_state_release)) {
		retval = PTR_ERR(fts_data->pinctrl_state_release);
		dev_dbg(&fts_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(fts_data->ts_pinctrl);
err_pinctrl_get:
	fts_data->ts_pinctrl = NULL;
	return retval;
}
#endif

#ifdef CONFIG_PM
/*******************************************************************************
*  Name: fts_ts_start
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_start(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	int err;
	#if 0
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = fts_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}
	#endif

	#ifdef MSM_NEW_VER
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
				data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Cannot get active pinctrl state\n");
	}
	#endif
#if 0
	err = fts_gpio_configure(data, true);
	if (err < 0) {
		LDBG(
			"failed to put gpios in resue state\n");
		goto err_gpio_configuration;
	}
#endif

	// add by leo ++
	err = gpio_direction_output(data->pdata->reset_gpio, 0);
	if(err)
	{
		LDBG("Failed to set reset_gpio direction low, error=%d\n", err);
		gpio_free(data->pdata->reset_gpio);
	}
	gpio_set_value(data->pdata->reset_gpio, 0);
	//msleep(20);
	udelay(200);
/* Mark by Tom for MTK without power_gpio
	err = gpio_direction_output(data->pdata->power_gpio, 1);
	if(err)
	{
		LDBG("Failed to set power_gpio direction high, error=%d\n", err);
		gpio_free(data->pdata->power_gpio);
	}
	//msleep(30);
	mdelay(2);
	// add by leo --
*/
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(data->pdata->soft_rst_dly);

//#if FTS_GESTRUE_EN
	if(!(private_ts->gesture & _GESTURE_ENABLE))
	{
	 	//enable_irq(data->client->irq);
		//ft5726_irq_enable(); Mark by Leo Focal
	}
//#endif
	queue_delayed_work(private_ts->ac_workqueue, &private_ts->touch_ac_work, 0.1 * HZ);
	data->suspended = false;

	return 0;

//err_gpio_configuration:
	#ifdef MSM_NEW_VER
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(dev, "Cannot get suspend pinctrl state\n");
	}
	#endif

	#if 0
	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err)
			dev_err(dev, "power off failed");
	} else {
		err = fts_power_on(data, false);
		if (err)
			dev_err(dev, "power off failed");
	}
	#endif

	return err;
}

/*******************************************************************************
*  Name: fts_ts_stop
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_stop(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2];
	int i;
	int value;

	LDBG("enter sleep mode\n");

	//disable_irq(data->client->irq);
	//ft5726_irq_disable(); Mark by Leo Focal

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FTS_REG_PMODE;
		txbuf[1] = FTS_PMODE_HIBERNATE;
		fts_i2c_write(data->client, txbuf, sizeof(txbuf));
	}

	#ifdef MSM_NEW_VER
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(dev, "Cannot get suspend pinctrl state\n");
	}
	#endif
#if 0	
	err = fts_gpio_configure(data, false);
	if (err < 0) {
		LDBG(
			"failed to put gpios in suspend state\n");
		goto gpio_configure_fail;
	}
#endif
	data->suspended = true;
	// add by leo for suspend issue ++
	for (i=0; i<2; i++)
	{
		value = gpio_get_value(private_ts->pdata->irq_gpio);
		printk(KERN_ERR "[ft5726] %s: touch interrupt pin value = %d, (retry %d times)\n", __func__, value, i);
		if(value > 0)
			break;
		else
			queue_work(private_ts->ts_workqueue, &private_ts->touch_event_work);
		msleep(100);
	}
	// add by leo for suspend issue --

	return 0;

//gpio_configure_fail:
	#ifdef MSM_NEW_VER
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Cannot get active pinctrl state\n");
	}
	#endif
/*	
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err)
			dev_err(dev, "power on failed");
	} else {
		err = fts_power_on(data, true);
		if (err)
			dev_err(dev, "power on failed");
	}
	*/
#if 0
pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
#endif
	//ft5726_irq_enable(); Mark by Leo Focal
	return 0;
}

/*******************************************************************************
*  Name: fts_ts_suspend
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_ts_suspend(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
	int err = 0;
	#endif

	LDBG("enter SUSPEND function\n");

#if FTS_GESTRUE_EN
	if(private_ts->gesture & _GESTURE_ENABLE)
	{
	      	fts_write_reg(fts_i2c_client, 0xd0, 0x01);
		if (fts_updateinfo_curr.CHIP_ID==0x54 || fts_updateinfo_curr.CHIP_ID==0x58 || fts_updateinfo_curr.CHIP_ID==0x86)
		{
		  	fts_write_reg(fts_i2c_client, 0xd1, 0xff);
			fts_write_reg(fts_i2c_client, 0xd2, 0xff);
			fts_write_reg(fts_i2c_client, 0xd5, 0xff);
			fts_write_reg(fts_i2c_client, 0xd6, 0xff);
			fts_write_reg(fts_i2c_client, 0xd7, 0xff);
			fts_write_reg(fts_i2c_client, 0xd8, 0xff);
		}

		data->suspended = true;
		LDBG("enter gesture mode\n");
	       return 0;
	}
#endif
	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR	
	if (fts_psensor_support_enabled() && data->pdata->psensor_support &&
		device_may_wakeup(dev) &&
		data->psensor_pdata->tp_psensor_opened) {

		err = enable_irq_wake(data->client->irq);
		if (err)
			LDBG("set_irq_wake failed\n");
		data->suspended = true;
		return err;
	}
	#endif

	return fts_ts_stop(dev);
}

/*******************************************************************************
*  Name: fts_ts_resume
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_ts_resume(struct device *dev)
{
	int err;
	struct fts_ts_data *data = dev_get_drvdata(dev);

	LDBG("enter RESUME function\n");

	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
	if (fts_psensor_support_enabled() && data->pdata->psensor_support &&
		device_may_wakeup(dev) &&
		data->psensor_pdata->tp_psensor_opened) {
		err = disable_irq_wake(data->client->irq);
		if (err)
			LDBG("disable_irq_wake failed\n",
				);
		data->suspended = false;
		return err;
	}
#endif

	err = fts_ts_start(dev);
	if (err < 0)
		return err;

	return 0;
}

static const struct dev_pm_ops fts_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = fts_ts_suspend,
	.resume = fts_ts_resume,
#endif
};
#else
/*******************************************************************************
*  Name: fts_ts_suspend
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_suspend(struct device *dev)
{
	return 0;
}
/*******************************************************************************
*  Name: fts_ts_resume
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_resume(struct device *dev)
{
	return 0;
}
#endif

#if defined(CONFIG_FB)
/*******************************************************************************
*  Name: fb_notifier_callback
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct fts_ts_data *fts_data =
		container_of(self, struct fts_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			fts_data && fts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			fts_ts_resume(&fts_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			fts_ts_suspend(&fts_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************************************
*  Name: fts_ts_early_suspend
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_ts_early_suspend(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler,
						   struct fts_ts_data,
						   early_suspend);

	fts_ts_suspend(&data->client->dev);
}

/*******************************************************************************
*  Name: fts_ts_late_resume
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_ts_late_resume(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler,
						   struct fts_ts_data,
						   early_suspend);

	fts_ts_resume(&data->client->dev);
}
#endif


#ifdef CONFIG_OF
/*******************************************************************************
*  Name: fts_get_dt_coords
*  Brief:
*  Input:
*  Output:
*  Return: 
*******************************************************************************/
static int fts_get_dt_coords(struct device *dev, char *name,
				struct fts_ts_platform_data *pdata)
{
	u32 coords[FTS_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);	
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FTS_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

/*******************************************************************************
*  Name: fts_parse_dt
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = fts_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	//if (rc && (rc != -EINVAL))
		//return rc;

	rc = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	// add by leo for testtest ++
/* Mark by Tom for MTK without power_gpio
	pdata->power_gpio = of_get_named_gpio_flags(np, "focaltech,power-gpio", 0, &pdata->power_gpio_flags);
	if(pdata->power_gpio < 0)
		return pdata->power_gpio;
	LDBG("pdata->power_gpio=%d \n", pdata->power_gpio);
	pdata->tpid_gpio = of_get_named_gpio_flags(np, "focaltech,tpid-gpio", 0, &pdata->tpid_gpio_flags);
	if(pdata->tpid_gpio < 0)
		return pdata->tpid_gpio;
	LDBG("pdata->tpid_gpio=%d \n", pdata->tpid_gpio);
	// add by leo for testtest --
*/
	pdata->i2c_pull_up = of_property_read_bool(np, "focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np, "focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "rst-gpio", 0, NULL);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;
	LDBG("rst_gpio=%d \n", pdata->reset_gpio);

	pdata->irq_gpio = of_get_named_gpio_flags(np, "int-gpio", 0, NULL);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
	LDBG("int_gpio=%d \n", pdata->irq_gpio);

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms", &temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms", &temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.AUTO_CLB = of_property_read_bool(np, "focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np, "focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np, "focaltech,ignore-id-check");

	pdata->psensor_support = of_property_read_bool(np,
						"focaltech,psensor-support");
	
	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np, "focaltech,button-map", button_map, num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	dev_err(dev, "fts_parse_dt done.\n");
	return 0;
}
#else
/*******************************************************************************
*  Name: fts_parse_dt
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

/*******************************************************************************
*  Name: fts_debug_addr_is_valid
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static bool fts_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

/*******************************************************************************
*  Name: fts_debug_data_set
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_data_set(void *_data, u64 val)
{
	struct fts_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (fts_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

/*******************************************************************************
*  Name: fts_debug_data_get
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_data_get(void *_data, u64 *val)
{
	struct fts_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (fts_debug_addr_is_valid(data->addr)) {
		rc = fts_read_reg(data->client, data->addr, &reg);
		if (rc < 0) {
			LDBG("FT read register 0x%x failed (%d)\n", data->addr, rc);
		} else {
			*val = reg;
		}
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, fts_debug_data_get, fts_debug_data_set, "0x%02llX\n");

/*******************************************************************************
*  Name: fts_debug_addr_set
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_addr_set(void *_data, u64 val)
{
	struct fts_ts_data *data = _data;

	if (fts_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

/*******************************************************************************
*  Name: fts_debug_addr_get
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_addr_get(void *_data, u64 *val)
{
	struct fts_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (fts_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, fts_debug_addr_get, fts_debug_addr_set, "0x%02llX\n");

/*******************************************************************************
*  Name: fts_debug_suspend_set
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_suspend_set(void *_data, u64 val)
{
	struct fts_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		fts_ts_suspend(&data->client->dev);
	else
		fts_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

/*******************************************************************************
*  Name: fts_debug_suspend_get
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_suspend_get(void *_data, u64 *val)
{
	struct fts_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, fts_debug_suspend_get, fts_debug_suspend_set, "%lld\n");

/*******************************************************************************
*  Name: fts_debug_dump_info
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_debug_dump_info(struct seq_file *m, void *v)
{
	struct fts_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

/*******************************************************************************
*  Name: debugfs_dump_info_open
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, fts_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

// add by leo ++
u8 *gt5726_test_ini=NULL;
static int ft5726_reload_test_ini(void)
{
	int i=0, len=0;
	struct file *filp=NULL;
	mm_segment_t old_fs;
	//u8 *temp=NULL;
	//u16 *raw_acc_info=NULL;

	LDBG("START ");

	gt5726_test_ini = (u8*)kmalloc(sizeof(u8)*4000, GFP_KERNEL);
	memset(gt5726_test_ini, '\n', sizeof(u8)*4000);

	filp=filp_open(TEST_INI_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if (IS_ERR_OR_NULL(filp))
	{
		LDBG("TEST_INI_FILE_PATH Open Failed \n");
		return -ENOENT;
	}

	if(filp->f_op != NULL && filp->f_op->read != NULL)
	{
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		len = filp->f_op->read(filp, gt5726_test_ini, 4000, &filp->f_pos);
		set_fs(old_fs);
	}
	LDBG("len=%d\n",len);
	filp_close(filp,NULL);

	for(i=0; i<4000; i++)
	{
		//if(gt5726_test_ini[i]=='\n')
			//LDBG("\n");
		
		//GuitarTestPlatformINI[i]=raw_acc_info[i];
		LDBG("%c", gt5726_test_ini[i]);
	}
	LDBG("\n");

	kfree(gt5726_test_ini);
	//kfree(raw_acc_info);
	return 0;
}

// 0: TYPE_NONE
// other: CHARGER IN
#define TYPE_NONE	0
#define AC_IN	1
#define PC_IN	2
#define POWERBANK_IN	3

static void ft5726_send_ac_cmd(struct work_struct *work)
{
	s32 usb_in = 0x1;
	s32 usb_out = 0x0;
	u8 state;
	int ret =0;

	switch(usb_state)
 	{
  		case TYPE_NONE:
   			LDBG("TYPE_NONE\n");
			ret = fts_write_reg(private_ts->client, FTS_USB_PLUG_IN, usb_out);
			if (ret < 0)
				LDBG("write FTS_USB_PLUG_IN fail, ret =%d \n", ret);
			break;

    		case AC_IN:
    			LDBG("AC_IN\n");
			ret = fts_write_reg(private_ts->client, FTS_USB_PLUG_IN, usb_in);
                     if (ret < 0)
                        	LDBG("write FTS_USB_PLUG_IN fail, ret =%d \n", ret);
			break;

    		case PC_IN:
     			LDBG("PC_IN\n");
                     ret = fts_write_reg(private_ts->client, FTS_USB_PLUG_IN, usb_in);
                     if (ret < 0)
                     	LDBG("write FTS_USB_PLUG_IN fail, ret =%d \n", ret);
			break;

     		case POWERBANK_IN:
    			LDBG("POWERBANK_IN\n");
	         	ret = fts_write_reg(private_ts->client, FTS_USB_PLUG_IN, usb_in);
                    	if (ret < 0)
                    		LDBG("write FTS_USB_PLUG_IN fail, ret =%d \n", ret);
			break;

 		default:
			LDBG("wrong cable type ..\n");
                     break;
	}

	ret = fts_read_reg(private_ts->client, FTS_USB_PLUG_IN, &state);
	if(ret < 0)
		LDBG("read FTS_USB_PLUG_IN fail, ret =%d \n", ret);
	
	return;
}

int ft5726_cable_status_handler(int state)
{
	if(is_obe_lcm == 1) {
		return 0;
	}

	if(is_probe_success == 0) {
		LDBG("Not yet probe completed\n");
		return 0;
	}

	if(private_ts->suspended) {
		LDBG("Skip cable status notifier when system suspend\n");
		return 0;
	}

	LDBG("cable state = %d\n", state);
	usb_state = state;

	queue_delayed_work(private_ts->ac_workqueue, &private_ts->touch_ac_work, 0.1 * HZ);
	return 0;
}
EXPORT_SYMBOL(ft5726_cable_status_handler);

void ft5726_irq_enable(void)
{
        unsigned long irqflags = 0;

        spin_lock_irqsave(&private_ts->irq_lock, irqflags);
        if (irq_is_disable)
        {
                enable_irq(private_ts->client->irq);
                irq_is_disable = 0;
		//LDBG("Enable irq\n");
        }
        spin_unlock_irqrestore(&private_ts->irq_lock, irqflags);

        return;
}

void ft5726_irq_disable(void)
{
        unsigned long irqflags;

        spin_lock_irqsave(&private_ts->irq_lock, irqflags);
        if (!irq_is_disable)
        {
                irq_is_disable = 1;
                disable_irq_nosync(private_ts->client->irq);
		//LDBG("Disable irq\n");
        }
        spin_unlock_irqrestore(&private_ts->irq_lock, irqflags);

        return;
}

static ssize_t ft5726_proc_gesture_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	LDBG("START\n");

	LDBG("ts->gesture = 0x%02x \n", private_ts->gesture);
	return 0;
}

static ssize_t ft5726_proc_gesture_write(struct file *filp, const char __user *buf, size_t count, loff_t *off)
{
	LDBG("GESTURE_DOUBLE_CLICK = %c, GESTURE_ENABLE = %c, GESTURE_W = %c, GESTURE_S = %c, GESTURE_E = %c, GESTURE_C = %c, GESTURE_Z = %c, GESTURE_V = %c \n",
	        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

	if((buf[0] == '0') && (buf[1] == '0'))
	{
		private_ts->gesture = (private_ts->gesture & (~_GESTURE_ENABLE));
		LDBG("Gesture Disable (private_ts->gesture = 0x%02x)\n", private_ts->gesture);
	}
	else
	{
		if(buf[0] != '0')  // double click
			private_ts->gesture = (private_ts->gesture | _GESTURE_DOUBLE_CLICK);
		else
			private_ts->gesture = (private_ts->gesture & (~_GESTURE_DOUBLE_CLICK));

		if(buf[2] != '0') // W
			private_ts->gesture = (private_ts->gesture | _GESTURE_W);
		else
			private_ts->gesture = (private_ts->gesture & (~_GESTURE_W));

		if(buf[3] != '0') // S
			private_ts->gesture = (private_ts->gesture | _GESTURE_S);
		else
			private_ts->gesture = (private_ts->gesture & (~_GESTURE_S));

		if(buf[4] != '0') // e
			private_ts->gesture = (private_ts->gesture | _GESTURE_E);
		else
			private_ts->gesture = (private_ts->gesture & (~_GESTURE_E));

		if(buf[5] != '0') // C
			private_ts->gesture = (private_ts->gesture | _GESTURE_C);
		else
			private_ts->gesture = (private_ts->gesture & (~_GESTURE_C));

		if(buf[6] != '0') // Z
			private_ts->gesture = (private_ts->gesture | _GESTURE_Z);
		else
			private_ts->gesture = (private_ts->gesture & (~_GESTURE_Z));

		if(buf[7] != '0') // V
			private_ts->gesture = (private_ts->gesture | _GESTURE_V);
		else
			private_ts->gesture = (private_ts->gesture & (~_GESTURE_V));

		private_ts->gesture = (private_ts->gesture | _GESTURE_ENABLE);
		LDBG("Gesture Enable (private_ts->gesture = 0x%02x) \n", private_ts->gesture);
	}

	return count;
}

static const struct file_operations ft5726_gesture_fops =
{
	.owner = THIS_MODULE,
	.read = ft5726_proc_gesture_read,
	.write = ft5726_proc_gesture_write,
};

static void ft5726_create_proc_gesture_file(void)
{
	ft5726_proc_gesture_file = proc_create(FT5726_PROC_GESTURE_FILE, 0666, NULL, &ft5726_gesture_fops);
	if(ft5726_proc_gesture_file) {
		LDBG("proc Gesture file create sucessed\n");
	} else {
		LDBG("proc Gesture file create failed\n");
	}
	return;
}

static void ft5726_remove_proc_gesture_file(void)
{
	extern struct proc_dir_entry proc_root;
	LDBG("proc Gesture file removed.\n");
	remove_proc_entry(FT5726_PROC_GESTURE_FILE, &proc_root);

	return;
}

static ssize_t ft5726_proc_tp_debug_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	LDBG("START\n");

	ft5726_debug = !ft5726_debug;
	msleep(30);
	LDBG("ft5726_debug = %d\n", ft5726_debug);
	return 0;
}

static ssize_t ft5726_proc_tp_debug_write(struct file *filp, const char __user *buf, size_t count, loff_t *off)
{
	int err = 0;

	LDBG("debug_function START\n");

	if((buf[0] == 'd') && (buf[1] == 'e') && (buf[2] == 'b') && (buf[3] == 'u') && (buf[4] == 'g'))
	{
		ft5726_debug = buf[6] - 48;
		//LDBG("buf[6] = %d\n", buf[6]);
		LDBG("ft5726_debug = %d\n", ft5726_debug);
	}
	else if((buf[0] == 'i') && (buf[1] == 'r') && (buf[2] == 'q'))
	{
		if(buf[4] == '0')
		{
			LDBG("disable_irq\n");
			//disable_irq(private_ts->client->irq);
			ft5726_irq_disable();
		}
		else if(buf[4] == '1')
		{
			LDBG("enable_irq\n");
			//enable_irq(private_ts->client->irq);
			ft5726_irq_enable();
		}
		LDBG("irq_is_disable = %d\n", irq_is_disable);
	}
	else if((buf[0] == 'r') && (buf[1] == 's') && (buf[2] == 't') && (buf[3] == 'p') && (buf[4] == 'i') && (buf[5] == 'n'))
	{
		LDBG("get rst_pin = %d\n", gpio_get_value(private_ts->pdata->reset_gpio));

		if(buf[7] == '0')
		{
			gpio_direction_output(private_ts->pdata->reset_gpio, 0);
			LDBG("set rst_pin. = %s\n", "Low");
		}
		else if(buf[7] == '1')
		{
			gpio_direction_output(private_ts->pdata->reset_gpio, 1);
			LDBG("set rst_pin. = %s\n", "High");
		}
		else
		{
			gpio_direction_input(private_ts->pdata->reset_gpio);
			LDBG("set rst_pin. = %s\n", "Input");
		}
	}
	else if((buf[0] == 'i') && (buf[1] == 'n') && (buf[2] == 't') && (buf[3] == 'r') && (buf[4] == 'p') && (buf[5] == 'i') && (buf[6] == 'n'))
	{
		LDBG("get intr_pin = %d, irq_gpio [%d] , gpio_to_irq(irq_gpio) [%d]\n", gpio_get_value(private_ts->pdata->irq_gpio) , 
		private_ts->pdata->irq_gpio , gpio_to_irq(private_ts->pdata->irq_gpio));

		if(buf[8] == '0')
		{
			gpio_direction_output(private_ts->pdata->irq_gpio, 0);
			LDBG("set intr_pin. = %s\n", "Low");
		}
		else if(buf[8] == '1')
		{
			gpio_direction_output(private_ts->pdata->irq_gpio, 1);
			LDBG("set intr_pin. = %s\n", "High");
		}
		else
		{
			gpio_direction_input(private_ts->pdata->irq_gpio);
			LDBG("set intr_pin. = %s\n", "Input");
		}
	}
	else if((buf[0] == 'p') && (buf[1] == 'o') && (buf[2] == 'w') && (buf[3] == 'e') && (buf[4] == 'r') && (buf[5] == 'p') && (buf[6] == 'i') && (buf[7] == 'n'))
	{
		if(buf[9] == '0')
		{
			gpio_direction_output(private_ts->pdata->power_gpio, 0);
			LDBG("set power_pin. = %s\n", "Low");
		}
		else if(buf[9] == '1')
		{
			gpio_direction_output(private_ts->pdata->power_gpio, 1);
			LDBG("set power_pin. = %s\n", "High");
		}
		else if(buf[9] == '2')
		{
			gpio_direction_input(private_ts->pdata->power_gpio);
			LDBG("set power_pin. = %s\n", "Input");
		}
	}
	else if((buf[0] == 'r') && (buf[1] == 'e') && (buf[2] == 'c') && (buf[3] == 'v'))
	{
		err = fts_read_Touchdata(fts_wq_data);
		if(err < 0)
			LDBG("Received the packet Error.\n");
	}
	else if((buf[0] == 'r') && (buf[1] == 'e') && (buf[2] == 'l') && (buf[3] == 'o') && (buf[4] == 'a') && (buf[5] == 'd'))
	{
		//if(buf[7] == '1')
		//{
			LDBG("reload... %s\n", buf);

		//}
	}
	else if((buf[0] == 't') && (buf[1] == 'e') && (buf[2] == 's') && (buf[3] == 't') && (buf[4] == 't') && (buf[5] == 'p'))
	{
		if(buf[7] == '1')
		{
			LDBG("start test tp ... \n");
			start_test_tp();
		}
	}

	if((buf[0] == 's') && (buf[1] == 'u') && (buf[2] == 's') && (buf[3] == 'p') && (buf[4] == 'e') && (buf[5] == 'n') && (buf[6] == 'd'))
	{
		if(buf[8] == '0')
		{
			LDBG("fts_ts_suspend ... \n");
			fts_ts_suspend(&private_ts->client->dev);
		}
		else if(buf[8] == '1')
		{
			LDBG("fts_ts_resume ... \n");
			fts_ts_resume(&private_ts->client->dev);
		}
	}
	// JUST FOR TEST
	if((buf[0] == 'g') && (buf[1] == 'e') && (buf[2] == 's') && (buf[3] == 't') && (buf[4] == 'u') && (buf[5] == 'r') && (buf[6] == 'e'))
	{
		LDBG("debug_function = testtest \n");

		if(buf[8] == '1')
		{
			LDBG("report gesture KEY_POWER test. \n");
			input_report_key(private_ts->input_dev, KEY_POWER, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_POWER, 0);
			input_sync(private_ts->input_dev);
		}
		else if(buf[8] == '2')
		{
			LDBG("report gesture KEY_POWER2 test. \n");
			input_report_key(private_ts->input_dev, KEY_POWER2, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_POWER2, 0);
			input_sync(private_ts->input_dev);
		}
		else if((buf[8] == 'd') && (buf[9] == 'c'))
		{
			LDBG("report gesture Double Click test. \n");
			input_report_key(private_ts->input_dev, KEY_F24, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_F24, 0);
			input_sync(private_ts->input_dev);
		}
		else if(buf[8] == 'w')
		{
			LDBG("report gesture W test. \n");
			input_report_key(private_ts->input_dev, KEY_F23, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_F23, 0);
			input_sync(private_ts->input_dev);
		}
		else if(buf[8] == 's')
		{
			LDBG("report gesture S test. \n");
			input_report_key(private_ts->input_dev, KEY_F22, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_F22, 0);
			input_sync(private_ts->input_dev);
		}
		else if(buf[8] == 'e')
		{
			LDBG("report gesture e test.");
			input_report_key(private_ts->input_dev, KEY_F21, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_F21, 0);
			input_sync(private_ts->input_dev);
		}
		else if(buf[8] == 'c')
		{
			LDBG("report gesture C test. \n");
			input_report_key(private_ts->input_dev, KEY_F20, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_F20, 0);
			input_sync(private_ts->input_dev);
		}
		else if(buf[8] == 'z')
		{
			LDBG("report gesture S test. \n");
			input_report_key(private_ts->input_dev, KEY_F19, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_F19, 0);
			input_sync(private_ts->input_dev);
		}
		else if(buf[8] == 'v')
		{
			LDBG("report gesture V test. \n");
			input_report_key(private_ts->input_dev, KEY_F18, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_F18, 0);
			input_sync(private_ts->input_dev);
		}
		else if(buf[8] == 'o')
		{
			LDBG("report gesture V test. \n");
			input_report_key(private_ts->input_dev, KEY_F17, 1);
			input_sync(private_ts->input_dev);
			input_report_key(private_ts->input_dev, KEY_F17, 0);
			input_sync(private_ts->input_dev);
		}
		else
		{
			input_report_key(private_ts->input_dev, BTN_TOUCH, 1);
			input_report_abs(private_ts->input_dev, ABS_MT_POSITION_X, 400);
			input_report_abs(private_ts->input_dev, ABS_MT_POSITION_Y, 500);
			input_report_abs(private_ts->input_dev, ABS_MT_TOUCH_MAJOR, 65);
			input_report_abs(private_ts->input_dev, ABS_MT_WIDTH_MAJOR, 65);
			input_report_abs(private_ts->input_dev, ABS_MT_TRACKING_ID, 0);
			input_mt_sync(private_ts->input_dev);

			msleep(30);
			input_report_key(private_ts->input_dev, BTN_TOUCH, 0);
		}
	}
	return count;
}

static const struct file_operations ft5726_tp_debug_fops =
{
	.owner = THIS_MODULE,
	.read = ft5726_proc_tp_debug_read,
	.write = ft5726_proc_tp_debug_write,
};

static void ft5726_create_proc_tp_debug_file(void)
{
	ft5726_proc_tp_debug_file = proc_create(FT5726_PROC_DEBUG_FILE, 0666, NULL, &ft5726_tp_debug_fops);
	if(ft5726_proc_tp_debug_file) {
		LDBG("proc TP debug file create sucessed\n");
	} else {
		LDBG("proc TP debug file create failed\n");
	}
	return;
}

static void ft5726_remove_proc_tp_debug_file(void)
{
	extern struct proc_dir_entry proc_root;
	LDBG("proc debug file removed.\n");
	remove_proc_entry(FT5726_PROC_DEBUG_FILE, &proc_root);

	return;
}

static ssize_t ft5726_proc_tp_test_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	LDBG("START\n");
	start_test_tp();
	return 0;
}

static ssize_t ft5726_proc_tp_test_write(struct file *filp, const char __user *buf, size_t count, loff_t *off)
{
	LDBG("START\n");
	LDBG("ini: %s", buf);

	 ft5726_reload_test_ini();
	set_param_data(gt5726_test_ini);
	return count;
}

static const struct file_operations ft5726_tp_test_fops =
{
	.owner = THIS_MODULE,
	.read = ft5726_proc_tp_test_read,
	.write = ft5726_proc_tp_test_write,
};

static void ft5726_create_proc_tp_test_file(void)
{
	ft5726_proc_tp_test_file = proc_create(FT5726_PROC_TEST_FILE, 0666, NULL, &ft5726_tp_test_fops);
	if(ft5726_proc_tp_test_file) {
		LDBG("proc TP test file create sucessed\n");
	} else {
		LDBG("proc TP test file create failed\n");
	}
	return;
}

static void ft5726_remove_proc_tp_test_file(void)
{
	extern struct proc_dir_entry proc_root;
	LDBG("proc debug file removed.\n");
	remove_proc_entry(FT5726_PROC_TEST_FILE, &proc_root);

	return;
}

static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	switch(FT5726_TP_ID) 
	{
		case BIEL_OGS_BLACK:
			return sprintf(buf, "BIEL_OGS_BLACK_V%d.%d.%d\n", private_ts->fw_ver[0], private_ts->fw_ver[1], private_ts->fw_ver[2]);
			break;
		case BIEL_GFF_WHITE:
			return sprintf(buf, "BIEL_GFF_WHITE_V%d.%d.%d\n", private_ts->fw_ver[0], private_ts->fw_ver[1], private_ts->fw_ver[2]);
			break;
		case BIEL_OGS_WHITE:
			return sprintf(buf, "BIEL_OGS_WHITE_V%d.%d.%d\n", private_ts->fw_ver[0], private_ts->fw_ver[1], private_ts->fw_ver[2]);
			break;
		case LAIB_OGS_BLACK:
			return sprintf(buf, "LAIB_OGS_BLACK_V%d.%d.%d\n", private_ts->fw_ver[0], private_ts->fw_ver[1], private_ts->fw_ver[2]);
			break;
		default:
			return sprintf(buf, "FOCAL_V%d.%d.%d\n", private_ts->fw_ver[0], private_ts->fw_ver[1], private_ts->fw_ver[2]);
			break;
	}
}
static ssize_t touch_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ft5726_touch_status);
}

static void fts_fw_update_work(struct work_struct *work)
{
	mutex_lock(&fts_input_dev->mutex);

	ft5726_irq_disable();
#if GTP_ESD_PROTECT
	apk_debug_flag = 1;
#endif
	/*
	#define FOCAL_BIEL_OGS_BLACK_FW_PATH "FOCAL_BIEL_OGS_BLACK_FW.bin"
	#define FOCAL_BIEL_GFF_WHITE_FW_PATH "FOCAL_BIEL_GFF_WHITE_FW.bin"
	#define FOCAL_BIEL_OGS_WHITE_FW_PATH "FOCAL_BIEL_OGS_WHITE_FW.bin"
	#define FOCAL_LAIB_OGS_BLACK_FW_PATH "FOCAL_LAIB_OGS_BLACK_FW.bin"
	*/
	switch(FT5726_TP_ID) 
	{
		case BIEL_OGS_BLACK:
			fts_ctpm_fw_upgrade_with_app_file(fts_wq_data->client, FOCAL_BIEL_OGS_BLACK_FW_PATH);
			break;
		case BIEL_GFF_WHITE:
			fts_ctpm_fw_upgrade_with_app_file(fts_wq_data->client, FOCAL_BIEL_GFF_WHITE_FW_PATH);
			break;
		case BIEL_OGS_WHITE:
			fts_ctpm_fw_upgrade_with_app_file(fts_wq_data->client, FOCAL_BIEL_OGS_WHITE_FW_PATH);
			break;
		case LAIB_OGS_BLACK:
			fts_ctpm_fw_upgrade_with_app_file(fts_wq_data->client, FOCAL_LAIB_OGS_BLACK_FW_PATH);
			break;
		default:
			LDBG("Unknown FT5726_TP_ID [%d]\n", FT5726_TP_ID);
			break;
	}
#if GTP_ESD_PROTECT
	apk_debug_flag = 0;
#endif
	ft5726_irq_enable();

	fts_update_fw_ver(fts_wq_data);
	fts_update_fw_vendor_id(fts_wq_data);

	mutex_unlock(&fts_input_dev->mutex);
}

// add by leo --

/*******************************************************************************
*  Name: fts_ts_probe
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fts_ts_platform_data *pdata;
	
	#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
	struct fts_psensor_platform_data *psensor_pdata;
	struct input_dev *psensor_input_dev;
	#endif
	
	struct fts_ts_data *data;
	struct input_dev *input_dev;
	
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;
	
	/* Check is OBE LCM */
	int lcm_id_gpio = 0;
	int tp_id_0_gpio = 0;
	int tp_id_0 = 0;
	int tp_id_1_gpio = 0;
	int tp_id_1 = 0;


	LDBG("START v3 \n"); // add by leo for testtest
	
        lcm_id_gpio = of_get_named_gpio(client->dev.of_node, "lcmid0-gpio", 0);
        is_obe_lcm = gpio_get_value(lcm_id_gpio);
	
	LDBG("LCM ID GPIO [%d] = [%d] \n", lcm_id_gpio ,is_obe_lcm);
	if (is_obe_lcm == 1) {
		LDBG("Is CPT panel , LCM ID = [%d] , skip probe\n", is_obe_lcm);
		return 0;
	}

	/* Add by Tom Cheng for cat sys/kernel/android_touch/tp_id (self_test) */
        tp_id_0_gpio = of_get_named_gpio(client->dev.of_node, "tpid0-gpio", 0);
        tp_id_0 = gpio_get_value(tp_id_0_gpio);
        tp_id_1_gpio = of_get_named_gpio(client->dev.of_node, "tpid1-gpio", 0);
        tp_id_1 = gpio_get_value(tp_id_1_gpio);

	/* 
	 * 0 : 00 BIEL   OGS BLACK
	 * 1 : 01 BIEL   GFF WHITE
	 * 2 : 10 BIEL   OGS WHITE
	 * 3 : 11 LAIBAO OGS BLACK
	 */
	FT5726_TP_ID = (tp_id_0 == 0) ? ((tp_id_1 == 0) ? 0 : 1) : ((tp_id_1 == 0) ? 2 : 3);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct fts_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = fts_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			//return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	// add by leo ++
	FT5726_HW_ID = Read_HW_ID();

	LDBG("set reset_gpio direction low ... \n");
	err = gpio_request(pdata->reset_gpio, "focaltech-reset");
	if(err < 0)
		LDBG("Failed to request GPIO %d (focaltech-reset) error=%d\n", pdata->reset_gpio, err);

	err = gpio_direction_output(pdata->reset_gpio, 0);
	if(err)
	{
		LDBG("Failed to set reset_gpio direction low, error=%d\n", err);
		gpio_free(pdata->reset_gpio);
	}
	gpio_set_value(pdata->reset_gpio, 0);
	msleep(20);

/*  	Mark by Tom for MTK
	LDBG("set VDD direction high ... \n");
	err = gpio_request(pdata->power_gpio, "focaltech-power");
	if(err < 0)
		LDBG("Failed to request GPIO %d (focaltech-power) error=%d\n", pdata->power_gpio, err);

	err = gpio_direction_output(pdata->power_gpio, 1);
	if(err)
	{
		LDBG("Failed to set power_gpio direction high, error=%d\n", err);
		gpio_free(pdata->power_gpio);
	}
	msleep(30);
	//LDBG("check TP_ID ... \n");
	err = gpio_request(pdata->tpid_gpio, "focaltech-tpid");
	if(err < 0)
		LDBG("Failed to request GPIO %d (focaltech-tpid) error=%d\n", pdata->tpid_gpio, err);

	FT5726_TP_ID = gpio_get_value(pdata->tpid_gpio);
	LDBG("FT5726_TP_ID = %d \n", FT5726_TP_ID);
	// add by leo --
*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct fts_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	fts_wq_data = data;

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FTS_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FTS_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev, data->tch_data_len, GFP_KERNEL);
	
	if (!data->tch_data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	private_ts = data; // add by leo for testtest

	input_dev->name = "fts_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	spin_lock_init(&data->irq_lock);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches,0);
	//input_mt_init_slots(input_dev, pdata->num_max_touches);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0x0f, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xff, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}

	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "pdata->power_init power init failed");
			goto unreg_inputdev;
		}
	} else {
		LDBG("fts_power_init ... \n"); // add by leo for testtest
		err = fts_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "fts_power_init power init failed");
			goto unreg_inputdev;
		}
	}

	/*
	LDBG("power enable ... \n"); // add by leo for testtest
	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		LDBG("fts_power_on ... \n"); // add by leo for testtest
		err = fts_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}
	*/
	#ifdef MSM_NEW_VER
	err = fts_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
		}
	}
	#endif

	LDBG("configure touch gpio & power on by reset ... \n"); // add by leo for testtest

	err = fts_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&client->dev,
			"Failed to configure the gpios\n");
		goto err_gpio_req;
	}

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	INIT_WORK(&data->touch_event_work, fts_touch_irq_work);
	data->ts_workqueue = create_workqueue(FTS_WORKQUEUE_NAME);
	if (!data->ts_workqueue)
	{
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	#ifdef FTS_SYSFS_DEBUG
		LDBG("create sysfs nodes ... \n"); // add by leo for testtest
		fts_create_sysfs(client);
		/* Add by Tom for Create Node in sys/kernel/android_touch */
		LDBG("create sysfs nodes in sys/kernel/android_touch ... \n"); 
		fts_touch_sysfs_init();
	#endif

	// add by leo ++
	data->ts_update_workqueue = create_workqueue(FTS_UPDATE_WORKQUEUE_NAME);
	if (!data->ts_update_workqueue)
	{
		LDBG("create ft5726_wq workqueue failed\n");
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	INIT_DELAYED_WORK(&data->touch_update_work, fts_fw_update_work);

	data->ac_workqueue = create_workqueue(FTS_UPDATE_WORKQUEUE_NAME);
	if(!data->ac_workqueue)
	{
		LDBG("create ac_wq workqueue failed\n");
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	INIT_DELAYED_WORK(&data->touch_ac_work, ft5726_send_ac_cmd);

	LDBG("create proc nodes ... \n"); // add by leo for testtest
	ft5726_create_proc_gesture_file();
	ft5726_create_proc_tp_debug_file();
	ft5726_create_proc_tp_test_file();
	// add by leo --

	LDBG("check device ID ... \n"); // add by leo for testtest
	ft5726_touch_status = 1; // add by leo

	/* check the controller id */
	reg_addr = FTS_REG_ID;
	err = fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		ft5726_touch_status = 0; // add by leo
		//goto free_gpio;
	}

	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		LDBG("Unsupported controller\n");
		//goto free_gpio;
	}

	data->family_id = pdata->family_id;

	fts_i2c_client = client;
	fts_input_dev = input_dev;

	fts_get_upgrade_array();

	LDBG("pdata->irqflags [%d] \n", pdata->irqflags ); 
	LDBG("request touch interrupt, client->irq = %d, gpio_to_irq(pdata->irq_gpio) = %d ... \n", 
		client->irq, gpio_to_irq(pdata->irq_gpio)); // add by leo for testtest
	LDBG("client->name [%s] \n", client->name );
	/*
	err = request_threaded_irq(client->irq, NULL, fts_ts_interrupt,
				     IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND , 
				     client->name, data); 
				     */
	err = request_threaded_irq(client->irq, NULL, fts_ts_interrupt,
			IRQF_ONESHOT | IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING,
			client->dev.driver->name, data);

	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_gpio;
	}
	
	LDBG("enable_irq_wake ... \n"); 
	err = enable_irq_wake(client->irq);
	if (err)
		LDBG("set_irq_wake failed\n");
	
	LDBG("request touch interrupt, client->irq = %d, gpio_to_irq(pdata->irq_gpio) = %d ... \n", 
		client->irq, gpio_to_irq(pdata->irq_gpio)); // add by leo for testtest
	
	//disable_irq(client->irq);
	ft5726_irq_disable();
	
	LDBG("create debug nodes ... \n"); // add by leo for testtest
	
	data->dir = debugfs_create_dir(FTS_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}
	
	data->ts_info = devm_kzalloc(&client->dev, FTS_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	/*get some register information */
	reg_addr = FTS_REG_POINT_RATE;
	fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FTS_REG_THGROUP;
	err = fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	LDBG("read FW version ... \n"); // add by leo for testtest

	fts_update_fw_ver(data);
	fts_update_fw_vendor_id(data);

	/* add by Tom Cheng for auto touch FW update */
	LDBG("fw_vendor_id = %d \n", data->fw_vendor_id);
	LDBG("FW version check and auto-update (data->fw_ver[0] = %d) ... \n", data->fw_ver[0]);
	if(build_version == 1){
		LDBG("Skip touch FW update in eng mode ...\n");
	} else {
		switch(FT5726_TP_ID) 
		{
			case BIEL_OGS_BLACK:
				if(data->fw_ver[0] < 0x1B) {
					queue_delayed_work(data->ts_update_workqueue, &data->touch_update_work, 15 * HZ);
					LDBG("[BIEL_OGS_BLACK] force update to version: [0x%2d] -> [0x1B]\n", data->fw_ver[0]);
				} else {
					LDBG("[BIEL_OGS_BLACK][V%2d] check version pass, skip force FW update\n", data->fw_ver[0]);
				}
				break;
			case BIEL_GFF_WHITE:
				if(data->fw_ver[0] < 0x0E) {
					queue_delayed_work(data->ts_update_workqueue, &data->touch_update_work, 15 * HZ);
					LDBG("[BIEL_GFF_WHITE] force update to version: [0x%2d] -> [0x0E]\n", data->fw_ver[0]);
				} else {
					LDBG("[BIEL_GFF_WHITE][V%2d] check version pass, skip force FW update\n", data->fw_ver[0]);
				}
				break;
			case BIEL_OGS_WHITE:
				if(data->fw_ver[0] < 0x1B) {
					queue_delayed_work(data->ts_update_workqueue, &data->touch_update_work, 15 * HZ);
					LDBG("[BIEL_OGS_WHITE] force update to version: [0x%2d] -> [0x1B]\n", data->fw_ver[0]);
				} else {
					LDBG("[BIEL_OGS_WHITE][V%2d] check version pass, skip force FW update\n", data->fw_ver[0]);
				}
				break;
			case LAIB_OGS_BLACK:
				if(data->fw_ver[0] < 0x1C) {
					queue_delayed_work(data->ts_update_workqueue, &data->touch_update_work, 15 * HZ);
					LDBG("[LAIB_OGS_BLACK] force update to version: [0x%2d] -> [0x1C]\n", data->fw_ver[0]);
				} else {
					LDBG("[LAIB_OGS_BLACK][V%2d] check version pass, skip force FW update\n", data->fw_ver[0]);
				}
				break;
			default:
				LDBG("Unknown FT5726_TP_ID [%d]\n", FT5726_TP_ID);
				break;
		}
	}
	// add by leo for auto touch FW update --

	FTS_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

	#ifdef FTS_APK_DEBUG
		fts_create_apk_debug_channel(client);
	#endif

#if 0
	#ifdef FTS_SYSFS_DEBUG
		LDBG("create sysfs nodes ... \n"); // add by leo for testtest
		fts_create_sysfs(client);
	#endif
#endif

	#ifdef FTS_CTL_IIC
		if (fts_rw_iic_drv_init(client) < 0)	
		{
			LDBG("[FTS] create fts control iic driver failed\n");
		}
	#endif
	

	#if FTS_GESTRUE_EN
		fts_Gesture_init(input_dev);
		//init_para(720,1280,0,0,0);
	#endif

	/*
	#ifdef FTS_AUTO_UPGRADE
	LDBG("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(client);
	#endif 
	*/

//add by leo ++
	data->touch_sdev.name = TOUCH_SDEV_NAME;
	data->touch_sdev.print_name = touch_switch_name;
	data->touch_sdev.print_state = touch_switch_state;
	if(switch_dev_register(&data->touch_sdev) < 0)
	{
		LDBG("switch_dev_register failed!\n");
	}
//add by leo --

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n", err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
	data->early_suspend.suspend = fts_ts_early_suspend;
	data->early_suspend.resume = fts_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
	wake_lock_init(&int_wakelock, WAKE_LOCK_SUSPEND, "fts-touchscreen");

	//enable_irq(client->irq);
	ft5726_irq_enable();

	/* Add by Tom Cheng for USB Status Notify */
	is_probe_success = 1;

	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
	
#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
unregister_psensor_input_device:
	if (fts_psensor_support_enabled() && data->pdata->psensor_support)
		input_unregister_device(data->psensor_pdata->input_psensor_dev);
free_psensor_input_dev:
	if (fts_psensor_support_enabled() && data->pdata->psensor_support)
		input_free_device(data->psensor_pdata->input_psensor_dev);
free_psensor_pdata:
	if (fts_psensor_support_enabled() && data->pdata->psensor_support) {
		devm_kfree(&client->dev, psensor_pdata);
		data->psensor_pdata = NULL;
	}
irq_free:
	if ((fts_psensor_support_enabled() &&
		data->pdata->psensor_support))
		device_init_wakeup(&client->dev, 0);
	free_irq(client->irq, data);
#endif

free_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
exit_create_singlethread:
	LDBG("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
err_gpio_req:
	#ifdef MSM_NEW_VER
	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
			if (err)
				pr_err("failed to select relase pinctrl state\n");
		}
	}
	#endif
/*
	if (pdata->power_on)
		pdata->power_on(false);
	else
		fts_power_on(data, false);
*/
//pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		fts_power_init(data, false);
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	return err;
}

/*******************************************************************************
*  Name: fts_ts_remove
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *data = i2c_get_clientdata(client);

	cancel_work_sync(&data->touch_event_work);
	destroy_workqueue(data->ts_workqueue);

	debugfs_remove_recursive(data->dir);

#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
	if (fts_psensor_support_enabled() && data->pdata->psensor_support) {

		device_init_wakeup(&client->dev, 0);
		sensors_classdev_unregister(&data->psensor_pdata->ps_cdev);
		input_unregister_device(data->psensor_pdata->input_psensor_dev);
		devm_kfree(&client->dev, data->psensor_pdata);
		data->psensor_pdata = NULL;
	}
#endif
	
#ifdef FTS_APK_DEBUG
		fts_release_apk_debug_channel();
#endif

	// add by leo ++
	ft5726_remove_proc_gesture_file();
	ft5726_remove_proc_tp_debug_file();
	ft5726_remove_proc_tp_test_file();
	// add by leo --

#ifdef FTS_SYSFS_DEBUG
		fts_remove_sysfs(fts_i2c_client);
#endif


#ifdef FTS_CTL_IIC
		fts_rw_iic_drv_exit();
#endif


#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
/*
	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		fts_power_on(data, false);
*/
	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		fts_power_init(data, false);

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id fts_ts_id[] = {
	{"fts_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, fts_ts_id);

#ifdef CONFIG_OF
static struct of_device_id fts_match_table[] = {
	{ .compatible = "focaltech,fts",},
	{ },
};
#else
#define fts_match_table NULL
#endif

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		   .name = "fts_ts",
		   .owner = THIS_MODULE,
		   .of_match_table = fts_match_table,
#ifdef CONFIG_PM
		   .pm = &fts_ts_pm_ops,
#endif
		   },
	.id_table = fts_ts_id,
};

/*******************************************************************************
*  Name: fts_ts_init
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int __init fts_ts_init(void)
{

	LDBG("touch driver initialize \n"); 
        /* Add by Tom Cheng for check is BOE Panel */
	is_obe_lcm = gpio_get_value(958);
	LDBG("LCM ID GPIO [%d] = [%d] \n", 958 ,is_obe_lcm);
	if (is_obe_lcm == 1) {
		LDBG("Is CPT panel , LCM ID = [%d] , skip probe\n", is_obe_lcm);
		return 0;
	}

	/* Check is MOS */
	if(entry_mode != 1) {
		LDBG("Not in MOS, skip probe. No in entry_mode:%d\n",entry_mode);
		return 0;
	}

	return i2c_add_driver(&fts_ts_driver);
}

/*******************************************************************************
*  Name: fts_ts_exit
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL v2");
