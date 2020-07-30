/*!
 * @section LICENSE
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename    bmm050_driver.c
 * @date        2014/03/11
 * @id          "d123cc5"
 * @version     v2.8.1
 *
 * @brief       BMM050 Linux Driver
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif

#include "cust_mag.h"
#include "mag.h"
#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <linux/io.h>

#include "bmm050.h"
#include "bs_log.h"

/* sensor specific */
#define SENSOR_NAME "bmm050"

#define SENSOR_CHIP_ID_BMM (0x32)
#define CHECK_CHIP_ID_TIME_MAX   5

#define BMM_REG_NAME(name) BMM050_##name
#define BMM_VAL_NAME(name) BMM050_##name
#define BMM_CALL_API(name) bmm050_##name

#define BMM_I2C_WRITE_DELAY_TIME 5

#define BMM_DEFAULT_REPETITION_XY BMM_VAL_NAME(REGULAR_REPXY)
#define BMM_DEFAULT_REPETITION_Z BMM_VAL_NAME(REGULAR_REPZ)
#define BMM_DEFAULT_ODR BMM_VAL_NAME(REGULAR_DR)
/* generic */
#define BMM_MAX_RETRY_I2C_XFER (100)
#define BMM_MAX_RETRY_WAKEUP (5)
#define BMM_MAX_RETRY_WAIT_DRDY (100)

#define BMM_DELAY_MIN (1)
#define BMM_DELAY_DEFAULT (200)

#define MAG_VALUE_MAX (32767)
#define MAG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMM_SELF_TEST 1
#define BMM_ADV_TEST 2

#define BMM_OP_MODE_UNKNOWN (-1)

/*! Bosch sensor unknown place*/
#define BOSCH_SENSOR_PLACE_UNKNOWN (-1)
/*! Bosch sensor remapping table size P0~P7*/
#define MAX_AXIS_REMAP_TAB_SZ 8
#define Z380M_BMM050_DEFAULT_PLACE 4	// default P4

#ifdef CONFIG_BMM_USE_PLATFORM_DATA
struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	int place;
	int irq;
	int (*irq_gpio_cfg)(void);
};
#endif

/*!
 * we use a typedef to hide the detail,
 * because this type might be changed
 */
struct bosch_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};

struct bosch_sensor_data {
	union {
		int16_t v[3];
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

static const u8 odr_map[] = {10, 2, 6, 8, 15, 20, 25, 30};
static const long op_mode_maps[] = {
	BMM_VAL_NAME(NORMAL_MODE),
	BMM_VAL_NAME(FORCED_MODE),
	BMM_VAL_NAME(SUSPEND_MODE),
	BMM_VAL_NAME(SLEEP_MODE)
};


struct bmm050_data {
	struct bmm050 device;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_handler;
#endif

	atomic_t delay;
	/* whether the system in suspend state */
	atomic_t in_suspend;

	struct bmm050_mdata_s32 value;
	u8 enable:1;
	s8 op_mode:4;
	u8 odr;
	u8 rept_xy;
	u8 rept_z;

	s16 result_test;

	struct mutex mutex_power_mode;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct mutex mutex_odr;
	struct mutex mutex_rept_xy;
	struct mutex mutex_rept_z;

	struct mutex mutex_value;
#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	struct bosch_sensor_specific *bst_pd;
#endif
};
static struct bmm050_data *g_mag_data;

/* i2c operation for API */
static void bmm_delay(u32 msec);
static int bmm_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static int bmm_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

static void bmm_dump_reg(struct i2c_client *client);
static int bmm_wakeup(struct i2c_client *client);
static int bmm_check_chip_id(struct i2c_client *client);

static int bmm_pre_suspend(struct i2c_client *client);
static int bmm_post_resume(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmm_early_suspend(struct early_suspend *handler);
static void bmm_late_resume(struct early_suspend *handler);
#endif

static int bmm_restore_hw_cfg(struct i2c_client *client);

static int bmm050_local_init(void);
static int bmm050_remove(void);
static int bmm050_init_flag = -1; /* 1<==>OK -1 <==> fail */
static struct kobject *android_mag_kobj = NULL;	// Sys kobject variab

static const struct bosch_sensor_axis_remap
bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

//-------------------------------flag----------------------------------------------------
#define APS_TAG					"alp.D Mag : "
#define APS_ERR(fmt, args...)	printk(KERN_ERR APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)	printk(KERN_INFO APS_TAG fmt, ##args)
//---------------------------------------------------------------------------------------

static void bst_remap_sensor_data(struct bosch_sensor_data *data,
		const struct bosch_sensor_axis_remap *remap)
{
	struct bosch_sensor_data tmp;

	tmp.x = data->v[remap->src_x] * remap->sign_x;
	tmp.y = data->v[remap->src_y] * remap->sign_y;
	tmp.z = data->v[remap->src_z] * remap->sign_z;

	memcpy(data, &tmp, sizeof(*data));
}

static void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
		int place)
{
	/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= MAX_AXIS_REMAP_TAB_SZ))
		return;

	bst_remap_sensor_data(data, &bst_axis_remap_tab_dft[place]);
}

static void bmm_remap_sensor_data(struct bmm050_mdata_s32 *val,
		struct bmm050_data *client_data)
{
#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	struct bosch_sensor_data bsd;

	if (NULL == client_data->bst_pd)
		return;

	bsd.x = val->datax;
	bsd.y = val->datay;
	bsd.z = val->dataz;

	bst_remap_sensor_data_dft_tab(&bsd,
			client_data->bst_pd->place);

	val->datax = bsd.x;
	val->datay = bsd.y;
	val->dataz = bsd.z;
#else
	(void)val;
	(void)client_data;
#endif
}

static int bmm_check_chip_id(struct i2c_client *client)
{
	int err = -1;
	u8 chip_id = 0;
	u8 read_count = 0;

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), &chip_id, 1);
		APS_LOG("read chip id result: %#x", chip_id);

		if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMM) {
			mdelay(1);
		} else {
			err = 0;
			break;
		}
	}

	return err;
}

static void bmm_delay(u32 msec)
{
	mdelay(msec);
}

static inline int bmm_get_forced_drdy_time(int rept_xy, int rept_z)
{
	return  (145 * rept_xy + 500 * rept_z + 980 + (1000 - 1)) / 1000;
}


static void bmm_dump_reg(struct i2c_client *client)
{
#ifdef DEBUG
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);

	bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);
#endif
}

static int bmm_wakeup(struct i2c_client *client)
{
	int err = 0;
	int try_times = BMM_MAX_RETRY_WAKEUP;
	const u8 value = 0x01;
	u8 dummy;

	PINFO("waking up the chip...");

	mdelay(BMM_I2C_WRITE_DELAY_TIME);
	while (try_times) {
		err = bmm_i2c_write(client,
				BMM_REG_NAME(POWER_CNTL), (u8 *)&value, 1);
		mdelay(BMM_I2C_WRITE_DELAY_TIME);
		dummy = 0;
		err = bmm_i2c_read(client, BMM_REG_NAME(POWER_CNTL), &dummy, 1);
		if (value == dummy)
			break;

		try_times--;
	}

	PINFO("wake up result: %s, tried times: %d",
			(try_times > 0) ? "succeed" : "fail",
			BMM_MAX_RETRY_WAKEUP - try_times + 1);

	err = (try_times > 0) ? 0 : -1;

	return err;
}

/*i2c read routine for API*/
static int bmm_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMM_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMM_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			PERR("i2c bus read error");
			return -1;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
			return -1;

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BMM_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(BMM_I2C_WRITE_DELAY_TIME);
	}

	if (BMM_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

/*i2c write routine for */
static int bmm_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMM_USE_BASIC_I2C_FUNC
	s32 dummy;

#ifndef BMM_SMBUS
	u8 buffer[2];
#endif

	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMM_SMBUS
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
		reg_addr++;
		data++;
		if (dummy < 0) {
			PERR("error writing i2c bus");
			return -1;
		}

	}
	return 0;
#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buffer,
		},
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < BMM_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				mdelay(BMM_I2C_WRITE_DELAY_TIME);
			}
		}
		if (BMM_MAX_RETRY_I2C_XFER <= retry) {
			PERR("I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
#endif
}

static int bmm_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err = 0;
	err = bmm_i2c_read(g_mag_data->client, reg_addr, data, len);
	return err;
}

static int bmm_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err = 0;
	err = bmm_i2c_write(g_mag_data->client, reg_addr, data, len);
	return err;
}

/* this function exists for optimization of speed,
 * because it is frequently called */
static inline int bmm_set_forced_mode(struct i2c_client *client)
{
	int err = 0;

	/* FORCED_MODE */
	const u8 value = 0x02;
	err = bmm_i2c_write(client, BMM_REG_NAME(CONTROL), (u8 *)&value, 1);

	return err;
}

static void bmm_work_func(struct work_struct *work)
{
	unsigned long delay =
		msecs_to_jiffies(atomic_read(&g_mag_data->delay));

	mutex_lock(&g_mag_data->mutex_value);

	mutex_lock(&g_mag_data->mutex_op_mode);
	if (BMM_VAL_NAME(NORMAL_MODE) != g_mag_data->op_mode)
		bmm_set_forced_mode(g_mag_data->client);
	mutex_unlock(&g_mag_data->mutex_op_mode);

	BMM_CALL_API(read_mdataXYZ_s32)(&g_mag_data->value);
	bmm_remap_sensor_data(&g_mag_data->value, g_mag_data);

	input_report_abs(g_mag_data->input, ABS_X, g_mag_data->value.datax);
	input_report_abs(g_mag_data->input, ABS_Y, g_mag_data->value.datay);
	input_report_abs(g_mag_data->input, ABS_Z, g_mag_data->value.dataz);

	mutex_unlock(&g_mag_data->mutex_value);

	input_sync(g_mag_data->input);

	schedule_delayed_work(&g_mag_data->work, delay);
}


static int bmm_set_odr(struct i2c_client *client, u8 odr)
{
	int err = 0;

	err = BMM_CALL_API(set_datarate)(odr);
	mdelay(BMM_I2C_WRITE_DELAY_TIME);

	return err;
}

static int bmm_get_odr(struct i2c_client *client, u8 *podr)
{
	int err = 0;
	u8 value;

	err = BMM_CALL_API(get_datarate)(&value);
	if (!err)
		*podr = value;

	return err;
}

static ssize_t bmm_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", SENSOR_CHIP_ID_BMM);
}

static inline int bmm_get_op_mode_idx(u8 op_mode)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(op_mode_maps); i++) {
		if (op_mode_maps[i] == op_mode)
			break;
	}

	if (i < ARRAY_SIZE(op_mode_maps))
		return i;
	else
		return -1;
}


static int bmm_set_op_mode(struct bmm050_data *client_data, int op_mode)
{
	int err = 0;

	err = BMM_CALL_API(set_functional_state)(
			op_mode);

	if (BMM_VAL_NAME(SUSPEND_MODE) == op_mode)
		atomic_set(&client_data->in_suspend, 1);
	else
		atomic_set(&client_data->in_suspend, 0);

	return err;
}

static ssize_t bmm_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	u8 op_mode = 0xff;
	u8 power_mode;

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&g_mag_data->mutex_op_mode);
		BMM_CALL_API(get_functional_state)(&op_mode);
		mutex_unlock(&g_mag_data->mutex_op_mode);
	} else {
		op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	}

	mutex_unlock(&g_mag_data->mutex_power_mode);

	PDEBUG("op_mode: %d", op_mode);

	ret = sprintf(buf, "%d\n", op_mode);

	return ret;
}

static ssize_t bmm_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err = 0;
	int i;
	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	APS_LOG("bmm_store_op_mode %ld!!\n",op_mode);
	mutex_lock(&g_mag_data->mutex_power_mode);
	i = bmm_get_op_mode_idx(op_mode);
	if (i != -1) {
		mutex_lock(&g_mag_data->mutex_op_mode);
		if (op_mode != g_mag_data->op_mode) {
			if (BMM_VAL_NAME(FORCED_MODE) == op_mode) {
				// special treat of forced mode for optimization
				APS_LOG("bmm_set_forced_mode %ld!!\n",op_mode);
				err = bmm_set_forced_mode(g_mag_data->client);
			} else {
				APS_LOG("bmm_set_op_mode %ld!!\n",op_mode);
				err = bmm_set_op_mode(g_mag_data, op_mode);
			}

			if (!err) {
				if (BMM_VAL_NAME(FORCED_MODE) == op_mode)
					g_mag_data->op_mode =
						BMM_OP_MODE_UNKNOWN;
				else
					g_mag_data->op_mode = op_mode;
			}
		}
		mutex_unlock(&g_mag_data->mutex_op_mode);
	}
	else {
		APS_LOG("bmm_store_mode error return !!\n");
		err = -EINVAL;
	}
	mutex_unlock(&g_mag_data->mutex_power_mode);

	if (err)
		return err;
	else
		return count;
}

static ssize_t bmm_show_odr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	int err;
	u8 power_mode;

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&g_mag_data->mutex_odr);
		err = bmm_get_odr(g_mag_data->client, &data);
		mutex_unlock(&g_mag_data->mutex_odr);
	} else {
		err = -EIO;
	}
	mutex_unlock(&g_mag_data->mutex_power_mode);

	if (!err) {
		if (data < ARRAY_SIZE(odr_map))
			err = sprintf(buf, "%d\n", odr_map[data]);
		else
			err = -EINVAL;
	}

	return err;
}

static ssize_t bmm_store_odr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	unsigned char data;
	int err;
	u8 power_mode;
	int i;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		for (i = 0; i < ARRAY_SIZE(odr_map); i++) {
			if (odr_map[i] == data)
				break;
		}

		if (i < ARRAY_SIZE(odr_map)) {
			mutex_lock(&g_mag_data->mutex_odr);
			err = bmm_set_odr(g_mag_data->client, i);
			if (!err)
				g_mag_data->odr = i;

			mutex_unlock(&g_mag_data->mutex_odr);
		} else {
			err = -EINVAL;
		}
	} else {
		err = -EIO;
	}

	mutex_unlock(&g_mag_data->mutex_power_mode);
	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_xy(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	int err;
	u8 power_mode;

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&g_mag_data->mutex_rept_xy);
		err = BMM_CALL_API(get_repetitions_XY)(&data);
		mutex_unlock(&g_mag_data->mutex_rept_xy);
	} else {
		err = -EIO;
	}

	mutex_unlock(&g_mag_data->mutex_power_mode);

	if (err)
		return err;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bmm_store_rept_xy(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp = 0;
	int err;
	u8 data;
	u8 power_mode;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&g_mag_data->mutex_rept_xy);
		err = BMM_CALL_API(set_repetitions_XY)(data);
		if (!err) {
			mdelay(BMM_I2C_WRITE_DELAY_TIME);
			g_mag_data->rept_xy = data;
		}
		mutex_unlock(&g_mag_data->mutex_rept_xy);
	} else {
		err = -EIO;
	}
	mutex_unlock(&g_mag_data->mutex_power_mode);

	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_z(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	int err;
	u8 power_mode;

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&g_mag_data->mutex_rept_z);
		err = BMM_CALL_API(get_repetitions_Z)(&data);
		mutex_unlock(&g_mag_data->mutex_rept_z);
	} else {
		err = -EIO;
	}

	mutex_unlock(&g_mag_data->mutex_power_mode);

	if (err)
		return err;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bmm_store_rept_z(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp = 0;
	int err;
	u8 data;
	u8 power_mode;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&g_mag_data->mutex_rept_z);
		err = BMM_CALL_API(set_repetitions_Z)(data);
		if (!err) {
			mdelay(BMM_I2C_WRITE_DELAY_TIME);
			g_mag_data->rept_z = data;
		}
		mutex_unlock(&g_mag_data->mutex_rept_z);
	} else {
		err = -EIO;
	}
	mutex_unlock(&g_mag_data->mutex_power_mode);

	if (err)
		return err;

	return count;
}


static ssize_t bmm_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count;
	struct bmm050_mdata_s32 value = {0, 0, 0, 0, 0};

	BMM_CALL_API(read_mdataXYZ_s32)(&value);
	if (value.drdy) {
		bmm_remap_sensor_data(&value, g_mag_data);
		g_mag_data->value = value;
	} else
		PERR("data not ready");

	count = sprintf(buf, "%d %d %d\n",
			g_mag_data->value.datax,
			g_mag_data->value.datay,
			g_mag_data->value.dataz);
	PDEBUG("%d %d %d",
			g_mag_data->value.datax,
			g_mag_data->value.datay,
			g_mag_data->value.dataz);

	return count;
}


static ssize_t bmm_show_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bmm050_mdata value;
	int count;

	BMM_CALL_API(get_raw_xyz)(&value);

	count = sprintf(buf, "%hd %hd %hd\n",
			value.datax,
			value.datay,
			value.dataz);

	return count;
}


static ssize_t bmm_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;

	mutex_lock(&g_mag_data->mutex_enable);
	err = sprintf(buf, "%d\n", g_mag_data->enable);
	mutex_unlock(&g_mag_data->mutex_enable);
	return err;
}

static ssize_t bmm_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&g_mag_data->mutex_enable);
	if (data != g_mag_data->enable) {
		if (data) {
			schedule_delayed_work(&g_mag_data->work, msecs_to_jiffies(atomic_read(&g_mag_data->delay)));
		} else {
			cancel_delayed_work_sync(&g_mag_data->work);
		}

		g_mag_data->enable = data;
	}
	mutex_unlock(&g_mag_data->mutex_enable);

	return count;
}

static ssize_t bmm_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&g_mag_data->delay));
}

static ssize_t bmm_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMM_DELAY_MIN)
		data = BMM_DELAY_MIN;

	atomic_set(&g_mag_data->delay, data);

	return count;
}

static ssize_t bmm_show_test(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	err = sprintf(buf, "%d\n", g_mag_data->result_test);
	return err;
}

static ssize_t bmm_store_test(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	u8 dummy;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	/* the following code assumes the work thread is not running */
	if (BMM_SELF_TEST == data) {
		/* self test */
		err = bmm_set_op_mode(g_mag_data, BMM_VAL_NAME(SLEEP_MODE));
		mdelay(3);
		err = BMM_CALL_API(set_selftest)(1);
		mdelay(3);
		err = BMM_CALL_API(get_self_test_XYZ)(&dummy);
		g_mag_data->result_test = dummy;
	} else if (BMM_ADV_TEST == data) {
		/* advanced self test */
		err = BMM_CALL_API(perform_advanced_selftest)(
				&g_mag_data->result_test);
	} else {
		err = -EINVAL;
	}

	if (!err) {
		BMM_CALL_API(soft_reset)();
		mdelay(BMM_I2C_WRITE_DELAY_TIME);
		bmm_restore_hw_cfg(g_mag_data->client);
	}

	if (err)
		count = -1;

	return count;
}


static ssize_t bmm_show_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err = 0;
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	memcpy(buf, dbg_buf_str, BYTES_PER_LINE * 3);

	for (i = 0; i < BYTES_PER_LINE * 3 - 1; i++)
		dbg_buf_str[i] = '-';

	dbg_buf_str[i] = '\n';
	memcpy(buf + BYTES_PER_LINE * 3, dbg_buf_str, BYTES_PER_LINE * 3);


	bmm_i2c_read(g_mag_data->client, BMM_REG_NAME(CHIP_ID), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	memcpy(buf + BYTES_PER_LINE * 3 + BYTES_PER_LINE * 3,
			dbg_buf_str, 64 * 3);

	err = BYTES_PER_LINE * 3 + BYTES_PER_LINE * 3 + 64 * 3;
	return err;
}


static ssize_t bmm_show_place(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int place = Z380M_BMM050_DEFAULT_PLACE;

#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	if (NULL != g_mag_data->bst_pd)
		place = g_mag_data->bst_pd->place;
#endif
	return sprintf(buf, "%d\n", place);
}

static ssize_t bmm_show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	APS_LOG("show bmm status !!\n");
	return sprintf(buf, "%d\n", bmm050_init_flag);
}


static DEVICE_ATTR(chip_id, 0660, 
		bmm_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, 0660, 
		bmm_show_op_mode, bmm_store_op_mode);
static DEVICE_ATTR(odr, 0660, 
		bmm_show_odr, bmm_store_odr);
static DEVICE_ATTR(rept_xy, 0660, 
		bmm_show_rept_xy, bmm_store_rept_xy);
static DEVICE_ATTR(rept_z, 0660, 
		bmm_show_rept_z, bmm_store_rept_z);
static DEVICE_ATTR(value, 0660, 
		bmm_show_value, NULL);
static DEVICE_ATTR(raw, 0660, 
		bmm_show_raw, NULL);
static DEVICE_ATTR(enable, 0660, 
		bmm_show_enable, bmm_store_enable);
static DEVICE_ATTR(delay, 0660, 
		bmm_show_delay, bmm_store_delay);
static DEVICE_ATTR(test, 0660, 
		bmm_show_test, bmm_store_test);
static DEVICE_ATTR(reg, 0660, 
		bmm_show_reg, NULL);
static DEVICE_ATTR(place, 0660, 
		bmm_show_place, NULL);
static DEVICE_ATTR(state, 0660, bmm_show_state, NULL);

static struct attribute *bmm_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_odr.attr,
	&dev_attr_rept_xy.attr,
	&dev_attr_rept_z.attr,
	&dev_attr_value.attr,
	&dev_attr_raw.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_test.attr,
	&dev_attr_reg.attr,
	&dev_attr_place.attr,
	&dev_attr_state.attr,
	NULL
};


static struct attribute_group bmm_attribute_group = {
	.attrs = bmm_attributes
};


static int bmm_input_init(struct bmm050_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->input = dev;

	return 0;
}

static void bmm_input_destroy(struct bmm050_data *client_data)
{
	struct input_dev *dev = client_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int bmm_restore_hw_cfg(struct i2c_client *client)
{
	int err = 0;
	u8 value;
	int op_mode;

	mutex_lock(&g_mag_data->mutex_op_mode);
	err = bmm_set_op_mode(g_mag_data, BMM_VAL_NAME(SLEEP_MODE));

	if (bmm_get_op_mode_idx(g_mag_data->op_mode) != -1)
		err = bmm_set_op_mode(g_mag_data, g_mag_data->op_mode);

	op_mode = g_mag_data->op_mode;
	mutex_unlock(&g_mag_data->mutex_op_mode);

	if (BMM_VAL_NAME(SUSPEND_MODE) == op_mode)
		return err;

	PINFO("app did not close this sensor before suspend");

	mutex_lock(&g_mag_data->mutex_odr);
	BMM_CALL_API(set_datarate)(g_mag_data->odr);
	mdelay(BMM_I2C_WRITE_DELAY_TIME);
	mutex_unlock(&g_mag_data->mutex_odr);

	mutex_lock(&g_mag_data->mutex_rept_xy);
	err = bmm_i2c_write(client, BMM_REG_NAME(NO_REPETITIONS_XY),
			&g_mag_data->rept_xy, 1);
	mdelay(BMM_I2C_WRITE_DELAY_TIME);
	err = bmm_i2c_read(client, BMM_REG_NAME(NO_REPETITIONS_XY), &value, 1);
	PINFO("BMM_NO_REPETITIONS_XY: %02x", value);
	mutex_unlock(&g_mag_data->mutex_rept_xy);

	mutex_lock(&g_mag_data->mutex_rept_z);
	err = bmm_i2c_write(client, BMM_REG_NAME(NO_REPETITIONS_Z),
			&g_mag_data->rept_z, 1);
	mdelay(BMM_I2C_WRITE_DELAY_TIME);
	err = bmm_i2c_read(client, BMM_REG_NAME(NO_REPETITIONS_Z), &value, 1);
	PINFO("BMM_NO_REPETITIONS_Z: %02x", value);
	mutex_unlock(&g_mag_data->mutex_rept_z);

	mutex_lock(&g_mag_data->mutex_op_mode);
	if (BMM_OP_MODE_UNKNOWN == g_mag_data->op_mode) {
		bmm_set_forced_mode(client);
		PINFO("set forced mode after hw_restore");
		mdelay(bmm_get_forced_drdy_time(g_mag_data->rept_xy,
					g_mag_data->rept_z));
	}
	mutex_unlock(&g_mag_data->mutex_op_mode);


	PINFO("register dump after init");
	bmm_dump_reg(client);

	return err;
}

static int bmm050_m_enable(int en)
{
	int value=en;
	int err=0;

	mutex_lock(&g_mag_data->mutex_enable);
	if (value != g_mag_data->enable) {
		if (value) {
			schedule_delayed_work(&g_mag_data->work, msecs_to_jiffies(atomic_read(&g_mag_data->delay)));
		} else {
			cancel_delayed_work_sync(&g_mag_data->work);
		}
		g_mag_data->enable = value;
	}
	mutex_unlock(&g_mag_data->mutex_enable);
	return err;
}
static int bmm050_m_set_delay(u64 ns)
{
	int value = (int)ns/1000/1000;
	if (value < BMM_DELAY_MIN)
		value = BMM_DELAY_MIN;
	APS_LOG("bmm050_m_set_delay ns : %lld\n", ns);
	atomic_set(&g_mag_data->delay, value);
	value = msecs_to_jiffies(atomic_read(&g_mag_data->delay));
	APS_LOG("bmm050_m_set_delay : %d\n", value);
	return 0;
}
static int bmm050_m_open_report_data(int open)
{
	return 0;
}
static int bmm050_m_get_data(int *x , int *y, int *z, int *status)
{
	struct bmm050_mdata value;
//	int count;

	BMM_CALL_API(get_raw_xyz)(&value);
	*x = value.datax;
	*y = value.datay;
	*z = value.dataz;
	return 0;
}

static int bmm050_o_enable(int en)
{
	int value=en;
	int err=0;

	mutex_lock(&g_mag_data->mutex_enable);
	if (value != g_mag_data->enable) {
		if (value) {
			schedule_delayed_work(&g_mag_data->work, msecs_to_jiffies(atomic_read(&g_mag_data->delay)));
		} else {
			cancel_delayed_work_sync(&g_mag_data->work);
		}
		g_mag_data->enable = value;
	}
	mutex_unlock(&g_mag_data->mutex_enable);
	return err;
}
static int bmm050_o_set_delay(u64 ns)
{
	int value = (int)ns/1000/1000;
	if (value < BMM_DELAY_MIN)
		value = BMM_DELAY_MIN;
	APS_LOG("bmm050_o_set_delay ns : %lld\n", ns);
	atomic_set(&g_mag_data->delay, value);
	value = msecs_to_jiffies(atomic_read(&g_mag_data->delay));
	APS_LOG("bmm050_o_set_delay : %d\n", value);
	return 0;
}
static int bmm050_o_open_report_data(int open)
{
	return 0;
}
static int bmm050_o_get_data(int *x , int *y, int *z, int *status)
{
	struct bmm050_mdata value;
//	int count;

	BMM_CALL_API(get_raw_xyz)(&value);
	*x = value.datax;
	*y = value.datay;
	*z = value.dataz;
	return 0;
}

static int bmm_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	int dummy;
	struct mag_control_path ctl = {0};
	struct mag_data_path mag_data = {0};

	APS_LOG("bmm_probe start ++++\n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		goto exit_err_clean;
	}

	/* wake up the chip */
	APS_LOG("mag  wake up the chip !!\n");
	dummy = bmm_wakeup(client);
	if (dummy < 0) {
		PERR("Cannot wake up %s, I2C xfer error", SENSOR_NAME);
		err = -EIO;
		goto exit_err_clean;
	}

	APS_LOG("mag register dump after waking up\n");
	bmm_dump_reg(client);
	/* check chip id */
	APS_LOG("mag check chip id\n");
	err = bmm_check_chip_id(client);
	if (!err) {
		PNOTICE("Bosch Sensortec Device %s detected, i2c_addr: %#x",
				SENSOR_NAME, client->addr);
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		goto exit_err_clean;
	}

	g_mag_data = kzalloc(sizeof(struct bmm050_data), GFP_KERNEL);
	if (NULL == g_mag_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	i2c_set_clientdata(client, g_mag_data);
	g_mag_data->client = client;

	mutex_init(&g_mag_data->mutex_power_mode);
	mutex_init(&g_mag_data->mutex_op_mode);
	mutex_init(&g_mag_data->mutex_enable);
	mutex_init(&g_mag_data->mutex_odr);
	mutex_init(&g_mag_data->mutex_rept_xy);
	mutex_init(&g_mag_data->mutex_rept_z);
	mutex_init(&g_mag_data->mutex_value);

	/* input device init */
	APS_LOG("mag input device init\n");
	err = bmm_input_init(g_mag_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	APS_LOG("mag add attr-node\n");
	// sys init ++
	android_mag_kobj = kobject_create_and_add("android_compass", NULL);	
	err += sysfs_create_file(android_mag_kobj, &dev_attr_op_mode.attr);
	err += sysfs_create_file(android_mag_kobj, &dev_attr_odr.attr);
	err += sysfs_create_file(android_mag_kobj, &dev_attr_delay.attr);
	err += sysfs_create_file(android_mag_kobj, &dev_attr_value.attr);
	err += sysfs_create_file(android_mag_kobj, &dev_attr_raw.attr);
	err += sysfs_create_file(android_mag_kobj, &dev_attr_enable.attr);
	err += sysfs_create_file(android_mag_kobj, &dev_attr_reg.attr);
	err += sysfs_create_file(android_mag_kobj, &dev_attr_state.attr);
	if (err != 0) {
		APS_ERR("create mag attribute err = %d\n", err);
		goto exit_err_clean;
	}	
	// sys init --
	err = sysfs_create_group(&g_mag_data->input->dev.kobj, &bmm_attribute_group);

	if (err < 0)
		goto exit_err_sysfs;
/*
#ifdef CONFIG_BMM_USE_PLATFORM_DATA
	if (NULL != client->dev.platform_data) {
		g_mag_data->bst_pd = kzalloc(sizeof(*g_mag_data->bst_pd),
				GFP_KERNEL);

		if (NULL != g_mag_data->bst_pd) {
			memcpy(g_mag_data->bst_pd, client->dev.platform_data,
					sizeof(*g_mag_data->bst_pd));

			APS_LOG("platform data of bmm %s: place: %d, irq: %d",
					g_mag_data->bst_pd->name,
					g_mag_data->bst_pd->place,
					g_mag_data->bst_pd->irq);
		}
	}
#endif
*/

	// workqueue init
	INIT_DELAYED_WORK(&g_mag_data->work, bmm_work_func);
	atomic_set(&g_mag_data->delay, BMM_DELAY_DEFAULT);

	// hw init
	g_mag_data->device.bus_read = bmm_i2c_read_wrapper;
	g_mag_data->device.bus_write = bmm_i2c_write_wrapper;
	g_mag_data->device.delay_msec = bmm_delay;
	BMM_CALL_API(init)(&g_mag_data->device);

	bmm_dump_reg(client);
	APS_LOG("trimming_reg x1: %d y1: %d x2: %d y2: %d xy1: %d xy2: %d",
			g_mag_data->device.dig_x1,
			g_mag_data->device.dig_y1,
			g_mag_data->device.dig_x2,
			g_mag_data->device.dig_y2,
			g_mag_data->device.dig_xy1,
			g_mag_data->device.dig_xy2);

	APS_LOG("trimming_reg z1: %d z2: %d z3: %d z4: %d xyz1: %d",
			g_mag_data->device.dig_z1,
			g_mag_data->device.dig_z2,
			g_mag_data->device.dig_z3,
			g_mag_data->device.dig_z4,
			g_mag_data->device.dig_xyz1);

	g_mag_data->enable = 0;
	/* now it's power on which is considered as resuming from suspend */
	g_mag_data->op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	g_mag_data->odr = BMM_DEFAULT_ODR;
	g_mag_data->rept_xy = BMM_DEFAULT_REPETITION_XY;
	g_mag_data->rept_z = BMM_DEFAULT_REPETITION_Z;


	APS_LOG("bmm_set_op_mode !!\n");
	err = bmm_set_op_mode(g_mag_data, BMM_VAL_NAME(SUSPEND_MODE));
	if (err) {
		PERR("fail to init h/w of %s", SENSOR_NAME);
		err = -EIO;
		goto exit_err_sysfs;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	g_mag_data->early_suspend_handler.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_mag_data->early_suspend_handler.suspend = bmm_early_suspend;
	g_mag_data->early_suspend_handler.resume = bmm_late_resume;
	register_early_suspend(&g_mag_data->early_suspend_handler);
#endif

	APS_LOG("sensor %s probed successfully", SENSOR_NAME);
	APS_LOG("i2c_client: %p client_data: %p i2c_device: %p input: %p",client, g_mag_data, &client->dev, g_mag_data->input);

	ctl.is_use_common_factory = false;
	ctl.m_enable = bmm050_m_enable;
	ctl.m_set_delay  = bmm050_m_set_delay;
	ctl.m_open_report_data = bmm050_m_open_report_data;
	ctl.o_enable = bmm050_o_enable;
	ctl.o_set_delay  = bmm050_o_set_delay;
	ctl.o_open_report_data = bmm050_o_open_report_data;
	ctl.is_report_input_direct = false;
//	ctl.is_support_batch = g_mag_data->hw->is_batch_supported;
	err = mag_register_control_path(&ctl);
	if (err) {
		APS_ERR("register mag control path err\n");
		goto exit_err_sysfs;
	}

	mag_data.div_m = 1;
	mag_data.div_o = 1;
	mag_data.get_data_o = bmm050_o_get_data;
	mag_data.get_data_m = bmm050_m_get_data;

	err = mag_register_data_path(&mag_data);
	if (err) {
		APS_ERR("register data control path err\n");
		goto exit_err_sysfs;
	}


	bmm050_init_flag = 1;

	return 0;

exit_err_sysfs:
	if (err)
		bmm_input_destroy(g_mag_data);

exit_err_clean:
	if (err) {
		if (g_mag_data != NULL) {
#ifdef CONFIG_BMM_USE_PLATFORM_DATA
			if (NULL != g_mag_data->bst_pd) {
				kfree(g_mag_data->bst_pd);
				g_mag_data->bst_pd = NULL;
			}
#endif
			kfree(g_mag_data);
			g_mag_data = NULL;
		}
	}
	return err;
}

static int bmm_pre_suspend(struct i2c_client *client)
{
	int err = 0;
	PDEBUG("function entrance");

	mutex_lock(&g_mag_data->mutex_enable);
	if (g_mag_data->enable) {
		cancel_delayed_work_sync(&g_mag_data->work);
		PDEBUG("cancel work");
	}
	mutex_unlock(&g_mag_data->mutex_enable);

	return err;
}

static int bmm_post_resume(struct i2c_client *client)
{
	int err = 0;
	mutex_lock(&g_mag_data->mutex_enable);
	if (g_mag_data->enable) {
		schedule_delayed_work(&g_mag_data->work,
				msecs_to_jiffies(
					atomic_read(&g_mag_data->delay)));
	}
	mutex_unlock(&g_mag_data->mutex_enable);

	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmm_early_suspend(struct early_suspend *handler)
{
	u8 power_mode;
	PDEBUG("function entrance");

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		bmm_pre_suspend(g_mag_data->client);
		bmm_set_op_mode(g_mag_data, BMM_VAL_NAME(SUSPEND_MODE));
	}
	mutex_unlock(&g_mag_data->mutex_power_mode);

}

static void bmm_late_resume(struct early_suspend *handler)
{
	PDEBUG("function entrance");

	mutex_lock(&g_mag_data->mutex_power_mode);

	bmm_restore_hw_cfg(g_mag_data->client);
	/* post resume operation */
	bmm_post_resume(g_mag_data->client);

	mutex_unlock(&g_mag_data->mutex_power_mode);
}
#else
static int bmm_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	u8 power_mode;

	PDEBUG("function entrance");

	mutex_lock(&g_mag_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		err = bmm_pre_suspend(g_mag_data->client);
		err = bmm_set_op_mode(g_mag_data, BMM_VAL_NAME(SUSPEND_MODE));
	}
	mutex_unlock(&g_mag_data->mutex_power_mode);

	return err;
}

static int bmm_resume(struct i2c_client *client)
{
	int err = 0;
	PDEBUG("function entrance");

	mutex_lock(&g_mag_data->mutex_power_mode);
	err = bmm_restore_hw_cfg(g_mag_data->client);
	/* post resume operation */
	bmm_post_resume(g_mag_data->client);

	mutex_unlock(&g_mag_data->mutex_power_mode);

	return err;
}
#endif

void bmm_shutdown(struct i2c_client *client)
{
	mutex_lock(&g_mag_data->mutex_power_mode);
	bmm_set_op_mode(g_mag_data, BMM_VAL_NAME(SUSPEND_MODE));
	mutex_unlock(&g_mag_data->mutex_power_mode);
}

static int bmm_remove(struct i2c_client *client)
{
	int err = 0;
	if (NULL != g_mag_data) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&g_mag_data->early_suspend_handler);
#endif

		mutex_lock(&g_mag_data->mutex_op_mode);
		if (BMM_VAL_NAME(NORMAL_MODE) == g_mag_data->op_mode) {
			cancel_delayed_work_sync(&g_mag_data->work);
			PDEBUG("cancel work");
		}
		mutex_unlock(&g_mag_data->mutex_op_mode);

		err = bmm_set_op_mode(g_mag_data, BMM_VAL_NAME(SUSPEND_MODE));
		mdelay(BMM_I2C_WRITE_DELAY_TIME);

		sysfs_remove_group(&g_mag_data->input->dev.kobj,
				&bmm_attribute_group);
		bmm_input_destroy(g_mag_data);

#ifdef CONFIG_BMM_USE_PLATFORM_DATA
			if (NULL != g_mag_data->bst_pd) {
				kfree(g_mag_data->bst_pd);
				g_mag_data->bst_pd = NULL;
			}
#endif
		kfree(g_mag_data);
	}
	return err;
}

static const struct i2c_device_id bmm_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bmm_id);

#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,bmm050"},
	{},
};
#endif

static struct i2c_driver bmm050_i2c_driver = {
	.driver = {
//		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = mag_of_match,
#endif
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bmm_id,
	.probe = bmm_probe,
	.remove = bmm_remove,
	.shutdown = bmm_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = bmm_suspend,
	.resume = bmm_resume,
#endif
};

//----------------------------------added  for  MTK--------------------------------------
static struct mag_init_info bmm050_init_info = {
	.name = SENSOR_NAME,
	.init = bmm050_local_init,
	.uninit = bmm050_remove,
};

static int bmm050_local_init(void)
{
	if (i2c_add_driver(&bmm050_i2c_driver)) {
		APS_ERR("add driver bmm050 error !!\n");
		return -1;
	}
	if (-1 == bmm050_init_flag) {
		APS_ERR("add driver--bmm050_local_init check error\n");
		return -1;
	}

	return 0;
}
static int bmm050_remove(void)
{
	i2c_del_driver(&bmm050_i2c_driver);
	bmm050_init_flag = -1;

	return 0;
}
//--------------------------------- ------ End ------------------------------------------

static int __init BMM_init(void)
{
	return  mag_driver_add(&bmm050_init_info);	
//	return i2c_add_driver(&bmm_driver);
}

static void __exit BMM_exit(void)
{
//	i2c_del_driver(&bmm_driver);
}

MODULE_AUTHOR("contact@bosch.sensortec.com");
MODULE_DESCRIPTION("BMM MAGNETIC SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMM_init);
module_exit(BMM_exit);
