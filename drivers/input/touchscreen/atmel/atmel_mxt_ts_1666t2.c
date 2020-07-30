/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2014 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include "atmel_mxt_ts_1666t2.h"
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/fs.h>
#include <asm/segment.h>
#ifdef CONFIG_HAS_EARLYSUSPEND_DISABLE
#include <linux/earlysuspend.h>
#endif

#define CONFIG_PM_SCREEN_STATE_NOTIFIER
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define CONFIG_OF 1
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include <linux/power_supply.h>

// #include <linux/HWVersion.h>

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE	256

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_PROCI_ACTIVE_STYLUS_T63	63
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 100
#define MXT_SPT_AUXTOUCHCONFIG_T104	104
#define MXT_PROCI_ACTIVESTYLUS_T107	107
#define MXT_PROC_SYMBOLGESTUREPROCESSOR_T92 92
#define MXT_PROCI_TOUCHSEQUENCELOGGER_T93   93

/* MXT T92 status */
#define MXT_T92_ENABLE 		1
#define MXT_T92_DISABLE 	2

/* MXT T92 Configuration */
#define MXT_T92_CTRL_ENABLE  (1 << 0)
#define MXT_T92_CTRL_RPTEN   (1 << 1)

/* MXT T93 status */
#define MXT_T93_ENABLE  	1
#define MXT_T93_DISABLE  	2

/* MXT T93 Configuration */
#define MXT_T93_CTRL_ENABLE  (1 << 0)
#define MXT_T93_CTRL_RPTEN   (1 << 1)

/* MXT T107 status */
#define MXT_T107_ENABLE		1
#define MXT_T107_DISABLE		2

/* MXT T107 Configuration */
#define MXT_T107_CTRL_ENABLE (1 << 0)

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

/* MXT_T8 field */
struct t8_config {
	u8 byte0;
	u8 byte1;
	u8 byte2;
	u8 byte3;
	u8 byte4;
	u8 byte5;
	u8 byte6;
	u8 byte7;
	u8 byte8;
	u8 byte9;
	u8 byte10;
	u8 byte11;
} __packed;

/* MXT_T104 field */
struct t104_config {
	u8 byte0;
	u8 byte1;
	u8 byte2;
	u8 byte3;
	u8 byte4;
	u8 byte5;
	u8 byte6;
	u8 byte7;
	u8 byte8;
	u8 byte9;
	u8 byte10;
} __packed;

/* MXT_T107 field */
struct t107_config {
	u8 byte97;
} __packed;

#define MXT_STYLUS_DISABLE		0
#define MXT_STYLUS_ENABLE		1

#define MXT_POWER_CFG_RUN			0
#define MXT_POWER_CFG_DEEPSLEEP		1
#define MXT_POWER_CFG_POWERSAVE		2
#define MXT_POWER_CFG_GESTURE		3

/* MXT_GEN_ACQUISITIONCONFIG_T8 field */
#define MXT_MEASALLOW_MUTUALTCH		1 << 0
#define MXT_MEASALLOW_SELFTCH		1 << 1
#define MXT_MEASALLOW_HOVER			1 << 2
#define MXT_MEASALLOW_SELFPROX		1 << 3
#define MXT_MEASALLOW_ACTVSTY		1 << 4


/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)

struct t9_range {
	u16 x;
	u16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	(1 << 0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN      (1 << 6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	(1 << 0)

/* T47 Stylus */
#define MXT_TOUCH_MAJOR_T47_STYLUS	1

/* Hardcoded touch major for stylus */
#define MXT_TOUCH_MAJOR_STYLUS	1

/* T63 Stylus */
#define MXT_T63_STYLUS_PRESS	(1 << 0)
#define MXT_T63_STYLUS_RELEASE	(1 << 1)
#define MXT_T63_STYLUS_MOVE		(1 << 2)
#define MXT_T63_STYLUS_SUPPRESS	(1 << 3)

#define MXT_T63_STYLUS_DETECT	(1 << 4)
#define MXT_T63_STYLUS_TIP		(1 << 5)
#define MXT_T63_STYLUS_ERASER	(1 << 6)
#define MXT_T63_STYLUS_BARREL	(1 << 7)

#define MXT_T63_STYLUS_PRESSURE_MASK	0x3F

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CTRL_ENABLE	(1 << 0)
#define MXT_T100_CTRL_RPTEN		(1 << 1)
#define MXT_T100_CTRL_DISSCRMSG	(1 << 2)
#define MXT_T100_CTRL_SCANEN	(1 << 7)
#define MXT_T100_CFG_SWITCHXY	(1 << 5)

#define MXT_T100_TCHAUX_VECT	(1 << 0)
#define MXT_T100_TCHAUX_AMPL	(1 << 1)
#define MXT_T100_TCHAUX_AREA	(1 << 2)

#define MXT_T100_DETECT		(1 << 7)
#define MXT_T100_TYPE_MASK	0x70
#define MXT_T100_TYPE_STYLUS	0x20

enum t100_type {
	MXT_T100_TYPE_FINGER			= 1,
	MXT_T100_TYPE_PASSIVE_STYLUS	= 2,
	MXT_T100_TYPE_ACTIVE_STYLUS		= 3,
	MXT_T100_TYPE_HOVERING_FINGER	= 4,
	MXT_T100_TYPE_GLOVE				= 5,
	MXT_T100_TYPE_LARGE_TOUCH		= 6,
};

/* Gen2 Active Stylus */
#define MXT_T107_STYLUS_STYAUX		37
#define MXT_T107_STYLUS_STYAUX_PRESSURE	(1 << 0)
#define MXT_T107_STYLUS_STYAUX_BATLVL	(1 << 1)

#define MXT_T107_STYLUS_HOVER		(1 << 0)
#define MXT_T107_STYLUS_TIPSWITCH	(1 << 1)
#define MXT_T107_STYLUS_BUTTON0		(1 << 2)
#define MXT_T107_STYLUS_BUTTON1		(1 << 3)
#define MXT_T107_STYLUS_PRESSURE	1

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	3000	/* msec */
#define MXT_FW_CHG_TIMEOUT	600	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_REGULATOR_DELAY	150	/* msec */
#define MXT_CHG_DELAY	        100	/* msec */
#define MXT_POWERON_DELAY	150	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	(1 << 5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_PIXELS_PER_MM	20

#define DEBUG_MSG_MAX		200

//#define TP_ENABLE

#ifdef TP_ENABLE
#define TOUCH_ENABLE_GPIO	46
#endif
#define TOUCH_INT_GPIO		13
#define TOUCH_RST_GPIO		12
#define STYLUS_COUNT		2500

#define DRIVER_VERSION	"B.4.8"
#define MXT_CONFIG_NAME_1666T2 "mTX1666T2_cfg.raw"
#define MXT_CONFIG_NAME_1666T2_SECOND "mTX1666T2_cfg_second.raw"
#define MXT_FIRMWARE_NAME_1666T2 "maxtouch.fw"

#define TOUCH_SDEV_NAME "touch"
#define NEWEST_FW_VERSION 0x22
#define NEWEST_FW_BUILD 0xAA

#define MXT_VDD_VTG_MIN_UV	2600000
#define MXT_VDD_VTG_MAX_UV	3300000
#define MXT_AVDD_VTG_MIN_UV	1800000
#define MXT_AVDD_VTG_MAX_UV	1800000

#define INIT_COMPLETION(x)      ((x).done = 0)

#define mxt_double_tap 1
#define mxt_gesture 1
#define mxt_stylus 1
/* mini porting
#define mxt_pen 0
*/
#ifdef mxt_double_tap
#define MXT_PROC_DOUBLE_TAP_FILE	"mxt_double_tap"
static struct proc_dir_entry *mxt_proc_double_tap_file = NULL;
#endif


#ifdef mxt_gesture
#define MXT_PROC_GESTURE_FILE		"mxt_gesture"
static struct proc_dir_entry *mxt_proc_gesture_file = NULL;
#endif

#ifdef mxt_stylus
#define MXT_PROC_STYLUS_FILE		"mxt_stylus"
static struct proc_dir_entry *mxt_proc_stylus_file = NULL;
#endif

// add by leo for testtest ++
#define MXT_PROC_LOGTOOL_FILE		"touch_debug_log"
/* defined but not used
static struct proc_dir_entry *mxt_proc_logtool_file = NULL;
*/
// add by leo for testtes --

/* Add by Tom Cheng for print debug log */
#define LDBG(s,args...) {printk(KERN_ERR "[ATMEL] : func [%10s], line [%d], ",__func__,__LINE__); printk(s,## args);}


struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

struct stylus_data {
	u16 attach;
	u16 active;
};

static struct workqueue_struct *maxtouch_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND_DISABLE
static void mxt_early_suspend(struct early_suspend *es);
static void mxt_late_resume(struct early_suspend *es);
#endif
static bool irq_enable = true;
static bool irq_wake_enable = false;
static bool touch_ic_status = false;
static bool power_save_mode = false;
#ifdef mxt_pen
static	u8 active_pen_mode = 0;
static	u8 old_batlvl = 3;
static	u8 current_batlvl = 0;
static	u16 batlvl_count = 0;
#endif
static	u8 active_rep = 7;
static	u8 idle_rep = 20;
static	u8 gesture_active_rep = 15;
static	u8 gesture_idle_rep = 50;
extern int entry_mode; // add by josh for skip COS/POS
extern int build_version; // add by josh for skip eng poweroff
/*
extern bool hall_trigger_suspend;
extern int Read_HW_ID(void);
*/
//int entry_mode = 1; // add by josh for skip COS/POS
//int build_version = 1; // add by josh for skip eng poweroff
bool hall_trigger_suspend = false;

/* Add by Tom Cheng for cat sys/kernel/android_touch/tp_id (self_test) */
static int MXT_TP_ID = -1;
/* Add by Tom Cheng for check is cpt panel */
static int is_cpt_lcm = 0;

static bool hall_status = false;
static int mxt_irq_disable(unsigned int irq);
static int mxt_irq_enable(unsigned int irq);
static int mxt_wake_irq_disable(unsigned int irq);
static int mxt_wake_irq_enable(unsigned int irq);
//static int update_result_to_file(int status);

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int int_gpio; //add by josh
	unsigned int reset_gpio; //add by josh
	unsigned int power_gpio; //add by josh
	unsigned int max_x;
	unsigned int max_y;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 bootloader_addr;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	struct t7_config t7_cfg;
	struct t8_config t8_cfg;
	struct t104_config t104_cfg;
	struct t107_config t107_cfg;
	u8 num_stylusids;
	unsigned long t15_keystatus;
	bool use_retrigen_workaround;
	bool use_regulator;
	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	char *fw_name;
	char *cfg_name;
	u8 stylus_aux_pressure;
	u8 stylus_aux_batlvl;
	struct stylus_data stylus_status;
	
	u8 double_tap_enable; //add by josh, enable = 1
	u8 gesture_enable; //add by josh
	u8 gesture_type_1;
	u8 gesture_type_2;
	u8 gesture_type_3;
	u8 gesture_type_4;
	u8 gesture_type_5;
	u8 gesture_type_6;
	u8 z_stylus;
	struct mxt_config_info *config_info;
	u8 maj_version;
	u8 min_version;
	u8 display_id;
	struct workqueue_struct *mxt_wq;
	struct delayed_work touch_chip_firmware_upgrade_work;
	struct delayed_work power_save_work;
	struct delayed_work stylus_batlvl_work;
	struct switch_dev touch_sdev;
	int debug_log_enabled;
	struct regulator *vcc_i2c;
	struct regulator *vdd;
	u8 btn_stylus;
	u8 btn_stylus2;
	
	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T8_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u8 T48_reportid;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
	u16 T92_address;
	u8 T92_reportid;
	u16 T93_address;
	u8 T93_reportid;
	u16 T100_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;
	u16 T104_address;
	u16 T107_address;
#ifdef CONFIG_HAS_EARLYSUSPEND_DISABLE
	struct early_suspend early_suspend;
#endif

	/* for fw update in bootloader */
	struct completion bl_completion;

	/* for reset handling */
	struct completion reset_completion;

	/* for config update handling */
	struct completion crc_completion;

	/* Indicates whether device is in suspend */
	bool suspended;

	/* Indicates whether device is updating configuration */
	bool updating_config;
	
	/* Indicates whether device is updating configuration */
	u8 update_result;
		
	/* for firmware and config file recovery and upgrade */
	struct work_struct work_upgrade;
	bool force_upgrade;
	
	struct work_struct work_function;
	
	spinlock_t touch_spinlock;
};

	// Wakelock Protect start
	static struct wake_lock mxt_wake_lock;
	// Wakelock Protect end
	
#ifdef mxt_pen
//++++++ add by josh ++++++
static enum power_supply_property asus_pen_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_PRESENT,
};
static int asus_battery_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int ret = 0;
	switch(psp) {
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = current_batlvl;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = active_pen_mode;
		break;
	default:
        return -EINVAL;
	}
	return ret;
}

static struct power_supply asus_pen_power_supplies[] = {
	{
		.name = "pen_bat",
		.type = POWER_SUPPLY_TYPE_PEN_BATTERY,
		.properties = asus_pen_props,
		.num_properties = ARRAY_SIZE(asus_pen_props),
		.get_property = asus_battery_get_property,
	},
};
#endif
static struct mxt_data *touch_chip;

static int mxt_read_t38_data(struct mxt_data *data);
static void mxt_update_firmware_cfg(struct work_struct *);

/* defined but not used
static void mxt_power_save(struct work_struct *);
static void mxt_batlvl_report(struct work_struct *);
*/
static int mxt_touch_sysfs_init(void);	// Sys filesystem initial
static struct kobject *android_touch_kobj = NULL;	// Sys kobject variable
static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep);
static int mxt_self_update_cfg(struct mxt_data *data, int config_check);
static int mxt_get_t8_cfg(struct mxt_data *data);
/* defined but not used
static int mxt_get_t104_cfg(struct mxt_data *data);
*/
static int mxt_get_t107_cfg(struct mxt_data *data, u16 cmd_offset);
static void disable_stylus(struct mxt_data *data);
static void enable_stylus(struct mxt_data *data);

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static int atmel_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);

static struct notifier_block atmel_fb_notifier = {
        .notifier_call = atmel_fb_notifier_callback,
};

#endif
static size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	if(data->debug_enabled){
		print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
				message, data->T5_msg_size, false);
	}
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data)
		return;

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	LDBG("Enabled message output\n");
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{

	if (!data->debug_v2_enabled)
		return;

	LDBG("disabling message output\n");
	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	LDBG("Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		LDBG("No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data +
		       data->debug_msg_count * data->T5_msg_size,
		       msg,
		       data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		LDBG("Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data->debug_msg_data) {
		LDBG("No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{
	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = 0666;
	data->debug_msg_attr.read = mxt_debug_msg_read;
	data->debug_msg_attr.write = mxt_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&data->client->dev.kobj,
				  &data->debug_msg_attr) < 0) {
		dev_err(&data->client->dev, "Failed to create %s\n",
			data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->client->dev.kobj,
				      &data->debug_msg_attr);
}

static int mxt_wait_for_completion(struct mxt_data *data,
				   struct completion *comp,
				   unsigned int timeout_ms)
{
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;
	
	mxt_irq_enable(touch_chip->irq);
	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		return ret;
	} else if (ret == 0) {
		LDBG("Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data,
			       u8 *val, unsigned int count)
{
	int ret = 0;
	int retry_count = 0;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

retry_read:
	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		if(retry_count < 3){
			retry_count++;
			LDBG("i2c transfer failed (%d) ,retry:%d\n", ret, retry_count);
			mdelay(50 * retry_count);
			goto retry_read;
		} else {
			LDBG("i2c transfer failed (%d)\n",ret);
			return -EIO;
		}
	}

	return ret;
}

static int mxt_bootloader_write(struct mxt_data *data,
				const u8 * const val, unsigned int count)
{
	int ret = 0;
	int retry_count = 0;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

retry_write:
	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		if(retry_count < 3){
			retry_count++;
			LDBG("i2c transfer failed (%d) ,retry:%d\n", ret, retry_count);
			mdelay(50 * retry_count);
			goto retry_write;
		} else {
			LDBG("i2c transfer failed (%d)\n",ret);
			return -EIO;
		}
	}

	return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, bool retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = data->info ? data->info->family_id : 0;

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if (retry || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;

	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, bool alt_address)
{
	int error;
	u8 val;
	bool crc_failure;

	error = mxt_lookup_bootloader_address(data, alt_address);
	if (error)
		return error;

	error = mxt_bootloader_read(data, &val, 1);
	if (error)
		return error;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	LDBG("Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	u8 buf[3];

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			LDBG("i2c failure\n");
			return val;
		}

		LDBG("Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		LDBG("Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state,
				bool wait)
{
	u8 val;
	int ret;

recheck:
	if (wait) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
		ret = mxt_wait_for_completion(data, &data->bl_completion,
					      MXT_FW_CHG_TIMEOUT);
		if (ret) {
			/*
			 * TODO: handle -ERESTARTSYS better by terminating
			 * fw update process before returning to userspace
			 * by writing length 0x000 to device (iff we are in
			 * WAITING_FRAME_DATA state).
			 */
			LDBG("Update wait error %d\n", ret);
			return ret;
		}
	}

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			LDBG("Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		LDBG("Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret, retry_count = 0;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		/*if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
				__func__, ret);
			return -EIO;
		}*/
		if(retry_count < 3){
			retry_count++;
			LDBG("i2c transfer failed (%d) ,retry:%d\n", ret, retry_count);
			mdelay(50 * retry_count);
			goto retry_read;
		} else {
			LDBG("i2c transfer failed (%d)\n", ret);
			return -EIO;
		}
	}

	return 0;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
	int ret = 0;
	int retry_count = 0;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

retry_write:
	ret = i2c_master_send(client, buf, count);
	if (ret != count) {
		/*if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_write;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			ret = -EIO;
		}*/
		if(retry_count < 5){
			retry_count++;
			LDBG("i2c transfer failed (%d) ,retry:%d\n", ret, retry_count);
			mdelay(50 * retry_count);
			goto retry_write;
		} else {
			LDBG("i2c transfer failed (%d)\n", ret);
			return -EIO;
		}
	} else {
		ret = 0;
	}

	kfree(buf);
	return ret;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	LDBG("Invalid object type T %u\n", type);
	return NULL;
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		LDBG("T6 Config Checksum: 0x%06X\n", crc);
	}

	complete(&data->crc_completion);

	/* Detect reset */
	if (status & MXT_T6_STATUS_RESET)
		complete(&data->reset_completion);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		LDBG("T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			status == 0 ? " OK" : "",
			status & MXT_T6_STATUS_RESET ? " RESET" : "",
			status & MXT_T6_STATUS_OFL ? " OFL" : "",
			status & MXT_T6_STATUS_SIGERR ? " SIGERR" : "",
			status & MXT_T6_STATUS_CAL ? " CAL" : "",
			status & MXT_T6_STATUS_CFGERR ? " CFGERR" : "",
			status & MXT_T6_STATUS_COMSERR ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;
}

static void mxt_proc_t92_messages(struct mxt_data *data, u8 *msg)
{
	u8 status = msg[1];
	LDBG(":T92 gesture %d\n", status);
	if (status & 0x80){
		LDBG("T92 symbol reports 0x%x\n", status & 0x7f);
	} else {
		//LDBG("T92 long storke reports 0x%x\n", status & 0x0f);
		if(status  == 0x77 && data->gesture_type_1 == 1){
			LDBG(" T92 gesture w!!!\n");
			input_report_key(data->input_dev, KEY_F13, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_F13, 0);
			input_sync(data->input_dev);
		}
		if(status  == 0x73 && data->gesture_type_2 == 1){
			LDBG(" T92 gesture s!!!\n");
			input_report_key(data->input_dev, KEY_F14, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_F14, 0);
			input_sync(data->input_dev);
		}
		if(status == 0x65 && data->gesture_type_3 == 1){
			LDBG(" T92 gesture e!!!\n");
			input_report_key(data->input_dev, KEY_F15, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_F15, 0);
			input_sync(data->input_dev);
		}
		if(status  == 0x43 && data->gesture_type_4 == 1){
			LDBG(" T92 gesture c!!!\n");
			input_report_key(data->input_dev, KEY_F16, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_F16, 0);
			input_sync(data->input_dev);
		}
		if(status  == 0x7A && data->gesture_type_5 == 1){
			LDBG(" T92 gesture z!!!\n");
			input_report_key(data->input_dev, KEY_F17, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_F17, 0);
			input_sync(data->input_dev);
		}
		if(status  == 0x76 && data->gesture_type_6 == 1){
			LDBG(" T92 gesture v!!!\n");
			input_report_key(data->input_dev, KEY_F18, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_F18, 0);
			input_sync(data->input_dev);
		}
	}
	
}

static void mxt_proc_t93_messages(struct mxt_data *data, u8 *msg)
{
	u8 status = msg[0];

	LDBG("T93 double tap %d\n",status);
	input_report_key(data->input_dev, KEY_F19, 1);
	input_sync(data->input_dev);
	input_report_key(data->input_dev, KEY_F19, 0);
	input_sync(data->input_dev);
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	bool button;
	int i;

	/* Active-low switch */
	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;
		button = !(message[1] & (1 << i));
		input_report_key(input, pdata->t19_keymap[i], button);
	}
}

static void mxt_input_sync(struct mxt_data *data)
{
	if (data->input_dev) {
		input_mt_report_pointer_emulation(data->input_dev,
				data->pdata->t19_num_keys);
		input_sync(data->input_dev);
	}
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];

	dev_dbg(dev,
		"[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
		id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);

	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/*
		 * Multiple bits may be set if the host is slow to read
		 * the status messages, indicating all the events that
		 * have happened.
		 */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev,
						   MT_TOOL_FINGER, 0);
			mxt_input_sync(data);
		}

		/* A size of zero indicates touch is from a linked T47 Stylus */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_T47_STYLUS;
			tool = MT_TOOL_PEN;
		} else {
			tool = MT_TOOL_FINGER;
		}

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	data->update_input = true;
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	u8 type;
	int x;
	int y;
	int tool;
	bool active = false;
	bool hover = false;
	u8 major = 0;
	u8 finger_pressure = 0;
	u8 orientation = 0;
	u16 pressure = 0;
	u16 batlvl = 0;
	u16 pressure_1 = 0;
	u16 pressure_2 = 0;
	bool eraser = false, barrel = false;
		
	id = message[0] - data->T100_reportid_min - 2;

	/* ignore SCRSTATUS events */
	if (id < 0)
		return;

	status = message[1];
	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];

	dev_dbg(dev,
		"[%u] status:%02X x:%u y:%u area:%02X amp:%02X vec:%02X\n",
		id,
		status,
		x, y,
		data->t100_aux_area ? message[data->t100_aux_area] : 0,
		data->t100_aux_ampl ? message[data->t100_aux_ampl] : 0,
		data->t100_aux_vect ? message[data->t100_aux_vect] : 0);

	if(data->debug_log_enabled == 3){
		LDBG("id= %d, status= 0x%x\n",id,status);
	}

	if (status & MXT_T100_DETECT) {
		type = (status & MXT_T100_TYPE_MASK) >> 4;
		
		switch (type) {
			case MXT_T100_TYPE_HOVERING_FINGER:
				hover = true;
				/* fall through */
			case MXT_T100_TYPE_FINGER:
			case MXT_T100_TYPE_GLOVE:
				active = true;	
				type = MXT_T100_TYPE_FINGER;
				
				if (data->t100_aux_area)
					major = message[data->t100_aux_area];
				if (data->t100_aux_ampl)
					finger_pressure = message[data->t100_aux_ampl];
				if (data->t100_aux_vect)
					orientation = message[data->t100_aux_vect];
			
				break;
			case MXT_T100_TYPE_PASSIVE_STYLUS:
			case MXT_T100_TYPE_ACTIVE_STYLUS:
				active = true;	
				if (message[6] & MXT_T107_STYLUS_HOVER) {
					/* hover */
					if (message[6] & MXT_T107_STYLUS_TIPSWITCH) {
						if (data->stylus_aux_pressure){
							pressure_1 = ((message[6] & 0xC0) >> 6) & 0x0003;
							pressure_2 = message[7] & 0x00ff;
							pressure = pressure_1 | (pressure_2 << 2);
							//pressure = message[data->stylus_aux_pressure];
						}else{
							pressure = MXT_T107_STYLUS_PRESSURE;
						}
					} else {
						/* hover */
						pressure = 0;
						major = 0;
					}
				} else {
					active = 	false;
				}				
				break;
			case MXT_T100_TYPE_LARGE_TOUCH:
				break;
			default:
				LDBG("Unexpected T100 type\n");
				return;
		}	
	}
	if (hover) {
		finger_pressure = 0;
		major = 0;
	} else if (active) {
		/*
		 * Values reported should be non-zero if tool is touching the
		 * device
		 */
		if (finger_pressure == 0)
			finger_pressure = 1;

		if (major == 0)
			major = 1;
	}
	
	//For Stylus data services++++++
	/*if(touch_chip->stylus_status.active == 2){
		update_result_to_file(11);
		touch_chip->stylus_status.active = 1;
	}*/
	//For Stylus data services------
	
	if(active){	
		if (type == MXT_T100_TYPE_ACTIVE_STYLUS){
				
			//For Stylus data services++++++
			/*if(touch_chip->stylus_status.attach == 0){
				update_result_to_file(10);
				touch_chip->stylus_status.attach = 1;
			}*/
			//For Stylus data services------
			
			tool = MT_TOOL_PEN;
			//eraser = (message[6] & MXT_T107_STYLUS_BUTTON1)>>3;
			//barrel = (message[6] & MXT_T107_STYLUS_BUTTON0)>>2;
			//batlvl = message[8] & 0x000f;
						
			//batlvl+++
			/*if(batlvl_count < STYLUS_COUNT && batlvl == 0){
				batlvl_count++;
				current_batlvl = old_batlvl;
			}else{
				batlvl_count = 0;
				old_batlvl = batlvl;
				//need to update batlvl
				if(current_batlvl != batlvl){
					current_batlvl = batlvl;
					active_pen_mode == 0;
				}
			}*/
			
			//old_batlvl = batlvl;
			//if(old_batlvl == 0) old_batlvl = current_batlvl;
			
			//need to update batlvl
			//if(current_batlvl != old_batlvl){
			//	current_batlvl = old_batlvl;
				//active_pen_mode == 0;
			//}
			
			/*switch(current_batlvl){
			case 0:
				//current_batlvl = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
				current_batlvl = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
				break;
			case 1:
				current_batlvl = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
				break;
			case 2:	
				current_batlvl = POWER_SUPPLY_CAPACITY_LEVEL_MIDDLE;
				break;
			case 3:
				current_batlvl = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
				break;
			default:
				current_batlvl = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
				break;
			} 
			//batlvl--
			
			if(active_pen_mode == 0){
				active_pen_mode = 1;
				power_supply_changed(&asus_pen_power_supplies[0]);
			}*/
			

		} else {
			tool = MT_TOOL_FINGER;
		}

		/* Touch active */
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);


		
		if (data->t100_aux_area) {
			if (type == MXT_T100_TYPE_FINGER)
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, major);
			else
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 MXT_TOUCH_MAJOR_T47_STYLUS);
		}

		if (data->t100_aux_vect)
			input_report_abs(input_dev, ABS_MT_ORIENTATION, orientation);
					 
		if(type == MXT_T100_TYPE_ACTIVE_STYLUS) {
			input_report_abs(input_dev, ABS_MT_PRESSURE,pressure);	
		} else {
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					 finger_pressure * 4); //1024/255 = 4.0..
	
		}
		
		if(type == MXT_T100_TYPE_ACTIVE_STYLUS && 
			((data->btn_stylus != eraser) || (data->btn_stylus2 != barrel)
			|| eraser == 1 || barrel == 1)){
			data->btn_stylus = eraser;
			data->btn_stylus2 = barrel;
			input_report_key(input_dev, BTN_STYLUS, eraser);
			input_report_key(input_dev, BTN_STYLUS2, barrel);
			if(data->debug_log_enabled == 2){
				LDBG("eraser= %d, barrel= %d\n",eraser,barrel);
			}
		}
			
		if(data->debug_log_enabled == 3){
			LDBG(" mesg[6]=0x%2x, tool=%d, batlvl=%d, type=%d\n",message[6],tool,batlvl,type);
		}
		if(data->debug_log_enabled == 2 || data->debug_log_enabled == 3){
			LDBG("id= %d, x= %d, y= %d, major= %d ",id,x,y,major);
			
			if(type == MXT_T100_TYPE_ACTIVE_STYLUS) {
				LDBG("Pressure= %d\n",pressure);
			} else { 
				LDBG("Pressure= %d\n",finger_pressure*4);
			}
		}
			
	} else {
		/* Touch no longer active, close out slot */
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		
		if(data->debug_log_enabled == 2 || data->debug_log_enabled == 3){
			LDBG(" Touch up!!!!\n");
		}
		
		if(data->btn_stylus == 1 || data->btn_stylus2 == 1){
			data->btn_stylus = 0; 
			data->btn_stylus2 = 0;
			input_report_key(input_dev, BTN_STYLUS, 0);
			input_report_key(input_dev, BTN_STYLUS2, 0);
		}
		
	//	batlvl_count = 0;
	//	current_batlvl = 0;

	//	if(active_pen_mode == 1){
	//		queue_delayed_work(maxtouch_wq, &data->stylus_batlvl_work, 5*HZ);
	//	}
	}

	data->update_input = true;
}


static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);

	for (key = 0; key < data->pdata->t15_num_keys; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);

		if (!curr_state && new_state) {
			LDBG("T15 key press: %u\n", key);
			__set_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 1);
			sync = true;
		} else if (curr_state && !new_state) {
			LDBG("T15 key release: %u\n", key);
			__clear_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 0);
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP) {
		LDBG("T42 suppress\n");
	} else {
		LDBG("T42 normal\n");
	}
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	LDBG("T48 state %d status %02X %s%s%s%s%s\n", state, status,
		status & 0x01 ? "FREQCHG " : "",
		status & 0x02 ? "APXCHG " : "",
		status & 0x04 ? "ALGOERR " : "",
		status & 0x10 ? "STATCHG " : "",
		status & 0x20 ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		LDBG("invalid stylus id %d, max slot is %d\n",
			id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_T63_STYLUS_PRESSURE_MASK;

	dev_dbg(dev,
		"[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
		id,
		msg[1] & MXT_T63_STYLUS_SUPPRESS ? 'S' : '.',
		msg[1] & MXT_T63_STYLUS_MOVE     ? 'M' : '.',
		msg[1] & MXT_T63_STYLUS_RELEASE  ? 'R' : '.',
		msg[1] & MXT_T63_STYLUS_PRESS    ? 'P' : '.',
		x, y, pressure,
		msg[2] & MXT_T63_STYLUS_BARREL   ? 'B' : '.',
		msg[2] & MXT_T63_STYLUS_ERASER   ? 'E' : '.',
		msg[2] & MXT_T63_STYLUS_TIP      ? 'T' : '.',
		msg[2] & MXT_T63_STYLUS_DETECT   ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_T63_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
	}

	input_report_key(input_dev, BTN_STYLUS,
			 (msg[2] & MXT_T63_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2,
			 (msg[2] & MXT_T63_STYLUS_BARREL));

	mxt_input_sync(data);
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (data->input_dev && report_id == data->T92_reportid) {
		mxt_proc_t92_messages(data, message);
	} else if (data->input_dev && report_id == data->T93_reportid) {
		mxt_proc_t93_messages(data, message);
	} else if(data->suspended){
		/*skip touch when system suspend*/
		mxt_dump_message(data, message);
	} else if (!data->input_dev || data->suspended) {
		/*
		 * Do not report events if input device is not
		 * yet registered or returning from suspend
		 */
		mxt_dump_message(data, message);
	} else if (report_id >= data->T9_reportid_min
	    && report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
	} else if (report_id >= data->T100_reportid_min
	    && report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else if (report_id >= data->T63_reportid_min
		   && report_id <= data->T63_reportid_max) {
		mxt_proc_t63_messages(data, message);
	} else if (report_id >= data->T15_reportid_min
		   && report_id <= data->T15_reportid_max) {
		mxt_proc_t15_messages(data, message);
	} else {
		dump = true;
	}

	if (dump)
		mxt_dump_message(data, message);

	if (data->debug_v2_enabled)
		mxt_debug_msg_add(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		LDBG("Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	int ret;
	u8 count, num_left, retry = 0;

	/* Read T44 and T5 together */
	while(retry < 3){
		ret = __mxt_read_reg(data->client, data->T44_address,
			data->T5_msg_size + 1, data->msg_buf);
		if (ret) {
			if(retry < 3){
				LDBG("Failed to read T44 and T5 (%d), retry:%d\n", ret,retry);
				retry++;
			} else {
				return IRQ_NONE;
			}
		} else {
			retry = 3;
		}
	}
	
	count = data->msg_buf[0];

	if (count == 0) {
		/*
		 * This condition is caused by the CHG line being configured
		 * in Mode 0. It results in unnecessary I2C operations but it
		 * is benign.
		 */
		// LDBG("Interrupt triggered but zero messages\n");
		return IRQ_NONE;
	} else if (count > data->max_reportid) {
		LDBG("T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		LDBG("Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			LDBG("Unexpected invalid message\n");
	}

end:
	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	LDBG("CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* keep reading two msgs until one is invalid or reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static void mxt_work_function(struct work_struct *work)
{
	

	struct mxt_data *data =
			container_of(work,
				struct mxt_data, work_function);
		
		wake_lock(&mxt_wake_lock);
		if(data->suspended == true){
			msleep(60);
		}
		if (data->in_bootloader) {
			complete(&data->bl_completion);
			mxt_irq_enable(data->irq);
			return;
		}
	
		if (!data->object_table){
			mxt_irq_enable(data->irq);
			return;
		}
		
		if (data->T44_address) {
			mxt_process_messages_t44(data);
		} else {
			mxt_process_messages(data);
		}
		
		mxt_irq_enable(data->irq);
		wake_unlock(&mxt_wake_lock);


}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	
	if(data->debug_log_enabled == 4)
		LDBG(" ++ \n");
	
	if(touch_ic_status == true){
		mxt_irq_disable(data->irq);
		queue_work(maxtouch_wq, &data->work_function);
	}
	return IRQ_HANDLED;
}
/*
static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	irqreturn_t result_handle = IRQ_NONE;
	if(data->debug_log_enabled == 4)
	LDBG(" ++ \n");
	
	wake_lock(&mxt_wake_lock);
	if(touch_ic_status == true){
		if (data->in_bootloader) {
			complete(&data->bl_completion);
			result_handle = IRQ_HANDLED;
			goto end;
		}
	
		if (!data->object_table){
			result_handle = IRQ_HANDLED;
			goto end;
		}
	
		if (data->T44_address) {
			result_handle = mxt_process_messages_t44(data);
			goto end;
		} else {
			result_handle = mxt_process_messages(data);
			goto end;
		}
	}
	
end:	
	wake_unlock(&mxt_wake_lock);
	return result_handle;
}*/

static int mxt_t92_configuration(struct mxt_data *data, u16 cmd_offset,
				u8 status)
{
	u16 reg;
	u8 command_register;
	int ret;

	reg = data->T92_address + cmd_offset;

	ret = __mxt_read_reg(data->client, reg, 1, &command_register);
	if (ret)
		return ret;

	if (status == MXT_T92_ENABLE)
		command_register |= (MXT_T92_CTRL_ENABLE|MXT_T92_CTRL_RPTEN);
	if (status == MXT_T92_DISABLE)
		command_register &= ~(MXT_T92_CTRL_ENABLE|MXT_T92_CTRL_RPTEN);

	ret = mxt_write_reg(data->client, reg, command_register);
	if (ret)
		return ret;

	return 0;
}

static int mxt_t93_configuration(struct mxt_data *data, u16 cmd_offset,
				u8 status)
{
	u16 reg;
	u8 command_register;
	int ret;

	reg = data->T93_address + cmd_offset;

	ret = __mxt_read_reg(data->client, reg, 1, &command_register);
	if (ret)
		return ret;

	if (status == MXT_T93_ENABLE)
		command_register |= (MXT_T93_CTRL_ENABLE|MXT_T93_CTRL_RPTEN);
	if (status == MXT_T93_DISABLE)
		command_register &= ~(MXT_T93_CTRL_ENABLE|MXT_T93_CTRL_RPTEN);

	ret = mxt_write_reg(data->client, reg, command_register);
	if (ret)
		return ret;

	return 0;
}

/* defined but not used
static int mxt_t107_configuration(struct mxt_data *data, u16 cmd_offset,
				u8 status)
{
	u16 reg;
	u8 command_register;
	int ret;

	reg = data->T107_address + cmd_offset;

	ret = __mxt_read_reg(data->client, reg, 1, &command_register);
	if (ret)
		return ret;

	if (status == MXT_T107_ENABLE)
		command_register |= (MXT_T107_CTRL_ENABLE);
	if (status == MXT_T107_DISABLE)
		command_register &= ~(MXT_T107_CTRL_ENABLE);

	ret = mxt_write_reg(data->client, reg, command_register);
	if (ret)
		return ret;

	return 0;
}
*/

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while (command_register != 0 && timeout_counter++ <= 100);

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	int ret = 0;

	LDBG("Resetting chip\n");

	INIT_COMPLETION(data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	ret = mxt_wait_for_completion(data, &data->reset_completion,
				      MXT_RESET_TIMEOUT);
	if (ret)
		return ret;

	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/*
	 * On failure, CRC is set to 0 and config will always be
	 * downloaded.
	 */
	data->config_crc = 0;
	INIT_COMPLETION(data->crc_completion);

	mxt_t6_command(data, cmd, value, true);

	/*
	 * Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded.
	 */
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	int val;

	/*irqd = irq_get_irq_data(data->irq);
	if (irqd_get_trigger_type(irqd) & IRQF_TRIGGER_LOW)
		return 0;*/

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				       data->T18_address + MXT_COMMS_CTRL,
				       1, &val);
		if (error)
			return error;
	}
	
	if (val & MXT_COMMS_RETRIGEN) {
		dev_err(&client->dev, "Enabling RETRIGEN workaround\n");
		data->use_retrigen_workaround = true;
		}
	return 0;
}

static int mxt_cfg_read_version(struct mxt_data *data,
			       const struct firmware *cfg,
			       unsigned int data_pos, int config_match)
{
	unsigned int type, instance, size;
	int offset, i;
	int ret = 0;
	u8 val = 0;
	u8 config_MSB = 0;
	u8 config_LSB = 0;
	
	while (data_pos < cfg->size) {
		/* Read type, instance, length */
		ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
			     &type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			LDBG("Bad format: failed to parse object\n");
			return -EINVAL;
		}
		data_pos += offset;
		
		if(type == 0x0026){
			for (i = 0; i < 2; i++) {
				ret = sscanf(cfg->data + data_pos, "%hhx%n",
						&val, &offset);
					/*if((i == 0 && val > data->maj_version) || (i == 1 && val > data->min_version)){
							return 1;
					}*/
					if(i == 0)
						config_MSB = val;
					if(i == 1)	
						config_LSB = val;
				if (ret != 1) {
					LDBG("Bad format in T%d at %d\n",
						type, i);
					return -EINVAL;
				}
				data_pos += offset;
			}
			break;
		}else{
			data_pos += 3*size;
		}
	}
	if((config_MSB > data->maj_version) || ((config_MSB == data->maj_version) && (config_LSB > data->min_version))){
		LDBG("File config version(%d-%d) is large than chip config version(%d-%d)\n",
				config_MSB, config_LSB, data->maj_version,data->min_version);
		return 1;
	}
	
	if((config_MSB == data->maj_version) && (config_LSB == data->min_version) && config_match == 0){
		LDBG("File config version(%d-%d) and chip config version(%d-%d) is the same, but checksum is difficult\n",
				config_MSB, config_LSB, data->maj_version,data->min_version);

		return 1;
	}
	
		
	
	return 0;
}

static int mxt_prepare_cfg_mem(struct mxt_data *data,
			       const struct firmware *cfg,
			       unsigned int data_pos,
			       unsigned int cfg_start_ofs,
			       u8 *config_mem,
			       size_t config_mem_size)
{
	struct mxt_object *object;
	unsigned int type, instance, size, byte_offset;
	int offset;
	int ret;
	int i;
	u16 reg;
	u8 val;

	while (data_pos < cfg->size) {
		/* Read type, instance, length */
		ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
			     &type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			// LDBG("Bad format: failed to parse object Retry Again, data_pos:%d, cfg->size:%d\n",data_pos,cfg->size);
			ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
			     &type, &instance, &size, &offset);
			if(data_pos == cfg->size - 2) {
				break;
			}
					
			if (ret == 0) {
				/* EOF */
				break;
			} else if (ret != 3) {
				// LDBG("Bad format: failed to parse object, data_pos:%d, cfg->size:%d\n",data_pos,cfg->size);
				return -EINVAL;
			}
		}
		data_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->data + data_pos, "%hhx%n",
					     &val, &offset);
				if (ret != 1) {
					LDBG("Bad format in T%d at %d\n",
						type, i);
					return -EINVAL;
				}
				data_pos += offset;
			}
			continue;
		}

		if (size > mxt_obj_size(object)) {
			/*
			 * Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited.
			 */
			LDBG("Discarding %zu byte(s) in T%u\n",
				 size - mxt_obj_size(object), type);
		} else if (mxt_obj_size(object) > size) {
			/*
			 * If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration.
			 */
			LDBG("Zeroing %zu byte(s) in T%d\n",
				 mxt_obj_size(object) - size, type);
		}

		if (instance >= mxt_obj_instances(object)) {
			LDBG("Object instances exceeded!\n");
			return -EINVAL;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->data + data_pos, "%hhx%n",
				     &val,
				     &offset);
			if (ret != 1) {
				LDBG("Bad format in T%d at %d\n",
					type, i);
				return -EINVAL;
			}
			data_pos += offset;

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg_start_ofs;

			if (byte_offset >= 0 && byte_offset < config_mem_size) {
				*(config_mem + byte_offset) = val;
			} else {
				LDBG("Bad object: reg:%d, T%d, ofs=%d\n",
					reg, object->type, byte_offset);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int mxt_upload_cfg_mem(struct mxt_data *data, unsigned int cfg_start,
			      u8 *config_mem, size_t config_mem_size)
{
	unsigned int byte_offset = 0,retry = 0;
	int error;

	/* Write configuration as blocks */
	while (byte_offset < config_mem_size) {
		unsigned int size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;
		
		while(retry < 3){
			//mdelay(1);
			error = __mxt_write_reg(data->client,
						cfg_start + byte_offset,
						size, config_mem + byte_offset);
			if (error) {
				retry ++;
				dev_err(&data->client->dev,
					"Config write error, ret=%d,retry = %d\n", error,retry);
				//return error;
			} else {
				retry = 0;
				break;
			}
		}
		if(retry == 3)
			return error;

		byte_offset += size;
	}

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

/*
 * mxt_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_update_cfg(struct mxt_data *data, const struct firmware *cfg, int config_check)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info;
	int ret;
	int offset;
	int data_pos;
	int i,retry = 0,config_match = 1;
	int cfg_start_ofs;
	u32 info_crc, config_crc, calculated_crc;
	u8 *config_mem;
	size_t config_mem_size;

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		LDBG("Unrecognised config file\n");
		return -EINVAL;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg->data + data_pos, "%hhx%n",
			     (unsigned char *)&cfg_info + i,
			     &offset);
		if (ret != 1) {
			LDBG("Bad format\n");
			return -EINVAL;
		}

		data_pos += offset;
	}
	
	if (cfg_info.family_id != data->info->family_id) {
		LDBG("Family ID mismatch!\n");
		return -EINVAL;
	}

	if (cfg_info.variant_id != data->info->variant_id) {
		LDBG("Variant ID mismatch!\n");
		return -EINVAL;
	}

	/* Read CRCs */
	ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		LDBG("Bad format: failed to parse Info CRC\n");
		return -EINVAL;
	}
	data_pos += offset;

	ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		LDBG("Bad format: failed to parse Config CRC\n");
		return -EINVAL;
	}
	data_pos += offset;

	/*
	 * The Info Block CRC is calculated over mxt_info and the object
	 * table. If it does not match then we are trying to load the
	 * configuration from a different chip or firmware version, so
	 * the configuration CRC is invalid anyway.
	 */
	if (info_crc == data->info_crc) {
		if (config_crc == 0 || data->config_crc == 0) {
			LDBG("CRC zero, attempting to apply config\n");
		} else if (config_crc == data->config_crc) {
			LDBG("Config CRC 0x%06X: OK\n",
				 data->config_crc);
			/*return 0;*/ 
		} else {
			LDBG("Config CRC 0x%06X: does not match file 0x%06X\n",
				 data->config_crc, config_crc);
				 config_match = 0;
		}
	} else {
		dev_warn(dev,
			 "Warning: Info CRC error - device=0x%06X file=0x%06X\n",
			 data->info_crc, info_crc);
	}

	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START +
			data->info->object_num * sizeof(struct mxt_object) +
			MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem) {
		LDBG("Failed to allocate memory\n");
		return -ENOMEM;
	}
	
	if(config_check == 1){
		ret = mxt_cfg_read_version(data, cfg, data_pos, config_match);
		if (!ret){
			LDBG(" No Need to update Config\n");
			goto release_mem;
		}else{
			LDBG(" Need to update Config\n");
		}
	}
	ret = mxt_prepare_cfg_mem(data, cfg, data_pos, cfg_start_ofs,
				  config_mem, config_mem_size);
	if (ret)
		goto release_mem;

	/* Calculate crc of the received configs (not the raw config file) */
	if (data->T7_address < cfg_start_ofs) {
		LDBG("Bad T7 address, T7addr = %x, config offset %x\n",
			data->T7_address, cfg_start_ofs);
		ret = 0;
		goto release_mem;
	}

	calculated_crc = mxt_calculate_crc(config_mem,
					   data->T7_address - cfg_start_ofs,
					   config_mem_size);

	if (config_crc > 0 && config_crc != calculated_crc)
		LDBG("Config CRC error, calculated=%06X, file=%06X\n",
			 calculated_crc, config_crc);

	do{
		ret = mxt_upload_cfg_mem(data, cfg_start_ofs,
				 config_mem, config_mem_size);
		if (ret)
			goto release_mem;
	
		mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);
	
		ret = mxt_check_retrigen(data);
		if (ret)
			goto release_mem;
	
		ret = mxt_soft_reset(data);
		if (ret)
			goto release_mem;
		
		retry++;
		
		if(retry > 2){
			ret = -1;
			break;
		}
		LDBG(" Config CRC 0x%06X, retry:%d\n",data->config_crc, retry);	
	} while(data->config_crc != config_crc || retry == 0 );
	
	if(ret == 0){
		/* T7 config may have changed */
		mxt_init_t7_power_cfg(data);
		LDBG("Config successfully updated\n");
		mxt_read_t38_data(data);
		LDBG(" Config Version : %d.%d - 0x%06X\n",data->maj_version,data->min_version,data->config_crc);
	} else{
		LDBG("Config updated fail ret:%d\n",ret);
	}

release_mem:
	kfree(config_mem);	
	return ret;
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	mxt_irq_enable(data->irq);

	if (data->use_retrigen_workaround) {
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	mxt_debug_msg_remove(data);

	data->object_table = NULL;
	data->info = NULL;
	kfree(data->raw_info_block);
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;

	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T8_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T48_reportid = 0;
	data->T63_reportid_min = 0;
	data->T63_reportid_max = 0;
	data->T92_address = 0;
	data->T92_reportid = 0;
	data->T93_address = 0;
	data->T93_reportid = 0;
	data->T100_address = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data,
				  struct mxt_object *object_table)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_dbg(&data->client->dev,
			"T%u Start:%u Size:%zu Instances:%zu Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80 &&
			    data->info->version < 0x20) {
				/*
				 * On mXT224 firmware versions prior to V2.0
				 * read and discard unused CRC byte otherwise
				 * DMA reads are misaligned.
				 */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_GEN_ACQUIRE_T8:
			data->T8_address = object->start_address;
			break;	
		case MXT_TOUCH_MULTI_T9:
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_PROCI_ACTIVE_STYLUS_T63:
			/* Only handle messages from first T63 instance */
			data->T63_reportid_min = min_id;
			data->T63_reportid_max = min_id;
			data->num_stylusids = 1;
			break;
		case MXT_PROC_SYMBOLGESTUREPROCESSOR_T92:
			data->T92_address = object->start_address;
			data->T92_reportid = min_id;
			break;
		case MXT_PROCI_TOUCHSEQUENCELOGGER_T93:
			data->T93_address = object->start_address;
			data->T93_reportid = min_id;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->T100_address = object->start_address;
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			break;
		case MXT_SPT_AUXTOUCHCONFIG_T104:
			data->T104_address = object->start_address;
			break;
		case MXT_PROCI_ACTIVESTYLUS_T107:
			data->T107_address = object->start_address;
			break;	
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		dev_err(&client->dev, "Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, 0, size, id_buf);
	if (error) {
		kfree(id_buf);
		return error;
	}

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mxt_info *)id_buf)->object_num;
	size += (num_objects * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(id_buf, size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = __mxt_read_reg(client, MXT_OBJECT_START,
			       size - MXT_OBJECT_START,
			       buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/*
	 * CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol (eg i2c-hid)
	 */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	}

	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;

	dev_info(&client->dev,
		 "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);
	
	/* Parse object table information */
	error = mxt_parse_object_table(data, buf + MXT_OBJECT_START);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n", error);
		mxt_free_object_table(data);
		return error;
	}
	
	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);
	touch_ic_status = true;
	
	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	int error;

	gpio_set_value(data->pdata->gpio_reset, 0);

	error = regulator_enable(data->reg_vdd);
	if (error)
		return;

	error = regulator_enable(data->reg_avdd);
	if (error)
		return;

	msleep(MXT_REGULATOR_DELAY);
	gpio_set_value(data->pdata->gpio_reset, 1);
	msleep(MXT_CHG_DELAY);

retry_wait:
	INIT_COMPLETION(data->bl_completion);
	data->in_bootloader = true;
	error = mxt_wait_for_completion(data, &data->bl_completion,
					MXT_POWERON_DELAY);
	if (error == -EINTR)
		goto retry_wait;

	data->in_bootloader = false;
}

static void mxt_regulator_disable(struct mxt_data *data)
{
	regulator_disable(data->reg_vdd);
	regulator_disable(data->reg_avdd);
}

static void mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	/*
	 * According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage
	 */
	if (!gpio_is_valid(data->pdata->gpio_reset)) {
		LDBG("Must have reset GPIO to use regulator support\n");
	//	goto fail;
	}

	data->reg_vdd = regulator_get(dev, "vdd");
	if (IS_ERR(data->reg_vdd)) {
		error = PTR_ERR(data->reg_vdd);
		LDBG("Error %d getting vdd regulator\n", error);
	//	goto fail;
	}else{
	    error = regulator_set_voltage(data->reg_vdd, MXT_VDD_VTG_MIN_UV, MXT_VDD_VTG_MAX_UV);
	    if (error)
	    {
	        LDBG("Regulator set_vtg failed vdd error=%d\n", error);
	  //      goto fail;
	    }

	    error = regulator_enable(data->reg_vdd);
	    if (error)
	    {
	        LDBG("Regulator vdd enable failed error=%d\n", error);
	    //    goto fail;
	    }
	}

	pr_info("[Atmel] Initialised regulators\n");
	data->reg_avdd = regulator_get(dev, "iovcc");
	if (IS_ERR(data->reg_avdd)) {
		error = PTR_ERR(data->reg_avdd);
		LDBG("Error %d getting avdd regulator\n", error);
		goto fail_release;
	}else{
		error = regulator_set_voltage(data->reg_avdd, MXT_AVDD_VTG_MIN_UV, MXT_AVDD_VTG_MAX_UV);
	    if (error)
	    {
	        dev_err(dev,
	                "Regulator set_vtg failed avdd error=%d\n",
	                error);
	        goto fail_release;
	    }
	
	    pr_info("[Atmel] enable iovcc \n");
	    error = regulator_enable(data->reg_avdd);
	    if (error)
	    {
	        LDBG("Regulator avdd enable failed error=%d\n", error);
	        goto fail_release;
	    }
	}
	/* Modify by Tom for Don't disable Power when suspend */
	data->use_regulator = false;
	mxt_regulator_enable(data);

	return;

	gpio_set_value(data->reset_gpio, 1);
	msleep(100);
	
fail_release:
	regulator_put(data->reg_vdd);
// fail:
	data->reg_vdd = NULL;
	data->reg_avdd = NULL;
	data->use_regulator = false;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(&range.x);
	le16_to_cpus(&range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 1023;

	if (range.y == 0)
		range.y = 1023;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	dev_dbg(&client->dev,
		"Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static void mxt_start(struct mxt_data *data);
static void mxt_stop(struct mxt_data *data);
static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_t9_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	const struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	int i;

	error = mxt_read_t9_resolution(data);
	if (error)
		LDBG("Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		LDBG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "Atmel maXTouch Touchscreen";
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	if (pdata->t19_num_keys) {
		__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

		for (i = 0; i < pdata->t19_num_keys; i++)
			if (pdata->t19_keymap[i] != KEY_RESERVED)
				input_set_capability(input_dev, EV_KEY,
						     pdata->t19_keymap[i]);

		__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
		__set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_QUADTAP, input_dev->keybit);

		input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_X,
				  MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
				  MXT_PIXELS_PER_MM);

		input_dev->name = "Atmel maXTouch Touchpad";
	}

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
	num_mt_slots = data->num_touchids + data->num_stylusids;
	error = input_mt_init_slots(input_dev, num_mt_slots, INPUT_MT_DIRECT); //modify by josh
	if (error) {
		LDBG("Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	/* For T63 active stylus */
	if (data->T63_reportid_min) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
			0, MT_TOOL_MAX, 0, 0);
	}

	/* For T15 key array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;

		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					     data->pdata->t15_keymap[i]);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		LDBG("Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_read_t107_stylus_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u8 styaux;
	int aux;

	object = mxt_get_object(data, MXT_PROCI_ACTIVESTYLUS_T107);
	if (!object)
		return 0;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T107_STYLUS_STYAUX,
			       1, &styaux);
	if (error)
		return error;

	/* map aux bits */
	aux = 7;
        
    
        //styaux shoud be 0x??11 to enable t107 pressure value, so we hack it for temp solution
	//if (styaux & MXT_T107_STYLUS_STYAUX_PRESSURE)
		data->stylus_aux_pressure = aux++;

	if (styaux & MXT_T107_STYLUS_STYAUX_BATLVL)
		data->stylus_aux_batlvl = aux++;

	LDBG("styaux = 0x%2x\n[ATMEL] Enabling T107 active stylus, aux map pressure:%u batlvl:%u\n",
		styaux, data->stylus_aux_pressure, data->stylus_aux_batlvl);

	return 0;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;
	u8 cfg, tchaux;
	u8 aux;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(&range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(&range_y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_y;
		data->max_y = range_x;
	} else {
		data->max_x = range_x;
		data->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	dev_info(&client->dev,
		 "T100 Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	struct input_dev *input_dev;
	int error;

	error = mxt_read_t100_config(data);
	if (error)
		LDBG("Failed to initialize T9 resolution\n");

	mxt_read_t107_stylus_config(data);
	
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		LDBG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	if (data->pdata->input_name)
		input_dev->name = data->pdata->input_name;
	else
		input_dev->name = "atmel_mxt_ts";

	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_PRESSURE,
				     0, 255, 0, 0);

	/* For multi touch */
	error = input_mt_init_slots(input_dev, data->num_touchids, INPUT_MT_DIRECT); //modify by josh
	if (error) {
		LDBG("Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_FINGER, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);

	if (data->T93_address)
		input_set_capability(input_dev, EV_KEY, KEY_F19);
	
	if(data->T92_address){
		input_set_capability(input_dev, EV_KEY, KEY_F13); //w
		input_set_capability(input_dev, EV_KEY, KEY_F14); //s
		input_set_capability(input_dev, EV_KEY, KEY_F15); //e
		input_set_capability(input_dev, EV_KEY, KEY_F16); //c
		input_set_capability(input_dev, EV_KEY, KEY_F17); //z
		input_set_capability(input_dev, EV_KEY, KEY_F18); //v
	}
	if (data->T107_address) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
	}

	if (data->t100_aux_area)
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				     0, MXT_MAX_AREA, 0, 0);

	if (data->t100_aux_ampl | data->stylus_aux_pressure)
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
				     0, 1024, 0, 0);

	if (data->t100_aux_vect)
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				     0, 255, 0, 0);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		LDBG("Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg, int config_check);

static void mxt_config_cb(const struct firmware *cfg, void *ctx)
{
	mxt_configure_objects(ctx, cfg, 0);
	release_firmware(cfg);
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int recovery_attempts = 0;
	int error;

	while (1) {
		error = mxt_read_info_block(data);
		if (!error)
			break;

		/* Check bootloader state */
		error = mxt_probe_bootloader(data, false);
		if (error) {
			dev_info(&client->dev, "Trying alternate bootloader address\n");
			error = mxt_probe_bootloader(data, true);
			if (error) {
				/* Chip is not in appmode or bootloader mode */
				return error;
			}
		}

		/* OK, we are in bootloader, see if we can recover */
		if (++recovery_attempts > 1) {
			dev_err(&client->dev, "Could not recover from bootloader mode\n");
			/*
			 * We can reflash from this state, so do not
			 * abort initialization.
			 */
			data->in_bootloader = true;
			return 0;
		}

		/* Attempt to exit bootloader into app mode */
		mxt_send_bootloader_cmd(data, false);
		msleep(MXT_FW_RESET_TIME);
	}

	error = mxt_check_retrigen(data);
	if (error)
		goto err_free_object_table;

	error = mxt_acquire_irq(data);
	if (error)
		goto err_free_object_table;

	error = mxt_debug_msg_init(data);
	if (error)
		goto err_free_object_table;

	if (data->cfg_name) {
		error = request_firmware_nowait(THIS_MODULE, true,
					data->cfg_name, &data->client->dev,
					GFP_KERNEL, data, mxt_config_cb);
		if (error) {
			dev_err(&client->dev, "Failed to invoke firmware loader: %d\n",
				error);
			goto err_free_object_table;
		}
	} else {
		error = mxt_configure_objects(data, NULL, 0);
		if (error)
			goto err_free_object_table;
	}

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

static int mxt_set_t100_multitouchscreen_cfg(struct mxt_data *data,
					u16 cmd_offset, u8 type)
{
	    u16 reg;
        u8 command_register;
        int ret;

        reg = data->T100_address + cmd_offset;

		     command_register = type;

        ret = mxt_write_reg(data->client, reg, command_register);
        if (ret)
                return ret;

        return 0;
}

static int mxt_set_t8_acquisition_cfg(struct mxt_data *data,
					u16 cmd_offset, u8 type)
{
	u16 reg;
        u8 command_register;
        int ret;

	if(build_version == 1)
		return 0;

        reg = data->T8_address + cmd_offset;

		     command_register = type;

        ret = mxt_write_reg(data->client, reg, command_register);
        if (ret)
                return ret;

        return 0;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	int error;
	struct t7_config *new_config;
	struct t7_config normal_config = { .active = active_rep, .idle = idle_rep };
	struct t7_config powersave = { .active = active_rep, .idle = 100 };
	struct t7_config deepsleep = { .active = 0, .idle = 0 };
	/* TODO
	*/
	struct t7_config gesture_mode = { .active = gesture_active_rep, .idle = gesture_idle_rep };
	power_save_mode = false;
	
	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else if (sleep == MXT_POWER_CFG_GESTURE)
		new_config = &gesture_mode;
	else if (sleep == MXT_POWER_CFG_POWERSAVE)
		new_config = &powersave;
	else
		new_config = &normal_config;

	error = __mxt_write_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), new_config);
	if (error)
		return error;

	LDBG("Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);
	//LDBG(" Set T7 ACTV:%d IDLE:%d\n",new_config->active, new_config->idle);
	
	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			LDBG("T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
			LDBG("T7 cfg zero after reset, overriding\n");
			data->t7_cfg.active = active_rep;
			data->t7_cfg.idle = idle_rep;
			return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	}
	active_rep = data->t7_cfg.active;
	idle_rep = data->t7_cfg.idle;
	
	LDBG("Initialized power cfg: ACTV %d, IDLE %d\n",
		data->t7_cfg.active, data->t7_cfg.idle);
	
	mxt_get_t8_cfg(data);
	//mxt_get_t104_cfg(data);
	mxt_get_t107_cfg(data, 97);
	
	return 0;
}

static int mxt_get_t8_cfg(struct mxt_data *data)
{
	int error;

	error = __mxt_read_reg(data->client, data->T8_address,
				sizeof(data->t8_cfg), &data->t8_cfg);
	if (error)
		return error;
	
	/*LDBG("[Atmel] \n [Atmel]byte0: %d\n [Atmel]byte1: %d\n [Atmel]byte2: %d\n [Atmel]byte3: %d\n [Atmel]byte4: %d\n [Atmel]byte5: %d\n [Atmel]byte6: %d\n [Atmel]byte7: %d\n [Atmel]byte8: %d\n [Atmel]byte9: %d\n",
		data->t8_cfg.byte0, data->t8_cfg.byte1, data->t8_cfg.byte2, data->t8_cfg.byte3,
		data->t8_cfg.byte4, data->t8_cfg.byte5, data->t8_cfg.byte6, data->t8_cfg.byte7,
		data->t8_cfg.byte8, data->t8_cfg.byte9);*/
	return 0;
}

/* defined but not used
static int mxt_get_t104_cfg(struct mxt_data *data)
{
	int error;
	bool retry = false;

	error = __mxt_read_reg(data->client, data->T104_address,
				sizeof(data->t104_cfg), &data->t104_cfg);
	if (error)
		return error;

	return 0;
}
*/
static int mxt_get_t107_cfg(struct mxt_data *data,u16 cmd_offset)
{
	int error;

	error = __mxt_read_reg(data->client, data->T107_address + cmd_offset,
				sizeof(data->t107_cfg), &data->t107_cfg);
	if (error)
		return error;

	return 0;
}

/* defined but not used
static int mxt_set_t8_cfg(struct mxt_data *data, u8 status)
{
	int error;
	struct t8_config *new_config;
	struct t8_config disable_config = data->t8_cfg;
	
	//init rewrite value
	if (status == MXT_STYLUS_DISABLE){
		disable_config.byte4 = 0;
		disable_config.byte6 = 255;
		disable_config.byte7 = 1;
		disable_config.byte8 = 0;
		disable_config.byte9 = 0;
		disable_config.byte10 = 3;
		disable_config.byte11 = 2;
		new_config = &disable_config;
	} else {
		new_config = &data->t8_cfg;
	}
	
	error = __mxt_write_reg(data->client, data->T8_address,
				sizeof(data->t8_cfg), new_config);
	if (error)
		return error;
	
	return 0;
}

static int mxt_set_t104_cfg(struct mxt_data *data, u8 status)
{
	int error;
	struct t104_config *new_config;
	struct t104_config disable_config = data->t104_cfg;
	
	//init rewrite value
	if (status == MXT_STYLUS_DISABLE){
		disable_config.byte2 = 255;
		disable_config.byte3 = 63;
		disable_config.byte4 = 255;
		disable_config.byte5 = 63;
		disable_config.byte7 = 255;
		disable_config.byte8 = 63;
		disable_config.byte9 = 255;
		disable_config.byte10 = 63;
		new_config = &disable_config;
	} else {
		new_config = &data->t104_cfg;
	}
	
	error = __mxt_write_reg(data->client, data->T104_address,
				sizeof(data->t104_cfg), new_config);
	if (error)
		return error;
	
	return 0;
}
*/
static int mxt_set_t107_cfg(struct mxt_data *data, u16 cmd_offset, u8 status)
{
	int error;
	struct t107_config *new_config;
	struct t107_config disable_config = data->t107_cfg;
	
	//init rewrite value
	if (status == MXT_STYLUS_DISABLE){
		disable_config.byte97 = 0;
		new_config = &disable_config;
	} else {
		new_config = &data->t107_cfg;
	}
	
	error = __mxt_write_reg(data->client, data->T107_address + cmd_offset,
				sizeof(data->t107_cfg), new_config);
	if (error)
		return error;
	
	return 0;
}

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg, int config_check)
{
	int error = 0, error2 = 0,retry = 0;

	while(retry < 3){
		error = mxt_init_t7_power_cfg(data);
		if (error) {
			LDBG("Failed to initialize power cfg\n");
			retry++;
			mdelay(500);
			LDBG("mxt_init_t7_power_cfg retry %d!!!\n",retry);
			//goto err_free_object_table;
		}else{
			break;
		}
	}

	if (cfg) {
		error2 = mxt_update_cfg(data, cfg, config_check);
		if (error2)
			LDBG("error2 %d updating config\n", error2);
	}

	if (data->T9_reportid_min) {
		error = mxt_initialize_t9_input_device(data);
		if (error)
			goto err_free_object_table;
	} else if (data->T100_reportid_min) {
		error = mxt_initialize_t100_input_device(data);
		if (error)
			goto err_free_object_table;
	} else {
		LDBG("No touch object detected\n");
	}

	return error2;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

// Read T38[0] and T38[1] from TP chip.
static int mxt_read_t38_data(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    struct mxt_object *object;
    u8 Ver1, Ver2;
	
    object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
    if (!object)
            return -EINVAL;

    error = __mxt_read_reg(client,
                           object->start_address + 0,
                           sizeof(Ver1), &Ver1);
    if (error)
            return error;
    error = __mxt_read_reg(client,
                           object->start_address + 1,
                           sizeof(Ver2), &Ver2);
    if (error)
            return error;
	
	data->maj_version = Ver1;
    data->min_version = Ver2;

	//LDBG("T38 User data2 : %d.%d\n",data->maj_version,data->min_version);
    return 0;
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t mxt_config_csum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->config_crc);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (!data->object_table)
		return -EINVAL;
		
	//mxt_update_firmware_cfg(data);
	//mxt_read_t38_data(data);
	LDBG(" %u.%u.%02X-%u.%u-%06x\n",
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build,data->maj_version,data->min_version,data->config_crc);
			 
	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X-%u.%u-%06x\n",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build,data->maj_version,data->min_version,data->config_crc);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (!data->object_table)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	if (!data->object_table)
		return -EINVAL;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/*
	 * To convert file try:
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw
	 */
	LDBG("Aborting: firmware file must be in binary format\n");

	return -EINVAL;
}

static int mxt_load_cfg(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *cfg;
	int error;
	LDBG(".\n");
	
	error = request_firmware(&cfg, MXT_CONFIG_NAME_1666T2, dev);
	if (error < 0) {
		LDBG("Failure to request config file %s\n",
			data->cfg_name);
		error = -ENOENT;
		goto out;
	}

	data->updating_config = true;

	mxt_free_input_device(data);

	error = mxt_configure_objects(data, cfg, 0);

out:
	data->updating_config = false;
	return error;
}

static int mxt_load_fw(struct device *dev,int reset,int path)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret,write_retry = 0,temp_frame = 0,index = 0;
	
	data->updating_config = true;
	
	LDBG(" ++ \n");
	if(path == 0){
		ret = request_firmware(&fw, data->fw_name, dev);
	} else {
		ret = request_firmware(&fw, MXT_FIRMWARE_NAME_1666T2, dev);
	}
	if (ret) {
		LDBG("Unable to open firmware %s\n", data->fw_name);
		data->updating_config = false;
		return ret;
	}

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);
	if (ret)
		goto release_firmware;

	if (data->suspended) {
		if (data->use_regulator)
			mxt_regulator_enable(data);

		mxt_irq_enable(data->irq);
		data->suspended = false;
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
				     MXT_BOOT_VALUE, false);
		if (ret)
			goto release_firmware;

		msleep(MXT_RESET_TIME);

		/* Do not need to scan since we know family ID */
		ret = mxt_lookup_bootloader_address(data, 0);
		if (ret)
			goto release_firmware;

		mxt_free_input_device(data);
		mxt_free_object_table(data);
	} else {
		mxt_irq_enable(data->irq);
	}

	INIT_COMPLETION(data->bl_completion);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD, false);
	if (ret) {
		/* Bootloader may still be unlocked from previous attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, false);
		if (ret)
			goto disable_irq;
	} else {
		LDBG("Unlocking bootloader\n");

		/* Unlock bootloader */
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret)
			goto disable_irq;
	}

	while (pos < fw->size) {
		if(reset == 0){
			ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, false);
		} else {
			ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, true);
		}
		if (ret)
			goto disable_irq;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;
		
		if(frame_size < 32){
			while(write_retry < 10){
			/* Write one frame to device */
				ret = mxt_bootloader_write(data, fw->data + pos, frame_size);
				if (ret){
					LDBG("write_retry %d!!!!\n",write_retry);
					write_retry++;
					mdelay(10 * write_retry);
				} else {
					write_retry = 0;
					break;
				}
			}
		} else {
			temp_frame = frame_size;
			while(temp_frame !=0){
				if(temp_frame >= 32){
					while(write_retry < 10){
					/* Write one frame to device */
						ret = mxt_bootloader_write(data, fw->data + pos + 32*index, 32);
						if (ret){
							LDBG("write_retry %d!!!!\n",write_retry);
							write_retry++;
							mdelay(10 * write_retry);
						} else {
							write_retry = 0;
							break;
						}
					}
					index++;
					temp_frame-=32;
				} else {	
					while(write_retry < 10){
					/* Write one frame to device */
						ret = mxt_bootloader_write(data, fw->data + pos + 32*index, temp_frame);
						if (ret){
							LDBG("write_retry %d!!!!\n",write_retry);
							write_retry++;
							mdelay(10 * write_retry);
						} else {
							write_retry = 0;
							break;
						}
					}
					temp_frame = 0;
				}
			}
		}
		
		temp_frame = 0;
		index = 0;
		
		if(write_retry == 9 && ret != 0)
			goto disable_irq;
			
		//msleep(1);
		if(reset == 0){
			ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS, false);
		} else {
			ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS, true);
		}
		if (ret) {
			retry++;

			/* Back off by 20ms per retry */
			mdelay(retry * 20);

			if (retry > 20) {
				LDBG("Retry count exceeded\n");
				goto disable_irq;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (frame % 50 == 0){
			LDBG("Sent %d frames, %d/%zd bytes\n",
				frame, pos, fw->size);
				mdelay(10);
		}
	}

	/* Wait for flash. */
	ret = mxt_wait_for_completion(data, &data->bl_completion,
				      MXT_FW_RESET_TIME);
	if (ret)
		goto disable_irq;

	LDBG("Sent %d frames, %d bytes\n", frame, pos);

	/*
	 * Wait for device to reset. Some bootloader versions do not assert
	 * the CHG line after bootloading has finished, so ignore potential
	 * errors.
	 */
	mxt_wait_for_completion(data, &data->bl_completion, MXT_FW_RESET_TIME);

	data->in_bootloader = false;

disable_irq:
	mxt_irq_disable(data->irq);
release_firmware:
	release_firmware(fw);
	data->updating_config = false;
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		LDBG("File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp) {
		LDBG("no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	error = mxt_update_file_name(dev, &data->fw_name, buf, count);
	if (error)
		return error;
		
	wake_lock(&mxt_wake_lock);
	error = mxt_load_fw(dev, 1, 0);
	wake_unlock(&mxt_wake_lock);
	
	if (error) {
		LDBG("The firmware update failed(%d)\n", error);
		count = error;
	} else {
		LDBG("The firmware update succeeded\n");

		data->suspended = false;
		msleep(MXT_RESET_TIME);
		wake_lock(&mxt_wake_lock);
		error = mxt_initialize(data);
		wake_unlock(&mxt_wake_lock);
		if (error)
			return error;
	}

	return count;
}

static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *cfg;
	int ret;

	if (data->in_bootloader) {
		LDBG("Not in appmode\n");
		return -EINVAL;
	}
	
	wake_lock(&mxt_wake_lock);
	
	ret = mxt_update_file_name(dev, &data->cfg_name, buf, count);
	if (ret)
		return ret;

	ret = request_firmware(&cfg, data->cfg_name, dev);
	if (ret < 0) {
		LDBG("Failure to request config file %s\n",
			data->cfg_name);
		ret = -ENOENT;
		goto out;
	}

	data->updating_config = true;

	mxt_free_input_device(data);

	if (data->suspended) {
		if (data->use_regulator) {
			mxt_irq_enable(data->irq);
			mxt_regulator_enable(data);
		} else {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg, 0);
	if (ret)
		goto release;

	ret = count;

release:
	release_firmware(cfg);
out:
	data->updating_config = false;
	wake_unlock(&mxt_wake_lock);
	return ret;
}

static int mxt_self_update_firmware(struct mxt_data *data){
	
	int error = 0;
	struct device *dev = &data->client->dev;
	u8 chip_fw_version = 0;
	u8 chip_fw_build = 0;
	
	if(data->info != NULL){
		chip_fw_version = data->info->version;
		chip_fw_build = data->info->build;
	} else {
		LDBG("The data->info is NULL\n");
	}
	
	//LDBG(" ++ %d,%d\n",(data->info->version),(data->info->build) );
	if((NEWEST_FW_VERSION != chip_fw_version) || (NEWEST_FW_BUILD != chip_fw_build)){
		LDBG(" ++ \n");
		/*mxt_self_update_cfg(touch_chip, 0);
		msleep(50);*/
		//file_name_tmp = touch_chip->cfg_name;
		
		touch_chip->cfg_name = MXT_CONFIG_NAME_1666T2;
	
		
		
		wake_lock(&mxt_wake_lock);
		
		error = mxt_load_fw(dev, 1, 1);
		
		if(error){
			LDBG("The self update firmware update failed(%d), retry again\n", error);
			error = mxt_load_fw(dev, 1, 1);
		}
		
		if(error){
			LDBG("The self update firmware force update failed(%d)\n", error);
			error = mxt_load_fw(dev, 0, 1);
		}
		
		wake_unlock(&mxt_wake_lock);
		touch_chip->cfg_name = NULL;
		if (error) {
			LDBG("The self update firmware update failed(%d)\n", error);
			data->update_result = 1;
			
		} else {
			data->update_result = 0;
			data->suspended = false;
			error = mxt_initialize(data);
			mdelay(MXT_RESET_TIME);
			LDBG("The self update firmware update succeeded\n");
			if (error)
				return error;
		}
	}
	return error;
}

static int mxt_self_update_cfg(struct mxt_data *data, int config_check){
	
	struct device *dev = &data->client->dev;
	const struct firmware *cfg;
	int ret = 0;

		
	LDBG(" ++ \n");
	if (data->in_bootloader) {
		LDBG("Not in appmode\n");
		return -EINVAL;
	}
	
	wake_lock(&mxt_wake_lock);
	ret = request_firmware(&cfg, MXT_CONFIG_NAME_1666T2, dev);
	
	if (ret < 0) {
		LDBG("Failure to request config file %s\n",
				MXT_CONFIG_NAME_1666T2);
		ret = -ENOENT;
		goto out;
	}
	data->updating_config = true;
	
	mxt_free_input_device(data);

	if (data->suspended) {
		if (data->use_regulator) {
			mxt_irq_enable(data->irq);
			mxt_regulator_enable(data);
		} else {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg, config_check);
	data->update_result = 0;
	if(ret)
		data->update_result = 2;
		
	release_firmware(cfg);	
	out:
	data->updating_config = false;
	wake_unlock(&mxt_wake_lock);
	return ret;
}
#ifdef mxt_pen
static void mxt_batlvl_report(struct work_struct *work)
{
	//LDBG(" ++\n");
	active_pen_mode = 0;
	power_supply_changed(&asus_pen_power_supplies[0]);
}
#endif
/* defined but not used
static void mxt_power_save(struct work_struct *work)
{
	LDBG(" ++\n");
	mxt_set_t7_power_cfg(touch_chip, MXT_POWER_CFG_POWERSAVE);
	power_save_mode = true;
	mxt_wake_irq_disable(touch_chip->irq);
}
*/

static void mxt_update_firmware_cfg(struct work_struct *work)
{
	int ret,retry = 0;
//	struct input_dev *input_dev = touch_chip->input_dev;
	
	LDBG(" ++ %d\n",build_version);

	if(build_version != 1 && touch_ic_status == true){
		/*ret = mxt_self_update_cfg(touch_chip, 1);
		mdelay(50);*/
		ret = mxt_self_update_firmware(touch_chip);
		mdelay(200);
		if(ret != 0) return;
		
		while(retry < 5){
			ret = mxt_self_update_cfg(touch_chip, 1);
			if(ret == 0){
				retry = 5;
				break;
			}
			mxt_update_crc(touch_chip, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);
			mdelay(10);
			mxt_soft_reset(touch_chip);
			mdelay(200);
			retry++;
		}
	}
}
		
static void mxt_force_updating(struct work_struct *work)
{
	struct mxt_data *data =
			container_of(work,
				struct mxt_data, work_upgrade);
	u8 fw_info;
	int error;
	
	LDBG(" ++ %d\n",build_version);
	msleep(20);
	if (data->info != NULL)
		fw_info = data->info->build;


	data->force_upgrade = 1;

	if (data->force_upgrade) {
		data->force_upgrade = 0;
		error = mxt_load_fw(&data->client->dev, 0, 1);
		data->in_bootloader = false;
		if (error) {
			dev_err(&data->client->dev, "The firmware update failed(%d)\n", error);
		} else {
			dev_info(&data->client->dev, "The firmware update succeeded\n");

		}
		msleep(MXT_RESET_TIME);
		mxt_initialize(data);
	}

	error = mxt_load_cfg(&data->client->dev);
	if (error) {
		dev_err(&data->client->dev, "The config update failed(%d)\n", error);
	}

	mxt_irq_enable(data->irq);
}

static ssize_t mxt_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		if (i == 1)
			mxt_debug_msg_enable(data);
		else
			mxt_debug_msg_disable(data);

		ret = count;
	} else {
		LDBG("debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t mxt_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;
	
	if (buf[0] == '0'){
		LDBG(" Close debug log\n");
		data->debug_log_enabled = 0;
	}
	else if (buf[0] == '2'){
		LDBG(" Open debug log status: 2\n");
		data->debug_log_enabled = 2;
	}else if (buf[0] == '3'){
		LDBG(" Open debug log status: 3\n");
		data->debug_log_enabled = 3;
	}else if(buf[0] == '4'){
		LDBG(" Open debug log status: 3\n");
		data->debug_log_enabled = 4;
	}
	
	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->debug_enabled = (i == 1);

		LDBG("%s\n", i ? "debug enabled" : "debug disabled");
		ret = count;
	} else {
		LDBG("debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}
/*  implicit declaration of function
//For Stylus data services++++++
static int read_update_result(void){
	char file_path[] = "/factory/stylus_result";
	struct file *filePtr = NULL;
	char temp_status[2];
	mm_segment_t oldfs;
	loff_t pos = 0;
	int len;
	
	filePtr = filp_open(file_path, O_RDONLY, 0);
	if(!IS_ERR_OR_NULL(filePtr)) {
		oldfs = get_fs();
		set_fs(get_ds());
		pos = 0;
		len = filePtr->f_op->read(filePtr, temp_status, sizeof(temp_status), &pos);
		set_fs(oldfs);
		filp_close(filePtr, NULL);
		touch_chip->stylus_status.attach =  temp_status[0] - '0';
		touch_chip->stylus_status.active =  temp_status[1] - '0';
		LDBG(" read %d - %d. \n",  touch_chip->stylus_status.attach, touch_chip->stylus_status.active);
		return 0;
	}else if(PTR_ERR(filePtr) == -ENOENT) {
		touch_chip->stylus_status.attach = 0;
		touch_chip->stylus_status.active = 0;
		LDBG(" %s not found\n",  file_path);
		return 1;
	} else {
		touch_chip->stylus_status.attach = 0;
		touch_chip->stylus_status.active = 0;
		LDBG(" %s open error\n",  file_path);
		return 1;
	}
	return 1;
}

static int update_result_to_file(int status){
	
	char file_path[] = "/factory/stylus_result";
	struct file *filePtr = NULL;
	mm_segment_t oldfs;
	char temp_status[2];
	loff_t pos = 0;
	int len;
	
	sprintf(temp_status, "%d", status);
	filePtr = filp_open(file_path, O_RDWR|O_CREAT, (S_IWUSR|S_IRUGO));
	if(!IS_ERR_OR_NULL(filePtr)) {
		oldfs = get_fs();
		set_fs(get_ds());
		pos = 0;
		len = filePtr->f_op->write(filePtr, &temp_status, sizeof(temp_status), &pos);
		set_fs(oldfs);
		filp_close(filePtr, NULL);
		LDBG(" write %s done. \n",  file_path);
		return 0;
	}else if(PTR_ERR(filePtr) == -ENOENT) {
		LDBG(" %s not found\n",  file_path);
		return 1;
	} else {
		LDBG(" %s open error\n",  file_path);
		return 1;
	}
	return 1;
}
*/
static ssize_t atmel_get_ssn_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "NULL\n");
}
static ssize_t atmel_get_stylus_type(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}
static ssize_t atmel_get_model_name(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Z-STYLUS\n");
}
/*  implicit declaration of function
static ssize_t atmel_get_attached(struct device *dev,struct device_attribute *attr, char *buf)
{
	read_update_result();
	return sprintf(buf, "%d\n",touch_chip->stylus_status.attach);
	//return sprintf(buf, "1\n");
}
static ssize_t atmel_get_active_flag(struct device *dev,struct device_attribute *attr, char *buf)
{
	read_update_result();
	return sprintf(buf, "%d\n",touch_chip->stylus_status.active);
	//return sprintf(buf, "1\n");
}
*/

static ssize_t atmel_update_success(struct device *dev,struct device_attribute *attr, char *buf)
{
	int result = 0;
	if(touch_chip->stylus_status.attach == 1){
		touch_chip->stylus_status.active = 2; //need to update
	}
	//update_result_to_file(11);
	return sprintf(buf, "%d\n",result);
}

//For Stylus data services------	

static ssize_t atmel_get_touch_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	LDBG(" touch status=%d. \n",touch_ic_status);
	return sprintf(buf, "%d \n",touch_ic_status);
}

static ssize_t atmel_get_touch_id(struct device *dev,struct device_attribute *attr, char *buf)
{
	gpio_direction_output(touch_chip->int_gpio, 1);	//reset set high
	return sprintf(buf, "%d \n",touch_ic_status);
}

static ssize_t atmel_enable_gpio(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(build_version != 1){
		LDBG(" ++ skip,Not in ENG mode\n");
		return 0;
	}
	LDBG(" Enable_irq(%d).\n",touch_chip->irq);
	mxt_irq_enable(touch_chip->irq);
	return 0;
}

static ssize_t atmel_disable_gpio(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(build_version != 1){
		LDBG(" ++ skip,Not in ENG mode\n");
		return 0;
	}
	LDBG(" Disable_irq(%d).\n",touch_chip->irq);
	mxt_irq_disable(touch_chip->irq);
	return 0;
}
static ssize_t mxt_gpio_enable_disable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if(buf[0] == '0'){
		LDBG(" Disable_irq(%d).\n",touch_chip->irq);
		mxt_irq_disable(touch_chip->irq);
	} else {
		LDBG(" Enable_irq(%d).\n",touch_chip->irq);
		mxt_irq_enable(touch_chip->irq);
	}
	return count;
}
static ssize_t mxt_report_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	active_rep = (buf[0] - '0')*10 + (buf[1]- '0');
	idle_rep = (buf[3] - '0')*10 + (buf[4]- '0');
	LDBG(" active_rep(%d). idle_rep(%d)\n",active_rep, idle_rep);
	return count;
}

static ssize_t mxt_gesture_report_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	gesture_active_rep = (buf[0] - '0')*10 + (buf[1]- '0');
	gesture_idle_rep = (buf[3] - '0')*10 + (buf[4]- '0');
	LDBG(" gesture_active_rep(%d). gesture_idle_rep(%d)\n",gesture_active_rep, gesture_idle_rep);
	return count;
}

static ssize_t atmel_update_firmware_config(struct device *dev,struct device_attribute *attr, char *buf)
{	
	int err = 0,retry = 0;
//	struct input_dev *input_dev = touch_chip->input_dev;
	
	if(build_version != 1){
		LDBG(" ++ skip,Not in ENG mode\n");
		return sprintf(buf, "0\n");
	}
	LDBG(" ,build_version:%d++ \n",build_version);
	
	touch_chip->update_result = 0;
	mxt_self_update_firmware(touch_chip);
	mdelay(300);
	if(touch_chip->update_result == 0){
		//mxt_irq_disable(touch_chip->irq);
		err = mxt_self_update_cfg(touch_chip, 0);
			LDBG(" mxt_self_update_cfg fail,err:%d!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",err);
		while(err != 0 && retry < 5)
		{
			mxt_update_crc(touch_chip, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);
			mdelay(100);
			mxt_soft_reset(touch_chip);

			mdelay(300);
			err = mxt_self_update_cfg(touch_chip, 0);
			LDBG(" mxt_self_update_cfg fail, retry:%d\n",retry);
			retry++;
		}
		//mxt_irq_enable(touch_chip->irq);
	} else {
		return sprintf(buf, "%d \n",touch_chip->update_result);
	}
	//return touch_chip->update_result;
	return sprintf(buf, "%d \n",touch_chip->update_result);
}

/* Add by Tom Cheng for cat sys/kernel/android_touch/tp_id (self_test) */
static ssize_t mxt_tp_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", MXT_TP_ID);
}
static DEVICE_ATTR(tp_id, S_IRUGO, mxt_tp_id_show, NULL);

static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(config_csum, S_IRUGO, mxt_config_csum_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
		   mxt_debug_enable_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, NULL,
		   mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_debug_notify_show, NULL);
static DEVICE_ATTR(touch_status, S_IRUGO, atmel_get_touch_status, NULL);
//For Stylus data services++++++
static DEVICE_ATTR(ssn_info, S_IRUGO, atmel_get_ssn_info, NULL);
static DEVICE_ATTR(stylus_type, S_IRUGO, atmel_get_stylus_type, NULL);
static DEVICE_ATTR(model_name, S_IRUGO, atmel_get_model_name, NULL);
//static DEVICE_ATTR(attached, S_IRUGO, atmel_get_attached, NULL);
//static DEVICE_ATTR(active_flag, S_IRUGO, atmel_get_active_flag, NULL);
static DEVICE_ATTR(update_success, S_IRUGO, atmel_update_success, NULL);
//For Stylus data services------	
static DEVICE_ATTR(touch_id, S_IRUGO, atmel_get_touch_id, NULL);
static DEVICE_ATTR(enable_irq, S_IWUSR | S_IRUSR, atmel_enable_gpio, mxt_gpio_enable_disable);
static DEVICE_ATTR(disable_irq, S_IRUGO, atmel_disable_gpio, NULL);
static DEVICE_ATTR(update_touch_firmware_config, S_IRUGO, atmel_update_firmware_config, NULL);
static DEVICE_ATTR(set_report_rate, S_IWUSR | S_IRUSR, NULL, mxt_report_rate_store);
static DEVICE_ATTR(set_gesture_report_rate, S_IWUSR | S_IRUSR, NULL, mxt_gesture_report_rate_store);


static struct attribute *mxt_attrs[] = {
	&dev_attr_tp_id.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_config_csum.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_touch_status.attr,
	//For Stylus data services++++++
	&dev_attr_ssn_info.attr,
	&dev_attr_stylus_type.attr,
	&dev_attr_model_name.attr,
	//&dev_attr_attached.attr,
	//&dev_attr_active_flag.attr,
	&dev_attr_update_success.attr,
	//For Stylus data services------	
	&dev_attr_touch_id.attr,
	&dev_attr_update_touch_firmware_config.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_disable_irq.attr,
	&dev_attr_set_report_rate.attr,
	&dev_attr_set_gesture_report_rate.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

#ifdef mxt_double_tap
static int mxt_proc_double_tap_read(struct seq_file *buf, void *v)
{
	LDBG("double_tap_ENABLE = %d\n", touch_chip->double_tap_enable);
	return 0;
}

static ssize_t mxt_proc_double_tap_write(struct file *filp, const char __user *buf, size_t len, loff_t *data)
{
	if (buf[0]=='0'){
		touch_chip->double_tap_enable = 0;	
	}else{
		touch_chip->double_tap_enable = 1;
	}
	LDBG("double_tap_ENABLE = %d\n", touch_chip->double_tap_enable);
	return len;
}

static int mxt_proc_double_tap_open(struct inode *inode, struct  file *file)
{
	return single_open(file, mxt_proc_double_tap_read, NULL);
}

static const struct file_operations mxt_double_tap_fops =
{
	.owner = THIS_MODULE,
	.open = mxt_proc_double_tap_open,
	.read = seq_read,
	.write = mxt_proc_double_tap_write,
};

void mxt_create_proc_double_tap_file(void)
{	
	mxt_proc_double_tap_file = proc_create(MXT_PROC_DOUBLE_TAP_FILE, 0666, NULL, &mxt_double_tap_fops);
	if(mxt_proc_double_tap_file)
	{
		//LDBG("%s:: proc double_tap file create sucessed\n");
	}
	else
	{
		LDBG(" proc double_tap file create failed\n");
	}
	return;
}

void mxt_remove_proc_double_tap_file(void)
{
	extern struct proc_dir_entry proc_root;
	LDBG(" proc double_tap file removed.");
	remove_proc_entry(MXT_PROC_DOUBLE_TAP_FILE, &proc_root);
	return;
}
#endif

#ifdef mxt_gesture
static int mxt_proc_gesture_read(struct seq_file *buf, void *v)
{
	/*LDBG("%s: gesture_enable = %d\n",  touch_chip->gesture_enable);
	LDBG("%s: gesture_type_w = %d\n",  touch_chip->gesture_type_1);
	LDBG("%s: gesture_type_s = %d\n",  touch_chip->gesture_type_2);
	LDBG("%s: gesture_type_e = %d\n",  touch_chip->gesture_type_3);
	LDBG("%s: gesture_type_c = %d\n",  touch_chip->gesture_type_4);
	LDBG("%s: gesture_type_z = %d\n",  touch_chip->gesture_type_5);
	LDBG("%s: gesture_type_v = %d\n",  touch_chip->gesture_type_6);*/
	return 0;
}

static ssize_t mxt_proc_gesture_write(struct file *filp, const char __user *buf, size_t len, loff_t *data)
{
	LDBG("gesture_ENABLE0 = %c\n",  buf[0]);
	if (buf[0]=='0'){
		touch_chip->gesture_enable = 0;	
	}else{
		touch_chip->gesture_enable = 1;
	}
	if (buf[1]=='0'){//w
		touch_chip->gesture_type_1 = 0;	
	}else{
		touch_chip->gesture_type_1 = 1;
	}
	if (buf[2]=='0'){//s
		touch_chip->gesture_type_2 = 0;	
	}else{
		touch_chip->gesture_type_2 = 1;
	}
	if (buf[3]=='0'){//e
		touch_chip->gesture_type_3 = 0;	
	}else{
		touch_chip->gesture_type_3 = 1;
	}
	if (buf[4]=='0'){//c
		touch_chip->gesture_type_4 = 0;	
	}else{
		touch_chip->gesture_type_4 = 1;
	}
	if (buf[5]=='0'){//z
		touch_chip->gesture_type_5 = 0;	
	}else{
		touch_chip->gesture_type_5 = 1;
	}
	if (buf[6]=='0'){//v
		touch_chip->gesture_type_6 = 0;	
	}else{
		touch_chip->gesture_type_6 = 1;
	}
	return len;
}

static int mxt_proc_gesture_open(struct inode *inode, struct  file *file)
{
	return single_open(file, mxt_proc_gesture_read, NULL);
}

static const struct file_operations mxt_gesture_fops =
{
	.owner = THIS_MODULE,
	.open = mxt_proc_gesture_open,
	.read = seq_read,
	.write = mxt_proc_gesture_write,
};

void mxt_create_proc_gesture_file(void)
{
	mxt_proc_gesture_file = proc_create(MXT_PROC_GESTURE_FILE, 0666, NULL, &mxt_gesture_fops);
	if(mxt_proc_gesture_file)
	{
		//LDBG(" proc gesture file create sucessed\n");
	}
	else
	{
		LDBG(" proc gesture file create failed\n");
	}
	return;
}

void mxt_remove_proc_gesture_file(void)
{
	extern struct proc_dir_entry proc_root;
	LDBG(" proc gesture file removed.");
	remove_proc_entry(MXT_PROC_GESTURE_FILE, &proc_root);
	return;
}
#endif

#ifdef mxt_stylus
static int mxt_proc_stylus_read(struct seq_file *buf, void *v)
{
	return 0;
}

static ssize_t mxt_proc_stylus_write(struct file *filp, const char __user *buf, size_t len, loff_t *data)
{
	if (buf[0]=='0'){
		touch_chip->z_stylus = 0;
		mxt_set_t8_acquisition_cfg(touch_chip, 10,
			(MXT_MEASALLOW_MUTUALTCH | MXT_MEASALLOW_SELFTCH));
		/*mxt_t107_configuration(touch_chip, 0, MXT_T107_DISABLE);*/
	}else{
		touch_chip->z_stylus = 1;
		mxt_set_t8_acquisition_cfg(touch_chip, 10,
			(MXT_MEASALLOW_MUTUALTCH | MXT_MEASALLOW_SELFTCH | MXT_MEASALLOW_ACTVSTY));
		/*mxt_t107_configuration(touch_chip, 0, MXT_T107_ENABLE);*/
	}
	LDBG(" z_stylus = %d\n", touch_chip->z_stylus);	
	return len;
}

static int mxt_proc_stylus_open(struct inode *inode, struct  file *file)
{
	return single_open(file, mxt_proc_stylus_read, NULL);
}

static const struct file_operations mxt_stylus_fops =
{
	.owner = THIS_MODULE,
	.open = mxt_proc_stylus_open,
	.read = seq_read,
	.write = mxt_proc_stylus_write,
};

void mxt_create_proc_stylus_file(void)
{	
	mxt_proc_stylus_file = proc_create(MXT_PROC_STYLUS_FILE, 0666, NULL, &mxt_stylus_fops);
	if(mxt_proc_stylus_file)
	{
		//LDBG(": proc double_tap file create sucessed\n");
	}
	else
	{
		LDBG(" proc double_tap file create failed\n");
	}
	return;
}

void mxt_remove_proc_stylus_file(void)
{
	extern struct proc_dir_entry proc_root;
	LDBG(" proc double_tap file removed.");
	remove_proc_entry(MXT_PROC_STYLUS_FILE, &proc_root);
	return;
}
#endif

// add by leo for testtest ++
static int mxt_proc_logtool_read(struct seq_file *buf, void *v)
{
	LDBG(" data->debug_log_enabled = %d\n", touch_chip->debug_log_enabled);
	return 0;
}

static ssize_t mxt_proc_logtool_write(struct file *filp, const char __user *buf, size_t len, loff_t *data)
{
	// u8 i;

	if (buf[0] == '0'){
		LDBG(" Close debug log\n");
		touch_chip->debug_log_enabled = 0;
	}
	else if (buf[0] == '1'){
		LDBG(" Open debug log status: 2\n");
		touch_chip->debug_log_enabled = 2;
	}

	/*
	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		touch_chip->debug_enabled = (i == 1);

		LDBG(" %s\n", i ? "debug enabled" : "debug disabled");
	} else {
		LDBG(" debug_enabled write error\n");
	}
	*/
	return len;
}

static int mxt_proc_logtool_open(struct inode *inode, struct  file *file)
{
	return single_open(file, mxt_proc_logtool_read, NULL);
}

static const struct file_operations mxt_logtool_fops =
{
	.owner = THIS_MODULE,
	.open = mxt_proc_logtool_open,
	.read = seq_read,
	.write = mxt_proc_logtool_write,
};

void mxt_create_proc_logtool_file(void)
{
	mxt_proc_stylus_file = proc_create(MXT_PROC_LOGTOOL_FILE, 0666, NULL, &mxt_logtool_fops);
	if(mxt_proc_stylus_file)
	{
		//LDBG(": proc logtool file create sucessed\n");
	}
	else
	{
		LDBG(" proc logtool file create failed\n");
	}
	return;
}

void mxt_remove_proc_logtool_file(void)
{
	extern struct proc_dir_entry proc_root;
	LDBG(" proc logtool file removed.");
	remove_proc_entry(MXT_PROC_LOGTOOL_FILE, &proc_root);
	return;
}
// add by leo for testtest --

int hall_status_notify(void){
	
	if(touch_ic_status == true){
		LDBG("\n");
		if((touch_chip->suspended == true) && (hall_trigger_suspend == true)){
			hall_status = true;
			mxt_set_t7_power_cfg(touch_chip, MXT_POWER_CFG_DEEPSLEEP);	
		}
		if((touch_chip->suspended == true) && (hall_trigger_suspend == false)){
			if((touch_chip->double_tap_enable == 0 && touch_chip->gesture_enable == 0) || (build_version == 1)){
			} else {
				mxt_set_t7_power_cfg(touch_chip, MXT_POWER_CFG_GESTURE);
				mxt_wake_irq_enable(touch_chip->irq);
			}
		}
	}
	return 0;
}
EXPORT_SYMBOL(hall_status_notify);

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	unsigned int num_mt_slots;
	int id;

	if (!input_dev)
		return;

	num_mt_slots = data->num_touchids + data->num_stylusids;

	for (id = 0; id < num_mt_slots; id++) {
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	mxt_input_sync(data);
}

static void mxt_start(struct mxt_data *data)
{
	if (!data->suspended || data->in_bootloader)
		return;

	if (data->use_regulator) {
		mxt_irq_enable(data->irq);
		mxt_regulator_enable(data);
	} else {
		/*
		 * Discard any messages still in message buffer
		 * from before chip went to sleep
		 */
		mxt_process_messages_until_invalid(data);
		
		enable_stylus(data);
		
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		
		/*if(data->z_stylus == 1){
			mxt_t107_configuration(touch_chip, 0, MXT_T107_ENABLE);
		} else {
			mxt_t107_configuration(touch_chip, 0, MXT_T107_DISABLE);
		}*/

		mxt_set_t100_multitouchscreen_cfg(data, 0, 
		(MXT_T100_CTRL_ENABLE | MXT_T100_CTRL_RPTEN | MXT_T100_CTRL_DISSCRMSG | MXT_T100_CTRL_SCANEN));
		
		/* Recalibrate since chip has been in deep sleep */
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);

		/* Disable double tap and report messages */
		if (data->T93_address) {
			mxt_t93_configuration(data, 0, MXT_T93_DISABLE);
		}

		/* Disable gesture and report messages */
		if (data->T92_address) {
			mxt_t92_configuration(data, 0, MXT_T92_DISABLE);
		}

		//mxt_acquire_irq(data);
		if (data->use_retrigen_workaround) {
			mxt_process_messages_until_invalid(data);
		}	
	}
	if((data->double_tap_enable == 0 && data->gesture_enable == 0) || (build_version == 1) || (hall_status == true)){
		mxt_irq_disable(data->irq);
		hall_status = false;
	}else{
		mxt_wake_irq_disable(data->irq);
	}
	mxt_irq_enable(data->irq);
	
	data->suspended = false;
	
}

static void gesture_disable(struct mxt_data *data)
{
	/* Enable double tap and report messages */
	if (data->T93_address) {
		mxt_t93_configuration(data, 0, MXT_T93_DISABLE);
	}

	/* Enable gesture and report messages */
	if (data->T92_address) {
		mxt_t92_configuration(data, 0, MXT_T92_DISABLE);
	}

}

static void disable_stylus(struct mxt_data *data)
{
	//LDBG("[Atmel1] %s:");

	//Rewrite T8 T104 T107config
	//mxt_set_t8_cfg(data, MXT_STYLUS_DISABLE);
	mxt_set_t8_acquisition_cfg(data, 10, 3);
	//mxt_set_t8_acquisition_cfg(data, 11, 2);
	//mxt_set_t8_acquisition_cfg(data, 4, 0);
	//mxt_set_t8_acquisition_cfg(data, 6, 255);
	//mxt_set_t8_acquisition_cfg(data, 7, 1);
	//mxt_set_t8_acquisition_cfg(data, 8, 0);
	//mxt_set_t8_acquisition_cfg(data, 9, 0);
	//mxt_set_t104_cfg(data, MXT_STYLUS_DISABLE);
	mxt_set_t107_cfg(data, 97 ,MXT_STYLUS_DISABLE);

}

static void enable_stylus(struct mxt_data *data)
{
	//LDBG("[Atmel1] %s:");
	
	//Rewrite T8 T104 T107config
	//mxt_set_t8_cfg(data, MXT_STYLUS_ENABLE);
	if(data->z_stylus == 1){
		mxt_set_t8_acquisition_cfg(data, 10, 19);
	} else {
		mxt_set_t8_acquisition_cfg(data, 10, 3);
	}
	
	//mxt_set_t104_cfg(data, MXT_STYLUS_ENABLE);
	//mxt_set_t107_cfg(data, 97, MXT_STYLUS_ENABLE);
}

static void mxt_stop(struct mxt_data *data)
{
	if (data->suspended || data->in_bootloader || data->updating_config)
		return;

	mxt_irq_disable(data->irq); 

	/* Enable double tap and report messages */
	if (data->T93_address && data->double_tap_enable == 1) {
		mxt_t93_configuration(data, 0, MXT_T93_ENABLE);
	}

	/* Enable gesture and report messages */
	if (data->T92_address && data->gesture_enable == 1) {
		mxt_t92_configuration(data, 0, MXT_T92_ENABLE);
	}
	
	if(hall_trigger_suspend == true)
		hall_status = true;

	if (data->use_regulator){
		mxt_regulator_disable(data);
	} else {
		if((data->double_tap_enable == 0 && data->gesture_enable == 0) || (build_version == 1)  || (hall_trigger_suspend == true)){
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);		
			LDBG(" ATMEL Touch DEEPSLEEP ... \n");
		} else {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_GESTURE);
			mxt_wake_irq_enable(data->irq);
			LDBG(" ATMEL Touch GESTURE ... \n");
		}
		disable_stylus(data);
		/*if(data->z_stylus == 1)
			mxt_t107_configuration(touch_chip, 0, MXT_T107_DISABLE);*/
		
		mxt_set_t100_multitouchscreen_cfg(data, 0, 
		(MXT_T100_CTRL_ENABLE | MXT_T100_CTRL_DISSCRMSG | MXT_T100_CTRL_SCANEN));
	}
	

#ifdef mxt_pen
	if(active_pen_mode == 1){
		queue_delayed_work(maxtouch_wq, &data->stylus_batlvl_work, 0);
	}
#endif
		
	mxt_reset_slots(data);
	mxt_irq_enable(data->irq);
	data->suspended = true;
	LDBG(" ATMEL Touch Suspend Done ... \n");
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}

static int mxt_touch_sysfs_init(void)
{
	int ret;
	
	android_touch_kobj = kobject_create_and_add("android_touch", kernel_kobj);
	if (android_touch_kobj == NULL) 
	{
		LDBG(" subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
	
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_status.attr);
	if (ret) 
	{
		LDBG(" create_file touch status failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_set_report_rate.attr);
	if (ret) 
	{
		LDBG(" create_file set report rate failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_set_gesture_report_rate.attr);
	if (ret) 
	{
		LDBG(" create_file set gesture report rate failed\n");
		return ret;
	}
	
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_ssn_info.attr);
	if (ret) 
	{
		LDBG(" create_file dev_attr_ssn_info failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_stylus_type.attr);
	if (ret) 
	{
		LDBG(" create_file dev_attr_stylus_type failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_model_name.attr);
	if (ret) 
	{
		LDBG(" create_file dev_attr_model_name failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_update_success.attr);
	if (ret) 
	{
		LDBG(" create_file dev_attr_update_success rate failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_fw_version.attr);
	if (ret) 
	{
		LDBG(" create_file &dev_attr_fw_version.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_id.attr);
	if (ret) 
	{
		LDBG(" create_file &dev_attr_tp_id.attr failed\n");
		return ret;
	}
	
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_hw_version.attr);
	if (ret) {
		LDBG(" create_file &dev_attr_hw_version.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_update_fw.attr);
	if (ret) {
		LDBG(" create_file &dev_attr_update_fw.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_update_cfg.attr);
	if (ret) {
		LDBG(" create_file &dev_attr_update_cfg.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_config_csum.attr);
	if (ret) {
		LDBG(" create_file &dev_attr_config_csum.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_enable.attr);
	if (ret) {
		LDBG(" create_file &dev_attr_debug_enable.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_update_touch_firmware_config.attr);
	if (ret) {
		LDBG(" create_file &dev_attr_update_touch_firmware_config.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_enable_irq.attr);
	if (ret) {
		LDBG(" create_file &dev_attr_enable_gpio.attr failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_disable_irq.attr);
	if (ret) {
		LDBG(" create_file &dev_attr_disable_gpio.attr failed\n");
		return ret;
	}
	return 0;
}

static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{ 
	return sprintf(buf, "%u.%u.%02X-%u.%u-%06x\n",
		 touch_chip->info->version >> 4, touch_chip->info->version & 0xf,
		 touch_chip->info->build,touch_chip->maj_version,touch_chip->min_version,touch_chip->config_crc);
}

static ssize_t touch_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", touch_ic_status);
}

#ifdef CONFIG_OF
static struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;
	//u32 *keymap;
	//int proplen, ret;

	LDBG(".1\n");
	
	if (!client->dev.of_node)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	/* reset gpio */
	pdata->gpio_reset = of_get_named_gpio_flags(client->dev.of_node,
		"rst-gpio", 0, NULL);
	pdata->gpio_irq = of_get_named_gpio_flags(client->dev.of_node,
		"int-gpio", 0, NULL);	

	/*of_property_read_string(client->dev.of_node, "atmel,cfg_name",
				&pdata->cfg_name);

	of_property_read_string(client->dev.of_node, "atmel,input_name",
				&pdata->input_name);

	if (of_find_property(client->dev.of_node, "linux,gpio-keymap",
			     &proplen)) {
		pdata->t19_num_keys = proplen / sizeof(u32);

		keymap = devm_kzalloc(&client->dev,
				pdata->t19_num_keys * sizeof(keymap[0]),
				GFP_KERNEL);
		if (!keymap)
			return ERR_PTR(-ENOMEM);

		ret = of_property_read_u32_array(client->dev.of_node,
			"linux,gpio-keymap", keymap, pdata->t19_num_keys);
		if (ret) {
			dev_err(&client->dev,
				"Unable to read device tree key codes: %d\n",
				 ret);
			return NULL;
		}

		pdata->t19_keymap = keymap;
	}*/

	return pdata;
}
#else
static struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;
	
	LDBG(".2\n");

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	/* Set default parameters */
	pdata->irqflags = IRQF_TRIGGER_FALLING;

	return pdata;
}
#endif

static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mxt_data *data;
	const struct mxt_platform_data *pdata;
	int error,err,retry=0;
	
	/* Check is CPT LCM */
	int lcm_id_gpio = 0;
	
	LDBG(". ver 1 :%s\n",DRIVER_VERSION);
        
	lcm_id_gpio = of_get_named_gpio(client->dev.of_node, "lcmid0-gpio", 0);
        is_cpt_lcm = gpio_get_value(lcm_id_gpio);
	LDBG(" LCM ID GPIO [%d] = [%d] \n", lcm_id_gpio ,is_cpt_lcm);
	if (is_cpt_lcm == 0) {
		LDBG(" Is OBE panel , LCM ID =  [%d] , skip probe\n", is_cpt_lcm);
		return 0;
	}
	/* Add by Tom Cheng for cat sys/kernel/android_touch/tp_id (self_test) */
	MXT_TP_ID = 4;
	
	LDBG(" mxt_touch_sysfs_init \n");
	mxt_touch_sysfs_init();
		
	pdata = dev_get_platdata(&client->dev);
	if (!pdata) {
		pdata = mxt_parse_dt(client);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}
	
	maxtouch_wq = create_singlethread_workqueue("maxtouch_wq");
	if (!maxtouch_wq) {
		dev_err(&client->dev, "create_singlethread_workqueue error\n");
		return -ENOMEM;
	}

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	//add by josh +++
	data->int_gpio = pdata->gpio_irq;
	data->reset_gpio = pdata->gpio_reset;
	//client->irq = gpio_to_irq(data->int_gpio);
	//client->irq = pdata->gpio_irq;
	data->maj_version = 0;
    data->min_version = 0;
	//add by josh ---
	
	data->client = client;
	data->pdata = pdata;
	data->irq = client->irq;
	data->double_tap_enable = 0;
	data->gesture_enable = 0;
	data->gesture_type_1 = 0;
	data->gesture_type_2 = 0;
	data->gesture_type_3 = 0;
	data->gesture_type_4 = 0;
	data->gesture_type_5 = 0;
	data->gesture_type_6 = 0;
	data->z_stylus = 0;
	data->debug_log_enabled = 0;
	data->btn_stylus = 0;
	data->btn_stylus2 = 0;
	data->stylus_status.attach = 0;
	data->stylus_status.active = 0;
	i2c_set_clientdata(client, data);
	touch_chip = data;
	
	//*********************************************************************************************************
	// spin lock init
	//*********************************************************************************************************
	spin_lock_init(&data->touch_spinlock);
	
	data->mxt_wq = create_singlethread_workqueue("mxt_wq");
	INIT_DELAYED_WORK(&data->touch_chip_firmware_upgrade_work, mxt_update_firmware_cfg);
	queue_delayed_work(data->mxt_wq, &data->touch_chip_firmware_upgrade_work, 15*HZ);
	//INIT_DELAYED_WORK(&data->power_save_work, mxt_power_save);
//	INIT_DELAYED_WORK(&data->stylus_batlvl_work, mxt_batlvl_report);
	
	//*********************************************************************************************************
	// Power Pin
	// VGP2_PMU -> IOVCC (1.8V)
	//*********************************************************************************************************
	//*********************************************************************************************************
	// TODO Interrupt Pin add by Josh 
	//*********************************************************************************************************
	if( gpio_request(data->int_gpio, "AtmelTouch-irq") != 0 )
	{
		LDBG(": interrupt gpio %d request fail.\n",data->int_gpio);
	}
	err = gpio_direction_input(data->int_gpio);
	if(err)
	{
		LDBG(" Failed to set interrupt direction, error=%d.\n", err);
	}
	else
	{
		LDBG(" init interrupt pin OK. \n");
	}
	//*********************************************************************************************************
	// TODO Reset Pin
	//*********************************************************************************************************
	if( gpio_request(data->reset_gpio, "AtmelTouch-reset") != 0)
	{
		LDBG(": reset gpio %d request fail.\n",data->reset_gpio);
	}
	err = gpio_direction_output(data->reset_gpio, 0);	//reset set keep low
	if(err)
	{
		LDBG(" Failed to set interrupt direction, error=%d.\n", err);
	} else {
		LDBG(" init reset pin OK. \n");
	}
	msleep(100);
	
	
	if (data->pdata->cfg_name)
		mxt_update_file_name(&data->client->dev,
				     &data->cfg_name,
				     data->pdata->cfg_name,
				     strlen(data->pdata->cfg_name));

	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	mutex_init(&data->debug_msg_lock);
	
	//Wakelock Protect Start
	wake_lock_init(&mxt_wake_lock, WAKE_LOCK_SUSPEND, "atmel_wake_lock");
	//Wakelock Protect End
	
	INIT_WORK(&data->work_upgrade, mxt_force_updating);
	INIT_WORK(&data->work_function, mxt_work_function);
		
	dev_err(&client->dev, "Register interrupt [%d]...\n", client->irq);
	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
				     pdata->irqflags | IRQF_ONESHOT | IRQF_NO_SUSPEND , 
				     client->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		//goto err_free_mem;
	}

	mxt_probe_regulators(data);

	disable_irq(data->irq);

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
			error);
		goto err_free_irq;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND_DISABLE
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif	
	while(retry < 3){

		error = mxt_initialize(data);
		
		if(data->in_bootloader)
			break;
	
		if(error != 0 && touch_ic_status != true){

			LDBG(" retry time: %d\n",retry);
			retry++;
		} else {
			break;
		}
	}
	
	if(data->in_bootloader){
		LDBG(" Is in bootloader mode!!!!!");
		queue_work(maxtouch_wq, &data->work_upgrade);
		cancel_delayed_work(&data->touch_chip_firmware_upgrade_work);
	}
	if (error)
		goto err_remove_mem_access;
	

	if(!data->in_bootloader){
		mxt_read_t38_data(data);
		LDBG(" Config Version : %d.%d\n",data->maj_version,data->min_version);
	
		gesture_disable(data);
	}
#ifdef mxt_pen
	error = power_supply_register(&client->dev, &asus_pen_power_supplies[0]);
#endif
    if (error) {
        dev_err(&client->dev, "Fail to register pen battery\n");
        goto batt_err_reg_fail_battery;
    }

	enable_irq(data->irq);
	//init gesture proc function
	mxt_create_proc_double_tap_file();
	mxt_create_proc_gesture_file();
	#ifdef mxt_stylus
	mxt_create_proc_stylus_file();
	#endif
	mxt_create_proc_logtool_file(); // add by leo for testtest

	
	data->touch_sdev.name = TOUCH_SDEV_NAME;
	data->touch_sdev.print_name = touch_switch_name;
	data->touch_sdev.print_state = touch_switch_state;
	if(switch_dev_register(&data->touch_sdev) < 0){
		LDBG(" touch switch device register failed!\n");
	}
	
	// reg callback
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
	if (fb_register_client(&atmel_fb_notifier)) {
		LDBG("Register FB client failed!\n");
	}
#endif

	//mxt_self_update_cfg(data);
	LDBG(" ++ success touch_ic_status: %d\n",touch_ic_status);
	return 0;
batt_err_reg_fail_battery:
err_remove_mem_access:
	sysfs_remove_bin_file(&client->dev.kobj, &data->mem_access_attr);
	data->mem_access_attr.attr.name = NULL;
err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
err_free_irq:
	free_irq(client->irq, data);
//err_free_mem:
	cancel_delayed_work(&data->touch_chip_firmware_upgrade_work);
	LDBG(" ++ fail touch_ic_status: %d\n",touch_ic_status);
	//kfree(data);
	destroy_workqueue(maxtouch_wq);
	maxtouch_wq = NULL;
	mxt_remove_proc_gesture_file();
	mxt_remove_proc_double_tap_file();
	mxt_remove_proc_logtool_file(); // add by leo for testtest
	return error;
}

static int mxt_irq_disable(unsigned int irq)
{
	unsigned long irqflags;
	
	spin_lock_irqsave(&touch_chip->touch_spinlock, irqflags);
	if(irq_enable == true)
	{
		irq_enable = false;
		disable_irq_nosync(touch_chip->irq);
	}
	spin_unlock_irqrestore(&touch_chip->touch_spinlock, irqflags);

	return 0;
}

static int mxt_irq_enable(unsigned int irq)
{
    unsigned long irqflags = 0;
	    
    spin_lock_irqsave(&touch_chip->touch_spinlock, irqflags);
    if(irq_enable == false)
	{
		irq_enable = true;
		enable_irq(touch_chip->irq);
	}	
    spin_unlock_irqrestore(&touch_chip->touch_spinlock, irqflags);
    
    return 0;
}


static int mxt_wake_irq_disable(unsigned int irq)
{
	unsigned long irqflags;
	
	spin_lock_irqsave(&touch_chip->touch_spinlock, irqflags);
	if(irq_wake_enable == true)
	{
		irq_wake_enable = false;
		disable_irq_wake(touch_chip->irq);
	}
	spin_unlock_irqrestore(&touch_chip->touch_spinlock, irqflags);

	return 0;
}

static int mxt_wake_irq_enable(unsigned int irq)
{
    unsigned long irqflags = 0;
	    
    spin_lock_irqsave(&touch_chip->touch_spinlock, irqflags);
    if(irq_wake_enable == false)
	{
		irq_wake_enable = true;
		enable_irq_wake(touch_chip->irq);
	}	
    spin_unlock_irqrestore(&touch_chip->touch_spinlock, irqflags);
    
    return 0;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND_DISABLE
	unregister_early_suspend(&data->early_suspend);
#endif


	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
				      &data->mem_access_attr);

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	free_irq(data->irq, data);
	regulator_put(data->reg_avdd);
	regulator_put(data->reg_vdd);
	mxt_free_input_device(data);
	mxt_free_object_table(data);
	kfree(data);

	return 0;
}

static int mxt_suspend(struct device *dev)
{
	if (is_cpt_lcm == 0) {
		LDBG(" Is OBE panel , LCM ID =  [%d] , skip probe\n", is_cpt_lcm);
		return 0;
	}

	LDBG(". %d.%d.%d. %d\n",touch_chip->suspended,touch_chip->in_bootloader,touch_chip->updating_config,hall_trigger_suspend);

	if (touch_chip->update_result == 1 || touch_chip->in_bootloader == 1){
		LDBG("firmware update fail %d\n",touch_chip->update_result);
		return 0;
	}	
	
	if(touch_ic_status == true){
		struct i2c_client *client = to_i2c_client(dev);
		struct mxt_data *data = i2c_get_clientdata(client);
		struct input_dev *input_dev = data->input_dev;

		mutex_lock(&input_dev->mutex);

		if (input_dev->users)
			mxt_stop(data);
		
		mutex_unlock(&input_dev->mutex);
		
	}
		
	return 0;
}

static int mxt_resume(struct device *dev)
{
	if (is_cpt_lcm == 0) {
		LDBG(" Is OBE panel , LCM ID =  [%d] , skip probe\n", is_cpt_lcm);
		return 0;
	}

	LDBG(". %d.%d.%d.%d\n",touch_chip->suspended,touch_chip->in_bootloader,touch_chip->updating_config,hall_trigger_suspend);
	
	if (touch_chip->update_result == 1 || touch_chip->in_bootloader == 1){
		LDBG("firmware update fail %d\n",touch_chip->update_result);
		return 0;
	}	
	
	if(touch_ic_status == true){
		
		struct i2c_client *client = to_i2c_client(dev);
		struct mxt_data *data = i2c_get_clientdata(client);
		struct input_dev *input_dev = data->input_dev;
		
		
		mdelay(50);
		
		mutex_lock(&input_dev->mutex);

		if (input_dev->users)
			mxt_start(data);
		
		mutex_unlock(&input_dev->mutex);
	}
	
	return 0;
}

/* defined but not used
static int mxt_suspend_disable(struct device *dev)
{
	LDBG("[Atme] %s.\n");

	
	return 0;
}
static int mxt_resume_disable(struct device *dev)
{
	LDBG(". \n");
	
	return 0;
}
*/
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static int atmel_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    
    if (event == FB_EVENT_BLANK){
        blank = evdata->data;
	if (*blank == FB_BLANK_UNBLANK) { 
		LDBG("Screen On\n");
		mxt_resume(&touch_chip->client->dev);
	} else if (*blank == FB_BLANK_POWERDOWN) {
		LDBG("Screen Off\n");
		mxt_suspend(&touch_chip->client->dev);
	}
    }
    return 0;
}

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND_DISABLE
static void mxt_early_suspend(struct early_suspend *es)
{
	struct mxt_data *mxt;
	mxt = container_of(es, struct mxt_data, early_suspend);

	if (mxt_suspend(&mxt->client->dev) != 0)
		LDBG(" failed\n");
}

static void mxt_late_resume(struct early_suspend *es)
{
	struct mxt_data *mxt;
	LDBG(".\n");
	mxt = container_of(es, struct mxt_data, early_suspend);

	if (mxt_resume(&mxt->client->dev) != 0)
		LDBG(" failed\n");
}
#endif



// Mark by Tom for use CONFIG_PM_SCREEN_STATE_NOTIFIER
// static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);

#ifdef CONFIG_OF
static const struct of_device_id mxt_of_match[] = {
	{ .compatible = "atmel,maxtouch", },
	{},
};
MODULE_DEVICE_TABLE(of, mxt_of_match);
#else
#define mxt_of_match NULL
#endif

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
		.of_match_table = mxt_of_match,
		// Mark by Tom for use CONFIG_PM_SCREEN_STATE_NOTIFIER
		// .pm	= &mxt_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{

	LDBG("  Init Function \n");

	/* Add by Tom Cheng to Check CPT panel */
        is_cpt_lcm = gpio_get_value(958);
	LDBG(" LCM ID GPIO [%d] = [%d] \n", 958 ,is_cpt_lcm);
	if (is_cpt_lcm == 0) {
		LDBG(" Is OBE panel , LCM ID =  [%d] , skip probe\n", is_cpt_lcm);
		return 0;
	}

	/* Check is entry_mode */
	if(entry_mode != 1) {
		LDBG(" Not in MOS, skip probe. No in entry_mode:%d\n",entry_mode);
		return 0;
	}
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
