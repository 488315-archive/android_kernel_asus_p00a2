#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/battery_meter.h>
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include "bq25896.h"
#include <mt-plat/mt_gpio.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mt-plat/diso.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#ifdef MTK_DISCRETE_SWITCH
#include <mt-plat/mt_gpio.h>
#endif
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#endif
#endif

/* ============================================================*/
/*define*/
/* ============================================================*/
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


/*============================================================*/
/*global variable*/
/*============================================================*/
/*#if !defined(GPIO_CHR_CE_PIN)
#ifdef GPIO_SWCHARGER_EN_PIN
#define GPIO_CHR_CE_PIN GPIO_SWCHARGER_EN_PIN
#else
#define  GPIO_CHR_CE_PIN (19 | 0x80000000)
#endif
#endif*/
kal_bool charging_type_det_done = KAL_TRUE;

const unsigned int VBAT_CV_VTH[] = {
	3504000,    3520000,    3536000,    3552000,
	3568000,    3584000,    3600000,    3616000,
	3632000,    3648000,    3664000,    3680000,
	3696000,    3712000,	3728000,    3744000,
	3760000,    3776000,    3792000,    3808000,
	3824000,    3840000,    3856000,    3872000,
	3888000,    3904000,    3920000,    3936000,
	3952000,    3968000,    3984000,    4000000,
	4016000,    4032000,    4048000,    4064000,
	4080000,    4096000,    4112000,    4128000,
	4144000,    4160000,    4176000,    4192000,
	4208000,    4224000,    4240000,    4256000,
	4272000,    4288000,    4304000,    4320000,
	4336000,    4352000,    4368000
};

const unsigned int CS_VTH[] = {
	51200,  57600,  64000,  70400,
	76800,  83200,  89600,  96000,
	102400, 108800, 115200, 121600,
	128000, 134400, 140800, 147200,
	153600, 160000, 166400, 172800,
	179200, 185600, 192000, 198400,
	204800, 211200, 217600, 224000
};

const unsigned int INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA,  CHARGE_CURRENT_150_00_MA,	CHARGE_CURRENT_500_00_MA,
	CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1500_00_MA,  CHARGE_CURRENT_2000_00_MA,
	CHARGE_CURRENT_MAX
};

const unsigned int VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,
	BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,
	BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,
	BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,
	BATTERY_VOLT_10_500000_V
};

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#define SW_POLLING_PERIOD 100 /*100 ms*/
#define MSEC_TO_NSEC(x)		(x * 1000000UL)

static DEFINE_MUTEX(diso_polling_mutex);
static DECLARE_WAIT_QUEUE_HEAD(diso_polling_thread_wq);
static struct hrtimer diso_kthread_timer;
static kal_bool diso_thread_timeout = KAL_FALSE;
static struct delayed_work diso_polling_work;
static void diso_polling_handler(struct work_struct *work);
static DISO_Polling_Data DISO_Polling;
#else
DISO_IRQ_Data DISO_IRQ;
#endif
int g_diso_state	= 0;

static char *DISO_state_s[8] = {
		  "IDLE",
		  "OTG_ONLY",
		  "USB_ONLY",
		  "USB_WITH_OTG",
		  "DC_ONLY",
		  "DC_WITH_OTG",
		  "DC_WITH_USB",
		  "DC_USB_OTG",
};
#endif


/*============================================================*/
/*function prototype*/
/*============================================================*/
static inline int get_pack_bat_rsoc(int *rsoc);

/*============================================================*/
/*extern variable*/
/*============================================================*/
/*============================================================*/
/*extern function*/
/*============================================================*/
static unsigned int charging_error;
static unsigned int charging_get_error_state(void);
static unsigned int charging_set_error_state(void *data);
extern bool IS_STAND_AC(void);
extern bool _IS_CA81_(void);
extern bool _IS_CB81_(void);
#if defined(CONFIG_Z380M)
extern int smb3xxc_pre_config(bool initial, bool cover_changed_cable_changed);
extern bool disable_cover_otg(void);
extern void upi_ug31xx_alarm_callback_for_asus(void);
extern bool COVER_ATTACHED_UPI(void);
extern int  smb3xxc_jeita_control_for_sw_gauge(int usb_state, bool exec_jeita);
extern bool VBUS_IN(void);
extern int g_Flag2;
#elif defined(CONFIG_Z300M)
extern void charging_pad_or_standac(void);
#endif
/*============================================================*/
static inline struct power_supply *get_psy_pack_bat(void)
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

static inline int get_pack_bat_rsoc(int *rsoc)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_pack_bat();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}


unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
const unsigned int val)
{
	unsigned int temp_param;

	if (val < array_size) {
		temp_param = parameter[val];
	} else {
		battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
		temp_param = parameter[0];
	}

	return temp_param;
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
const unsigned int val)
{
	unsigned int i;

	pr_debug("array_size = %d\r\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_debug("NO register value match. val=%d\r\n", val);
	/*TODO: ASSERT(0);*/
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number, unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		/*max value in the last element*/
		for (i = (number-1); i != 0; i--) {
			if (pList[i] <= level)
				return pList[i];
		}

		pr_debug("Can't find closest level, small value first \r\n");
		return pList[0];
		/*return CHARGE_CURRENT_0_00_MA;*/
	} else {
		/*max value in the first element*/
		for (i = 0; i < number; i++) {
			if (pList[i] <= level)
				return pList[i];
		}

		pr_debug("Can't find closest level, large value first\r\n");
		return pList[number - 1];
		/*return CHARGE_CURRENT_0_00_MA;*/
	}
}

static unsigned int charging_hw_init(void *data)
{
	unsigned int status = STATUS_OK;

#ifdef CONFIG_Z380M
	if ((BMT_status.old_charging_mode == SDP) ||
			(BMT_status.old_charging_mode == CDP) ||
			(BMT_status.old_charging_mode == AC) ||
			(BMT_status.old_charging_mode == COVER) ||	
			(BMT_status.old_charging_mode == OTG) ||
			(BMT_status.old_charging_mode == CINIT)){
		BMT_status.old_charging_mode = PAD;
		Set_VGP2_PMU(0);
		bq25896_set_watchdog(0x3);	// 1. watch dog timer = 160 sec
		bq25896_set_en_ilim(0x1);	// 2. enable current limit
		bq25896_set_iinlim(0x8);	// 3. set input current limit = 500 mA
		bq25896_set_auto_dpmp_en(0x0);	// 4. disable auto_dpdm
		bq25896_set_en_hiz(0x0);	// 8. Disable PAD Charger HIZ mode
		charger_set_gpio(0,0,1,0);
	}
#endif
#ifdef CONFIG_Z300M
	charging_pad_or_standac();
#endif	
	return status;
}

static unsigned int charging_dump_register(void *data)
{
	unsigned int status = STATUS_OK;

	pr_debug("charging_dump_register\r\n");

	bq25896_dump_register();

	return status;
}

static unsigned int charging_enable(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int *)(data);

	if (KAL_TRUE == enable) {
		bq25896_set_en_hiz(0x0);
		bq25896_set_chg_config(0x1); /*charger enable*/
		battery_log(BAT_LOG_FULL, "[charging_enable] bq25896_set_en_hiz(0x0)\n");
	} else {
	#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if (mt_usb_is_device())
	#endif
			bq25896_set_chg_config(0x0);
		if (charging_get_error_state()) {
			battery_log(BAT_LOG_FULL, "[charging_enable] bq25896_set_en_hiz(0x1)\n");
			bq25896_set_en_hiz(0x1);	/* disable power path */
		}
	}
	return status;
}

static unsigned int charging_set_cv_voltage(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned short register_value;
	unsigned int cv_value = *(unsigned int *)(data);

	if (cv_value == BATTERY_VOLT_04_200000_V) {
	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
		/*highest of voltage will be 4.3V, because powerpath limitation*/
#if 1 /*if use BQ24296M, modify this condition to 0*/
		cv_value = 4304000;
#else
		cv_value = 4352000;
#endif
	#else
		/*use nearest value*/
		cv_value = 4208000;
	#endif
	}
	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), cv_value);
	bq25896_set_vreg(register_value);
	
	return status;
}

static unsigned int charging_get_current(void *data)
{
	unsigned int status = STATUS_OK;
	/*unsigned int array_size;*/
	/*unsigned char reg_value;*/

	unsigned char ret_val = 0;
	//unsigned char ret_force_20pct = 0;

	/*Get current level*/
	bq25896_read_interface((unsigned char)(bq25896_CON4), (&ret_val), (unsigned char)(CON4_ICHG_MASK),(unsigned char)(CON4_ICHG_SHIFT));

	/*Get Force 20% option*/
	//bq25896_read_interface(bq25896_CON2, &ret_force_20pct, CON2_FORCE_20PCT_MASK, CON2_FORCE_20PCT_SHIFT);

	/*Parsing*/
	ret_val = (ret_val*64) + 512;

	//if (ret_force_20pct == 0)
		*(unsigned int *)data = ret_val;
	//else
	//	*(unsigned int *)data = (int)(ret_val<<1)/10;
	return status;
}

static unsigned int charging_set_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	unsigned int current_value = *(unsigned int *)data;

	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size, set_chr_current);
	bq25896_set_ichg(register_value);

	return status;
}

static unsigned int charging_set_input_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, *(unsigned int *)data);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size, set_chr_current);
	
	bq25896_set_iinlim(register_value);

	return status;
}

static unsigned int charging_get_charging_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int ret_val;

	ret_val = bq25896_get_chrg_stat();

	if (ret_val == 0x3)
		*(unsigned int *)data = KAL_TRUE;
	else
		*(unsigned int *)data = KAL_FALSE;

	return status;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	unsigned int status = STATUS_OK;

	pr_debug("charging_reset_watch_dog_timer\r\n");
	battery_log(BAT_LOG_FULL, "charging_reset_watch_dog_timer\n");

	bq25896_set_wd_rst(0x1); /*Kick watchdog*/

	return status;
}

static unsigned int charging_set_hv_threshold(void *data)
{
	unsigned int status = STATUS_OK;

	unsigned int set_hv_voltage;
	unsigned int array_size;
	unsigned short register_value;
	unsigned int voltage = *(unsigned int *)(data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
	upmu_set_rg_vcdt_hv_vth(register_value);

	return status;
}

static unsigned int charging_get_hv_status(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *)(data) = upmu_get_rgs_vcdt_hv_det();

	return status;
}


static unsigned int charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;

	upmu_set_baton_tdet_en(1);
	upmu_set_rg_baton_en(1);
	*(kal_bool *)(data) = upmu_get_rgs_baton_undet();

	return status;
}

static unsigned int charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *)(data) = upmu_get_rgs_chrdet();

	return status;
}

kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}

static unsigned int charging_get_charger_type(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
#endif
	return status;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
	unsigned int status = STATUS_OK;

	return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	pr_debug("charging_set_platform_reset\n");
	kernel_restart("battery service reboot system");
#endif
	return status;
}

static unsigned int charging_get_platform_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;

	*(unsigned int *)(data) = get_boot_mode();

	pr_debug("get_boot_mode=%d\n", get_boot_mode());

	return status;
}

static unsigned int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

	pr_debug("charging_set_power_off\n");
	kernel_power_off();

	return status;
}

static unsigned int charging_get_power_source(void *data)
{
	unsigned int status = STATUS_OK;

#if 0	/*#if defined(MTK_POWER_EXT_DETECT)*/
	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *)data = KAL_FALSE;
	else
		*(kal_bool *)data = KAL_TRUE;
#else
	*(kal_bool *)data = KAL_FALSE;
#endif

	return status;
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static unsigned int charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;
}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
void set_vusb_auxadc_irq(bool enable, bool flag)
{
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_cancel(&diso_kthread_timer);

	DISO_Polling.reset_polling = KAL_TRUE;
	DISO_Polling.vusb_polling_measure.notify_irq_en = enable;
	DISO_Polling.vusb_polling_measure.notify_irq = flag;

	hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)), HRTIMER_MODE_REL);
#else
	unsigned short threshold = 0;

	if (enable) {
		if (flag == 0)
			threshold = DISO_IRQ.vusb_measure_channel.falling_threshold;
		else
			threshold = DISO_IRQ.vusb_measure_channel.rising_threshold;

		threshold = (threshold * R_DISO_VBUS_PULL_DOWN)/(R_DISO_VBUS_PULL_DOWN + R_DISO_VBUS_PULL_UP);
		mt_auxadc_enableBackgroundDection(DISO_IRQ.vusb_measure_channel.number, threshold,
		DISO_IRQ.vusb_measure_channel.period, DISO_IRQ.vusb_measure_channel.debounce, flag);
	} else {
		mt_auxadc_disableBackgroundDection(DISO_IRQ.vusb_measure_channel.number);
	}
#endif
	pr_debug(" [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}

void set_vdc_auxadc_irq(bool enable, bool flag)
{
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_cancel(&diso_kthread_timer);

	DISO_Polling.reset_polling = KAL_TRUE;
	DISO_Polling.vdc_polling_measure.notify_irq_en = enable;
	DISO_Polling.vdc_polling_measure.notify_irq = flag;

	hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)), HRTIMER_MODE_REL);
#else
	unsigned short threshold = 0;

	if (enable) {
		if (flag == 0)
			threshold = DISO_IRQ.vdc_measure_channel.falling_threshold;
		else
			threshold = DISO_IRQ.vdc_measure_channel.rising_threshold;

		threshold = (threshold * R_DISO_DC_PULL_DOWN)/(R_DISO_DC_PULL_DOWN + R_DISO_DC_PULL_UP);
		mt_auxadc_enableBackgroundDection(DISO_IRQ.vdc_measure_channel.number, threshold,
		DISO_IRQ.vdc_measure_channel.period, DISO_IRQ.vdc_measure_channel.debounce, flag);
	} else {
		mt_auxadc_disableBackgroundDection(DISO_IRQ.vdc_measure_channel.number);
	}
#endif
	pr_debug(" [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
static void diso_polling_handler(struct work_struct *work)
{
	int trigger_channel = -1;
	int trigger_flag = -1;

	if (DISO_Polling.vdc_polling_measure.notify_irq_en)
		trigger_channel = AP_AUXADC_DISO_VDC_CHANNEL;
	else if (DISO_Polling.vusb_polling_measure.notify_irq_en)
		trigger_channel = AP_AUXADC_DISO_VUSB_CHANNEL;

	pr_debug("[DISO]auxadc handler triggered\n");
	switch (trigger_channel) {
	case AP_AUXADC_DISO_VDC_CHANNEL:
		trigger_flag = DISO_Polling.vdc_polling_measure.notify_irq;
		pr_debug("[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
#ifdef MTK_DISCRETE_SWITCH /*for DSC DC plugin handle */
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[2]);
		}
#else /*for load switch OTG leakage handle*/
		set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[5]);
		} else if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[1]);
		} else
			pr_debug("[%s] wrong trigger flag!\n", __func__);
#endif
		break;
	case AP_AUXADC_DISO_VUSB_CHANNEL:
		trigger_flag = DISO_Polling.vusb_polling_measure.notify_irq;
		pr_debug("[DISO]VUSB IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[4]);
		} else if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[6]);
		} else
			pr_debug("[%s] wrong trigger flag!\n", __func__);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag)&0x1);
		break;
	default:
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
		return; /* in error or unexecpt state just return */
	}

	g_diso_state = *(int *)&DISO_data.diso_state;
	pr_debug("[DISO]g_diso_state: 0x%x\n", g_diso_state);
	DISO_data.irq_callback_func(0, NULL);
}
#else
static irqreturn_t diso_auxadc_irq_handler(int irq, void *dev_id)
{
	int trigger_channel = -1;
	int trigger_flag = -1;

	trigger_channel = mt_auxadc_getCurrentChannel();
	pr_debug("[DISO]auxadc handler triggered\n");
	switch (trigger_channel) {
	case AP_AUXADC_DISO_VDC_CHANNEL:
		trigger_flag = mt_auxadc_getCurrentTrigger();
		pr_debug("[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
#ifdef MTK_DISCRETE_SWITCH /*for DSC DC plugin handle */
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[2]);
		}
#else /*for load switch OTG leakage handle*/
		set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[5]);
			} else {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[1]);
		}
#endif
		break;
	case AP_AUXADC_DISO_VUSB_CHANNEL:
		trigger_flag = mt_auxadc_getCurrentTrigger();
		pr_debug("[DISO]VUSB IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[4]);
		} else {
			DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[6]);
		}
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag)&0x1);
		break;
	default:
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
		return IRQ_HANDLED; /* in error or unexecpt state just return */
	}
	g_diso_state = *(int *)&DISO_data.diso_state;
	return IRQ_WAKE_THREAD;
}
#endif

static unsigned int diso_get_current_voltage(int Channel)
{
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0, times = 5;

	if (IMM_IsAdcInitReady() == 0) {
		pr_debug("[DISO] AUXADC is not ready");
		return 0;
	}

	i = times;
	while (i--) {
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
		if (ret_value == 0) {
			ret += ret_temp;
		} else {
			times = times > 1 ? times - 1 : 1;
			pr_debug("[diso_get_current_voltage] ret_value=%d, times=%d\n",
			ret_value, times);
		}
	}

	ret = ret*1500/4096;
	ret = ret/times;

	return  ret;
}

static void _get_diso_interrupt_state(void)
{
	int vol = 0;
	int diso_state = 0;

	mdelay(AUXADC_CHANNEL_DELAY_PERIOD);

	vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
	vol = (R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN)*100*vol/(R_DISO_DC_PULL_DOWN)/100;
	pr_debug("[DISO]	Current DC voltage mV = %d\n", vol);

	/* force delay for switching as no flag for check switching done */
	mdelay(SWITCH_RISING_TIMING + LOAD_SWITCH_TIMING_MARGIN);
	if (vol > VDC_MIN_VOLTAGE/1000 && vol < VDC_MAX_VOLTAGE/1000)
		diso_state |= 0x4; /*SET DC bit as 1*/
	else
		diso_state &= ~0x4; /*SET DC bit as 0*/


	vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
	vol = (R_DISO_VBUS_PULL_UP + R_DISO_VBUS_PULL_DOWN)*100*vol/(R_DISO_VBUS_PULL_DOWN)/100;
	pr_debug("[DISO]	Current VBUS voltage  mV = %d\n", vol);

	if (vol > VBUS_MIN_VOLTAGE/1000 && vol < VBUS_MAX_VOLTAGE/1000) {
		if (!mt_usb_is_device()) {
			diso_state |= 0x1; /*SET OTG bit as 1*/
			diso_state &= ~0x2; /*SET VBUS bit as 0*/
		} else {
			diso_state &= ~0x1; /*SET OTG bit as 0*/
			diso_state |= 0x2; /*SET VBUS bit as 1;*/
		}

	} else {
		diso_state &= 0x4; /*SET OTG and VBUS bit as 0*/
	}
	pr_debug("[DISO] DISO_STATE==0x%x\n", diso_state);
	g_diso_state = diso_state;
}
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
int _get_irq_direction(int pre_vol, int cur_vol)
{
	int ret = -1;

	/*threshold 1000mv*/
	if ((cur_vol - pre_vol) > 1000)
		ret = DISO_IRQ_RISING;
	else if ((pre_vol - cur_vol) > 1000)
		ret = DISO_IRQ_FALLING;

	return ret;
}

static void _get_polling_state(void)
{
	int vdc_vol = 0, vusb_vol = 0;
	int vdc_vol_dir = -1;
	int vusb_vol_dir = -1;

	DISO_polling_channel *VDC_Polling = &DISO_Polling.vdc_polling_measure;
	DISO_polling_channel *VUSB_Polling = &DISO_Polling.vusb_polling_measure;

	vdc_vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
	vdc_vol = (R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN)*100*vdc_vol/(R_DISO_DC_PULL_DOWN)/100;

	vusb_vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
	vusb_vol = (R_DISO_VBUS_PULL_UP + R_DISO_VBUS_PULL_DOWN)*100*vusb_vol/(R_DISO_VBUS_PULL_DOWN)/100;

	VDC_Polling->preVoltage = VDC_Polling->curVoltage;
	VUSB_Polling->preVoltage = VUSB_Polling->curVoltage;
	VDC_Polling->curVoltage = vdc_vol;
	VUSB_Polling->curVoltage = vusb_vol;

	if (DISO_Polling.reset_polling) {
		DISO_Polling.reset_polling = KAL_FALSE;
		VDC_Polling->preVoltage = vdc_vol;
		VUSB_Polling->preVoltage = vusb_vol;

		if (vdc_vol > 1000)
			vdc_vol_dir = DISO_IRQ_RISING;
		else
			vdc_vol_dir = DISO_IRQ_FALLING;

		if (vusb_vol > 1000)
			vusb_vol_dir = DISO_IRQ_RISING;
		else
			vusb_vol_dir = DISO_IRQ_FALLING;
	} else {
		/*get voltage direction*/
		vdc_vol_dir = _get_irq_direction(VDC_Polling->preVoltage, VDC_Polling->curVoltage);
		vusb_vol_dir = _get_irq_direction(VUSB_Polling->preVoltage, VUSB_Polling->curVoltage);
	}

	if (VDC_Polling->notify_irq_en &&
	(vdc_vol_dir == VDC_Polling->notify_irq)) {
		schedule_delayed_work(&diso_polling_work, 10*HZ/1000); /*10ms*/
		pr_debug("[%s] ready to trig VDC irq, irq: %d\n",
		__func__, VDC_Polling->notify_irq);
	} else if (VUSB_Polling->notify_irq_en && (vusb_vol_dir == VUSB_Polling->notify_irq)) {
		schedule_delayed_work(&diso_polling_work, 10*HZ/1000);
		pr_debug("[%s] ready to trig VUSB irq, irq: %d\n",
		__func__, VUSB_Polling->notify_irq);
	} else if ((vdc_vol == 0) && (vusb_vol == 0)) {
		VDC_Polling->notify_irq_en = 0;
		VUSB_Polling->notify_irq_en = 0;
	}

}

enum hrtimer_restart diso_kthread_hrtimer_func(struct hrtimer *timer)
{
	diso_thread_timeout = KAL_TRUE;
	wake_up(&diso_polling_thread_wq);

	return HRTIMER_NORESTART;
}

int diso_thread_kthread(void *x)
{
	/* Run on a process content */
	while (1) {
		wait_event(diso_polling_thread_wq, (diso_thread_timeout == KAL_TRUE));

		diso_thread_timeout = KAL_FALSE;

		mutex_lock(&diso_polling_mutex);

		_get_polling_state();

		if (DISO_Polling.vdc_polling_measure.notify_irq_en || DISO_Polling.vusb_polling_measure.notify_irq_en)
			hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)),
			HRTIMER_MODE_REL);
		else
			hrtimer_cancel(&diso_kthread_timer);

		mutex_unlock(&diso_polling_mutex);
	}

	return 0;
}
#endif
#endif


static unsigned int charging_get_error_state(void)
{
	return charging_error;
}

static unsigned int charging_set_error_state(void *data)
{
	unsigned int status = STATUS_OK;

	charging_error = *(unsigned int *)(data);

	return status;
}

static unsigned int charging_diso_init(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

	/* Initialization DISO Struct */
	pDISO_data->diso_state.cur_otg_state	 = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vdc_state	 = DISO_OFFLINE;

	pDISO_data->diso_state.pre_otg_state	 = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vdc_state	 = DISO_OFFLINE;

	pDISO_data->chr_get_diso_state = KAL_FALSE;
	pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_init(&diso_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	diso_kthread_timer.function = diso_kthread_hrtimer_func;
	INIT_DELAYED_WORK(&diso_polling_work, diso_polling_handler);

	kthread_run(diso_thread_kthread, NULL, "diso_thread_kthread");
	pr_debug("[%s] done\n", __func__);
#else
	struct device_node *node;
	int ret;

	/*Initial AuxADC IRQ*/
	DISO_IRQ.vdc_measure_channel.number   = AP_AUXADC_DISO_VDC_CHANNEL;
	DISO_IRQ.vusb_measure_channel.number  = AP_AUXADC_DISO_VUSB_CHANNEL;
	DISO_IRQ.vdc_measure_channel.period   = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vusb_measure_channel.period  = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vdc_measure_channel.debounce	  = AUXADC_CHANNEL_DEBOUNCE;
	DISO_IRQ.vusb_measure_channel.debounce  = AUXADC_CHANNEL_DEBOUNCE;

	/* use default threshold voltage, if use high voltage,maybe refine*/
	DISO_IRQ.vusb_measure_channel.falling_threshold = VBUS_MIN_VOLTAGE/1000;
	DISO_IRQ.vdc_measure_channel.falling_threshold = VDC_MIN_VOLTAGE/1000;
	DISO_IRQ.vusb_measure_channel.rising_threshold = VBUS_MIN_VOLTAGE/1000;
	DISO_IRQ.vdc_measure_channel.rising_threshold = VDC_MIN_VOLTAGE/1000;

	node = of_find_compatible_node(NULL, NULL, "mediatek,AUXADC");
	if (!node) {
		pr_debug("[diso_adc]: of_find_compatible_node failed!!\n");
	} else {
		pDISO_data->irq_line_number = irq_of_parse_and_map(node, 0);
		pr_debug("[diso_adc]: IRQ Number: 0x%x\n", pDISO_data->irq_line_number);
	}

	mt_irq_set_sens(pDISO_data->irq_line_number, MT_EDGE_SENSITIVE);
	mt_irq_set_polarity(pDISO_data->irq_line_number, MT_POLARITY_LOW);

	ret = request_threaded_irq(pDISO_data->irq_line_number, diso_auxadc_irq_handler,
	pDISO_data->irq_callback_func, IRQF_ONESHOT, "DISO_ADC_IRQ", NULL);

	if (ret) {
		pr_debug("[diso_adc]: request_irq failed.\n");
	} else {
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[diso_adc]: diso_init success.\n");
	}
#endif
#endif

	return status;
}

static unsigned int charging_get_diso_state(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	int diso_state = 0x0;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

	_get_diso_interrupt_state();
	diso_state = g_diso_state;
	pr_debug("[do_chrdet_int_task] current diso state is %s!\n", DISO_state_s[diso_state]);
	if (((diso_state >> 1) & 0x3) != 0x0) {
		switch (diso_state) {
		case USB_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
			set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_USB:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_OTG:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			break;
		default: /*OTG only also can trigger vcdt IRQ*/
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			pr_debug(" switch load vcdt irq triggerd by OTG Boost!\n");
			break; /*OTG plugin no need battery sync action*/
		}
	}

	if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state)
		pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
	else
		pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;
#endif

	return status;
}

static unsigned int charging_set_vindpm_os(void *data)
{
	unsigned int status = STATUS_OK;

	bq25896_set_vindpm_os(0x7);
	return status;
}

static unsigned int charging_set_sys_min(void *data)
{
	unsigned int status = STATUS_OK;

	bq25896_set_sys_min(0x6);
	return status;
}

extern int g_projector_on;

static unsigned int charging_AC(void *data){
#ifdef CONFIG_Z380M
battery_log(BAT_LOG_CRTI, "Z380M_charging_AC\n");
	if ((BMT_status.old_charging_mode == PAD) ||
			(BMT_status.old_charging_mode == COVER_AC) ||
			(BMT_status.old_charging_mode == CINIT)){
		Set_VGP2_PMU(0);
		bq25896_set_watchdog(0x3); 	// 1. WatchDog Timer = 160 sec
		bq25896_set_ichg(0x1E); 	// 2. Set Fast Charge Current Limit = 1920 mA
		bq25896_set_iprechg(0x2); 	//    Set Pre-Charge Current Limit = 192 mA
		bq25896_set_iterm(0x2); 	//    Set Termination Current Limit = 192 mA
		if(g_projector_on){
			bq25896_set_iinlim(0x8);        // 3. Set Input Current Limit = 500 mA
		}else{
			bq25896_set_iinlim(0x12);       // 3. Set Input Current Limit = 1000 mA
		}
		bq25896_set_vreg(0x20); 	// 4. Set Charge Voltage Limit = 4.35 V
		bq25896_set_vindpm_os(0x7); 	// 5. Voltage DPM offset = 700 mV
		bq25896_set_sys_min(0x6); 	// 6. Minimum System Voltage = 3.6 V
		bq25896_set_en_ilim(0x0); 	// 7. Disable Current Limit
		bq25896_set_en_hiz(0x0);	// 8. Disable PAD Charger HIZ mode
		charger_set_gpio( 0, 0, 1, 0);
		BMT_status.old_charging_mode = AC;
	}
	else
		battery_log(BAT_LOG_CRTI, "there is some thing wrong, [%d] -> AC \n", BMT_status.old_charging_mode);
#endif
#ifdef CONFIG_Z300M
battery_log(BAT_LOG_CRTI, "Z300M_charging_AC\n");
	Set_VIBR_PMU(0);
        bq25896_set_watchdog(0x3); 	// 1. WatchDog Timer = 160 sec
        bq25896_set_ichg(0x22); 	// 2. Set Fast Charge Current Limit = 2176 mA
        bq25896_set_iprechg(0x5); 	//    Set Pre-Charge Current Limit = 384 mA
        bq25896_set_iterm(0x2); 	//    Set Termination Current Limit = 192 mA
        bq25896_set_iinlim(0x12); 	// 3. Set Input Current Limit = 1000 mA
        bq25896_set_vreg(0x21); 	// 4. Set Charge Voltage Limit = 4.368 V
        bq25896_set_vindpm_os(0x7); 	// 5. Voltage DPM offset = 700 mV
        bq25896_set_sys_min(0x6); 	// 6. Minimum System Voltage = 3.6 V
        bq25896_set_en_ilim(0x0); 	// 7. Disable Current Limit
        charger_set_gpio( 0, 0, 1, 0);
#endif
	return 0;
}

static unsigned int charging_SDP(void *data){
#ifdef CONFIG_Z380M
battery_log(BAT_LOG_CRTI, "Z380M_charging_SDP\n");
	if ((BMT_status.old_charging_mode == PAD) || 
			(BMT_status.old_charging_mode == COVER_SDP) ||
			(BMT_status.old_charging_mode == CINIT)){
		Set_VGP2_PMU(0);
		bq25896_set_watchdog(0x3); 	// 1. WatchDog Timer = 160 sec
		bq25896_set_ichg(0x1E); 	// 2. Set Fast Charge Current Limit = 1920 mA
		bq25896_set_iprechg(0x2); 	//    Set Pre-Charge Current Limit = 192 mA
		bq25896_set_iterm(0x2); 	//    Set Termination Current Limit = 192 mA
		bq25896_set_iinlim(0x8); 	// 3. Set Input Current Limit = 500 mA
		bq25896_set_vreg(0x20); 	// 4. Set Charge Voltage Limit = 4.35 V
		bq25896_set_vindpm_os(0x7); 	// 5. Voltage DPM offset = 700 mV
		bq25896_set_sys_min(0x6); 	// 6. Minimum System Voltage = 3.6 V
		bq25896_set_en_ilim(0x0); 	// 7. Disable Current Limit
		bq25896_set_en_hiz(0x0);	// 8. Disable PAD Charger HIZ mode
		charger_set_gpio( 0, 0, 1, 0);
		BMT_status.old_charging_mode = SDP;
	}
	else
		battery_log(BAT_LOG_CRTI, "there is some thing wrong, [%d] -> SDP \n", BMT_status.old_charging_mode);
#endif
#ifdef CONFIG_Z300M
battery_log(BAT_LOG_CRTI, "Z300M_charging_SDP\n");
	Set_VIBR_PMU(0);
        bq25896_set_watchdog(0x3); 	// 1. WatchDog Timer = 160 sec
        bq25896_set_ichg(0x22); 	// 2. Set Fast Charge Current Limit = 2176 mA
        bq25896_set_iprechg(0x5); 	//    Set Pre-Charge Current Limit = 384 mA
        bq25896_set_iterm(0x2); 	//    Set Termination Current Limit = 192 mA
        bq25896_set_iinlim(0x8); 	// 3. Set Input Current Limit = 500 mA
        bq25896_set_vreg(0x21); 	// 4. Set Charge Voltage Limit = 4.368 V
        bq25896_set_vindpm_os(0x7); 	// 5. Voltage DPM offset = 700 mV
        bq25896_set_sys_min(0x6); 	// 6. Minimum System Voltage = 3.6 V
        bq25896_set_en_ilim(0x0); 	// 7. Disable Current Limit
        charger_set_gpio( 0, 0, 1, 0);
#endif
	return 0;
}

static unsigned int charging_CDP(void *data){
#if defined(CONFIG_Z380M)
battery_log(BAT_LOG_CRTI, "Z380M_charging_CDP\n");
	if ((BMT_status.old_charging_mode == PAD) ||
			(BMT_status.old_charging_mode == COVER_CDP) ||
			(BMT_status.old_charging_mode == CINIT)){
		Set_VGP2_PMU(0);
		bq25896_set_watchdog(0x3); 	// 1. WatchDog Timer = 160 sec
		bq25896_set_ichg(0x1E); 	// 2. Set Fast Charge Current Limit = 1920 mA
		bq25896_set_iprechg(0x2); 	//    Set Pre-Charge Current Limit = 192 mA
		bq25896_set_iterm(0x2); 	//    Set Termination Current Limit = 192 mA
		bq25896_set_iinlim(0x1C); 	// 3. Set Input Current Limit = 1500 mA
		bq25896_set_vreg(0x20); 	// 4. Set Charge Voltage Limit = 4.35 V
		bq25896_set_vindpm_os(0x7); 	// 5. Voltage DPM offset = 700 mV
		bq25896_set_sys_min(0x6); 	// 6. Minimum System Voltage = 3.6 V
		bq25896_set_en_ilim(0x0); 	// 7. Disable Current Limit
		bq25896_set_en_hiz(0x0);	// 8. Disable PAD Charger HIZ mod
		charger_set_gpio( 0, 0, 1, 0);
		BMT_status.old_charging_mode = CDP;
	}
	else
		battery_log(BAT_LOG_CRTI, "there is some thing wrong, [%d] -> SDP \n", BMT_status.old_charging_mode);
#elif defined(CONFIG_Z300M)
battery_log(BAT_LOG_CRTI, "Z300M_charging_CDP\n");
	Set_VIBR_PMU(0);
        bq25896_set_watchdog(0x3); 	// 1. WatchDog Timer = 160 sec
        bq25896_set_ichg(0x22); 	// 2. Set Fast Charge Current Limit = 2176 mA
        bq25896_set_iprechg(0x5); 	//    Set Pre-Charge Current Limit = 384 mA
        bq25896_set_iterm(0x2); 	//    Set Termination Current Limit = 192 mA
        bq25896_set_iinlim(0x1C); 	// 3. Set Input Current Limit = 1500 mA
        bq25896_set_vreg(0x21); 	// 4. Set Charge Voltage Limit = 4.368 V
        bq25896_set_vindpm_os(0x7); 	// 5. Voltage DPM offset = 700 mV
        bq25896_set_sys_min(0x6); 	// 6. Minimum System Voltage = 3.6 V
        bq25896_set_en_ilim(0x0); 	// 7. Disable Current Limit
        charger_set_gpio( 0, 0, 1, 0);
#endif
	return 0;
}

static unsigned int charging_COVER(void *data){
battery_log(BAT_LOG_CRTI, "charging_COVER\n");
#if defined(CONFIG_Z380M)
	if ((BMT_status.old_charging_mode == PAD) || 
		(BMT_status.old_charging_mode == COVER_SDP) ||
		(BMT_status.old_charging_mode == COVER_CDP) ||
		(BMT_status.old_charging_mode == COVER_AC) ||
		(BMT_status.old_charging_mode == COVER_OTG) ||
		(BMT_status.old_charging_mode == CINIT)){

		BMT_status.old_charging_mode = COVER;
		Set_VGP2_PMU(0);
		if(_IS_CA81_()){
			charger_set_gpio( 0, 0, 0, 0);
		}	
		else if(_IS_CB81_()){
			msleep_interruptible(1000);
        		bq25896_set_watchdog(0x3); 	// 1. WatchDog Timer = 160 sec
        		bq25896_set_ichg(0x1E); 	// 2. Set Fast Charge Current Limit = 1920 mA
	        	bq25896_set_iprechg(0x2); 	//    Set Pre-Charge Current Limit = 192 mA
        		bq25896_set_iterm(0x2); 	//    Set Termination Current Limit = 192 mA
        		bq25896_set_vreg(0x20); 	// 4. Set Charge Voltage Limit = 4.35 V
        		bq25896_set_vindpm_os(0x7); 	// 5. Voltage DPM offset = 700 mV
	        	bq25896_set_sys_min(0x6); 	// 6. Minimum System Voltage = 3.6 V
        		bq25896_set_en_ilim(0x0); 	// 7. Disable Current Limit
		
/*cover charger setting */
			smb3xxc_pre_config(true, true);
			charger_set_gpio( 0, 0, 0, 0);
		}
	}
	else
		battery_log(BAT_LOG_CRTI, "there is some thing wrong, [%d] -> COVER \n", BMT_status.old_charging_mode);
	
#endif
        return 0;
}

static unsigned int charging_COVER_AC(void *data){
battery_log(BAT_LOG_CRTI, "COVER_AC\n");
	if ((BMT_status.old_charging_mode == COVER)||
			(BMT_status.old_charging_mode == AC) ||
			(BMT_status.old_charging_mode == CINIT) ||
			(BMT_status.old_charging_mode == COVER_AC)){  
		BMT_status.old_charging_mode = COVER_AC;
#if defined(CONFIG_Z380M)
		Set_VGP2_PMU(0);
	        bq25896_set_watchdog(0x3); // 1. WatchDog Timer = 160 sec
	        bq25896_set_ichg(0x1E); // 2. Set Fast Charge Current Limit = 1920 mA
	        bq25896_set_iprechg(0x2); // Set Pre-Charge Current Limit = 192 mA
	        bq25896_set_iterm(0x2); // Set Termination Current Limit = 192 mA
	        bq25896_set_iinlim(0x12); // 3. Set Input Current Limit = 1000 mA
	        bq25896_set_vreg(0x20); // 4. Set Charge Voltage Limit = 4.35 V
	        bq25896_set_vindpm_os(0x7); // 5. Voltage DPM offset = 700 mV
	        bq25896_set_sys_min(0x6); // 6. Minimum System Voltage = 3.6 V
	        bq25896_set_en_ilim(0x0); // 7. Disable Current Limit
		bq25896_set_en_hiz(0x0);	// 8. Disable PAD Charger HIZ mode

/* cover charger setting */
		smb3xxc_pre_config(true, true);
/* disable cover otg */
		disable_cover_otg();
#endif
        	charger_set_gpio( 0, 1, 1, 0);
	}
	else
		battery_log(BAT_LOG_CRTI, "there is some thing wrong, [%d] -> COVER_AC \n", BMT_status.old_charging_mode);

        return 0;
}

static unsigned int charging_COVER_SDP(void *data){
battery_log(BAT_LOG_CRTI, "COVER_SDP\n");
	if ((BMT_status.old_charging_mode == COVER) ||
			(BMT_status.old_charging_mode == SDP) ||
			(BMT_status.old_charging_mode == CINIT)){
		BMT_status.old_charging_mode = COVER_SDP;
#if defined(CONFIG_Z380M)
		Set_VGP2_PMU(0);
	        bq25896_set_watchdog(0x3); // 1. WatchDog Timer = 160 sec
	        bq25896_set_ichg(0x1E); // 2. Set Fast Charge Current Limit = 1920 mA
	        bq25896_set_iprechg(0x2); // Set Pre-Charge Current Limit = 192 mA
	        bq25896_set_iterm(0x2); // Set Termination Current Limit = 192 mA
	        bq25896_set_iinlim(0x8); // 3. Set Input Current Limit = 500 mA
	        bq25896_set_vreg(0x20); // 4. Set Charge Voltage Limit = 4.35 V
	        bq25896_set_vindpm_os(0x7); // 5. Voltage DPM offset = 700 mV
	        bq25896_set_sys_min(0x6); // 6. Minimum System Voltage = 3.6 V
	        bq25896_set_en_ilim(0x0); // 7. Disable Current Limit
	
/* cover charger setting */
		smb3xxc_pre_config(true, true);
/* disable cover otg */
		disable_cover_otg();
#endif
	        charger_set_gpio( 0, 1, 1, 0);
	}	
	else
		battery_log(BAT_LOG_CRTI, "there is some thing wrong, [%d] -> COVER_SDP \n", BMT_status.old_charging_mode);
/* read cover gauge */
//	bq25896_dump_register();
        return 0;
}

static unsigned int charging_COVER_CDP(void *data){
battery_log(BAT_LOG_CRTI, "COVER_CDP\n");
	if ((BMT_status.old_charging_mode == COVER) ||
			(BMT_status.old_charging_mode == CDP) ||
			(BMT_status.old_charging_mode == CINIT)){
		BMT_status.old_charging_mode = COVER_CDP;
#if defined(CONFIG_Z380M)
		Set_VGP2_PMU(0);
	        bq25896_set_watchdog(0x3); // 1. WatchDog Timer = 160 sec
	        bq25896_set_ichg(0x1E); // 2. Set Fast Charge Current Limit = 1920 mA
	        bq25896_set_iprechg(0x2); // Set Pre-Charge Current Limit = 192 mA
	        bq25896_set_iterm(0x2); // Set Termination Current Limit = 192 mA
	        bq25896_set_iinlim(0x1C); // 3. Set Input Current Limit = 1500 mA
	        bq25896_set_vreg(0x20); // 4. Set Charge Voltage Limit = 4.35 V
	        bq25896_set_vindpm_os(0x7); // 5. Voltage DPM offset = 700 mV
	        bq25896_set_sys_min(0x6); // 6. Minimum System Voltage = 3.6 V
	        bq25896_set_en_ilim(0x0); // 7. Disable Current Limit

/* cover charger setting */
		smb3xxc_pre_config(true, true);
/* disable cover otg */
		disable_cover_otg();
#endif
	        charger_set_gpio( 0, 1, 1, 0);
	}
	else
		battery_log(BAT_LOG_CRTI, "there is some thing wrong, [%d] -> COVER_CDP \n", BMT_status.old_charging_mode);
	return 0;
}

static unsigned int charging_COVER_OTG(void *data){
battery_log(BAT_LOG_CRTI, "COVER_OTG\n");

/*cover charger setting */
//disable otg
//need to modify
	BMT_status.old_charging_mode = COVER_OTG;
        charger_set_gpio( 0, 0, 1, 0);
        return 0;
}
static unsigned int charging_COVER_DISABLEOTG(void *data){
#if defined(CONFIG_Z380M)
	disable_cover_otg();
#endif
	return 0;
}
static unsigned int charging_JEITA(void *data){
	int temp;
	unsigned int bq25896_temp;
	static int bq25896_recharging = 0;
	temp = battery_meter_get_battery_temperature();
	battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] charging_JEITA sense BAT_temp = %d!\n",temp);
	charging_reset_watch_dog_timer(NULL);	
	bq25896_set_conv_rate(0x1);//Start 1s Continuous Conversion
	bq25896_set_jeita_iset(0x0);//JEITA Low Temperature Current Setting
	bq25896_temp = bq25896_get_tspct();
	battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] bq25896_get_tspct = %u\n",bq25896_temp);
#ifdef CONFIG_Z380M
battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] charging_JEITA Start!\n");
	/*Judge Temperature, then set charge boltage limit*/
//battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] BMT_status.temperature = %d\n",BMT_status.temperature);
	if (bq25896_recharging==0){
		if (temp >= 55 || temp <= 0){
			bq25896_set_chg_config(0x0);// Disable charging
			bq25896_set_vreg(0x20);//Set Charge Voltage Limit = 4.35 V
			bq25896_recharging = 1;
		}else{
			if(bq25896_temp <= 51){//Temp>=45 ( 51=0x0110011 )
				bq25896_set_chg_config(0x1);// Enable charging
				bq25896_set_vreg(0x1D);//Set Charge Voltage Limit = 4.3 V
			}else{//Temp<45
				bq25896_set_chg_config(0x1);// Enable charging
				bq25896_set_vreg(0x20);//Set Charge Voltage Limit = 4.35 V
			}
		}
	}else{
		if(3 < temp && temp < 52){
			bq25896_recharging = 0;
			if(bq25896_temp <= 51){//Temp>=45 ( 51=0x0110011 )
				bq25896_set_chg_config(0x1);// Enable charging
				bq25896_set_vreg(0x1D);//Set Charge Voltage Limit = 4.3 V
			}else{//Temp<45
				bq25896_set_chg_config(0x1);// Enable charging
				bq25896_set_vreg(0x20);//Set Charge Voltage Limit = 4.35 V
			}
		}
	}
	if (COVER_ATTACHED_UPI()){
		if((BMT_status.charger_type == STANDARD_CHARGER)||
			(BMT_status.charger_type == NONSTANDARD_CHARGER)||
			(BMT_status.charger_type == STANDARD_HOST && g_Flag2==0)||
			(BMT_status.charger_type == CHARGING_HOST && g_Flag2==0)){
			smb3xxc_jeita_control_for_sw_gauge(0, true);
		}
	}
#endif
#ifdef CONFIG_Z300M
	//check USB_TYPE=DCP_2A, wait for US5578
	battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] 5W JEITA Start\n");
	if(bq25896_recharging == 0){
		if (temp >= 55 || temp <= 0){
			bq25896_set_chg_config(0x0);// Disable charging
			bq25896_set_vreg(0x21);//Set Charge Voltage Limit = 4.368 V
			bq25896_recharging = 1;
		}else{
			if(bq25896_temp <= 51){//Temp>=45 ( 51=0x0110011 )
				bq25896_set_chg_config(0x1);// Enable charging
				bq25896_set_vreg(0x1D);//Set Charge Voltage Limit = 4.3 V
			}else{//Temp<45
				bq25896_set_chg_config(0x1);// Enable charging
				bq25896_set_vreg(0x21);//Set Charge Voltage Limit = 4.368 V
			}
		}
	}else{
		if (3 < temp && temp < 52){
			bq25896_recharging = 0;
			if(bq25896_temp <= 51){//Temp>=45 ( 51=0x0110011 )
				bq25896_set_chg_config(0x1);// Enable charging
				bq25896_set_vreg(0x1D);//Set Charge Voltage Limit = 4.3 V
			}else{//Temp<45
				bq25896_set_chg_config(0x1);// Enable charging
				bq25896_set_vreg(0x21);//Set Charge Voltage Limit = 4.368 V
			}
		}
	}
#endif
	battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] JEITA End\n");
        return 0;
}

static unsigned int charging_get_VREG(void *data){
	int vreg = 0;
	int value;
	int i;
	int mV = 16;
	value = bq25896_get_vreg();
	for( i=0; i<6; i++){
		if ((value & BIT(i))==BIT(i))
			vreg += mV;
			mV *= 2;
	}
	vreg = vreg + 3840;
	*(unsigned int *)(data) = vreg;
	battery_log(BAT_LOG_CRTI, "[%s] Charge Voltage Limit =%d\n", __func__,vreg);
	return 0;
}

static unsigned int charging_get_bq25896_ICharging(void *data){
        int current1 = 0;
        int value ;
        int i;
        int mA = 50;
        value = bq25896_get_ichgr();
        for(i=0;i<7;i++)
        {
                if ((value & BIT(i))==BIT(i))
                        current1 += mA;
                mA *= 2;
        }
        battery_log(BAT_LOG_CRTI, "[show_ICHGR] current =%d\n", current1);
	*(unsigned int *)(data) = current1;
        return 0;
}

#ifdef CONFIG_Z300M
bool b_charging_type = true;

static void ASUS_Adapter_Charging(void){
        charger_set_ADC_SW_EN_gpio(1);
//Need to add process
}

static void f_check(void){
int count=0;
        while((BMT_status.SOC > 3) && (b_charging_type==false)){
                ASUS_Adapter_Charging();
                if (count==3){//avoid always in this condition
                        break;
                }
                count++;
        }
}

static unsigned int charging_ADC_SW_EN(void *data){
	battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] charging_ADC_SW_EN Start!\n");
	if(BMT_status.charger_type == STANDARD_HOST){//SDP
		bq25896_set_iinlim(0x8);//Set Input Current Limit = 500 mA
		battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] charging type = SDP, Set ICL = 500mA \n");
	}else if(BMT_status.charger_type == CHARGING_HOST){//CDP
		bq25896_set_iinlim(0x1C);//Set Input Current Limit = 1500 mA
		battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] charging type = CDP, Set ICL = 1500mA \n");
	}else if((BMT_status.charger_type==NONSTANDARD_CHARGER)||(BMT_status.charger_type == STANDARD_CHARGER)){//DCP
		if(BMT_status.SOC > 3){
			battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] charging type = ASUS Adapter Charging \n");
			ASUS_Adapter_Charging();
		}else{
			battery_log(BAT_LOG_CRTI, "[charging_hw_bq25896] charging type = Undefined \n");
			b_charging_type = false;//Undefined
		}
	}
	f_check();
	b_charging_type = true;
	charger_set_ADC_SW_EN_gpio(0);
	return 0;
}
#endif
static unsigned int (* const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
	charging_hw_init,
	charging_dump_register,
	charging_enable,
	charging_set_cv_voltage,
	charging_get_current,
	charging_set_current,
	charging_set_input_current,
	charging_get_charging_status,
	charging_reset_watch_dog_timer,
	charging_set_hv_threshold,
	charging_get_hv_status,
	charging_get_battery_status,
	charging_get_charger_det_status,
	charging_get_charger_type,
	charging_get_is_pcm_timer_trigger,
	charging_set_platform_reset,
	charging_get_platform_boot_mode,
	charging_set_power_off,
	charging_get_power_source,
	charging_get_csdac_full_flag,
	charging_set_ta_current_pattern,
	charging_set_error_state,
	charging_diso_init,
	charging_get_diso_state,
	charging_set_vindpm_os,
	charging_set_sys_min,
	charging_AC,
	charging_SDP,
	charging_CDP,
	charging_COVER,
	charging_COVER_AC,
	charging_COVER_SDP,
	charging_COVER_CDP,
	charging_COVER_OTG,
	charging_COVER_DISABLEOTG,
	charging_JEITA,
	charging_get_VREG,
#ifdef CONFIG_Z300M
	charging_ADC_SW_EN,
#endif
	charging_get_bq25896_ICharging,
};

 /*
 *FUNCTION
 *		Internal_chr_control_handler
 *
 *DESCRI
 *		 This function is called to set the charger hw
 *
 *CALLS
 *
 * PARAMETERS
 *		None
 *
 *RETURNS
 *
 * GLOBALS AFFECTED
 *	   None
 */
signed int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	signed int status;

	if (cmd < CHARGING_CMD_NUMBER)
		status = charging_func[cmd](data);
	else
		return STATUS_UNSUPPORTED;

	return status;
}
