#include <linux/delay.h>
#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>
#include "bq25896.h"
#include <mt-plat/battery_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

//shilun_huang@asus.com
#if defined(CONFIG_Z380M)
//static struct workqueue_struct *cvbus_wq;
//int g_cvbus_in_int = 0;
//EXPORT_SYMBOL_GPL(g_cvbus_in_int);
static struct workqueue_struct *otg_wq;
#elif defined(CONFIG_Z300M)
static struct workqueue_struct *stand_wq;
static struct workqueue_struct *cvbus_wq;
int g_cvbus_in_int = 0;
int g_stand_int = 0;
EXPORT_SYMBOL(g_stand_int);
#endif
struct charger_gpio{
	struct device	*dev;
	unsigned int 	PG_OTG_EN_SOC;
	unsigned int 	Cover_BUS_on;
	unsigned int 	USB_BUS_on_NUM;
	unsigned int 	Cover_BUS_off;
#if defined(CONFIG_Z380M)
	unsigned int 	I2C_POWER_EN; // add by leo
#endif
	struct 		delayed_work cvbus_work;
	int 		cvbus_irq;
#if defined(CONFIG_Z380M)
	struct		delayed_work otg_cb81_work;
#elif defined(CONFIG_Z300M)
	unsigned int 	DCIN_VBUS_IN_DET_N;	//stand ac in:0
	unsigned int 	N1_VBUS_IN_DET_NUM;	//usb det  in:0
	unsigned int 	N1_FUNCTION_NUN;	//stand det in:0
	struct 		delayed_work stand_work;
	struct 		delayed_work stand_timer_work;
	struct 		delayed_work otg_hub_work;
	struct 		delayed_work usb_work;
	int 		stand_irq;
	int 		stand_ac_irq;
	int 		usb_irq;
	//Michael_Nieh@asus.com
	unsigned int 	ADC_SW_EN;
	unsigned int 	PAD_QB_DET;
	//static unsigned int N1_VBUS_IN_DET;
	//static unsigned int DCIN_VBUS_IN_DET_N;
#endif
};
struct charger_gpio *charger_gpio;
#define DEBUG_REG 1 
/**********************************************************
 *
 *	[I2C Slave Setting]
 *
 *********************************************************/
#define bq25896_SLAVE_ADDR_WRITE   0xD6
#define bq25896_SLAVE_ADDR_READ    0xD7

#ifdef CONFIG_OF
static const struct of_device_id bq25896_id[] = {
		{ .compatible = "ti,bq25896" },
		{},
};
MODULE_DEVICE_TABLE(of, bq25896_id);
#endif

static struct i2c_client *new_client;
static const struct i2c_device_id bq25896_i2c_id[] = { {"bq25896", 0}, {} };

kal_bool chargin_hw_init_done = KAL_FALSE;
static int bq25896_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

static void bq25896_shutdown(struct i2c_client *client)
{
	battery_log(BAT_LOG_CRTI, "[bq25896_shutdown] driver shutdown\n");
//	bq25896_set_chg_config(0x0);
}
static struct i2c_driver bq25896_driver = {
		.driver = {
				.name    = "bq25896",
#ifdef CONFIG_OF
				.of_match_table = of_match_ptr(bq25896_id),
#endif
		},
		.probe       = bq25896_driver_probe,
		.id_table    = bq25896_i2c_id,
		.shutdown    = bq25896_shutdown,
};

/**********************************************************
 *
 *[Global Variable]
 *
 *********************************************************/
#define bq25896_REG_NUM 21 
unsigned char bq25896_reg[bq25896_REG_NUM] = {0};
extern void do_chrdet_int_task(void);
extern int Read_HW_ID(void);
extern int g_otg_enable;
static DEFINE_MUTEX(bq25896_i2c_access);
#if defined(CONFIG_Z380M)
struct regulator *VGP2_PMU;
bool b_vgp2_enable = false;
extern void get_soc_p_c(int *pad_soc, int *cover_soc);
extern bool disable_cover_otg(void);
extern void cover_otg_current(int curr);
extern void cover_otg(int on);
extern bool _IS_CA81_(void);
extern bool _IS_CB81_(void);
extern bool VBUS_IN(void);
extern void call_cover_interupt(void);
#elif defined(CONFIG_Z300M)
int usb_in_int;
EXPORT_SYMBOL(usb_in_int);
struct regulator *VIBR_PMU;
bool b_vibr_enable = false;
#endif

/**********************************************************
 *
 *	[I2C Function For Read/Write bq25896]
 *
 *********************************************************/
int bq25896_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char     readData = 0;
	int      ret = 0;
	struct i2c_msg msg[2];
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&bq25896_i2c_access);
	msg[0].addr = new_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &cmd;

	msg[1].addr = new_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &readData;

	ret = i2c_transfer(adap, msg, 2);
	if (ret < 0) {
		mutex_unlock(&bq25896_i2c_access);
		return 0;
	}
	*returnData = readData;

	mutex_unlock(&bq25896_i2c_access);
	return 1;
}

int bq25896_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;
	struct i2c_msg msg;
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&bq25896_i2c_access);
	write_data[0] = cmd;
	write_data[1] = writeData;
	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = (char *)write_data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret < 0) {
		mutex_unlock(&bq25896_i2c_access);
		return 0;
	}

	mutex_unlock(&bq25896_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq25896_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq25896_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq25896_read_byte(RegNum, &bq25896_reg);

	battery_log(BAT_LOG_FULL, "[bq25896_read_interface] Reg[%x]=0x%x\n", RegNum, bq25896_reg);

	bq25896_reg &= (MASK << SHIFT);
	*val = (bq25896_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[bq25896_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq25896_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq25896_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq25896_read_byte(RegNum, &bq25896_reg);
	battery_log(BAT_LOG_FULL, "[bq25896_config_interface] Reg[%x]=0x%x\n", RegNum, bq25896_reg);

	bq25896_reg &= ~(MASK << SHIFT);
	bq25896_reg |= (val << SHIFT);

	ret = bq25896_write_byte(RegNum, bq25896_reg);
	battery_log(BAT_LOG_CRTI, "[bq25896_config_interface] write Reg[%x]=0x%x\n", RegNum, bq25896_reg);

	/* Check */
	/* bq25896_read_byte(RegNum, &bq25896_reg); */
	/* battery_log(BAT_LOG_FULL, "[bq25896_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq25896_reg); */

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0--00------------------------------------------------ */

void bq25896_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
	    );
}
}
void bq25896_set_en_ilim(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_ILIM_MASK),
				       (unsigned char) (CON0_EN_ILIM_SHIFT)
	    );
}
}

void bq25896_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
	    );
}
}

/* CON1--01------------------------------------------------ */
void bq25896_set_bhot(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_BHOT_MASK), 
                                       (unsigned char) (CON1_BHOT_SHIFT)
	    );
}
}

void bq25896_set_bcold(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_BCOLD_MASK), 
                                       (unsigned char) (CON1_BCOLD_SHIFT)
	    );
}
}

void bq25896_set_vindpm_os(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_VINDPM_OS_MASK), 
                                       (unsigned char) (CON1_VINDPM_OS_SHIFT)
	    );
}
}

/* CON2--02------------------------------------------------ */
void bq25896_set_conv_start(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_CONV_START_MASK),
                                       (unsigned char) (CON2_CONV_START_SHIFT)
	    );
}
}

void bq25896_set_conv_rate(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_CONV_RATE_MASK), 
                                       (unsigned char) (CON2_CONV_RATE_SHIFT)
	    );
}
}

void bq25896_set_boost_freq(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BOOST_FREQ_MASK), 
                                       (unsigned char) (CON2_BOOST_FREQ_SHIFT)
	    );
}
}

void bq25896_set_ico_en(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICO_EN_MASK), 
                                       (unsigned char) (CON2_ICO_EN_SHIFT)
	    );
}
}

void bq25896_set_force_dpmp(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_FORCE_DPDM_MASK), 
                                       (unsigned char) (CON2_FORCE_DPDM_SHIFT)
	    );
}
}

void bq25896_set_auto_dpmp_en(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_AUTO_DPDM_EN_MASK), 
                                       (unsigned char) (CON2_AUTO_DPDM_EN_SHIFT)
	    );
}
}

/* CON3--03------------------------------------------------ */
void bq25896_set_bat_loaden(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){
	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_BAT_LOADEN_MASK), 
                                       (unsigned char) (CON3_BAT_LOADEN_SHIFT)
	    );
}
}

void bq25896_set_wd_rst(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){
	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_WD_RST_MASK), 
                                       (unsigned char) (CON3_WD_RST_SHIFT)
	    );
}
}

void bq25896_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){
	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_OTG_CONFIG_MASK), 
                                       (unsigned char) (CON3_OTG_CONFIG_SHIFT)
	    );
}
}

void bq25896_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_CHG_CONFIG_MASK), 
                                       (unsigned char) (CON3_CHG_CONFIG_SHIFT)
	    );
}
}

void bq25896_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_SYS_MIN_MASK), 
                                       (unsigned char) (CON3_SYS_MIN_SHIFT)
	    );
}
}

void bq25896_set_min_vbat_sel(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_MIN_VBAT_SEL_MASK), 
                                       (unsigned char) (CON3_MIN_VBAT_SEL_SHIFT)
	    );
}
}

/* CON4--04------------------------------------------------ */
void bq25896_set_en_pumpx(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_EN_PUMPX_MASK), 
                                       (unsigned char) (CON4_EN_PUMPX_VREG_SHIFT)
	    );
}
}

void bq25896_set_ichg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_ICHG_MASK), 
                                       (unsigned char) (CON4_ICHG_SHIFT)
	    );
}
}

/* CON5--05------------------------------------------------ */
void bq25896_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IPRECHG_MASK), 
                                       (unsigned char) (CON5_IPRECHG_SHIFT)
	    );
}
}

void bq25896_set_iterm(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_ITERM_MASK), 
                                       (unsigned char) (CON5_ITERM_SHIFT)
	    );
}
}

/* CON6--06------------------------------------------------ */
void bq25896_set_vreg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VREG_MASK), 
                                       (unsigned char) (CON6_VREG_SHIFT)
	    );
}
}

unsigned int bq25896_get_vreg(void)
{
        unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){

        ret = bq25896_read_interface((unsigned char) (bq25896_CON6),
                                       (&val),
                                       (unsigned char) (CON6_VREG_MASK),
                                       (unsigned char) (CON6_VREG_SHIFT)
            );
}
	return val;
}

void bq25896_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BATLOWV_MASK), 
                                       (unsigned char) (CON6_BATLOWV_SHIFT)
	    );
}
}

void bq25896_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VRECHG_MASK), 
                                       (unsigned char) (CON6_VRECHG_SHIFT)
	    );
}
}
/* CON7--07------------------------------------------------ */
void bq25896_set_en_term(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_EN_TERM_MASK), 
                                       (unsigned char) (CON7_EN_TERM_SHIFT)
	    );
}
}

void bq25896_set_stat_dis(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_STAT_DIS_MASK), 
                                       (unsigned char) (CON7_STAT_DIS_SHIFT)
	    );
}
}

void bq25896_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_WATCHDOG_MASK), 
                                       (unsigned char) (CON7_WATCHDOG_SHIFT)
	    );
}
}

void bq25896_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_EN_TIMER_MASK), 
                                       (unsigned char) (CON7_EN_TIMER_SHIFT)
	    );
}
}

void bq25896_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_CHG_TIMER_MASK), 
                                       (unsigned char) (CON7_CHG_TIMER_SHIFT)
	    );
}
}

void bq25896_set_jeita_iset(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_JEITA_ISET_MASK), 
                                       (unsigned char) (CON7_JEITA_ISET_SHIFT)
	    );
}
}

/* CON8--08------------------------------------------------ */
void bq25896_set_bat_comp(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON8),
				       (unsigned char) (val),
				       (unsigned char) (CON8_BAT_COMP_MASK), 
                                       (unsigned char) (CON8_BAT_COMP_SHIFT)
	    );
}
}

void bq25896_set_vclamp(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON8),
				       (unsigned char) (val),
				       (unsigned char) (CON8_VCLAMP_MASK), 
                                       (unsigned char) (CON8_VCLAMP_SHIFT)
	    );
}
}

void bq25896_set_treg(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON8),
				       (unsigned char) (val),
				       (unsigned char) (CON8_TREG_MASK), 
                                       (unsigned char) (CON8_TREG_SHIFT)
	    );
}
}

/* CON9--09------------------------------------------------ */
void bq25896_set_force_ico(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_FORCE_ICO_MASK), 
                                       (unsigned char) (CON9_FORCE_ICO_SHIFT)
	    );
}
}

void bq25896_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_TRM2X_EN_MASK), 
                                       (unsigned char) (CON9_TRM2X_EN_SHIFT)
	    );
}
}

void bq25896_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_BATFET_DIS_MASK), 
                                       (unsigned char) (CON9_BATFET_DIS_SHIFT)
	    );
}
}

void bq25896_set_jeita_vset(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_JEITA_VSET_MASK), 
                                       (unsigned char) (CON9_JEITA_VSET_SHIFT)
	    );
}
}

void bq25896_set_batfet_ddlay(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_BATFET_DLY_MASK), 
                                       (unsigned char) (CON9_BATFET_DLY_SHIFT)
	    );
}
}

void bq25896_set_batfet_rst_en(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_BATFET_RST_EN_MASK), 
                                       (unsigned char) (CON9_BATFET_RST_EN_SHIFT)
	    );
}
}

void bq25896_set_pumpx_up(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_PUMPX_UP_MASK), 
                                       (unsigned char) (CON9_PUMPX_UP_SHIFT)
	    );
}
}

void bq25896_set_pumpx_dn(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON9),
				       (unsigned char) (val),
				       (unsigned char) (CON9_PUMPX_DN_MASK), 
                                       (unsigned char) (CON9_PUMPX_DN_SHIFT)
	    );
}
}
/* CON10-0A------------------------------------------------ */
void bq25896_set_boostv(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_BOOSTV_MASK), 
                                       (unsigned char) (CON10_BOOSTV_SHIFT)
	    );
}
}

void bq25896_set_pfm_otg_dis(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_PFM_OTG_DIS_MASK), 
                                       (unsigned char) (CON10_PFM_OTG_DIS_SHIFT)
	    );
}
}

void bq25896_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_BOOST_LIM_MASK), 
                                       (unsigned char) (CON10_BOOST_LIM_SHIFT)
	    );
}
}

/* CON11-0B------------------------------------------------ */
unsigned int bq25896_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON11),
				       (&val),
				       (unsigned char) (CON11_VBUS_STAT_MASK), 
                                       (unsigned char) (CON11_VBUS_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON11),
				       (&val),
				       (unsigned char) (CON11_CHRG_STAT_MASK), 
                                       (unsigned char) (CON11_CHRG_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_pg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON11),
				       (&val),
				       (unsigned char) (CON11_PG_STAT_MASK), 
                                       (unsigned char) (CON11_PG_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON11),
				       (&val),
				       (unsigned char) (CON11_VSYS_STAT_MASK), 
                                       (unsigned char) (CON11_VSYS_STAT_SHIFT)
	    );
}
	return val;
}

/* CON12-0C------------------------------------------------ */
unsigned int bq25896_get_watchdog_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_WATCHDOG_FAULT_MASK), 
                                       (unsigned char) (CON12_WATCHDOG_FAULT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_boost_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_BOOST_FAULT_MASK), 
                                       (unsigned char) (CON12_BOOST_FAULT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_chrg_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_CHRG_FAULT_MASK), 
                                       (unsigned char) (CON12_CHRG_FAULT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_bat_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_BAT_FAULT_MASK), 
                                       (unsigned char) (CON12_BAT_FAULT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_ntc_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON12),
				       (&val),
				       (unsigned char) (CON12_NTC_FAULT_MASK), 
                                       (unsigned char) (CON12_NTC_FAULT_SHIFT)
	    );
}
	return val;
}

/* CON13-0D------------------------------------------------ */
void bq25896_set_force_vindpm(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON13),
				       (unsigned char) (val),
				       (unsigned char) (CON13_FORCE_VINDPM_MASK), 
                                       (unsigned char) (CON13_FORCE_VINSPM_SHIFT)
	    );
}
}

void bq25896_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON13),
				       (unsigned char) (val),
				       (unsigned char) (CON13_VINDPM_MASK), 
                                       (unsigned char) (CON13_VINDPM_SHIFT)
	    );
}
}

/* CON14-0E------------------------------------------------ */
unsigned int bq25896_get_therm_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON14),
				       (&val),
				       (unsigned char) (CON14_THERM_STAT_MASK), 
                                       (unsigned char) (CON14_THERM_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_batv(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON14),
				       (&val),
				       (unsigned char) (CON14_BATV_MASK), 
                                       (unsigned char) (CON14_BATV_SHIFT)
	    );
}
	return val;
}

/* CON15-0F------------------------------------------------ */
unsigned int bq25896_get_sysv(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON15),
				       (&val),
				       (unsigned char) (CON15_SYSV_MASK), 
                                       (unsigned char) (CON15_SYSV_SHIFT)
	    );
}
	return val;
}

/* CON16-10------------------------------------------------ */
unsigned int bq25896_get_tspct(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON16),
				       (&val),
				       (unsigned char) (CON16_TSPCT_MASK), 
                                       (unsigned char) (CON16_TSPCT_SHIFT)
	    );
}
	return val;
}

/* CON17-11------------------------------------------------ */
unsigned int bq25896_get_vbus_gd(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON17),
				       (&val),
				       (unsigned char) (CON17_VBUS_GD_MASK), 
                                       (unsigned char) (CON17_VBUS_GD_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_vbusv(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON17),
				       (&val),
				       (unsigned char) (CON17__MASK), 
                                       (unsigned char) (CON17__SHIFT)
	    );
}
	return val;
}

/* CON18-12------------------------------------------------ */
unsigned int bq25896_get_ichgr(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON18),
				       (&val),
				       (unsigned char) (CON18_ICHGR_MASK), 
                                       (unsigned char) (CON18_ICHGR_SHIFT)
	    );
}
	return val;
}

/* CON19-13------------------------------------------------ */
unsigned int bq25896_get_vdpm_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON19),
				       (&val),
				       (unsigned char) (CON19_VDPM_STAT_MASK), 
                                       (unsigned char) (CON19_VDPM_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_idpm_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON19),
				       (&val),
				       (unsigned char) (CON19_IDPM_STAT_MASK), 
                                       (unsigned char) (CON19_IDPM_STAT_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_idpm_lim(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON19),
				       (&val),
				       (unsigned char) (CON19_IDPM_LIM_MASK), 
                                       (unsigned char) (CON19_IDPM_LIM_SHIFT)
	    );
}
	return val;
}

/* CON20-14------------------------------------------------ */
void bq25896_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON20),
				       (unsigned char) (val),
				       (unsigned char) (CON20_REG_RST_MASK), 
                                       (unsigned char) (CON20_REG_RST_SHIFT)
	    );
}
}

void bq25896_set_ico_optimized(unsigned int val)
{
	unsigned int ret = 0;
if (DEBUG_REG){

	ret = bq25896_config_interface((unsigned char) (bq25896_CON20),
				       (unsigned char) (val),
				       (unsigned char) (CON20_ICO_OPTIMIZED_MASK), 
                                       (unsigned char) (CON20_ICO_OPTIMIZED_SHIFT)
	    );
}
}

unsigned int bq25896_get_pn(void)
{
	unsigned char val = 0;
	unsigned int ret = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON20),
				       (&val),
				       (unsigned char) (CON20_PN_MASK), 
                                       (unsigned char) (CON20_PN_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_ts_profile(void)
{
	unsigned char val = 0;
	unsigned int ret = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON20),
				       (&val),
				       (unsigned char) (CON20_TS_PROFILE_MASK), 
                                       (unsigned char) (CON20_TS_PROFILE_SHIFT)
	    );
}
	return val;
}

unsigned int bq25896_get_dev_rev(void)
{
	unsigned char val = 0;
	unsigned int ret = 0;
if (DEBUG_REG){
	
	ret = bq25896_read_interface((unsigned char) (bq25896_CON20),
				       (&val),
				       (unsigned char) (CON20_DEV_REV_MASK), 
                                       (unsigned char) (CON20_DEV_REV_SHIFT)
	    );
}
	return val;
}

/**********************************************************
  *
  *   [External Function]
  *
 *********************************************************/
#if defined(CONFIG_Z300M)
bool IS_STAND_AC(void)
{
	return gpio_get_value(charger_gpio->DCIN_VBUS_IN_DET_N)?false:true;
} 
EXPORT_SYMBOL(IS_STAND_AC);

bool VBUS_IN(void)
{
	return gpio_get_value(charger_gpio->N1_VBUS_IN_DET_NUM)?false:true;
}
EXPORT_SYMBOL(VBUS_IN);

void charging_pad_or_standac(void)
{
	if (IS_STAND_AC()){
                battery_log(BAT_LOG_CRTI, "<BATT>[%s]charging mode: pad+stand(ac)\n", __func__);
		g_stand_int = 1 ;
                Set_VIBR_PMU(0);
                msleep(1000);
                bq25896_set_watchdog(0x3);      // 1. watch dog timer = 160 sec
                bq25896_set_ichg(0x22);         // 2. Set Fast Charge Current Limit = 2176 mA
                bq25896_set_iprechg(0x5);       //    Set Pre-Charge Current Limit = 384 mA
                bq25896_set_iterm(0x2);         //    Set Termination Current Limit = 192 mA
                bq25896_set_iinlim(0x12);       // 3. Set Input Current Limit = 1000 mA
                bq25896_set_vreg(0x21);         // 4. Set Charge Voltage Limit = 4.368 V
                bq25896_set_vindpm_os(0x7);     // 5. Voltage DPM offset = 700 mV
                bq25896_set_sys_min(0x6);       // 6. Minimum System Voltage = 3.6 V
                bq25896_set_en_ilim(0x0);       // 7. Disable Current Limit
                charger_set_gpio(0,0,0,0);
		bq25896_set_wd_rst(0x1);
		cancel_delayed_work(&charger_gpio->otg_hub_work);
		queue_delayed_work(stand_wq, &charger_gpio->otg_hub_work, msecs_to_jiffies(60000));
        }
        else {
                battery_log(BAT_LOG_CRTI, "<BATT>[%s]charging mode: pad only\n", __func__);
                Set_VIBR_PMU(0);
                bq25896_set_watchdog(0x3);      // 1. watch dog timer = 160 sec
                bq25896_set_en_ilim(0x1);       // 2. enable current limit
                bq25896_set_iinlim(0x8);        // 3. set input current limit = 500 mA
                bq25896_set_auto_dpmp_en(0x0);  // 4. disable auto_dpdm
                charger_set_gpio(0,0,1,0);
		bq25896_set_wd_rst(0x1);
		cancel_delayed_work(&charger_gpio->otg_hub_work);
		queue_delayed_work(stand_wq, &charger_gpio->otg_hub_work, msecs_to_jiffies(60000));
        }
}
EXPORT_SYMBOL(charging_pad_or_standac);
#endif
/**********************************************************
  *
  *   [Internal Function]
  *
 *********************************************************/
void bq25896_dump_register(void)
{
	int i = 0;

	battery_log(BAT_LOG_CRTI, "[bq25896] ");
	for (i = 0; i < bq25896_REG_NUM; i++) {
		bq25896_read_byte(i, &bq25896_reg[i]);
		battery_log(BAT_LOG_CRTI, "[0x%x]=0x%x ", i, bq25896_reg[i]);
	}
	battery_log(BAT_LOG_CRTI, "\n");
}
#if defined(CONFIG_Z380M)
void Set_VGP2_PMU(int Enable){
	int ret;
	int HW_ID;
	HW_ID = Read_HW_ID();
	printk("[%s] Want to set VGP2 [0x%d]\n",__func__,Enable);
	if(HW_ID != 1){//EVB, ER, PR, MP
		if(IS_ERR(VGP2_PMU)){//if get regulator error, re-get
                        ret=PTR_ERR(VGP2_PMU);
			printk("[%s] Fail to get VGP2 \n",__func__);
                        VGP2_PMU = regulator_get(&new_client->dev,"reg_vgp2");
		}
		ret = regulator_is_enabled(VGP2_PMU);//check enable status
		b_vgp2_enable=Enable;
		if(Enable == true){
                	if (ret){//11
                        	printk("[%s] Already [0x%d] before, Enable VGP2 pass!!\n",__func__,ret);	//Already enable, ignore
                        }else{//10
				ret = regulator_set_voltage(VGP2_PMU,3000000,3000000);
	                	ret = regulator_enable(VGP2_PMU);
        		        if (ret)
	        	                printk("[%s] Enable VGP2 fail!!\n",__func__);
				else
					printk("[%s] Enable VGP2 pass!!\n",__func__);
			}
		}else{
                        if (ret){//01
				ret = regulator_disable(VGP2_PMU);
				if (ret)
                                        printk("[%s] Disable VGP2 fail!!\n",__func__);
                                else
                                        printk("[%s] Disable VGP2 pass!!\n",__func__);
                        }else{//00
					printk("[%s] Already [0x%d] before, Disable VGP2 pass!!\n",__func__,ret);	//Already disable, ignore
			}
		}
	}else{
		printk("[%s] VGP2 set by touch@SR\n",__func__);
	}
}
#if 0
static void charging_otg_balance(int mode)
{
	battery_log(BAT_LOG_CRTI, "<BATT>[%s](%d)\n", __func__, mode);
	switch(mode)
	{
		case 0:
			if(g_flag_case != mode)
			{
				battery_log(BAT_LOG_CRTI, "[%s] g_flag_case = %d\n", __func__, g_flag_case);
				/* PAD OTG Disable */
				bq25896_set_boost_lim(0);	// 1. Set Boost current limit = 500 mA
				bq25896_set_otg_config(0);	// 2. Disable OTG
				gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, 0);
				bq25896_set_iinlim(0x4);	// 4. Set Input Current Limit = 300 mA
				bq25896_set_en_ilim(0);		// 5. Disable Current Limit

				/* Delay 1 Sec */
				msleep_interruptible(1000);

				/* Cover OTG Enable */
				cover_otg_current(900);		// 1. Allow Violate Register can be written
								// 2. Set Cover OTG output current = 900 mA
				cover_otg(1);			// 3. Set OTG/ID Pin Control
				charger_set_gpio(0, 1, 1, 0);
				g_flag_case = 0;
			}
			break;
		case 1:
			if(g_flag_case != mode)
			{
				battery_log(BAT_LOG_CRTI, "[%s] g_flag_case = %d\n", __func__, g_flag_case);
				charger_set_gpio(0, 0, 1, 0);
				/* Cover OTG Disable */
				disable_cover_otg();		// 1. Allow Violate Register can be written
								// 2. Set OTG/ID I2C Control
								// 3. Diable OTG

				/* Delay 1 sec */
				msleep_interruptible(1000);

				/* PAD OTG Enable*/
				bq25896_set_boost_lim(0x6);	// 1. Set Boost current limit = 1400 mA
				bq25896_set_otg_config(1);	// 2. Enable OTG
				gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, 1);
				g_flag_case = 1;
			}
			break;
	}
}
#endif

static void otg_cb81_function(struct work_struct *work)
{
	int pad = 101, cover =101;
	cancel_delayed_work(&charger_gpio->otg_cb81_work);
	battery_log(BAT_LOG_CRTI, "<BATT>[%s]\n", __func__);
	get_soc_p_c(&pad, &cover);	
	bq25896_set_wd_rst(0x1);
	if(g_otg_enable || !VBUS_IN())
		queue_delayed_work(otg_wq, &charger_gpio->otg_cb81_work, msecs_to_jiffies(60000));
}
#elif defined(CONFIG_Z300M)
/* interrupt_handler */
static irqreturn_t stand_interrupt_handler(int irq, void *dev_id)
{
	battery_log(BAT_LOG_CRTI, "[%s] stand_interrupt = %d\n", "stand_irq", irq);
	/* Begin to deal with interrupt bottom half */
	if(!g_otg_enable && !VBUS_IN())
	{
		battery_log(BAT_LOG_CRTI, "[%s] !g_otg_enable && !VBUS_IN()\n", "__func__" );
		if (IS_STAND_AC()){
			g_stand_int = 1;
			battery_log(BAT_LOG_CRTI, "[%s]IS_STAND_AC()=%d \n", "__func__", IS_STAND_AC());
		}
		cancel_delayed_work(&charger_gpio->stand_work);
		queue_delayed_work(stand_wq, &charger_gpio->stand_work, msecs_to_jiffies(0));
	}
	/* Create the wakelock to ensure work can be finished before system suspend */
	return IRQ_HANDLED;
}

void call_stand_interupt(void)
{
	battery_log(BAT_LOG_CRTI, "[%s] \n", __func__);
	stand_interrupt_handler(charger_gpio->stand_ac_irq, charger_gpio->dev);
}
EXPORT_SYMBOL_GPL(call_stand_interupt);

static irqreturn_t cvbus_interrupt_handler(int irq, void *dev_id)
{
        battery_log(BAT_LOG_CRTI, "[%s] cover_vbus_interrupt = %d, g_cvbus_in_int = %d\n", "cvbus_irq", irq, g_cvbus_in_int);
        /* Begin to deal with interrupt bottom half */
        g_cvbus_in_int += 1;
//        cancel_delayed_work(&charger_gpio->cvbus_work);
        queue_delayed_work(cvbus_wq, &charger_gpio->cvbus_work, msecs_to_jiffies(160));
        if (g_cvbus_in_int > 8)
	{
		if(gpio_get_value(charger_gpio->PG_OTG_EN_SOC))
		{
			bq25896_set_otg_config(0x0);	// 1. Disable OTG
			msleep_interruptible(20);	// 2. delay 20 ms
			bq25896_set_otg_config(0x1);	// 3. Enable OTG
		}
	}
        return IRQ_HANDLED;
}

static irqreturn_t usb_interrupt_handler(int irq, void *dev_id)
{
        pr_err("[%s] usb_interrupt = %d\n", "usb_irq", irq);
        /* Begin to deal with interrupt bottom half */
//	if(!VBUS_IN())
//		bq25896_set_en_ilim(0x1);
	usb_in_int = 1;
        cancel_delayed_work(&charger_gpio->usb_work);
        queue_delayed_work(stand_wq, &charger_gpio->usb_work, msecs_to_jiffies(1000));
        /* Create the wakelock to ensure work can be finished before system suspend */
        return IRQ_HANDLED;
}
/* set gpio to irq */
static int set_irq_stand(struct charger_gpio *chip)
{
        int rc = 0 ;
        int irq_flags;

        /* Accroding to irq domain mappping GPIO number to IRQ number */
        chip->stand_ac_irq = gpio_to_irq(chip->DCIN_VBUS_IN_DET_N);
        pr_err("[%s] stand ac irq = %d\n", __func__, chip->stand_ac_irq);
        chip->usb_irq = gpio_to_irq(chip->N1_VBUS_IN_DET_NUM);
        pr_err("[%s] usb irq = %d\n", __func__, chip->usb_irq);
        chip->cvbus_irq = gpio_to_irq(chip->DCIN_VBUS_IN_DET_N);
        pr_err("[%s] cover vbus in irq = %d\n", __func__, chip->cvbus_irq);

        irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
        /* IRQs requested with this function will be automatically freed on driver detach */

        rc = devm_request_irq(chip->dev, chip->stand_ac_irq, stand_interrupt_handler, irq_flags ,"stand_irq" ,chip);
        if (rc < 0) {
            pr_err("[%s]Couldn't register for stand ac interrupt,irq = %d, rc = %d\n", "stand_irq", chip->stand_ac_irq ,rc);
            rc = -EIO;
            return rc ;
        }

        rc = devm_request_irq(chip->dev, chip->usb_irq, usb_interrupt_handler, irq_flags ,"usb_irq" ,chip);
        if (rc < 0) {
            pr_err("[%s]Couldn't register for usb interrupt,irq = %d, rc = %d\n", "stand_irq", chip->usb_irq ,rc);
            rc = -EIO;
            return rc ;
        }

        rc = devm_request_irq(chip->dev, chip->cvbus_irq, cvbus_interrupt_handler, irq_flags ,"cvbus_irq" ,chip);
        if (rc < 0) {
            pr_err("[%s]Couldn't register for cover vbus in interrupt,irq = %d, rc = %d\n", "cvbus_irq", chip->cvbus_irq ,rc);
            rc = -EIO;
            return rc ;
        }

        /* Enable this irq line */
        enable_irq_wake(chip->stand_ac_irq);
        enable_irq_wake(chip->cvbus_irq);
        enable_irq_wake(chip->usb_irq);

        return 0;
}

static void usb_report_function(struct work_struct *work)
{
	battery_log(BAT_LOG_CRTI, "<BATT>[%s]\n", __func__);
	if (!gpio_get_value(charger_gpio->DCIN_VBUS_IN_DET_N))
	{
		BMT_status.charger_type = CHARGER_UNKNOWN;
		do_chrdet_int_task();
	}
        usb_in_int = 0;
}
static void cvbus_report_function(struct work_struct *work)
{
	battery_log(BAT_LOG_CRTI, "<BATT>[%s]\n", __func__);
	g_cvbus_in_int = 0;
        cancel_delayed_work(&charger_gpio->cvbus_work);
}


//static struct timer_list stand_timer;
//void timer_set_stand(unsigned long data){
static void stand_timer_function(struct work_struct *work)
{
       int stand_in = 2;
       int stand_ac_in = 2;
       int usb_in = 2;

	pr_err("%s\n",__func__);
	
        stand_in = gpio_get_value(charger_gpio->N1_FUNCTION_NUN);
        stand_ac_in = gpio_get_value(charger_gpio->DCIN_VBUS_IN_DET_N);
        usb_in = gpio_get_value(charger_gpio->N1_VBUS_IN_DET_NUM);
        battery_log(BAT_LOG_CRTI, "stand=%d, stand_ac=%d, usb=%d\n",stand_in,stand_ac_in,usb_in);

	if (g_otg_enable)
		battery_log(BAT_LOG_CRTI, "<BATT>[%s] OTG\n", __func__);
	else if (usb_in_int == 0 && usb_in == 0)
		do_chrdet_int_task();
	else
		charging_pad_or_standac();
        usb_in_int = 0;

}
static void stand_report_function(struct work_struct *work)
{
       battery_log(BAT_LOG_CRTI, "%s\n",__func__);

	cancel_delayed_work(&charger_gpio->stand_timer_work);
	queue_delayed_work(stand_wq, &charger_gpio->stand_timer_work, msecs_to_jiffies(900));
}

void Set_VIBR_PMU(int Enable){
	int ret;
	printk("[%s] Want to set VIBR [0x%d] \n",__func__,Enable);
	if(IS_ERR(VIBR_PMU)){//if get regulator error
		ret=PTR_ERR(VIBR_PMU);
		printk("[%s] Failed to get VIBR \n",__func__);
        	VIBR_PMU = regulator_get(&new_client->dev,"reg_vibr");//re-get
	}
	b_vibr_enable = Enable;
	ret = regulator_is_enabled(VIBR_PMU);//check enable status
	if(Enable == true){
		if (ret){//11
                	printk("[%s] Already [0x%d] before, Enable VIBR pass!!\n",__func__,ret);//Already enable
	        }else{//10
			ret = regulator_set_voltage(VIBR_PMU,1800000,1800000);
			ret = regulator_enable(VIBR_PMU);
        		if (ret)
	                	printk("[%s] Enable VIBR fail!!\n",__func__);
			else
				printk("[%s] Enable VIBR pass!!\n",__func__);
		}
	}else{
		if (ret){//01
                        printk("[%s] Enable status of reg-vibr [0x%d]\n",__func__,ret);
			ret = regulator_disable(VIBR_PMU);
                        if (ret)
                        	printk("[%s] Disable VIBR fail!!\n",__func__);
			else
				printk("[%s] Disable VIBR pass!!\n",__func__);
                }else{//00
                        printk("[%s] Already [0x%d] before, Disable VIBR pass!!\n",__func__,ret);//Already Disable
                }
	}
}

static void otg_hub_function(struct work_struct *work) // plug in usb hub, N1_VBUS_IN_DET# hi/lo/hi/lo....
{
	bq25896_set_wd_rst(0x1);	
	cancel_delayed_work(&charger_gpio->otg_hub_work);
	if( g_otg_enable || !VBUS_IN())
		queue_delayed_work(stand_wq, &charger_gpio->otg_hub_work, msecs_to_jiffies(60000));
	
}
#endif

static int bq25896_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#if defined(CONFIG_Z300M)
	int ret;
#endif
	struct device *dev = &client->dev;
	struct charger_gpio *charger;
	kal_bool chr_status;

	battery_log(BAT_LOG_CRTI, "[bq25896_driver_probe]\n");
	charger = devm_kzalloc(dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;
	charger->dev = dev;
	charger_gpio = charger;

	new_client = client;
	charger_get_gpio_info();
	charger_gpio_request();
	
	/* --------------------- */
	/*bq25896_hw_component_detect();*/
	
#ifdef CONFIG_Z380M    
        VGP2_PMU = devm_regulator_get(&client->dev,"vgp2_pmu");
//	cvbus_wq = create_singlethread_workqueue("cvbus_wq");
//	INIT_DEFERRABLE_WORK(&charger_gpio->cvbus_work, cvbus_report_function);
//	ret = set_irq_cvbus(charger_gpio);
	otg_wq = create_singlethread_workqueue("otg_wq");
	INIT_DEFERRABLE_WORK(&charger_gpio->otg_cb81_work, otg_cb81_function);
#endif
#if defined(CONFIG_Z300M)
	/* Initialize workqueue and assign stand_report_function to work */
	stand_wq = create_singlethread_workqueue("stand_wq");
	cvbus_wq = create_singlethread_workqueue("cvbus_wq");
	INIT_DEFERRABLE_WORK(&charger_gpio->cvbus_work, cvbus_report_function);
	INIT_DEFERRABLE_WORK(&charger_gpio->stand_work, stand_report_function);
	INIT_DEFERRABLE_WORK(&charger_gpio->stand_timer_work, stand_timer_function);
	INIT_DEFERRABLE_WORK(&charger_gpio->otg_hub_work, otg_hub_function);
	INIT_DEFERRABLE_WORK(&charger_gpio->usb_work, usb_report_function);

	ret = set_irq_stand(charger_gpio);
	if (ret < 0){
		battery_log(BAT_LOG_CRTI, "fail to set stand_irq\n");
		gpio_free(charger_gpio->N1_FUNCTION_NUN);
                gpio_free(charger_gpio->DCIN_VBUS_IN_DET_N);
                gpio_free(charger_gpio->N1_VBUS_IN_DET_NUM);
		return ret;
	}
	VIBR_PMU = regulator_get(&client->dev,"reg_vibr");
#endif
	printk("[%s] CHARGING_CMD_GET_CHARGER_DET_STATUS\n",__func__);
	chr_control_interface(CHARGING_CMD_GET_CHARGER_DET_STATUS, &chr_status);
        if(chr_status == TRUE){
		printk("[%s] Can't distinguish Charger Type > Continue\n",__func__);
        }else{
                printk("[%s] PAD only\n",__func__);
                chr_control_interface(CHARGING_CMD_INIT, NULL);
        }
        bq25896_dump_register();
        chargin_hw_init_done = KAL_TRUE;

	return 0;
}

/**********************************************************
 *
 *	[platform_driver API]
 *
 *********************************************************/
unsigned char g_reg_value_bq25896 = 1;
bool b_eng_charging_limit = true;
#if defined(CONFIG_Z300M)
bool b_adc_sw_en = false;
#endif
static ssize_t show_ICHGR(struct device *dev, struct device_attribute *attr, char *buf)
{
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
	return sprintf(buf, "%d\n", current1);
}
static ssize_t store_ICHGR(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        return size;
}
static DEVICE_ATTR(ICHGR, 0664, show_ICHGR, store_ICHGR);
#if defined(CONFIG_Z380M)
static ssize_t show_charging_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[%s] charging mode = %d\n", __func__, BMT_status.old_charging_mode);
	battery_log(BAT_LOG_CRTI, "PAD = 0,\nAC = 1,\nSDP = 2,\nCDP = 3,\nCOVER = 4,\nCOVER_AC = 5,\nCOVER_SDP = 6,\nCOVER_CDP = 7,\nCOVER_OTG = 8,\nOTG = 9,\n");
	
	return sprintf(buf, "%d\n", BMT_status.old_charging_mode);
}
static ssize_t store_charging_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        return size;
}
static DEVICE_ATTR(charging_mode, 0664, show_charging_mode, store_charging_mode);

/*establish file node for ADC_SW_EN control test*/
#elif defined(CONFIG_Z300M)
static ssize_t show_bq25896_ADC_SW_EN(struct device *dev, struct device_attribute *attr, char *buf)
{
        battery_log(BAT_LOG_CRTI, "[show_bq25896_ADC_SW_EN] b_adc_sw_en =%d\n",b_adc_sw_en);
        return sprintf(buf, "%d\n", b_adc_sw_en);
}

static ssize_t store_bq25896_ADC_SW_EN(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int request;
	battery_log(BAT_LOG_CRTI, "[store_bq25896_ADC_SW_EN] Access\n");
	sscanf(buf, "%du", &request);
	if (!request){
		b_adc_sw_en = false;
                gpio_direction_output(charger_gpio->ADC_SW_EN, request);
		battery_log(BAT_LOG_CRTI, "[store_bq25896_ADC_SW_EN] Set ADC_SW_EN = %d\n",request);
        }else{
		b_adc_sw_en = true;
                gpio_direction_output(charger_gpio->ADC_SW_EN, request);
		battery_log(BAT_LOG_CRTI, "[store_bq25896_ADC_SW_EN] Set ADC_SW_EN = %d\n",request);
        }
        battery_log(BAT_LOG_CRTI, "[store_bq25896_ADC_SW_EN] Finish\n");
        return size;
}
static DEVICE_ATTR(bq25896_ADC_SW_EN, 0664, show_bq25896_ADC_SW_EN, store_bq25896_ADC_SW_EN);
#endif
/*establish file node for eng_charging limit enable*/
static ssize_t show_bq25896_eng_charging_limit(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_bq25896_eng_charging_limit] eng_charging_limit = %d\n", b_eng_charging_limit);
	return sprintf(buf, "%d\n", b_eng_charging_limit);
}

static ssize_t store_bq25896_eng_charging_limit(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int request;
	battery_log(BAT_LOG_CRTI, "[store_bq25896_access]\n");
	sscanf(buf, "%du", &request);
	
	if (!request){
		b_eng_charging_limit = false;
                battery_log(BAT_LOG_CRTI, "[store_bq25896_eng_charging_limit] eng_charging_limit = %d\n",b_eng_charging_limit);
	}else{
		b_eng_charging_limit = true;
		battery_log(BAT_LOG_CRTI, "[store_bq25896_eng_charging_limit] eng_charging_limit = %d\n",b_eng_charging_limit);
	}
	BAT_thread();
	return size;
}
#ifdef ENG_BUILD
static DEVICE_ATTR(bq25896_eng_charging_limit, 0666, show_bq25896_eng_charging_limit, store_bq25896_eng_charging_limit);
#else
static DEVICE_ATTR(bq25896_eng_charging_limit, 0664, show_bq25896_eng_charging_limit, store_bq25896_eng_charging_limit);
#endif
/*end charging limit file node.*/
#ifdef CONFIG_Z300M
/*establish file node for VIBR_Enable*/
static ssize_t show_bq25896_VIBR_Enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        int ret;
	battery_log(BAT_LOG_CRTI, "[%s] b_vibr_enable = %d\n",__func__, b_vibr_enable);
	ret = regulator_is_enabled(VIBR_PMU);
	battery_log(BAT_LOG_CRTI, "[%s] b_vibr_enable = %d\n",__func__, ret);
        return sprintf(buf, "%d\n", b_vibr_enable);
}

static ssize_t store_bq25896_VIBR_Enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        int request;
        battery_log(BAT_LOG_CRTI, "[%s] Start!!\n",__func__);
        sscanf(buf, "%du", &request);

        if (!request){
                b_vibr_enable = false;
                battery_log(BAT_LOG_CRTI, "[%s] b_vibr_enable = %d\n",__func__,b_vibr_enable);
        }else{
                b_vibr_enable = true;
                battery_log(BAT_LOG_CRTI, "[%s] b_vibr_enable = %d\n",__func__,b_vibr_enable);
        }
        Set_VIBR_PMU(b_vibr_enable);
        return size;
}
static DEVICE_ATTR(bq25896_vibr_enable, 0664, show_bq25896_VIBR_Enable, store_bq25896_VIBR_Enable);
/*end VIBR_Enable file node.*/
#endif
#ifdef CONFIG_Z380M
/*establish file node for VGP2_Enable*/
static ssize_t show_bq25896_VGP2_Enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        int ret;
        battery_log(BAT_LOG_CRTI, "[%s] b_vgp2_enable = %d\n",__func__, b_vgp2_enable);
        ret = regulator_is_enabled(VGP2_PMU);
        battery_log(BAT_LOG_CRTI, "[%s] b_vgp2_enable = %d\n",__func__, ret);
        return sprintf(buf, "%d\n", b_vgp2_enable);
}

static ssize_t store_bq25896_VGP2_Enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        int request;
        battery_log(BAT_LOG_CRTI, "[%s] Start!!\n",__func__);
        sscanf(buf, "%du", &request);

        if (!request){
                b_vgp2_enable = false;
                battery_log(BAT_LOG_CRTI, "[%s] b_vgp2_enable = %d\n",__func__,b_vgp2_enable);
        }else{
                b_vgp2_enable = true;
                battery_log(BAT_LOG_CRTI, "[%s] b_vgp2_enable = %d\n",__func__,b_vgp2_enable);
        }
        Set_VGP2_PMU(b_vgp2_enable);
        return size;
}
static DEVICE_ATTR(bq25896_vgp2_enable, 0664, show_bq25896_VGP2_Enable, store_bq25896_VGP2_Enable);
/*end VGP2_Enable file node.*/
#endif
static ssize_t show_bq25896_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_bq25896_access] 0x%x\n", g_reg_value_bq25896);
	return sprintf(buf, "%u\n", g_reg_value_bq25896);
}
static ssize_t store_bq25896_access(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	battery_log(BAT_LOG_CRTI, "[store_bq25896_access]\n");

	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[store_bq25896_access] buf is %s and size is %zu\n", buf, size);
		/*reg_address = kstrtoul(buf, 16, &pvalue);*/

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			battery_log(BAT_LOG_CRTI,
			    "[store_bq25896_access] write bq25896 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = bq25896_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq25896_read_interface(reg_address, &g_reg_value_bq25896, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq25896_access] read bq25896 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_bq25896);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq25896_access] Please use \"cat bq25896_access\" to get value\r\n");
		}
	}
	return size;
}
static DEVICE_ATTR(bq25896_access, 0664, show_bq25896_access, store_bq25896_access);

static ssize_t show_bq25896_dump (struct device *dev, struct device_attribute *attr, char *buf)
{
        int i = 0;
	char *p = buf;

        p += sprintf(p,"[bq25896]\n");
        for (i = 0; i < bq25896_REG_NUM; i++) {
                bq25896_read_byte(i, &bq25896_reg[i]);
		p += sprintf(p, " [0x%x]=0x%x \n",i,bq25896_reg[i]);
        }
	i = p - buf;		
	return i;
}
static ssize_t store_bq25896_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static DEVICE_ATTR(bq25896_dump, 0664, show_bq25896_dump, store_bq25896_dump);

static ssize_t show_otg_on (struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "otg_on\n");;	
}
static ssize_t store_otg_on(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        int ret = 0;
        char *pvalue = NULL, *gpio;
        unsigned int value = 0;

        printk(KERN_ERR "%s\n", __func__);

        pvalue = (char *)buf;
        gpio = strsep(&pvalue, " ");
        ret = kstrtou32(gpio, 16, (unsigned int *)&value);
        gpio_direction_output(charger_gpio->PG_OTG_EN_SOC,value);
	bq25896_set_otg_config(value);	
			
        printk(KERN_ERR "PG_OTG_EN_SOC = %d\n",gpio_get_value(charger_gpio->PG_OTG_EN_SOC));
	
	return size;
}
static DEVICE_ATTR(otg_on, 0664, show_otg_on, store_otg_on);

static ssize_t show_gpio_set(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *p = buf;
	int ret = 0;
	p += sprintf(p,"USB_BUS_on_NUM = %d\n",gpio_get_value(charger_gpio->USB_BUS_on_NUM));
	p += sprintf(p,"Cover_BUS_on = %d\n",gpio_get_value(charger_gpio->Cover_BUS_on));
	p += sprintf(p,"Cover_BUS_off = %d\n",gpio_get_value(charger_gpio->Cover_BUS_off));
	ret = p - buf;
	return ret;
}
static ssize_t store_gpio_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *gpio;
	unsigned int value = 0;

	printk(KERN_ERR "%s\n", __func__);

	pvalue = (char *)buf;
	gpio = strsep(&pvalue, " ");
	ret = kstrtou32(gpio, 16, (unsigned int *)&value);
	gpio_direction_output(charger_gpio->USB_BUS_on_NUM,value);
	printk(KERN_ERR "USB_BUS_on_NUM = %d\n",gpio_get_value(charger_gpio->USB_BUS_on_NUM));
	
	gpio = strsep(&pvalue, " ");
        ret = kstrtou32(gpio, 16, (unsigned int *)&value);
        gpio_direction_output(charger_gpio->Cover_BUS_on,value);
        printk(KERN_ERR "Cover_BUS_on = %d\n",gpio_get_value(charger_gpio->Cover_BUS_on));

	gpio = strsep(&pvalue, " ");
        ret = kstrtou32(gpio, 16, (unsigned int *)&value);
        gpio_direction_output(charger_gpio->Cover_BUS_off,value);
        printk(KERN_ERR "= %d\n",gpio_get_value(charger_gpio->Cover_BUS_off));
        return size;
}
static DEVICE_ATTR(gpio_set, 0664, show_gpio_set, store_gpio_set);



static int bq25896_user_space_probe(struct platform_device *dev)
{
		int ret_device_file = 0;
	battery_log(BAT_LOG_CRTI, "******** bq25896_user_space_probe!! ********\n");

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_access);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_dump);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_eng_charging_limit);
#ifdef CONFIG_Z300M
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_vibr_enable);
#endif
#ifdef CONFIG_Z380M
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_vgp2_enable);
#endif
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_gpio_set);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ICHGR);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_otg_on);
#if defined(CONFIG_Z380M)
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_charging_mode);
#elif defined(CONFIG_Z300M)
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_ADC_SW_EN);
#endif
		return 0;
}

struct platform_device bq25896_user_space_device = {
		.name   = "bq25896-user",
		.id     = -1,
};

static struct platform_driver bq25896_user_space_driver = {
		.probe      = bq25896_user_space_probe,
		.driver     = {
				.name = "bq25896-user",
		},
};


extern int g_vcdt_irq;
int g_projector_on = 0;

void Projector_control(bool on)
{
	if (on)
	{
		disable_irq(g_vcdt_irq);
		bq25896_PG_OTG_EN_SOC(0);
		g_otg_enable = 0;
		g_projector_on = 1;
		if(!VBUS_IN()){
			BMT_status.old_charging_mode = AC;
			BMT_status.charger_type = CHARGER_UNKNOWN;
		}
		else{
			BMT_status.old_charging_mode = PAD;
			BMT_status.charger_type = STANDARD_CHARGER;
		}
	}
	else
	{
		enable_irq(g_vcdt_irq);
		BMT_status.charger_type = CHARGER_UNKNOWN;
		g_projector_on =0;
	}
}
EXPORT_SYMBOL(Projector_control);

void bq25896_PG_OTG_EN_SOC(int val)
{
#if defined(CONFIG_Z380M)
	battery_log(BAT_LOG_CRTI, "<BATT>[%s]\n", __func__);
	if(val)	{
		cancel_delayed_work(&charger_gpio->otg_cb81_work);
		Set_VGP2_PMU(1);
		if (_IS_CA81_()){
			BMT_status.old_charging_mode = COVER_OTG;
			bq25896_set_boost_lim(0x6);	// 1. Set Boost current limit = 2150 mA
			bq25896_set_boostv(0xA);	// 2. Boost Voltage = 5.19 V
			bq25896_set_otg_config(0x1);	// 3. Enable OTG
			gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, val);
			charger_set_gpio(0, 0, 0, 0);
		}
		else if (_IS_CB81_()){
			BMT_status.old_charging_mode = COVER_OTG;
			charger_set_gpio(0, 0, 1, 0);
			disable_cover_otg();

			/* Delay 1 sec */
			msleep_interruptible(1000);

			/* PAD OTG Enable*/
			bq25896_set_boost_lim(0x6);	// 1. Set Boost current limit = 2150 mA
			bq25896_set_boostv(0xA);	// 2. Boost Voltage = 5.19 V
			bq25896_set_otg_config(1);	// 3. Enable OTG
			gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, 1);
		}
		else{
			charger_set_gpio(0, 0, 1, 0);
			bq25896_set_boost_lim(0x6);	// 1. Set Boost current limit = 2150 mA
			bq25896_set_boostv(0xA);	// 2. Boost Voltage = 5.19 V
			bq25896_set_otg_config(0x1);	// 3. Enable OTG
			gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, val);
		}
		queue_delayed_work(otg_wq, &charger_gpio->otg_cb81_work, msecs_to_jiffies(0));
	}
	else {
		cancel_delayed_work(&charger_gpio->otg_cb81_work);
		bq25896_set_otg_config(0x0);	// 1. Disable OTG
		gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, val);
		bq25896_set_en_ilim(0x1);	// 3. enable current limit
		if(_IS_CA81_() || _IS_CB81_())
			call_cover_interupt();
		else{
			BMT_status.charger_type = CHARGER_UNKNOWN;
			BMT_status.old_charging_mode = PAD;
		}
		Projector_control(0);
	}	
#elif defined(CONFIG_Z300M)
	battery_log(BAT_LOG_CRTI, "<BATT>[%s]\n", __func__);
	if (val){
		charger_set_gpio(0, 0, 1, 0);
		Set_VIBR_PMU(1);
		msleep_interruptible(1000);	// delay 1 s
		bq25896_set_boost_lim(0x6);	// 1. Set Boost current limit = 2150 mA
		bq25896_set_boostv(0xA);	// 2. Boost Voltage = 5.19 V
		bq25896_set_otg_config(0x1);	// 2. Enable OTG
		gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, val);
		bq25896_set_wd_rst(0x1);
		cancel_delayed_work(&charger_gpio->otg_hub_work);
		queue_delayed_work(stand_wq, &charger_gpio->otg_hub_work, msecs_to_jiffies(60000));
	}
	else{
		bq25896_set_otg_config(0x0);	// 1. Disable OTG
                gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, val);
		bq25896_set_en_ilim(0x1);	// 3. enable current limit
		cancel_delayed_work(&charger_gpio->otg_hub_work);
		charging_pad_or_standac();
	}
#endif
}


/* when overcurrent, it sent interrupt again and again, we would disable OTG, and turn on later*/
void otg_workaround(void)
{
	bq25896_set_otg_config(0x0);	// 1. diable OTG
	bq25896_set_boost_lim(0x6);     // 2. Set Boost current limit = 2150 mA
	msleep_interruptible(20);	// 3. delay 20 ms
	bq25896_set_otg_config(0x1);	// 4. enable OTG
}
EXPORT_SYMBOL_GPL(otg_workaround);

void charger_get_gpio_info(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "ti,bq25896");
#if defined(CONFIG_Z380M)
	// add by leo ++
	charger_gpio->I2C_POWER_EN = of_get_named_gpio(node, "i2c_power_gpio", 0);
	battery_log(BAT_LOG_CRTI, "I2C_POWER_EN = %d\n", charger_gpio->I2C_POWER_EN);
	// add by leo --
#endif
	charger_gpio->PG_OTG_EN_SOC = of_get_named_gpio(node, "pg_otg_en_soc_gpio", 0);
	battery_log(BAT_LOG_CRTI, "PG_OTG_EN_SOC = %d\n", charger_gpio->PG_OTG_EN_SOC);

	charger_gpio->Cover_BUS_on = of_get_named_gpio(node, "cover_bus_on_gpio", 0);
	battery_log(BAT_LOG_CRTI, "Cover_BUS_on = %d\n", charger_gpio->Cover_BUS_on);

	charger_gpio->USB_BUS_on_NUM = of_get_named_gpio(node, "usb_bus_on_num_gpio", 0);
	battery_log(BAT_LOG_CRTI, "USB_BUS_on_NUM = %d\n", charger_gpio->USB_BUS_on_NUM);

	charger_gpio->Cover_BUS_off = of_get_named_gpio(node, "cover_bus_off_gpio", 0);
	battery_log(BAT_LOG_CRTI, "Cover_BUS_off = %d\n", charger_gpio->Cover_BUS_off);

#if defined(CONFIG_Z300M)	
	charger_gpio->DCIN_VBUS_IN_DET_N = of_get_named_gpio(node, "dcin_vbus_in_det_n", 0);
	battery_log(BAT_LOG_CRTI, " DCIN_VBUS_IN_DET_N = %d\n", charger_gpio->DCIN_VBUS_IN_DET_N);

	charger_gpio->ADC_SW_EN = of_get_named_gpio(node, "adc_sw_en_gpio", 0);
	battery_log(BAT_LOG_CRTI, "ADC_SW_EN = %d\n", charger_gpio->ADC_SW_EN);

	charger_gpio->PAD_QB_DET = of_get_named_gpio(node, "pad_qb_det_gpio", 0);
        battery_log(BAT_LOG_CRTI, " PAD_QB_DET = %d\n", charger_gpio->PAD_QB_DET);

	charger_gpio->N1_FUNCTION_NUN = of_get_named_gpio(node, "n1_function_num", 0);
	battery_log(BAT_LOG_CRTI, " N1_FUNCTION_NUN = %d\n", charger_gpio->N1_FUNCTION_NUN);

	charger_gpio->N1_VBUS_IN_DET_NUM = of_get_named_gpio(node, "n1_vbus_in_det_num", 0);
	battery_log(BAT_LOG_CRTI, " N1_VBUS_IN_DET_NUM = %d\n", charger_gpio->N1_VBUS_IN_DET_NUM);
#endif


}

void charger_gpio_request(void)
{
	int err = 0;
#if defined(CONFIG_Z380M)
	// add by leo ++
   	battery_log(BAT_LOG_CRTI, "%s: enable touch power\n", __func__);
	err = gpio_request(charger_gpio->I2C_POWER_EN, "I2C_POWER_EN");
	if(err < 0)
		printk("%s: gpio_request failed for charger_gpio->I2C_POWER_EN = %d\n", __func__, charger_gpio->I2C_POWER_EN);
	else
		gpio_direction_output(charger_gpio->I2C_POWER_EN, 1);
	// add by leo --
#endif
	err = gpio_request(charger_gpio->PG_OTG_EN_SOC, "CHARGER_PG_OTG_EN_SOC");
	if (err<0)
   		battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio PG_OTG_EN_SOC = %d\n", __func__, charger_gpio->PG_OTG_EN_SOC);
	else
    		gpio_direction_output(charger_gpio->PG_OTG_EN_SOC, 0);
 
        err = gpio_request(charger_gpio->Cover_BUS_on, "CHARGER_Cover_BUS_on");
	if (err<0)
   		battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio Cover_BUS_on = %d\n", __func__, charger_gpio->Cover_BUS_on);
	else
    		gpio_direction_output(charger_gpio->Cover_BUS_on, 0);

        err = gpio_request(charger_gpio->USB_BUS_on_NUM, "CHARGER_USB_BUS_on#");
	if (err<0)
   		battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio USB_BUS_on_NUM = %d\n", __func__, charger_gpio->USB_BUS_on_NUM);
	else
    		gpio_direction_output(charger_gpio->USB_BUS_on_NUM, 0);
 
        err = gpio_request(charger_gpio->Cover_BUS_off, "CHARGER_Cover_BUS_off");
	if (err<0)
   		battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio Cover_BUS_off = %d\n", __func__, charger_gpio->Cover_BUS_off);
	else
    		gpio_direction_output(charger_gpio->Cover_BUS_off, 0);
#if 0
	err = gpio_request(charger_gpio->DCIN_VBUS_IN_DET_N, "CHARGER_DCIN_VBUS_IN_DET_N");
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio DCIN_VBUS_IN_DET_N = %d\n", __func__, charger_gpio->DCIN_VBUS_IN_DET_N);
        else
                gpio_direction_input(charger_gpio->DCIN_VBUS_IN_DET_N);
#endif
#ifdef CONFIG_Z300M
	err = gpio_request(charger_gpio->ADC_SW_EN, "ADC_SW_EN");
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio ADC_SW_EN = %d\n", __func__, charger_gpio->ADC_SW_EN);
        else
                gpio_direction_output(charger_gpio->ADC_SW_EN, 0);

	err = gpio_request(charger_gpio->PAD_QB_DET, "CHARGER_PAD_QB_DET");
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio PAD_QB_DET = %d\n", __func__, charger_gpio->PAD_QB_DET);
        else
                gpio_direction_output(charger_gpio->PAD_QB_DET, 0);

	err = gpio_request(charger_gpio->N1_FUNCTION_NUN, "CHARGER_N1_FUNCTION#");
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio N1_FUNCTION_NUN = %d\n", __func__, charger_gpio->N1_FUNCTION_NUN);
        else
                gpio_direction_input(charger_gpio->N1_FUNCTION_NUN);

	err = gpio_request(charger_gpio->N1_VBUS_IN_DET_NUM, "CHARGER_N1_VBUS_IN_DET#");
        if (err<0)
                battery_log(BAT_LOG_CRTI, "%s: gpio_request failed for gpio N1_VBUS_IN_DET_NUM = %d\n", __func__, charger_gpio->N1_VBUS_IN_DET_NUM);
        else
                gpio_direction_input(charger_gpio->N1_VBUS_IN_DET_NUM);
#endif
}

void charger_set_gpio(int gpio_usbbusonnum, int gpio_coverbuson, int gpio_coverbusoff, int gpio_padqbdet) 
{
        gpio_direction_output(charger_gpio->USB_BUS_on_NUM, gpio_usbbusonnum);
        gpio_direction_output(charger_gpio->Cover_BUS_on, gpio_coverbuson);
        gpio_direction_output(charger_gpio->Cover_BUS_off, gpio_coverbusoff);
#if defined(CONFIG_Z380M)
	battery_log(BAT_LOG_CRTI, "[%s] ( %d, %d, %d)\n", __func__, gpio_get_value(charger_gpio->USB_BUS_on_NUM),gpio_get_value(charger_gpio->Cover_BUS_on),gpio_get_value(charger_gpio->Cover_BUS_off));
#endif
#ifdef CONFIG_Z300M
	gpio_direction_output(charger_gpio->PAD_QB_DET, gpio_padqbdet);
	battery_log(BAT_LOG_CRTI, "%s: ( %d, %d, %d, %d)\n", __func__, gpio_get_value(charger_gpio->USB_BUS_on_NUM),gpio_get_value(charger_gpio->Cover_BUS_on),gpio_get_value(charger_gpio->Cover_BUS_off),gpio_get_value(charger_gpio->PAD_QB_DET));
#endif
}
EXPORT_SYMBOL(charger_set_gpio);
#ifdef CONFIG_Z300M
void charger_set_ADC_SW_EN_gpio(int value)
{
	battery_log(BAT_LOG_CRTI, "%s: \n ADC_SW_EN = %d\n", __func__, value);
	if (value==0)
		b_adc_sw_en = false;
	else
		b_adc_sw_en = true;
	gpio_direction_output(charger_gpio->ADC_SW_EN, value);
}
#endif
//void charger_set_PAD_QB_DET(int val){
//       battery_log(BAT_LOG_CRTI, "%s: \n pad_qb_det# = %d\n", __func__, val);
//       gpio_direction_output(PAD_QB_DET, val);
//       battery_log(BAT_LOG_CRTI, "%s: \n USB_BUS_on_NUM = %d\n", __func__ ,gpio_get_value(PAD_QB_DET));
//}

static int __init bq25896_init(void)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "[bq25896_init] init start\n");

#ifndef CONFIG_OF
	i2c_register_board_info(BQ25896_BUSNUM, &i2c_bq25896, 1);
#endif

	if (i2c_add_driver(&bq25896_driver) != 0)
		battery_log(BAT_LOG_CRTI, "[bq25896_init] failed to register bq25896 i2c driver.\n");
	else
		battery_log(BAT_LOG_CRTI, "[bq25896_init] Success to register bq25896 i2c driver.\n");

	/*bq25896 user space access interface*/
	ret = platform_device_register(&bq25896_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq25896_init] Unable to device register(%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&bq25896_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq25896_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit bq25896_exit(void)
{
		i2c_del_driver(&bq25896_driver);
}
//late_initcall(bq25896_init);
module_init(bq25896_init);
module_exit(bq25896_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq25896 Driver");
MODULE_AUTHOR("Shilun Huang<shilun_huang@asus.com>");
