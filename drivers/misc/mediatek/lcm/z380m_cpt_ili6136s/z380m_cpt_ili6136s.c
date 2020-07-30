#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif

#include "lcm_drv.h"
/*#include "ddp_irq.h"*/

/*static unsigned int GPIO_LCD_PWR_EN;*/
static struct regulator *lcm_vgp;
static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_RST_EN;
static unsigned int GPIO_LCD_BL_EN;
static unsigned int GPIO_LM_ID0;
static unsigned int GPIO_LM_ID1;
static unsigned int GPIO_LM_ID2;

extern int Read_HW_ID(void);
extern char *lk_lcmname;

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

//#define CPT_LCM_ID2 0x0
//#define AUO_LCM_ID2 0x1

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define PUSH_TABLET_USING
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_END_OF_TABLE	0xFFFD

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)							lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)							lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


#ifdef PUSH_TABLET_USING
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF, 3, {0x61,0x36,0x08}},
	{0x1C, 1, {0xA0}},
	{0xFF, 3, {0x61,0x36,0x08}},
	{0x4C, 1, {0x00}},
	{0xFF, 3, {0x61,0x36,0x01}},
	{0xA0, 1, {0x05}},
	{0xA1, 1, {0x05}},
	{0xA2, 1, {0x06}},
	{0xA3, 1, {0x0d}},
	{0xA4, 1, {0x04}},
	{0xA5, 1, {0x07}},
	{0xA6, 1, {0x0f}},
	{0xA7, 1, {0x14}},
	{0xA8, 1, {0x1F}},
	{0xA9, 1, {0x28}},
	{0xAA, 1, {0x31}},
	{0xAB, 1, {0x39}},
	{0xAC, 1, {0x39}},
	{0xAD, 1, {0x31}},
	{0xAE, 1, {0x2e}},
	{0xAF, 1, {0x2f}},
	{0xB0, 1, {0x3c}},
	{0xFF, 3, {0x61,0x36,0x01}},
	{0xC0, 1, {0x05}},
	{0xC1, 1, {0x05}},
	{0xC2, 1, {0x06}},
	{0xC3, 1, {0x0d}},
	{0xC4, 1, {0x04}},
	{0xC5, 1, {0x07}},
	{0xC6, 1, {0x0f}},
	{0xC7, 1, {0x14}},
	{0xC8, 1, {0x1F}},
	{0xC9, 1, {0x28}},
	{0xCA, 1, {0x31}},
	{0xCB, 1, {0x39}},
	{0xCC, 1, {0x39}},
	{0xCD, 1, {0x31}},
	{0xCE, 1, {0x2e}},
	{0xCF, 1, {0x2f}},
	{0xD0, 1, {0x3c}},
	{0xFF, 3, {0x61,0x36,0x08}},
	{0xE9, 1, {0x0B}},
	{0xFF, 3, {0x61,0x36,0x06}},
	{0x72, 1, {0x01}},
	{0xFF, 3, {0x61,0x36,0x08}},
	{0x93, 1, {0x08}},
	{0x8E, 1, {0x12}},
	{0x76, 1, {0xB4}},
	{0x78, 1, {0x02}},
	{0xFF, 3, {0x61,0x36,0x01}},
	{0x42, 1, {0x43}},
	{0x60, 1, {0x14}},
	{0xFF, 3, {0x61,0x36,0x07}},
	{0x1A, 1, {0x05}},
	{0x16, 1, {0x1F}},
	{0x17, 1, {0x1F}},
	{0x18, 1, {0x05}},
	{0x19, 1, {0x00}},
	{0x0D, 1, {0x05}},
	{0x0A, 1, {0x03}},
	{0x0E, 1, {0x35}},
	{0x0B, 1, {0x1F}},
	{0x1C, 1, {0xEB}},
	{0xFF, 3, {0x61,0x36,0x08}},
	{0x6C, 1, {0x02}},
	{0x5F, 1, {0x0F}},
	{0xAB, 1, {0x24}},
	{0xFF, 3, {0x61,0x36,0x01}},
	{0x38, 1, {0x00}},
	{0x39, 1, {0x1F}},
	{0x50, 1, {0x85}},
	{0x51, 1, {0x85}},
	{0xFF, 3, {0x61,0x36,0x00}},
	{0x35, 1, {0x01}},
	{REGFLAG_DELAY, 20, {}},
	{0x11, 0, {}},
	{REGFLAG_DELAY, 150, {}},
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 20, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 180, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY :
				if (table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}
#endif

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	pr_debug("LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

/**
 * LCM Driver Implementations
 */
static void lcm_get_gpio_infor(void)
{
	static struct device_node *node;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	ret = gpio_request(GPIO_LCD_PWR_EN, "LM_LCM_3V3_EN");
	if (ret)
		pr_err("[DISPLAY] LM_LCM_3V3_EN pin, failure of setting\n");

	GPIO_LCD_RST_EN = of_get_named_gpio(node, "lcm_reset_gpio", 0);
	ret = gpio_request(GPIO_LCD_RST_EN, "LM_LCM_RST");
	if (ret)
		pr_err("[DISPLAY] LM_LCM_RST pin, failure of setting\n");

	GPIO_LCD_BL_EN = of_get_named_gpio(node, "lcm_bl_gpio", 0);
	ret = gpio_request(GPIO_LCD_BL_EN, "PW_BL_EN");
	if (ret)
		pr_err("[DISPLAY] PW_BL_EN pin, failure of setting\n");

	if (Read_HW_ID() == 0) { //EVB
		GPIO_LM_ID0 = of_get_named_gpio(node, "evb_lm_id0_gpio", 0);
		ret = gpio_request(GPIO_LM_ID0, "LM_ID0");
		if (ret)
			pr_err("[DISPLAY] LM_ID0 pin, failure of setting\n");

		GPIO_LM_ID1 = of_get_named_gpio(node, "evb_lm_id1_gpio", 0);
		ret = gpio_request(GPIO_LM_ID1, "LM_ID1");
		if (ret)
			pr_err("[DISPLAY] LM_ID1 pin, failure of setting\n");

		GPIO_LM_ID2 = of_get_named_gpio(node, "evb_lm_id2_gpio", 0);
		ret = gpio_request(GPIO_LM_ID2, "LM_ID2");
		if (ret)
			pr_err("[DISPLAY] LM_ID2 pin, failure of setting\n");
	} else {
		GPIO_LM_ID0 = of_get_named_gpio(node, "lm_id0_gpio", 0);
		ret = gpio_request(GPIO_LM_ID0, "LM_ID0");
		if (ret)
			pr_err("[DISPLAY] LM_ID0 pin, failure of setting\n");

		GPIO_LM_ID1 = of_get_named_gpio(node, "lm_id1_gpio", 0);
		ret = gpio_request(GPIO_LM_ID1, "LM_ID1");
		if (ret)
			pr_err("[DISPLAY] LM_ID1 pin, failure of setting\n");

		GPIO_LM_ID2 = of_get_named_gpio(node, "lm_id2_gpio", 0);
		ret = gpio_request(GPIO_LM_ID2, "LM_ID2");
		if (ret)
			pr_err("[DISPLAY] LM_ID2 pin, failure of setting\n");
	}

}

int cpt_lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_debug("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp)
		return 0;

	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 1800000)
		pr_err("LCM: check regulator voltage=1800000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=1800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

int cpt_lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_debug("LCM: lcm query regulator enable status[0x%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}

	return ret;
}

static void init_lcm_registers(void)
{
#ifndef PUSH_TABLET_USING
	unsigned int data_array[16];
#endif

	pr_debug("%s, KE\n", __func__);

#ifdef PUSH_TABLET_USING
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#else
	data_array[0] = 0x00043902;
	data_array[1] = 0x083661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0xa01c1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x083661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x004c1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x013661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x05a01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05a11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06a21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0da31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04a41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07a51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0fa61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14a71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1fa81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28a91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x31aa1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39ab1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39ac1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x31ad1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2eae1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2faf1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3cb01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x013661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x05c01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05c11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06c21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0dc31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04c41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07c51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0fc61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14c71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1fc81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28c91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x31ca1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39cb1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39cc1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x31cd1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2ece1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2fcf1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3cd01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x083661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0be91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x063661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x01721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x083661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x08931500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x128e1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xb4761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x013661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x43421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x073661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x051a1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1f161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1f171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x050d1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x030a1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x350e1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1f0b1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xeb1c1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x083661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x026c1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0f5f1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x24ab1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x013661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1f391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x85501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x85511500;
	dsi_set_cmdq(data_array, 1, 1);

	//BIST
#if 0
	data_array[0] = 0x00043902;
	data_array[1] = 0x013661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x05361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x083661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x54261500;
	dsi_set_cmdq(data_array, 1, 1);
#endif

	data_array[0] = 0x00043902;
	data_array[1] = 0x003661ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x01351500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
#endif
}

static void lcm_init_power(void)
{
	pr_debug("[Kernel/LCM] lcm_init_power() enter\n");
}

static void lcm_suspend_power(void)
{
	pr_debug("[Kernel/LCM] lcm_suspend_power() enter\n");

	lcm_set_gpio_output(GPIO_LCD_RST_EN, 0);
	MDELAY(1);

	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 0);
	MDELAY(1);

	cpt_lcm_vgp_supply_disable();
	MDELAY(1);
}

static void lcm_resume_power(void)
{
	pr_debug("[Kernel/LCM] lcm_resume_power() enter\n");

	cpt_lcm_vgp_supply_enable();
	MDELAY(1);

	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
	MDELAY(1);
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode = SYNC_EVENT_VDO_MODE;

	/* DSI */
	/* Command mode setting */
	/* Three lane or Four lane */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	/* The following defined the format for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.vertical_sync_active = 6;
	params->dsi.vertical_backporch = 3;
	params->dsi.vertical_frontporch = 5;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 6;
	params->dsi.horizontal_backporch = 48;
	params->dsi.horizontal_frontporch = 16;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	//params->dsi.ssc_disable = 1;
	params->dsi.PLL_CLOCK = 221;
	params->dsi.cont_clock = 0;

	params->physical_width = 107;
	params->physical_height =  172;
}

static void lcm_init_lcm(void)
{
	pr_debug("[Kernel/LCM] lcm_init_lcm() enter\n");
}

static void lcm_suspend(void)
{
#ifndef PUSH_TABLET_USING
	unsigned int data_array[16];
#endif
	pr_debug("[Kernel/LCM] lcm_suspend() enter\n");

	lcm_set_gpio_output(GPIO_LCD_BL_EN, 0);
	//MDELAY(20);

#ifdef PUSH_TABLET_USING
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
#else
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
#endif

}

static void lcm_resume(void)
{
	pr_debug("[Kernel/LCM] lcm_resume() enter\n");
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 0);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 0);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	MDELAY(20);

	init_lcm_registers();

	lcm_set_gpio_output(GPIO_LCD_BL_EN, 1);

}

LCM_DRIVER z380m_cpt_ili6136s_lcm_drv = {
	.name = "z380m_cpt_ili6136s",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};

static int lcm_probe(struct device *dev)
{
	lcm_get_vgp_supply(dev);
	lcm_get_gpio_infor();

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "mtk_lcm",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_notice("LCM: lcm_init() enter\n");

	if (!strcmp(z380m_cpt_ili6136s_lcm_drv.name, lk_lcmname)) {
		pr_notice("LCM: Register lcm driver\n");
		if (platform_driver_register(&lcm_driver)) {
			pr_err("LCM: failed to register disp driver\n");
			return -ENODEV;
		}
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
