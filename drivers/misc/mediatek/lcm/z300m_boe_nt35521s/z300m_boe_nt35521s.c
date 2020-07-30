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

static struct regulator *lcm_vgp;
static unsigned int GPIO_LCM_EN;
static unsigned int GPIO_LM_LCM_RST;
static unsigned int GPIO_PW_BL_EN;
static unsigned int GPIO_LM_ID0;
static unsigned int GPIO_LM_ID1;

extern char *lk_lcmname;

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

//#define BOE_LCM_ID2 0x0
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
	{0xFF, 4, {0xAA,0x55,0x25,0x01}},
	{0xFC, 1, {0x08}},
	{REGFLAG_DELAY, 1, {}},
	{0xFC, 1, {0x00}},
	{0xFF, 4, {0xAA,0x55,0x25,0x00}},
	{REGFLAG_DELAY, 20, {}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},
	{0xB1, 2, {0x68,0x01}},
	{0xB8, 4, {0x01,0x02,0x02,0x02}},
	{0xB6, 1, {0x11}},
	{0xBB, 2, {0x63,0x63}},
	{0xBC, 2, {0x00,0x00}},
	{0xBD, 5, {0x02,0xBB,0x11,0x10,0x00}},
	{0xC8, 1, {0x80}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x01}},
	{0xB3, 2, {0x28,0x28}},
	{0xB4, 2, {0x14,0x14}},
	{0xB9, 2, {0x44,0x44}},
	{0xBA, 2, {0x24,0x24}},
	{0xBC, 2, {0x70,0x00}},
	{0xBD, 2, {0x70,0x00}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x02}},
	{0xEE, 1, {0x01}},
	{0xB0, 16, {0x00,0x16,0x00,0x26,0x00,0x45,0x00,0x5F,0x00,0x72,0x00,0x96,0x00,0xB3,0x00,0xE3}},
	{0xB1, 16, {0x01,0x0A,0x01,0x47,0x01,0x77,0x01,0xC3,0x01,0xFF,0x02,0x00,0x02,0x36,0x02,0x70}},
	{0xB2, 16, {0x02,0x93,0x02,0xC8,0x02,0xEB,0x03,0x19,0x03,0x37,0x03,0x5D,0x03,0x71,0x03,0x8D}},
	{0xB3, 4, {0x03,0xBA,0x03,0xD0}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x03}},
	{0xB0, 2, {0x00,0x00}},
	{0xB1, 2, {0x00,0x00}},
	{0xB2, 5, {0x05,0x01,0x55,0x00,0x1A}},
	{0xB3, 5, {0x05,0x01,0x55,0x00,0x1A}},
	{0xBA, 5, {0x53,0x01,0x55,0x00,0x3A}},
	{0xC4, 1, {0x60}},
	{0xC5, 1, {0x40}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x04}},
	{0xB1, 4, {0x03,0x02,0x05,0x07}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x05}},
	{0xBD, 5, {0x01,0x03,0x00,0x00,0x03}},
	{0xC0, 1, {0x09}},
	{0xC1, 1, {0x07}},
	{0xC8, 2, {0x07,0x38}},
	{0xD0, 1, {0x00}},
	{0xD1, 5, {0x00,0x05,0x01,0x00,0x00}},
	{0xE5, 1, {0x03}},
	{0xE6, 1, {0x03}},
	{0xE9, 1, {0x03}},
	{0xEA, 1, {0x03}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x06}},
	{0xB0, 2, {0x2D,0x2D}},
	{0xB1, 2, {0x01,0x01}},
	{0xB2, 2, {0x03,0x03}},
	{0xB3, 2, {0x2E,0x2E}},
	{0xB4, 2, {0x2E,0x2D}},
	{0xB5, 2, {0x2D,0x13}},
	{0xB6, 2, {0x13,0x11}},
	{0xB7, 2, {0x11,0x36}},
	{0xB8, 2, {0x36,0x15}},
	{0xB9, 2, {0x15,0x2E}},
	{0xBA, 2, {0x2E,0x14}},
	{0xBB, 2, {0x14,0x35}},
	{0xBC, 2, {0x35,0x10}},
	{0xBD, 2, {0x10,0x12}},
	{0xBE, 2, {0x12,0x2D}},
	{0xBF, 2, {0x2D,0x2E}},
	{0xC0, 2, {0x2E,0x2E}},
	{0xC1, 2, {0x02,0x02}},
	{0xC2, 2, {0x00,0x00}},
	{0xC3, 2, {0x2D,0x2D}},
	{0xE5, 2, {0x34,0x34}},
	{0xC4, 2, {0x2E,0x2E}},
	{0xC5, 2, {0x02,0x02}},
	{0xC6, 2, {0x00,0x00}},
	{0xC7, 2, {0x2D,0x2D}},
	{0xC8, 2, {0x2E,0x2D}},
	{0xC9, 2, {0x2D,0x14}},
	{0xCA, 2, {0x14,0x35}},
	{0xCB, 2, {0x35,0x10}},
	{0xCC, 2, {0x10,0x12}},
	{0xCD, 2, {0x12,0x2E}},
	{0xCE, 2, {0x2E,0x13}},
	{0xCF, 2, {0x13,0x11}},
	{0xD0, 2, {0x11,0x36}},
	{0xD1, 2, {0x36,0x15}},
	{0xD2, 2, {0x15,0x2D}},
	{0xD3, 2, {0x2D,0x2E}},
	{0xD4, 2, {0x2D,0x2D}},
	{0xD5, 2, {0x01,0x01}},
	{0xD6, 2, {0x03,0x03}},
	{0xD7, 2, {0x2E,0x2E}},
	{0xE6, 2, {0x34,0x34}},
	{0xD8, 5, {0x00,0x00,0x00,0x00,0x00}},
	{0xD9, 5, {0x00,0x00,0x00,0x00,0x00}},
	{0xF0, 5, {0x55,0xAA,0x52,0x00,0x00}},
	{0x35, 1, {0x00}},
	{0x11, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 100, {}},
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

	GPIO_LCM_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	ret = gpio_request(GPIO_LCM_EN, "LCM_EN");
	if (ret)
		pr_err("[DISPLAY] LCM_EN pin, failure of setting\n");

	GPIO_LM_LCM_RST = of_get_named_gpio(node, "lcm_reset_gpio", 0);
	ret = gpio_request(GPIO_LM_LCM_RST, "LM_LCM_RST");
	if (ret)
		pr_err("[DISPLAY] LM_LCM_RST pin, failure of setting\n");

	GPIO_PW_BL_EN = of_get_named_gpio(node, "lcm_bl_gpio", 0);
	ret = gpio_request(GPIO_PW_BL_EN, "PW_BL_EN");
	if (ret)
		pr_err("[DISPLAY] PW_BL_EN pin, failure of setting\n");

	GPIO_LM_ID0 = of_get_named_gpio(node, "lm_id0_gpio", 0);
	ret = gpio_request(GPIO_LM_ID0, "LM_ID0");
	if (ret)
		pr_err("[DISPLAY] LM_ID0 pin, failure of setting\n");

	GPIO_LM_ID1 = of_get_named_gpio(node, "lm_id1_gpio", 0);
	ret = gpio_request(GPIO_LM_ID1, "LM_ID1");
	if (ret)
		pr_err("[DISPLAY] LM_ID1 pin, failure of setting\n");

}

int lcm2_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_debug("LCM: lcm2_vgp_supply_enable\n");

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

int lcm2_vgp_supply_disable(void)
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
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00033902;
	data_array[1] = 0x005a5af0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x280040c3;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);

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

	lcm_set_gpio_output(GPIO_LM_LCM_RST, 0);
	MDELAY(1);

	lcm_set_gpio_output(GPIO_LCM_EN, 0);
	MDELAY(1);

	lcm2_vgp_supply_disable();
	MDELAY(1);
}

static void lcm_resume_power(void)
{
	pr_debug("[Kernel/LCM] lcm_resume_power() enter\n");

	lcm2_vgp_supply_enable();
	MDELAY(1);

	lcm_set_gpio_output(GPIO_LCM_EN, 1);
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

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 16;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 40;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	//params->dsi.ssc_disable = 1;
	params->dsi.PLL_CLOCK = 210;
	params->dsi.cont_clock = 0;

	params->physical_width = 135;
	params->physical_height =  216;
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

	lcm_set_gpio_output(GPIO_PW_BL_EN, 0);
	//MDELAY(20);

#ifdef PUSH_TABLET_USING
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
#else
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);
#endif

}

static void lcm_resume(void)
{
	pr_debug("[Kernel/LCM] lcm_resume() enter\n");
	lcm_set_gpio_output(GPIO_LM_LCM_RST, 0);
	MDELAY(15);
	lcm_set_gpio_output(GPIO_LM_LCM_RST, 1);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LM_LCM_RST, 0);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LM_LCM_RST, 1);
	MDELAY(20);

	init_lcm_registers();

	lcm_set_gpio_output(GPIO_PW_BL_EN, 1);
}

LCM_DRIVER z300m_boe_nt35521s_lcm_drv = {
	.name = "z300m_boe_nt35521s",
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

	if (!strcmp(z300m_boe_nt35521s_lcm_drv.name, lk_lcmname)) {
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
