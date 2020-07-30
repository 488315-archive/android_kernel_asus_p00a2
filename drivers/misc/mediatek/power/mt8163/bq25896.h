/*****************************************************************************
*
* Filename:
* ---------
*   bq25896.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   bq25896 header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _bq25896_SW_H_
#define _bq25896_SW_H_

#define HIGH_BATTERY_VOLTAGE_SUPPORT

#define bq25896_CON0      0x00
#define bq25896_CON1      0x01
#define bq25896_CON2      0x02
#define bq25896_CON3      0x03
#define bq25896_CON4      0x04
#define bq25896_CON5      0x05
#define bq25896_CON6      0x06
#define bq25896_CON7      0x07
#define bq25896_CON8      0x08
#define bq25896_CON9      0x09
#define bq25896_CON10     0x0A
#define bq25896_CON11     0x0B
#define bq25896_CON12     0x0C
#define bq25896_CON13     0x0D
#define bq25896_CON14     0x0E
#define bq25896_CON15     0x0F
#define bq25896_CON16     0x10
#define bq25896_CON17     0x11
#define bq25896_CON18     0x12
#define bq25896_CON19     0x13
#define bq25896_CON20     0x14

/**********************************************************
  *
  *   [MASK/SHIFT]
  *
  *********************************************************/
/*CON0*/
#define CON0_EN_HIZ_MASK         0x01
#define CON0_EN_HIZ_SHIFT        7

#define CON0_EN_ILIM_MASK        0x01
#define CON0_EN_ILIM_SHIFT       6 

#define CON0_IINLIM_MASK         0x3F
#define CON0_IINLIM_SHIFT        0

/*CON1*/
#define CON1_BHOT_MASK           0x3
#define CON1_BHOT_SHIFT          6	 

#define CON1_BCOLD_MASK          0x01
#define CON1_BCOLD_SHIFT         5

#define CON1_VINDPM_OS_MASK      0x1F
#define CON1_VINDPM_OS_SHIFT     0

/*CON2*/
#define CON2_CONV_START_MASK     0x01
#define CON2_CONV_START_SHIFT    7

#define CON2_CONV_RATE_MASK      0x01
#define CON2_CONV_RATE_SHIFT     6

#define CON2_BOOST_FREQ_MASK     0x01
#define CON2_BOOST_FREQ_SHIFT    5

#define CON2_ICO_EN_MASK         0x01
#define CON2_ICO_EN_SHIFT        4

#define CON2_FORCE_DPDM_MASK     0x01
#define CON2_FORCE_DPDM_SHIFT    1

#define CON2_AUTO_DPDM_EN_MASK   0x01
#define CON2_AUTO_DPDM_EN_SHIFT  0

/*CON3*/
#define CON3_BAT_LOADEN_MASK     0x01
#define CON3_BAT_LOADEN_SHIFT    7

#define CON3_WD_RST_MASK         0x01
#define CON3_WD_RST_SHIFT        6

#define CON3_OTG_CONFIG_MASK     0x01
#define CON3_OTG_CONFIG_SHIFT    5

#define CON3_CHG_CONFIG_MASK     0x01
#define CON3_CHG_CONFIG_SHIFT    4

#define CON3_SYS_MIN_MASK        0x07
#define CON3_SYS_MIN_SHIFT       1

#define CON3_MIN_VBAT_SEL_MASK   0x01
#define CON3_MIN_VBAT_SEL_SHIFT  0

/*CON4*/
#define CON4_EN_PUMPX_MASK       0x01
#define CON4_EN_PUMPX_VREG_SHIFT 7

#define CON4_ICHG_MASK           0x7F
#define CON4_ICHG_SHIFT          0

/*CON5*/
#define CON5_IPRECHG_MASK        0x0F
#define CON5_IPRECHG_SHIFT       4
 
#define CON5_ITERM_MASK          0x0F
#define CON5_ITERM_SHIFT         0

/*CON6*/
#define CON6_VREG_MASK           0x3F
#define CON6_VREG_SHIFT          2

#define CON6_BATLOWV_MASK        0x01
#define CON6_BATLOWV_SHIFT       1

#define CON6_VRECHG_MASK         0x01
#define CON6_VRECHG_SHIFT        0

/*CON7*/
#define CON7_EN_TERM_MASK        0x01
#define CON7_EN_TERM_SHIFT       7

#define CON7_STAT_DIS_MASK       0x01
#define CON7_STAT_DIS_SHIFT      6

#define CON7_WATCHDOG_MASK       0x03
#define CON7_WATCHDOG_SHIFT      4

#define CON7_EN_TIMER_MASK       0x01
#define CON7_EN_TIMER_SHIFT      3

#define CON7_CHG_TIMER_MASK      0x03
#define CON7_CHG_TIMER_SHIFT     1

#define CON7_JEITA_ISET_MASK     0x01
#define CON7_JEITA_ISET_SHIFT    0

/*CON8*/
#define CON8_BAT_COMP_MASK       0x07
#define CON8_BAT_COMP_SHIFT      5

#define CON8_VCLAMP_MASK         0x07
#define CON8_VCLAMP_SHIFT        2 

#define CON8_TREG_MASK           0x03
#define CON8_TREG_SHIFT          0

/*CON9*/
#define CON9_FORCE_ICO_MASK      0x01
#define CON9_FORCE_ICO_SHIFT     7

#define CON9_TRM2X_EN_MASK       0x01
#define CON9_TRM2X_EN_SHIFT      6

#define CON9_BATFET_DIS_MASK     0x01
#define CON9_BATFET_DIS_SHIFT    5

#define CON9_JEITA_VSET_MASK     0x01
#define CON9_JEITA_VSET_SHIFT    4

#define CON9_BATFET_DLY_MASK     0x01
#define CON9_BATFET_DLY_SHIFT    3

#define CON9_BATFET_RST_EN_MASK  0x01
#define CON9_BATFET_RST_EN_SHIFT 2

#define CON9_PUMPX_UP_MASK       0x01
#define CON9_PUMPX_UP_SHIFT      1

#define CON9_PUMPX_DN_MASK       0x01
#define CON9_PUMPX_DN_SHIFT      0

/*CON10*/
#define CON10_BOOSTV_MASK        0x0F
#define CON10_BOOSTV_SHIFT       4

#define CON10_PFM_OTG_DIS_MASK   0x01
#define CON10_PFM_OTG_DIS_SHIFT  3

#define CON10_BOOST_LIM_MASK     0x07
#define CON10_BOOST_LIM_SHIFT    0

/*CON11*/
#define CON11_VBUS_STAT_MASK     0x07
#define CON11_VBUS_STAT_SHIFT    5

#define CON11_CHRG_STAT_MASK     0x03
#define CON11_CHRG_STAT_SHIFT    3

#define CON11_PG_STAT_MASK       0x01
#define CON11_PG_STAT_SHIFT      2

#define CON11_VSYS_STAT_MASK     0x01
#define CON11_VSYS_STAT_SHIFT    0

/*CON12*/
#define CON12_WATCHDOG_FAULT_MASK  0x01
#define CON12_WATCHDOG_FAULT_SHIFT 7

#define CON12_BOOST_FAULT_MASK   0x01
#define CON12_BOOST_FAULT_SHIFT  6

#define CON12_CHRG_FAULT_MASK    0x03
#define CON12_CHRG_FAULT_SHIFT   4

#define CON12_BAT_FAULT_MASK     0x01
#define CON12_BAT_FAULT_SHIFT    3

#define CON12_NTC_FAULT_MASK     0x07
#define CON12_NTC_FAULT_SHIFT    0

/*CON13*/
#define CON13_FORCE_VINDPM_MASK  0x01
#define CON13_FORCE_VINSPM_SHIFT 7

#define CON13_VINDPM_MASK        0x7F
#define CON13_VINDPM_SHIFT       0

/*CON14*/
#define CON14_THERM_STAT_MASK    0x01
#define CON14_THERM_STAT_SHIFT   7
 
#define CON14_BATV_MASK          0x7F
#define CON14_BATV_SHIFT         0

/*CON15*/
#define CON15_SYSV_MASK          0x7F              
#define CON15_SYSV_SHIFT         0

/*CON16*/
#define CON16_TSPCT_MASK         0x7F
#define CON16_TSPCT_SHIFT        0

/*CON17*/
#define CON17_VBUS_GD_MASK       0x01
#define CON17_VBUS_GD_SHIFT      7

#define CON17__MASK              0x7F
#define CON17__SHIFT             0

/*CON18*/
#define CON18_ICHGR_MASK         0x7F
#define CON18_ICHGR_SHIFT        0

/*CON19*/
#define CON19_VDPM_STAT_MASK     0x01
#define CON19_VDPM_STAT_SHIFT    7

#define CON19_IDPM_STAT_MASK     0x01 
#define CON19_IDPM_STAT_SHIFT    6 

#define CON19_IDPM_LIM_MASK      0x3F
#define CON19_IDPM_LIM_SHIFT     0

/*CON20*/
#define CON20_REG_RST_MASK       0x01
#define CON20_REG_RST_SHIFT      7

#define CON20_ICO_OPTIMIZED_MASK  0x01
#define CON20_ICO_OPTIMIZED_SHIFT 6

#define CON20_PN_MASK            0x07
#define CON20_PN_SHIFT           3

#define CON20_TS_PROFILE_MASK    0x01
#define CON20_TS_PROFILE_SHIFT   2

#define CON20_DEV_REV_MASK       0x03
#define CON20_DEV_REV_SHIFT      0


/**********************************************************
  *
  *[Extern Function]
  *
  *********************************************************/
/*CON0--00--------------------------------------------------*/
extern void bq25896_set_en_hiz(unsigned int val);
extern void bq25896_set_en_ilim(unsigned int val);
extern void bq25896_set_iinlim(unsigned int val);
/*CON1--01--------------------------------------------------*/
extern void bq25896_set_bhot(unsigned int val);
extern void bq25896_set_bcold(unsigned int val);
extern void bq25896_set_vindpm_os(unsigned int val);
/*CON2--02--------------------------------------------------*/
extern void bq25896_set_conv_start(unsigned int val);
extern void bq25896_set_conv_rate(unsigned int val);
extern void bq25896_set_boost_freq(unsigned int val);
extern void bq25896_set_ico_en(unsigned int val);
extern void bq25896_set_force_dpmp(unsigned int val);
extern void bq25896_set_auto_dpmp_en(unsigned int val);
/*CON3--03--------------------------------------------------*/
extern void bq25896_set_bat_loaden(unsigned int val);
extern void bq25896_set_wd_rst(unsigned int val);
extern void bq25896_set_otg_config(unsigned int val);
extern void bq25896_set_chg_config(unsigned int val);
extern void bq25896_set_sys_min(unsigned int val);
extern void bq25896_set_min_vbat_sel(unsigned int val);
/*CON4--04--------------------------------------------------*/
extern void bq25896_set_en_pumpx(unsigned int val);
extern void bq25896_set_ichg(unsigned int val);
/*CON5--05--------------------------------------------------*/
extern void bq25896_set_iprechg(unsigned int val);
extern void bq25896_set_iterm(unsigned int val);
/*CON6--06--------------------------------------------------*/
extern void bq25896_set_vreg(unsigned int val);
extern unsigned int bq25896_get_vreg(void);
extern void bq25896_set_batlowv(unsigned int val);
extern void bq25896_set_vrechg(unsigned int val);
/*CON7--07--------------------------------------------------*/
extern void bq25896_set_en_term(unsigned int val);
extern void bq25896_set_stat_dis(unsigned int val);
extern void bq25896_set_watchdog(unsigned int val);
extern void bq25896_set_en_timer(unsigned int val);
extern void bq25896_set_chg_timer(unsigned int val);
extern void bq25896_set_jeita_iset(unsigned int val);
/*CON8--08--------------------------------------------------*/
extern void bq25896_set_bat_comp(unsigned int val);
extern void bq25896_set_vclamp(unsigned int val);
extern void bq25896_set_treg(unsigned int val);
/*CON9--09--------------------------------------------------*/
extern void bq25896_set_force_ico(unsigned int val);
extern void bq25896_set_tmr2x_en(unsigned int val);
extern void bq25896_set_batfet_disable(unsigned int val);
extern void bq25896_set_jeita_vset(unsigned int val);
extern void bq25896_set_batfet_ddlay(unsigned int val);
extern void bq25896_set_batfet_rst_en(unsigned int val);
extern void bq25896_set_pumpx_up(unsigned int val);
extern void bq25896_set_pumpx_dn(unsigned int val);
/*CON10--0A--------------------------------------------------*/
extern void bq25896_set_boostv(unsigned int val);
extern void bq25896_set_pfm_otg_dis(unsigned int val);
extern void bq25896_set_boost_lim(unsigned int val);
/*CON11--0B--------------------------------------------------*/
extern unsigned int bq25896_get_vbus_stat(void);
extern unsigned int bq25896_get_chrg_stat(void);
extern unsigned int bq25896_get_pg_stat(void);
extern unsigned int bq25896_get_vsys_stat(void);
/*CON12--0C--------------------------------------------------*/
extern unsigned int bq25896_get_watchdog_fault(void);
extern unsigned int bq25896_get_boost_fault(void);
extern unsigned int bq25896_get_chrg_fault(void);
extern unsigned int bq25896_get_bat_fault(void);
extern unsigned int bq25896_get_ntc_fault(void);
/*CON13--0D--------------------------------------------------*/
extern void bq25896_set_force_vindpm(unsigned int val);
extern void bq25896_set_vindpm(unsigned int val);
/*CON14--0E--------------------------------------------------*/
extern unsigned int bq25896_get_therm_stat(void);
extern unsigned int bq25896_get_batv(void);
/*CON15--0F--------------------------------------------------*/
extern unsigned int bq25896_get_sysv(void);
/*CON16--10--------------------------------------------------*/
extern unsigned int bq25896_get_tspct(void);
/*CON17--11--------------------------------------------------*/
extern unsigned int bq25896_get_vbus_gd(void);
extern unsigned int bq25896_get_vbusv(void);
/*CON18--12--------------------------------------------------*/
extern unsigned int bq25896_get_ichgr(void);
/*CON19--13--------------------------------------------------*/
extern unsigned int bq25896_get_vdpm_stat(void);
extern unsigned int bq25896_get_idpm_stat(void);
extern unsigned int bq25896_get_idpm_lim(void);
/*CON20--14--------------------------------------------------*/
extern void bq25896_set_reg_rst(unsigned int val);
extern void bq25896_set_ico_optimized(unsigned int val);
extern unsigned int bq25896_get_pn(void);
extern unsigned int bq25896_get_ts_profile(void);
extern unsigned int bq25896_get_dev_rev(void);
/*---------------------------------------------------------*/
extern void bq25896_dump_register(void);
extern unsigned int bq25896_read_interface(unsigned char RegNum, unsigned char *val,
	unsigned char MASK, unsigned char SHIFT);
extern void charger_get_gpio_info(void);
extern void charger_gpio_request(void);
extern void bq25896_PG_OTG_EN_SOC(int val);
extern void charger_set_gpio(int gpio_usbbusonnum, int gpio_coverbuson, int gpio_coverbusoff, int gpio_padqbdet);
#ifdef CONFIG_Z380M
extern void Set_VGP2_PMU(int Enable);
#endif
#ifdef CONFIG_Z300M
extern void charger_set_ADC_SW_EN_gpio(int value);
extern void Set_VIBR_PMU(int Enable);
//extern void charger_set_PAD_QB_DET(int val);
#endif
#endif /* _bq25896_SW_H_*/

