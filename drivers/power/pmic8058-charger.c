/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mfd/pmic8058.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/msm-charger.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/msm_adc.h>
#include <linux/notifier.h>
#include <linux/pmic8058-batt-alarm.h>
#include <linux/pmic8058-charger.h>

#include <mach/msm_xo.h>
#include <mach/msm_hsusb.h>

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#include <mach/board_lge.h>
#include "../../lge/include/lg_power_common.h"
#endif


/* Config Regs  and their bits*/
#define PM8058_CHG_TEST			0x75
#define IGNORE_LL			2
#define PM8058_CHG_TEST_2		0xEA
#define PM8058_CHG_TEST_3		0xEB
#define PM8058_OVP_TEST_REG		0xF6
#define FORCE_OVP_OFF			3

#define PM8058_CHG_CNTRL		0x1E
#define CHG_TRICKLE_EN			7
#define CHG_USB_SUSPEND			6
#define CHG_IMON_CAL			5
#define CHG_IMON_GAIN			4
#define CHG_CHARGE_BAT			3
#define CHG_VBUS_FROM_BOOST_OVRD	2
#define CHG_CHARGE_DIS			1
#define CHG_VCP_EN			0

#define PM8058_CHG_CNTRL_2		0xD8
#define ATC_DIS				7	/* coincell backed */
#define CHARGE_AUTO_DIS			6
#define DUMB_CHG_OVRD			5	/* coincell backed */
#define ENUM_DONE			4
#define CHG_TEMP_MODE			3
#define CHG_BATT_TEMP_DIS		1	/* coincell backed */
#define CHG_FAILED_CLEAR		0

#define PM8058_CHG_VMAX_SEL		0x21
#define PM8058_CHG_VBAT_DET		0xD9
#define PM8058_CHG_IMAX			0x1F
#define PM8058_CHG_TRICKLE		0xDB
#define PM8058_CHG_ITERM		0xDC
#define PM8058_CHG_TTRKL_MAX		0xE1
#define PM8058_CHG_TCHG_MAX		0xE4
#define PM8058_CHG_TEMP_THRESH		0xE2
#define PM8058_CHG_TEMP_REG		0xE3
#define PM8058_CHG_PULSE		0x22

/* IRQ STATUS and CLEAR */
#define PM8058_CHG_STATUS_CLEAR_IRQ_1	0x31
#define PM8058_CHG_STATUS_CLEAR_IRQ_3	0x33
#define PM8058_CHG_STATUS_CLEAR_IRQ_10	0xB3
#define PM8058_CHG_STATUS_CLEAR_IRQ_11	0xB4

/* IRQ MASKS */
#define PM8058_CHG_MASK_IRQ_1		0x38

#define PM8058_CHG_MASK_IRQ_3		0x3A
#define PM8058_CHG_MASK_IRQ_10		0xBA
#define PM8058_CHG_MASK_IRQ_11		0xBB

/* IRQ Real time status regs */
#define PM8058_CHG_STATUS_RT_1		0x3F
#define STATUS_RTCHGVAL			7
#define STATUS_RTCHGINVAL		6
#define STATUS_RTBATT_REPLACE		5
#define STATUS_RTVBATDET_LOW		4
#define STATUS_RTCHGILIM		3
#define STATUS_RTPCTDONE		1
#define STATUS_RTVCP			0
#define PM8058_CHG_STATUS_RT_3		0x41
#define PM8058_CHG_STATUS_RT_10		0xC1
#define PM8058_CHG_STATUS_RT_11		0xC2

/* VTRIM */
#define PM8058_CHG_VTRIM		0x1D
#define PM8058_CHG_VBATDET_TRIM		0x1E
#define PM8058_CHG_ITRIM		0x1F
#define PM8058_CHG_TTRIM		0x20

#define AUTO_CHARGING_VMAXSEL				4200
#define AUTO_CHARGING_FAST_TIME_MAX_MINUTES		512
#define AUTO_CHARGING_TRICKLE_TIME_MINUTES		30
#ifdef CONFIG_LGE_CHARGER_VOLTAGE_CURRENT_SCENARIO
#define AUTO_CHARGING_VEOC_ITERM			100 //160 in case of VS920
#else
#define AUTO_CHARGING_VEOC_ITERM			100
#endif
#define AUTO_CHARGING_IEOC_ITERM			160

#ifdef CONFIG_LGE_CHARGER_VOLTAGE_CURRENT_SCENARIO
#define AUTO_CHARGING_RESUME_MV				4120
#else
#define AUTO_CHARGING_RESUME_MV				4100
#endif

#ifdef CONFIG_LGE_CHARGER_VOLTAGE_CURRENT_SCENARIO
#define AUTO_CHARGING_VBATDET				4170
#define AUTO_CHARGING_VBATDET_DEBOUNCE_TIME_MS		3000
#define AUTO_CHARGING_VEOC_VBATDET			4120
#else
#define AUTO_CHARGING_VBATDET				4150
#define AUTO_CHARGING_VBATDET_DEBOUNCE_TIME_MS		3000
#define AUTO_CHARGING_VEOC_VBATDET			4100
#endif

#define AUTO_CHARGING_VEOC_TCHG				16
#define AUTO_CHARGING_VEOC_TCHG_FINAL_CYCLE		32
#define AUTO_CHARGING_VEOC_BEGIN_TIME_MS		5400000

#define AUTO_CHARGING_VEOC_VBAT_LOW_CHECK_TIME_MS	60000
#define AUTO_CHARGING_RESUME_CHARGE_DETECTION_COUNTER	5

#define AUTO_CHARGING_DONE_CHECK_TIME_MS		1000

#define PM8058_CHG_I_STEP_MA 50
#define PM8058_CHG_I_MIN_MA 50
#define PM8058_CHG_T_TCHG_SHIFT 2
#define PM8058_CHG_I_TERM_STEP_MA 10
#define PM8058_CHG_V_STEP_MV 25
#define PM8058_CHG_V_MIN_MV  2400

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
#define BATT_UNKNOWN    0
#define BATT_DS2704_N   17
#define BATT_DS2704_L   32
#define BATT_ISL6296_N  73
#define BATT_ISL6296_L  94
#else
#define BATT_ID_MAX_MV  800
#define BATT_ID_MIN_MV  600
#endif

#if 1//def CONFIG_LGE_PM_BATTERY_ID_CHECKER
extern uint16_t battery_info_get(void);
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ALARM
int is_chg_plugged_in(void);
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ALARM
/* kiwone.seo@lge.com 2011-08-14, bug fix of resume charging and low battery alarm */
extern int pm8058_batt_alarm_config(void);
#endif
/*
 * enum pmic_chg_interrupts: pmic interrupts
 * @CHGVAL_IRQ: charger V between 3.3 and 7.9
 * @CHGINVAL_IRQ: charger V outside 3.3 and 7.9
 * @VBATDET_LOW_IRQ: VBAT < VBATDET
 * @VCP_IRQ: VDD went below VBAT: BAT_FET is turned on
 * @CHGILIM_IRQ: mA consumed>IMAXSEL: chgloop draws less mA
 * @ATC_DONE_IRQ: Auto Trickle done
 * @ATCFAIL_IRQ: Auto Trickle fail
 * @AUTO_CHGDONE_IRQ: Auto chg done
 * @AUTO_CHGFAIL_IRQ: time exceeded w/o reaching term current
 * @CHGSTATE_IRQ: something happend causing a state change
 * @FASTCHG_IRQ: trkl charging completed: moving to fastchg
 * @CHG_END_IRQ: mA has dropped to termination current
 * @BATTTEMP_IRQ: batt temp is out of range
 * @CHGHOT_IRQ: the pass device is too hot
 * @CHGTLIMIT_IRQ: unused
 * @CHG_GONE_IRQ: charger was removed
 * @VCPMAJOR_IRQ: vcp major
 * @VBATDET_IRQ: VBAT >= VBATDET
 * @BATFET_IRQ: BATFET closed
 * @BATT_REPLACE_IRQ:
 * @BATTCONNECT_IRQ:
 */
enum pmic_chg_interrupts {
	CHGVAL_IRQ,
	CHGINVAL_IRQ,
	VBATDET_LOW_IRQ,
	VCP_IRQ,
	CHGILIM_IRQ,
	ATC_DONE_IRQ,
	ATCFAIL_IRQ,
	AUTO_CHGDONE_IRQ,
	AUTO_CHGFAIL_IRQ,
	CHGSTATE_IRQ,
	FASTCHG_IRQ,
	CHG_END_IRQ,
	BATTTEMP_IRQ,
	CHGHOT_IRQ,
	CHGTLIMIT_IRQ,
	CHG_GONE_IRQ,
	VCPMAJOR_IRQ,
	VBATDET_IRQ,
	BATFET_IRQ,
	BATT_REPLACE_IRQ,
	BATTCONNECT_IRQ,
	PMIC_CHG_MAX_INTS
};

struct pm8058_charger {
	struct pmic_charger_pdata *pdata;
	struct pm8058_chip *pm_chip;
	struct device *dev;

	int pmic_chg_irq[PMIC_CHG_MAX_INTS];
	DECLARE_BITMAP(enabled_irqs, PMIC_CHG_MAX_INTS);

	struct delayed_work chg_done_check_work;
	struct delayed_work check_vbat_low_work;
	struct delayed_work veoc_begin_work;
	struct delayed_work charging_check_work;
	int waiting_for_topoff;
	int waiting_for_veoc;
	int vbatdet;
	struct msm_hardware_charger hw_chg;
	int current_charger_current;
	int disabled;

	struct msm_xo_voter *voter;
	struct dentry *dent;

	int inited;
	int present;
};

static struct pm8058_charger pm8058_chg;
static struct msm_hardware_charger usb_hw_chg;


static int msm_battery_gague_alarm_notify(struct notifier_block *nb,
					  unsigned long status, void *unused);

static struct notifier_block alarm_notifier = {
	.notifier_call = msm_battery_gague_alarm_notify,
};

static int resume_mv = AUTO_CHARGING_RESUME_MV;
static DEFINE_MUTEX(batt_alarm_lock);
static int resume_mv_set(const char *val, struct kernel_param *kp);
module_param_call(resume_mv, resume_mv_set, param_get_int,
				&resume_mv, S_IRUGO | S_IWUSR);

static int resume_mv_set(const char *val, struct kernel_param *kp)
{
	int rc;

	mutex_lock(&batt_alarm_lock);

	rc = param_set_int(val, kp);
	if (rc)
		goto out;

#ifdef CONFIG_LGE_PM_BATTERY_ALARM
	if (is_chg_plugged_in())
#endif
#ifdef CONFIG_LGE_CHARGER_VOLTAGE_CURRENT_SCENARIO
	rc = pm8058_batt_alarm_threshold_set(resume_mv, 4200);
#else
	rc = pm8058_batt_alarm_threshold_set(resume_mv, 4300);
#endif

out:
	mutex_unlock(&batt_alarm_lock);
	return rc;
}

static void pm8058_chg_enable_irq(int interrupt)
{
	if (!__test_and_set_bit(interrupt, pm8058_chg.enabled_irqs)) {
		dev_dbg(pm8058_chg.dev, "%s %d\n", __func__,
			pm8058_chg.pmic_chg_irq[interrupt]);
		enable_irq(pm8058_chg.pmic_chg_irq[interrupt]);
	}
}

static void pm8058_chg_disable_irq(int interrupt)
{
	if (__test_and_clear_bit(interrupt, pm8058_chg.enabled_irqs)) {
		dev_dbg(pm8058_chg.dev, "%s %d\n", __func__,
			pm8058_chg.pmic_chg_irq[interrupt]);
		disable_irq_nosync(pm8058_chg.pmic_chg_irq[interrupt]);
	}
}

static int pm_chg_get_rt_status(int irq)
{
	int count = 3;
	int ret;

	while ((ret =
		pm8058_irq_get_rt_status(pm8058_chg.pm_chip, irq)) == -EAGAIN
	       && count--) {
		dev_info(pm8058_chg.dev, "%s trycount=%d\n", __func__, count);
		cpu_relax();
	}
	if (ret == -EAGAIN)
		return 0;
	else
		return ret;
}

#ifdef CONFIG_LGE_PM_BATTERY_ALARM
int is_chg_plugged_in(void)
{
	return pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHGVAL_IRQ]);
}
EXPORT_SYMBOL(is_chg_plugged_in);
#else
static int is_chg_plugged_in(void)
{
	return pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHGVAL_IRQ]);
}
#endif

#ifdef DEBUG
static void __dump_chg_regs(void)
{
	u8 temp;
	int temp2;

	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_CNTRL = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_CNTRL_2 = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_VMAX_SEL, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_VMAX_SEL = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_VBAT_DET, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_VBAT_DET = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_IMAX, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_IMAX = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_TRICKLE, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_TRICKLE = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_ITERM, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_ITERM = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_TTRKL_MAX, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_TTRKL_MAX = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_TCHG_MAX, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_TCHG_MAX = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_TEMP_THRESH, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_TEMP_THRESH = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_TEMP_REG, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_TEMP_REG = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_PULSE, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_PULSE = 0x%x\n", temp);

	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_STATUS_CLEAR_IRQ_1,
		    &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_STATUS_CLEAR_IRQ_1 = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_STATUS_CLEAR_IRQ_3,
		    &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_STATUS_CLEAR_IRQ_3 = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_STATUS_CLEAR_IRQ_10,
		    &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_STATUS_CLEAR_IRQ_10 = 0x%x\n",
		temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_STATUS_CLEAR_IRQ_11,
		    &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_STATUS_CLEAR_IRQ_11 = 0x%x\n",
		temp);

	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_MASK_IRQ_1, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_MASK_IRQ_1 = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_MASK_IRQ_3, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_MASK_IRQ_3 = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_MASK_IRQ_10, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_MASK_IRQ_10 = 0x%x\n", temp);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_MASK_IRQ_11, &temp, 1);
	dev_dbg(pm8058_chg.dev, "PM8058_CHG_MASK_IRQ_11 = 0x%x\n", temp);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHGVAL_IRQ]);
	dev_dbg(pm8058_chg.dev, "CHGVAL_IRQ = %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHGINVAL_IRQ]);
	dev_dbg(pm8058_chg.dev, "CHGINVAL_IRQ = %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[VBATDET_LOW_IRQ]);
	dev_dbg(pm8058_chg.dev, "VBATDET_LOW_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[VCP_IRQ]);
	dev_dbg(pm8058_chg.dev, "VCP_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHGILIM_IRQ]);
	dev_dbg(pm8058_chg.dev, "CHGILIM_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[ATC_DONE_IRQ]);
	dev_dbg(pm8058_chg.dev, "ATC_DONE_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[ATCFAIL_IRQ]);
	dev_dbg(pm8058_chg.dev, "ATCFAIL_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[AUTO_CHGDONE_IRQ]);
	dev_dbg(pm8058_chg.dev, "AUTO_CHGDONE_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[AUTO_CHGFAIL_IRQ]);
	dev_dbg(pm8058_chg.dev, "AUTO_CHGFAIL_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHGSTATE_IRQ]);
	dev_dbg(pm8058_chg.dev, "CHGSTATE_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[FASTCHG_IRQ]);
	dev_dbg(pm8058_chg.dev, "FASTCHG_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHG_END_IRQ]);
	dev_dbg(pm8058_chg.dev, "CHG_END_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[BATTTEMP_IRQ]);
	dev_dbg(pm8058_chg.dev, "BATTTEMP_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHGHOT_IRQ]);
	dev_dbg(pm8058_chg.dev, "CHGHOT_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHGTLIMIT_IRQ]);
	dev_dbg(pm8058_chg.dev, "CHGTLIMIT_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[CHG_GONE_IRQ]);
	dev_dbg(pm8058_chg.dev, "CHG_GONE_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[VCPMAJOR_IRQ]);
	dev_dbg(pm8058_chg.dev, "VCPMAJOR_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[VBATDET_IRQ]);
	dev_dbg(pm8058_chg.dev, "VBATDET_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[BATFET_IRQ]);
	dev_dbg(pm8058_chg.dev, "BATFET_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[BATT_REPLACE_IRQ]);
	dev_dbg(pm8058_chg.dev, "BATT_REPLACE_IRQ= %d\n", temp2);

	temp2 = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[BATTCONNECT_IRQ]);
	dev_dbg(pm8058_chg.dev, "BATTCONNECT_IRQ= %d\n", temp2);
}
#else
static inline void __dump_chg_regs(void)
{
}
#endif

/* SSBI register access helper functions */
static int pm_chg_suspend(int value)
{
	u8 temp;
	int ret;

	ret = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
	if (ret)
		return ret;
	if (value)
		temp |= BIT(CHG_USB_SUSPEND);
	else
		temp &= ~BIT(CHG_USB_SUSPEND);

	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
}

#ifdef CONFIG_LGE_PM
int pm_chg_vbatt_fet_on(int value)
{
    u8 temp;
    int ret;

    ret = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
	if (ret)
		return ret;
	if (value)
		temp |= BIT(3);
	else
		temp &= ~BIT(3);
	
    pr_info("TEST: pm_chg_vbatt_fet_ctrl is value=%d,temp=%d\n", value,temp);

    return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
}
EXPORT_SYMBOL(pm_chg_vbatt_fet_on);
#endif


#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
int pm_chg_auto_disable(int value)
{
	u8 temp;
	int ret;

	ret = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
	if (ret)
		return ret;
	if (value)
		temp |= BIT(CHARGE_AUTO_DIS);
	else
		temp &= ~BIT(CHARGE_AUTO_DIS);

	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
}
EXPORT_SYMBOL(pm_chg_auto_disable);
#else
static int pm_chg_auto_disable(int value)
{
	u8 temp;
	int ret;

	ret = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
	if (ret)
		return ret;
	if (value)
		temp |= BIT(CHARGE_AUTO_DIS);
	else
		temp &= ~BIT(CHARGE_AUTO_DIS);

	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
}
#endif

static int pm_chg_batt_temp_disable(int value)
{
	u8 temp;
	int ret;

	ret = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
	if (ret)
		return ret;
	if (value)
		temp |= BIT(CHG_BATT_TEMP_DIS);
	else
		temp &= ~BIT(CHG_BATT_TEMP_DIS);

	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
}

static int pm_chg_vbatdet_set(int voltage)
{
	u8 temp;
	int diff;

	diff = (voltage - PM8058_CHG_V_MIN_MV);
	if (diff < 0) {
		dev_warn(pm8058_chg.dev, "%s bad mV=%d asked to set\n",
			 __func__, voltage);
		return -EINVAL;
	}

	temp = diff / PM8058_CHG_V_STEP_MV;
	dev_dbg(pm8058_chg.dev, "%s voltage=%d setting %02x\n", __func__,
		voltage, temp);
	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_VBAT_DET, &temp, 1);
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
int pm_chg_imaxsel_set(int chg_current)
{
	u8 temp;
	int diff;

	diff = chg_current - PM8058_CHG_I_MIN_MA;
	if (diff < 0) {
		dev_warn(pm8058_chg.dev, "%s bad mA=%d asked to set\n",
			 __func__, chg_current);
		return -EINVAL;
	}
	temp = diff / PM8058_CHG_I_STEP_MA;
	/* make sure we arent writing more than 5 bits of data */
	if (temp > 31) {
		dev_warn(pm8058_chg.dev, "%s max mA=1500 requested mA=%d\n",
			__func__, chg_current);
		temp = 31;
	}
	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_IMAX, &temp, 1);
}
EXPORT_SYMBOL(pm_chg_imaxsel_set);

#else
static int pm_chg_imaxsel_set(int chg_current)
{
	u8 temp;
	int diff;

	diff = chg_current - PM8058_CHG_I_MIN_MA;
	if (diff < 0) {
		dev_warn(pm8058_chg.dev, "%s bad mA=%d asked to set\n",
			 __func__, chg_current);
		return -EINVAL;
	}
	temp = diff / PM8058_CHG_I_STEP_MA;
	/* make sure we arent writing more than 5 bits of data */
	if (temp > 31) {
		dev_warn(pm8058_chg.dev, "%s max mA=1500 requested mA=%d\n",
			__func__, chg_current);
		temp = 31;
	}
	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_IMAX, &temp, 1);
}
#endif

#define PM8058_CHG_VMAX_MIN  3300
#define PM8058_CHG_VMAX_MAX  5500
static int pm_chg_vmaxsel_set(int voltage)
{
	u8 temp;

	if (voltage < PM8058_CHG_VMAX_MIN || voltage > PM8058_CHG_VMAX_MAX) {
		dev_warn(pm8058_chg.dev, "%s bad mV=%d asked to set\n",
			 __func__, voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8058_CHG_V_MIN_MV) / PM8058_CHG_V_STEP_MV;
	dev_dbg(pm8058_chg.dev, "%s mV=%d setting %02x\n", __func__, voltage,
		temp);
	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_VMAX_SEL, &temp, 1);
}

static int pm_chg_failed_clear(int value)
{
	u8 temp;
	int ret;

	ret = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
	if (ret)
		return ret;
	if (value)
		temp |= BIT(CHG_FAILED_CLEAR);
	else
		temp &= ~BIT(CHG_FAILED_CLEAR);
	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
}

static int pm_chg_iterm_set(int chg_current)
{
	u8 temp;

	temp = (chg_current / PM8058_CHG_I_TERM_STEP_MA) - 1;
	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_ITERM, &temp, 1);
}

static int pm_chg_tchg_set(int minutes)
{
	u8 temp;

	temp = (minutes >> PM8058_CHG_T_TCHG_SHIFT) - 1;
	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TCHG_MAX, &temp, 1);
}

static int pm_chg_ttrkl_set(int minutes)
{
	u8 temp;

	temp = minutes - 1;
	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TTRKL_MAX, &temp, 1);
}

static int pm_chg_enum_done_enable(int value)
{
	u8 temp;
	int ret;

	ret = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
	if (ret)
		return ret;
	if (value)
		temp |= BIT(ENUM_DONE);
	else
		temp &= ~BIT(ENUM_DONE);

	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL_2, &temp, 1);
}

static uint32_t get_fsm_state(void)
{
	u8 temp;

	temp = 0x00;
	pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TEST_3, &temp, 1);
	pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_TEST_3, &temp, 1);
	return (uint32_t)temp;
}

static int get_fsm_status(void *data, u64 * val)
{
	*val = get_fsm_state();
	return 0;
}

enum pmic8058_chg_state pmic8058_get_fsm_state(void)
{
	if (!pm8058_chg.inited) {
		pr_err("%s: called when not inited\n", __func__);
		return -EINVAL;
	}

	return get_fsm_state();
}

static int pm_chg_disable(int value)
{
	u8 temp;
	int ret;

	ret = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
	if (ret)
		return ret;
	if (value)
		temp |= BIT(CHG_CHARGE_DIS);
	else
		temp &= ~BIT(CHG_CHARGE_DIS);

	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
}

static void pm8058_start_system_current(struct msm_hardware_charger *hw_chg,
								int max_current)
{
	int ret = 0;

	if (pm8058_chg.disabled)
		return;

	ret = pm_chg_imaxsel_set(max_current);
	ret |= pm_chg_enum_done_enable(1);
	ret |= pm_chg_disable(0);
	if (ret)
		pr_err("%s: failed to turn on system power err=%d",
							__func__, ret);
}

static void pm8058_stop_system_current(struct msm_hardware_charger *hw_chg)
{
	int ret = 0;

	ret = pm_chg_enum_done_enable(0);
	ret |= pm_chg_disable(1);
	if (ret)
		pr_err("%s: failed to turn off system power err=%d",
							__func__, ret);
}

static int __pm8058_start_charging(int chg_current, int termination_current,
				   int time)
{
	int ret = 0;

	if (pm8058_chg.disabled)
		goto out;

	dev_info(pm8058_chg.dev, "%s %dmA %dmin\n",
			__func__, chg_current, time);

	ret = pm_chg_auto_disable(1);
	if (ret)
		goto out;

	ret = pm_chg_suspend(0);
	if (ret)
		goto out;

	ret = pm_chg_imaxsel_set(chg_current);
	if (ret)
		goto out;

	ret = pm_chg_failed_clear(1);
	if (ret)
		goto out;

	ret = pm_chg_iterm_set(termination_current);
	if (ret)
		goto out;

	ret = pm_chg_tchg_set(time);
	if (ret)
		goto out;

	ret = pm_chg_ttrkl_set(AUTO_CHARGING_TRICKLE_TIME_MINUTES);
	if (ret)
		goto out;
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/* kiwone.seo@lge.com, we must enable batt temp monitoring. because the interrupt never be occured 
and must change register value in HW. */
	ret = pm_chg_batt_temp_disable(0);
#endif 
	if (ret)
		goto out;

	if (pm8058_chg.voter == NULL)
		pm8058_chg.voter = msm_xo_get(MSM_XO_TCXO_D1, "pm8058_charger");
	msm_xo_mode_vote(pm8058_chg.voter, MSM_XO_MODE_ON);

	ret = pm_chg_enum_done_enable(1);
	if (ret)
		goto out;

	wmb();

	ret = pm_chg_auto_disable(0);
	if (ret)
		goto out;

	/* wait for the enable to update interrupt status*/
	msleep(20);

	pm8058_chg_enable_irq(AUTO_CHGFAIL_IRQ);
	pm8058_chg_enable_irq(CHGHOT_IRQ);
	pm8058_chg_enable_irq(AUTO_CHGDONE_IRQ);
	pm8058_chg_enable_irq(CHG_END_IRQ);
	pm8058_chg_enable_irq(CHGSTATE_IRQ);

out:
	return ret;
}

static void chg_done_cleanup(void)
{
	dev_info(pm8058_chg.dev, "%s notify pm8058 charging completion"
		"\n", __func__);

	pm8058_chg_disable_irq(AUTO_CHGDONE_IRQ);
	cancel_delayed_work_sync(&pm8058_chg.veoc_begin_work);
	cancel_delayed_work_sync(&pm8058_chg.check_vbat_low_work);

	pm8058_chg_disable_irq(CHG_END_IRQ);

	pm8058_chg_disable_irq(VBATDET_LOW_IRQ);
	pm8058_chg_disable_irq(VBATDET_IRQ);
	pm8058_chg.waiting_for_veoc = 0;
	pm8058_chg.waiting_for_topoff = 0;

	pm_chg_auto_disable(1);

	msm_charger_notify_event(&usb_hw_chg, CHG_DONE_EVENT);
}

static void chg_done_check_work(struct work_struct *work)
{
	chg_done_cleanup();
}

static void charging_check_work(struct work_struct *work)
{
	uint32_t fsm_state = get_fsm_state();
	int rc;

	switch (fsm_state) {
	/* We're charging, so disarm alarm */
	case PMIC8058_CHG_STATE_ATC:
	case PMIC8058_CHG_STATE_FAST_CHG:
	case PMIC8058_CHG_STATE_TRKL_CHG:
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
		rc = is_chg_plugged_in();
		if (rc)
#endif
		rc = pm8058_batt_alarm_state_set(0, 0);
		if (rc)
			dev_err(pm8058_chg.dev,
				"%s: unable to set alarm state\n", __func__);
		break;
	default:
		/* Still not charging, so update driver state */
//		chg_done_cleanup();
		break;
	};
}

static int pm8058_start_charging(struct msm_hardware_charger *hw_chg,
				 int chg_voltage, int chg_current)
{
	int vbat_higher_than_vbatdet;
	int ret = 0;

	cancel_delayed_work_sync(&pm8058_chg.charging_check_work);

	/*
	 * adjust the max current for PC USB connection - set the higher limit
	 * to 450 and make sure we never cross it
	 */
#ifdef CONFIG_LGE_PM_CURRENT_CABLE_TYPE
// for charging scenario.
#else
	if (chg_current == 500)
		chg_current = 450;
#endif	
	pm8058_chg.current_charger_current = chg_current;
	pm8058_chg_enable_irq(FASTCHG_IRQ);

	ret = pm_chg_vmaxsel_set(chg_voltage);
	if (ret)
		goto out;

	/* set vbat to  CC to CV threshold */
	ret = pm_chg_vbatdet_set(AUTO_CHARGING_VBATDET);
	if (ret)
		goto out;

	pm8058_chg.vbatdet = AUTO_CHARGING_VBATDET;
	/*
	 * get the state of vbat and if it is higher than
	 * AUTO_CHARGING_VBATDET we start the veoc start timer
	 * else wait for the voltage to go to AUTO_CHARGING_VBATDET
	 * and then start the 90 min timer
	 */
	vbat_higher_than_vbatdet =
	    pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[VBATDET_IRQ]);
	if (vbat_higher_than_vbatdet) {
		/*
		 * we are in constant voltage phase of charging
		 * IEOC should happen withing 90 mins of this instant
		 * else we enable VEOC
		 */
		dev_info(pm8058_chg.dev, "%s begin veoc timer\n", __func__);
		schedule_delayed_work(&pm8058_chg.veoc_begin_work,
				      round_jiffies_relative(msecs_to_jiffies
				     (AUTO_CHARGING_VEOC_BEGIN_TIME_MS)));
	} else
		pm8058_chg_enable_irq(VBATDET_IRQ);

	ret = __pm8058_start_charging(chg_current, AUTO_CHARGING_IEOC_ITERM,
				AUTO_CHARGING_FAST_TIME_MAX_MINUTES);
	pm8058_chg.current_charger_current = chg_current;

	/*
	 * We want to check the FSM state to verify we're charging. We must
	 * wait before doing this to allow the VBATDET to settle. The worst
	 * case for this is two seconds. The batt alarm does not have this
	 * delay.
	 */
	schedule_delayed_work(&pm8058_chg.charging_check_work,
				      round_jiffies_relative(msecs_to_jiffies
			     (AUTO_CHARGING_VBATDET_DEBOUNCE_TIME_MS)));

out:
	return ret;
}


#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
int pm8058_start_charging_for_ATCMD(void)
{
	int vbat_higher_than_vbatdet;
	int ret = 0;
  int chg_current = 0;

	/*
	 * adjust the max current for PC USB connection - set the higher limit
	 * to 450 and make sure we never cross it
	 */
  chg_current = 800;

	pm8058_chg.current_charger_current = chg_current;
	pm8058_chg_enable_irq(FASTCHG_IRQ);

	ret = pm_chg_vmaxsel_set(5000);
	if (ret)
		goto out;

	/* set vbat to  CC to CV threshold */
	ret = pm_chg_vbatdet_set(AUTO_CHARGING_VBATDET);

	if (ret)
		goto out;

	pm8058_chg.vbatdet = AUTO_CHARGING_VBATDET;
	/*
	 * get the state of vbat and if it is higher than
	 * AUTO_CHARGING_VBATDET we start the veoc start timer
	 * else wait for the voltage to go to AUTO_CHARGING_VBATDET
	 * and then start the 90 min timer
	 */
	vbat_higher_than_vbatdet =
	    pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[VBATDET_IRQ]);
	if (vbat_higher_than_vbatdet) {
		/*
		 * we are in constant voltage phase of charging
		 * IEOC should happen withing 90 mins of this instant
		 * else we enable VEOC
		 */
		dev_info(pm8058_chg.dev, "%s begin veoc timer\n", __func__);
		schedule_delayed_work(&pm8058_chg.veoc_begin_work,
				      round_jiffies_relative(msecs_to_jiffies
				     (AUTO_CHARGING_VEOC_BEGIN_TIME_MS)));
	} else
		pm8058_chg_enable_irq(VBATDET_IRQ);

	ret = __pm8058_start_charging(chg_current, AUTO_CHARGING_IEOC_ITERM,
				AUTO_CHARGING_FAST_TIME_MAX_MINUTES);
	pm8058_chg.current_charger_current = chg_current;
out:
	return ret;
}

EXPORT_SYMBOL(pm8058_start_charging_for_ATCMD);

int pm8058_start_charging_for_TESTMODE(void)
{
	int vbat_higher_than_vbatdet;
	int ret = 0;
  int chg_current = 0;

	/*
	 * adjust the max current for PC USB connection - set the higher limit
	 * to 450 and make sure we never cross it
	 */
  chg_current = 800;

	pm8058_chg.current_charger_current = chg_current;
	pm8058_chg_enable_irq(FASTCHG_IRQ);

	ret = pm_chg_vmaxsel_set(4200);
	if (ret)
		goto out;

	/* set vbat to  CC to CV threshold */
	ret = pm_chg_vbatdet_set(AUTO_CHARGING_VBATDET);

	if (ret)
		goto out;

	pm8058_chg.vbatdet = AUTO_CHARGING_VBATDET;
	/*
	 * get the state of vbat and if it is higher than
	 * AUTO_CHARGING_VBATDET we start the veoc start timer
	 * else wait for the voltage to go to AUTO_CHARGING_VBATDET
	 * and then start the 90 min timer
	 */
	vbat_higher_than_vbatdet =
	    pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[VBATDET_IRQ]);
	if (vbat_higher_than_vbatdet) {
		/*
		 * we are in constant voltage phase of charging
		 * IEOC should happen withing 90 mins of this instant
		 * else we enable VEOC
		 */
		dev_info(pm8058_chg.dev, "%s begin veoc timer\n", __func__);
		schedule_delayed_work(&pm8058_chg.veoc_begin_work,
				      round_jiffies_relative(msecs_to_jiffies
				     (AUTO_CHARGING_VEOC_BEGIN_TIME_MS)));
	} else
		pm8058_chg_enable_irq(VBATDET_IRQ);

	ret = __pm8058_start_charging(chg_current, AUTO_CHARGING_IEOC_ITERM,
				AUTO_CHARGING_FAST_TIME_MAX_MINUTES);
	pm8058_chg.current_charger_current = chg_current;
out:
	return ret;
}

EXPORT_SYMBOL(pm8058_start_charging_for_TESTMODE);

#endif


static void veoc_begin_work(struct work_struct *work)
{
	/* we have been doing CV for 90mins with no signs of IEOC
	 * start checking for VEOC in addition with 16min charges*/
	dev_info(pm8058_chg.dev, "%s begin veoc detection\n", __func__);
	pm8058_chg.waiting_for_veoc = 1;
	/*
	 * disable VBATDET irq we dont need it unless we are at the end of
	 * charge cycle
	 */
	pm8058_chg_disable_irq(VBATDET_IRQ);
	__pm8058_start_charging(pm8058_chg.current_charger_current,
				AUTO_CHARGING_VEOC_ITERM,
				AUTO_CHARGING_VEOC_TCHG);
}

static void vbat_low_work(struct work_struct *work)
{
	/*
	 * It has been one minute and the battery still holds voltage
	 * start the final topoff - charging is almost done
	 */
	dev_info(pm8058_chg.dev, "%s vbatt maintains for a minute"
		"starting topoff\n", __func__);
	pm8058_chg.waiting_for_veoc = 0;
	pm8058_chg.waiting_for_topoff = 1;
	pm8058_chg_disable_irq(VBATDET_LOW_IRQ);
	pm8058_chg_disable_irq(VBATDET_IRQ);
	__pm8058_start_charging(pm8058_chg.current_charger_current,
				AUTO_CHARGING_VEOC_ITERM,
				AUTO_CHARGING_VEOC_TCHG_FINAL_CYCLE);
}


static irqreturn_t pm8058_chg_chgval_handler(int irq, void *dev_id)
{
	u8 old, temp;
	int ret;

	if (is_chg_plugged_in()) {	/* this debounces it */
		if (!pm8058_chg.present) {
			msm_charger_notify_event(&usb_hw_chg,
						CHG_INSERTED_EVENT);
			pm8058_chg.present = 1;
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
/* kiwone.seo@lge.com 2011-08-14, bug fix of resume charging and low battery alarm */
			pm8058_batt_alarm_config();
#endif
		}
	} else {
		if (pm8058_chg.present) {
			ret = pm8058_read(pm8058_chg.pm_chip,
						PM8058_OVP_TEST_REG,
						&old, 1);
			temp = old | BIT(FORCE_OVP_OFF);
			ret = pm8058_write(pm8058_chg.pm_chip,
						PM8058_OVP_TEST_REG,
						&temp, 1);
			temp = 0xFC;
			ret = pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TEST,
						&temp, 1);
			/* 10 ms sleep is for the VCHG to discharge */
			msleep(10);
			temp = 0xF0;
			ret = pm8058_write(pm8058_chg.pm_chip,
						PM8058_CHG_TEST,
						&temp, 1);
			ret = pm8058_write(pm8058_chg.pm_chip,
						PM8058_OVP_TEST_REG,
						&old, 1);

			pm_chg_enum_done_enable(0);
			pm_chg_auto_disable(1);
			msm_charger_notify_event(&usb_hw_chg,
						CHG_REMOVED_EVENT);
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
/* kiwone.seo@lge.com 2011-08-14, bug fix of resume charging and low battery alarm */
			pm8058_batt_alarm_config();			
#endif			
			pm8058_chg.present = 0;
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t pm8058_chg_chginval_handler(int irq, void *dev_id)
{
	u8 old, temp;
	int ret;

	if (pm8058_chg.present) {
		pm8058_chg_disable_irq(CHGINVAL_IRQ);

		pm_chg_enum_done_enable(0);
		pm_chg_auto_disable(1);
		ret = pm8058_read(pm8058_chg.pm_chip,
				PM8058_OVP_TEST_REG, &old, 1);
		temp = old | BIT(FORCE_OVP_OFF);
		ret = pm8058_write(pm8058_chg.pm_chip,
				PM8058_OVP_TEST_REG, &temp, 1);
		temp = 0xFC;
		ret = pm8058_write(pm8058_chg.pm_chip,
				PM8058_CHG_TEST, &temp, 1);
		/* 10 ms sleep is for the VCHG to discharge */
		msleep(10);
		temp = 0xF0;
		ret = pm8058_write(pm8058_chg.pm_chip,
				PM8058_CHG_TEST, &temp, 1);
		ret = pm8058_write(pm8058_chg.pm_chip,
				PM8058_OVP_TEST_REG, &old, 1);

		if (!is_chg_plugged_in()) {
			msm_charger_notify_event(&usb_hw_chg,
					CHG_REMOVED_EVENT);
			pm8058_chg.present = 0;
		} else {
			/* was a fake */
			pm8058_chg_enable_irq(CHGINVAL_IRQ);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t pm8058_chg_auto_chgdone_handler(int irq, void *dev_id)
{
	dev_info(pm8058_chg.dev, "%s waiting a sec to confirm\n",
		__func__);
	pm8058_chg_disable_irq(AUTO_CHGDONE_IRQ);
	pm8058_chg_disable_irq(VBATDET_IRQ);
	if (!delayed_work_pending(&pm8058_chg.chg_done_check_work)) {
		schedule_delayed_work(&pm8058_chg.chg_done_check_work,
				      round_jiffies_relative(msecs_to_jiffies
			     (AUTO_CHARGING_DONE_CHECK_TIME_MS)));
	}
	return IRQ_HANDLED;
}

/* can only happen with the pmic charger when it has been charing
 * for either 16 mins wating for VEOC or 32 mins for topoff
 * without a IEOC indication */
static irqreturn_t pm8058_chg_auto_chgfail_handler(int irq, void *dev_id)
{
	pm8058_chg_disable_irq(AUTO_CHGFAIL_IRQ);

	if (pm8058_chg.waiting_for_topoff == 1) {
		dev_info(pm8058_chg.dev, "%s topoff done, charging done\n",
			__func__);
		pm8058_chg.waiting_for_topoff = 0;
		/* notify we are done charging */
		msm_charger_notify_event(&usb_hw_chg, CHG_DONE_EVENT);
	} else {
		/* start one minute timer and monitor VBATDET_LOW */
		dev_info(pm8058_chg.dev, "%s monitoring vbat_low for a"
			"minute\n", __func__);
		schedule_delayed_work(&pm8058_chg.check_vbat_low_work,
				      round_jiffies_relative(msecs_to_jiffies
			     (AUTO_CHARGING_VEOC_VBAT_LOW_CHECK_TIME_MS)));

		/* note we are waiting on veoc */
		pm8058_chg.waiting_for_veoc = 1;

		pm_chg_vbatdet_set(AUTO_CHARGING_VEOC_VBATDET);
		pm8058_chg.vbatdet = AUTO_CHARGING_VEOC_VBATDET;
		pm8058_chg_enable_irq(VBATDET_LOW_IRQ);
	}
	return IRQ_HANDLED;
}

static irqreturn_t pm8058_chg_chgstate_handler(int irq, void *dev_id)
{
	u8 temp;

	temp = 0x00;
	if (!pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TEST_3, &temp, 1)) {
		pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_TEST_3, &temp, 1);
		dev_dbg(pm8058_chg.dev, "%s state=%d\n", __func__, temp);
	}
	return IRQ_HANDLED;
}

static irqreturn_t pm8058_chg_fastchg_handler(int irq, void *dev_id)
{
	pm8058_chg_disable_irq(FASTCHG_IRQ);

	/* we have begun the fast charging state */
	dev_info(pm8058_chg.dev, "%s begin fast charging"
		, __func__);
	msm_charger_notify_event(&usb_hw_chg, CHG_BATT_BEGIN_FAST_CHARGING);
	return IRQ_HANDLED;
}

static irqreturn_t pm8058_chg_batttemp_handler(int irq, void *dev_id)
{
	int ret;

	/* we could get temperature
	 * interrupt when the battery is plugged out
	 */
	ret = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[BATTCONNECT_IRQ]);
	if (ret) {
		msm_charger_notify_event(&usb_hw_chg, CHG_BATT_REMOVED);
	} else {
		/* read status to determine we are inrange or outofrange */
		ret =
		    pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[BATTTEMP_IRQ]);
		if (ret)
			msm_charger_notify_event(&usb_hw_chg,
						 CHG_BATT_TEMP_OUTOFRANGE);
		else
			msm_charger_notify_event(&usb_hw_chg,
						 CHG_BATT_TEMP_INRANGE);
	}

	return IRQ_HANDLED;
}

static irqreturn_t pm8058_chg_vbatdet_handler(int irq, void *dev_id)
{
	int ret;

	/* settling time */
	msleep(20);
	ret = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[VBATDET_IRQ]);

	if (ret) {
		if (pm8058_chg.vbatdet == AUTO_CHARGING_VBATDET
			&& !delayed_work_pending(&pm8058_chg.veoc_begin_work)) {
			/*
			 * we are in constant voltage phase of charging
			 * IEOC should happen withing 90 mins of this instant
			 * else we enable VEOC
			 */
			dev_info(pm8058_chg.dev, "%s entered constant voltage"
				"begin veoc timer\n", __func__);
			schedule_delayed_work(&pm8058_chg.veoc_begin_work,
				      round_jiffies_relative
				      (msecs_to_jiffies
				      (AUTO_CHARGING_VEOC_BEGIN_TIME_MS)));
		}
	} else {
		if (pm8058_chg.vbatdet == AUTO_CHARGING_VEOC_VBATDET) {

			cancel_delayed_work_sync(
				&pm8058_chg.check_vbat_low_work);

			if (pm8058_chg.waiting_for_topoff
			    || pm8058_chg.waiting_for_veoc) {
				/*
				 * the battery dropped its voltage below 4100
				 * around a minute charge the battery for 16
				 * mins and check vbat again for a minute
				 */
				dev_info(pm8058_chg.dev, "%s batt dropped vlt"
					"within a minute\n", __func__);
				pm8058_chg.waiting_for_topoff = 0;
				pm8058_chg.waiting_for_veoc = 1;
				pm8058_chg_disable_irq(VBATDET_IRQ);
				__pm8058_start_charging(pm8058_chg.
						current_charger_current,
						AUTO_CHARGING_VEOC_ITERM,
						AUTO_CHARGING_VEOC_TCHG);
			}
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t pm8058_chg_batt_replace_handler(int irq, void *dev_id)
{
	int ret;

	pm8058_chg_disable_irq(BATT_REPLACE_IRQ);
	ret = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[BATT_REPLACE_IRQ]);
	if (ret) {
		msm_charger_notify_event(&usb_hw_chg, CHG_BATT_INSERTED);
		/*
		 * battery is present enable batt removal
		 * and batt temperatture interrupt
		 */
		pm8058_chg_enable_irq(BATTCONNECT_IRQ);
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
extern void msm_power_off(void);
#endif
static irqreturn_t pm8058_chg_battconnect_handler(int irq, void *dev_id)
{
	int ret;

	ret = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[BATTCONNECT_IRQ]);
	if (ret) {
		msm_charger_notify_event(&usb_hw_chg, CHG_BATT_REMOVED);
	} else {
		msm_charger_notify_event(&usb_hw_chg, CHG_BATT_INSERTED);
		pm8058_chg_enable_irq(BATTTEMP_IRQ);
	}
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
			msm_power_off();
#endif
	return IRQ_HANDLED;
}

static int get_rt_status(void *data, u64 * val)
{
	int i = (int)data;
	int ret;

	ret = pm_chg_get_rt_status(i);
	*val = ret;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(rt_fops, get_rt_status, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fsm_fops, get_fsm_status, NULL, "%llu\n");

static void free_irqs(void)
{
	int i;

	for (i = 0; i < PMIC_CHG_MAX_INTS; i++)
		if (pm8058_chg.pmic_chg_irq[i]) {
			free_irq(pm8058_chg.pmic_chg_irq[i], NULL);
			pm8058_chg.pmic_chg_irq[i] = 0;
		}
}

static int __devinit request_irqs(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	ret = 0;
	bitmap_fill(pm8058_chg.enabled_irqs, PMIC_CHG_MAX_INTS);

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "CHGVAL");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource CHGVAL\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_chgval_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				  res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[CHGVAL_IRQ] = res->start;
			pm8058_chg_disable_irq(CHGVAL_IRQ);
			enable_irq_wake(pm8058_chg.pmic_chg_irq[CHGVAL_IRQ]);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "CHGINVAL");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource CHGINVAL\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_chginval_handler,
				  IRQF_TRIGGER_RISING, res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[CHGINVAL_IRQ] = res->start;
			pm8058_chg_disable_irq(CHGINVAL_IRQ);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					   "AUTO_CHGDONE");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource AUTO_CHGDONE\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_auto_chgdone_handler,
				  IRQF_TRIGGER_RISING,
				  res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[AUTO_CHGDONE_IRQ] = res->start;
			pm8058_chg_disable_irq(AUTO_CHGDONE_IRQ);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					   "AUTO_CHGFAIL");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource AUTO_CHGFAIL\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_auto_chgfail_handler,
				  IRQF_TRIGGER_RISING, res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[AUTO_CHGFAIL_IRQ] = res->start;
			pm8058_chg_disable_irq(AUTO_CHGFAIL_IRQ);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "CHGSTATE");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource CHGSTATE\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_chgstate_handler,
				  IRQF_TRIGGER_RISING, res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[CHGSTATE_IRQ] = res->start;
			pm8058_chg_disable_irq(CHGSTATE_IRQ);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "FASTCHG");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource FASTCHG\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_fastchg_handler,
				  IRQF_TRIGGER_RISING, res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[FASTCHG_IRQ] = res->start;
			pm8058_chg_disable_irq(FASTCHG_IRQ);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "BATTTEMP");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource CHG_END\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_batttemp_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				  res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[BATTTEMP_IRQ] = res->start;
			pm8058_chg_disable_irq(BATTTEMP_IRQ);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					   "BATT_REPLACE");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource BATT_REPLACE\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_batt_replace_handler,
				  IRQF_TRIGGER_RISING, res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[BATT_REPLACE_IRQ] = res->start;
			pm8058_chg_disable_irq(BATT_REPLACE_IRQ);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "BATTCONNECT");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource BATTCONNECT\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_battconnect_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				  res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[BATTCONNECT_IRQ] = res->start;
			pm8058_chg_disable_irq(BATTCONNECT_IRQ);
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "VBATDET");
	if (res == NULL) {
		dev_err(pm8058_chg.dev,
			"%s:couldnt find resource VBATDET\n", __func__);
		goto err_out;
	} else {
		ret = request_threaded_irq(res->start, NULL,
				  pm8058_chg_vbatdet_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				  res->name, NULL);
		if (ret < 0) {
			dev_err(pm8058_chg.dev, "%s:couldnt request %d %d\n",
				__func__, res->start, ret);
			goto err_out;
		} else {
			pm8058_chg.pmic_chg_irq[VBATDET_IRQ] = res->start;
			pm8058_chg_disable_irq(VBATDET_IRQ);
		}
	}

	return 0;

err_out:
	free_irqs();
	return -EINVAL;
}

static int pm8058_get_charge_batt(void)
{
	u8 temp;
	int rc;

	rc = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
	if (rc)
		return rc;

	temp &= BIT(CHG_CHARGE_BAT);
	if (temp)
		temp = 1;
	return temp;
}
EXPORT_SYMBOL(pm8058_get_charge_batt);

static int pm8058_set_charge_batt(int on)
{
	u8 temp;
	int rc;

	rc = pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);
	if (rc)
		return rc;
	if (on)
		temp |= BIT(CHG_CHARGE_BAT);
	else
		temp &= ~BIT(CHG_CHARGE_BAT);

	return pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_CNTRL, &temp, 1);

}
EXPORT_SYMBOL(pm8058_set_charge_batt);

static int get_charge_batt(void *data, u64 * val)
{
	int ret;

	ret = pm8058_get_charge_batt();
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}

static int set_charge_batt(void *data, u64 val)
{
	return pm8058_set_charge_batt(val);
}
DEFINE_SIMPLE_ATTRIBUTE(fet_fops, get_charge_batt, set_charge_batt, "%llu\n");

static void pm8058_chg_determine_initial_state(void)
{
	if (is_chg_plugged_in()) {
		pm8058_chg.present = 1;
		msm_charger_notify_event(&usb_hw_chg, CHG_INSERTED_EVENT);
		dev_info(pm8058_chg.dev, "%s charger present\n", __func__);
	} else {
		pm8058_chg.present = 0;
		dev_info(pm8058_chg.dev, "%s charger absent\n", __func__);
	}
	pm8058_chg_enable_irq(CHGVAL_IRQ);
}

static int pm8058_stop_charging(struct msm_hardware_charger *hw_chg)
{
	int ret;

	dev_info(pm8058_chg.dev, "%s stopping charging\n", __func__);

	/* disable the irqs enabled while charging */
	pm8058_chg_disable_irq(AUTO_CHGFAIL_IRQ);
	pm8058_chg_disable_irq(CHGHOT_IRQ);
	pm8058_chg_disable_irq(AUTO_CHGDONE_IRQ);
	pm8058_chg_disable_irq(FASTCHG_IRQ);
	pm8058_chg_disable_irq(CHG_END_IRQ);
	pm8058_chg_disable_irq(VBATDET_IRQ);
	pm8058_chg_disable_irq(VBATDET_LOW_IRQ);

	cancel_delayed_work_sync(&pm8058_chg.veoc_begin_work);
	cancel_delayed_work_sync(&pm8058_chg.check_vbat_low_work);
	cancel_delayed_work_sync(&pm8058_chg.chg_done_check_work);
	cancel_delayed_work_sync(&pm8058_chg.charging_check_work);

	ret = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[FASTCHG_IRQ]);
	if (ret == 1)
		pm_chg_suspend(1);
	else
		dev_err(pm8058_chg.dev,
			"%s called when not fast-charging\n", __func__);

	pm_chg_failed_clear(1);

	pm8058_chg.waiting_for_veoc = 0;
	pm8058_chg.waiting_for_topoff = 0;

	if (pm8058_chg.voter)
		msm_xo_mode_vote(pm8058_chg.voter, MSM_XO_MODE_OFF);

	return 0;
}

#ifdef CONFIG_LGE_AT_COMMAND_ABOUT_POWER
int pm8058_stop_charging_for_ATCMD(void)
{
	int ret;

	dev_info(pm8058_chg.dev, "%s stopping charging\n", __func__);
	cancel_delayed_work_sync(&pm8058_chg.veoc_begin_work);
	cancel_delayed_work_sync(&pm8058_chg.check_vbat_low_work);
	cancel_delayed_work_sync(&pm8058_chg.chg_done_check_work);

	ret = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[FASTCHG_IRQ]);
	if (ret == 1)
		pm_chg_suspend(1);
	else
		dev_err(pm8058_chg.dev,
			"%s called when not fast-charging\n", __func__);

	pm_chg_failed_clear(1);

	pm8058_chg.waiting_for_veoc = 0;
	pm8058_chg.waiting_for_topoff = 0;


	/* disable the irqs enabled while charging */
	pm8058_chg_disable_irq(AUTO_CHGFAIL_IRQ);
	pm8058_chg_disable_irq(CHGHOT_IRQ);
	pm8058_chg_disable_irq(AUTO_CHGDONE_IRQ);
	pm8058_chg_disable_irq(FASTCHG_IRQ);
	pm8058_chg_disable_irq(CHG_END_IRQ);
	pm8058_chg_disable_irq(VBATDET_IRQ);
	pm8058_chg_disable_irq(VBATDET_LOW_IRQ);
	if (pm8058_chg.voter)
		msm_xo_mode_vote(pm8058_chg.voter, MSM_XO_MODE_OFF);

  if (!delayed_work_pending(&pm8058_chg.chg_done_check_work)) 
  {
		schedule_delayed_work(&pm8058_chg.chg_done_check_work,
				      round_jiffies_relative(msecs_to_jiffies
			     (100)));
  }

	return 0;
}

EXPORT_SYMBOL(pm8058_stop_charging_for_ATCMD);

int pm8058_stop_charging_for_TESTMODE(void)
{
	int ret;

	dev_info(pm8058_chg.dev, "%s stopping charging\n", __func__);
	cancel_delayed_work_sync(&pm8058_chg.veoc_begin_work);
	cancel_delayed_work_sync(&pm8058_chg.check_vbat_low_work);
	cancel_delayed_work_sync(&pm8058_chg.chg_done_check_work);

	ret = pm_chg_get_rt_status(pm8058_chg.pmic_chg_irq[FASTCHG_IRQ]);
	if (ret == 1)
		pm_chg_suspend(1);
	else
		dev_err(pm8058_chg.dev,
			"%s called when not fast-charging\n", __func__);

	pm_chg_failed_clear(1);

	pm8058_chg.waiting_for_veoc = 0;
	pm8058_chg.waiting_for_topoff = 0;


	/* disable the irqs enabled while charging */
	pm8058_chg_disable_irq(AUTO_CHGFAIL_IRQ);
	pm8058_chg_disable_irq(CHGHOT_IRQ);
	pm8058_chg_disable_irq(AUTO_CHGDONE_IRQ);
	pm8058_chg_disable_irq(FASTCHG_IRQ);
	pm8058_chg_disable_irq(CHG_END_IRQ);
	pm8058_chg_disable_irq(VBATDET_IRQ);
	pm8058_chg_disable_irq(VBATDET_LOW_IRQ);
	if (pm8058_chg.voter)
		msm_xo_mode_vote(pm8058_chg.voter, MSM_XO_MODE_OFF);

  if (!delayed_work_pending(&pm8058_chg.chg_done_check_work)) 
  {
		schedule_delayed_work(&pm8058_chg.chg_done_check_work,
				      round_jiffies_relative(msecs_to_jiffies
			     (100)));
  }

	return 0;
}

EXPORT_SYMBOL(pm8058_stop_charging_for_TESTMODE);

#endif


static int get_status(void *data, u64 * val)
{
	*val = pm8058_chg.current_charger_current;
	return 0;
}

static int set_status(void *data, u64 val)
{

	pm8058_chg.current_charger_current = val;
	if (pm8058_chg.current_charger_current)
		pm8058_start_charging(NULL,
			AUTO_CHARGING_VMAXSEL,
			pm8058_chg.current_charger_current);
	else
		pm8058_stop_charging(NULL);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(chg_fops, get_status, set_status, "%llu\n");

static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	if (pm8058_chg.inited && pm8058_chg.disabled) {
		/*
		 * stop_charging is called during usb suspend
		 * act as the usb is removed by disabling auto and enum
		 */
		pm_chg_enum_done_enable(0);
		pm_chg_auto_disable(1);
		pm8058_stop_charging(NULL);
	}
	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&(pm8058_chg.disabled), 0644);

static int pm8058_charging_switched(struct msm_hardware_charger *hw_chg)
{
	u8 temp;

	temp = 0xA3;
	pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TEST_2, &temp, 1);
	temp = 0x84;
	pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TEST_2, &temp, 1);
	msleep(2);
	temp = 0x80;
	pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TEST_2, &temp, 1);
	return 0;
}

static int get_reg(void *data, u64 * val)
{
	int i = (int)data;
	int ret;
	u8 temp;

	ret = pm8058_read(pm8058_chg.pm_chip, i, &temp, 1);
	if (ret)
		return -EAGAIN;
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	int i = (int)data;
	int ret;
	u8 temp;

	temp = (u8) val;
	ret = pm8058_write(pm8058_chg.pm_chip, i, &temp, 1);
	mb();
	if (ret)
		return -EAGAIN;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "%llu\n");

#ifdef CONFIG_DEBUG_FS
static void create_debugfs_entries(void)
{
	pm8058_chg.dent = debugfs_create_dir("pm8058_usb_chg", NULL);

	if (IS_ERR(pm8058_chg.dent)) {
		pr_err("pmic charger couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("CHG_CNTRL", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_CNTRL, &reg_fops);
	debugfs_create_file("CHG_CNTRL_2", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_CNTRL_2, &reg_fops);
	debugfs_create_file("CHG_VMAX_SEL", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_VMAX_SEL, &reg_fops);
	debugfs_create_file("CHG_VBAT_DET", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_VBAT_DET, &reg_fops);
	debugfs_create_file("CHG_IMAX", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_IMAX, &reg_fops);
	debugfs_create_file("CHG_TRICKLE", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_TRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITERM", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_ITERM, &reg_fops);
	debugfs_create_file("CHG_TTRKL_MAX", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_TTRKL_MAX, &reg_fops);
	debugfs_create_file("CHG_TCHG_MAX", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_TCHG_MAX, &reg_fops);
	debugfs_create_file("CHG_TEMP_THRESH", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_TEMP_THRESH, &reg_fops);
	debugfs_create_file("CHG_TEMP_REG", 0644, pm8058_chg.dent,
			    (void *)PM8058_CHG_TEMP_REG, &reg_fops);

	debugfs_create_file("FSM_STATE", 0644, pm8058_chg.dent, NULL,
			    &fsm_fops);

	debugfs_create_file("stop", 0644, pm8058_chg.dent, NULL,
			    &chg_fops);

	if (pm8058_chg.pmic_chg_irq[CHGVAL_IRQ])
		debugfs_create_file("CHGVAL", 0444, pm8058_chg.dent,
				    (void *)pm8058_chg.pmic_chg_irq[CHGVAL_IRQ],
				    &rt_fops);

	if (pm8058_chg.pmic_chg_irq[CHGINVAL_IRQ])
		debugfs_create_file("CHGINVAL", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[CHGINVAL_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[CHGILIM_IRQ])
		debugfs_create_file("CHGILIM", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[CHGILIM_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[VCP_IRQ])
		debugfs_create_file("VCP", 0444, pm8058_chg.dent,
				    (void *)pm8058_chg.pmic_chg_irq[VCP_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[ATC_DONE_IRQ])
		debugfs_create_file("ATC_DONE", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[ATC_DONE_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[ATCFAIL_IRQ])
		debugfs_create_file("ATCFAIL", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[ATCFAIL_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[AUTO_CHGDONE_IRQ])
		debugfs_create_file("AUTO_CHGDONE", 0444, pm8058_chg.dent,
				    (void *)
				    pm8058_chg.pmic_chg_irq[AUTO_CHGDONE_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[AUTO_CHGFAIL_IRQ])
		debugfs_create_file("AUTO_CHGFAIL", 0444, pm8058_chg.dent,
				    (void *)
				    pm8058_chg.pmic_chg_irq[AUTO_CHGFAIL_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[CHGSTATE_IRQ])
		debugfs_create_file("CHGSTATE", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[CHGSTATE_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[FASTCHG_IRQ])
		debugfs_create_file("FASTCHG", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[FASTCHG_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[CHG_END_IRQ])
		debugfs_create_file("CHG_END", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[CHG_END_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[BATTTEMP_IRQ])
		debugfs_create_file("BATTTEMP", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[BATTTEMP_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[CHGHOT_IRQ])
		debugfs_create_file("CHGHOT", 0444, pm8058_chg.dent,
				    (void *)pm8058_chg.pmic_chg_irq[CHGHOT_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[CHGTLIMIT_IRQ])
		debugfs_create_file("CHGTLIMIT", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[CHGTLIMIT_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[CHG_GONE_IRQ])
		debugfs_create_file("CHG_GONE", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[CHG_GONE_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[VCPMAJOR_IRQ])
		debugfs_create_file("VCPMAJOR", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[VCPMAJOR_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[BATFET_IRQ])
		debugfs_create_file("BATFET", 0444, pm8058_chg.dent,
				    (void *)pm8058_chg.pmic_chg_irq[BATFET_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[BATT_REPLACE_IRQ])
		debugfs_create_file("BATT_REPLACE", 0444, pm8058_chg.dent,
				    (void *)
				    pm8058_chg.pmic_chg_irq[BATT_REPLACE_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[BATTCONNECT_IRQ])
		debugfs_create_file("BATTCONNECT", 0444, pm8058_chg.dent,
				    (void *)
				    pm8058_chg.pmic_chg_irq[BATTCONNECT_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[VBATDET_IRQ])
		debugfs_create_file("VBATDET", 0444, pm8058_chg.dent, (void *)
				    pm8058_chg.pmic_chg_irq[VBATDET_IRQ],
				    &rt_fops);
	if (pm8058_chg.pmic_chg_irq[VBATDET_LOW_IRQ])
		debugfs_create_file("VBATDET_LOW", 0444, pm8058_chg.dent,
				    (void *)
				    pm8058_chg.pmic_chg_irq[VBATDET_LOW_IRQ],
				    &rt_fops);
	debugfs_create_file("CHARGE_BATT", 0444, pm8058_chg.dent,
				    NULL,
				    &fet_fops);
}
#else
static inline void create_debugfs_entries(void)
{
}
#endif

static void remove_debugfs_entries(void)
{
	debugfs_remove_recursive(pm8058_chg.dent);
}

static struct msm_hardware_charger usb_hw_chg = {
	.type			= CHG_TYPE_USB,
#ifdef CONFIG_LGE_WIRELESS_CHARGER_BQ24160
        .rating 		= 2,
#else
	.rating			= 1,
#endif
	.name			= "pm8058-usb",
	.start_charging		= pm8058_start_charging,
	.stop_charging		= pm8058_stop_charging,
	.charging_switched	= pm8058_charging_switched,
	.start_system_current	= pm8058_start_system_current,
	.stop_system_current	= pm8058_stop_system_current,
};

#ifdef CONFIG_LGE_PM
#define MSM_CHARGER_GAUGE_MISSING_TEMP_ADC 1000
#define MSM_CHARGER_GAUGE_MISSING_TEMP       35
#define MSM_PMIC_ADC_READ_TIMEOUT          3000
#define CHANNEL_ADC_ACC_MISSING            2200
extern int32_t pm8058_xoadc_clear_recentQ(void);
extern struct wake_lock adc_wake_lock;
//for suspend reume work around code 
static int g_temp = MSM_CHARGER_GAUGE_MISSING_TEMP;
#endif
static int batt_read_adc(int channel, int *mv_reading)
{
	int ret;
	void *h;
	struct adc_chan_result adc_chan_result;
	struct completion  conv_complete_evt;
#ifdef CONFIG_LGE_PM
    int wait_ret;
    wake_lock(&adc_wake_lock);
#endif
	pr_debug("%s: called for %d\n", __func__, channel);
	ret = adc_channel_open(channel, &h);
	if (ret) {
		pr_err("%s: couldnt open channel %d ret=%d\n",
					__func__, channel, ret);
		goto out;
	}
	init_completion(&conv_complete_evt);
	ret = adc_channel_request_conv(h, &conv_complete_evt);
	if (ret) {
		pr_err("%s: couldnt request conv channel %d ret=%d\n",
						__func__, channel, ret);
		goto out;
	}
#ifdef CONFIG_LGE_PM
    wait_ret = wait_for_completion_timeout(&conv_complete_evt, msecs_to_jiffies(MSM_PMIC_ADC_READ_TIMEOUT));
    if(wait_ret <= 0)
    {
		printk(KERN_ERR "===%s: failed to adc wait for completion!===\n",__func__);
        goto sanity_out;
    }
#else
	wait_for_completion(&conv_complete_evt);
#endif
	ret = adc_channel_read_result(h, &adc_chan_result);
	if (ret) {
		pr_err("%s: couldnt read result channel %d ret=%d\n",
						__func__, channel, ret);
		goto out;
	}
	ret = adc_channel_close(h);
	if (ret) {
		pr_err("%s: couldnt close channel %d ret=%d\n",
						__func__, channel, ret);
	}
	if (mv_reading)
		*mv_reading = adc_chan_result.measurement;

	pr_debug("%s: done for %d\n", __func__, channel);
#ifdef CONFIG_LGE_PM
    wake_unlock(&adc_wake_lock);
    if(channel == CHANNEL_ADC_BATT_THERM)
    g_temp = adc_chan_result.physical;
#endif
	return adc_chan_result.physical;
out:
	pr_debug("%s: done for %d\n", __func__, channel);
#ifdef CONFIG_LGE_PM
    if(ret == -EBUSY)
        adc_channel_close(h);
    
    wake_unlock(&adc_wake_lock);
#endif
	return -EINVAL;

#ifdef CONFIG_LGE_PM
sanity_out:

    pm8058_xoadc_clear_recentQ();

	ret = adc_channel_close(h);

    wake_unlock(&adc_wake_lock);
	if (ret) {
		pr_err("%s: couldnt close channel %d ret=%d\n",
						__func__, channel, ret);
	}
    
    if(channel == CHANNEL_ADC_BATT_THERM)
    {
        printk(KERN_ERR "============== batt temp adc read fail so default temp ===============\n");
	    if (mv_reading)
		    *mv_reading = MSM_CHARGER_GAUGE_MISSING_TEMP_ADC;
        return MSM_CHARGER_GAUGE_MISSING_TEMP;
    }
    else if(channel == CHANNEL_ADC_ACC)
    {
        printk(KERN_ERR "============== ACC adc read fail so default usb ===============\n");
        return CHANNEL_ADC_ACC_MISSING;
    }
    else
    {
        printk(KERN_ERR "============== adc read fail  ===============\n");
	    return -EINVAL;
    }
#endif

}

#define BATT_THERM_OPEN_MV  2000
static int pm8058_is_battery_present(void)
{
	int mv_reading;
	mv_reading = 0;
	batt_read_adc(CHANNEL_ADC_BATT_THERM, &mv_reading);
	pr_err("%s: therm_raw is %d\n", __func__, mv_reading);
#ifdef CONFIG_LGE_PM_IGNORE_BATT_THERM_FOR_EVENTS
	mv_reading = 960;
#endif
	if (mv_reading > 0 && mv_reading < BATT_THERM_OPEN_MV)
		return 1;
    return 0;
}

static int pm8058_get_battery_temperature(void)
{
#ifdef CONFIG_LGE_PM_IGNORE_BATT_THERM_FOR_EVENTS
	return 38;
#else
	return batt_read_adc(CHANNEL_ADC_BATT_THERM, NULL);
#endif
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static int pm8058_get_battery_temperature_adc(void)
{
	int mv_reading;

	mv_reading = 0;
	batt_read_adc(CHANNEL_ADC_BATT_THERM, &mv_reading);
	pr_err("%s: therm_raw is %d\n", __func__, mv_reading);
#ifdef CONFIG_LGE_PM_IGNORE_BATT_THERM_FOR_EVENTS
	return 960;
#else
	return mv_reading;
#endif

}
#endif

int testmode_battery_read_temperature_adc(void)
{
	int batt_adc;
	batt_adc = pm8058_get_battery_temperature_adc();
	return batt_adc;
}
EXPORT_SYMBOL(testmode_battery_read_temperature_adc);

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/* kiwone.seo@lge.com, we will check later. because temp is not matching */
#define BATT_THERM_OPERATIONAL_MAX_CELCIUS 100	//temporarily modify 40 -> 60 sungwook
#else
#define BATT_THERM_OPERATIONAL_MAX_CELCIUS 40
#endif
#define BATT_THERM_OPERATIONAL_MIN_CELCIUS 0
static int pm8058_is_battery_temp_within_range(void)
{
	int therm_celcius;

	therm_celcius = pm8058_get_battery_temperature();
	pr_err("%s: therm_celcius is %d\n", __func__, therm_celcius);
#ifndef CONFIG_LGE_FUEL_GAUGE 
	if (therm_celcius > 0
		&& therm_celcius > BATT_THERM_OPERATIONAL_MIN_CELCIUS
		&& therm_celcius < BATT_THERM_OPERATIONAL_MAX_CELCIUS)
#endif
		return 1;

	return 0;
}


static int pm8058_is_battery_id_valid(void)
{
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
    u16 batt_id = 0;

    batt_id = battery_info_get();

//	pr_err("%s: batt_id is %x\n", __func__, batt_id);

    if(batt_id == BATT_UNKNOWN)
   	{   	    
/* ws.chang@lge.com 2011-06-10, Temporarily Battery ID TRUE, Battery ID Pin is Ground in Cayman, This Function must be removed in Cayman */
	    return 1; //0;
   	}
    else
        return 1;
#else
	int batt_id_mv;

	batt_id_mv = batt_read_adc(CHANNEL_ADC_BATT_ID, NULL);
	pr_err("%s: batt_id_mv is %d\n", __func__, batt_id_mv);

	/*
	 * The readings are not in range
	 * assume battery is present for now
	 */
	return 1;

	if (batt_id_mv > 0
		&& batt_id_mv > BATT_ID_MIN_MV
		&& batt_id_mv < BATT_ID_MAX_MV)
		return 1;

	return 0;
#endif
}

/* returns voltage in mV */
#ifdef CONFIG_LGE_FUEL_GAUGE
extern int max17040_get_battery_mvolts(void);
extern void max17040_update_rcomp(int temperature);
#endif
static int pm8058_get_battery_mvolts(void)
{
	int vbatt_mv;

#ifdef CONFIG_LGE_FUEL_GAUGE
//	max17040_update_rcomp(batt_read_adc(CHANNEL_ADC_BATT_THERM, NULL));
	max17040_update_rcomp(g_temp);
	vbatt_mv = max17040_get_battery_mvolts();
#else
	vbatt_mv = batt_read_adc(CHANNEL_ADC_VBATT, NULL);
#endif
	pr_debug("%s: vbatt_mv is %d\n", __func__, vbatt_mv);
	if (vbatt_mv > 0)
		return vbatt_mv;
	/*
	 * return 0 to tell the upper layers
	 * we couldnt read the battery voltage
	 */
	return 0;
}

static int msm_battery_gague_alarm_notify(struct notifier_block *nb,
		unsigned long status, void *unused)
{
	int rc;

	pr_info("%s: status: %lu\n", __func__, status);

	switch (status) {
	case 0:
		dev_err(pm8058_chg.dev,
			"%s: spurious interrupt\n", __func__);
		break;
	/* expected case - trip of low threshold */
	case 1:
#ifdef CONFIG_LGE_PM_BATTERY_ALARM
		if (is_chg_plugged_in()) {
			rc = pm8058_batt_alarm_state_set(0, 0);
			if (rc)
				dev_err(pm8058_chg.dev,
					"%s: unable to set alarm state\n", __func__);
			msm_charger_notify_event(NULL, CHG_BATT_NEEDS_RECHARGING);
		}
#else
		rc = pm8058_batt_alarm_state_set(0, 0);
		if (rc)
			dev_err(pm8058_chg.dev,
				"%s: unable to set alarm state\n", __func__);
		msm_charger_notify_event(NULL, CHG_BATT_NEEDS_RECHARGING);
#endif
		break;
	case 2:
		dev_err(pm8058_chg.dev,
			"%s: trip of high threshold\n", __func__);
		break;
	default:
		dev_err(pm8058_chg.dev,
			"%s: error received\n", __func__);
	};

	return 0;
}

static int pm8058_monitor_for_recharging(void)
{
	/* enable low comparator */
#ifdef CONFIG_LGE_PM	
/* kiwone.seo@lge.com 2011-07-20, when the charger is invalid, we must stop charging animation and stop charging */
/* work around code, we remove this */
#if 0
	uint32_t fsm_state_check = get_fsm_state();
	pr_err("%s fsm_state_check= %d,chg_plugged?=%d\n",__func__,fsm_state_check,is_chg_plugged_in());
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ALARM
	if (!is_chg_plugged_in())
		return 0;
#endif
/* work around code, we remove this */
#if 0
	if(fsm_state_check == 3)
		return 0;
	else
#endif		
		return pm8058_batt_alarm_state_set(1, 0);
#else
	return pm8058_batt_alarm_state_set(1, 0);
#endif
}

static struct msm_battery_gauge pm8058_batt_gauge = {
	.get_battery_mvolts = pm8058_get_battery_mvolts,
	.get_battery_temperature = pm8058_get_battery_temperature,
	.is_battery_present = pm8058_is_battery_present,
	.is_battery_temp_within_range = pm8058_is_battery_temp_within_range,
	.is_battery_id_valid = pm8058_is_battery_id_valid,
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO	
	.get_battery_temperature_adc = pm8058_get_battery_temperature_adc,
#endif
	.monitor_for_recharging = pm8058_monitor_for_recharging,
#ifdef CONFIG_LGE_PM
	.get_hw_fsm_state = get_fsm_state,
#endif
};

static int pm8058_usb_voltage_lower_limit(void)
{
	u8 temp, old;
	int ret = 0;

	temp = 0x10;
	ret |= pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TEST, &temp, 1);
	ret |= pm8058_read(pm8058_chg.pm_chip, PM8058_CHG_TEST, &old, 1);
	old = old & ~BIT(IGNORE_LL);
	temp = 0x90  | (0xF & old);
	ret |= pm8058_write(pm8058_chg.pm_chip, PM8058_CHG_TEST, &temp, 1);

	return ret;
}

static int __devinit pm8058_charger_probe(struct platform_device *pdev)
{
	struct pm8058_chip *pm_chip;
	int rc = 0;

	pm_chip = platform_get_drvdata(pdev);
	if (pm_chip == NULL) {
		pr_err("%s:no parent data passed in.\n", __func__);
		return -EFAULT;
	}

	pm8058_chg.pm_chip = pm_chip;
	pm8058_chg.pdata = pdev->dev.platform_data;
	pm8058_chg.dev = &pdev->dev;

	rc = request_irqs(pdev);
	if (rc) {
		pr_err("%s: couldnt register interrupts\n", __func__);
		goto out;
	}

	rc = pm8058_usb_voltage_lower_limit();
	if (rc) {
		pr_err("%s: couldnt set ignore lower limit bit to 0\n",
								__func__);
		goto free_irq;
	}

	rc = msm_charger_register(&usb_hw_chg);
	if (rc) {
		pr_err("%s: msm_charger_register failed ret=%d\n",
							__func__, rc);
		goto free_irq;
	}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/* kiwone.seo@lge.com, we must enable batt temp monitoring. because the interrupt never be occured 
and must change register value in HW. */
	pm_chg_batt_temp_disable(0);
#endif

	msm_battery_gauge_register(&pm8058_batt_gauge);
	__dump_chg_regs();

	create_debugfs_entries();
	INIT_DELAYED_WORK(&pm8058_chg.veoc_begin_work, veoc_begin_work);
	INIT_DELAYED_WORK(&pm8058_chg.check_vbat_low_work, vbat_low_work);
	INIT_DELAYED_WORK(&pm8058_chg.chg_done_check_work, chg_done_check_work);
	INIT_DELAYED_WORK(&pm8058_chg.charging_check_work, charging_check_work);

	/* determine what state the charger is in */
	pm8058_chg_determine_initial_state();

	pm8058_chg_enable_irq(BATTTEMP_IRQ);
	pm8058_chg_enable_irq(BATTCONNECT_IRQ);

	rc = pm8058_batt_alarm_state_set(0, 0);
	if (rc) {
		pr_err("%s: unable to set batt alarm state\n", __func__);
		goto free_irq;
	}

	/*
	 * The batt-alarm driver requires sane values for both min / max,
	 * regardless of whether they're both activated.
	 */
#ifdef CONFIG_LGE_CHARGER_VOLTAGE_CURRENT_SCENARIO
	rc = pm8058_batt_alarm_threshold_set(resume_mv, 4200);
#else
	rc = pm8058_batt_alarm_threshold_set(resume_mv, 4300);
#endif
	if (rc) {
		pr_err("%s: unable to set batt alarm threshold\n", __func__);
		goto free_irq;
	}

	rc = pm8058_batt_alarm_hold_time_set(PM8058_BATT_ALARM_HOLD_TIME_16_MS);
	if (rc) {
		pr_err("%s: unable to set batt alarm hold time\n", __func__);
		goto free_irq;
	}

	/* PWM enabled at 2Hz */
	rc = pm8058_batt_alarm_pwm_rate_set(1, 7, 4);
	if (rc) {
		pr_err("%s: unable to set batt alarm pwm rate\n", __func__);
		goto free_irq;
	}

	rc = pm8058_batt_alarm_register_notifier(&alarm_notifier);
	if (rc) {
		pr_err("%s: unable to register alarm notifier\n", __func__);
		goto free_irq;
	}

	pm8058_chg.inited = 1;

	pr_err("%s: pm8058_charger_probe OK\n", __func__);

	return 0;

free_irq:
	free_irqs();
out:
	return rc;
}

static int __devexit pm8058_charger_remove(struct platform_device *pdev)
{
	struct pm8058_charger_chip *chip = platform_get_drvdata(pdev);
	int rc;

	msm_charger_notify_event(&usb_hw_chg, CHG_REMOVED_EVENT);
	msm_charger_unregister(&usb_hw_chg);
	cancel_delayed_work_sync(&pm8058_chg.veoc_begin_work);
	cancel_delayed_work_sync(&pm8058_chg.check_vbat_low_work);
	cancel_delayed_work_sync(&pm8058_chg.charging_check_work);
	free_irqs();
	remove_debugfs_entries();
	kfree(chip);

	rc = pm8058_batt_alarm_state_set(0, 0);
	if (rc)
		pr_err("%s: unable to set batt alarm state\n", __func__);

	rc |= pm8058_batt_alarm_unregister_notifier(&alarm_notifier);
	if (rc)
		pr_err("%s: unable to register alarm notifier\n", __func__);
	return rc;
}

static struct platform_driver pm8058_charger_driver = {
	.probe = pm8058_charger_probe,
	.remove = __devexit_p(pm8058_charger_remove),
	.driver = {
		   .name = "pm8058-charger",
		   .owner = THIS_MODULE,
	},
};

static int __init pm8058_charger_init(void)
{
	return platform_driver_register(&pm8058_charger_driver);
}

static void __exit pm8058_charger_exit(void)
{
	platform_driver_unregister(&pm8058_charger_driver);
}

late_initcall(pm8058_charger_init);
module_exit(pm8058_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8058 BATTERY driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8058_charger");
