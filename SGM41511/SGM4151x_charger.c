/*
* SGM4151x battery charging driver
*
* Copyright (C) 2019 SG MICRO
*
* This package is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.

* THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
* WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*/

#define pr_fmt(fmt) "SGM4151x: %s: " fmt, __func__
//********************************************************************************************************************************************
//created by yangwensen@20200401
#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#include "power_supply.h"
#include "errno.h"
#include <stdio.h>

#define LOG_TAG                 "SGM4151x"
#define LOG_LVL                 LOG_LVL_DBG
#include <ulog.h>
//********************************************************************************************************************************************
#define SGM4151X_I2C_NAME       "i2c2"
#define SGM4151X_I2C_ADDR       0x55
//********************************************************************************************************************************************
#ifndef BIT
    #define BIT(x)              (1<<(x))
#endif

typedef unsigned char           bool;
typedef uint8_t                 u8;
typedef int32_t                 s32;
typedef rt_base_t               ssize_t;
typedef enum irqreturn {IRQ_NONE, IRQ_HANDLED, IRQ_WAKE_THREAD,}irqreturn_t;

#define pr_err                  LOG_E
#define dev_err(a,...)          LOG_E(__VA_ARGS__)
#define printk                  rt_kprintf
#define devm_kzalloc            rt_malloc

#define mutex                   rt_mutex
#define mutex_lock(a)           rt_mutex_take(a,RT_WAITING_FOREVER)
#define mutex_unlock(a)         rt_mutex_release(a)

#define spin_lock_irqsave(a,b)          b = rt_hw_interrupt_disable()
#define spin_unlock_irqrestore(a,b)     rt_hw_interrupt_enable(b)

#define EXPORT_SYMBOL_GPL(a)    MSH_CMD_EXPORT(a,#a)

#ifndef true
    #define true                RT_TRUE
    #define false               RT_FALSE
#endif

#define container_of            rt_container_of

#define min(a,b)                ((a)<(b)?(a):(b))
#define ARRAY_SIZE(a)           (sizeof(a)/sizeof(a[0]))
//********************************************************************************************************************************************
static int SGM4151x_charger_probe(struct rt_i2c_bus_device *client);
//********************************************************************************************************************************************
struct rt_i2c_bus_device *i2c_device;
//********************************************************************************************************************************************
//by yangwensen@20200401
static s32 i2c_smbus_read_byte_data(struct rt_i2c_bus_device *client, u8 reg)
{
    struct rt_i2c_msg msg;

    RT_ASSERT(client != RT_NULL);

    msg.addr = SGM4151X_I2C_ADDR;
    msg.flags = RT_I2C_WR;
    msg.len = 1;
    msg.buf = &reg;

    if( rt_i2c_transfer(client, &msg, 1) != 1)
        return -1;
    
    msg.flags = RT_I2C_RD;
    msg.len = 1;
    
    if( rt_i2c_transfer(client, &msg, 1) != 1)
        return -2;
    
    return 1;
}
//********************************************************************************************************************************************
//by yangwensen@20200401
static s32 i2c_smbus_write_byte_data(struct rt_i2c_bus_device *client, u8 reg, u8 value)
{
    struct rt_i2c_msg msg;
    uint8_t buf[2];

    RT_ASSERT(client != RT_NULL);

    buf[0] = reg;
    buf[1] = value;
    
    msg.addr = SGM4151X_I2C_ADDR;
    msg.flags = RT_I2C_WR;
    msg.len = 2;
    msg.buf = buf;

    if( rt_i2c_transfer(client, &msg, 1) != 1)
        return -1;
    
    return 1;
}
//********************************************************************************************************************************************
//by yangwensen@20181028
static void TaskFuelGauge(void* parameter)
{
    i2c_device = (struct rt_i2c_bus_device *)rt_device_find(SGM4151X_I2C_NAME);
    if(i2c_device == RT_NULL)
    {
        LOG_E("i2c bus device %s not found!\r\n", SGM4151X_I2C_NAME);
    }
    
	SGM4151x_charger_probe(i2c_device);
	while(1)
	{
        rt_thread_mdelay(500);
//		bq27xxx_battery_update(&battery_info);
//		show_battery_info();
	}
}
//********************************************************************************************************************************************
//by yangwensen@20181028
extern void bms_startup(void)
{
    rt_thread_t thread_id;

	thread_id = rt_thread_create("BMS",
		TaskFuelGauge, RT_NULL,
		1024,	//stack
		10,		//Priority
		5);		//TimeSlice

	if (thread_id != RT_NULL)rt_thread_startup(thread_id);
}
//********************************************************************************************************************************************
void power_supply_changed(struct power_supply *psy)
{
	unsigned long flags;

//	dev_dbg(&psy->dev, "%s\n", __func__);

	spin_lock_irqsave(&psy->changed_lock, flags);
	psy->changed = true;
//	pm_stay_awake(&psy->dev);
	spin_unlock_irqrestore(&psy->changed_lock, flags);
//	schedule_work(&psy->changed_work);
}
//EXPORT_SYMBOL_GPL(power_supply_changed);
//********************************************************************************************************************************************
#if 0	
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>
#endif

#include "SGM4151x_reg.h"
#include "SGM4151x.h"

#if 1
    #undef pr_debug
    #define pr_debug LOG_D
    #undef pr_info
    #define pr_info LOG_I
    #undef dev_dbg
    #define dev_dbg LOG_D
#else
    #undef pr_info
    #define pr_info pr_debug
#endif
	
enum SGM4151x_vbus_type {
    SGM4151x_VBUS_NONE = REG08_VBUS_TYPE_NONE,
    SGM4151x_VBUS_USB = REG08_VBUS_TYPE_USB,
    SGM4151x_VBUS_ADAPTER = REG08_VBUS_TYPE_ADAPTER,
    SGM4151x_VBUS_OTG = REG08_VBUS_TYPE_OTG,
};
	
enum SGM4151x_part_no {
	SGM41509 = 0x00,
	SGM41510 = 0x00,
	SGM41511 = 0x02,
	SGM41512 = 0x07,
};

enum {
    USER = BIT(0),
    JEITA = BIT(1),
    BATT_FC = BIT(2),
    BATT_PRES = BIT(3),
};

enum wakeup_src {
    WAKEUP_SRC_MONITOR = 0,
    WAKEUP_SRC_JEITA,
    WAKEUP_SRC_MAX,
};
	
#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))
struct SGM4151x_wakeup_source {
    //	struct wakeup_source source;
    unsigned long enabled_bitmap;
    //	spinlock_t ws_lock;
};
	
enum SGM4151x_charge_state {
    CHARGE_STATE_IDLE = REG08_CHRG_STAT_IDLE,
    CHARGE_STATE_PRECHG = REG08_CHRG_STAT_PRECHG,
    CHARGE_STATE_FASTCHG = REG08_CHRG_STAT_FASTCHG,
    CHARGE_STATE_CHGDONE = REG08_CHRG_STAT_CHGDONE,
};

struct SGM4151x_otg_regulator {
//	struct regulator_desc rdesc;
    struct regulator_dev *rdev;
};
	
	
struct SGM4151x 
{
	struct device *dev;
	struct rt_i2c_bus_device *client;
	
	enum SGM4151x_part_no part_no;
	int revision;
	
	int gpio_ce;
	
	int vbus_type;
	
	int status;
	
	struct rt_mutex data_lock;
	struct rt_mutex i2c_rw_lock;
	struct rt_mutex profile_change_lock;
	struct rt_mutex charging_disable_lock;
	struct rt_mutex irq_complete;
	
	struct SGM4151x_wakeup_source SGM4151x_ws;
	
	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;
	
	bool batt_present;
	bool usb_present;
	
	bool batt_full;
	
	bool charge_enabled;/* Register bit status */
	bool otg_enabled;
	bool batfet_enabled;
	bool in_hiz;
	bool dis_safety;
	
	bool vindpm_triggered;
	bool iindpm_triggered;
	
	bool in_therm_regulation;
	bool in_vsys_regulation;
	
	bool power_good;
	bool vbus_good;
	
	bool topoff_active;
	bool acov_triggered;
	
	/* if use software jeita in case of NTC is connected to gauge */
	bool software_jeita_supported;
	bool jeita_active;
	
	bool batt_hot;
	bool batt_cold;
	bool batt_warm;
	bool batt_cool;
	
	int batt_hot_degc;
	int batt_warm_degc;
	int batt_cool_degc;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;
	
	int batt_cool_ma;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;
	
	
	int batt_temp;
	
	int jeita_ma;
	int jeita_mv;
	
	unsigned int thermal_levels;
	unsigned int therm_lvl_sel;
	unsigned int *thermal_mitigation;
	
	int usb_psy_ma;
	int charge_state;
	int charging_disabled_status;
	
	int fault_status;
	
	int skip_writes;
	int skip_reads;
	
	struct SGM4151x_platform_data* platform_data;
	
//	struct delayed_work discharge_jeita_work; /*normal no charge mode*/
//	struct delayed_work charge_jeita_work; /*charge mode jeita work*/
	
//	struct alarm jeita_alarm;
	
	struct dentry *debug_root;
	
	struct SGM4151x_otg_regulator otg_vreg;
	
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply batt_psy;
};
	
static int BatteryTestStatus_enable = 0;

static int __SGM4151x_read_reg(struct SGM4151x* SGM, u8 reg, u8 *data)
{
    s32 ret;

    ret = i2c_smbus_read_byte_data(SGM->client, reg);
    if (ret < 0) {
        pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
        return ret;
    }

    *data = (u8)ret;

    return 0;
}
	
static int __SGM4151x_write_reg(struct SGM4151x* SGM, int reg, u8 val)
{
    s32 ret;

    ret = i2c_smbus_write_byte_data(SGM->client, reg, val);
    if (ret < 0) {
        pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
        val, reg, ret);
        return ret;
    }
    return 0;
}
	
static int SGM4151x_read_byte(struct SGM4151x *SGM, u8 *data, u8 reg)
{
    int ret;

    if (SGM->skip_reads) {
        *data = 0;
        return 0;
    }

    mutex_lock(&SGM->i2c_rw_lock);
    ret = __SGM4151x_read_reg(SGM, reg, data);
    mutex_unlock(&SGM->i2c_rw_lock);

    return ret;
}
	
#if 0	
static int SGM4151x_write_byte(struct SGM4151x *SGM, u8 reg, u8 data)
{
    int ret;

    if (SGM->skip_writes) {
        return 0;
    }

    mutex_lock(&SGM->i2c_rw_lock);
    ret = __SGM4151x_write_reg(SGM, reg, data);
    mutex_unlock(&SGM->i2c_rw_lock);

    if (ret) {
        pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
    }

    return ret;
}
#endif	
	
static int SGM4151x_update_bits(struct SGM4151x *SGM, u8 reg, u8 mask, u8 data)
{
    int ret;
    u8 tmp;

    if (SGM->skip_reads || SGM->skip_writes)
        return 0;

    mutex_lock(&SGM->i2c_rw_lock);
    ret = __SGM4151x_read_reg(SGM, reg, &tmp);
    if (ret) {
        pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
        goto out;
    }

    tmp &= ~mask;
    tmp |= data & mask;

    ret = __SGM4151x_write_reg(SGM, reg, tmp);
    if (ret) {
        pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
    }

    out:
        mutex_unlock(&SGM->i2c_rw_lock);
    return ret;
}

#if 0
static void SGM4151x_stay_awake(struct SGM4151x_wakeup_source *source, enum wakeup_src wk_src)
{
	unsigned long flags;
	
	spin_lock_irqsave(&source->ws_lock, flags);
	
//	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap)) {
//	__pm_stay_awake(&source->source);
//	pr_debug("enabled source %s, wakeup_src %d\n",
//	source->source.name, wk_src);
//	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
}
#endif

static void SGM4151x_relax(struct SGM4151x_wakeup_source *source, enum wakeup_src wk_src)
{
	unsigned long flags;
	
	spin_lock_irqsave(&source->ws_lock, flags);
//	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
//	!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
//	__pm_relax(&source->source);
//	pr_debug("disabled source %s\n", source->source.name);
//	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
	
//	pr_debug("relax source %s, wakeup_src %d\n",
//	source->source.name, wk_src);
}

static void SGM4151x_wakeup_src_init(struct SGM4151x *SGM)
{
//	spin_lock_init(&SGM->SGM4151x_ws.ws_lock);
//	wakeup_source_init(&SGM->SGM4151x_ws.source, "SGM4151x");
}
	
	
#if 0	
static int SGM4151x_enable_otg(struct SGM4151x *SGM)
{
    u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_01,
    REG01_OTG_CONFIG_MASK, val);

}
#endif

#if 0
static int SGM4151x_disable_otg(struct SGM4151x *SGM)
{
    u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_01,
    REG01_OTG_CONFIG_MASK, val);

}
#endif

static int SGM4151x_enable_charger(struct SGM4151x *SGM)
{
    int ret;
    u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

    ret = SGM4151x_update_bits(SGM, SGM4151X_REG_01, REG01_CHG_CONFIG_MASK, val);

    return ret;
}

static int SGM4151x_disable_charger(struct SGM4151x *SGM)
{
    int ret;
    u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

    ret = SGM4151x_update_bits(SGM, SGM4151X_REG_01, REG01_CHG_CONFIG_MASK, val);
    return ret;
}
	
int SGM4151x_set_chargecurrent(struct SGM4151x *SGM, int curr)
{
    u8 ichg;

    if (curr < REG02_ICHG_BASE)
    curr = REG02_ICHG_BASE;

    ichg = (curr - REG02_ICHG_BASE)/REG02_ICHG_LSB;
    return SGM4151x_update_bits(SGM, SGM4151X_REG_02, REG02_ICHG_MASK, 
    ichg << REG02_ICHG_SHIFT);

}
	
int SGM4151x_set_term_current(struct SGM4151x *SGM, int curr)
{
    u8 iterm;

    if (curr < REG03_ITERM_BASE)
    curr = REG03_ITERM_BASE;

    iterm = (curr - REG03_ITERM_BASE) / REG03_ITERM_LSB;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_03, REG03_ITERM_MASK, 
    iterm << REG03_ITERM_SHIFT);
}
	
int SGM4151x_set_prechg_current(struct SGM4151x *SGM, int curr)
{
    u8 iprechg;

    if (curr < REG03_IPRECHG_BASE)
    curr = REG03_IPRECHG_BASE;

    iprechg = (curr - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_03, REG03_IPRECHG_MASK, 
    iprechg << REG03_IPRECHG_SHIFT);
    }

    int SGM4151x_set_chargevolt(struct SGM4151x *SGM, int volt)
    {
    u8 val;

    if (volt < REG04_VREG_BASE)
    volt = REG04_VREG_BASE;

    val = (volt - REG04_VREG_BASE)/REG04_VREG_LSB;
    return SGM4151x_update_bits(SGM, SGM4151X_REG_04, REG04_VREG_MASK, 
    val << REG04_VREG_SHIFT);
}
	
int SGM4151x_set_input_volt_limit(struct SGM4151x *SGM, int volt)
{
    u8 val;

    if (volt < REG06_VINDPM_BASE)
    volt = REG06_VINDPM_BASE;

    val = (volt - REG06_VINDPM_BASE) / REG06_VINDPM_LSB;
    return SGM4151x_update_bits(SGM, SGM4151X_REG_06, REG06_VINDPM_MASK, 
    val << REG06_VINDPM_SHIFT);
}
	
int SGM4151x_set_input_current_limit(struct SGM4151x *SGM, int curr)
{
    u8 val;

    if (curr < REG00_IINLIM_BASE)
    curr = REG00_IINLIM_BASE;

    val = (curr - REG00_IINLIM_BASE) / REG00_IINLIM_LSB;
    return SGM4151x_update_bits(SGM, SGM4151X_REG_00, REG00_IINLIM_MASK, 
    val << REG00_IINLIM_SHIFT);
}
	
	
int SGM4151x_set_watchdog_timer(struct SGM4151x *SGM, u8 timeout)
{
    u8 temp;

    temp = (u8)(((timeout - REG05_WDT_BASE) / REG05_WDT_LSB) << REG05_WDT_SHIFT);

    return SGM4151x_update_bits(SGM, SGM4151X_REG_05, REG05_WDT_MASK, temp); 
}
//EXPORT_SYMBOL_GPL(SGM4151x_set_watchdog_timer);
	
int SGM4151x_disable_watchdog_timer(struct SGM4151x *SGM)
{
	u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;
	
	return SGM4151x_update_bits(SGM, SGM4151X_REG_05, REG05_WDT_MASK, val);
}
//EXPORT_SYMBOL_GPL(SGM4151x_disable_watchdog_timer);
	
int SGM4151x_reset_watchdog_timer(struct SGM4151x *SGM)
{
    u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_01, REG01_WDT_RESET_MASK, val);
}
//EXPORT_SYMBOL_GPL(SGM4151x_reset_watchdog_timer);
	
int SGM4151x_reset_chip(struct SGM4151x *SGM)
{
    int ret;
    u8 val = REG0B_REG_RESET << REG0B_REG_RESET_SHIFT;

    ret = SGM4151x_update_bits(SGM, SGM4151X_REG_0B, REG0B_REG_RESET_MASK, val);
    return ret;
}
//EXPORT_SYMBOL_GPL(SGM4151x_reset_chip);
	
int SGM4151x_enter_hiz_mode(struct SGM4151x *SGM)
{
    u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_00, REG00_ENHIZ_MASK, val);

}
//EXPORT_SYMBOL_GPL(SGM4151x_enter_hiz_mode);
	
int SGM4151x_exit_hiz_mode(struct SGM4151x *SGM)
{

    u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_00, REG00_ENHIZ_MASK, val);

}
//EXPORT_SYMBOL_GPL(SGM4151x_exit_hiz_mode);
	
int SGM4151x_get_hiz_mode(struct SGM4151x *SGM, u8 *state)
{
    u8 val;
    int ret;

    ret = SGM4151x_read_byte(SGM, &val, SGM4151X_REG_00);
    if (ret)
    return ret;
    *state = (val & REG00_ENHIZ_MASK) >> REG00_ENHIZ_SHIFT;

    return 0;
}
//EXPORT_SYMBOL_GPL(SGM4151x_get_hiz_mode);
	
#if 0	
static int SGM4151x_enable_term(struct SGM4151x* SGM, bool enable)
{
    u8 val;
    int ret;

    if (enable)
    val = REG05_TERM_ENABLE << REG05_EN_TERM_SHIFT;
    else
    val = REG05_TERM_DISABLE << REG05_EN_TERM_SHIFT;

    ret = SGM4151x_update_bits(SGM, SGM4151X_REG_05, REG05_EN_TERM_MASK, val);

    return ret;
}
//EXPORT_SYMBOL_GPL(SGM4151x_enable_term);
#endif

int SGM4151x_set_boost_current(struct SGM4151x *SGM, int curr)
{
    u8 val;

    val = REG02_BOOST_LIM_0P5A;
    if (curr == BOOSTI_1200)
    val = REG02_BOOST_LIM_1P2A;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_02, REG02_BOOST_LIM_MASK, 
    val << REG02_BOOST_LIM_SHIFT);
}
	
int SGM4151x_set_boost_voltage(struct SGM4151x *SGM, int volt)
{
    u8 val;

    if (volt == BOOSTV_4850)
    val = REG06_BOOSTV_4P85V;
    else if (volt == BOOSTV_5150)
    val = REG06_BOOSTV_5P15V;
    else if (volt == BOOSTV_5300)
    val = REG06_BOOSTV_5P3V;
    else
    val = REG06_BOOSTV_5V;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_06, REG06_BOOSTV_MASK, 
    val << REG06_BOOSTV_SHIFT);
}
	
static int SGM4151x_set_acovp_threshold(struct SGM4151x *SGM, int volt)
{
    u8 val;

    if (volt == VAC_OVP_14300)
    val = REG06_OVP_14P3V;
    else if (volt == VAC_OVP_10500)
    val = REG06_OVP_10P5V;
    else if (volt == VAC_OVP_6200)
    val = REG06_OVP_6P2V;
    else
    val = REG06_OVP_5P5V;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_06, REG06_OVP_MASK, 
    val << REG06_OVP_SHIFT);
}
	
static int SGM4151x_set_stat_ctrl(struct SGM4151x *SGM, int ctrl)
{
    u8 val;

    val = ctrl;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_00, REG00_STAT_CTRL_MASK, 
    val << REG00_STAT_CTRL_SHIFT);
}
	
static int SGM4151x_set_int_mask(struct SGM4151x *SGM, int mask)
{
    u8 val;

    val = mask;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_0A, REG0A_INT_MASK_MASK,
    val << REG0A_INT_MASK_SHIFT);
}
	
static int SGM4151x_enable_batfet(struct SGM4151x *SGM)
{
	const u8 val = REG07_BATFET_ON << REG07_BATFET_DIS_SHIFT;
	
	return SGM4151x_update_bits(SGM, SGM4151X_REG_07, REG07_BATFET_DIS_MASK,
	val);
}
//EXPORT_SYMBOL_GPL(SGM4151x_enable_batfet);

#if 0
static int SGM4151x_disable_batfet(struct SGM4151x *SGM)
{
    const u8 val = REG07_BATFET_OFF << REG07_BATFET_DIS_SHIFT;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_07, REG07_BATFET_DIS_MASK,
    val);
}
//EXPORT_SYMBOL_GPL(SGM4151x_disable_batfet);
#endif

#if 0
static int SGM4151x_set_batfet_delay(struct SGM4151x *SGM, uint8_t delay)
{
    u8 val;

    if (delay == 0)
    val = REG07_BATFET_DLY_0S;
    else
    val = REG07_BATFET_DLY_10S;

    val <<= REG07_BATFET_DLY_SHIFT;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_07, REG07_BATFET_DLY_MASK, val);
}
//EXPORT_SYMBOL_GPL(SGM4151x_set_batfet_delay);
#endif

static int SGM4151x_set_vdpm_bat_track(struct SGM4151x *SGM)
{
	const u8 val = REG07_VDPM_BAT_TRACK_200MV << REG07_VDPM_BAT_TRACK_SHIFT;
	
	return SGM4151x_update_bits(SGM, SGM4151X_REG_07, REG07_VDPM_BAT_TRACK_MASK,
	val);
}
//EXPORT_SYMBOL_GPL(SGM4151x_set_vdpm_bat_track);

#if 0
static int SGM4151x_enable_safety_timer(struct SGM4151x *SGM)
{
	const u8 val = REG05_CHG_TIMER_ENABLE << REG05_EN_TIMER_SHIFT;
	
	return SGM4151x_update_bits(SGM, SGM4151X_REG_05, REG05_EN_TIMER_MASK,
	val);
}
//EXPORT_SYMBOL_GPL(SGM4151x_enable_safety_timer);
#endif

#if 0
static int SGM4151x_disable_safety_timer(struct SGM4151x *SGM)
{
    const u8 val = REG05_CHG_TIMER_DISABLE << REG05_EN_TIMER_SHIFT;

    return SGM4151x_update_bits(SGM, SGM4151X_REG_05, REG05_EN_TIMER_MASK, val);
}
//EXPORT_SYMBOL_GPL(SGM4151x_disable_safety_timer);
#endif

static int SGM4151x_charging_disable(struct SGM4151x *SGM, int reason, int disable)
{
    int ret = 0;
    int disabled;

    mutex_lock(&SGM->charging_disable_lock);

    disabled = SGM->charging_disabled_status;

    pr_err("reason=%d requested_disable=%d disabled_status=%d\n",
    reason, disable, disabled);

    if (disable == true)
        disabled |= reason;
    else
        disabled &= ~reason;

    if (disabled && SGM->charge_enabled)
        ret = SGM4151x_disable_charger(SGM);
    else if (!disabled && !SGM->charge_enabled)
        ret = SGM4151x_enable_charger(SGM);

    if (ret) {
        pr_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
        ret, reason);
    } else {
        SGM->charging_disabled_status = disabled;
        mutex_lock(&SGM->data_lock);
        SGM->charge_enabled = !disabled;
        mutex_unlock(&SGM->data_lock);
    }
    mutex_unlock(&SGM->charging_disable_lock);

    return ret;
}
	
#if 0	
	static struct power_supply *get_bms_psy(struct SGM4151x *SGM)
	{
	if (SGM->bms_psy)
	return SGM->bms_psy;
	SGM->bms_psy = power_supply_get_by_name("bms");
	if (!SGM->bms_psy)
	pr_debug("bms power supply not found\n");
	
	return SGM->bms_psy;
	}
#endif

static int SGM4151x_get_batt_property(struct SGM4151x *SGM,
enum power_supply_property psp,
union power_supply_propval *val)
{
    struct power_supply *bms_psy;// = get_bms_psy(SGM);

    int ret;

    if (!bms_psy)
        return -EINVAL;

    //	ret = bms_psy->get_property(bms_psy, psp, val);

    return ret;
}
	
static inline bool is_device_suspended(struct SGM4151x *SGM);
static int SGM4151x_get_prop_charge_type(struct SGM4151x *SGM)
{
	u8 val = 0;
	if (is_device_suspended(SGM))
	    return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	
	SGM4151x_read_byte(SGM, &val, SGM4151X_REG_08);
	val &= REG08_CHRG_STAT_MASK;
	val >>= REG08_CHRG_STAT_SHIFT;
	switch (val) {
    	case CHARGE_STATE_FASTCHG:
        	return POWER_SUPPLY_CHARGE_TYPE_FAST;
    	case CHARGE_STATE_PRECHG:
        	return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
    	case CHARGE_STATE_CHGDONE:
    	case CHARGE_STATE_IDLE:
        	return POWER_SUPPLY_CHARGE_TYPE_NONE;
    	default:
        	return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}
	
static int SGM4151x_get_prop_batt_present(struct SGM4151x *SGM)
{
//	union power_supply_propval batt_prop = {0,};
	int ret;
	
//	ret = SGM4151x_get_batt_property(SGM,
//	POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret){
    	mutex_lock(&SGM->data_lock);
//	SGM->batt_present = batt_prop.intval;
    	mutex_unlock(&SGM->data_lock);
	}
	return ret;
}
	
static int SGM4151x_get_prop_batt_full(struct SGM4151x *SGM)
{
//	union power_supply_propval batt_prop = {0,};
	int ret;

#if 0
	ret = SGM4151x_get_batt_property(SGM, 
	POWER_SUPPLY_PROP_STATUS, &batt_prop);
#endif
	if (!ret) {
    	mutex_lock(&SGM->data_lock);
//	SGM->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);
    	mutex_unlock(&SGM->data_lock);
	}
	return ret;
}
	
static int SGM4151x_get_prop_charge_status(struct SGM4151x *SGM)
{
//	union power_supply_propval batt_prop = {0,};
	int ret;
	u8 status;
	
//	ret = SGM4151x_get_batt_property(SGM, 
//	POWER_SUPPLY_PROP_STATUS, &batt_prop);
//	if (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL)
//	return POWER_SUPPLY_STATUS_FULL;
	
	ret = SGM4151x_read_byte(SGM, &status, SGM4151X_REG_08);
	if (ret) {
    	return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	
	mutex_lock(&SGM->data_lock);
	SGM->charge_state = (status & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT;
	mutex_unlock(&SGM->data_lock);
	
	if (SGM->usb_present && SGM->jeita_active 
	&& (SGM->batt_warm || SGM->batt_cool) 
	&& SGM->charge_state == CHARGE_STATE_CHGDONE)
    	return POWER_SUPPLY_STATUS_FULL;
	
	switch(SGM->charge_state) {
    	case CHARGE_STATE_FASTCHG:
    	case CHARGE_STATE_PRECHG:
        	return POWER_SUPPLY_STATUS_CHARGING;
    	case CHARGE_STATE_CHGDONE:
        	return POWER_SUPPLY_STATUS_NOT_CHARGING;
    	case CHARGE_STATE_IDLE:
        	return POWER_SUPPLY_STATUS_DISCHARGING;
    	default:
        	return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}
	
static int SGM4151x_get_prop_health(struct SGM4151x *SGM)
{
	int ret;
	union power_supply_propval batt_prop = {0,};
	
	if (SGM->software_jeita_supported) {
    	if (SGM->jeita_active) {
        	if (SGM->batt_hot) 
            	ret = POWER_SUPPLY_HEALTH_OVERHEAT;
        	else if (SGM->batt_warm)
            	ret = POWER_SUPPLY_HEALTH_WARM;
        	else if (SGM->batt_cool)
            	ret = POWER_SUPPLY_HEALTH_COOL;
        	else if (SGM->batt_cold)
            	ret = POWER_SUPPLY_HEALTH_COLD;
    	} else {
        	ret = POWER_SUPPLY_HEALTH_GOOD;
    	}
	} else {/* get health status from gauge */
	ret = SGM4151x_get_batt_property(SGM, 
	POWER_SUPPLY_PROP_HEALTH, &batt_prop);
	if (!ret)
    	ret = batt_prop.intval;
	else
    	ret = POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	return ret;
}
	
	
static enum power_supply_property SGM4151x_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE, 
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
};
	
void static runin_work(struct SGM4151x *SGM, int batt_capacity)
{
	int rc;
	
	printk("%s:BatteryTestStatus_enable = %d SGM->usb_present = %d \n",
	__func__,BatteryTestStatus_enable,SGM->usb_present);
	
	if (/*!SGM->usb_present || */!BatteryTestStatus_enable) {
    	if (SGM->in_hiz) {
        	rc = SGM4151x_exit_hiz_mode(SGM);
        	if (rc) {
            	dev_err(SGM->dev, "Couldn't enable charge rc=%d\n", rc);
        	} else {
            	pr_err("Exit Hiz Successfully\n");
        	SGM->in_hiz = false;
        	}
    	}
    	return;
	}
	
	if (batt_capacity >= 80) {
    	pr_debug("SGM4151x_get_prop_batt_capacity > 80\n");
    	//rc = SGM4151x_charging_disable(SGM, USER, true);
    	if (!SGM->in_hiz) {
        	rc = SGM4151x_enter_hiz_mode(SGM);
        	if (rc) {
            	dev_err(SGM->dev, "Couldn't disenable charge rc=%d\n", rc);
        	} else {
            	pr_err("Enter Hiz Successfully\n");
            	SGM->in_hiz = true;
        	}
    	}
	} else if (batt_capacity < 60) {
    	pr_debug("SGM4151x_get_prop_batt_capacity < 60\n");
    	//rc = SGM4151x_charging_disable(SGM, USER, false);
    	if (SGM->in_hiz) {
        	rc = SGM4151x_exit_hiz_mode(SGM);
    	if (rc) {
        	dev_err(SGM->dev, "Couldn't enable charge rc=%d\n", rc);
    	} else {
        	pr_err("Exit Hiz Successfully\n");
        	SGM->in_hiz = false;
    	}
    	} 
	}
}

static int SGM4151x_charger_get_property(struct power_supply *psy,
enum power_supply_property psp,
union power_supply_propval *val)
{
	
	struct SGM4151x *SGM = container_of(psy, struct SGM4151x, batt_psy);
	switch (psp) {
    	case POWER_SUPPLY_PROP_CHARGE_TYPE:
        	val->intval = SGM4151x_get_prop_charge_type(SGM);
        	pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n", val->intval);
    	break;
    	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        	val->intval = SGM->charge_enabled;
    	break;
    	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        	val->intval = 3080;
    	break;
    	
    	case POWER_SUPPLY_PROP_STATUS:
        	val->intval = SGM4151x_get_prop_charge_status(SGM);
    	break;
    	case POWER_SUPPLY_PROP_HEALTH:
        	val->intval = SGM4151x_get_prop_health(SGM);
    	break;
    	case POWER_SUPPLY_PROP_CAPACITY:
        	SGM4151x_get_batt_property(SGM, psp, val);
        	runin_work(SGM, val->intval);
    	break;
    	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
        	val->intval = SGM->therm_lvl_sel;
    	break;
    	case POWER_SUPPLY_PROP_PRESENT:
    	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
    	case POWER_SUPPLY_PROP_CURRENT_NOW:
    	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
    	case POWER_SUPPLY_PROP_TEMP:
    	case POWER_SUPPLY_PROP_CHARGE_FULL:
    	case POWER_SUPPLY_PROP_TECHNOLOGY:
    	case POWER_SUPPLY_PROP_RESISTANCE_ID:
        	return SGM4151x_get_batt_property(SGM, psp, val);
    	default:
        	return -EINVAL;
	
	}
	
	return 0;
}
	
static int SGM4151x_system_temp_level_set(struct SGM4151x *SGM, int);
	
static int SGM4151x_charger_set_property(struct power_supply *psy,
enum power_supply_property prop,
const union power_supply_propval *val)
{
	struct SGM4151x *SGM = container_of(psy,
	struct SGM4151x, batt_psy);
	
	switch (prop) {
    	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        	SGM4151x_charging_disable(SGM, USER, !val->intval);
        	
        	power_supply_changed(&SGM->batt_psy);
        	power_supply_changed(SGM->usb_psy);
        	pr_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n", 
        	val->intval ? "enable" : "disable");
        	break;
    	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
        	SGM4151x_system_temp_level_set(SGM, val->intval);
        	break;
    	default:
        	return -EINVAL;
	}
	
	return 0;
}
	
static int SGM4151x_charger_is_writeable(struct power_supply *psy,
enum power_supply_property prop)
{
	int ret;
	
	switch (prop) {
    	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
    	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
        	ret = 1;
        	break;
    	default:
        	ret = 0;
        	break;
    	}
    	return ret;
	}
	
static int SGM4151x_update_charging_profile(struct SGM4151x *SGM)
{
	int ret;
	int chg_ma;
	int chg_mv;
	int icl;
	int therm_ma;
	
	union power_supply_propval prop = {0,};
	
	
	if (!SGM->usb_present) 
    	return 0;
	
//	ret = SGM->usb_psy->get_property(SGM->usb_psy, 
//	POWER_SUPPLY_PROP_TYPE, &prop);
	
	if (ret < 0) {
    	pr_err("couldn't read USB TYPE property, ret=%d\n", ret);
    	return ret;
	}
	pr_err("charge type = %d\n", prop.intval);
	mutex_lock(&SGM->profile_change_lock);
	if (SGM->jeita_active) {
    	chg_ma = SGM->jeita_ma;
    	chg_mv = SGM->jeita_mv;
	} else {
    	if (prop.intval == POWER_SUPPLY_TYPE_USB_DCP
    	|| prop.intval == POWER_SUPPLY_TYPE_USB_CDP) {
    	chg_ma = SGM->platform_data->ta.ichg;
    	chg_mv = SGM->platform_data->ta.vreg;
    	} else {
    	chg_ma = SGM->platform_data->usb.ichg;
    	chg_mv = SGM->platform_data->usb.vreg;
    	}
	}
	
	icl = SGM->usb_psy_ma;
	if (SGM->usb_psy_ma < chg_ma) {
    	chg_ma = SGM->usb_psy_ma;
	}
	
	if (SGM->therm_lvl_sel > 0
	&& SGM->therm_lvl_sel < (SGM->thermal_levels - 1))
	/*
	* consider thermal limit only when it is active and not at
	* the highest level
	*/
    	therm_ma = SGM->thermal_mitigation[SGM->therm_lvl_sel];
	else
    	therm_ma = chg_ma;
	
	chg_ma = min(therm_ma, chg_ma);
	
	pr_err("charge volt = %d, charge curr = %d, input curr limit = %d\n",
	chg_mv, chg_ma, icl);
	
	ret = SGM4151x_set_input_current_limit(SGM, icl);
	if (ret < 0)
    	pr_err("couldn't set input current limit, ret=%d\n", ret);
	
	ret = SGM4151x_set_input_volt_limit(SGM, SGM->platform_data->ta.vlim);
	if (ret < 0)
    	pr_err("couldn't set input voltage limit, ret=%d\n", ret);
	
	ret = SGM4151x_set_chargevolt(SGM, chg_mv);
	if (ret < 0)
    	pr_err("couldn't set charge voltage ret=%d\n", ret);
	
	ret = SGM4151x_set_chargecurrent(SGM, chg_ma);
	if (ret < 0)
    	pr_err("couldn't set charge current, ret=%d\n", ret);
	
	mutex_unlock(&SGM->profile_change_lock);
	
	return 0;
}
	
	
static int SGM4151x_system_temp_level_set(struct SGM4151x *SGM,
int lvl_sel)
{
	int ret = 0;
	int prev_therm_lvl;
	
	pr_err("lvl_sel=%d, SGM->therm_lvl_sel = %d\n", lvl_sel, SGM->therm_lvl_sel);
	if (BatteryTestStatus_enable)
    	return 0;
	
	if (!SGM->thermal_mitigation) {
    	pr_err("Thermal mitigation not supported\n");
    	return -EINVAL;
	}
	
	if (lvl_sel < 0) {
    	pr_err("Unsupported level selected %d\n", lvl_sel);
    	return -EINVAL;
	}
	
	if (lvl_sel >= SGM->thermal_levels) {
    	pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
    	SGM->thermal_levels - 1);
    	lvl_sel = SGM->thermal_levels - 1;
	}
	
	if (lvl_sel == SGM->therm_lvl_sel)
    	return 0;
	
	prev_therm_lvl = SGM->therm_lvl_sel;
	SGM->therm_lvl_sel = lvl_sel;
	
	ret = SGM4151x_update_charging_profile(SGM);
	if (ret)
    	pr_err("Couldn't set USB current ret = %d\n", ret);
	
    (void)prev_therm_lvl;
        
	return ret;
}
	
	
static void SGM4151x_external_power_changed(struct power_supply *psy)
{
	struct SGM4151x *SGM = container_of(psy, struct SGM4151x, batt_psy);
	
	union power_supply_propval prop = {0,};
	int ret, current_limit = 0;
	
	
//	ret = SGM->usb_psy->get_property(SGM->usb_psy, 
//	POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
    	pr_err("could not read USB current_max property, ret=%d\n", ret);
	else
    	current_limit = prop.intval / 1000;
	
	pr_err("current_limit = %d\n", current_limit);
	
	if (SGM->usb_psy_ma != current_limit) {
    	SGM->usb_psy_ma = current_limit;
    	SGM4151x_update_charging_profile(SGM);
	}
	
//	ret = SGM->usb_psy->get_property(SGM->usb_psy, 
//	POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
    	pr_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
    	pr_info("usb online status =%d\n", prop.intval);
	
	ret = 0;
	SGM4151x_get_prop_charge_status(SGM);
	if (SGM->usb_present /*&& SGM->charge_state != CHARGE_STATE_IDLE*//* && SGM->charge_enabled *//*!SGM->charging_disabled_status*/
	/*&& SGM->usb_psy_ma != 0*/) {
    	if (prop.intval == 0){
        	pr_err("set usb online\n");
        //	ret = power_supply_set_online(SGM->usb_psy, true);
    	}
	} else {
    	if (prop.intval == 1) {
        	pr_err("set usb offline\n");
        //	ret = power_supply_set_online(SGM->usb_psy, false);
    	}
	}
	
	if (ret < 0)
    	pr_info("could not set usb online state, ret=%d\n", ret);
	
}
	
	
static int SGM4151x_psy_register(struct SGM4151x *SGM)
{
	int ret = 0;
    
	SGM->batt_psy.name = "battery";
	SGM->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	SGM->batt_psy.properties = SGM4151x_charger_props;
	SGM->batt_psy.num_properties = ARRAY_SIZE(SGM4151x_charger_props);
	SGM->batt_psy.get_property = SGM4151x_charger_get_property;
	SGM->batt_psy.set_property = SGM4151x_charger_set_property;
	SGM->batt_psy.external_power_changed = SGM4151x_external_power_changed;
	SGM->batt_psy.property_is_writeable = SGM4151x_charger_is_writeable;
    
//	ret = power_supply_register(SGM->dev, &SGM->batt_psy);
	if (ret < 0) {
    	pr_err("failed to register batt_psy:%d\n", ret);
    	return ret;
	}
	
	return 0;
}

#if 0
static void SGM4151x_psy_unregister(struct SGM4151x *SGM)
{
//    power_supply_unregister(&SGM->batt_psy);
}
#endif

#if 0
static int SGM4151x_otg_regulator_enable(struct regulator_dev *rdev)
{
    int ret;
    struct SGM4151x *SGM;// = rdev_get_drvdata(rdev);

    ret = SGM4151x_enable_otg(SGM);
    if (ret) {
        pr_err("Couldn't enable OTG mode ret=%d\n", ret);
    } else {
        SGM->otg_enabled = true;
        pr_info("SGM4151x OTG mode Enabled!\n");
    }

    return ret;
}
#endif	

#if 0
static int SGM4151x_otg_regulator_disable(struct regulator_dev *rdev)
{
    int ret;
    struct SGM4151x *SGM;// = rdev_get_drvdata(rdev);

    ret = SGM4151x_disable_otg(SGM);
    if (ret) {
        pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
    } else {
        SGM->otg_enabled = false;
        pr_info("SGM4151x OTG mode Disabled\n");
    }

    return ret;
}
#endif	

#if 0
static int SGM4151x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    int ret;
    u8 status;
    u8 enabled;
    struct SGM4151x *SGM;// = rdev_get_drvdata(rdev);

    ret = SGM4151x_read_byte(SGM, &status, SGM4151X_REG_01);
    if (ret)
        return ret;
    enabled = ((status & REG01_OTG_CONFIG_MASK) >> REG01_OTG_CONFIG_SHIFT);

    return (enabled == REG01_OTG_ENABLE) ? 1 : 0;

}
#endif

#if 0	
	struct regulator_ops SGM4151x_otg_reg_ops = {
	.enable = SGM4151x_otg_regulator_enable,
	.disable = SGM4151x_otg_regulator_disable,
	.is_enabled = SGM4151x_otg_regulator_is_enable,
	};
#endif	
	
static int SGM4151x_regulator_init(struct SGM4151x *SGM)
{
	int ret = 0;
#if 0	
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	
	init_data = of_get_regulator_init_data(SGM->dev, SGM->dev->of_node);
	if (!init_data) {
	dev_err(SGM->dev, "Unable to allocate memory\n");
	return -ENOMEM;
	}
    
	if (init_data->constraints.name) {
	SGM->otg_vreg.rdesc.owner = THIS_MODULE;
	SGM->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
	SGM->otg_vreg.rdesc.ops = &SGM4151x_otg_reg_ops;
	SGM->otg_vreg.rdesc.name = init_data->constraints.name;
	pr_info("regualtor name = %s\n", SGM->otg_vreg.rdesc.name);
	
	cfg.dev = SGM->dev;
	cfg.init_data = init_data;
	cfg.driver_data = SGM;
	cfg.of_node = SGM->dev->of_node;
	
	init_data->constraints.valid_ops_mask
	|= REGULATOR_CHANGE_STATUS;
	
	SGM->otg_vreg.rdev = regulator_register(
	&SGM->otg_vreg.rdesc, &cfg);
	if (IS_ERR(SGM->otg_vreg.rdev)) {
	ret = PTR_ERR(SGM->otg_vreg.rdev);
	SGM->otg_vreg.rdev = NULL;
	if (ret != -EPROBE_DEFER)
	dev_err(SGM->dev,
	"OTG reg failed, rc=%d\n", ret);
	}
	}
#endif	
	
	return ret;
}
	
#if 0	
static int SGM4151x_parse_jeita_dt(struct device *dev, struct SGM4151x* SGM)
{
	struct device_node *np = dev->of_node;
	int ret;
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-hot-degc",
	&SGM->batt_hot_degc);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-hot-degc\n");
	return ret;
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-warm-degc",
	&SGM->batt_warm_degc);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-warm-degc\n");
	return ret;
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-cool-degc",
	&SGM->batt_cool_degc);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-cool-degc\n");
	return ret;
	}
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-cold-degc",
	&SGM->batt_cold_degc);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-cold-degc\n");
	return ret;
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-hot-hysteresis",
	&SGM->hot_temp_hysteresis);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-hot-hysteresis\n");
	return ret;
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-cold-hysteresis",
	&SGM->cold_temp_hysteresis);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-cold-hysteresis\n");
	return ret;
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-cool-ma",
	&SGM->batt_cool_ma);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-cool-ma\n");
	return ret;
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-cool-mv",
	&SGM->batt_cool_mv);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-cool-mv\n");
	return ret;
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-warm-ma",
	&SGM->batt_warm_ma);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-warm-ma\n");
	return ret;
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,jeita-warm-mv",
	&SGM->batt_warm_mv);
	if(ret) {
	pr_err("Failed to read SGM,SGM4151x,jeita-warm-mv\n");
	return ret;
	}
	
	SGM->software_jeita_supported = 
	of_property_read_bool(np,"SGM,SGM4151x,software-jeita-supported");
	
return 0;
}
#endif	
#if 0	
	static struct SGM4151x_platform_data* SGM4151x_parse_dt(struct device *dev, 
	struct SGM4151x * SGM)
	{
	int ret;
	struct device_node *np = dev->of_node;
	struct SGM4151x_platform_data* pdata;
	
	pdata = devm_kzalloc(dev, sizeof(struct SGM4151x_platform_data), 
	GFP_KERNEL);
	if (!pdata) {
	pr_err("Out of memory\n");
	return NULL;
	}
	
	ret = of_property_read_u32(np, "SGM,SGM4151x,chip-enable-gpio", &SGM->gpio_ce);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,chip-enable-gpio\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,usb-vlim",&pdata->usb.vlim);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,usb-vlim\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,usb-ilim",&pdata->usb.ilim);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,usb-ilim\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,usb-vreg",&pdata->usb.vreg);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,usb-vreg\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,usb-ichg",&pdata->usb.ichg);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,usb-ichg\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,ta-vlim",&pdata->ta.vlim);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,ta-vlim\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,ta-ilim",&pdata->ta.ilim);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,ta-ilim\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,ta-vreg",&pdata->ta.vreg);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,ta-vreg\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,ta-ichg",&pdata->ta.ichg);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,ta-ichg\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,stat-pin-ctrl",&pdata->statctrl);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,stat-pin-ctrl\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,precharge-current",&pdata->iprechg);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,precharge-current\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,termination-current",&pdata->iterm);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,termination-current\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,boost-voltage",&pdata->boostv);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,boost-voltage\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,boost-current",&pdata->boosti);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,boost-current\n");
	}
	
	ret = of_property_read_u32(np,"SGM,SGM4151x,vac-ovp-threshold",&pdata->vac_ovp);
	if(ret) {
	pr_err("Failed to read node of SGM,SGM4151x,vac-ovp-threshold\n");
	}
	
	if (of_find_property(np, "SGM,thermal-mitigation",
	&SGM->thermal_levels)) {
	SGM->thermal_mitigation = devm_kzalloc(SGM->dev,
	SGM->thermal_levels,
	GFP_KERNEL);
	
	if (SGM->thermal_mitigation == NULL) {
	pr_err("thermal mitigation kzalloc() failed.\n");
	}
	
	SGM->thermal_levels /= sizeof(int);
	ret = of_property_read_u32_array(np,
	"SGM,thermal-mitigation",
	SGM->thermal_mitigation, SGM->thermal_levels);
	if (ret) {
	pr_err("Couldn't read thermal limits ret = %d\n", ret);
	}
	}
	
	
	return pdata; 
	}
#endif	
	
static void SGM4151x_init_jeita(struct SGM4151x *SGM)
{

    SGM->batt_temp = -EINVAL;

    /* set default value in case of dts read fail */
    SGM->batt_hot_degc = 600;
    SGM->batt_warm_degc = 450;
    SGM->batt_cool_degc = 100;
    SGM->batt_cold_degc = 0;

    SGM->hot_temp_hysteresis = 50;
    SGM->cold_temp_hysteresis = 50;

    SGM->batt_cool_ma = 400;
    SGM->batt_cool_mv = 4100;
    SGM->batt_warm_ma = 400;
    SGM->batt_warm_mv = 4100;

    SGM->software_jeita_supported = true;

    /* DTS setting will overwrite above default value */

//	SGM4151x_parse_jeita_dt(&SGM->client->dev, SGM);
}
	
static int SGM4151x_init_device(struct SGM4151x *SGM)
{
	int ret;
	
	SGM4151x_disable_watchdog_timer(SGM);
	
	SGM4151x_enable_batfet(SGM);
	SGM4151x_set_vdpm_bat_track(SGM);
	
	ret = SGM4151x_set_stat_ctrl(SGM, SGM->platform_data->statctrl);
	if (ret)
    	pr_err("Failed to set stat pin control mode, ret = %d\n",ret);
	
	ret = SGM4151x_set_prechg_current(SGM, SGM->platform_data->iprechg);
	if (ret)
    	pr_err("Failed to set prechg current, ret = %d\n",ret);
	
	ret = SGM4151x_set_term_current(SGM, SGM->platform_data->iterm);
	if (ret)
    	pr_err("Failed to set termination current, ret = %d\n",ret);
	
	ret = SGM4151x_set_boost_voltage(SGM, SGM->platform_data->boostv);
	if (ret)
    	pr_err("Failed to set boost voltage, ret = %d\n",ret);
	
	ret = SGM4151x_set_boost_current(SGM, SGM->platform_data->boosti);
	if (ret)
    	pr_err("Failed to set boost current, ret = %d\n",ret);
	
	ret = SGM4151x_set_acovp_threshold(SGM, SGM->platform_data->vac_ovp);
	if (ret)
    	pr_err("Failed to set acovp threshold, ret = %d\n",ret); 
	
	ret = SGM4151x_set_int_mask(SGM, REG0A_IINDPM_INT_MASK | REG0A_VINDPM_INT_MASK);
	if (ret)
    	pr_err("Failed to set vindpm and iindpm int mask\n");
	
	ret = SGM4151x_enable_charger(SGM);
	if (ret) {
    	pr_err("Failed to enable charger, ret = %d\n",ret); 
	} else {
	SGM->charge_enabled = true;
    	pr_err("Charger Enabled Successfully!\n");
	}
	
	return 0;
}
	
//by yangwensen@20200401	
static int SGM4151x_detect_device(struct SGM4151x* SGM)
{
    int ret;
    u8 data;

    ret = SGM4151x_read_byte(SGM, &data, SGM4151X_REG_0B);
    if(ret == 0)
    {
        SGM->part_no = (enum SGM4151x_part_no)((data & REG0B_PN_MASK) >> REG0B_PN_SHIFT);
        SGM->revision = (data & REG0B_DEV_REV_MASK) >> REG0B_DEV_REV_SHIFT;
    }

    return ret;
}
	
static void SGM4151x_check_jeita(struct SGM4151x *SGM)
{
	int ret;
	bool last_hot, last_warm, last_cool, last_cold;
	bool chg_disabled_jeita, jeita_hot_cold;
	union power_supply_propval batt_prop = {0,};
	
	ret = SGM4151x_get_batt_property(SGM,
	POWER_SUPPLY_PROP_TEMP, &batt_prop);
	if (!ret)
    	SGM->batt_temp = batt_prop.intval;
	
	if (SGM->batt_temp == -EINVAL) 
    	return;
	
	last_hot = SGM->batt_hot;
	last_warm = SGM->batt_warm;
	last_cool = SGM->batt_cool;
	last_cold = SGM->batt_cold;
	
	if (SGM->batt_temp >= SGM->batt_hot_degc) {/* HOT */
    	if (!SGM->batt_hot) {
        	SGM->batt_hot = true;
        	SGM->batt_warm = false;
        	SGM->batt_cool = false;
        	SGM->batt_cold = false;
        	SGM->jeita_ma = 0;
        	SGM->jeita_mv = 0;
    	}
	} else if (SGM->batt_temp >= SGM->batt_warm_degc) {/* WARM */
    	if (!SGM->batt_hot
    	||(SGM->batt_temp < SGM->batt_hot_degc - SGM->hot_temp_hysteresis)){
    	SGM->batt_hot = false;
    	SGM->batt_warm = true;
    	SGM->batt_cool = false;
    	SGM->batt_cold = false;
    	SGM->jeita_mv = SGM->batt_warm_mv;
    	SGM->jeita_ma = SGM->batt_warm_ma;
    	}
    	} else if (SGM->batt_temp < SGM->batt_cold_degc) {/* COLD */
    	if (!SGM->batt_cold) {
    	SGM->batt_hot = false;
    	SGM->batt_warm = false;
    	SGM->batt_cool = false;
    	SGM->batt_cold = true;
    	SGM->jeita_ma = 0;
    	SGM->jeita_mv = 0;
    	}
	} else if (SGM->batt_temp < SGM->batt_cool_degc) {/* COOL */
    	if (!SGM->batt_cold ||
    	(SGM->batt_temp > SGM->batt_cold_degc + SGM->cold_temp_hysteresis)) {
    	SGM->batt_hot = false;
    	SGM->batt_warm = false;
    	SGM->batt_cool = true;
    	SGM->batt_cold = false;
    	SGM->jeita_mv = SGM->batt_cool_mv;
    	SGM->jeita_ma = SGM->batt_cool_ma;
    	}
	} else {/* NORMAL */
	SGM->batt_hot = false;
	SGM->batt_warm = false;
	SGM->batt_cool = false;
	SGM->batt_cold = false;
	}
	
	SGM->jeita_active = SGM->batt_cool || SGM->batt_hot ||
	SGM->batt_cold || SGM->batt_warm;
	
	if ((last_cold != SGM->batt_cold) || (last_warm != SGM->batt_warm) ||
	(last_cool != SGM->batt_cool) || (last_hot != SGM->batt_hot)) {
    	SGM4151x_update_charging_profile(SGM);
    	power_supply_changed(&SGM->batt_psy);
    	power_supply_changed(SGM->usb_psy);
	} else if (SGM->batt_hot || SGM->batt_cold) { /*continuely update event */
    	power_supply_changed(&SGM->batt_psy);
    	power_supply_changed(SGM->usb_psy);
	}
	
	jeita_hot_cold = SGM->jeita_active && (SGM->batt_hot || SGM->batt_cold);
	chg_disabled_jeita = !!(SGM->charging_disabled_status & JEITA);
	if (jeita_hot_cold ^ chg_disabled_jeita)
	SGM4151x_charging_disable(SGM, JEITA, jeita_hot_cold);
}
	
static void SGM4151x_check_batt_pres(struct SGM4151x *SGM)
{
	int ret = 0;
	bool chg_disabled_pres;
	
	ret = SGM4151x_get_prop_batt_present(SGM);
	if (!ret) {
    	chg_disabled_pres = !!(SGM->charging_disabled_status & BATT_PRES);
    	if (chg_disabled_pres ^ !SGM->batt_present) {
        	ret = SGM4151x_charging_disable(SGM, BATT_PRES, !SGM->batt_present);
    	if (ret) {
        	pr_err("failed to %s charging, ret = %d\n", 
        	SGM->batt_present ? "disable" : "enable",
        	ret);
    	}
    	power_supply_changed(&SGM->batt_psy);
    	power_supply_changed(SGM->usb_psy);
    	}
	}
	
}
	
static void SGM4151x_check_batt_full(struct SGM4151x *SGM)
{
    int ret = 0;
    bool chg_disabled_fc;

    ret = SGM4151x_get_prop_batt_full(SGM);
    if (!ret) {
        chg_disabled_fc = !!(SGM->charging_disabled_status & BATT_FC);
        if (chg_disabled_fc ^ SGM->batt_full) {
            ret = SGM4151x_charging_disable(SGM, BATT_FC, SGM->batt_full);
        if (ret) {
            pr_err("failed to %s charging, ret = %d\n", 
            SGM->batt_full ? "disable" : "enable",
            ret);
        }
        power_supply_changed(&SGM->batt_psy);
        power_supply_changed(SGM->usb_psy);
        }
    }
}
	
static int calculate_jeita_poll_interval(struct SGM4151x* SGM)
{
	int interval;
	
	if (SGM->batt_hot || SGM->batt_cold)
    	interval = 5;
	else if (SGM->batt_warm || SGM->batt_cool)
    	interval = 10;
	else
    	interval = 15;

    return interval;
}
	
#define FG_LOG_INTERVAL 120
static void SGM4151x_dump_fg_reg(struct SGM4151x *SGM)
{
    union power_supply_propval val = {0,};
    static int dump_cnt;

    if (++dump_cnt >= (FG_LOG_INTERVAL / calculate_jeita_poll_interval(SGM))) {
        dump_cnt = 0;
        val.intval = 0;
        SGM->bms_psy->set_property(SGM->bms_psy, 
        POWER_SUPPLY_PROP_UPDATE_NOW, &val); 
    }
}

#if 0
	static enum alarmtimer_restart SGM4151x_jeita_alarm_cb(struct alarm *alarm,
	ktime_t now)
	{
	struct SGM4151x *SGM = container_of(alarm, 
	struct SGM4151x, jeita_alarm);
	unsigned long ns;
	
	SGM4151x_stay_awake(&SGM->SGM4151x_ws, WAKEUP_SRC_JEITA);
	schedule_delayed_work(&SGM->charge_jeita_work, HZ/2);
	
	ns = calculate_jeita_poll_interval(SGM) * 1000000000LL;
	alarm_forward_now(alarm, ns_to_ktime(ns));
	return ALARMTIMER_RESTART;
	}
#endif

static void SGM4151x_dump_status(struct SGM4151x* SGM);
//static void SGM4151x_charge_jeita_workfunc(struct work_struct *work)
static void SGM4151x_charge_jeita_workfunc(struct SGM4151x* SGM)
{
#if 0
	struct SGM4151x *SGM = container_of(work, 
	struct SGM4151x, charge_jeita_work.work);
#endif	
	
	SGM4151x_reset_watchdog_timer(SGM);
	
	SGM4151x_check_batt_pres(SGM);
	SGM4151x_check_batt_full(SGM);
	SGM4151x_dump_fg_reg(SGM);
	
	SGM4151x_check_jeita(SGM);
	SGM4151x_dump_status(SGM);
	SGM4151x_relax(&SGM->SGM4151x_ws, WAKEUP_SRC_JEITA);
}

//static void SGM4151x_discharge_jeita_workfunc(struct work_struct *work)
static void SGM4151x_discharge_jeita_workfunc(struct SGM4151x *SGM)
{
//	struct SGM4151x *SGM = container_of(work, 
//	struct SGM4151x, discharge_jeita_work.work);
	
	SGM4151x_check_batt_pres(SGM);
	SGM4151x_check_batt_full(SGM);
	SGM4151x_dump_fg_reg(SGM);
	
	SGM4151x_check_jeita(SGM);
//	schedule_delayed_work(&SGM->discharge_jeita_work, calculate_jeita_poll_interval(SGM) * HZ);
}

static const unsigned char* charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};
	
static void SGM4151x_dump_status(struct SGM4151x* SGM)
{
	u8 status;
	u8 addr;
	int ret;
	u8 val;
	union power_supply_propval batt_prop = {0,};
	
	ret = SGM4151x_get_batt_property(SGM,
	POWER_SUPPLY_PROP_CURRENT_NOW, &batt_prop);
	
	if (!ret)
    	pr_err("FG current:%d\n", batt_prop.intval);
	
	for (addr = 0x0; addr <= 0x0B; addr++) {
    	if (addr == 0x09) {
        	pr_err("SGM Reg[09] = 0x%02X\n", SGM->fault_status);
    	continue;
    	}
    	ret = SGM4151x_read_byte(SGM, &val, addr);
    	if (!ret)
        	pr_err("SGM Reg[%02X] = 0x%02X\n", addr, val);
    	else
        	pr_err("SGM Reg red err\n");
	}
	
	ret = SGM4151x_read_byte(SGM, &status, SGM4151X_REG_0A);
	if (ret) {
    	pr_err("failed to read reg0a\n");
    	return;
    }
	
	mutex_lock(&SGM->data_lock);
	SGM->vbus_good = !!(status & REG0A_VBUS_GD_MASK);
	SGM->vindpm_triggered = !!(status & REG0A_VINDPM_STAT_MASK);
	SGM->iindpm_triggered = !!(status & REG0A_IINDPM_STAT_MASK);
	SGM->topoff_active = !!(status & REG0A_TOPOFF_ACTIVE_MASK);
	SGM->acov_triggered = !!(status & REG0A_ACOV_STAT_MASK);
	mutex_unlock(&SGM->data_lock);
	
	
	if (!SGM->power_good)
    	pr_info("Power Poor\n");
	if (!SGM->vbus_good)
    	pr_err("Vbus voltage not good!\n");
	if (SGM->vindpm_triggered)
    	pr_err("VINDPM triggered\n");
	if (SGM->iindpm_triggered)
    	pr_err("IINDPM triggered\n");
	if (SGM->acov_triggered)
    	pr_err("ACOV triggered\n");
	
	if (SGM->fault_status & REG09_FAULT_WDT_MASK)
    	pr_err("Watchdog timer expired!\n");
	if (SGM->fault_status & REG09_FAULT_BOOST_MASK)
    	pr_err("Boost fault occurred!\n");
	
	status = (SGM->fault_status & REG09_FAULT_CHRG_MASK) >> REG09_FAULT_CHRG_SHIFT;
	if (status == REG09_FAULT_CHRG_INPUT)
    	pr_err("input fault!\n");
	else if (status == REG09_FAULT_CHRG_THERMAL)
    	pr_err("charge thermal shutdown fault!\n");
	else if (status == REG09_FAULT_CHRG_TIMER)
    	pr_err("charge timer expired fault!\n");
	
	if (SGM->fault_status & REG09_FAULT_BAT_MASK)
    	pr_err("battery ovp fault!\n");
	
	if (!SGM->software_jeita_supported) {
	status = (SGM->fault_status & REG09_FAULT_NTC_MASK) >> REG09_FAULT_NTC_SHIFT;
	
	if (status == REG09_FAULT_NTC_WARM)
    	pr_debug("JEITA ACTIVE: WARM\n");
	else if (status == REG09_FAULT_NTC_COOL)
    	pr_debug("JEITA ACTIVE: COOL\n");
	else if (status == REG09_FAULT_NTC_COLD)
    	pr_debug("JEITA ACTIVE: COLD\n");
	else if (status == REG09_FAULT_NTC_HOT)
    	pr_debug("JEITA ACTIVE: HOT!\n");
	} else if (SGM->jeita_active) {
    	if (SGM->batt_hot)
        	pr_debug("JEITA ACTIVE: HOT\n");
    	else if (SGM->batt_warm)
        	pr_debug("JEITA ACTIVE: WARM\n");
    	else if (SGM->batt_cool)
        	pr_debug("JEITA ACTIVE: COOL\n");
    	else if (SGM->batt_cold)
        	pr_debug("JEITA ACTIVE: COLD\n");
	}
    	
	pr_err("%s\n",charge_stat_str[SGM->charge_state]);
}
	
	
static void SGM4151x_update_status(struct SGM4151x *SGM)
{
	u8 status;
	int ret;
	
	/* Read twice to get present status */
	ret = SGM4151x_read_byte(SGM, &status, SGM4151X_REG_09);
	if (ret)
    	return;
	pr_err("First read of REG[09] = 0x%02x\n", status);
	ret = SGM4151x_read_byte(SGM, &status, SGM4151X_REG_09);
	if (ret)
    	return;
	
	pr_err("Second read of REG[09] = 0x%02x\n", status);
	mutex_lock(&SGM->data_lock);
	SGM->fault_status = status;
	mutex_unlock(&SGM->data_lock);
	
}
	
static irqreturn_t SGM4151x_charger_interrupt(int irq, void *dev_id)
{
	struct SGM4151x *SGM = dev_id;
	
	u8 status;
	int ret; 
	
	mutex_lock(&SGM->irq_complete);
	SGM->irq_waiting = true;
	if (!SGM->resume_completed) {
    	LOG_D("IRQ triggered before device-resume\n");
    	if (!SGM->irq_disabled) {
//        	disable_irq_nosync(irq);
        	SGM->irq_disabled = true;
    	}
    	mutex_unlock(&SGM->irq_complete);
    	return IRQ_HANDLED;
	}
    
	SGM->irq_waiting = false;
	ret = SGM4151x_read_byte(SGM, &status, SGM4151X_REG_08);
	if (ret) {
    	mutex_unlock(&SGM->irq_complete);
    	return IRQ_HANDLED;
	}
	
	mutex_lock(&SGM->data_lock);
	SGM->power_good = !!(status & REG08_PG_STAT_MASK);
	mutex_unlock(&SGM->data_lock);
	
	if(!SGM->power_good) {
    	if(SGM->usb_present) {
    	SGM->usb_present = false;
//    	power_supply_set_present(SGM->usb_psy, SGM->usb_present);
    	}
	
    	if (SGM->software_jeita_supported) {
//        	alarm_try_to_cancel(&SGM->jeita_alarm);
    	}
	
    	SGM4151x_disable_watchdog_timer(SGM);
	
//    	schedule_delayed_work(&SGM->discharge_jeita_work,
//    	calculate_jeita_poll_interval(SGM) * HZ);
	
    	pr_err("usb removed, set usb present = %d\n", SGM->usb_present);
	} else if (SGM->power_good && !SGM->usb_present) {
    	SGM->usb_present = true;
    	rt_thread_mdelay(10);/*for cdp detect*/
        
//   	power_supply_set_present(SGM->usb_psy, SGM->usb_present);
    	
//    	cancel_delayed_work(&SGM->discharge_jeita_work);
	
    	if (SGM->software_jeita_supported) { 
    #if 0	
    	ret = alarm_start_relative(&SGM->jeita_alarm, 
    	ns_to_ktime(1 * 1000000000LL));
    	if (ret) 
    	pr_err("start alarm for JEITA detection failed, ret=%d\n",
    	ret);
    #endif
    	}
    
    	SGM4151x_set_watchdog_timer(SGM, 80);
	
    	pr_err("usb plugged in, set usb present = %d\n", SGM->usb_present);
	}
	
	SGM4151x_update_status(SGM);
	
	mutex_unlock(&SGM->irq_complete);
	
	power_supply_changed(&SGM->batt_psy);
	
	return IRQ_HANDLED;
}

static void determine_initial_status(struct SGM4151x *SGM)
{
    int ret;
    u8 status = 0;
    ret = SGM4151x_get_hiz_mode(SGM, &status);
    if (!ret) 
        SGM->in_hiz = !!status;

//    SGM4151x_charger_interrupt(SGM->client->irq, SGM);
    SGM4151x_charger_interrupt(RT_NULL, SGM);
}

#if 0
static ssize_t SGM4151x_show_registers(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct SGM4151x *SGM = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret ;
	
	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "SGM4151x Reg");
	for (addr = 0x0; addr <= 0x0B; addr++) {
	ret = SGM4151x_read_byte(SGM, &val, addr);
	if (ret == 0) {
	len = snprintf(tmpbuf, PAGE_SIZE - idx,"Reg[0x%.2x] = 0x%.2x\n", addr, val);
	memcpy(&buf[idx], tmpbuf, len);
	idx += len;
	}
	}
	
	return idx;
}
#endif

#if 0
static ssize_t SGM4151x_store_registers(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	struct SGM4151x *SGM = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;
	
	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x0B) {
	SGM4151x_write_byte(SGM, (unsigned char)reg, (unsigned char)val);
	}
	
	return count;
}
#endif

#if 0
static ssize_t SGM4151x_battery_test_status_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
    return rt_sprintf(buf, "%d\n", BatteryTestStatus_enable);
}
#endif

#if 0
static ssize_t SGM4151x_battery_test_status_store(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	
	if (sscanf(buf, "%u", &input) != 1)
    	retval = -EINVAL;
	else
    	BatteryTestStatus_enable = input;
	
	pr_err("BatteryTestStatus_enable = %d\n", BatteryTestStatus_enable);
	
	return retval;
}
#endif

#if 0
static ssize_t SGM4151x_show_hiz(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct SGM4151x *SGM;// = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", SGM->in_hiz);
}
#endif

#if 0
static ssize_t SGM4151x_store_hiz(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	struct SGM4151x *SGM;// = dev_get_drvdata(dev);
	int ret;
	unsigned int val;
	
	ret = sscanf(buf, "%d", &val);
	if (ret == 1) {
    	if (val)
        	ret = SGM4151x_enter_hiz_mode(SGM);
    	else 
        	ret = SGM4151x_exit_hiz_mode(SGM);
	}
	if (!ret)
	SGM->in_hiz = !!val;
	
	return ret;
}
#endif

#if 0
static ssize_t SGM4151x_show_dis_safety(struct device *dev,
struct device_attribute *attr, char *buf)
{
    struct SGM4151x *SGM;// = dev_get_drvdata(dev);
    return rt_sprintf(buf, "%d\n", SGM->dis_safety);
}
#endif

#if 0
	static ssize_t SGM4151x_store_dis_safety(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
	{
	struct SGM4151x *SGM;// = dev_get_drvdata(dev);
	int ret;
	unsigned int val;
	
	ret = sscanf(buf, "%d", &val);
	if (ret == 1) {
	if (val)
	ret = SGM4151x_disable_safety_timer(SGM);
	else 
	ret = SGM4151x_enable_safety_timer(SGM);
	}
	if (!ret)
	SGM->dis_safety = !!val;
	
	return count;
	}
	
	static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, SGM4151x_show_registers, SGM4151x_store_registers);
	static DEVICE_ATTR(BatteryTestStatus, S_IRUGO | S_IWUSR, SGM4151x_battery_test_status_show, SGM4151x_battery_test_status_store);
	static DEVICE_ATTR(hiz, S_IRUGO | S_IWUSR, SGM4151x_show_hiz, SGM4151x_store_hiz);
	static DEVICE_ATTR(dissafety, S_IRUGO | S_IWUSR, SGM4151x_show_dis_safety, SGM4151x_store_dis_safety);
	
	static struct attribute *SGM4151x_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_BatteryTestStatus.attr,
	&dev_attr_hiz.attr,
	&dev_attr_dissafety.attr,
	NULL,
	};
	static const struct attribute_group SGM4151x_attr_group = {
	.attrs = SGM4151x_attributes,
	};
#endif	
	
#if 0	
	static int show_registers(struct seq_file *m, void *data)
	{
	struct SGM4151x *SGM;// = m->private;
	u8 addr;
	int ret;
	u8 val;
	
	for (addr = 0x0; addr <= 0x0B; addr++) {
	ret = SGM4151x_read_byte(SGM, &val, addr);
	if (!ret)
	seq_printf(m, "Reg[%02X] = 0x%02X\n", addr, val);
	}
	return 0; 
	}
#endif

#if 0	
	static int reg_debugfs_open(struct inode *inode, struct file *file)
	{
	struct SGM4151x *SGM;// = inode->i_private;
	
	return single_open(file, show_registers, SGM);
	}
#endif

#if 0	
	static const struct file_operations reg_debugfs_ops = {
	.owner = THIS_MODULE,
	.open = reg_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	};
#endif

#if 0
	static void create_debugfs_entry(struct SGM4151x *SGM)
	{
	SGM->debug_root = debugfs_create_dir("SGM4151x", NULL);
	if (!SGM->debug_root)
	pr_err("Failed to create debug dir\n");
	
	if (SGM->debug_root) {
	
	debugfs_create_file("registers", S_IFREG | S_IRUGO,
	SGM->debug_root, SGM, &reg_debugfs_ops);
	
	debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
	SGM->debug_root, &(SGM->charging_disabled_status));
	
	debugfs_create_x32("fault_status", S_IFREG | S_IRUGO,
	SGM->debug_root, &(SGM->fault_status));
	
	debugfs_create_x32("vbus_type", S_IFREG | S_IRUGO,
	SGM->debug_root, &(SGM->vbus_type)); 
	
	debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
	SGM->debug_root, &(SGM->charge_state)); 
	
	debugfs_create_x32("skip_reads",
	S_IFREG | S_IWUSR | S_IRUGO,
	SGM->debug_root,
	&(SGM->skip_reads));
	debugfs_create_x32("skip_writes",
	S_IFREG | S_IWUSR | S_IRUGO,
	SGM->debug_root,
	&(SGM->skip_writes));
	} 
	}
#endif	

//static int SGM4151x_charger_probe(struct rt_i2c_bus_device *client, const struct i2c_device_id *id)
static int SGM4151x_charger_probe(struct rt_i2c_bus_device *client)
{
	struct SGM4151x *SGM;
	int ret;
    
#if 0
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	
	
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
//	dev_dbg(&client->dev, "USB supply not found, defer probe\n");
	return -EPROBE_DEFER;
	}
	
	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
//	dev_dbg(&client->dev, "bms supply not found, defer probe\n");
	return -EPROBE_DEFER;
	}
#endif

    SGM = (struct SGM4151x *)rt_malloc(sizeof(struct SGM4151x));

    if (!SGM) 
    {
    	pr_err("Out of memory\n");
    	return -ENOMEM;
	}
    rt_memset(SGM, 0, sizeof(struct SGM4151x));
    
#if 0	
	SGM->dev = &client->dev;
	SGM->usb_psy = usb_psy;
	SGM->bms_psy = bms_psy;
#endif

	SGM->client = client;
//	i2c_set_clientdata(client, SGM);
	
	rt_mutex_init(&SGM->i2c_rw_lock, "i2c_rw", RT_IPC_FLAG_FIFO);
	rt_mutex_init(&SGM->data_lock, "data_lock", RT_IPC_FLAG_FIFO);
	rt_mutex_init(&SGM->profile_change_lock, "profile_change", RT_IPC_FLAG_FIFO);
	rt_mutex_init(&SGM->charging_disable_lock, "charging_disable", RT_IPC_FLAG_FIFO);
	rt_mutex_init(&SGM->irq_complete, "irq_complete", RT_IPC_FLAG_FIFO);
	
	SGM->resume_completed = true;
	SGM->irq_waiting = false;
    
	ret = SGM4151x_detect_device(SGM);
	if(ret) 
    {
    	pr_err("No SGM4151x device found!\n");
    	return -ENODEV;
	}
    
	SGM4151x_init_jeita(SGM);
#if 0
	if (client->dev.of_node)
	SGM->platform_data = SGM4151x_parse_dt(&client->dev, SGM);
	else
	SGM->platform_data = client->dev.platform_data;
	
	if (!SGM->platform_data) {
	pr_err("No platform data provided.\n");
	return -EINVAL;
	}

	if (gpio_is_valid(SGM->gpio_ce)) {
	ret = devm_gpio_request(&client->dev, SGM->gpio_ce, "SGM4151x_ce");
	if (ret) {
	pr_err("Failed to request chip enable gpio %d:, err: %d\n", SGM->gpio_ce, ret);
	return ret;
	}
	gpio_direction_output(SGM->gpio_ce, 0);
	}
#endif
	
	ret = SGM4151x_init_device(SGM);
	if (ret) {
    	pr_err("Failed to init device\n");
    	return ret;
	}
	
	ret = SGM4151x_psy_register(SGM);
	if (ret)
	    return ret;

    ret = SGM4151x_regulator_init(SGM);
	if (ret) 
    {
    	pr_err("Couldn't initialize SGM4151x regulator ret=%d\n", ret);
    	return ret;
	}
    
//	INIT_DELAYED_WORK(&SGM->charge_jeita_work, SGM4151x_charge_jeita_workfunc);
//	INIT_DELAYED_WORK(&SGM->discharge_jeita_work, SGM4151x_discharge_jeita_workfunc);
    SGM4151x_charge_jeita_workfunc(SGM);
    SGM4151x_discharge_jeita_workfunc(SGM);
#if 0	
	
	alarm_init(&SGM->jeita_alarm, ALARM_BOOTTIME, SGM4151x_jeita_alarm_cb);
	
	if (client->irq) {
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
	SGM4151x_charger_interrupt,
	IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	"SGM4151x charger irq", SGM);
	if (ret < 0) {
	pr_err("request irq for irq=%d failed, ret =%d\n", client->irq, ret);
	goto err_1;
	}
	enable_irq_wake(client->irq);
	}
#endif

	SGM4151x_wakeup_src_init(SGM);

#if 0
	device_init_wakeup(SGM->dev, 1); 
	create_debugfs_entry(SGM);
	
	ret = sysfs_create_group(&SGM->dev->kobj, &SGM4151x_attr_group);
	if (ret) {
	dev_err(SGM->dev, "failed to register sysfs. err: %d\n", ret);
	}
#endif	
	
	determine_initial_status(SGM);
	
	pr_err("SGM4151x probe successfully, Part Num:%d, Revision:%d\n!", 
	SGM->part_no, SGM->revision);
	
	return 0;
	
//	err_1:
//	SGM4151x_psy_unregister(SGM);
	
//	return ret;
}
    
static inline bool is_device_suspended(struct SGM4151x *SGM)
{
    return !SGM->resume_completed;
}
#if 0
	
	static int SGM4151x_suspend(struct device *dev)
	{
	struct i2c_client *client = to_i2c_client(dev);
	struct SGM4151x *SGM = i2c_get_clientdata(client);
	
	mutex_lock(&SGM->irq_complete);
	SGM->resume_completed = false;
	mutex_unlock(&SGM->irq_complete);
	
	return 0;
	}
#endif

#if 0
	static int SGM4151x_suspend_noirq(struct device *dev)
	{
	struct i2c_client *client = to_i2c_client(dev);
	struct SGM4151x *SGM = i2c_get_clientdata(client);
	
	if (SGM->irq_waiting) {
	pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
	return -EBUSY;
	}
	return 0;
	}
#endif

#if 0
	static int SGM4151x_resume(struct device *dev)
	{
	struct i2c_client *client = to_i2c_client(dev);
	struct SGM4151x *SGM = i2c_get_clientdata(client);
	
	
	mutex_lock(&SGM->irq_complete);
	SGM->resume_completed = true;
	if (SGM->irq_waiting) {
	SGM->irq_disabled = false;
	enable_irq(client->irq);
	mutex_unlock(&SGM->irq_complete);
	SGM4151x_charger_interrupt(client->irq, SGM);
	} else {
	mutex_unlock(&SGM->irq_complete);
	}
	
	power_supply_changed(&SGM->batt_psy);
	
	return 0;
	}
#endif

#if 0
	static int SGM4151x_charger_remove(struct i2c_client *client)
	{
	struct SGM4151x *SGM = i2c_get_clientdata(client);
	
	alarm_try_to_cancel(&SGM->jeita_alarm);
	
	cancel_delayed_work_sync(&SGM->charge_jeita_work);
	cancel_delayed_work_sync(&SGM->discharge_jeita_work);
	
	regulator_unregister(SGM->otg_vreg.rdev);
	
	SGM4151x_psy_unregister(SGM);
	
	mutex_destroy(&SGM->charging_disable_lock);
	mutex_destroy(&SGM->profile_change_lock);
	mutex_destroy(&SGM->data_lock);
	mutex_destroy(&SGM->i2c_rw_lock);
	mutex_destroy(&SGM->irq_complete);
	
	debugfs_remove_recursive(SGM->debug_root);
	sysfs_remove_group(&SGM->dev->kobj, &SGM4151x_attr_group);
	
	
	return 0;
	}
#endif	

#if 0
static void SGM4151x_charger_shutdown(struct rt_i2c_bus_device *client)
{
}
#endif

#if 0
	static struct of_device_id SGM4151x_charger_match_table[] = {
	{.compatible = "SG,SGM41511-charger",},
	{.compatible = "SG,SGM41512-charger",},
	{},
	};
	MODULE_DEVICE_TABLE(of,SGM4151x_charger_match_table);
	
	static const struct i2c_device_id SGM4151x_charger_id[] = {
	{ "SGM41511-charger", SGM41511 },
	{ "SGM41512-charger", SGM41512 },
	{},
	};
	MODULE_DEVICE_TABLE(i2c, SGM4151x_charger_id);
	
	static const struct dev_pm_ops SGM4151x_pm_ops = {
	.resume = SGM4151x_resume,
	.suspend_noirq = SGM4151x_suspend_noirq,
	.suspend = SGM4151x_suspend,
	};
static struct i2c_driver SGM4151x_charger_driver = {
    .driver = {
    .name = "SGM4151x-charger",
    .owner = THIS_MODULE,
    .of_match_table = SGM4151x_charger_match_table,
    .pm = &SGM4151x_pm_ops,
    },
    .id_table = SGM4151x_charger_id,

    .probe = SGM4151x_charger_probe,
    .remove = SGM4151x_charger_remove,
    .shutdown = SGM4151x_charger_shutdown,

};
	
	module_i2c_driver(SGM4151x_charger_driver);
	
	MODULE_DESCRIPTION("SGM SGM4151x Charger Driver");
	MODULE_LICENSE("GPL v2");
	MODULE_AUTHOR("SG MICRO");
#endif
