/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author        Notes
 * 2018-12-04   balanceTWK    first version
 */

#include "board.h"

#ifdef BSP_USING_ONCHIP_RTC

//#define DRV_DEBUG
#define LOG_TAG             "drv.rtc"
#include <drv_log.h>

#define BKUP_REG_DATA 0xA5A5

static struct rt_device rtc;

static time_t get_rtc_timestamp(void)
{
    struct tm tm_new;
    cy_stc_rtc_config_t dateTime = {0};

    Cy_RTC_GetDateAndTime(&dateTime);

    tm_new.tm_sec  = dateTime.sec;
    tm_new.tm_min  = dateTime.min;
    tm_new.tm_hour = dateTime.hour;
    tm_new.tm_mday = dateTime.date;
    tm_new.tm_mon  = dateTime.month - 1;
    tm_new.tm_year = dateTime.year + 70;

    LOG_D("get rtc time.");
    return mktime(&tm_new);
}

static rt_err_t set_rtc_time_stamp(time_t time_stamp)
{
    struct tm *p_tm;
    cy_stc_rtc_config_t t = {0};
    cy_en_rtc_status_t status;

    p_tm = localtime(&time_stamp);

    if((p_tm->tm_year < 70)||(p_tm->tm_year>=170))
        return -RT_ERROR;
    
    t.sec = p_tm->tm_sec ;
    t.min = p_tm->tm_min ;
    t.hour = p_tm->tm_hour;
    t.date    = p_tm->tm_mday;
    t.month   = p_tm->tm_mon + 1 ;
    t.year    = p_tm->tm_year - 70;
    t.dayOfWeek = p_tm->tm_wday + 1;

    status = Cy_RTC_SetDateAndTimeDirect(t.sec, t.min, t.hour, t.date, t.month, t.year );
    if( status != CY_RTC_SUCCESS)
    {
        LOG_E("set rtc time error.[0x%02x]", ~(CY_RTC_ID|CY_PDL_STATUS_ERROR)&status);
        return -RT_ERROR;
    }

    LOG_D("set rtc time ok.");

    return RT_EOK;
}

static void rt_rtc_init(void)
{
    RTCALM_Start();
}

static rt_err_t rt_rtc_config(struct rt_device *dev)
{
    if (1)
    {
        LOG_I("RTC hasn't been configured, please use <date> command to config.");

#ifdef BSP_RTC_USING_LSI
#else
#endif /* BSP_RTC_USING_LSI */
    }

    return RT_EOK;
}

static rt_err_t rt_rtc_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *(rt_uint32_t *)args = get_rtc_timestamp();
        LOG_D("RTC: get rtc_time %x\n", *(rt_uint32_t *)args);
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        if (set_rtc_time_stamp(*(rt_uint32_t *)args))
        {
            result = -RT_ERROR;
        }
        LOG_D("RTC: set rtc_time %x\n", *(rt_uint32_t *)args);
        break;
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rtc_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_rtc_control
};
#endif

static rt_err_t rt_hw_rtc_register(rt_device_t device, const char *name, rt_uint32_t flag)
{
    RT_ASSERT(device != RT_NULL);

    rt_rtc_init();
    if (rt_rtc_config(device) != RT_EOK)
    {
        return -RT_ERROR;
    }
#ifdef RT_USING_DEVICE_OPS
    device->ops         = &rtc_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_rtc_control;
#endif
    device->type        = RT_Device_Class_RTC;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->user_data   = RT_NULL;

    /* register a character device */
    return rt_device_register(device, name, flag);
}

int rt_hw_rtc_init(void)
{
    rt_err_t result;
    result = rt_hw_rtc_register(&rtc, "rtc", RT_DEVICE_FLAG_RDWR);
    if (result != RT_EOK)
    {
        LOG_E("rtc register err code: %d", result);
        return result;
    }
    LOG_D("rtc init success");
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_rtc_init);

#endif /* BSP_USING_ONCHIP_RTC */
