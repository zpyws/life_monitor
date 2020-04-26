/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-4-30     Sanjay_Wu  the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include <string.h>

#define DBG_ENABLE
#define DBG_SECTION_NAME "max30208"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#include "max30208.h"

#define MAX30208_CONVERTION_TIMEOUT         30

//by yangwensen@20200415
static rt_err_t max30208_read_regs(struct rt_i2c_bus_device *bus, uint16_t addr, rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

    if (rt_i2c_transfer(bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }
    
    return RT_EOK;
}

//by yangwensen@20200415
static rt_err_t max30208_write_reg(struct rt_i2c_bus_device *bus, uint16_t addr, rt_uint8_t reg, rt_uint8_t val)
{
    struct rt_i2c_msg msgs[1];
    uint8_t buf[2];
    
    buf[0] = reg;
    buf[1] = val;
    
    msgs[0].addr = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 2;

    if (rt_i2c_transfer(bus, msgs, 1) != 1)
        return -RT_ERROR;
    
    return RT_EOK;
}

//by yangwensen@20200415
static rt_err_t max30208_reset(max30208_device_t hdev, uint16_t addr)
{
    RT_ASSERT(hdev);
    
    return max30208_write_reg(hdev->bus, addr, MAX30208_SYSTEM_CONTROL, 0x01);
}

//by yangwensen@20200415
static rt_err_t max30208_convert(max30208_device_t hdev, uint16_t addr)
{
    rt_err_t ret;
    uint8_t val;
    uint8_t i;
    
    RT_ASSERT(hdev);
    
    ret =  max30208_write_reg(hdev->bus, addr, MAX30208_TEMP_SENSOR_SETUP, MAX30208_START_CONVERT);
    if(ret!=RT_EOK)
    {
        LOG_E("max30208_write_reg error %d\n", ret);
        return ret;
    }
    
    //wait until temperature measuerment completes
    for(i=0; i<MAX30208_CONVERTION_TIMEOUT; i++)
    {
        rt_thread_mdelay(1);
        ret = max30208_read_regs(hdev->bus, addr, MAX30208_TEMP_SENSOR_SETUP, &val, 1);
        if(val & 1)
            continue;
        break;
    }
    
    if(i>=MAX30208_CONVERTION_TIMEOUT)
    {
        LOG_E("convert timeout\n");
        return RT_ERROR;
    }
    
    return RT_EOK;
}

//by yangwensen@20200426
static rt_err_t max30208_get_uid(max30208_device_t hdev, uint16_t addr, uint8_t *buff)
{
    return max30208_read_regs(hdev->bus, addr, MAX30208_PART_ID1, buff, 6);
}

//by yangwensen@20200426
extern rt_err_t max30208_get_part_id(max30208_device_t hdev, uint16_t addr, uint8_t *buff)
{
    return max30208_read_regs(hdev->bus, addr, MAX30208_PART_IDENTIFIER, buff, 1);
}

//by yangwensen@20200415
static rt_err_t max30208_get_id(max30208_device_t hdev, uint16_t addr)
{
    RT_ASSERT(hdev);
    
    rt_err_t ret;
    uint8_t uid[6];
    
    ret = max30208_get_uid(hdev, addr, uid);
    if(ret==RT_EOK)
        LOG_I("MAX30208 UID:0x%02x%02x%02x%02x%02x%02x\n", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
        
    ret = max30208_get_part_id(hdev, addr, uid);
    if(ret==RT_EOK)
        LOG_I("max30208 part identifier:0x%02x\n", uid[0]);
        
    return ret;
}

//by yangwensen@20200424
static rt_err_t max30208_flush_fifo(max30208_device_t hdev, uint16_t addr)
{
    return max30208_write_reg(hdev->bus, addr, MAX30208_FIFO_CONFIGURATION2, 0x10);
}

rt_err_t max30208_power_on(max30208_device_t hdev)
{
    RT_ASSERT(hdev);

    return RT_EOK;
}

rt_err_t max30208_power_down(max30208_device_t hdev)
{
    RT_ASSERT(hdev);

    return RT_EOK;
}

rt_err_t max30208_init(max30208_device_t hdev, struct rt_sensor_intf *intf)
{
    hdev->bus = rt_i2c_bus_device_find(intf->dev_name);
    if (hdev->bus == RT_NULL)
    {
        LOG_E("Can't find max30208 device on '%s' ", intf->dev_name);
        rt_free(hdev->bus);
        return RT_NULL;
    }
	
    if( max30208_reset(hdev, (uint32_t)(intf->user_data)) != RT_EOK )
        return -RT_ERROR;
    
    if( max30208_get_id(hdev, (uint32_t)(intf->user_data)) != RT_EOK )
        return -RT_ERROR;
    
    return RT_EOK;
}

//by yangwensen@20200424
static int max30208_get_samples(max30208_device_t hdev, uint16_t addr, uint8_t samples)
{
    rt_err_t ret;
    uint8_t val;
    uint16_t raw[32];
    uint8_t *p;
    uint32_t sum = 0;
    
    if(samples>32)
        return -1;
    
    ret = max30208_flush_fifo(hdev, addr);
    if(ret!=RT_EOK)
        return -2;
    
    ret = max30208_read_regs(hdev->bus, addr, MAX30208_FIFO_OVERFLOW_COUNTER, &val, 1);
    if(ret!=RT_EOK)
        return -3;

    if(val==32)
    {
        LOG_E("FIFO Over Flow");
        max30208_flush_fifo(hdev, addr);
    }
    else if(val!=0)
    {
        LOG_E("flush fifo fail");
    }
    
    while(1)
    {
        ret = max30208_read_regs(hdev->bus, addr, MAX30208_FIFO_OVERFLOW_COUNTER, &val, 1);
        if(ret!=RT_EOK)
            return -4;
        
        ret = max30208_convert(hdev, addr);
        if(ret!=RT_EOK)
            LOG_E("max30208_convert error %d\n", ret);
            
        if(val == samples)
            break;
    }
    
    ret = max30208_write_reg(hdev->bus, addr, MAX30208_FIFO_WRITE_POINTER, 0);      // restrat from FIFO[0]
    if(ret!=RT_EOK)
        LOG_E("write MAX30208_FIFO_WRITE_POINTER error\n");
    
    ret = max30208_write_reg(hdev->bus, addr, MAX30208_FIFO_READ_POINTER, 0);      // read from FIFO[0]
    if(ret!=RT_EOK)
        LOG_E("write MAX30208_FIFO_WRITE_POINTER error\n");
    
    ret = max30208_read_regs(hdev->bus, addr, MAX30208_FIFO_DATA, (uint8_t *)raw, samples*2);
    if(ret!=RT_EOK)
        return -5;
    
    p = (uint8_t *)raw;
    for(val=0; val<samples; val++)
    {
        raw[val] = (uint16_t)( (p[val*2]<<8) + p[(val*2)+1]);
        sum += raw[val];
        LOG_D("sample[%d] = %d\n", val, raw[val]);
    }
    sum /= samples;
    LOG_D("sample average[%d] = %d\n", samples, sum);
    
    return sum;
}

//by yangwensen@20200415
float max30208_read_temp(max30208_device_t hdev, uint16_t addr)
{
    int ret;
    float current_temp = 0;

    RT_ASSERT(hdev);

    ret = max30208_get_samples(hdev, addr, 1);
    if(ret < 0)
    {
        LOG_E("max30208_get_samples error\n");
        return 0;
    }
    
	current_temp = (float)(ret * 0.005f);

    return current_temp;
}



