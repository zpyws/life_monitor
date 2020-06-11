//created by yangwensen@20200506
#include <rtthread.h>
#include <rtdevice.h>
#include "sensor.h"

#define LOG_TAG                         "sensor"
#define LOG_LVL                         LOG_LVL_DBG
#include <ulog.h>
//*************************************************************************************************************
extern int LoRaWAN_Node_Send(uint8_t *buf, uint16_t len, uint8_t confirm);
//*************************************************************************************************************
//by yangwensen@20200506
static int sensor_temp(char *name, struct rt_sensor_data *sensor_data)
{
    rt_device_t dev = RT_NULL;
    rt_size_t res;
    
    dev = rt_device_find(name);
    if(dev==RT_NULL)
    {
        LOG_E("can't not find device %s", name);
        return -1;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        LOG_E("open device failed!");
        return -2;
    }
    
    res = rt_device_read(dev, 0, sensor_data, 1);
    if(res!=1)
    {
        LOG_E("read temp sensor failed!");
        rt_device_close(dev);
        return -3;
    }
    
    rt_device_close(dev);
    
    return 0;
}
//*************************************************************************************************************
//by yangwensen@20200507
extern int send_sensor_data(void)
{
    int ret;
    struct rt_sensor_data sensor_data;
    
    ret = sensor_temp("temp_a", &sensor_data);
    if(ret)
        return -1;
    
    LOG_I("temp=%3d.%dC", sensor_data.data.temp/10, sensor_data.data.temp%10);
    
    ret = LoRaWAN_Node_Send((uint8_t *)(&sensor_data.data.temp), sizeof(sensor_data.data.temp), 1);
    if(ret)
        return -2;
 
    ulog_hexdump("[LoRaTx]", 16, (uint8_t *)(&sensor_data.data.temp), sizeof(sensor_data.data.temp));

    return 0;
}
//*************************************************************************************************************
