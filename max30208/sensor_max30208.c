
#include "sensor_max30208.h"
#include "max30208.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.maxim.max30208"
#define DBG_COLOR
#include <rtdbg.h>


static max30208_device_t max30208_create(struct rt_sensor_intf *intf)
{
    max30208_device_t hdev;
	
    hdev  = rt_malloc(sizeof(max30208_device_t));
    if(hdev==RT_NULL)
        return RT_NULL;
    
    if( max30208_init(hdev, intf) != RT_EOK )
    {
        rt_free(hdev);
        return RT_NULL;
	}
    
    return hdev;
}

static rt_size_t max30208_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    max30208_device_t hdev = sensor->parent.user_data;
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    
    if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        float temp;

		temp = max30208_read_temp(hdev, (uint32_t)(sensor->config.intf.user_data));

        data->type = RT_SENSOR_CLASS_TEMP;
        data->data.temp = temp * 10;
        data->timestamp = rt_sensor_get_ts();
    }
    
    return 1;
}

rt_err_t max30208_set_power(max30208_device_t hdev, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_NORMAL)
    {
        max30208_power_on(hdev);
    }
    else if (power == RT_SENSOR_POWER_DOWN)
    {
        max30208_power_down(hdev);
    }
    else
    {
        return -RT_ERROR;
    }
	
    return RT_EOK;
}


static rt_err_t max30208_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    max30208_device_t hdev = sensor->parent.user_data;
    uint32_t addr = (uint32_t)(sensor->config.intf.user_data);

    switch (cmd)
    {
		case RT_SENSOR_CTRL_SET_POWER:
			result = max30208_set_power(hdev, (rt_uint32_t)args & 0xff);
		break;
		
		case RT_SENSOR_CTRL_SELF_TEST:
			result =  -RT_EINVAL;
        break;
		
        case RT_SENSOR_CTRL_GET_ID:
            max30208_get_part_id(hdev, addr, args);
        break;
            
		default:
			result = -RT_ERROR;
		break;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    max30208_fetch_data,
    max30208_control
};


int rt_hw_max30208_init(const char *name, struct rt_sensor_config *cfg)
{
	rt_int8_t result;
	rt_sensor_t sensor = RT_NULL;
	max30208_device_t hdev = max30208_create(&cfg->intf);
    if(hdev==RT_NULL)
    {
        LOG_E("could not find max30208 device\n");
        return -2;
    }

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_TEMP;
    sensor->info.vendor     = RT_SENSOR_VENDOR_DALLAS;
    sensor->info.model      = "max30208_temp";
    sensor->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 85;
    sensor->info.range_min  = -40;
    sensor->info.period_min = 80;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, hdev);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }

    LOG_I("temp sensor init success");
    return RT_EOK;
}

//by yangwensen@20200415
int max30208_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = "i2c2";
    cfg.intf.user_data = (void *)(MAX30208_ADDR+0);
    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_hw_max30208_init("max30208", &cfg);
	
    return 0;
}
INIT_APP_EXPORT(max30208_port);



