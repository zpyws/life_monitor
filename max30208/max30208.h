#ifndef __MAX30208__
#define __MAX30208__ 
#include <rthw.h>
#include <rtthread.h>

#include <rthw.h>
#include <rtdevice.h>

/*max30208 device address */
#define MAX30208_ADDR 0x50

/*max30208 registers define */


struct max30208_device
{	
    struct rt_i2c_bus_device *bus;
};


typedef struct max30208_device *max30208_device_t;

rt_err_t max30208_power_on(max30208_device_t hdev);
rt_err_t max30208_power_down(max30208_device_t hdev);
rt_err_t max30208_init(max30208_device_t hdev, const char *i2c_bus_name);
float max30208_read_temp(max30208_device_t hdev);



#endif /* __BH1750_H__ */
