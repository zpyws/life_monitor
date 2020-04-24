#ifndef __MAX30208__
#define __MAX30208__ 
#include <rthw.h>
#include <rtthread.h>
#include "sensor.h"

/*max30208 device address */
#define MAX30208_ADDR 0x50

/*max30208 registers define */
/**** MAX30208 REGISTER ADDRESS ****/
#define MAX30208_STAUS										0x00
#define MAX30208_INTERRUPT_EN							    0x01
#define MAX30208_FIFO_WRITE_POINTER				            0x04
#define MAX30208_FIFO_READ_POINTER				            0x05
#define MAX30208_FIFO_OVERFLOW_COUNTER		                0x06
#define MAX30208_FIFO_DATA_COUNTER				            0x07
#define MAX30208_FIFO_DATA								    0x08
#define MAX30208_FIFO_CONFIGURATION1			            0x09
#define MAX30208_FIFO_CONFIGURATION2			            0x0A
#define MAX30208_SYSTEM_CONTROL						        0x0C
#define MAX30208_ALARM_HIGH_MSB						        0x10
#define MAX30208_ALARM_HIGH_LSB						        0x11
#define MAX30208_ALARM_LOW_MSB						        0x12
#define MAX30208_ALARM_LOW_LSB						        0x13
#define	MAX30208_TEMP_SENSOR_SETUP                          0x14
#define MAX30208_GPIO_SETUP								    0x20
#define MAX30208_GPIO_CONTROL							    0x21
#define MAX30208_PART_ID1									0x31
#define MAX30208_PART_ID2									0x32
#define MAX30208_PART_ID3									0x33
#define MAX30208_PART_ID4									0x34
#define MAX30208_PART_ID5									0x35
#define MAX30208_PART_ID6									0x36
#define MAX30208_PART_IDENTIFIER					        0xFF


/**** MAX30208 Command ****/
#define MAX30208_START_CONVERT						        0xFF
#define MAX30208_STOP_CONVERT							    0xFE



struct max30208_device
{	
    struct rt_i2c_bus_device *bus;
};


typedef struct max30208_device *max30208_device_t;

rt_err_t max30208_power_on(max30208_device_t hdev);
rt_err_t max30208_power_down(max30208_device_t hdev);
rt_err_t max30208_init(max30208_device_t hdev, struct rt_sensor_intf *intf);
float max30208_read_temp(max30208_device_t hdev, uint16_t addr);



#endif /* __BH1750_H__ */
