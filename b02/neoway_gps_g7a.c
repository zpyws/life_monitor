//created by yangwensen@20200330
#include <rtthread.h>
#include <rtdevice.h>
//********************************************************************************************************************************************
#define GPS_UART            "uart1"
#define GPS_RX_BUFF_SIZE    512
//********************************************************************************************************************************************
static rt_device_t gps_uart_device = RT_NULL;
static struct rt_semaphore rx_sem;
static uint8_t gps_rx_buff[GPS_RX_BUFF_SIZE];
//********************************************************************************************************************************************
//by yangwensen@20200331
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (size > 0)
    {
        rt_sem_release(&rx_sem);
    }
    return RT_EOK;
}

//********************************************************************************************************************************************
//by yangwensen@20200331
static int gps_init(void)
{
    gps_uart_device = rt_device_find(GPS_UART);
    
    if(gps_uart_device == RT_NULL )
    {
        rt_kprintf("find device %s failed!\n", GPS_UART);
        return -1;
    }
    
    struct serial_configure gps_use_config = 
    {
        BAUD_RATE_9600,   /* 9600 bits/s */
        DATA_BITS_8,      /* 8 databits */
        STOP_BITS_1,      /* 1 stopbit */
        PARITY_NONE,      /* No parity  */ 
        BIT_ORDER_LSB,    /* LSB first sent */
        NRZ_NORMAL,       /* Normal mode */
        GPS_RX_BUFF_SIZE, /* Buffer size */
        0   
    };
    
    if (RT_EOK != rt_device_control(gps_uart_device, RT_DEVICE_CTRL_CONFIG,(void *)&gps_use_config))
    {
        rt_kprintf("uart config failed.\n");
        return -2;
    }
    
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    
    if (rt_device_open(gps_uart_device, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("uart open error.\n");
        return -3;
    }    
    
    rt_device_set_rx_indicate(gps_uart_device, uart_rx_ind);
    
    return RT_EOK;
}
//********************************************************************************************************************************************
//by yangwensen@20200331
static void task_gps(void *parameter)
{
    rt_size_t res;
    
    gps_init();
    while(1)
    {
        res = rt_device_read(gps_uart_device, -1, gps_rx_buff, sizeof(gps_rx_buff));
        if(res == 0)
            continue;
        
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        
        gps_rx_buff[res] = 0;
        rt_kprintf("%s", gps_rx_buff);
        rt_memset(gps_rx_buff, 0, sizeof(gps_rx_buff));
    }
}
//********************************************************************************************************************************************
//by yangwensen@20200331
extern void gps_startup(void)
{
    rt_thread_t thread = rt_thread_create("GPS", task_gps, RT_NULL, 512, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }    
}
//********************************************************************************************************************************************
