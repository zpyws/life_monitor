#include <rtthread.h>
#include <board.h>
//created by yangwensen@20200403
#define LOG_TAG                         "LoRa"
#define LOG_LVL                         LOG_LVL_DBG
#include <ulog.h>
//********************************************************************************************************************************************
#define HOME_KEY_PIN                    GET_PIN(7, 0)

#define LORA_UART_DEVICE                "uart1"
#define LORA_UART_RX_BUFF_SIZE          300

#define LORA_PRINT_RAW_DATA             1
//********************************************************************************************************************************************
static rt_device_t lora_uart_device = RT_NULL;
static struct rt_semaphore rx_sem;
static uint8_t lora_rx_buff[LORA_UART_RX_BUFF_SIZE];
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
//by yangwensen@20200403
static int lora_uart_init(void)
{
    rt_device_t serial;
    
    serial = rt_device_find(LORA_UART_DEVICE);
    
    if(serial == RT_NULL )
    {
        rt_kprintf("find device %s failed!\n", LORA_UART_DEVICE);
        return -1;
    }
    
    struct serial_configure lora_use_config = 
    {
        BAUD_RATE_9600,   /* 9600 bits/s */
        DATA_BITS_8,      /* 8 databits */
        STOP_BITS_1,      /* 1 stopbit */
        PARITY_NONE,      /* No parity  */ 
        BIT_ORDER_LSB,    /* LSB first sent */
        NRZ_NORMAL,       /* Normal mode */
        LORA_UART_RX_BUFF_SIZE, /* Buffer size */
        0   
    };
    
    if (RT_EOK != rt_device_control(serial, RT_DEVICE_CTRL_CONFIG,(void *)&lora_use_config))
    {
        rt_kprintf("uart config failed.\n");
        return -2;
    }
    
    rt_sem_init(&rx_sem, "lora_rx", 0, RT_IPC_FLAG_FIFO);
    
    if (rt_device_open(serial, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("uart open error.\n");
        return -3;
    }    
    
    rt_device_set_rx_indicate(serial, uart_rx_ind);
    
    lora_uart_device = serial;
    
    return RT_EOK;
}
//********************************************************************************************************************************************
//by yangwensen@20200403
static void task_lora(void *parameter)
{
    rt_size_t res;
    
    lora_uart_init();
    while(1)
    {
        res = rt_device_read(lora_uart_device, -1, lora_rx_buff, sizeof(lora_rx_buff));
        if(res == 0)
            continue;
        
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        
    #if LORA_PRINT_RAW_DATA > 0
        lora_rx_buff[res] = 0;
        rt_kprintf("%s", lora_rx_buff);
    #endif
    }
}
//********************************************************************************************************************************************
//by yangwensen@20200403
extern void lora_startup(void)
{
    rt_thread_t thread = rt_thread_create("LoRa", task_lora, RT_NULL, 512, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }    
}
//********************************************************************************************************************************************
