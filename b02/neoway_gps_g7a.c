//created by yangwensen@20200330
#include <rtthread.h>
#include <rtdevice.h>

#include "nmeaparser.h"

#define LOG_TAG                 "GPS"
#define LOG_LVL                 LOG_LVL_DBG
#include <ulog.h>
//********************************************************************************************************************************************
#define GPS_UART                "uart1"
#define GPS_RX_BUFF_SIZE        512

#define GPS_PRINT_RAW_DATA      0
//********************************************************************************************************************************************
static rt_device_t gps_uart_device = RT_NULL;
static struct rt_semaphore rx_sem;
static uint8_t gps_rx_buff[GPS_RX_BUFF_SIZE];
//********************************************************************************************************************************************
static int8_t gps_parser_init(void);
static void gps_parse(uint8_t *buff, uint32_t len);
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
    
    gps_parser_init();
    gps_init();
    while(1)
    {
        res = rt_device_read(gps_uart_device, -1, gps_rx_buff, sizeof(gps_rx_buff));
        if(res == 0)
            continue;
        
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        
    #if GPS_PRINT_RAW_DATA > 0
        gps_rx_buff[res] = 0;
        rt_kprintf("%s", gps_rx_buff);
    #endif
        gps_parse(gps_rx_buff, res);
    }
}
//********************************************************************************************************************************************
//by yangwensen@20200331
extern void gps_startup(void)
{
    rt_thread_t thread = rt_thread_create("GPS", task_gps, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }    
}
//********************************************************************************************************************************************
struct nmea_parser parser[1];
//********************************************************************************************************************************************
//by yangwensen@20200331
/*
 * Print navigation data and wait for user's keystroke
 * @navdata:    the navigation data
 */
void display_navdata(struct nav_data *navdata) {
    LOG_D("NAVDATA:");
    LOG_D("NAVDATA.FIX_VALID = %d", navdata->is_fixed);
    LOG_D("NAVDATA.DATE = %d-%02d-%02d", navdata->date.year, navdata->date.month, navdata->date.day);
    LOG_D("NAVDATA.TIME= %02d:%02d:%02d.%03d", navdata->time.hour, navdata->time.minute, navdata->time.second, navdata->time.ms);
    LOG_D("NAVDATA.LAT = %.6f", navdata->lat);
    LOG_D("NAVDATA.LON = %.6f", navdata->lon);
    LOG_D("NAVDATA.ALT = %.2f", navdata->alt);
    LOG_D("NAVDATA.HEADING = %.2f", navdata->heading);
    LOG_D("NAVDATA.SPEED = %.2f", navdata->speed);
    LOG_D("NAVDATA.HDOP = %.1f", navdata->hdop);
    LOG_D("NAVDATA.VDOP = %.1f", navdata->vdop);
    LOG_D("NAVDATA.PDOP = %.1f", navdata->pdop);
    LOG_D("NAVDATA.NUM_SV_FIX = %d", navdata->sv_inuse);
    LOG_D("NAVDATA.NUM_SV_VIEW = %d", navdata->sv_inview);

    int svid;
    for (svid = 0; svid < MAX_SVID; svid++) { 
            struct sate *sate = navdata->sates + svid;
            if (sate->valid)
                    LOG_D("NAVDATA.SATE[%02d]: constell=%s prn=%02d, cn0=%02d, azim=%03d, elev=%02d, use=%d", 
                                    svid, constell_name(sate->constell), sate->prn, sate->cn0, sate->azim, sate->elev, sate->in_use);
    } 

    rt_kprintf("\n");
}
//********************************************************************************************************************************************
//by yangwensen@20200331
static int8_t gps_parser_init(void)
{
    nmea_parser_init(parser);
    parser->report_nav_status = display_navdata;
    
    return 0;
}
//********************************************************************************************************************************************
//by yangwensen@20200331
static void gps_parse(uint8_t *buff, uint32_t len)
{
    uint32_t i;
    
    for(i=0; i<len; i++)
    {
        nmea_parser_putchar(parser, buff[i]);
    }
}
//********************************************************************************************************************************************
