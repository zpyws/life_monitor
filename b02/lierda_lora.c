#include <rtthread.h>
#include <board.h>
//created by yangwensen@20200403
#define LOG_TAG                         "LoRa"
#define LOG_LVL                         LOG_LVL_DBG
#include <ulog.h>
#include "M90_M91_Interface.h"
//********************************************************************************************************************************************
#define HOME_KEY_PIN                    GET_PIN(7, 0)

#define LORA_UART_DEVICE                "uart1"
#define LORA_UART_RX_BUFF_SIZE          300

#define LORA_PRINT_RAW_DATA             1

#define LORA_JOIN_TIMEOUT               1500
//********************************************************************************************************************************************
rt_device_t lora_uart_device = RT_NULL;
static struct rt_semaphore rx_sem;
//static uint8_t lora_rx_buff[LORA_UART_RX_BUFF_SIZE];
int LoRaWAN_Node_Send(uint8_t *buf, uint8_t confirm);
//********************************************************************************************************************************************
int LoRaWAN_Node_SetParameter(void);
void LoRaWAN_Join(void);
static uint8_t Time_Out_Break(uint32_t MAX_time , uint8_t *Sign);
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
    
    if (rt_device_open(serial, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX) != RT_EOK)
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
	int state = 0;
	uint8_t SS_Data[5]={0};
    
    lora_uart_init();
    M90_M91_Init();
    
	state = LoRaWAN_Node_SetParameter();
	if(state < 0)
		LOG_E("参数设置错误[%d]\n", state);
	else
		LOG_I("参数设置成功\n");

	// 模块入网
	LoRaWAN_Join();
    
    while(1)
    {
#if 0
        res = rt_device_read(lora_uart_device, -1, lora_rx_buff, sizeof(lora_rx_buff));
        if(res == 0)
            continue;
        
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        
    #if LORA_PRINT_RAW_DATA > 0
        lora_rx_buff[res] = 0;
        rt_kprintf("%s", lora_rx_buff);
    #endif
#endif
		SS_Data[0] = 0xFF;
		SS_Data[1] = 0x00;
		SS_Data[2] = 0x01;
		SS_Data[3] = 0x02;
		SS_Data[4] = 0xFF;

		// 上行数据
		state = LoRaWAN_Node_Send(SS_Data, 1);
		if(state == 0)
        {
			LOG_I("数据上行成功, 上行数据:");
            ulog_hexdump("[LoRaTx]", 16, SS_Data, sizeof(SS_Data));
        }
		else
			LOG_E("数据上行错误,错误代码:%d\n", state);
        
        rt_thread_mdelay(5000);
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
//by yangwensen@20200403
extern int lora_read(uint8_t *buff, uint32_t len)
{
    rt_size_t res;
    
    res = rt_device_read(lora_uart_device, -1, buff, len);
    if(res == 0)
        return -1;

    rt_sem_take(&rx_sem, RT_WAITING_FOREVER);

    return res;
}

//by yangwensen@20200403
extern int lora_write(uint8_t *buff, uint32_t len)
{
    return rt_device_write(lora_uart_device, -1, buff, len);
}
//********************************************************************************************************************************************
//********************************************************************************************************************************************
// （用户修改）
// 设置模块参数
int LoRaWAN_Node_SetParameter(void)
{
    uint8_t a[16];
	int result = 0;
    
	// 唤醒模块
	LoRaNode_SetWake(Mode_WakeUp);
	rt_thread_mdelay(10);
	// 模块进入命令模式
	LoRaNode_SetMode(Mode_CMD);
	rt_thread_mdelay(10);
	
    //by yangwensen@20200429
    LOG_D("Moudle:%s\n", LoRaNode_GetVer());
    
    rt_memset(a, 0, sizeof(a));
    LoRaNode_Getpoint("AT+DEVEUI?", a);
    LOG_D("DEVEUI:%02X %02X %02X %02X %02X %02X %02X %02X", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);

    // 设置频率
	result += LoRaNode_Setpoint("AT+FREQ=","1,8,475300000");
	// 0->20dBm, 1->17, 2->16dBm, 3->14dBm, 4->12dBm, 5->10dBm, 6->7dBm, 7->5dBm, 8->2dBm
	result += LoRaNode_Setinteger("AT+Power=", 0);
	// 0->SF12, 1->SF11, 2->SF10, 3->SF9, 4->SF8, 5->SF7
	result += LoRaNode_Setinteger("AT+DATARATE=", 3);
	// 0:UNCONFIRM 1:CONFIRM
	result += LoRaNode_Setinteger("AT+CONFIRM=", 1);
	// 设置RX2
	result += LoRaNode_Setpoint("AT+RX2=","0,505300000");
	// 设置CLASS
	result += LoRaNode_Setinteger("AT+CLASS=",Class_A);
	// 设置OTAA
	result += LoRaNode_Setinteger("AT+OTAA=",NET_OTAA);
	
	// 修改一下为自身应用的APPEUI
    result += LoRaNode_Setpoint("AT+APPEUI=","616E2F87C71DF625");    
    rt_thread_mdelay(30);    
	// 修改一下为自身应用的APPKEY
	result += LoRaNode_Setpoint("AT+APPKEY=","746A209F0E0FC0C5B94452B304066A59");    
	rt_thread_mdelay(30);
	
	result += LoRaNode_Setpoint("AT+SAVE","\0");
	rt_thread_mdelay(200);
//========================================================================================
    //by yangwensen@20200430
    rt_memset(a, 0, sizeof(a));
    LoRaNode_Getpoint("AT+APPEUI?", a);
    LOG_D("APPEUI:%02X %02X %02X %02X %02X %02X %02X %02X", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);

    rt_memset(a, 0, sizeof(a));
    LoRaNode_Getpoint("AT+APPKEY?", a);
    LOG_D("APPKEY:%02X %02X %02X %02X %02X %02X %02X %02X", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);
    
	return result;
}
//********************************************************************************************************************************************
void LoRaWAN_Join(void)
{
    uint32_t i;
    
	// 重启模块
	LoRaNode_Reset();
	rt_thread_mdelay(500); 
    
	// 进入透传模式
	LoRaNode_SetMode(Mode_Transparent);
    
	rt_kprintf("正在join...");
	// 判断入网标志
    for(i=0; i<LORA_JOIN_TIMEOUT; i++)
    {
        if((LoRaNode_STAT_STATUS()==PIN_HIGH) && (LoRaNode_BUSY_STATUS()==PIN_HIGH))
            break;
        
		rt_kprintf(".");
//		LED3_ON;
		rt_thread_mdelay(500);
//  	LED3_OFF;
		rt_thread_mdelay(500);
    }
    if(i<LORA_JOIN_TIMEOUT)
    {
    //	LED3_ON;
    	LOG_I("\n入网成功[%dS]\n", i);
    }
    else
    {
    	LOG_E("\n入网失败[%dS]\n", LORA_JOIN_TIMEOUT);
    }
}
//********************************************************************************************************************************************
int LoRaWAN_Node_Send(uint8_t *buf, uint8_t confirm)
{
	// 唤醒模块
	LoRaNode_SetWake(Mode_WakeUp);
	rt_thread_mdelay(20);
	// 进入透传模式
	LoRaNode_SetMode(Mode_Transparent);
	rt_thread_mdelay(20);
	
	uint8_t TimeOut_Sign = 0;
	// 等待模块空闲
	while(LoRaNode_BUSY_STATUS() == PIN_LOW)
	{
		if(Time_Out_Break(20000, &TimeOut_Sign) == 1)
		{
			return -1;
		}
        rt_thread_mdelay(10);
	}
	
    LOG_D("lora idle");
	LoRaNode_Send_AT(buf);

	//----等待 BUSY 引脚处于忙状态(低电平)，等待时可以加超时判断
	TimeOut_Sign = 0;
	while(LoRaNode_BUSY_STATUS() == PIN_HIGH)
	{
		if(Time_Out_Break(2000, &TimeOut_Sign) == 1)
		{
			return -2;  // 错误 -2  : 模块没有接收到串口发送的数据，或者模块没有工作
		}
        rt_thread_mdelay(10);
	}
    LOG_D("lora send data");

	//----等待数据发送完成  BUSY 引脚回到空闲状态(高电平)，等待时可以加超时判断
	TimeOut_Sign = 0;
	while(LoRaNode_BUSY_STATUS() == PIN_LOW)
	{
		if(Time_Out_Break(60000, &TimeOut_Sign) == 1)
		{
			return -3; // 错误 -3  : 模块工作异常
		}
        rt_thread_mdelay(10);
	}
    LOG_D("lora data sent");
	
	if(confirm == 1)
	{
		if(LoRaNode_STAT_STATUS() == PIN_LOW)
		{
			return -4; // 若为确认帧，则表示模块未收到ACK
		}
	}
	
	rt_thread_mdelay(500);
	
	LoRaNode_MODE_LOW();

	return 0; // 本次通信成功
}
//********************************************************************************************************************************************
// 超时判断
static uint8_t Time_Out_Break(uint32_t MAX_time , uint8_t *Sign)
{
	static rt_tick_t  time_start = 0;
	static rt_tick_t  time_new = 0;
	uint32_t temp=0;
	uint8_t TimeOut_Sign = *Sign;

	if(TimeOut_Sign == 0)
	{
		*Sign = 1;
		time_start = rt_tick_get();
	}
	if(TimeOut_Sign == 1)
	{
		time_new = rt_tick_get();

		if(time_new < time_start)
		{
			time_new = (time_new + (0xffffffff - time_start));
			time_start = 0;
		}
		temp = time_new - time_start;
		if(temp > MAX_time)
		{
			return 1;
		}else{
    		return 0;
		}
	}
	return 0;
}
//********************************************************************************************************************************************
