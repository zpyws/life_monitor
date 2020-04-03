/**
  ******************************************************************************
  * 文件名 ：   M90_M91_HAL.c
  * 作者   ：   LSD RF/LoRaWAN Team
  * 版本   ：   V1.0.0
  * 时间   ：   19-May-2018
  * 文件描述：
  *     该文件为M90_M91模块的硬件层，包含MCU与M90模块的串口配置，GPIO口初始化，以
  *及用于各个超时判断的低功耗时钟等。
  *    客户在使用M90模块时候需要移植该文件，保证各个函数名、函数形参不变的情况下
  *根据自己的MCU平台修改函数内容，使各个功能块正常运行。硬件层占用资源如下：
  *
  *串口：本例程使用STM32L4的低功耗串口LPUART1与M90模块进行通信。
  *定时器：本例程使用了STM32L4的TIM3作为串口的帧结尾判断。
  *        使用了STM32L4的LPTIM1作为应用层需要的各个超时定时器。
  *        如果用户在其他地方需要用到这两个定时器，其接口都已留出，用户可在回调函
  *        数中自行添加
  *GPIO口：本例程使用的GPIO口详情如下：
  *        PC3  ---> WAKE 
  *        PC4  ---> STAT
  *        PC5  ---> BUSY     
  *        PA0  ---> MODE 
  *        PA1  ---> RST 
*******************************************************************************/

#include <rtthread.h>
#include <board.h>
#include "M90_M91_HAL.h"


LoRaNode_USART_RECEIVETYPE LoRaNode_UART;
//UART_HandleTypeDef hlpuart1;                   
//TIM_HandleTypeDef TIM3_UARTHandler;      

//LPTIM_HandleTypeDef hlptim1;

LPTIM1_FlagTypeDef LPTIM1_Flag;
uint8_t LoRaNode_ReadTimeout_flag = 0,LoRaNode_BUSYTimeout_flag = 0,LoRaNode_LPsend_flag = 0;

uint16_t Rx_Count;
uint8_t aRxBuffer[1];

uint8_t Send_Sate_Sign;

extern int lora_read(uint8_t *buff, uint32_t len);
extern int lora_write(uint8_t *buff, uint32_t len);

//-----------------------------UART-----------------------------//
//该部分函数为MCU对LoRaWAN节点模块的串口通信部分内容，包含串口初始化
//串口发送字节、字符串函数、接收中断以及帧结尾判断。
//--------------------------------------------------------------//

/**
  * @简介：该函数用于MCU对LoRaWAN节点模块的串口操作的帧结尾判断定时器初始化。              
  * @参数： arr单位为ms  
  * @返回值：无
  */
void TIM3_UART_Init_ms(uint16_t arr)
{   
}

/**
  * @简介：该函数用于MCU对LoRaWAN节点模块的串口操作的帧结尾判断定时器回调函数。              
  * @参数： htim为串口配置  
  * @返回值：无
  */
void HAL_TIM_PeriodElapsedCallback(void)
{
//    if(htim==(&TIM3_UARTHandler))
    {
        LoRaNode_UART.receive_flag = 1;
//        HAL_TIM_Base_Stop_IT(&TIM3_UARTHandler);       
    }
//    else
    {
    }
}

/**
  * @简介：该函数用于MCU对LoRaWAN节点模块的串口初始化。              
  * @参数： 无  
  * @返回值：无
  */
void LoRaNode_UART_Init(void)                                               
{
}

/**
  * @简介：该函数用于MCU对LoRaWAN节点模块的串口接收中断回调函数。              
  * @参数： huart为串口配置参数  
  * @返回值：无
  */
void HAL_UART_RxCpltCallback(void)
{ 
    LoRaNode_UART.RX_Buf[LoRaNode_UART.rx_len] = aRxBuffer[0];
    LoRaNode_UART.rx_len++;
//    HAL_UART_Receive_IT(&hlpuart1,aRxBuffer,1);
//    __HAL_TIM_SET_COUNTER(&TIM3_UARTHandler,0);
//    HAL_TIM_Base_Start_IT (&TIM3_UARTHandler);
}

/**
  * @简介：该函数用于MCU对LoRaWAN节点模块发送一个字符串。              
  * @参数： str为被发送字符串  
  * @返回值：无
  */
void LoRaNode_UART_Send_String(uint8_t *str)
{
    while((*str)!=0)
    {
        lora_write(str++, 1);
    }
}

/**
  * @简介：该函数用于MCU对LoRaWAN节点模块发送一个字节。              
  * @参数： data为被发送字节  
  * @返回值：无
  */
void LoRaNode_UART_Send_Byte(uint8_t data)
{
    lora_write(&data, 1);
}

//-----------------------------LPTIM1-----------------------------//
//该部分函数为MCU低功耗定时器部分，主要为应用程序提供超时判断，包含低
//功耗定时器初始化，开始，停止以及中断回调函数。
//--------------------------------------------------------------//

/**
  * @简介：该函数为低功耗定时器初始化。              
  * @参数： 无  
  * @返回值：无
  */
void LPTIM1_Init(void)
{
    LoRaNode_ReadTimeout_flag = 0;
    LoRaNode_BUSYTimeout_flag = 0;
    LoRaNode_LPsend_flag = 0;
}

/**
  * @简介：该函数为低功耗定时器中断模式开启。              
  * @参数： arr单位为秒， LPTIM_Flag为应用中的超时类型，使用者可以自行增加 
  * @返回值：无
  */
void LPTIM1_SingleStart_s(uint16_t arr,LPTIM1_FlagTypeDef LPTIM_Flag)
{
    LPTIM1_Flag = LPTIM_Flag;
    arr = arr*500;//32000/64分频
//    HAL_LPTIM_SetOnce_Start_IT(&hlptim1,0,arr);
}
/**
  * @简介：该函数为低功耗定时器中断回调函数。              
  * @参数： hlptim为低功耗定时器配置 
  * @返回值：无
  */
void HAL_LPTIM_CompareMatchCallback(void)// case
{
//    HAL_LPTIM_SetOnce_Stop_IT(&hlptim1);
    switch(LPTIM1_Flag)
    {
    case LoRaNode_ReadTimeout:
        LoRaNode_ReadTimeout_flag = 1;
        break;
    case LoRaNode_BUSYTimeout:
        LoRaNode_BUSYTimeout_flag = 1;
        break;
    case LoRaNode_LPsendTimeout:
        LoRaNode_LPsend_flag = 1;
        break;
    default:
        break;
    }
    
}
/**
  * @简介：该函数为低功耗定时器中断配置。              
  * @参数： hlptim为低功耗定时器配置 
  * @返回值：无
  */
void HAL_LPTIM_MspInit(void)
{
}

//-----------------------------GPIO-----------------------------//
//该部分函数为系统用到的GPIO的初始化函数，用户根据自己的平台相应修改
//--------------------------------------------------------------//
//by yangwensen@20200403
void LoRaNode_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    
    /*Configure GPIO pins : PC3  fro WAKE in LoRaNode */
    rt_pin_mode(LoRaNode_WAKE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LoRaNode_WAKE_PIN, PIN_HIGH);
    
    /*Configure GPIO pins : PC4  fro STAT in LoRaNode ,PC5  fro BUSY in LoRaNode*/
    rt_pin_mode(LoRaNode_STAT_PIN, PIN_MODE_INPUT);
    
    rt_pin_mode(LoRaNode_BUSY_PIN, PIN_MODE_INPUT);

    /*Configure GPIO pins : PA0  fro MODE in LoRaNode ,PA1 fro RST in LoRaNode*/    
    rt_pin_mode(LoRaNode_MODE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LoRaNode_MODE_PIN, PIN_HIGH);

    rt_pin_mode(LoRaNode_NRST_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LoRaNode_NRST_PIN, PIN_HIGH);
}

//----------------------------Delay-----------------------------//
//by yangwensen@20200403
void Delay_ms(uint32_t Delay)
{
    rt_thread_mdelay(Delay);
}



