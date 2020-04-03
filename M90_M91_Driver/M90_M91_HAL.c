/**
  ******************************************************************************
  * �ļ��� ��   M90_M91_HAL.c
  * ����   ��   LSD RF/LoRaWAN Team
  * �汾   ��   V1.0.0
  * ʱ��   ��   19-May-2018
  * �ļ�������
  *     ���ļ�ΪM90_M91ģ���Ӳ���㣬����MCU��M90ģ��Ĵ������ã�GPIO�ڳ�ʼ������
  *�����ڸ�����ʱ�жϵĵ͹���ʱ�ӵȡ�
  *    �ͻ���ʹ��M90ģ��ʱ����Ҫ��ֲ���ļ�����֤�����������������ββ���������
  *�����Լ���MCUƽ̨�޸ĺ������ݣ�ʹ�������ܿ��������С�Ӳ����ռ����Դ���£�
  *
  *���ڣ�������ʹ��STM32L4�ĵ͹��Ĵ���LPUART1��M90ģ�����ͨ�š�
  *��ʱ����������ʹ����STM32L4��TIM3��Ϊ���ڵ�֡��β�жϡ�
  *        ʹ����STM32L4��LPTIM1��ΪӦ�ò���Ҫ�ĸ�����ʱ��ʱ����
  *        ����û��������ط���Ҫ�õ���������ʱ������ӿڶ����������û����ڻص���
  *        �����������
  *GPIO�ڣ�������ʹ�õ�GPIO���������£�
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
//�ò��ֺ���ΪMCU��LoRaWAN�ڵ�ģ��Ĵ���ͨ�Ų������ݣ��������ڳ�ʼ��
//���ڷ����ֽڡ��ַ��������������ж��Լ�֡��β�жϡ�
//--------------------------------------------------------------//

/**
  * @��飺�ú�������MCU��LoRaWAN�ڵ�ģ��Ĵ��ڲ�����֡��β�ж϶�ʱ����ʼ����              
  * @������ arr��λΪms  
  * @����ֵ����
  */
void TIM3_UART_Init_ms(uint16_t arr)
{   
}

/**
  * @��飺�ú�������MCU��LoRaWAN�ڵ�ģ��Ĵ��ڲ�����֡��β�ж϶�ʱ���ص�������              
  * @������ htimΪ��������  
  * @����ֵ����
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
  * @��飺�ú�������MCU��LoRaWAN�ڵ�ģ��Ĵ��ڳ�ʼ����              
  * @������ ��  
  * @����ֵ����
  */
void LoRaNode_UART_Init(void)                                               
{
}

/**
  * @��飺�ú�������MCU��LoRaWAN�ڵ�ģ��Ĵ��ڽ����жϻص�������              
  * @������ huartΪ�������ò���  
  * @����ֵ����
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
  * @��飺�ú�������MCU��LoRaWAN�ڵ�ģ�鷢��һ���ַ�����              
  * @������ strΪ�������ַ���  
  * @����ֵ����
  */
void LoRaNode_UART_Send_String(uint8_t *str)
{
    while((*str)!=0)
    {
        lora_write(str++, 1);
    }
}

/**
  * @��飺�ú�������MCU��LoRaWAN�ڵ�ģ�鷢��һ���ֽڡ�              
  * @������ dataΪ�������ֽ�  
  * @����ֵ����
  */
void LoRaNode_UART_Send_Byte(uint8_t data)
{
    lora_write(&data, 1);
}

//-----------------------------LPTIM1-----------------------------//
//�ò��ֺ���ΪMCU�͹��Ķ�ʱ�����֣���ҪΪӦ�ó����ṩ��ʱ�жϣ�������
//���Ķ�ʱ����ʼ������ʼ��ֹͣ�Լ��жϻص�������
//--------------------------------------------------------------//

/**
  * @��飺�ú���Ϊ�͹��Ķ�ʱ����ʼ����              
  * @������ ��  
  * @����ֵ����
  */
void LPTIM1_Init(void)
{
    LoRaNode_ReadTimeout_flag = 0;
    LoRaNode_BUSYTimeout_flag = 0;
    LoRaNode_LPsend_flag = 0;
}

/**
  * @��飺�ú���Ϊ�͹��Ķ�ʱ���ж�ģʽ������              
  * @������ arr��λΪ�룬 LPTIM_FlagΪӦ���еĳ�ʱ���ͣ�ʹ���߿����������� 
  * @����ֵ����
  */
void LPTIM1_SingleStart_s(uint16_t arr,LPTIM1_FlagTypeDef LPTIM_Flag)
{
    LPTIM1_Flag = LPTIM_Flag;
    arr = arr*500;//32000/64��Ƶ
//    HAL_LPTIM_SetOnce_Start_IT(&hlptim1,0,arr);
}
/**
  * @��飺�ú���Ϊ�͹��Ķ�ʱ���жϻص�������              
  * @������ hlptimΪ�͹��Ķ�ʱ������ 
  * @����ֵ����
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
  * @��飺�ú���Ϊ�͹��Ķ�ʱ���ж����á�              
  * @������ hlptimΪ�͹��Ķ�ʱ������ 
  * @����ֵ����
  */
void HAL_LPTIM_MspInit(void)
{
}

//-----------------------------GPIO-----------------------------//
//�ò��ֺ���Ϊϵͳ�õ���GPIO�ĳ�ʼ���������û������Լ���ƽ̨��Ӧ�޸�
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



