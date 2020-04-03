/**
******************************************************************************
* @file    M90_M91_HAL.c
* @author  LSD RF/LoRaWAN Team
* @version V0.0.0
* @date    2-April-2018
* @brief   This file contains all the functions prototypes for the HAL
*          module driver.
*/

#ifndef __M90_M91_HAL_H
#define __M90_M91_HAL_H

#include <stdint.h>

/* LoRawan LoRaNode pin configuration */
#define LoRaNode_WAKE_PIN					GPIO_PIN_3
#define LoRaNode_WAKE_GPIO_PORT         	GPIOC

#define LoRaNode_MODE_PIN					GPIO_PIN_0
#define LoRaNode_MODE_GPIO_PORT         	GPIOA

#define LoRaNode_NRST_PIN					GPIO_PIN_1
#define LoRaNode_NRST_GPIO_PORT         	GPIOA

/* INPUT */													
#define LoRaNode_STAT_PIN					GPIO_PIN_4
#define LoRaNode_STAT_GPIO_PORT         	GPIOC

#define LoRaNode_BUSY_PIN					GPIO_PIN_5
#define LoRaNode_BUSY_GPIO_PORT         	GPIOC

#define RECEIVELEN 1024

typedef struct  
{  
    uint8_t receive_flag;						
    uint16_t rx_len;			
    uint8_t RX_Buf[RECEIVELEN];          
}LoRaNode_USART_RECEIVETYPE; 

typedef enum  
{  
    LoRaNode_ReadTimeout=0,			
    LoRaNode_BUSYTimeout,
    LoRaNode_LPsendTimeout,			  
    reserve,            
}LPTIM1_FlagTypeDef;

extern LoRaNode_USART_RECEIVETYPE LoRaNode_UART;
extern UART_HandleTypeDef hlpuart1;                    
extern TIM_HandleTypeDef TIM3_UARTHandler;

extern LPTIM_HandleTypeDef hlptim1;

extern LPTIM1_FlagTypeDef LPTIM1_Flag;
extern uint8_t LoRaNode_ReadTimeout_flag,LoRaNode_BUSYTimeout_flag,LoRaNode_LPsend_flag;

extern uint8_t Send_Sate_Sign;


//-----------------------------UART-----------------------------//
void TIM3_UART_Init_ms(uint16_t arr);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void LoRaNode_UART_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void LoRaNode_UART_Send_String(uint8_t *str);
void LoRaNode_UART_Send_Byte(uint8_t data);

//----------------------------LPTIM1-----------------------------//
void LPTIM1_Init(void);
void LPTIM1_SingleStart_s(uint16_t arr,LPTIM1_FlagTypeDef LPTIM_Flag);
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* hlptim);

//-----------------------------GPIO-----------------------------//
void LoRaNode_GPIO_Init(void);

//----------------------------Delay-----------------------------//
void Delay_ms(uint32_t Delay);

#endif

