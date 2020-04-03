#ifndef __M90_M91_CONT_H
#define __M90_M91_CONT_H

#include "stm32l4xx_hal.h"
#include "M90_M91_HAL.h"

#define Up_FreqSign        1
#define Down_FreqSign      3


typedef enum {
  	Mode_Transparent,
	Mode_CMD,		//mode pin high
} LoRaNode_Mode_T;

typedef enum {
	NET_ABP,
	NET_OTAA,
} LoRaNode_NetMode_T;

typedef enum {
	Mode_WakeUp,
	Mode_Sleep,	
} LoRaNode_SleepMode_T;

typedef enum {
	Class_A,
	Class_B,
    Class_C,
} LoRaNode_Class;

typedef struct {
    
	uint8_t Ver;  
	uint8_t Join;
	LoRaNode_Mode_T Mode;
	uint8_t OnLine;
	uint8_t Busy;
	LoRaNode_NetMode_T NET_Mode;
    
    
	uint8_t DevEUI[20];	
	uint8_t AppEUI[20];	
	uint8_t AppKEY[36]; 
    
	uint8_t DevADDR[12];		
	uint8_t AppSKEY[36];	
	uint8_t NwkSKEY[36]; 
    
    
  LoRaNode_Class Class_x;          
	uint8_t Confirm;
	uint8_t ADR;
	uint8_t Band;
	uint8_t Up_Channel_Cnt;
	uint32_t Up_Start_Freq;             
	uint8_t Down_Channel_Cnt;
	uint32_t Down_Start_Freq; 
	uint32_t Freq[16];             
	uint8_t RX2_DR;
	uint8_t RX2_Freq;       
	uint8_t Debug;
	uint8_t Power;
    
    
} LoRaNode_Info;

typedef struct {
    
	uint8_t Up_Result;
	uint8_t Up_CH;	
	uint8_t Up_Rate;                                //即上行SF	
	uint8_t Up_TxPower;    
    uint16_t Up_Cache;
	uint16_t Up_nbTrials;
    uint32_t Up_Cnt;   
    
    uint8_t Down_CH;
    uint8_t Down_Rate;	  
    uint8_t Down_RxSlot;
    uint8_t Down_Port;
    int16_t Down_RSSI;
	int8_t Down_SNR;
    uint32_t Down_Cnt;        
    
    
    
}Status_Info;


#define LoRaNode_WAKE_HIGH()      HAL_GPIO_WritePin(LoRaNode_WAKE_GPIO_PORT, LoRaNode_WAKE_PIN, GPIO_PIN_SET)
#define LoRaNode_WAKE_LOW()     HAL_GPIO_WritePin(LoRaNode_WAKE_GPIO_PORT, LoRaNode_WAKE_PIN, GPIO_PIN_RESET)													

#define LoRaNode_MODE_HIGH()      HAL_GPIO_WritePin(LoRaNode_MODE_GPIO_PORT, LoRaNode_MODE_PIN, GPIO_PIN_SET)
#define LoRaNode_MODE_LOW()     HAL_GPIO_WritePin(LoRaNode_MODE_GPIO_PORT, LoRaNode_MODE_PIN, GPIO_PIN_RESET) 

#define LoRaNode_NRST_HIGH()      HAL_GPIO_WritePin(LoRaNode_NRST_GPIO_PORT, LoRaNode_NRST_PIN, GPIO_PIN_SET)
#define LoRaNode_NRST_LOW()     HAL_GPIO_WritePin(LoRaNode_NRST_GPIO_PORT, LoRaNode_NRST_PIN, GPIO_PIN_RESET)

#define LoRaNode_STAT_STATUS()	HAL_GPIO_ReadPin(LoRaNode_STAT_GPIO_PORT, LoRaNode_STAT_PIN)///
#define LoRaNode_BUSY_STATUS()	HAL_GPIO_ReadPin(LoRaNode_BUSY_GPIO_PORT, LoRaNode_BUSY_PIN)
#define LoRaNode_MODE_STATUS()	HAL_GPIO_ReadPin(LoRaNode_MODE_GPIO_PORT, LoRaNode_MODE_PIN)

extern LoRaNode_Info LoRaNode;
extern LoRaNode_Info *LoRaNode_str;

extern Status_Info LoRaNode_Status;
extern Status_Info *LoRaNode_Status_str;


extern uint8_t TimeOut_Sign;
extern uint8_t appEui[];
extern uint8_t appKey[];
extern uint8_t appSKey[];
extern uint8_t nwkSKey[];

//--------------------------------M90状态控制函数集-------------------------------//
void M90_M91_Init(void);
void LoRaNode_SetMode(LoRaNode_Mode_T Mode);
void LoRaNode_SetWake(LoRaNode_SleepMode_T Mode);
void LoRaNode_Reset(void);
//----------------------------------AT指令函数集----------------------------------//
char *LoRaNode_GetVer(void);
void LoRaNode_Getpoint(uint8_t *AT_Command,uint8_t *AT_Value);
void LoRaNode_GetState(Status_Info *LoRa_temp);

int LoRaNode_SetGPIO(uint32_t pin, uint32_t state);
// 该函数用于通过AT指令设置M90的参数（适用于参数为整型）
int LoRaNode_Setinteger(uint8_t *AT_Command,uint32_t AT_Value);
// 该函数用于通过AT指令设置M90的参数（适用于参数为数组，字符串）。 
int LoRaNode_Setpoint(uint8_t *AT_Command,uint8_t *AT_Key);
// 该函数用于在M90工作在Mini RF模式下，通过AT指令设置Mini RF的射频参数。  
int LoRaNode_SetP2P(uint32_t f,uint8_t a,uint8_t b,uint8_t c,uint8_t d,uint8_t e,uint8_t ff,uint8_t g,uint16_t h);
int LoRaNode_SetFreq(uint8_t Up_Dn,uint8_t Ch_Cnt,uint32_t Start_Freq);
    
//---------------------------------串口通信函数集---------------------------------//
void LoRaNode_SendData(uint8_t *pdata, uint16_t Length);
void LoRaNode_Send_AT(uint8_t *at_buf);
void LoRaNode_Read(uint8_t *str);

//----------------------------------公共函数集------------------------------------//
char *StringStr(char *str, char *dest);
uint8_t StrToHex(uint8_t temp);
uint8_t *StringConcat(uint8_t *str, const uint8_t *string);
uint8_t *StringConcat2(uint8_t *str, const uint8_t *string);
void IntToStr(uint8_t* str, int32_t intnum);
int CopyArray(uint8_t *str, const uint8_t *string);
void LoRaNode_ErrorHandler(void);

#endif 

