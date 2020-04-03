/**
  ******************************************************************************
  * 文件名 ：   M90_M91_Interface.c
  * 作者   ：   LSD RF/LoRaWAN Team
  * 版本   ：   V1.0.0
  * 时间   ：   19-May-2018
  * 文件描述：
  *     该文件为M90_M91模块的驱动层，包含对M90模块的各个状态控制，AT指令函数，
  *串口通信函数以及一些数据处理的公共函数
  *    客户在使用M90模块时候需要移植该文件，如果M90_M91_HAL.c中各个函数功能都已
  *实现且函数名等都没有改动，那本文件可以不用做更改直接使用，客户在应用层直接调
  *用本文件函数就可以实现各种操作。
*******************************************************************************/

#include "M90_M91_Interface.h"
#include <string.h>

#define NET_MODE

uint8_t EVK_Version[] = "LoRawan LoRaNode EVK Ver1.0.00 Apr 25 2018 20:45:28";
char *M91_Vel = NULL;    
LoRaNode_Info LoRaNode;
LoRaNode_Info *LoRaNode_str = &LoRaNode;

Status_Info LoRaNode_Status;
Status_Info *LoRaNode_Status_str = &LoRaNode_Status;

#define RXLEN    256
uint8_t AT_Data_buf[RXLEN];

//--------------------------------M90状态控制函数集-------------------------------//
/**
  * @简介：该函数用于M90模块进行初始化。              
  * @参数： 无  
  * @返回值：无
  */
void M90_M91_Init(void)
{
    LoRaNode_GPIO_Init();   
    LoRaNode_UART_Init();
    LPTIM1_Init();
    LoRaNode_Reset();
    LoRaNode_SetWake(Mode_WakeUp);   
    LoRaNode_SetMode(Mode_CMD);	
    Delay_ms(100);    
}

/**
  * @简介：该函数用于切换M90模块工作模式。              
  * @参数： Mode = Mode_CMD命令模式，Mode = Mode_Transparent透传模式  
  * @返回值：无
  */
void LoRaNode_SetMode(LoRaNode_Mode_T Mode)
{
    if (Mode == Mode_CMD )
        LoRaNode_MODE_HIGH();
    if (Mode == Mode_Transparent)
        LoRaNode_MODE_LOW();
}

/**
  * @简介：该函数用于切换M90模块状态。              
  * @参数： Mode = Mode_WakeUp激活状态，Mode = Mode_Sleep休眠状态  
  * @返回值：无
  */
void LoRaNode_SetWake(LoRaNode_SleepMode_T Mode)
{
    if (Mode == Mode_WakeUp)
    {
        if(HAL_GPIO_ReadPin(LoRaNode_WAKE_GPIO_PORT,LoRaNode_WAKE_PIN) != GPIO_PIN_SET)
        {
            LoRaNode_WAKE_HIGH();            
            Delay_ms(100);                
        }
    }
    
    if (Mode == Mode_Sleep)
    {
        if(HAL_GPIO_ReadPin(LoRaNode_WAKE_GPIO_PORT,LoRaNode_WAKE_PIN) != GPIO_PIN_RESET)
        {
            LoRaNode_WAKE_LOW();
            Delay_ms(100);    
        }
    }
}

/**
  * @简介：该函数用于复位M90模块。              
  * @参数： 无  
  * @返回值：无
  */
void LoRaNode_Reset(void)
{
    LoRaNode_NRST_LOW();        
    Delay_ms(15);    //15ms    
    LoRaNode_NRST_HIGH();        
    Delay_ms(15);    //15ms
}

//--------------------------------AT指令函数集-------------------------------//
/**
  * @简介：该函数用于读取M90固件版本。              
  * @参数： 无  
  * @返回值：固件版本
  */
char *LoRaNode_GetVer(void)
{
    uint8_t ASK_Ver[] = "AT+Ver?\r\n";
    char *temp = "+VER:";
    memset(AT_Data_buf,0,RXLEN);              
    LoRaNode_Send_AT(ASK_Ver);    
    Delay_ms(50); 
    LoRaNode_Read(AT_Data_buf);        
    if(StringStr((char *)AT_Data_buf, temp) != NULL)
    {
        M91_Vel = StringStr((char *)AT_Data_buf, temp);
        return M91_Vel;
    }    
    return M91_Vel;
}
/**
  * @简介：该函数用于通过AT指令读取M90各个参数（返回值可以通过指针形式返回）。              
  * @参数： AT_Command AT指令，  *AT_Value M90返回值
  * @返回值：AT_Value
  */
void LoRaNode_Getpoint(uint8_t *AT_Command,uint8_t *AT_Value)
{
    uint8_t Command[20] = {0};
    uint8_t Check[10] = {0};
    char *ptr = NULL;   
    uint8_t stringlen;
    uint8_t len = 0;
    
    stringlen = strlen((const char*)AT_Command);
    strcpy((char*)Command,(const char*)AT_Command);   
    Command[stringlen++] = '\r';
    Command[stringlen++] = '\n';
    Command[stringlen] = '\0';
    
    memset(AT_Data_buf,0,RXLEN);               
    LoRaNode_Send_AT(Command);    
    LoRaNode_Read(AT_Data_buf);
    strncpy((char*)Check,(const char*)AT_Command+2,stringlen-5);
    if((ptr = StringStr((char *)AT_Data_buf, (char*)Check)) != NULL)
    {
        if(!(strcmp((const char*)Check,"+DEVEUI")&&strcmp((const char*)Check,"+APPEUI")))
        {
            len = 8;
        }
        if(!strcmp((const char*)Check,"+DEVADDR"))
        {
            len = 4;
        }
        if(!(strcmp((const char*)Check,"+APPKEY")&&strcmp((const char*)Check,"+APPSKEY")\
            &&strcmp((const char*)Check,"++NWKSKEY")))
        {
            len = 16;
        }
        if(len > 0)
        {
            for(uint8_t i=0;i<len;i++)
            {
                uint8_t temp1=0,temp2=0;
                
                temp1 = *((ptr + stringlen - 4+(3*i))+1);
                temp2 = *((ptr + stringlen - 4+(3*i))+2);
                if(temp1 > 0x40)
                {
                    temp1 = temp1 - 55;
                }else{
                    temp1 = temp1 - 48;
                }
                if(temp2 > 0x40)
                {
                    temp2 = temp2 - 55;
                }else{
                    temp2 = temp2 - 48;
                }
                AT_Value[i] = temp1*16 + temp2;
            }
        }
        else
        {
            AT_Value[0]  = (*(ptr + stringlen - 3)) - 0x30; 
        }
    }   
}

/**
  * @简介：该函数用于通过AT指令读取M90状态值。              
  * @参数： *LoRa_temp 存放返回的状态值
  * @返回值：LoRa_temp
  */
void LoRaNode_GetState(Status_Info *LoRa_temp)                                    
{
    uint8_t GetSTATUS[] = "AT+STATUS?\r\n";
    char *temp = "+STATUS:";
    char *ptr = NULL;    
    uint8_t dec=0,dec1=0,dec2=0,dec3=0,dec4=0,dec5=0,dec6=0,dec7=0;
    
    memset(AT_Data_buf,0,RXLEN);            
    LoRaNode_Send_AT(GetSTATUS);       
    LoRaNode_Read(AT_Data_buf);        
    if((ptr = StringStr((char *)AT_Data_buf, temp)) != NULL)
    {
        dec =StrToHex( *(ptr + 9));
        dec1 = StrToHex( *(ptr + 10));
        
        LoRa_temp->Up_Result = dec*16 + dec1;
        
        dec = StrToHex( *(ptr + 12));
        dec1 = StrToHex( *(ptr + 13));
        
        LoRa_temp->Up_CH = dec*16 + dec1;
        
        dec = StrToHex( *(ptr + 15));
        dec1 = StrToHex( *(ptr + 16));
        
        LoRa_temp->Up_Rate = dec*16 + dec1;
        
        dec = StrToHex( *(ptr + 18));
        dec1 = StrToHex( *(ptr + 19));
        
        LoRa_temp->Up_TxPower = dec*16 + dec1;
        
        dec = StrToHex( *(ptr + 66));
        dec1 = StrToHex( *(ptr + 67));
        dec2 = StrToHex( *(ptr + 69));
        dec3 = StrToHex( *(ptr + 70));
        
        LoRa_temp->Up_Cache = (dec*16+ dec1) +(dec2*16 + dec3)*256 ;
        
        dec = StrToHex( *(ptr + 72));
        dec1 = StrToHex( *(ptr + 73));
        dec2 = StrToHex( *(ptr + 75));
        dec3 = StrToHex( *(ptr + 76));
        
        LoRa_temp->Up_nbTrials = (dec*16+ dec1) +(dec2*16 + dec3)*256 ;
        
        dec = StrToHex( *(ptr + 117));
        dec1 = StrToHex( *(ptr + 118));
        dec2 = StrToHex( *(ptr + 120));
        dec3 = StrToHex( *(ptr + 121));
        dec4 = StrToHex( *(ptr + 123));
        dec5 = StrToHex( *(ptr + 124));
        dec6 = StrToHex( *(ptr + 126));
        dec7 = StrToHex( *(ptr + 127));
        
        LoRa_temp->Up_Cnt = (dec*16+ dec1) +(dec2*16 + dec3)*256 + (dec4*16+ dec5)*65536 +(dec6*16 + dec7)*256*65536 ;
        
        dec = StrToHex( *(ptr + 21));
        dec1 = StrToHex( *(ptr + 22));
        LoRa_temp->Down_CH = dec*16 + dec1;
        
        dec = StrToHex( *(ptr + 24));
        dec1 = StrToHex( *(ptr + 25));
        LoRa_temp->Down_Rate = dec*16 + dec1;                
        
        dec = StrToHex( *(ptr + 30));
        dec1 = StrToHex( *(ptr + 31));
        LoRa_temp->Down_RxSlot = dec*16 + dec1;                   
        
        dec = StrToHex( *(ptr + 33));
        dec1 = StrToHex( *(ptr + 34));
        LoRa_temp->Down_Port = dec*16 + dec1;                   
        
        dec = StrToHex( *(ptr + 42));
        dec1 = StrToHex( *(ptr + 43));
        LoRa_temp->Down_SNR = dec*16 + dec1;                   
        
        dec = StrToHex( *(ptr + 48));
        dec1 = StrToHex( *(ptr + 49));
        dec2 = StrToHex( *(ptr + 51));
        dec3 = StrToHex( *(ptr + 52));
        LoRa_temp->Down_RSSI = (dec*16 + dec1) + (dec2*16 + dec3) * 256;                   
        
        dec = StrToHex( *(ptr + 54));
        dec1 = StrToHex( *(ptr + 55));
        dec2 = StrToHex( *(ptr + 57));
        dec3 = StrToHex( *(ptr + 58));
        dec4 = StrToHex( *(ptr + 60));
        dec5 = StrToHex( *(ptr + 61));
        dec6 = StrToHex( *(ptr + 63));
        dec7 = StrToHex( *(ptr + 64));
        LoRa_temp->Down_Cnt = (dec*16+ dec1) +(dec2*16 + dec3)*256 + (dec4*16+ dec5)*65536 +(dec6*16 + dec7)*256*65536 ;
        
    }    
}
/**
  * @简介：该函数用于通过AT指令设置M90的GPIO状态。              
  * @参数： pin GPIO口， state 高低电平状态
  * @返回值：0配置正确，-1配置错误
  */
int LoRaNode_SetGPIO(uint32_t pin, uint32_t state)
{    
    uint8_t GPIO[20] = "AT+GPIO=";
    uint8_t buf[5] = {0};
    char *temp = "OK";
    char *ptr = (char*)GPIO;
    
    IntToStr(buf, pin);
    strcat((char *)GPIO, (char *)buf);    
    while(*++ptr);    
    *ptr++ = ',';    
    memset(buf,0,5);
    IntToStr(buf, state);
    StringConcat(GPIO, buf);    
    memset(AT_Data_buf,0,RXLEN);           
    LoRaNode_Send_AT(GPIO);    
    LoRaNode_Read(AT_Data_buf);                
    if((ptr = StringStr((char *)AT_Data_buf, temp)) != NULL)
    {
        return 0;
    }    
    return -1;
}
/**
  * @简介：该函数用于通过AT指令设置M90的参数（适用于参数为整型）。              
  * @参数： AT_Command AT指令，AT_Value 参数值
  * @返回值：0配置正确，-1配置错误
  */
int LoRaNode_Setinteger(uint8_t *AT_Command,uint32_t AT_Value)
{    
    uint8_t Command[20] = {0};
    uint8_t buf[10] = {0};
    char *temp = "OK";
    
    strcpy((char*)Command,(const char*)AT_Command);   
    IntToStr(buf, AT_Value);
    StringConcat(Command, buf);    
    memset(AT_Data_buf,0,RXLEN);               
    LoRaNode_Send_AT(Command);    
    Delay_ms(50);   
    LoRaNode_Read(AT_Data_buf);                
    if(StringStr((char *)AT_Data_buf, temp) != NULL)
    {
        return 0;
    }    
    return -1;
}

/**
  * @简介：该函数用于通过AT指令设置M90的参数（适用于参数为数组，字符串）。              
  * @参数： AT_Command AT指令，AT_Key 参数值
  * @返回值：0配置正确，-1配置错误
  */
int LoRaNode_Setpoint(uint8_t *AT_Command,uint8_t *AT_Key)
{
    uint8_t Command[80] = {0};
    char *temp = "OK";
    
    strcpy((char*)Command,(const char*)AT_Command);
    StringConcat(Command, AT_Key);    
    memset(AT_Data_buf,0,RXLEN);               
    LoRaNode_Send_AT(Command);
    Delay_ms(100);     
    LoRaNode_Read(AT_Data_buf);                    
    if(StringStr((char *)AT_Data_buf, temp) != NULL)
    {
        return 0;
    }   
    return -1;    
}

/**
  * @简介：该函数用于通过AT指令设置M90的频点。              
  * @参数： Up_Dn 上行或者下行，Ch_Cnt 信道个数，Start_Freq 起始频点
  * @返回值：0配置正确，-1配置错误
  */
int LoRaNode_SetFreq(uint8_t Up_Dn,uint8_t Ch_Cnt,uint32_t Start_Freq)
{
    uint8_t FREQ[30];
    char *temp = "OK";
    
    sprintf(FREQ,"AT+FREQ=%d,%d,%d\r\n\0",Up_Dn,Ch_Cnt,Start_Freq);    
    memset(AT_Data_buf,0,RXLEN);               
    LoRaNode_Send_AT(FREQ);    
    LoRaNode_Read(AT_Data_buf);                    
    if(StringStr((char *)AT_Data_buf, temp) != NULL)
    {
        return 0;
    }    
    return -1;
}

/**
  * @简介：该函数用于在M90工作在Mini RF模式下，通过AT指令设置Mini RF的射频参数。              
  * @参数： 详见说明手册
  * @返回值：0配置正确，-1配置错误
  */
int LoRaNode_SetP2P(uint32_t f,uint8_t a,uint8_t b,uint8_t c,uint8_t d,uint8_t e,uint8_t ff,uint8_t g,uint16_t h)
{
    uint8_t SetDebug[50] = "AT+RADIO=";
    uint8_t buf[10] = {0}; 
    uint8_t buf1[10] = {0}; 
    uint8_t buf2[10] = {0}; 
    uint8_t buf3[10] = {0}; 
    uint8_t buf4[10] = {0}; 
    uint8_t buf5[10] = {0}; 
    uint8_t buf6[10] = {0}; 
    uint8_t buf7[10] = {0}; 
    uint8_t buf8[10] = {0}; 
    
    uint8_t dou[2] = ",";
    char *temp = "OK";
    
    IntToStr(buf, f);
    StringConcat2(SetDebug, buf);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf1, a);
    StringConcat2(SetDebug, buf1);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf2, b);
    StringConcat2(SetDebug, buf2);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf3, c);
    StringConcat2(SetDebug, buf3);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf4, d);
    StringConcat2(SetDebug, buf4);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf5, e);
    StringConcat2(SetDebug, buf5);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf6, ff);
    StringConcat2(SetDebug, buf6);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf7, g);
    StringConcat2(SetDebug, buf7);
    StringConcat2(SetDebug, dou);
    
    IntToStr(buf8, h);
    StringConcat(SetDebug, buf8);
    
    memset(AT_Data_buf,0,RXLEN);           
    
    LoRaNode_Send_AT(SetDebug);
    Delay_ms(50);
    LoRaNode_Read(AT_Data_buf);                
    
    if(StringStr((char *)AT_Data_buf, temp) != NULL)
    {
        return 0;
    }
    
    return -1;
}

//--------------------------------串口通信函数集-------------------------------//
/**
  * @简介：通过串口向M90模块发送数据。              
  * @参数： pdata 要发送的数据， Length 长度  
  * @返回值：无
  */
void LoRaNode_SendData(uint8_t *pdata, uint16_t Length)  
{  
    uint32_t i = 0;
    
    for (i = 0; i < Length; i++)
    {
        LoRaNode_UART_Send_Byte(pdata[i]);
    }
}

/**
  * @简介：通过串口向M90模块发送AT指令。              
  * @参数： at_buf 要发送的AT指令  
  * @返回值：无
  */
void LoRaNode_Send_AT(uint8_t *at_buf)
{
    LoRaNode_UART_Send_String(at_buf);
}

/**
  * @简介：通过串口读取M90模块发回的数据。              
  * @参数： str 发回的数据  
  * @返回值：无
  */
void LoRaNode_Read(uint8_t *str)
{
    uint32_t i = 0;
    LPTIM1_SingleStart_s(2,LoRaNode_ReadTimeout);
    while((LoRaNode_UART.receive_flag != 1)&&(LoRaNode_ReadTimeout_flag != 1))
    {        
    }
    HAL_LPTIM_SetOnce_Stop_IT(&hlptim1);
    LoRaNode_UART.receive_flag = 0;
    LoRaNode_ReadTimeout_flag = 0;
    
    for(i=0 ; i < LoRaNode_UART.rx_len; i++)
    {
        str[i] = LoRaNode_UART.RX_Buf[i];
    }    
    LoRaNode_UART.rx_len = 0;
}

//--------------------------------公共函数集-------------------------------//
/**
  * @简介：判断一个字符串中是否包含另一个字符串。              
  * @参数：   
  * @返回值：NULL 不包含
  */
char *StringStr(char *str, char *dest)
{
#define STR_BUFF_LEN    0x100
    int i = STR_BUFF_LEN;
    char *cp = str;
    char *s1, *s2;
    
    if (*dest == 0)
    {
        return str;
    }
    
    while(i--)
    {        
        s1 = cp;
        s2 = dest;
        
        while((*s1 == *s2) && *s1 && *s2)
        {
            s1++;
            s2++;
        }
        if(!*s2)
            return cp;
        cp++;
    }
    
    return NULL;
}



/**
  * @简介：字符串转换为16进制。              
  * @参数：   
  * @返回值：无
  */
uint8_t StrToHex(uint8_t temp)
{
    uint8_t ret=0;
    
    if(temp>=48 && temp<=57)
    {
        ret = temp - 48;
        return ret;
    }
    
    if(temp>=65 && temp<=70)
    {
        ret = temp - 55;
        return ret;
    }
    
    if(temp>=97 && temp<=102)
    {
        ret = temp - 87;
        return ret;
    }
    
    return ret; 
}

/**
  * @简介：连接两个字符串并在最后加上回车。              
  * @参数：   
  * @返回值：无
  */
uint8_t *StringConcat(uint8_t *str, const uint8_t *string)
{
    uint8_t *s = str;
    
    while(*s)
    {
        s++;
    }
    
    while(*string)
    {
        *s++ = *string++;
    }
    
    *s++ = '\r';
    *s++ = '\n';
    *s = '\0';
    
    return str;     
}

/**
  * @简介：连接两个字符串。              
  * @参数：   
  * @返回值：无
  */
uint8_t *StringConcat2(uint8_t *str, const uint8_t *string)
{
    uint8_t *s = str;
    
    while(*s)
    {
        s++;
    }
    
    while(*string)
    {
        *s++ = *string++;
    }
    
    return str;        
}

/**
  * @简介：整型转字符串。              
  * @参数：   
  * @返回值：无
  */
void IntToStr(uint8_t* str, int32_t intnum)
{
    uint32_t i, Div = 1000000000, j = 0, Status = 0;
    
    if(intnum < 0)
    {
        intnum = intnum*(-1);
        str[j++] = '-';
    }
    
    for (i = 0; i < 10; i++)
    {
        str[j++] = (intnum / Div) + 48; /* '0' */
        
        intnum = intnum % Div;
        Div /= 10;
        if ((str[j-1] == '0') & (Status == 0))
        {
            j = 0;
        }
        else
        {
            Status++;
        }
    }
}

/**
  * @简介：数组复制。              
  * @参数：   
  * @返回值：无
  */
int CopyArray(uint8_t *str, const uint8_t *string)
{
    if(str == NULL | *string == NULL)
        return -1;    
    while(*string)
    {
        *str++ = *string++;
    }
    return 0;
}

void LoRaNode_ErrorHandler(void)
{   
    while(1) 
    {
    }
}

