/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      balanceTWK   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtthread.h>
#include <project.h>            //by yangwensen
#include "drv_common.h"
#include "drv_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32_FLASH_START_ADRESS     ((uint32_t)0x08000000)
#define STM32_FLASH_SIZE             (1024 * 1024)
#define STM32_FLASH_END_ADDRESS      ((uint32_t)(STM32_FLASH_START_ADRESS + STM32_FLASH_SIZE))

#define CY8C63_SRAM_SIZE           (128)                                    //by yangwensen@20200311
#define CY8C63_SRAM_END            (0x08000000 + CY8C63_SRAM_SIZE * 1024)   //by yangwensen@20200311

#if defined(__CC_ARM) || defined(__CLANG_ARM)
extern int Image$$RW_IRAM1$$ZI$$Limit;
//#define HEAP_BEGIN      (&Image$$RW_IRAM1$$ZI$$Limit)
#define HEAP_BEGIN      (CY8C63_SRAM_END-64L*1024L)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#else
extern int __bss_end;
#define HEAP_BEGIN      (&__bss_end)
#endif

#define HEAP_END        CY8C63_SRAM_END     //by yangwensen@20200311

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif
