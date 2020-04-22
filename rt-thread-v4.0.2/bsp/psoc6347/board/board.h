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

#define CY8C63_FLASH_START_ADRESS     ((uint32_t)0x10080000)
#define CY8C63_FLASH_SIZE             (512 * 1024)
#define CY8C63_FLASH_END_ADDRESS      ((uint32_t)(CY8C63_FLASH_START_ADRESS + CY8C63_FLASH_SIZE))

#define CY8C63_SRAM_SIZE           (288-2)                                  //by yangwensen@20200311
#define CY8C63_SRAM_END            (0x08000000 + CY8C63_SRAM_SIZE * 1024)   //by yangwensen@20200311

#if defined(__CC_ARM) || defined(__CLANG_ARM)
extern int Image$$ER_RAM_NOINIT_DATA$$ZI$$Limit;							//by yangwensen@20200422
#define HEAP_BEGIN      (&Image$$ER_RAM_NOINIT_DATA$$ZI$$Limit)				//by yangwensen@20200422
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
