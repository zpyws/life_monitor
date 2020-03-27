/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-22     zylx         first version 
 */

#ifndef __QSPI_CONFIG_H__
#define __QSPI_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_QSPI

//by yangwensen@20200316
/* SMIF configuration parameters */
#define DESELECT_DELAY      (7u)
#define RX_CLOCK_SELECT     (1u)
#define AHB_BUS_ERROR       (0u)
#define SMIF_PRIORITY       (1u)
    
#define TIMEOUT_1_MS              (1000ul)  /* 1 ms timeout for all blocking functions */
#define SMIF_EnableInt()            NVIC_EnableIRQ(QSPI1_SMIF_IRQ_cfg.intrSrc)
#define TX_NOT_LAST_BYTE          (0u)   	/* Not the last byte in command transmission */
#define TX_LAST_BYTE       	      (1u) 	    /* The last byte in command transmission */
#define CY_SMIF_CMD_WITHOUT_PARAM  (0U)   



#ifndef QSPI_BUS_CONFIG
#define QSPI_BUS_CONFIG                                        \
    {                                                          \
        .spicfg.mode = CY_SMIF_NORMAL,                         \
        .spicfg.deselectDelay = DESELECT_DELAY,                \
        .spicfg.rxClockSel = RX_CLOCK_SELECT,                  \
        .spicfg.blockEvent = AHB_BUS_ERROR,                    \
        .intcfg.intrSrc = (IRQn_Type)smif_interrupt_IRQn,      \
        .intcfg.intrPriority = SMIF_PRIORITY,                  \
    }
#endif /* QSPI_BUS_CONFIG */
#endif /* BSP_USING_QSPI */

#ifdef BSP_QSPI_USING_DMA
#ifndef QSPI_DMA_CONFIG
#define QSPI_DMA_CONFIG                                        \
    {                                                          \
        .Instance = QSPI_DMA_INSTANCE,                         \
        .Init.Channel  = QSPI_DMA_CHANNEL,                     \
        .Init.Direction = DMA_PERIPH_TO_MEMORY,                \
        .Init.PeriphInc = DMA_PINC_DISABLE,                    \
        .Init.MemInc = DMA_MINC_ENABLE,                        \
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,       \
        .Init.MemDataAlignment = DMA_MDATAALIGN_BYTE,          \
        .Init.Mode = DMA_NORMAL,                               \
        .Init.Priority = DMA_PRIORITY_LOW                      \
    }
#endif /* QSPI_DMA_CONFIG */
#endif /* BSP_QSPI_USING_DMA */

#define QSPI_IRQn                   QUADSPI_IRQn
#define QSPI_IRQHandler             QUADSPI_IRQHandler

#ifdef __cplusplus
}
#endif

#endif /* __QSPI_CONFIG_H__ */
