/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-30     zylx         first version
 */

#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(BSP_USING_UART0)
#ifndef UART0_CONFIG
#define UART0_CONFIG                                                \
    {                                                               \
        .name = "uart0",                                            \
        .uart_base = UART0_HW,                                      \
        .uart_config = &UART0_config,                               \
        .context = &UART0_context,                                  \
        .irq_config = &UART0_SCB_IRQ_cfg,                           \
        .isr_handler = UART0_IRQHandler,                            \
    }
#endif /* UART0_CONFIG */
#endif /* BSP_USING_UART0 */

#if defined(BSP_UART0_RX_USING_DMA)
#ifndef UART0_DMA_RX_CONFIG
#define UART0_DMA_RX_CONFIG                                         \
    {                                                               \
    }
#endif /* UART0_DMA_RX_CONFIG */
#endif /* BSP_UART0_RX_USING_DMA */

#if defined(BSP_USING_UART1)
#ifndef UART1_CONFIG
#define UART1_CONFIG                                                \
    {                                                               \
        .name = "uart1",                                            \
        .uart_base = UART1_HW,                                      \
        .uart_config = &UART1_config,                               \
        .context = &UART1_context,                                  \
        .irq_config = &UART1_SCB_IRQ_cfg,                           \
        .isr_handler = UART1_IRQHandler,                            \
    }
#endif /* UART1_CONFIG */
#endif /* BSP_USING_UART1 */

#if defined(BSP_UART1_RX_USING_DMA)
#ifndef UART1_DMA_RX_CONFIG
#define UART1_DMA_RX_CONFIG                                         \
    {                                                               \
        .Instance = UART1_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART1_RX_DMA_RCC,                               \
        .dma_irq  = UART1_RX_DMA_IRQ,                               \
    }
#endif /* UART1_DMA_RX_CONFIG */
#endif /* BSP_UART1_RX_USING_DMA */

#if defined(BSP_USING_UART2)
#ifndef UART2_CONFIG
#define UART2_CONFIG                                                \
    {                                                               \
        .name = "uart2",                                            \
        .uart_base = UART2_HW,                                      \
        .uart_config = &UART2_config,                               \
        .context = &UART2_context,                                  \
        .irq_config = &UART2_SCB_IRQ_cfg,                           \
        .isr_handler = UART2_IRQHandler,                            \
    }
#endif /* UART2_CONFIG */
#endif /* BSP_USING_UART2 */
    
#if defined(BSP_UART2_RX_USING_DMA)
#ifndef UART2_DMA_RX_CONFIG
#define UART2_DMA_RX_CONFIG                                            \
    {                                                               \
        .Instance = UART2_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART2_RX_DMA_RCC,                               \
        .dma_irq  = UART2_RX_DMA_IRQ,                               \
    }
#endif /* UART2_DMA_RX_CONFIG */
#endif /* BSP_UART2_RX_USING_DMA */

#if defined(BSP_USING_UART3)
#ifndef UART3_CONFIG
#define UART3_CONFIG                                                \
    {                                                               \
        .name = "uart3",                                            \
        .uart_base = UART3_HW,                                      \
        .uart_config = &UART3_config,                               \
        .context = &UART3_context,                                  \
        .irq_config = &UART3_SCB_IRQ_cfg,                           \
        .isr_handler = UART3_IRQHandler,                            \
    }
#endif /* UART3_CONFIG */
#endif /* BSP_USING_UART3 */
    
#if defined(BSP_UART3_RX_USING_DMA)
#ifndef UART3_DMA_RX_CONFIG
#define UART3_DMA_RX_CONFIG                                         \
    {                                                               \
    }
#endif /* UART3_DMA_RX_CONFIG */
#endif /* BSP_UART3_RX_USING_DMA */

#if defined(BSP_USING_UART4)
#ifndef UART4_CONFIG
#define UART4_CONFIG                                                \
    {                                                               \
        .name = "uart4",                                            \
        .uart_base = UART4_HW,                                      \
        .uart_config = &UART4_config,                               \
        .context = &UART4_context,                                  \
        .irq_config = &UART4_SCB_IRQ_cfg,                           \
        .isr_handler = UART4_IRQHandler,                            \
    }
#endif /* UART4_CONFIG */
#endif /* BSP_USING_UART4 */
    
#if defined(BSP_UART4_RX_USING_DMA)
#ifndef UART4_DMA_RX_CONFIG
#define UART4_DMA_RX_CONFIG                                         \
    {                                                               \
    }
#endif /* UART4_DMA_RX_CONFIG */
#endif /* BSP_UART4_RX_USING_DMA */

#if defined(BSP_USING_UART5)
#ifndef UART5_CONFIG
#define UART5_CONFIG                                                \
    {                                                               \
        .name = "uart5",                                            \
        .uart_base = UART5_HW,                                      \
        .uart_config = &UART5_config,                               \
        .context = &UART5_context,                                  \
        .irq_config = &UART5_SCB_IRQ_cfg,                           \
        .isr_handler = UART5_IRQHandler,                            \
    }
#endif /* UART5_CONFIG */
#endif /* BSP_USING_UART5 */
    
#if defined(BSP_UART5_RX_USING_DMA)
#ifndef UART5_DMA_RX_CONFIG
#define UART5_DMA_RX_CONFIG                                         \
    {                                                               \
        .Instance = UART5_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART5_RX_DMA_RCC,                               \
        .dma_irq  = UART5_RX_DMA_IRQ,                               \
    }
#endif /* UART5_DMA_RX_CONFIG */
#endif /* BSP_UART5_RX_USING_DMA */

#if defined(BSP_USING_UART6)
#ifndef UART6_CONFIG
#define UART6_CONFIG                                                \
    {                                                               \
        .name = "uart6",                                            \
        .uart_base = UART6_HW,                                      \
        .uart_config = &UART6_config,                               \
        .context = &UART6_context,                                  \
        .irq_config = &UART6_SCB_IRQ_cfg,                           \
        .isr_handler = UART6_IRQHandler,                            \
    }
#endif /* UART6_CONFIG */
#endif /* BSP_USING_UART6 */
    
#if defined(BSP_UART6_RX_USING_DMA)
#ifndef UART6_DMA_RX_CONFIG
#define UART6_DMA_RX_CONFIG                                            \
    {                                                               \
        .Instance = UART6_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART6_RX_DMA_RCC,                               \
        .dma_irq  = UART6_RX_DMA_IRQ,                               \
    }
#endif /* UART6_DMA_RX_CONFIG */
#endif /* BSP_UART6_RX_USING_DMA */

#if defined(BSP_USING_UART7)
#ifndef UART7_CONFIG
#define UART7_CONFIG                                                \
    {                                                               \
        .name = "uart7",                                            \
        .uart_base = UART7_HW,                                      \
        .uart_config = &UART7_config,                               \
        .context = &UART7_context,                                  \
        .irq_config = &UART7_SCB_IRQ_cfg,                           \
        .isr_handler = UART7_IRQHandler,                            \
    }
#endif /* UART7_CONFIG */
#endif /* BSP_USING_UART7 */
    
#if defined(BSP_UART7_RX_USING_DMA)
#ifndef UART7_DMA_RX_CONFIG
#define UART7_DMA_RX_CONFIG                                         \
    {                                                               \
    }
#endif /* UART7_DMA_RX_CONFIG */
#endif /* BSP_UART7_RX_USING_DMA */

#if defined(BSP_USING_UART8)
#ifndef UART8_CONFIG
#define UART8_CONFIG                                                \
    {                                                               \
        .name = "uart8",                                            \
        .uart_base = UART8_HW,                                      \
        .uart_config = &UART8_config,                               \
        .context = &UART8_context,                                  \
        .irq_config = &UART8_SCB_IRQ_cfg,                           \
        .isr_handler = UART8_IRQHandler,                            \
    }
#endif /* UART8_CONFIG */
#endif /* BSP_USING_UART8 */
    
#if defined(BSP_UART8_RX_USING_DMA)
#ifndef UART8_DMA_RX_CONFIG
#define UART8_DMA_RX_CONFIG                                         \
    {                                                               \
    }
#endif /* UART8_DMA_RX_CONFIG */
#endif /* BSP_UART8_RX_USING_DMA */

#ifdef __cplusplus
}
#endif

#endif /* __UART_CONFIG_H__ */
