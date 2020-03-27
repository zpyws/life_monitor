//by yangwensen@20200320
#ifndef __I2C_CONFIG_H__
#define __I2C_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(BSP_USING_I2C1)
#ifndef I2C1_CONFIG
#define I2C1_CONFIG                                                 \
    {                                                               \
        .name = "i2c1",                                             \
        .i2c_base = GSENSOR_COMPASS_I2C_HW,                         \
        .irq_config = &GSENSOR_COMPASS_I2C_SCB_IRQ_cfg,             \
        .context = &GSENSOR_COMPASS_I2C_context,    	            \
		.i2c_init = GSENSOR_COMPASS_I2C_Start,						\
    }
#endif 
#endif

#if defined(BSP_I2C1_USING_DMA)
#ifndef UART1_DMA_RX_CONFIG
#define UART1_DMA_RX_CONFIG                                         \
    {                                                               \
        .Instance = UART1_RX_DMA_INSTANCE,                          \
        .dma_rcc  = UART1_RX_DMA_RCC,                               \
        .dma_irq  = UART1_RX_DMA_IRQ,                               \
    }
#endif
#endif 


#ifdef __cplusplus
}
#endif

#endif /* __I2C_CONFIG_H__ */
