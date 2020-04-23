//by yangwensen@20200320
#ifndef __I2C_CONFIG_H__
#define __I2C_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(BSP_USING_I2C0)
#ifndef I2C0_CONFIG
#define I2C0_CONFIG                                                 \
    {                                                               \
        .name = "i2c0",                                             \
        .i2c_base = I2C0_HW, 				                        \
        .irq_config = &I2C0_SCB_IRQ_cfg,    				        \
        .context = &I2C0_context,    	            				\
		.i2c_init = I2C0_Start,										\
    }
#endif 
#endif

#if defined(BSP_I2C0_USING_DMA)
#ifndef I2C0_DMA_RX_CONFIG
#define I2C0_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 

#if defined(BSP_USING_I2C1)
#ifndef I2C1_CONFIG
#define I2C1_CONFIG                                                 \
    {                                                               \
        .name = "i2c1",                                             \
        .i2c_base = I2C1_HW, 				                        \
        .irq_config = &I2C1_SCB_IRQ_cfg,    				        \
        .context = &I2C1_context,    	            			    \
		.i2c_init = I2C1_Start,   								    \
    }
#endif 
#endif

#if defined(BSP_I2C1_USING_DMA)
#ifndef I2C1_DMA_RX_CONFIG
#define I2C1_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 


#if defined(BSP_USING_I2C2)
#ifndef I2C2_CONFIG
#define I2C2_CONFIG                                                 \
    {                                                               \
        .name = "i2c2",                                             \
        .i2c_base = I2C2_HW, 				                        \
        .irq_config = &I2C2_SCB_IRQ_cfg,    				        \
        .context = &I2C2_context,    	            				\
		.i2c_init = I2C2_Start,										\
    }
#endif 
#endif

#if defined(BSP_I2C2_USING_DMA)
#ifndef I2C2_DMA_RX_CONFIG
#define I2C2_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 

#if defined(BSP_USING_I2C3)
#ifndef I2C3_CONFIG
#define I2C3_CONFIG                                                 \
    {                                                               \
        .name = "i2c3",                                             \
        .i2c_base = I2C3_HW, 				                        \
        .irq_config = &I2C3_SCB_IRQ_cfg,    				        \
        .context = &I2C3_context,    	            				\
		.i2c_init = I2C3_Start,										\
    }
#endif 
#endif

#if defined(BSP_I2C3_USING_DMA)
#ifndef I2C3_DMA_RX_CONFIG
#define I2C3_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 

#if defined(BSP_USING_I2C4)
#ifndef I2C4_CONFIG
#define I2C4_CONFIG                                                 \
    {                                                               \
        .name = "i2c4",                                             \
        .i2c_base = I2C4_HW, 				                        \
        .irq_config = &I2C4_SCB_IRQ_cfg,    				        \
        .context = &I2C4_context,    	            				\
		.i2c_init = I2C4_Start,										\
    }
#endif 
#endif

#if defined(BSP_I2C4_USING_DMA)
#ifndef I2C4_DMA_RX_CONFIG
#define I2C4_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 

#if defined(BSP_USING_I2C5)
#ifndef I2C5_CONFIG
#define I2C5_CONFIG                                                 \
    {                                                               \
        .name = "i2c5",                                             \
        .i2c_base = I2C5_HW, 				                        \
        .irq_config = &I2C5_SCB_IRQ_cfg,    				        \
        .context = &I2C5_context,    	            				\
		.i2c_init = I2C5_Start,										\
    }
#endif 
#endif

#if defined(BSP_I2C5_USING_DMA)
#ifndef I2C5_DMA_RX_CONFIG
#define I2C5_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 

#if defined(BSP_USING_I2C6)
#ifndef I2C6_CONFIG
#define I2C6_CONFIG                                                 \
    {                                                               \
        .name = "i2c6",                                             \
        .i2c_base = I2C6_HW, 				                        \
        .irq_config = &I2C6_SCB_IRQ_cfg,    				        \
        .context = &I2C6_context,    	            				\
		.i2c_init = I2C6_Start,										\
    }
#endif 
#endif

#if defined(BSP_I2C6_USING_DMA)
#ifndef I2C6_DMA_RX_CONFIG
#define I2C6_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 

#if defined(BSP_USING_I2C7)
#ifndef I2C7_CONFIG
#define I2C7_CONFIG                                                 \
    {                                                               \
        .name = "i2c7",                                             \
        .i2c_base = I2C7_HW, 				                        \
        .irq_config = &I2C7_SCB_IRQ_cfg,    				        \
        .context = &I2C7_context,    	            				\
		.i2c_init = I2C7_Start,										\
    }
#endif 
#endif

#if defined(BSP_I2C7_USING_DMA)
#ifndef I2C7_DMA_RX_CONFIG
#define I2C7_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 

#if defined(BSP_USING_I2C8)
#ifndef I2C8_CONFIG
#define I2C8_CONFIG                                                 \
    {                                                               \
        .name = "i2c8",                                             \
        .i2c_base = I2C8_HW, 				                        \
        .irq_config = &I2C8_SCB_IRQ_cfg,    				        \
        .context = &I2C8_context,    	            				\
		.i2c_init = I2C8_Start,										\
    }
#endif 
#endif

#if defined(BSP_I2C8_USING_DMA)
#ifndef I2C8_DMA_RX_CONFIG
#define I2C8_DMA_RX_CONFIG                                          \
    {                                                               \
    }
#endif
#endif 

#ifdef __cplusplus
}
#endif

#endif /* __I2C_CONFIG_H__ */
