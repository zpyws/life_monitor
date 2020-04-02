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
#ifndef I2C1_DMA_RX_CONFIG
#define I2C1_DMA_RX_CONFIG                                         \
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

#ifdef __cplusplus
}
#endif

#endif /* __I2C_CONFIG_H__ */
