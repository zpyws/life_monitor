/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-05     zylx         first version
 * 2018-12-12     greedyhao    Porting for stm32f7xx
 * 2019-02-01     yuneizhilin   fix the stm32_adc_init function initialization issue
 */

#include <board.h>

#if defined(BSP_USING_ADC1) || defined(BSP_USING_ADC2) || defined(BSP_USING_ADC3)
#include "drv_config.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.adc"
#include <drv_log.h>

    
//by yangwensen@20200336
struct cy8c63_adc_config
{
    SAR_Type *base;
    
    void (*init)(void);
    void (*start_convert)(void);
    
};

static struct cy8c63_adc_config adc_config[] =
{
#ifdef BSP_USING_ADC1
    ADC1_CONFIG,
#endif

#ifdef BSP_USING_ADC2
    ADC2_CONFIG,
#endif

#ifdef BSP_USING_ADC3
    ADC3_CONFIG,
#endif
};

struct cy8c63_adc
{
    struct cy8c63_adc_config config;
    struct rt_adc_device cy8c63_adc_device;
};

static struct cy8c63_adc cy8c63_adc_obj[sizeof(adc_config) / sizeof(adc_config[0])];

//by yangwensen@20200326
static rt_err_t cy8c63_adc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled)
{
    struct cy8c63_adc_config *cypress;
    RT_ASSERT(device != RT_NULL);
    cypress = device->parent.user_data;
    
    if (enabled)
    {
        Cy_SAR_Enable(cypress->base);
    }
    else
    {
        Cy_SAR_Disable(cypress->base);
    }

    return RT_EOK;
}

static rt_uint32_t cy8c63_adc_get_channel(rt_uint32_t channel)
{
    return channel;
}

//by yangwensen@20200326
static rt_err_t cy8c63_get_adc_value(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    struct cy8c63_adc_config *cypress;
    uint32_t ch;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(value != RT_NULL);

    cypress = device->parent.user_data;


    if (channel < 16)
    {
        ch =  cy8c63_adc_get_channel(channel);
    }
    else
    {
        LOG_E("ADC channel must be between 0 and 15.");
        return -RT_ERROR;
    }
    
    Cy_SAR_SetChanMask(cypress->base, 1ul<<ch);
    
    /* start ADC */
    cypress->start_convert();
    
    /* Wait for the ADC to convert */
    if( Cy_SAR_IsEndConversion(cypress->base, CY_SAR_WAIT_FOR_RESULT ) != CY_SAR_SUCCESS)
        return -RT_ERROR;
    
    /* get ADC value */
    *value = (rt_uint32_t)Cy_SAR_GetResult32(cypress->base, ch);
    
    return RT_EOK;
}

static const struct rt_adc_ops psoc_adc_ops =
{
    .enabled = cy8c63_adc_enabled,
    .convert = cy8c63_get_adc_value,
};

static int cy8c63_adc_init(void)
{
    int result = RT_EOK;
    /* save adc name */
    char name_buf[5] = {'a', 'd', 'c', '0', 0};
    int i = 0;

    for (i = 0; i < sizeof(adc_config) / sizeof(adc_config[0]); i++)
    {
        /* ADC init */
        name_buf[3] = '0';
        cy8c63_adc_obj[i].config = adc_config[i];
#if defined(BSP_USING_ADC1)
        name_buf[3] = '1';
#endif
#if defined(BSP_USING_ADC2)
        if (cy8c63_adc_obj[i].ADC_Handler.Instance == ADC2)
        {
            name_buf[3] = '2';
        }
#endif
#if defined(BSP_USING_ADC3)
        if (cy8c63_adc_obj[i].ADC_Handler.Instance == ADC3)
        {
            name_buf[3] = '3';
        }
#endif

        cy8c63_adc_obj[i].config.init();
        
        /* register ADC device */
        if (rt_hw_adc_register(&cy8c63_adc_obj[i].cy8c63_adc_device, name_buf, &psoc_adc_ops, &cy8c63_adc_obj[i].config) == RT_EOK)
        {
            LOG_D("%s init success", name_buf);
        }
        else
        {
            LOG_E("%s register failed", name_buf);
            result = -RT_ERROR;
        }
    }

    return result;
}
INIT_BOARD_EXPORT(cy8c63_adc_init);

#endif /* BSP_USING_ADC */
