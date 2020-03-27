/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2018-11-06     balanceTWK        first version
 */

#ifndef __DRV_GPIO_H__
#define __DRV_GPIO_H__

#include <drv_common.h>
#include <board.h>

#define __CY8C_PORT(port)  GPIO_PRT##port

#define GET_PIN(PORTx,PIN) (rt_base_t)((PORTx<<3) + PIN)

#define __CY8C_PIN(index, gpio, gpio_index)                                 \
    {                                                                       \
        index, GPIO_PRT##gpio, gpio_index                                   \
    }

#define __STM32_PIN_RESERVE                                                 \
    {                                                                       \
        -1, 0, 0                                                            \
    }

#define __CY8C_PORT_INT_MAP__(port)     {port, ioss_interrupts_gpio_##port##_IRQn}
    
/* cy8c GPIO driver */
struct pin_index
{
    int index;
    GPIO_PRT_Type *gpio;
    uint8_t pin;
};

struct port_irq_map
{
    rt_uint8_t portbit;
    IRQn_Type irqno;
};

int rt_hw_pin_init(void);

#endif /* __DRV_GPIO_H__ */

