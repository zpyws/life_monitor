/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2018-11-06     balanceTWK        first version
 * 2019-04-23     WillianChan       Fix GPIO serial number disorder
 */

#include <board.h>
#include "drv_gpio.h"

#ifdef RT_USING_PIN

//**************************************************************************
#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

#define CYPRESS_PORTS   15
    
#define get_port(x)     (x>>3)
#define get_pin(x)      (x&7)
#define MCUPIN2RTTPIN(port,pin)     (((port)<<3)+(pin))
#define PIN_MAX_VAL     (CYPRESS_PORTS*8)//(ITEM_NUM(pins))
#define IRQ_MAX_VAL     (CYPRESS_PORTS)//(ITEM_NUM(port_irq_map))
    
static const uint32_t BIT_MASK_TAB[] = {1UL<<0, 1UL<<1, 1UL<<2, 1UL<<3, 1UL<<4, 1UL<<5, 1UL<<7, 1UL<<8, 1UL<<9, 1UL<<10, 1UL<<11, 1UL<<12, 1UL<<13, 1UL<<14, 1UL<<15};
static const GPIO_PRT_Type *CYPRESS_PORT_BASE_TAB[] = 
{
    __CY8C_PORT(0),
    __CY8C_PORT(1),
    __CY8C_PORT(2),
    __CY8C_PORT(3),
    __CY8C_PORT(4),
    __CY8C_PORT(5),
    __CY8C_PORT(6),
    __CY8C_PORT(7),
    __CY8C_PORT(8),
    __CY8C_PORT(9),
    __CY8C_PORT(10),
    __CY8C_PORT(11),
    __CY8C_PORT(12),
    __CY8C_PORT(13),
    __CY8C_PORT(14),
};
//by yangwensen@20200319
static const struct pin_index pins[] = 
{
#if defined(GPIO_PRT0)
    __CY8C_PIN(0 ,  0, 0 ),
    __CY8C_PIN(1 ,  0, 1 ),
    __CY8C_PIN(2 ,  0, 2 ),
    __CY8C_PIN(3 ,  0, 3 ),
    __CY8C_PIN(4 ,  0, 4 ),
    __CY8C_PIN(5 ,  0, 5 ),
    __CY8C_PIN(6 ,  0, 6 ),
    __CY8C_PIN(7 ,  0, 7 ),
#if defined(GPIO_PRT1)
    __CY8C_PIN(8 ,  1, 0 ),
    __CY8C_PIN(9 ,  1, 1 ),
    __CY8C_PIN(10,  1, 2 ),
    __CY8C_PIN(11,  1, 3 ),
    __CY8C_PIN(12,  1, 4 ),
    __CY8C_PIN(13,  1, 5 ),
    __CY8C_PIN(14,  1, 6 ),
    __CY8C_PIN(15,  1, 7 ),
#if defined(GPIO_PRT2)
    __CY8C_PIN(16,  2, 0 ),
    __CY8C_PIN(17,  2, 1 ),
    __CY8C_PIN(18,  2, 2 ),
    __CY8C_PIN(19,  2, 3 ),
    __CY8C_PIN(20,  2, 4 ),
    __CY8C_PIN(21,  2, 5 ),
    __CY8C_PIN(22,  2, 6 ),
    __CY8C_PIN(23,  2, 7 ),
#if defined(GPIO_PRT3)
    __CY8C_PIN(24,  3, 0 ),
    __CY8C_PIN(25,  3, 1 ),
    __CY8C_PIN(26,  3, 2 ),
    __CY8C_PIN(27,  3, 3 ),
    __CY8C_PIN(28,  3, 4 ),
    __CY8C_PIN(29,  3, 5 ),
    __CY8C_PIN(30,  3, 6 ),
    __CY8C_PIN(31,  3, 7 ),
#if defined(GPIO_PRT4)
    __CY8C_PIN(32,  4, 0 ),
    __CY8C_PIN(33,  4, 1 ),
    __CY8C_PIN(34,  4, 2 ),
    __CY8C_PIN(35,  4, 3 ),
    __CY8C_PIN(36,  4, 4 ),
    __CY8C_PIN(37,  4, 5 ),
    __CY8C_PIN(38,  4, 6 ),
    __CY8C_PIN(39,  4, 7 ),
#if defined(GPIO_PRT5)
    __CY8C_PIN(40,  5, 0 ),
    __CY8C_PIN(41,  5, 1 ),
    __CY8C_PIN(42,  5, 2 ),
    __CY8C_PIN(43,  5, 3 ),
    __CY8C_PIN(44,  5, 4 ),
    __CY8C_PIN(45,  5, 5 ),
    __CY8C_PIN(46,  5, 6 ),
    __CY8C_PIN(47,  5, 7 ),
#if defined(GPIO_PRT6)
    __CY8C_PIN(48,  6, 0 ),
    __CY8C_PIN(49,  6, 1 ),
    __CY8C_PIN(50,  6, 2 ),
    __CY8C_PIN(51,  6, 3 ),
    __CY8C_PIN(52,  6, 4 ),
    __CY8C_PIN(53,  6, 5 ),
    __CY8C_PIN(54,  6, 6 ),
    __CY8C_PIN(55,  6, 7 ),
#if defined(GPIO_PRT7)
    __CY8C_PIN(56,  7, 0 ),
    __CY8C_PIN(57,  7, 1 ),
    __CY8C_PIN(58,  7, 2 ),
    __CY8C_PIN(59,  7, 3 ),
    __CY8C_PIN(60,  7, 4 ),
    __CY8C_PIN(61,  7, 5 ),
    __CY8C_PIN(62,  7, 6 ),
    __CY8C_PIN(63,  7, 7 ),
#if defined(GPIO_PRT8)
    __CY8C_PIN(64,  8, 0 ),
    __CY8C_PIN(65,  8, 1 ),
    __CY8C_PIN(66,  8, 2 ),
    __CY8C_PIN(67,  8, 3 ),
    __CY8C_PIN(68,  8, 4 ),
    __CY8C_PIN(69,  8, 5 ),
    __CY8C_PIN(70,  8, 6 ),
    __CY8C_PIN(71,  8, 7 ),
#if defined(GPIO_PRT9)
    __CY8C_PIN(72,  9, 0 ),
    __CY8C_PIN(73,  9, 1 ),
    __CY8C_PIN(74,  9, 2 ),
    __CY8C_PIN(75,  9, 3 ),
    __CY8C_PIN(76,  9, 4 ),
    __CY8C_PIN(77,  9, 5 ),
    __CY8C_PIN(78,  9, 6 ),
    __CY8C_PIN(79,  9, 7 ),
#if defined(GPIO_PRT10)
    __CY8C_PIN(80, 10, 0 ),
    __CY8C_PIN(81, 10, 1 ),
    __CY8C_PIN(82, 10, 2 ),
    __CY8C_PIN(83, 10, 3 ),
    __CY8C_PIN(84, 10, 4 ),
    __CY8C_PIN(85, 10, 5 ),
    __CY8C_PIN(86, 10, 6 ),
    __CY8C_PIN(87, 10, 7 ),
#if defined(GPIO_PRT11)
    __CY8C_PIN(88, 11, 0 ),
    __CY8C_PIN(89, 11, 1 ),
    __CY8C_PIN(90, 11, 2 ),
    __CY8C_PIN(91, 11, 3 ),
    __CY8C_PIN(91, 11, 4 ),
    __CY8C_PIN(93, 11, 5 ),
    __CY8C_PIN(94, 11, 6 ),
    __CY8C_PIN(95, 11, 7 ),
#if defined(GPIO_PRT12)
    __CY8C_PIN(96, 12, 0 ),
    __CY8C_PIN(97, 12, 1 ),
    __CY8C_PIN(98, 12, 2 ),
    __CY8C_PIN(99, 12, 3 ),
    __CY8C_PIN(100, 12, 4 ),
    __CY8C_PIN(101, 12, 5 ),
    __CY8C_PIN(102, 12, 6 ),
    __CY8C_PIN(103, 12, 7 ),
#if defined(GPIO_PRT13)
    __CY8C_PIN(104, 13, 0 ),
    __CY8C_PIN(105, 13, 1 ),
    __CY8C_PIN(106, 13, 2 ),
    __CY8C_PIN(107, 13, 3 ),
    __CY8C_PIN(108, 13, 4 ),
    __CY8C_PIN(109, 13, 5 ),
    __CY8C_PIN(110, 13, 6 ),
    __CY8C_PIN(111, 13, 7 ),
#if defined(GPIO_PRT14)
    __CY8C_PIN(112, 14, 0 ),
    __CY8C_PIN(113, 14, 1 ),
    __CY8C_PIN(114, 14, 2 ),
    __CY8C_PIN(115, 14, 3 ),
    __CY8C_PIN(116, 14, 4 ),
    __CY8C_PIN(117, 14, 5 ),
    __CY8C_PIN(118, 14, 6 ),
    __CY8C_PIN(119, 14, 7 ),
#endif /* defined(GPIOK) */
#endif /* defined(GPIOJ) */
#endif /* defined(GPIOI) */
#endif /* defined(GPIOH) */
#endif /* defined(GPIOG) */
#endif /* defined(GPIOF) */
#endif /* defined(GPIOE) */
#endif /* defined(GPIOD) */
#endif /* defined(GPIOC) */
#endif /* defined(GPIOB) */
#endif /* defined(GPIOA) */
#endif /* defined(GPIO_PRT3) */
#endif /* defined(GPIO_PRT2) */
#endif /* defined(GPIO_PRT1) */
#endif /* defined(GPIO_PRT0) */
};

#define __CYPRESS_PORT_IRQ_HANDLER(n)     {Port##n##_IRQHandler}
#define __CYPRESS_PORT_IRQ_HANDLER__(n)     __CYPRESS_PORT_IRQ_HANDLER(n)

void Port0_IRQHandler(void);
void Port1_IRQHandler(void);
void Port2_IRQHandler(void);
void Port3_IRQHandler(void);
void Port4_IRQHandler(void);
void Port5_IRQHandler(void);
void Port6_IRQHandler(void);
void Port7_IRQHandler(void);
void Port8_IRQHandler(void);
void Port9_IRQHandler(void);
void Port10_IRQHandler(void);
void Port11_IRQHandler(void);
void Port12_IRQHandler(void);
void Port13_IRQHandler(void);
void Port14_IRQHandler(void);
static const cy_israddress CYPRESS_PORT_IRQ_HANDLER_MAP[CYPRESS_PORTS] = 
{
    __CYPRESS_PORT_IRQ_HANDLER__(0),
    __CYPRESS_PORT_IRQ_HANDLER__(1),
    __CYPRESS_PORT_IRQ_HANDLER__(2),
    __CYPRESS_PORT_IRQ_HANDLER__(3),
    __CYPRESS_PORT_IRQ_HANDLER__(4),
    __CYPRESS_PORT_IRQ_HANDLER__(5),
    __CYPRESS_PORT_IRQ_HANDLER__(6),
    __CYPRESS_PORT_IRQ_HANDLER__(7),
    __CYPRESS_PORT_IRQ_HANDLER__(8),
    __CYPRESS_PORT_IRQ_HANDLER__(9),
    __CYPRESS_PORT_IRQ_HANDLER__(10),
    __CYPRESS_PORT_IRQ_HANDLER__(11),
    __CYPRESS_PORT_IRQ_HANDLER__(12),
    __CYPRESS_PORT_IRQ_HANDLER__(13),
    __CYPRESS_PORT_IRQ_HANDLER__(14)
};

#if 0
static const struct port_irq_map port_irq_map[] =
{
    __CY8C_PORT_INT_MAP__(0),
    __CY8C_PORT_INT_MAP__(1),
    __CY8C_PORT_INT_MAP__(2),
    __CY8C_PORT_INT_MAP__(3),
    __CY8C_PORT_INT_MAP__(4),
    __CY8C_PORT_INT_MAP__(5),
    __CY8C_PORT_INT_MAP__(6),
    __CY8C_PORT_INT_MAP__(7),
    __CY8C_PORT_INT_MAP__(8),
    __CY8C_PORT_INT_MAP__(9),
    __CY8C_PORT_INT_MAP__(10),
    __CY8C_PORT_INT_MAP__(11),
    __CY8C_PORT_INT_MAP__(12),
    __CY8C_PORT_INT_MAP__(13),
    __CY8C_PORT_INT_MAP__(14)
};
#endif

static struct rt_pin_irq_hdr pin_irq_hdr_tab[MAX_GPIO_INT_PINS];

//by yangwensen@20200325
static uint8_t port_irq_enable_mask[CYPRESS_PORTS] = {0};

//by yangwensen@20200319
static void cy8c_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    if (pin > PIN_MAX_VAL)
        return;
    
    Cy_GPIO_Write(pins[pin].gpio, pins[pin].pin, (uint32_t)value);
}

//by yangwensen@20200319
static int cy8c_pin_read(rt_device_t dev, rt_base_t pin)
{
    int value = PIN_LOW;

    if (pin > PIN_MAX_VAL)
        return value;

    value = Cy_GPIO_Read(pins[pin].gpio, pins[pin].pin);

    return value;
}

//by yangwensen@20200319
static void cy8c_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    if (pin > PIN_MAX_VAL)
        return;
    
    if (mode == PIN_MODE_OUTPUT)
    {
        /* output setting */
        Cy_GPIO_SetDrivemode(pins[pin].gpio, pins[pin].pin, CY_GPIO_DM_STRONG_IN_OFF);
    }
    else if (mode == PIN_MODE_INPUT)
    {
        //Digital High-Z. Input buffer on
        Cy_GPIO_SetDrivemode(pins[pin].gpio, pins[pin].pin, CY_GPIO_DM_HIGHZ);
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        //Resistive Pull-Up. Input buffer on
        Cy_GPIO_SetDrivemode(pins[pin].gpio, pins[pin].pin, CY_GPIO_DM_PULLUP);
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        //Resistive Pull-Down. Input buffer on
        Cy_GPIO_SetDrivemode(pins[pin].gpio, pins[pin].pin, CY_GPIO_DM_PULLDOWN);
    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        //Open Drain, Drives High.
        Cy_GPIO_SetDrivemode(pins[pin].gpio, pins[pin].pin, CY_GPIO_DM_OD_DRIVESHIGH_IN_OFF);
    }
}

//by yangwensen@20200325
rt_inline struct rt_pin_irq_hdr *get_pin_irq_handler(int32_t rtt_pin)
{
    uint32_t i;
    
    for(i=0; i<MAX_GPIO_INT_PINS; i++)
    {
        if(pin_irq_hdr_tab[i].pin == rtt_pin)
            return &pin_irq_hdr_tab[i];
    }
    
    return RT_NULL;
}

//by yangwensen@20200325
static rt_err_t cy8c63_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                     rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    rt_base_t level;
    struct rt_pin_irq_hdr *pin_irq_cfg;

    if (pin > PIN_MAX_VAL)
        return RT_ENOSYS;

    level = rt_hw_interrupt_disable();

    pin_irq_cfg = get_pin_irq_handler(-1);
    if(pin_irq_cfg==RT_NULL)
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }
    
    pin_irq_cfg->pin = pin;
    pin_irq_cfg->hdr = hdr;
    pin_irq_cfg->mode = mode;
    pin_irq_cfg->args = args;
    
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

//by yangwensen@20200325
static rt_err_t cy8c63_pin_dettach_irq(struct rt_device *device, rt_int32_t pin)
{
    rt_base_t level;
    struct rt_pin_irq_hdr *pin_irq_cfg;

    if (pin > PIN_MAX_VAL)
        return RT_ENOSYS;

    level = rt_hw_interrupt_disable();
    
    pin_irq_cfg = get_pin_irq_handler(pin);
    if(pin_irq_cfg==RT_NULL)
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }
    
    pin_irq_cfg->pin = -1;
    pin_irq_cfg->hdr = RT_NULL;
    pin_irq_cfg->mode = 0;
    pin_irq_cfg->args = RT_NULL;
    
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

//by yangwensen@20200325
static rt_err_t cy8c63_pin_irq_enable(struct rt_device *device, rt_base_t rtt_pin,
                                     rt_uint32_t enabled)
{
    rt_base_t level;
    uint32_t trigger_mode = CY_GPIO_INTR_DISABLE;
    GPIO_PRT_Type *port_base;
    uint8_t pin;
    struct rt_pin_irq_hdr *pin_irq_cfg;

    if (rtt_pin > PIN_MAX_VAL)
        return RT_ENOSYS;

    pin_irq_cfg = get_pin_irq_handler(rtt_pin);
    if(pin_irq_cfg==RT_NULL)
    {
        return RT_EBUSY;
    }
    
    switch (pin_irq_cfg->mode)
    {
    case PIN_IRQ_MODE_RISING:
        trigger_mode = CY_GPIO_INTR_RISING;
        break;
    case PIN_IRQ_MODE_FALLING:
        trigger_mode = CY_GPIO_INTR_FALLING;
        break;
    case PIN_IRQ_MODE_RISING_FALLING:
        trigger_mode = CY_GPIO_INTR_BOTH;
        break;
    }
        
    port_base = pins[rtt_pin].gpio;
    pin = pins[rtt_pin].pin;
    
    if (enabled == PIN_IRQ_ENABLE)
    {
        level = rt_hw_interrupt_disable();

        Cy_GPIO_SetInterruptEdge(port_base, pin, trigger_mode);
        Cy_GPIO_ClearInterrupt(port_base, pin);                     //clears the triggered pin interrupt
        Cy_GPIO_SetInterruptMask(port_base, pin, 1ul);
        
        NVIC_SetPriority((IRQn_Type)get_port(rtt_pin), 7);
        if (SCB->VTOR == (uint32_t)&__ramVectors)
        {
            (void)Cy_SysInt_SetVector((IRQn_Type)(get_port(rtt_pin)), CYPRESS_PORT_IRQ_HANDLER_MAP[get_port(rtt_pin)]);
        }
        NVIC_EnableIRQ((IRQn_Type)get_port(rtt_pin));

        port_irq_enable_mask[get_port(rtt_pin)] |= BIT_MASK_TAB[pin];

        rt_hw_interrupt_enable(level);
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        level = rt_hw_interrupt_disable();
        
        port_irq_enable_mask[get_port(rtt_pin)] &= ~BIT_MASK_TAB[pin];
        
        Cy_GPIO_SetInterruptMask(port_base, pin, 0ul);              //disable pin interrupt
        Cy_GPIO_ClearInterrupt(port_base, pin);                     //clears the triggered pin interrupt
        if( !port_irq_enable_mask[get_port(rtt_pin)] )
            NVIC_DisableIRQ((IRQn_Type)get_port(rtt_pin));          //disable port interrupt
            
        rt_hw_interrupt_enable(level);  
    }
    else
    {
        return -RT_ENOSYS;
    }

    return RT_EOK;
}

const static struct rt_pin_ops _cy8c_pin_ops =
{
    .pin_mode = cy8c_pin_mode,
    .pin_write = cy8c_pin_write,
    .pin_read = cy8c_pin_read,
    .pin_attach_irq = cy8c63_pin_attach_irq,
    .pin_detach_irq = cy8c63_pin_dettach_irq,
    .pin_irq_enable = cy8c63_pin_irq_enable
};

//by yangwensen@20200325
rt_inline void pin_irq_hdr(uint8_t port)
{
    GPIO_PRT_Type *port_base;
    uint8_t i;
    uint8_t port_int_mask;
    struct rt_pin_irq_hdr *pin_irq_handler;
    
    port_base = (GPIO_PRT_Type *)(CYPRESS_PORT_BASE_TAB[port]);
    port_int_mask = port_irq_enable_mask[port];
    
    for(i=0; i<8; i++)
    {
        if(port_int_mask & BIT_MASK_TAB[i])                         //pin interrupt enabled
        {
            if( Cy_GPIO_GetInterruptStatus(port_base, i) )          //interrupt triggered
            {
                Cy_GPIO_ClearInterrupt(port_base, i);               //Clears the triggered pin interrupt
                
                pin_irq_handler = get_pin_irq_handler( MCUPIN2RTTPIN(port,i) );
                if(pin_irq_handler->hdr)
                    pin_irq_handler->hdr(pin_irq_handler->args);    //do handler
            }
        }
    }
}

void Port0_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(0);
    rt_interrupt_leave();
}

void Port1_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(1);
    rt_interrupt_leave();
}

void Port2_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(2);
    rt_interrupt_leave();
}

void Port3_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(3);
    rt_interrupt_leave();
}

void Port4_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(4);
    rt_interrupt_leave();
}

void Port5_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(5);
    rt_interrupt_leave();
}

void Port6_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(6);
    rt_interrupt_leave();
}

void Port7_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(7);
    rt_interrupt_leave();
}

void Port8_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(8);
    rt_interrupt_leave();
}

void Port9_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(9);
    rt_interrupt_leave();
}

void Port10_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(10);
    rt_interrupt_leave();
}

void Port11_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(11);
    rt_interrupt_leave();
}

void Port12_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(12);
    rt_interrupt_leave();
}

void Port13_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(13);
    rt_interrupt_leave();
}

void Port14_IRQHandler(void)
{
    rt_interrupt_enter();
    pin_irq_hdr(14);
    rt_interrupt_leave();
}

int rt_hw_pin_init(void)
{
    uint32_t i;
    
    for(i=0; i<MAX_GPIO_INT_PINS; i++)
    {
        pin_irq_hdr_tab[i].pin = -1;
    }
    
    for(i=0; i<CYPRESS_PORTS; i++)
    {
        port_irq_enable_mask[i] = 0;
    }
    
#if defined(__HAL_RCC_GPIOA_CLK_ENABLE)
    __HAL_RCC_GPIOA_CLK_ENABLE();
#endif
    
#if defined(__HAL_RCC_GPIOB_CLK_ENABLE)
    __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
    
#if defined(__HAL_RCC_GPIOC_CLK_ENABLE)
    __HAL_RCC_GPIOC_CLK_ENABLE();
#endif
    
#if defined(__HAL_RCC_GPIOD_CLK_ENABLE)
    __HAL_RCC_GPIOD_CLK_ENABLE();
#endif

#if defined(__HAL_RCC_GPIOE_CLK_ENABLE)
    __HAL_RCC_GPIOE_CLK_ENABLE();
#endif

#if defined(__HAL_RCC_GPIOF_CLK_ENABLE)
    __HAL_RCC_GPIOF_CLK_ENABLE();
#endif

#if defined(__HAL_RCC_GPIOG_CLK_ENABLE)
    #ifdef SOC_SERIES_STM32L4
        HAL_PWREx_EnableVddIO2();
    #endif
    __HAL_RCC_GPIOG_CLK_ENABLE();
#endif

#if defined(__HAL_RCC_GPIOH_CLK_ENABLE)
    __HAL_RCC_GPIOH_CLK_ENABLE();
#endif

#if defined(__HAL_RCC_GPIOI_CLK_ENABLE)
    __HAL_RCC_GPIOI_CLK_ENABLE();
#endif

#if defined(__HAL_RCC_GPIOJ_CLK_ENABLE)
    __HAL_RCC_GPIOJ_CLK_ENABLE();
#endif

#if defined(__HAL_RCC_GPIOK_CLK_ENABLE)
    __HAL_RCC_GPIOK_CLK_ENABLE();
#endif

    return rt_device_pin_register("pin", &_cy8c_pin_ops, RT_NULL);
}

#endif /* RT_USING_PIN */
