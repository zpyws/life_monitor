/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

extern void gps_startup(void);
extern void bms_startup(void);
extern void lora_startup(void);

int main(void)
{
    int count = 1;

    gps_startup();
    bms_startup();
    lora_startup();
    while (count++)
    {
        rt_thread_mdelay(500);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
