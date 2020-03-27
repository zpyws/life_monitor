/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-01-01     aozima       first implementation.
 * 2012-07-27     aozima       fixed variable uninitialized.
 */
#include <board.h>
#include "drv_spi.h"

#define SPU_CS_PIN    GET_PIN(6, 3)

static int device_ciu98m25_init(void)
{
	rt_err_t result;

    rt_pin_mode(SPU_CS_PIN, PIN_MODE_OUTPUT);

	result = rt_hw_spi_device_attach("spi3", "spu", __CY8C_PORT(6), 3);
	if (result != RT_EOK)
	{
		return result;
	}


	return RT_EOK;
}
INIT_COMPONENT_EXPORT(device_ciu98m25_init);
