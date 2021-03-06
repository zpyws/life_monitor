/*
 * File      : drv_i2c.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-06-05     tanek        first implementation.
 */

#ifndef __DRV_I2C__
#define __DRV_I2C__

struct cy8c63_i2c_config
{
	const char *name;

	CySCB_Type *i2c_base;
	const cy_stc_sysint_t* irq_config;
//	void (*isr_handler)(void);
    cy_stc_scb_i2c_context_t *context;
    
	void (*i2c_init)(void);
};

struct cy8c63_i2c
{
	const struct cy8c63_i2c_config *config;
    struct rt_i2c_bus_device i2c;
};

#endif
