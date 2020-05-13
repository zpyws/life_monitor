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
 * 2018-04-19     misonyo      Porting for gd32f30x
 */

#include "board.h"
#include "drv_i2c.h"
#include "drv_config.h"
//#include <rtthread.h>
//#include "gd32f30x.h"

#ifdef RT_USING_I2C

#include <rtdevice.h>

#define DBG_TAG               "drv.I2C"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

#define I2CM_TIMEOUT            100

#define FIRST_FRAME             0x01
#define LAST_FRAME              0x02

//by yangwensen@20200320
enum
{
#ifdef BSP_USING_I2C0
    I2C0_INDEX,
#endif
#ifdef BSP_USING_I2C1
    I2C1_INDEX,
#endif
#ifdef BSP_USING_I2C2
    I2C2_INDEX,
#endif
#ifdef BSP_USING_I2C3
    I2C3_INDEX,
#endif
#ifdef BSP_USING_I2C4
    I2C4_INDEX,
#endif
#ifdef BSP_USING_I2C5
    I2C5_INDEX,
#endif
#ifdef BSP_USING_I2C6
    I2C6_INDEX,
#endif
#ifdef BSP_USING_I2C7
    I2C7_INDEX,
#endif
#ifdef BSP_USING_I2C8
    I2C8_INDEX,
#endif
};

//by yangwensen@20200320
static const struct cy8c63_i2c_config i2c_config[] =
{
#ifdef BSP_USING_I2C0
    I2C0_CONFIG,
#endif
#ifdef BSP_USING_I2C1
    I2C1_CONFIG,
#endif
#ifdef BSP_USING_I2C2
    I2C2_CONFIG,
#endif
#ifdef BSP_USING_I2C3
    I2C3_CONFIG,
#endif
#ifdef BSP_USING_I2C4
    I2C4_CONFIG,
#endif
#ifdef BSP_USING_I2C5
    I2C5_CONFIG,
#endif
#ifdef BSP_USING_I2C6
    I2C6_CONFIG,
#endif
#ifdef BSP_USING_I2C7
    I2C7_CONFIG,
#endif
#ifdef BSP_USING_I2C8
    I2C8_CONFIG,
#endif
};

static struct cy8c63_i2c i2c_obj[sizeof(i2c_config) / sizeof(i2c_config[0])] = {0};

struct cy8c_i2c_bus
{
    struct rt_i2c_bus_device parent;
    struct rt_i2c_msg *msg;
};

//by yangwensen@20200321
static int cy8c63_i2c_read(struct cy8c63_i2c *i2c, uint8_t frame_type, rt_uint16_t addr, rt_uint8_t* buff, rt_uint16_t len)
{
    cy_en_scb_i2c_status_t status;
    rt_uint16_t i;
    int result = 0;
    
    if(len==0)
        return -1;
    
    /* wait until I2C bus is idle */
//    while( Cy_SCB_I2C_IsBusBusy(i2c->config->i2c_base) );

    /* send a start condition to I2C bus */
    if(frame_type & FIRST_FRAME)
        status = Cy_SCB_I2C_MasterSendStart(i2c->config->i2c_base, addr, CY_SCB_I2C_READ_XFER, I2CM_TIMEOUT, i2c->config->context);
    else
        status = Cy_SCB_I2C_MasterSendReStart(i2c->config->i2c_base, addr, CY_SCB_I2C_READ_XFER, I2CM_TIMEOUT, i2c->config->context);
        
    if(status!=CY_SCB_I2C_SUCCESS)
    {
        result = -2;
        LOG_E("[Y]send start|restart condition & slave addr|RD error:%d", status&~(CY_SCB_ID | CY_PDL_STATUS_ERROR | CY_SCB_I2C_ID));
        goto I2C_READ_ERROR;
    }

    len--;
    for(i=0; i<len; i++)
    {
        status = Cy_SCB_I2C_MasterReadByte(i2c->config->i2c_base, CY_SCB_I2C_ACK, buff, I2CM_TIMEOUT, i2c->config->context);
        if(status!=CY_SCB_I2C_SUCCESS)
        {
            result = -3;
            LOG_E("[Y]read byte error:%d", status&~(CY_SCB_ID | CY_PDL_STATUS_ERROR | CY_SCB_I2C_ID));
            goto I2C_READ_ERROR;
        }
        buff++;
    }
    
    status = Cy_SCB_I2C_MasterReadByte(i2c->config->i2c_base, CY_SCB_I2C_NAK, buff, I2CM_TIMEOUT, i2c->config->context);
    if(status!=CY_SCB_I2C_SUCCESS)
    {
        result = -4;
        LOG_E("[Y]read last byte error:%d", status&~(CY_SCB_ID | CY_PDL_STATUS_ERROR | CY_SCB_I2C_ID));
        goto I2C_READ_ERROR;
    }
    
    if(frame_type & LAST_FRAME)  //generate stop condition
        goto I2C_READ_ERROR;
    else                        //no stop condition
        return result;
    
I2C_READ_ERROR:
    status = Cy_SCB_I2C_MasterSendStop(i2c->config->i2c_base, I2CM_TIMEOUT, i2c->config->context);
    if(status!=CY_SCB_I2C_SUCCESS)
    {
        result = -5;
        LOG_E("[Y]generate stop condition error:%d", status&~(CY_SCB_ID | CY_PDL_STATUS_ERROR | CY_SCB_I2C_ID));
    }
    
    return result;
}

//by yangwensen@20200321
static int cy8c63_i2c_write(struct cy8c63_i2c *i2c, uint8_t frame_type, uint16_t addr, uint8_t* buff, uint16_t len)
{
    cy_en_scb_i2c_status_t status;
    rt_uint16_t i;
    int result = 0;
    
    if(len==0)
        return -1;
    
    /* wait until I2C bus is idle */
//    while( Cy_SCB_I2C_IsBusBusy(i2c->config->i2c_base) );

    /* send a start condition to I2C bus */
    if(frame_type & FIRST_FRAME)
        status = Cy_SCB_I2C_MasterSendStart(i2c->config->i2c_base, addr, CY_SCB_I2C_WRITE_XFER, I2CM_TIMEOUT, i2c->config->context);
    else
        status = Cy_SCB_I2C_MasterSendReStart(i2c->config->i2c_base, addr, CY_SCB_I2C_WRITE_XFER, I2CM_TIMEOUT, i2c->config->context);

    if(status!=CY_SCB_I2C_SUCCESS)
    {
        result = -2;
        LOG_E("[Y]send start condition & slave addr|WR error:%d", status&~(CY_SCB_ID | CY_PDL_STATUS_ERROR | CY_SCB_I2C_ID));
        goto I2C_WRITE_ERROR;
    }

    for(i=0; i<len; i++)
    {
        status = Cy_SCB_I2C_MasterWriteByte(i2c->config->i2c_base, *buff, I2CM_TIMEOUT, i2c->config->context);
        if(status!=CY_SCB_I2C_SUCCESS)
        {
            result = -3;
            LOG_E("[Y]write byte error:%d", status&~(CY_SCB_ID | CY_PDL_STATUS_ERROR | CY_SCB_I2C_ID));
            goto I2C_WRITE_ERROR;
        }
        buff++;
    }

    if(frame_type & LAST_FRAME)  //generate stop condition
        goto I2C_WRITE_ERROR;
    else                        //no stop condition
        return result;

I2C_WRITE_ERROR:
    status = Cy_SCB_I2C_MasterSendStop(i2c->config->i2c_base, I2CM_TIMEOUT, i2c->config->context);
    if(status!=CY_SCB_I2C_SUCCESS)
    {
        result = -5;
        LOG_E("[Y]generate stop condition error:%d", status&~(CY_SCB_ID | CY_PDL_STATUS_ERROR | CY_SCB_I2C_ID));
    }
    
    return result;
}

//by yangwensen@20200320
static rt_size_t cy8c63_i2c_mst_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    rt_uint32_t i;
    rt_err_t ret = RT_ERROR;
    int result;
    uint8_t frame_type;
    
    struct cy8c63_i2c *i2c;
    i2c = rt_container_of(bus, struct cy8c63_i2c, i2c);

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];
        
        if(num==1)
            frame_type = FIRST_FRAME|LAST_FRAME;
        else if(i==0)
            frame_type = FIRST_FRAME;
        else if(i==num-1)
            frame_type = LAST_FRAME;
        else
            frame_type = 0;
            
        if (msg->flags & RT_I2C_ADDR_10BIT)
        {
        }
        else
        {
        }
        
        if (msg->flags & RT_I2C_RD)
        {
            result = cy8c63_i2c_read(i2c, frame_type, msg->addr, msg->buf, msg->len);
            if(result)
            {
                LOG_E("i2c bus write failed,i2c bus stop[%d]!", result);
                break;
            }
        }
        else
        {
            result = cy8c63_i2c_write(i2c, frame_type, msg->addr, msg->buf, msg->len);
            if (result)
            {
                LOG_E("i2c bus write failed,i2c bus stop[%d]!", result);
                break;
            }
        }

    #ifdef RT_I2C_DEBUG
        ulog_hexdump("I2C Data", 16, msg->buf, msg->len);
    #endif
    }
    
    ret = i;

    return ret;
}

static const struct rt_i2c_bus_device_ops cy8c63_i2c_ops =
{ 
    cy8c63_i2c_mst_xfer,
    RT_NULL,
    RT_NULL
};

int rt_hw_i2c_init(void)
{
    rt_size_t obj_num = sizeof(i2c_obj) / sizeof(struct cy8c63_i2c);
    rt_err_t result = 0;

    for (rt_size_t i = 0; i < obj_num; i++)
    {
        i2c_obj[i].config = &i2c_config[i];
        i2c_obj[i].config->i2c_init();
        
        i2c_obj[i].i2c.ops    = &cy8c63_i2c_ops;
        
        result = rt_i2c_bus_device_register(&i2c_obj[i].i2c, i2c_obj[i].config->name);

        RT_ASSERT(result == RT_EOK);
    }
    
    return result;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#endif
/* end of i2c driver */
