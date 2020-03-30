/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     zylx         first version
 */

#include "board.h"
#include "drv_qspi.h"
#include "drv_config.h"

#ifdef RT_USING_QSPI

//#define DRV_DEBUG
#define LOG_TAG              "drv.qspi"
#include <drv_log.h>

#if defined(BSP_USING_QSPI)

struct cy8c63_hw_spi_cs
{
    uint16_t Pin;
};

//by yangwensen@20200316
typedef struct
{
    cy_stc_smif_config_t spicfg;
    cy_stc_sysint_t intcfg;
}QSPI_HandleTypeDef;

//by yangwensen@20200317
typedef struct
{
    SMIF_Type *base;
    uint8_t cmd;
    cy_en_smif_txfr_width_t cmdTxfrWidth;
    uint8_t *cmdParam;
    uint32_t paramSize;
    cy_en_smif_txfr_width_t paramTxfrWidth;
    cy_en_smif_slave_select_t  slaveSelect;
    uint32_t cmpltTxfr;
    cy_stc_smif_context_t const *context;
}QSPI_CommandTypeDef;

struct cy8c63_qspi_bus
{
    QSPI_HandleTypeDef QSPI_Handler;
    
    char *bus_name;
#ifdef BSP_QSPI_USING_DMA
    DMA_HandleTypeDef hdma_quadspi;
#endif
};

struct rt_spi_bus _qspi_bus1;
struct cy8c63_qspi_bus _cy8c63_qspi_bus;

//cy_smif_event_cb_t RxCmpltCallback;
#ifndef CY_SMIF_WIDTH_NA
    #define CY_SMIF_WIDTH_NA        0xFFU  /**< The specific width parameter is not applicable for this memory command. */
#endif
/*******************************************************************************
* Function Name: ExtMemInterrupt
****************************************************************************//**
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external 
* memory are processed inside the SMIF ISR.
*  
*******************************************************************************/
void ExtMemInterrupt(void)
{
    Cy_SMIF_Interrupt(QSPI1_HW, &QSPI1_context);
}

static int cy8c63_qspi_init(struct rt_qspi_device *device, struct rt_qspi_configuration *qspi_cfg)
{
    cy_en_sysint_status_t intrStatus;
    cy_en_smif_status_t smifStatus;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(qspi_cfg != RT_NULL);

    struct rt_spi_configuration *cfg = &qspi_cfg->parent;
    struct cy8c63_qspi_bus *qspi_bus = device->parent.bus->parent.user_data;
    rt_memset(&qspi_bus->QSPI_Handler, 0, sizeof(qspi_bus->QSPI_Handler));

    QSPI_HandleTypeDef QSPI_Handler_config = QSPI_BUS_CONFIG;
    qspi_bus->QSPI_Handler = QSPI_Handler_config;

#if 0
    while (cfg->max_hz < HAL_RCC_GetHCLKFreq() / (i + 1))
    {
        i++;
        if (i == 255)
        {
            LOG_E("QSPI init failed, QSPI frequency(%d) is too low.", cfg->max_hz);
            return -RT_ERROR;
        }
    }
#endif

    if (!(cfg->mode & RT_SPI_CPOL))
    {
        /* QSPI MODE0 */
    }
    else
    {
        /* QSPI MODE3 */
    }

    /* flash size */
//    qspi_bus->QSPI_Handler.Init.FlashSize = POSITION_VAL(qspi_cfg->medium_size) - 1;

//    result = HAL_QSPI_Init(&qspi_bus->QSPI_Handler);
    
    /* Initializes the referenced interrupt. */
    intrStatus = Cy_SysInt_Init(&QSPI1_SMIF_IRQ_cfg, ExtMemInterrupt);
    if(intrStatus)
        LOG_E("SMIF interrupt initialization failed (%d)!\n", intrStatus);
    else
        LOG_D("SMIF interrupt is initialized\n");
        
    /* SMIF initialization */
    smifStatus = Cy_SMIF_Init(QSPI1_HW, &QSPI1_config, TIMEOUT_1_MS, &QSPI1_context);  /* SMIF initialization */ 
    if(smifStatus)
        LOG_E("SMIF initialization failed (%d)!\n", intrStatus);
    else
        LOG_D("SMIF is initialized\n");

    Cy_SMIF_SetDataSelect(QSPI1_HW, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_DATA_SEL0);/* SMIF data select for FRAM slave */

    Cy_SMIF_Enable(QSPI1_HW, &QSPI1_context);  /* Enables the operation of the SMIF block. */

    NVIC_EnableIRQ(QSPI1_SMIF_IRQ_cfg.intrSrc);

    LOG_I("SMIF initialization finished\r\n");    

    LOG_D("qspi init success!");

#ifdef BSP_QSPI_USING_DMA
    /* QSPI interrupts must be enabled when using the HAL_QSPI_Receive_DMA */
    HAL_NVIC_SetPriority(QSPI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(QSPI_IRQn);
    HAL_NVIC_SetPriority(QSPI_DMA_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(QSPI_DMA_IRQ);

    /* init QSPI DMA */
    if(QSPI_DMA_RCC  == RCC_AHB1ENR_DMA1EN)
    {
        __HAL_RCC_DMA1_CLK_ENABLE();
    }
    else
    {
        __HAL_RCC_DMA2_CLK_ENABLE();
    }
    
    HAL_DMA_DeInit(qspi_bus->QSPI_Handler.hdma);
    DMA_HandleTypeDef hdma_quadspi_config = QSPI_DMA_CONFIG;
    qspi_bus->hdma_quadspi = hdma_quadspi_config;

    if (HAL_DMA_Init(&qspi_bus->hdma_quadspi) != HAL_OK)
    {
        LOG_E("qspi dma init failed (%d)!", result);
    }

    __HAL_LINKDMA(&qspi_bus->QSPI_Handler, hdma, qspi_bus->hdma_quadspi);
#endif /* BSP_QSPI_USING_DMA */

    return RT_EOK;
}

static void qspi_send_cmd(struct cy8c63_qspi_bus *qspi_bus, struct rt_qspi_message *message)
{
    RT_ASSERT(qspi_bus != RT_NULL);
    RT_ASSERT(message != RT_NULL);

    QSPI_CommandTypeDef Cmdhandler;
    cy_en_smif_txfr_width_t cmd_width = CY_SMIF_WIDTH_SINGLE;
    uint8_t addr[3];

    /* set QSPI cmd struct */
    Cmdhandler.base = QSPI1_HW;
    Cmdhandler.cmd = message->instruction.content;
    Cmdhandler.slaveSelect = CY_SMIF_SLAVE_SELECT_0;
    Cmdhandler.context = &QSPI1_context;
    
    addr[0] = (uint8_t)(message->address.content >> 16);
    addr[1] = (uint8_t)(message->address.content >> 8);
    addr[2] = (uint8_t)(message->address.content >> 0);
    Cmdhandler.cmdParam = addr;
    
    if(message->parent.length)
        Cmdhandler.cmpltTxfr = TX_NOT_LAST_BYTE;
    else
        Cmdhandler.cmpltTxfr = TX_LAST_BYTE;

    if (message->instruction.qspi_lines == 0)
    {
        cmd_width = (cy_en_smif_txfr_width_t)CY_SMIF_WIDTH_NA;
    }
    else if (message->instruction.qspi_lines == 1)
    {
        cmd_width = CY_SMIF_WIDTH_SINGLE;
    }
    else if (message->instruction.qspi_lines == 2)
    {
        cmd_width = CY_SMIF_WIDTH_DUAL;    
    }
    else if (message->instruction.qspi_lines == 4)
    {
        cmd_width = CY_SMIF_WIDTH_QUAD;  
    }
    Cmdhandler.cmdTxfrWidth = cmd_width;
    
    if (message->address.qspi_lines == 0)
    {
        Cmdhandler.paramTxfrWidth = (cy_en_smif_txfr_width_t)CY_SMIF_CMD_WITHOUT_PARAM;
        Cmdhandler.cmdParam = CY_SMIF_CMD_WITHOUT_PARAM;
    }
    else if (message->address.qspi_lines == 1)
    {
        Cmdhandler.paramTxfrWidth = CY_SMIF_WIDTH_SINGLE;
    }
    else if (message->address.qspi_lines == 2)
    {
        Cmdhandler.paramTxfrWidth = CY_SMIF_WIDTH_DUAL;
    }
    else if (message->address.qspi_lines == 4)
    {
        Cmdhandler.paramTxfrWidth = CY_SMIF_WIDTH_QUAD;
    }
    
    if (message->address.size == 0)
    {
        Cmdhandler.paramSize = CY_SMIF_CMD_WITHOUT_PARAM;
    }
    else if (message->address.size == 24)
    {
        Cmdhandler.paramSize = 3;
    }
    else
    {
        Cmdhandler.paramSize = 4;
    }
    
    if (message->qspi_data_lines == 0)
    {
    }
    else if (message->qspi_data_lines == 1)
    {
    }
    else if (message->qspi_data_lines == 2)
    {
    }
    else if (message->qspi_data_lines == 4)
    {
    }

     /* Transmit command */
    Cy_SMIF_TransmitCommand(Cmdhandler.base,
                            Cmdhandler.cmd,				
                            Cmdhandler.cmdTxfrWidth, 
                            Cmdhandler.cmdParam,
                            Cmdhandler.paramSize,
                            Cmdhandler.paramTxfrWidth,
                            Cmdhandler.slaveSelect, 
                            Cmdhandler.cmpltTxfr, 
                            Cmdhandler.context);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    Cy_SMIF_SendDummyCycles(Cmdhandler.base, (uint32_t)(message->dummy_cycles));        //todo by yangwensen
}

static rt_uint32_t qspixfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    rt_size_t len = 0;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    struct rt_qspi_message *qspi_message = (struct rt_qspi_message *)message;
    struct cy8c63_qspi_bus *qspi_bus = device->bus->parent.user_data;
#ifdef BSP_QSPI_USING_SOFTCS
    struct stm32_hw_spi_cs *cs = device->parent.user_data;
#endif

    const rt_uint8_t *sndb = message->send_buf;
    rt_uint8_t *rcvb = message->recv_buf;
    rt_int32_t length = message->length;
    cy_en_smif_txfr_width_t data_width;
    cy_en_smif_status_t status;

#ifdef BSP_QSPI_USING_SOFTCS
    if (message->cs_take)
    {
        rt_pin_write(cs->pin, 0);
    }
#endif

    if (qspi_message->qspi_data_lines == 0)
    {
       data_width = CY_SMIF_WIDTH_SINGLE;
    }
    else if (qspi_message->qspi_data_lines == 1)
    {
       data_width = CY_SMIF_WIDTH_SINGLE;
    }
    else if (qspi_message->qspi_data_lines == 2)
    {
       data_width = CY_SMIF_WIDTH_DUAL;    
    }
    else if (qspi_message->qspi_data_lines == 4)
    {
       data_width = CY_SMIF_WIDTH_QUAD;   
    }

    /* send data */
    if (sndb)
    {
        qspi_send_cmd(qspi_bus, qspi_message);
        if (qspi_message->parent.length != 0)
        {
            status =  Cy_SMIF_TransmitDataBlocking(QSPI1_HW, 
                            (rt_uint8_t *)sndb,
                            length, 
                            data_width, 
                            &QSPI1_context);
            if(status == CY_SMIF_SUCCESS)
            {
                len = length;
            }
            else
            {
                LOG_E("QSPI send data failed(%d)!", status);
                goto __exit;
            }
        }
        else
        {
            len = 1;
        }
    }
    else if (rcvb)/* recv data */
    {
        qspi_send_cmd(qspi_bus, qspi_message);
        
#ifdef BSP_QSPI_USING_DMA
        if (HAL_QSPI_Receive_DMA(&qspi_bus->QSPI_Handler, rcvb) == HAL_OK)
#else
    #if 0
        status =  Cy_SMIF_ReceiveData(QSPI1_HW,
                            rcvb, 					
                            length, 
                            data_width, 
                            RxCmpltCallback,
                            &smifContext);
    #endif
        status = Cy_SMIF_ReceiveDataBlocking(QSPI1_HW, rcvb, length, data_width, &QSPI1_context);
        
        if(status ==CY_SMIF_SUCCESS )
#endif
        {
            len = length;
#ifdef BSP_QSPI_USING_DMA
            while (qspi_bus->QSPI_Handler.RxXferCount != 0);
#endif
        }
        else
        {
            LOG_E("QSPI recv data failed(%d)!", status);
            goto __exit;
        }
    }

__exit:
#ifdef BSP_QSPI_USING_SOFTCS
    if (message->cs_release)
    {
        rt_pin_write(cs->pin, 1);
    }
#endif
    return len;
}

static rt_err_t qspi_configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct rt_qspi_device *qspi_device = (struct rt_qspi_device *)device;
    return cy8c63_qspi_init(qspi_device, &qspi_device->config);
}

static const struct rt_spi_ops cy8c63_qspi_ops =
{
    .configure = qspi_configure,
    .xfer = qspixfer,
};

static int cy8c63_qspi_register_bus(struct cy8c63_qspi_bus *qspi_bus, const char *name)
{
    RT_ASSERT(qspi_bus != RT_NULL);
    RT_ASSERT(name != RT_NULL);

    _qspi_bus1.parent.user_data = qspi_bus;
    return rt_qspi_bus_register(&_qspi_bus1, name, &cy8c63_qspi_ops);
}

/**
  * @brief  This function attach device to QSPI bus.
  * @param  device_name      QSPI device name
  * @param  pin              QSPI cs pin number
  * @param  data_line_width  QSPI data lines width, such as 1, 2, 4
  * @param  enter_qspi_mode  Callback function that lets FLASH enter QSPI mode
  * @param  exit_qspi_mode   Callback function that lets FLASH exit QSPI mode
  * @retval 0 : success
  *        -1 : failed
  */
rt_err_t cy8c63_qspi_bus_attach_device(const char *bus_name, const char *device_name, rt_uint32_t pin, rt_uint8_t data_line_width, void (*enter_qspi_mode)(), void (*exit_qspi_mode)())
{
    struct rt_qspi_device *qspi_device = RT_NULL;
    struct cy8c63_hw_spi_cs *cs_pin = RT_NULL;
    rt_err_t result = RT_EOK;

    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);
    RT_ASSERT(data_line_width == 1 || data_line_width == 2 || data_line_width == 4);

    qspi_device = (struct rt_qspi_device *)rt_malloc(sizeof(struct rt_qspi_device));
    if (qspi_device == RT_NULL)
    {
        LOG_E("no memory, qspi bus attach device failed!");
        result = RT_ENOMEM;
        goto __exit;
    }
    cs_pin = (struct cy8c63_hw_spi_cs *)rt_malloc(sizeof(struct cy8c63_hw_spi_cs));
    if (qspi_device == RT_NULL)
    {
        LOG_E("no memory, qspi bus attach device failed!");
        result = RT_ENOMEM;
        goto __exit;
    }

    qspi_device->enter_qspi_mode = enter_qspi_mode;
    qspi_device->exit_qspi_mode = exit_qspi_mode;
    qspi_device->config.qspi_dl_width = data_line_width;

    cs_pin->Pin = pin;
#ifdef BSP_QSPI_USING_SOFTCS
    rt_pin_mode(pin, PIN_MODE_OUTPUT);
    rt_pin_write(pin, 1);
#endif

    result = rt_spi_bus_attach_device(&qspi_device->parent, device_name, bus_name, (void *)cs_pin);

__exit:
    if (result != RT_EOK)
    {
        if (qspi_device)
        {
            rt_free(qspi_device);
        }

        if (cs_pin)
        {
            rt_free(cs_pin);
        }
    }

    return  result;
}

#ifdef BSP_QSPI_USING_DMA
void QSPI_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_QSPI_IRQHandler(&_stm32_qspi_bus.QSPI_Handler);

    /* leave interrupt */
    rt_interrupt_leave();
}

void QSPI_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&_stm32_qspi_bus.hdma_quadspi);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_QSPI_USING_DMA */

static int rt_hw_qspi_bus_init(void)
{
    return cy8c63_qspi_register_bus(&_cy8c63_qspi_bus, "qspi1");
}
INIT_BOARD_EXPORT(rt_hw_qspi_bus_init);

#endif /* BSP_USING_QSPI */
#endif /* RT_USING_QSPI */
