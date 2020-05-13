/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#include <rtthread.h>

#define LOG_TAG                         "max32664"
#define LOG_LVL                         LOG_LVL_DBG
#include <ulog.h>

#include "SHComm.h"
//#include "cmdInterface.h"
#include "demoDefinitions.h"
#include "algoConfigAPI.h"
#include "simplest/simpleDataCapture.h"
#include "simplest/authentication.h"

/*************** PLATFORM ***************************/

#define POLL_PERIOD_25MS   (1)
#define POLL_PERIOD_1000MS (25)


/***************APP CONFIGURATION******************/

#define MEASURE_CONT_WHRM_CONT_WSPO2
//#define AUTHENTICATE_TO_SENSORHUB
//#define MEASURE_CONT_WHRM_ONESHOT_WSPO2
//#define MEASURE_CONT_HRM
//#define MEASURE_WHRM_WSPO2_EXTENDED_REPORT
//#define GET_RAW_GREEN_IR_RED_PPG
//#define BOOTLOADER_SEQUENCE


/***************APP ******************/

static void max32664_main(void *parameter) 
{
    int status;

#define WAIT_SENSORHUB_STABLE_BOOTUP_MS  ((uint32_t)2000)

	rt_thread_mdelay(WAIT_SENSORHUB_STABLE_BOOTUP_MS);

	sh_init_hwcomm_interface();
    
    LOG_I("SENSORHUB firmware version: %s\n", sh_get_hub_fw_version());
    LOG_I("SENSORHUB algo version: %s\n", sh_get_hub_algo_version());

#if defined(MEASURE_CONT_WHRM_CONT_WSPO2)
    status = measure_whrm_wspo2( (uint8_t) POLL_PERIOD_25MS , MXM_WEARABLE_ALGO_SUITE_CONTINUOUS_HRM_CONTINUOUS_SPO2_MODE);
#elif defined(MEASURE_CONT_WHRM_ONESHOT_WSPO2)
    status = measure_whrm_wspo2( (uint8_t) POLL_PERIOD_25MS , MXM_WEARABLE_ALGO_SUITE_CONTINUOUS_HRM_ONE_SHOT_SPO2_MODE);
#elif defined(MEASURE_CONT_HRM)
    status =  measure_whrm_wspo2( (uint8_t) POLL_PERIOD_25MS , MXM_WEARABLE_ALGO_SUITE_CONTINUOUS_HRM_MODE);
#elif defined(MEASURE_WHRM_WSPO2_EXTENDED_REPORT)
	status = measure_whrm_wspo2_extended_report();
#elif defined(GET_RAW_GREEN_IR_RED_PPG)
    status = get_raw_ppg();
#elif defined(AUTHENTICATE_TO_SENSORHUB)
    status = authenticate_to_sensorhub();		
#elif defined (BOOTLOADER_SEQUENCE)

     while(1) {

		char ch;
		while ( SERIAL_AVAILABLE() ) {
			ch = SERIALIN();
			cmdIntf_build_command(ch);
		}


	}
    
#endif
    (void)status;
    rt_kprintf("measure_whrm_wspo2()=%d\n", status);
    while(1)
    {
        rt_thread_mdelay(1000);
    }
}

//by yangwensen@20200508
extern void max32664_startup(void)
{
    rt_thread_t thread = rt_thread_create("max32664", max32664_main, RT_NULL, 2048, 26, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }    
}
//********************************************************************************************************************************************
