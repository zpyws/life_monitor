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
#include <rthw.h>
#include <board.h>

#include "demoDefinitions.h"
#include "SHComm.h"


#define SS_I2C_8BIT_SLAVE_ADDR      0xAA
#define SENSORHUB_I2C_ADRESS        SS_I2C_8BIT_SLAVE_ADDR

#define ENABLED   ((int)(1))
#define DISABLED  ((int)(0))

//#define SS_DUMP_REG_SLEEP_MS        (100)
//#define SS_ENABLE_SENSOR_SLEEP_MS   (20)
//#define SS_DEFAULT_CMD_SLEEP_MS     (2)
#define SS_WAIT_BETWEEN_TRIES_MS    (2)
#define SS_CMD_WAIT_PULLTRANS_MS    (5)
#define SS_FEEDFIFO_CMD_SLEEP_MS	(30)


#define SS_DEFAULT_RETRIES       ((int) (4))
#define SS_ZERO_DELAY               0
#define SS_ZERO_BYTES               0



#define BOOTLOADER_MAX_PAGE_SIZE 8192

/* BOOTLOADER HOST */
#define EBL_CMD_TRIGGER_MODE	0
#define EBL_GPIO_TRIGGER_MODE	1



/*
 *   define the "platform specific" hardware interface which SSinterface requires:
 *
 *   1. master i2c port ; ie m_i2cBus->write under read_cmd and write+cmd functions
 *   2. interrupt attachable I/O pin (mfio)
 *   3. I/O pin for reset and serial port
 *   4. wait_ms wait function taking wait time as integer param of milliseconds
 *   5. When Mbed specific I/O removed change file prefixes from .cpp to .c
 *
 *   Note: Definitions below are for MAX32630FTR Pagasus board . Modify for your platform.
 **/

//I2C *m_i2cBus;                /*i2c  bus sensor hub is connected to*/

#if defined(SYSTEM_USES_RST_PIN)
    #define SENSORHUB_NRST_PIN					GET_PIN(9, 2)
//PinName ss_reset(P5_6);            /* platform specific sensor hub reset pin */
//DigitalInOut reset_pin(ss_reset);  /* reset pin mode be I/O */
#endif
#define SENSORHUB_MFIO_PIN					GET_PIN(9, 3)

//PinName ss_mfio(P5_4);             /* platform specific mfio event pin */
//DigitalInOut mfio_pin(ss_mfio);    /* mfio pin mode be I/O */


#define WAIT_MS rt_thread_mdelay

//by yangwensen@20200508
struct max32664_device max32664_dev;
#define SENSORHUB_I2C_NAME                  "i2c0"


/* SENSOR HUB POLL TIMER. CUSTOMER SHALL USE THEIR OWN PLATFORM/OS TIMER */
//Ticker shubEventPollTimer;


/*
 * desc:
 *   platfrom specific function to init sensor comm interface and get data format.
 *
 * */
void sh_init_hubinterface(void){

	sh_init_hwcomm_interface();
    return;
}












/*
 * SSI API funcions
 * NOTE: Generic functions for any platform.
 *       exceptions: below needs needs modification according to platform and HAL drivers
 *       1. Hard reset function
 *       2. Enable/disable mfio event interrput
 *       3. mfio pin interrupt routine
 *
 * **/

/*global buffer for sensor i2c commands+data*/
uint8_t sh_write_buf[512];
static rt_bool_t m_irq_received_ = RT_FALSE;
//static rt_bool_t mfio_int_happened = RT_FALSE;

//static rt_bool_t in_bootldr;



/* Mode to control sesnor hub resets. ie via GPIO based hard reset or Command based soft reset*/
static uint8_t ebl_mode = EBL_GPIO_TRIGGER_MODE;

/* desc  :
 *         Func to init master i2c hardware comm interface with sennor hub
 *                 init mfio interrupt pin and attach irq to pin
 *                 init reset pin
 * params:
 *         N/A
 */

#if defined(SYSTEM_USES_MFIO_PIN)
void sh_irq_handler(void);
#endif

void sh_init_hwcomm_interface(void){
    max32664_dev.bus = (struct rt_i2c_bus_device *)rt_device_find(SENSORHUB_I2C_NAME);
    if (max32664_dev.bus == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", SENSORHUB_I2C_NAME);
    }
    
//	static I2C ssI2C(P3_4, P3_5);     /*set up sensor hub i2c communication at 400 kHz*/
//	ssI2C.frequency(400000);
//	m_i2cBus = &ssI2C;
#if defined(SYSTEM_USES_RST_PIN)
    rt_pin_mode(SENSORHUB_NRST_PIN, PIN_MODE_INPUT_PULLUP);
#endif

    rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SENSORHUB_MFIO_PIN, PIN_LOW);  /*set mfio as output to wake up me11 from sleep  */

    return;
}


#if defined(SYSTEM_USES_RST_PIN)
/*
 * desc:
 *    function to reset sensor hub and put to application mode after reset  interface and get data format.
 *
 * params:
 *
 *    __I wakeupMode : 0x00 : application mode
 *                     0x08 : bootloader mode
 * */
//by yangwensen@20200508
void sh_hard_reset(int wakeupMode)
{
    rt_pin_mode(SENSORHUB_NRST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_OUTPUT);
    
    rt_pin_write(SENSORHUB_NRST_PIN, PIN_LOW);
    rt_thread_mdelay(SS_RESET_TIME);

   if( (wakeupMode & 0xFF) == 0 ) {

        rt_pin_write(SENSORHUB_MFIO_PIN, PIN_HIGH);

        rt_pin_write(SENSORHUB_NRST_PIN, PIN_HIGH);
        rt_thread_mdelay(SS_STARTUP_TO_MAIN_APP_TIME);

   }else {

        rt_pin_write(SENSORHUB_MFIO_PIN, PIN_LOW);
        rt_pin_write(SENSORHUB_NRST_PIN, PIN_HIGH);
		rt_thread_mdelay(SS_STARTUP_TO_BTLDR_TIME);
   }

    rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(SENSORHUB_NRST_PIN, PIN_MODE_INPUT_PULLUP);
}
#endif

//#define COMPILER_INLINED
//by yangwensen@20200508
rt_inline void LPM_pull_mfio_to_low_and_keep(int waitDurationInUs)
{
    rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SENSORHUB_MFIO_PIN, PIN_LOW);
	rt_hw_us_delay(waitDurationInUs);

}
//#define COMPILER_INLINED
//by yangwensen@20200508
rt_inline void LPM_pull_mfio_to_high ( void )
{
    rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SENSORHUB_MFIO_PIN, PIN_HIGH);
}

#if 0
rt_inline void LPM_set_mfio_as_input ( void )
{
    rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_INPUT_PULLUP);
}
#endif


void sensor_hub_poll_event()
{
	m_irq_received_ = RT_TRUE;

}

void start_hub_event_poll( int pollPeriod_ms){

//	shubEventPollTimer.attach(&sensor_hub_poll_event , ((float) pollPeriod_ms) / 1000.0);
}

void stop_hub_event_poll(void){

//	shubEventPollTimer.detach();
}



void sh_clear_mfio_event_flag(void){
	m_irq_received_ = false;
}

rt_bool_t sh_has_mfio_event(void){
	return m_irq_received_;
}


int sh_set_ebl_mode(const uint8_t mode)
{
	int status;
	if (mode == EBL_CMD_TRIGGER_MODE || mode == EBL_GPIO_TRIGGER_MODE) {
		ebl_mode = mode;
		status =  SS_SUCCESS;
	} else
		status = SS_ERR_INPUT_VALUE;

	return status;
}

int sh_get_ebl_mode(void)
{
   return ebl_mode;
}

int sh_reset_to_bootloader(void){

	int status;
	uint8_t hubMode;
#if defined(SYSTEM_USES_RST_PIN)
     if(ebl_mode == EBL_GPIO_TRIGGER_MODE)
    	 sh_hard_reset(0x08);
     if(ebl_mode == EBL_CMD_TRIGGER_MODE)
#endif
     {
    	 status = sh_set_sensorhub_operating_mode(0x08);
     }
     status = sh_get_sensorhub_operating_mode(&hubMode);
     if( status != 0x00 /*SS_SUCCESS*/ || hubMode != 0x08 ){
    	 status = -1;
     }

     return status;

}




int in_bootldr_mode()
{

	uint8_t cmd_bytes[] = { 0x02, 0x00 };
	uint8_t rxbuf[2]    = { 0 };

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
			0, 0,
			&rxbuf[0], sizeof(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	if (status != SS_SUCCESS)
		return -1;

	return (rxbuf[1] & SS_MASK_MODE_BOOTLDR);
}

int exit_from_bootloader(void)
{

	uint8_t cmd_bytes[] = { 0x01, 0x00 };
	uint8_t data[]      = { 0x00 };
    int status = sh_write_cmd_with_data( &cmd_bytes[0], sizeof(cmd_bytes),
										 &data[0], 1 /*sizeof(data)*/,
										 10*SS_DEFAULT_CMD_SLEEP_MS);

//	in_bootldr = (status == SS_SUCCESS) ? true : false;

	return status;
}

int stay_in_bootloader()
{

	uint8_t cmd_bytes[] = { 0x01, 0x00 };
	uint8_t data[]      = { SS_MASK_MODE_BOOTLDR };

	int status = sh_write_cmd_with_data(
			&cmd_bytes[0], sizeof(cmd_bytes),
			&data[0], sizeof(data), SS_DEFAULT_CMD_SLEEP_MS);

//	in_bootldr = (status == SS_SUCCESS) ? true : false;
	return status;
}

#if 0
//by yangwensen@20200508
#if defined(SYSTEM_USES_MFIO_PIN)
static void cfg_mfio(PinDirection dir)
{
	if (dir == PIN_INPUT) {
        rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_INPUT_PULLUP);
	} else {

        rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_OUTPUT);
	}
}
#endif
#endif

//by yangwensen@20200508
#if defined(SYSTEM_USES_RST_PIN)
int sh_debug_reset_to_bootloader(void)
{

	int status = -1;

	if (ebl_mode == EBL_GPIO_TRIGGER_MODE) {

        rt_pin_mode(SENSORHUB_NRST_PIN, PIN_MODE_OUTPUT);
#if defined(SYSTEM_USES_MFIO_PIN)
        rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_OUTPUT);
#endif
        rt_pin_write(SENSORHUB_NRST_PIN, PIN_LOW);
		WAIT_MS(SS_RESET_TIME);

        rt_pin_write(SENSORHUB_MFIO_PIN, PIN_LOW);

        rt_pin_write(SENSORHUB_NRST_PIN, PIN_HIGH);
		WAIT_MS(SS_STARTUP_TO_BTLDR_TIME);

        rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_INPUT_PULLUP);

        rt_pin_mode(SENSORHUB_NRST_PIN, PIN_MODE_INPUT_PULLUP);

		stay_in_bootloader();
		if (in_bootldr_mode() < 0)
			status = SS_ERR_UNKNOWN;
		else
			status = SS_SUCCESS;

	}else{
		stay_in_bootloader();

		status = SS_SUCCESS;

	}

    return status;
}


int sh_reset_to_main_app(void)
{
	int status = -1;


	if (ebl_mode == EBL_GPIO_TRIGGER_MODE) {

        rt_pin_mode(SENSORHUB_NRST_PIN, PIN_MODE_OUTPUT);
#if defined(SYSTEM_USES_MFIO_PIN)
        rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(SENSORHUB_MFIO_PIN, PIN_LOW);
#endif
		WAIT_MS(SS_RESET_TIME - 5);
        rt_pin_write(SENSORHUB_NRST_PIN, PIN_LOW);
		WAIT_MS(SS_RESET_TIME - 5);
#if defined(SYSTEM_USES_MFIO_PIN)
        rt_pin_write(SENSORHUB_MFIO_PIN, PIN_HIGH);
#endif
		WAIT_MS(SS_RESET_TIME - 5);
        rt_pin_write(SENSORHUB_NRST_PIN, PIN_HIGH);
		//WAIT_MS(50);
		//mfio_pin.write(0);
		WAIT_MS(2*SS_STARTUP_TO_MAIN_APP_TIME);

        rt_pin_mode(SENSORHUB_MFIO_PIN, PIN_MODE_INPUT_PULLUP);

        rt_pin_mode(SENSORHUB_NRST_PIN, PIN_MODE_INPUT_PULLUP);

		// Verify we exited bootloader mode
		if (in_bootldr_mode() == 0)
			status = SS_SUCCESS;
		else
			status = SS_ERR_UNKNOWN;
	}else{
		status = exit_from_bootloader();

	}

	return status;

}

#endif



/*
 *
 *   SENSOR HUB COMMUNICATION INTERFACE ( Defined in MAX32664 User Guide ) API FUNCTIONS
 *
 *
 * */

int sh_self_test(int idx, uint8_t *result, int sleep_ms){

	uint8_t cmd_bytes[] = { 0x70, (uint8_t)idx };
    uint8_t rxbuf[2];
    result[0] = 0xFF;

    int status = sh_read_cmd(&cmd_bytes[0],sizeof(cmd_bytes) ,
                             0, 0,
						     &rxbuf[0], sizeof(rxbuf),
						     sleep_ms  );

	if (status != SS_SUCCESS)
		return SS_ERR_TRY_AGAIN;

    result[0] = rxbuf[1];
	return status;
}

const char* sh_get_hub_fw_version(void)
{
    uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];

    static char fw_version[32] = "SENSORHUB";

	int bootldr = sh_checkif_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_FWVERSION;
	} else {

		return &fw_version[0];
	}

    int status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
             	 	 	 	  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS );

    if (status == SS_SUCCESS) {
        rt_snprintf(fw_version, sizeof(fw_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
	}

    return &fw_version[0];
}


const char* sh_get_hub_algo_version(void)
{
    uint8_t cmd_bytes[3];
    uint8_t rxbuf[4];

    static char algo_version[64] = "SENSORHUBALGORITHMS";

	int bootldr = sh_checkif_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
		cmd_bytes[2] = 0;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_ALGOVER;
		cmd_bytes[2] = SS_CMDIDX_AVAILSENSORS;
	} else {

		return &algo_version[0];
	}

    int status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
                              0, 0,
                              &rxbuf[0], sizeof(rxbuf),
						      SS_DEFAULT_CMD_SLEEP_MS   );

    if (status == SS_SUCCESS) {
        rt_snprintf(algo_version, sizeof(algo_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);

    }

    return &algo_version[0];
}

int sh_send_raw(uint8_t *rawdata, int rawdata_sz)
{
	return sh_write_cmd(&rawdata[0], rawdata_sz, 5 * SS_ENABLE_SENSOR_SLEEP_MS);
}

int sh_get_log_len(int *log_len)
{

	uint8_t cmd_bytes[] = { 0x90, 0x01 };
	uint8_t rxbuf[2]    = {0};
    int logLen = 0;

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
								   0, 0,
								   &rxbuf[0], sizeof(rxbuf),
								   SS_DEFAULT_CMD_SLEEP_MS   );

	if (status == SS_SUCCESS) {
		logLen = (rxbuf[1] << 8) | rxbuf[0];
	}
	*log_len = logLen;

	return status;
}

int sh_read_ss_log(int num_bytes, uint8_t *log_buf, int log_buf_sz)
{
	int bytes_to_read = num_bytes + 1; //+1 for status byte
	//mxm_assert_msg((bytes_to_read <= log_buf_sz), "log_buf too small");

	uint8_t cmd_bytes[] = { 0x90, 0x00 };
    int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
						     0, 0,
							 log_buf, bytes_to_read,
							 SS_CMD_WAIT_PULLTRANS_MS  );

	return status;
}

//by yangwensen@20200508
int sh_write_cmd( uint8_t *tx_buf,
		          int tx_len,
				  int sleep_ms)
{
    struct rt_i2c_msg msg;
    rt_size_t ret;
	int retries = SS_DEFAULT_RETRIES;

	LPM_pull_mfio_to_low_and_keep(250);
    
    msg.addr = SS_I2C_8BIT_SLAVE_ADDR >> 1;
    msg.flags = RT_I2C_WR;
    msg.buf = tx_buf;
    msg.len = tx_len;
    ret = rt_i2c_transfer(max32664_dev.bus, &msg, 1);
	LPM_pull_mfio_to_high();
	//LPM_set_mfio_as_input();

	while (ret != 1 && retries-- > 0) {

		rt_thread_mdelay(1);
		LPM_pull_mfio_to_low_and_keep(250);
        ret = rt_i2c_transfer(max32664_dev.bus, &msg, 1);
    	LPM_pull_mfio_to_high();
    	//LPM_set_mfio_as_input();
	}
    if (ret != 1)
       return SS_ERR_UNAVAILABLE;


    rt_thread_mdelay(sleep_ms);

    rt_uint8_t status_byte;
    LPM_pull_mfio_to_low_and_keep(250);
    msg.flags = RT_I2C_RD;
    msg.buf = &status_byte;
    msg.len = 1;
    ret = rt_i2c_transfer(max32664_dev.bus, &msg, 1);
	LPM_pull_mfio_to_high();
	//LPM_set_mfio_as_input();

	rt_bool_t try_again = (status_byte == SS_ERR_TRY_AGAIN);
	while ((ret != 1 || try_again)
			&& retries-- > 0) 
    {
	 	rt_thread_mdelay(sleep_ms);

	    LPM_pull_mfio_to_low_and_keep(250);
        ret = rt_i2c_transfer(max32664_dev.bus, &msg, 1);
    	LPM_pull_mfio_to_high();
    	//LPM_set_mfio_as_input();

    	try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

    if (ret != 0 || try_again)
        return SS_ERR_UNAVAILABLE;

	return (int) (SS_STATUS)status_byte;
}


int sh_write_cmd_with_data(uint8_t *cmd_bytes,
		                   int cmd_bytes_len,
                           uint8_t *data,
						   int data_len,
                           int cmd_delay_ms)
{
    rt_memcpy(sh_write_buf, cmd_bytes, cmd_bytes_len);
    rt_memcpy(sh_write_buf + cmd_bytes_len, data, data_len);
    int status = sh_write_cmd(sh_write_buf,cmd_bytes_len + data_len, cmd_delay_ms);
    return status;
}

//by yangwensen@20200508
int sh_read_cmd( uint8_t *cmd_bytes,
		         int cmd_bytes_len,
	             uint8_t *data,
				 int data_len,
	             uint8_t *rxbuf,
				 int rxbuf_sz,
                 int sleep_ms )
{
    struct rt_i2c_msg msg;
    rt_size_t ret;
    rt_uint8_t txbuff[16];
	int retries = SS_DEFAULT_RETRIES;

    LPM_pull_mfio_to_low_and_keep(250);
    
    rt_memcpy(txbuff, cmd_bytes, cmd_bytes_len);
    rt_memcpy(&txbuff[cmd_bytes_len], data, data_len);    
    msg.addr = SS_I2C_8BIT_SLAVE_ADDR >> 1;
    msg.flags = RT_I2C_WR;
    msg.buf = txbuff;
    msg.len = cmd_bytes_len+data_len;
    ret = rt_i2c_transfer(max32664_dev.bus, &msg, 1);
	LPM_pull_mfio_to_high();
	//LPM_set_mfio_as_input();
#if 0
    if (data_len != 0) {
        LPM_pull_mfio_to_low_and_keep(250);
    	ret |= m_i2cBus->write(SS_I2C_8BIT_SLAVE_ADDR, (char*)data, data_len, false);
    	LPM_pull_mfio_to_high();
    	//LPM_set_mfio_as_input();
    }
#endif
	while (ret != 1 && retries-- > 0) {
		rt_thread_mdelay(1);

		LPM_pull_mfio_to_low_and_keep(250);
        ret = rt_i2c_transfer(max32664_dev.bus, &msg, 1);
    	LPM_pull_mfio_to_high();
    	//LPM_set_mfio_as_input();
#if 0
    	if (data_len != 0) {

    		LPM_pull_mfio_to_low_and_keep(250);
    		ret |= m_i2cBus->write(SS_I2C_8BIT_SLAVE_ADDR, (char*)data, data_len, false);
        	LPM_pull_mfio_to_high();
        	//LPM_set_mfio_as_input();

    	}
#endif
	}
    if (ret != 1)
    	return SS_ERR_UNAVAILABLE;


    rt_thread_mdelay(sleep_ms);

	LPM_pull_mfio_to_low_and_keep(250);
    msg.flags = RT_I2C_RD;
    msg.buf = rxbuf;
    msg.len = rxbuf_sz;
    ret = rt_i2c_transfer(max32664_dev.bus, &msg, 1);
	LPM_pull_mfio_to_high();
	//LPM_set_mfio_as_input();

	rt_bool_t try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	while ((ret != 1 || try_again) && retries-- > 0) {
		rt_thread_mdelay(sleep_ms);

		LPM_pull_mfio_to_low_and_keep(250);
        ret = rt_i2c_transfer(max32664_dev.bus, &msg, 1);
		LPM_pull_mfio_to_high();
		//LPM_set_mfio_as_input();

    	try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	}
    if (ret != 1 || try_again)
        return SS_ERR_UNAVAILABLE;

    return (int) ((SS_STATUS)rxbuf[0]);
}


int sh_get_sensorhub_status(uint8_t *hubStatus){

	uint8_t ByteSeq[] = {0x00,0x00};
	uint8_t rxbuf[2]  = { 0 };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
			                    0, 0,
			                    &rxbuf[0], sizeof(rxbuf),
								SS_DEFAULT_CMD_SLEEP_MS);

	*hubStatus = rxbuf[1];
	return status;
}


int sh_get_sensorhub_operating_mode(uint8_t *hubMode){

	uint8_t ByteSeq[] = {0x02,0x00};
	uint8_t rxbuf[2]  = { 0 };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
			                    0, 0,
			                    &rxbuf[0], sizeof(rxbuf),
								SS_DEFAULT_CMD_SLEEP_MS);

	*hubMode = rxbuf[1];
	return status;
}


int sh_set_sensorhub_operating_mode(uint8_t hubMode){

	uint8_t ByteSeq[] =  {0x01,0x00,hubMode};
	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_DEFAULT_CMD_SLEEP_MS);
    return status;

}


int sh_set_data_type(int data_type_, rt_bool_t sc_en_)
{

	uint8_t cmd_bytes[] = { 0x10, 0x00 };
	uint8_t data_bytes[] = { (uint8_t)((sc_en_ ? SS_MASK_OUTPUTMODE_SC_EN : 0) |
							((data_type_ << SS_SHIFT_OUTPUTMODE_DATATYPE) & SS_MASK_OUTPUTMODE_DATATYPE)) };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								&data_bytes[0], sizeof(data_bytes),
								SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}


int sh_get_data_type(int *data_type_, rt_bool_t *sc_en_){

	uint8_t ByteSeq[] = {0x11,0x00};
	uint8_t rxbuf[2]  = {0};

	int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS);
	if (status == 0x00 /*SS_SUCCESS*/) {
		*data_type_ =
			(rxbuf[1] & SS_MASK_OUTPUTMODE_DATATYPE) >> SS_SHIFT_OUTPUTMODE_DATATYPE;
		*sc_en_ =
			(bool)((rxbuf[1] & SS_MASK_OUTPUTMODE_SC_EN) >> SS_SHIFT_OUTPUTMODE_SC_EN);

	}

	return status;

}


int sh_set_fifo_thresh( int threshold ){

	uint8_t cmd_bytes[]  = { 0x10 , 0x01 };
	uint8_t data_bytes[] = { (uint8_t)threshold };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								&data_bytes[0], sizeof(data_bytes),
								SS_DEFAULT_CMD_SLEEP_MS
	                            );
	return status;

}


int sh_get_fifo_thresh(int *thresh){

	uint8_t ByteSeq[] = {0x11,0x01};
	uint8_t rxbuf[2]  = {0};

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 &rxbuf[0], sizeof(rxbuf),
							 SS_DEFAULT_CMD_SLEEP_MS);

	*thresh = (int) rxbuf[1];

	return status;

}


int sh_ss_comm_check(void){


	uint8_t ByteSeq[] = {0xFF, 0x00};
	uint8_t rxbuf[2];

	int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  SS_DEFAULT_CMD_SLEEP_MS );

	int tries = 4;
	while (status == SS_ERR_TRY_AGAIN && tries--) {
		WAIT_MS(1000);
		status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
									  0, 0,
									  &rxbuf[0], sizeof(rxbuf),
									  SS_DEFAULT_CMD_SLEEP_MS );

	}

	return status;
}


int sh_num_avail_samples(int *numSamples) {

	 uint8_t ByteSeq[] = {0x12,0x00};
	 uint8_t rxbuf[2]  = {0};

	 int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  &rxbuf[0], sizeof(rxbuf),
							  1);

	 *numSamples = (int) rxbuf[1];

	 return status;
}


int sh_read_fifo_data( int numSamples,
		               int sampleSize,
		               uint8_t* databuf,
					   int databufSz) {

	int bytes_to_read = numSamples * sampleSize + 1; //+1 for status byte

	uint8_t ByteSeq[] = {0x12,0x01};

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 databuf, bytes_to_read,
							 10);

	return status;
}


int sh_set_reg(int idx, uint8_t addr, uint32_t val, int regSz){

	uint8_t ByteSeq[] = { 0x40 , ((uint8_t)idx) , addr};
	uint8_t data_bytes[4];

	for (int i = 0; i < regSz; i++) {
		data_bytes[i] = (val >> (8 * (regSz - 1)) & 0xFF);
	}
	int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
							             &data_bytes[0], (uint8_t) regSz,
										 SS_DEFAULT_CMD_SLEEP_MS);

    return status;
}


int sh_get_reg(int idx, uint8_t addr, uint32_t *val){


	uint32_t i32tmp;
	uint8_t ByteSeq[] = { 0x42, ((uint8_t) idx)};
	uint8_t rxbuf[3]  = {0};

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
								0, 0,
							 &rxbuf[0], sizeof(rxbuf),
							 SS_DEFAULT_CMD_SLEEP_MS);


    if(status == 0x00 /* SS_SUCCESS */) {

    	int reg_width = rxbuf[1];
    	uint8_t ByteSeq2[] = { 0x41, ((uint8_t)idx) , addr} ;
    	uint8_t rxbuf2[5]  = {0};

    	status = sh_read_cmd(&ByteSeq2[0], sizeof(ByteSeq2),
    						0, 0,
    						&rxbuf2[0], reg_width + 1,
							SS_DEFAULT_CMD_SLEEP_MS);

    	if (status == 0x00  /* SS_SUCCESS */) {
    		i32tmp = 0;
    		for (int i = 0; i < reg_width; i++) {
    			i32tmp = (i32tmp << 8) | rxbuf2[i + 1];
    		}
            *val = i32tmp;
    	}
     }

    return status;

}



int sh_sensor_enable_( int idx , int mode, uint8_t ext_mode ){

	uint8_t ByteSeq[] = { 0x44, (uint8_t)idx, (uint8_t)mode, ext_mode };

	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), 5 * SS_ENABLE_SENSOR_SLEEP_MS);
    return status;

}


int sh_sensor_disable( int idx ){

	uint8_t ByteSeq[] = {0x44, ((uint8_t) idx), 0x00};

	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_ENABLE_SENSOR_SLEEP_MS);
	return status;

}


int sh_get_input_fifo_size(int *fifo_size)
{

	uint8_t ByteSeq[] = {0x13,0x01};
	uint8_t rxbuf[3]; /* status + fifo size */

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							  0, 0,
							  rxbuf, sizeof(rxbuf), 2*SS_DEFAULT_CMD_SLEEP_MS);

	*fifo_size = rxbuf[1] << 8 | rxbuf[2];
	return status;
}


int sh_feed_to_input_fifo(uint8_t *tx_buf, int tx_buf_sz, int *nb_written)
{
	int status;

	uint8_t rxbuf[3];
	tx_buf[0] = 0x14;
	tx_buf[1] = 0x00;

	status= sh_read_cmd(tx_buf, tx_buf_sz,
			            0, 0,
			            rxbuf, sizeof(rxbuf), SS_FEEDFIFO_CMD_SLEEP_MS);

	*nb_written = rxbuf[1] * 256 + rxbuf[2];

	return status;
}


int sh_get_num_bytes_in_input_fifo(int *fifo_size)
{

    uint8_t ByteSeq[] = {0x13,0x04};
	uint8_t rxbuf[3]; /* status + fifo size */

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
							 0, 0,
							 rxbuf, sizeof(rxbuf),
							 2*SS_DEFAULT_CMD_SLEEP_MS);

	*fifo_size = rxbuf[1] << 8 | rxbuf[2];

	return status;

}


/*
 * ALGARITIM RELATED FUNCTIONS :)
 *
 *
 *
 *
 *
 * */


int sh_enable_algo_(int idx, int mode)
{
    uint8_t cmd_bytes[] = { 0x52, (uint8_t)idx, (uint8_t)mode };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes), 0, 0, 25 * SS_ENABLE_SENSOR_SLEEP_MS);

	return status;
}

int sh_disable_algo(int idx){

	uint8_t ByteSeq[] = { 0x52, ((uint8_t) idx) , 0x00};

	int status = sh_write_cmd( &ByteSeq[0],sizeof(ByteSeq), SS_ENABLE_SENSOR_SLEEP_MS );

    return status;

}


int sh_set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz){

	uint8_t ByteSeq[] = { 0x50 , ((uint8_t) algo_idx) , ((uint8_t) cfg_idx) };

	int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                             cfg, cfg_sz,
										 SS_DEFAULT_CMD_SLEEP_MS);

	return status;

}


int sh_get_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz){

	uint8_t ByteSeq[] = { 0x50 , ((uint8_t) algo_idx) , ((uint8_t) cfg_idx) };

	int status = sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq),
						     0, 0,
							 cfg, cfg_sz,
							 SS_DEFAULT_CMD_SLEEP_MS);
	return status;

}



/*
 * BOOTLOADER RELATED FUNCTIONS
 *
 *
 * */

static const int aes_nonce_sz = 11;
static const int aes_auth_sz  = 16;
static int bl_comm_delay_factor = 1;



int sh_set_bootloader_delayfactor(const int factor ) {

	int status = -1;
	if( factor >= 1  && factor < 51){
	    bl_comm_delay_factor = factor;
	    status = 0x00;
	}

	return status;

}

int sh_get_bootloader_delayfactor(void){

     return bl_comm_delay_factor;
}

int sh_exit_from_bootloader(void)
{
#if defined(SYSTEM_USES_RST_PIN)
	return sh_reset_to_main_app(); //sh_set_sensorhub_operating_mode(0x00);
#else
	return sh_set_sensorhub_operating_mode(0x00);
#endif
}

int sh_put_in_bootloader(void)
{
	return sh_set_sensorhub_operating_mode( 0x08);
}

int sh_checkif_bootldr_mode(void)
{
	uint8_t hubMode;
	int status = sh_get_sensorhub_operating_mode(&hubMode);
	return (status != SS_SUCCESS)? -1:(hubMode & SS_MASK_MODE_BOOTLDR);
}

int sh_get_bootloader_pagesz(int *pagesz){


	uint8_t ByteSeq[]= { 0x81, 0x01 };
    uint8_t rxbuf[3];
    int sz = 0;

    int status = sh_read_cmd( &ByteSeq[0], sizeof(ByteSeq),
                          0, 0,
                          &rxbuf[0], sizeof(rxbuf),
						  SS_DEFAULT_CMD_SLEEP_MS);
    if (status == 0x00) {
           //rxbuf holds page size in big-endian format
            sz = (256*(int)rxbuf[1]) + rxbuf[2];
            if(sz > BOOTLOADER_MAX_PAGE_SIZE ) {
                   sz = -2;
            }
    }

    *pagesz = sz;

    return status;

}

int sh_set_bootloader_numberofpages(const int pageCount){


    //uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETNUMPAGES };
    uint8_t ByteSeq[] = { 0x80, 0x02 };

    uint8_t data_bytes[] = { (uint8_t)((pageCount >> 8) & 0xFF), (uint8_t)(pageCount & 0xFF) };

    int status = sh_write_cmd_with_data(&ByteSeq[0], sizeof(ByteSeq),
								        &data_bytes[0], sizeof(data_bytes),
										bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS );

    return status;

}

int sh_set_bootloader_iv(uint8_t iv_bytes[aes_nonce_sz]){

	 //uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETIV };
	 uint8_t ByteSeq[] = { 0x80, 0x00 };

	 int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                              &iv_bytes[0], aes_nonce_sz /*sizeof(iv_bytes)*/,
										  bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS
										  );

     return status;

}


int sh_set_bootloader_auth(uint8_t auth_bytes[aes_auth_sz]){

	 //uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SETAUTH };
	 uint8_t ByteSeq[] = { 0x80, 0x01 };

	 int status = sh_write_cmd_with_data( &ByteSeq[0], sizeof(ByteSeq),
			                              &auth_bytes[0], aes_auth_sz /*sizeof(auth_bytes)*/,
										  bl_comm_delay_factor * SS_DEFAULT_CMD_SLEEP_MS
										  );

     return status;

}


int sh_set_bootloader_erase(void){

    //uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_ERASE };
	uint8_t ByteSeq[] = { 0x80, 0x03 };

    int status = sh_write_cmd_with_data(&ByteSeq[0], sizeof(ByteSeq),
                                        0, 0,
										bl_comm_delay_factor * SS_BOOTLOADER_ERASE_DELAY);

    return status;

}


int sh_bootloader_flashpage(uint8_t *flashDataPreceedByCmdBytes , const int page_size){

	static const int flash_cmdbytes_len   = 2;
	static const int check_bytes_len      = 16;
	static const int page_write_time_ms   = 200;

    //static const uint8_t ByteSeq[] = { SS_FAM_W_BOOTLOADER, SS_CMDIDX_SENDPAGE };
    int status = -1;

    //if( (*flashDataPreceedByCmdBytes == SS_FAM_W_BOOTLOADER) &&  ( *(flashDataPreceedByCmdBytes+1) == SS_CMDIDX_SENDPAGE ) )
    if( (*flashDataPreceedByCmdBytes == 0x80) &&  ( *(flashDataPreceedByCmdBytes+1) == 0x04 ) )
    {

		/* We do not use sh_write_cmd_with_data function because internal buffers of the function
		   is limited to 512 bytes which does not support if flashing page size is bigger */
		status = sh_write_cmd(flashDataPreceedByCmdBytes, page_size + check_bytes_len + flash_cmdbytes_len, bl_comm_delay_factor * page_write_time_ms);

    }
	return status;

}


int sh_get_ss_fw_version(uint8_t *fwDesciptor  , uint8_t *descSize)
{

	int status = -1;
	uint8_t cmd_bytes[2];
    uint8_t rxbuf[4];

	int bootldr = in_bootldr_mode();

	if (bootldr > 0) {
		cmd_bytes[0] = 0x81 ; //SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = 0x00 ; //SS_CMDIDX_BOOTFWVERSION;
	} else if (bootldr == 0) {
		cmd_bytes[0] = 0xFF; //SS_FAM_R_IDENTITY;
		cmd_bytes[1] = 0x03; //SS_CMDIDX_FWVERSION;
	} else {
		return -1;
	}

    status = sh_read_cmd( &cmd_bytes[0], sizeof(cmd_bytes),
             	 	 	 	 	 	0, 0,
								    &rxbuf[0], sizeof(rxbuf) ,
									SS_DEFAULT_CMD_SLEEP_MS );

    if (status == 0x00 /*SS_SUCCESS*/) {
    	*fwDesciptor       = rxbuf[1];
    	*(fwDesciptor + 1) = rxbuf[2];
    	*(fwDesciptor + 2) = rxbuf[3];
    	*descSize = 3;
    }else{
    	*descSize = 0;
    }

    return status;

}


int sh_set_report_period(uint8_t period)
{

	uint8_t cmd_bytes[]  = { SS_FAM_W_COMMCHAN, SS_CMDIDX_REPORTPERIOD };
	uint8_t data_bytes[] = { (uint8_t)period };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								              &data_bytes[0], sizeof(data_bytes), SS_DEFAULT_CMD_SLEEP_MS );
	return status;
}


int sh_set_sensor_cfg(int sensor_idx, int cfg_idx, uint8_t *cfg, int cfg_sz, int sleep_ms){

	uint8_t cmd_bytes[] = { SS_FAM_W_SENSOR_CONFIG, (uint8_t)sensor_idx, (uint8_t)cfg_idx };
	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								 	 	 	 	  cfg, cfg_sz, sleep_ms   );
	return status;


}

int sh_disable_sensor_list(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, 0xFF, 2, SH_SENSORIDX_ACCEL, 0, SH_INPUT_DATA_DIRECT_SENSOR, SH_SENSORIDX_MAX8614X, 0, SH_INPUT_DATA_DIRECT_SENSOR };

	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes), 0, 0, 5 * SS_ENABLE_SENSOR_SLEEP_MS);

	return status;
}

/*
 *  Sensorhub Authentication Related Functions
 *
 *
 */

int sh_get_dhparams( uint8_t *response, int response_sz ){

	uint8_t cmd_bytes[] = { SS_FAM_R_INITPARAMS , (uint8_t) 0x00 };

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
								0, 0,
								response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}


int sh_set_dhlocalpublic(  uint8_t *response , int response_sz ){

	uint8_t cmd_bytes[] = { SS_FAM_W_DHPUBLICKEY, (uint8_t) 0x00 };
	int status = sh_write_cmd_with_data(&cmd_bytes[0], sizeof(cmd_bytes),
								        response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);
	return status;

}


int sh_get_dhremotepublic( uint8_t *response, int response_sz ){

	uint8_t cmd_bytes[] = { SS_FAM_R_DHPUBLICKEY , (uint8_t) 0x00 };

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
											0, 0,
											response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}


int sh_get_authentication( uint8_t *response, int response_sz )
{

//	const int auth_cfg_sz = 32; // fixed to 32 bytes

	uint8_t cmd_bytes[] = { SS_FAM_R_AUTHSEQUENCE , (uint8_t) 0x00 };

	int status = sh_read_cmd(&cmd_bytes[0], sizeof(cmd_bytes),
								   0, 0,
								   response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}



