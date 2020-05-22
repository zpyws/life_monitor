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

#include "demoDefinitions.h"
#include "algoConfigAPI.h"
#include "simpleDataCapture.h"

//by yangwensen@20200521
struct max32664_normal_algorithm_report
{
    uint8_t ppg1[3];
    uint8_t ppg2[3];
    uint8_t ppg3[3];
    uint8_t ppg4[3];
    uint8_t ppg5[3];
    uint8_t ppg6[3];
    
    uint8_t accx[2];
    uint8_t accy[2];
    uint8_t accz[2];
    
    uint8_t op_mode;
    uint8_t hr[2];
    uint8_t hr_confidence;
    uint8_t rr[2];
    uint8_t rr_confidence;
    uint8_t activity_class;
    uint8_t r[2];
    uint8_t spo2_confidence;
    uint8_t spo2[2];
    uint8_t spo2_complete;
    uint8_t spo2_low_signal_quality_flag;
    uint8_t spo2_motion_flag;
    uint8_t spo2_low_pi_flag;
    uint8_t spo2_unreliable_r_flag;
    uint8_t spo2_state;
    uint8_t scd_state;
};

static uint8_t accelBehavior = SH_INPUT_DATA_DIRECT_SENSOR; //SH_INPUT_DATA_FROM_HOST ;
static rt_bool_t hasActiveMeasurement = RT_FALSE;
static int sHubInputFifoSz = 5;
static uint8_t sh_data_report_mode = SSHUB_ALGO_RAW_DATA_REPORT_MODE;

void max32664_print_raw_data(uint8_t *buff, uint8_t len);
/* IMPORTANT:
 * Accel Data simualtor, accel sensor should be driven at 25Hz.
 * Accel data feeding to Sensor hub is habdled in block writes, ie multiple samples
 * in single transcation so accel samples needs to be queued for later feed to sesnsor hub.
 *
 **/
#if 0       //by yangwensen@20200508
void get_simulated_accel_sensor_sample( accel_mode1_data* queuedAccSamples , int poppedItemCnt ) {

	accel_mode1_data syntheticAccelData;
    int i;
    srand(time(0));
	for( i = 0 ; i < poppedItemCnt ; i++) {

		syntheticAccelData.x = -30  - (rand()%45);
	   	syntheticAccelData.y = -20  - (rand()%50);;
	   	syntheticAccelData.z =  950 + (rand()%55);

	   	*(queuedAccSamples+i) = syntheticAccelData;
	}

}
#endif

#if 0           //by yangwensen@20200508
int FeedAccDataIntoSensHub (void) {

	    static accel_mode1_data peek_buf[MAX_NUM_WR_ACC_SAMPLES];
        static int accelBufPtr = 0;

		accel_mode1_data acc_sample;
		int num_tx, num_samples, num_bytes = 0, num_wr_bytes = 0;
		int num_written_samples, nb_expected;
		int ret = 0;

        // reuqest number of occupied bytes in sensor hub input FIFO
		ret = sh_get_num_bytes_in_input_fifo(&num_bytes);
		if (ret != 0) {
			return -1;
		}
        // calculate free space in sensor hub input FIFO . sHubInputFifoSz is obtained with
		// sh_get_input_fifo_size( &sHubInputFifoSz) call.
		num_tx = sHubInputFifoSz - num_bytes;
		if (num_tx <= 0) {
			return -1;
		}
        // calculate number of accel samples that can fit to free space in sensor hub input fifo
		num_samples = num_tx / sizeof(accel_mode1_data);
		num_samples = MIN_MACRO(num_samples, MAX_NUM_WR_ACC_SAMPLES);

		// calculate byte sting lrngth to be send
		num_tx = num_samples * sizeof(accel_mode1_data);
		if (num_samples == 0) {
			return -1;
		}

		// deque synthetic accel sample
		get_simulated_accel_sensor_sample( &peek_buf[0] , num_samples );

		// prepare byte string representing accel samples to be feed
		uint8_t tx_buf[2 + num_tx]; /* 2 bytes were allocated for commands */
        for (int i = 2, j = 0; j < num_samples; i+= sizeof(accel_mode1_data), j++) {
			acc_sample = peek_buf[j];
			tx_buf[i] = acc_sample.x;
			tx_buf[i + 1] = acc_sample.x >> 8;
			tx_buf[i + 2] = acc_sample.y;
			tx_buf[i + 3] = acc_sample.y >> 8;
			tx_buf[i + 4] = acc_sample.z;
			tx_buf[i + 5] = acc_sample.z >> 8;

		}

        // feed accel samples to sensor hub
		ret = sh_feed_to_input_fifo(tx_buf, num_tx + 2, &num_wr_bytes);
		if(ret != 0) {
			return -1;
		}
		num_written_samples = num_wr_bytes / sizeof(accel_mode1_data);
		if(num_written_samples != num_samples) {
			return -1;
		}

		return 0;
}
#endif

int measure_whrm_wspo2(  uint8_t reportPeriod_in40msSteps ,   uint8_t algoSuiteOperatingMode ){

	 const int sensHubReportFifoThresh      = 1;
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
     const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
			                                  + sizeof(max8614x_mode1_data)
									          + sizeof(whrm_wspo2_suite_mode1_data);

	 static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];

	 int status;

    // enable data type to both raw sensor and algorithm data
     status = sh_set_data_type( SS_DATATYPE_BOTH, RT_FALSE );
     if( status != SS_SUCCESS )
    	 return -3;


     //set fifo threshold for mfio event frequency
     status = sh_set_fifo_thresh(sensHubReportFifoThresh);
     if( status != SS_SUCCESS )
     	 return -4;

	 status  =  sh_set_report_period(reportPeriod_in40msSteps);
     if( status != SS_SUCCESS )
        return -1;

	 if(accelBehavior == SH_INPUT_DATA_FROM_HOST) {

		 status = sh_get_input_fifo_size(&sHubInputFifoSz);
	     if( status != SS_SUCCESS )
            return -2;
	 }


     if(accelBehavior == SH_INPUT_DATA_FROM_HOST) {

    	 status = sh_sensor_enable_(SH_SENSORIDX_ACCEL, 1 , SH_INPUT_DATA_FROM_HOST);
    	 if( status != SS_SUCCESS )
    		 return -5;
     }

     status = sh_set_cfg_wearablesuite_algomode( algoSuiteOperatingMode );
     if( status != SS_SUCCESS )
     	 return -6;

    //added by yangwensen@20200514
    status = sh_set_cfg_wearablesuite_aecenable(RT_TRUE);
     if( status != SS_SUCCESS )
     	 return -7;
    
    //added by yangwensen@20200514
    status = sh_set_cfg_wearablesuite_autopdcurrentenable(RT_TRUE);
     if( status != SS_SUCCESS )
     	 return -8;
    
    //added by yangwensen@20200514
    status = sh_set_cfg_wearablesuite_scdenable(RT_TRUE);
     if( status != SS_SUCCESS )
     	 return -9;
    
     status = sh_enable_algo_(SS_ALGOIDX_WHRM_WSPO2_SUITE , (int) ALGO_REPORT_MODE_BASIC);
     if( status != SS_SUCCESS )
     	 return -10;

     int poolPeriod_ms =  ((int)reportPeriod_in40msSteps) * 40 * 5;
     start_hub_event_poll(poolPeriod_ms);

     hasActiveMeasurement = RT_TRUE;

     while(hasActiveMeasurement){

    	 /* THIS IS FOR SHOWING HOW TO PUSH ACCEL DATA TO SENSOR HUB. IN REALITY ACCEL DATA SHOULD BE COLLECTED IN 25HZ ie 40ms intervals
    	    and be pushed to sensor hub if accel count acceeds 5 samples if there is enaough free space in sesnor hub input FIFO
    	 */
    	 if( accelBehavior == SH_INPUT_DATA_FROM_HOST) {

//    		 FeedAccDataIntoSensHub ();
            LOG_E("error: unsupported mode\n");
    	 }

        rt_thread_mdelay(poolPeriod_ms);        //by yangwensen@20200518

//    	if( sh_has_mfio_event())
        {

    		 sh_clear_mfio_event_flag();

    		 uint8_t hubStatus = 0;
    	     status = sh_get_sensorhub_status(&hubStatus);

    		 if ( status == SS_SUCCESS && (hubStatus & SS_MASK_STATUS_DATA_RDY) == SS_MASK_STATUS_DATA_RDY ) {


    		    	 int num_samples = 1;
    		    	 status = sh_num_avail_samples(&num_samples);
    	    		 if(status == SS_SUCCESS ) {

						 rt_thread_mdelay(5);
						 status = sh_read_fifo_data(num_samples, PPG_REPORT_SIZE + ACCEL_REPORT_SIZE + ALGO_REPORT_SIZE, &databuf[0], sizeof(databuf));
						 if(status == SS_SUCCESS){

							 LOG_D(" data pull >> %d \r\n" , num_samples);

							 whrm_wspo2_suite_mode1_data     algoDataSamp;

							//first byte is status so skip it.
                            struct max32664_normal_algorithm_report *p = (struct max32664_normal_algorithm_report *)(&databuf[1]);
							 //uint8_t *end = &databuf[num_samples*WHRM_FRAME_SIZE];
							 //while( ptr < end )

							 int sampleIdx = 0;
							 while( sampleIdx < num_samples ) {
//                            ulog_hexdump("raw data", 16, ptr, 43);
//                            max32664_print_raw_data(ptr, 44);

								 algoDataSamp.current_operating_mode =  p->op_mode;
								 algoDataSamp.hr                     =  (p->hr[0]<<8) + p->hr[1];
								 algoDataSamp.hr_conf                =  p->hr_confidence;
								 algoDataSamp.rr                     =  (p->rr[0]<<8) + p->rr[1];
								 algoDataSamp.rr_conf      			 =  p->rr_confidence;
								 algoDataSamp.activity_class         =  p->activity_class;
								 algoDataSamp.r                      =  (p->r[0]<<8) + p->r[1];
								 algoDataSamp.spo2_conf              =  p->spo2_confidence;
								 algoDataSamp.spo2                   =  (p->spo2[0]<<8) + p->spo2[1];
								 algoDataSamp.percentComplete 		 =  p->spo2_complete;
								 algoDataSamp.lowSignalQualityFlag   =  p->spo2_low_signal_quality_flag;
								 algoDataSamp.motionFlag 			 =  p->spo2_motion_flag;
								 algoDataSamp.lowPiFlag 			 =  p->spo2_low_pi_flag;
								 algoDataSamp.unreliableRFlag 		 =  p->spo2_unreliable_r_flag;
								 algoDataSamp.spo2State 			 =  p->spo2_state;
								 algoDataSamp.scd_contact_state 	 =  p->scd_state;

								 LOG_D("hr= %d, hr_conf= %d, spo2= %d, spo2_conf= %d\r\n"
										   , algoDataSamp.hr , p->hr_confidence , algoDataSamp.spo2 , p->spo2_confidence );

                                sampleIdx += 1;
                                p++;

							 } //eof loop reading bytyes from hub report fifo

						 } // eof datas pull request form hub

    	          } // eof fifo data count query

    		 }// eof hub status query

    	 } //eof mfio event query

      }//eof main measurement loop
    return 0;               //by yangwensen@20200508
}


int measure_whrm_wspo2_extended_report( void ){

     const int sensHubReportFifoThresh      = 1;
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
     const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
			                                  + sizeof(max8614x_mode1_data)
									          + sizeof(whrm_wspo2_suite_mode2_data);

	 static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];
	 //static int sHubInputFifoSz = 0;

	 int status;

	 status  =  sh_set_report_period(1);
     if( status != SS_SUCCESS )
    	 return -1;

	 if(accelBehavior == SH_INPUT_DATA_FROM_HOST) {

		 status = sh_get_input_fifo_size(&sHubInputFifoSz);
	     if( status != SS_SUCCESS )
	    	 return -1;
	 }

    // enable data type to both raw sensor and algorithm data
     status = sh_set_data_type( SS_DATATYPE_BOTH, RT_FALSE );
     if( status != SS_SUCCESS )
    	 return -1;


     //set fifo threshold for mfio event frequency
     status = sh_set_fifo_thresh(sensHubReportFifoThresh);
     if( status != SS_SUCCESS )
     	 return -1;

     // enable accompanying accel sensor instance within sensorhub
     if(accelBehavior == SH_INPUT_DATA_FROM_HOST) {

    	 status = sh_sensor_enable_(SH_SENSORIDX_ACCEL, 1 , SH_INPUT_DATA_FROM_HOST);
    	 if( status != SS_SUCCESS )
    		 return -1;
     }

     status = sh_enable_algo_(SS_ALGOIDX_WHRM_WSPO2_SUITE , (int) ALGO_REPORT_MODE_EXTENDED);
     if( status != SS_SUCCESS )
     	 return -1;


     start_hub_event_poll(200);

     hasActiveMeasurement = RT_TRUE;

     while(hasActiveMeasurement){

    	 /* THIS IS FOR SHOWING HOW TO PUSH ACCEL DATA TO SENSOR HUB. IN REALITY ACCEL DATA SHOULD BE COLLECTED IN 25HZ ie 40ms intervals
    	    and be pushed to sensor hub if accel count acceeds 5 samples if there is enaough free space in sesnor hub input FIFO
    	 */
    	 if( accelBehavior == SH_INPUT_DATA_FROM_HOST) {

//    		 FeedAccDataIntoSensHub ();
            LOG_E("[Y]error:unsupported mode\n");
    	 }


    	 if( sh_has_mfio_event()) {

    		 sh_clear_mfio_event_flag();

    		 uint8_t hubStatus = 0;
    	     status = sh_get_sensorhub_status(&hubStatus);

    		 if ( status == SS_SUCCESS && (hubStatus & SS_MASK_STATUS_DATA_RDY) == SS_MASK_STATUS_DATA_RDY ) {


    		    	 int num_samples = 1;
    		    	 status = sh_num_avail_samples(&num_samples);
    	    		 if(status == SS_SUCCESS ) {

						 rt_thread_mdelay(5);
						 status = sh_read_fifo_data(num_samples, PPG_REPORT_SIZE + ACCEL_REPORT_SIZE + ALGO_EXTENDED_REPORT_SIZE, &databuf[0], sizeof(databuf));
						 if(status == SS_SUCCESS){

							 //SERIALOUT(" data pull >> %d \r\n" , num_samples);

							 max8614x_mode1_data             ppgDataSample;
							 accel_mode1_data                accelDataSamp;
							 whrm_wspo2_suite_mode2_data     algoDataSamp;

							 uint8_t *ptr = &databuf[1]; //first byte is status so skip it.
							 //uint8_t *end = &databuf[num_samples*WHRM_FRAME_SIZE];
							 //while( ptr < end )

							 int sampleIdx = 0;
							 while( sampleIdx < num_samples ) {

								 ppgDataSample.led1  			     =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
								 ppgDataSample.led2  			     =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
								 ppgDataSample.led3  			     =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
								 ppgDataSample.led4  				 =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
								 ppgDataSample.led5  				 =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
								 ppgDataSample.led6  				 =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);

								 accelDataSamp.x                     =  (*ptr++ << 8)  + (*ptr++ << 0);
								 accelDataSamp.y                     =  (*ptr++ << 8)  + (*ptr++ << 0);
								 accelDataSamp.z                     =  (*ptr++ << 8)  + (*ptr++ << 0);

								 algoDataSamp.current_operating_mode =  (*ptr++);
								 algoDataSamp.hr                     =  (*ptr++ << 8)  + (*ptr++ << 0);
								 algoDataSamp.hr_conf                =  (*ptr++);
								 algoDataSamp.rr                     =  (*ptr++ << 8)  + (*ptr++ << 0);
								 algoDataSamp.rr_conf      			 =  (*ptr++);
								 algoDataSamp.activity_class         =  (*ptr++);

								 algoDataSamp.walk_steps             =  (*ptr++ << 24) + (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
								 algoDataSamp.run_steps              =  (*ptr++ << 24) + (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
								 algoDataSamp.kcal                   =  (*ptr++ << 24) + (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
								 algoDataSamp.cadence                =  (*ptr++ << 24) + (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);

								 algoDataSamp.is_led_cur1_adj        =  (*ptr++);
								 algoDataSamp.adj_led_cur1           =  (*ptr++ << 8)  + (*ptr++ << 0);
								 algoDataSamp.is_led_cur2_adj        =  (*ptr++);
								 algoDataSamp.adj_led_cur2           =  (*ptr++ << 8)  + (*ptr++ << 0);
								 algoDataSamp.is_led_cur3_adj        =  (*ptr++);
								 algoDataSamp.adj_led_cur3           =  (*ptr++ << 8)  + (*ptr++ << 0);

								 algoDataSamp.is_int_time_adj        =  (*ptr++);
								 algoDataSamp.t_int_code             =  (*ptr++);
								 algoDataSamp.is_f_smp_adj           =  (*ptr++);
								 algoDataSamp.adj_f_smp 			 =  (*ptr++);
								 algoDataSamp.smp_ave                =  (*ptr++);
								 algoDataSamp.hrm_afe_state          =  (*ptr++);
								 algoDataSamp.is_high_motion         =  (*ptr++);
								 algoDataSamp.scd_contact_state      =  (*ptr++);

								 algoDataSamp.r                      =  (*ptr++ << 8)  + (*ptr++ << 0);
								 algoDataSamp.spo2_conf              =  (*ptr++);
								 algoDataSamp.spo2                   =  (*ptr++ << 8)  + (*ptr++ << 0);
								 algoDataSamp.percentComplete 		 =  (*ptr++);
								 algoDataSamp.lowSignalQualityFlag   =  (*ptr++);
								 algoDataSamp.motionFlag 			 =  (*ptr++);
								 algoDataSamp.lowPiFlag 			 =  (*ptr++);
								 algoDataSamp.unreliableRFlag 		 =  (*ptr++);
								 algoDataSamp.spo2State 			 =  (*ptr++);


								LOG_D(
									    "%lu,%lu,%lu,%lu,%lu,%lu,%.3f,%.3f,%.3f,%u,%.1f,%d,%.1f,%d,%u,%lu,%lu,%lu,%lu,%d,%.1f,%d,%.1f,%d,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%d,%.1f,%d,%d,%d,%d,%d,%d \r\n",
										ppgDataSample.led1,ppgDataSample.led2,ppgDataSample.led3,ppgDataSample.led4,ppgDataSample.led5,ppgDataSample.led6,
										accelDataSamp.x * 0.001,accelDataSamp.y * 0.001,accelDataSamp.z * 0.001,
										algoDataSamp.current_operating_mode,
										algoDataSamp.hr * 0.1              , algoDataSamp.hr_conf,
										algoDataSamp.rr * 0.1              , algoDataSamp.rr_conf,
										algoDataSamp.activity_class        ,
										algoDataSamp.walk_steps            , algoDataSamp.run_steps,
										algoDataSamp.kcal                  , algoDataSamp.cadence,
										algoDataSamp.is_led_cur1_adj       , algoDataSamp.adj_led_cur1* 0.1,
										algoDataSamp.is_led_cur2_adj       , algoDataSamp.adj_led_cur2* 0.1,
										algoDataSamp.is_led_cur3_adj       , algoDataSamp.adj_led_cur3* 0.1,
			                            algoDataSamp.is_int_time_adj		, algoDataSamp.t_int_code,
										algoDataSamp.is_f_smp_adj			, algoDataSamp.adj_f_smp,
										algoDataSamp.smp_ave               ,
										algoDataSamp.hrm_afe_state			,
										algoDataSamp.is_high_motion		,
										algoDataSamp.scd_contact_state     ,
										algoDataSamp.r	* 0.1				,
										algoDataSamp.spo2_conf             , algoDataSamp.spo2 * 0.1,
										algoDataSamp.percentComplete       ,
										algoDataSamp.lowSignalQualityFlag  , algoDataSamp.motionFlag,
										algoDataSamp.lowPiFlag				, algoDataSamp.unreliableRFlag,
										algoDataSamp.spo2State );

								 sampleIdx += 1;

							 } //eof loop reading bytyes from hub report fifo

						 } // eof datas pull request form hub

    	          } // eof fifo data count query

    		 }// eof hub status query

    	 } //eof mfio event query

      }//eof main measurement loop
    return 0;       //by yangwensen@20200508
}



int get_raw_ppg( void ){

     const int sensHubReportFifoThresh             = 1;
     const int sensHubAfeFundametalsamplePeriod    = 40;
     const int MAX_RAWPPG_SAMPLE_COUNT             = 45;
#ifdef PPG_ACCEL_RAW_MODE
     const int RAWPPG_FRAME_SIZE                   = sizeof(accel_mode1_data)
			                                         + sizeof(max8614x_mode1_data);
#else
     const int RAWPPG_FRAME_SIZE                   = sizeof(max8614x_mode1_data);
#endif

	 static uint8_t databuf[RAWPPG_FRAME_SIZE* MAX_RAWPPG_SAMPLE_COUNT + 1];
	 int status;

	 //enable data type to  raw sensor data
	 status = sh_set_data_type( SS_DATATYPE_RAW, RT_FALSE );
	 if( status != SS_SUCCESS )
		 return -1;

     //set fifo threshold for mfio event frequency
     status = sh_set_fifo_thresh(sensHubReportFifoThresh);
     if( status != SS_SUCCESS )
     	 return -1;

     int samplePeriod = 40;
     if( samplePeriod < sensHubAfeFundametalsamplePeriod)
    	 samplePeriod = sensHubAfeFundametalsamplePeriod;

	 status  =  sh_set_report_period( 1 /*(samplePeriod / sensHubAfeFundametalsamplePeriod) */);
     if( status != SS_SUCCESS )
    	 return -1;

	 status = sh_sensor_enable_(SH_SENSORIDX_MAX8614X, 1 , SH_INPUT_DATA_DIRECT_SENSOR);
	 if( status != SS_SUCCESS )
		 return -1;
#ifdef PPG_ACCEL_RAW_MODE
	 status = sh_sensor_enable_(SH_SENSORIDX_ACCEL, 1 , SH_INPUT_DATA_DIRECT_SENSOR);
	 if( status != SS_SUCCESS )
		 return -1;
#endif
	 // set PPG to 400HZ with 16 sample averaging
	 uint8_t  regAddr = 0x12;
	 uint32_t regVal  = 0x2C;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);
	 if( status != SS_SUCCESS )
		 return -1;

	 /*sh_get_reg( SH_SENSORIDX_MAX8614X, regAddr , &regVal);
	 printf("____DBG PT REG VAL =  %x \r\n" , regVal);*/

	 // Set MAX86141 to integration time: 117us and ADC 1/2 range: 32uA.
	 regAddr = 0x11;
	 regVal  = 0x3F;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // Set MAX86141 LED1 current to half of full scale. Reduce [7F] if signal is saturated.
	 regAddr = 0x23;
	 regVal  = 0x7F;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // Set MAX86141 LED2 current to half of full scale. Reduce [7F] if signal is saturated.
	 regAddr = 0x24;
	 regVal  = 0x7F;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // Set MAX86141 LED3 current to half of full scale. Reduce [7F] if signal is saturated.
	 regAddr = 0x25;
	 regVal  = 0x7F;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // Set MAX86141 LEDs 1, 2 and 3 current full range = 124mA.
	 regAddr = 0x2A;
	 regVal  = 0x3F;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // Set MAX86141 LED sequence:LED1 -> LED2 -> LED3 -> NONE
	 regAddr = 0x20;
	 regVal  = 0x21;

	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);
	 regAddr = 0x21;
	 regVal  = 0x03;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // Set MAX86141 FIFO interrupt threshold to 122.
	 regAddr = 0x09;
	 regVal  = 0x7A;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // Set MAX86141 FIFO interrupt threshold to 122.
	 regAddr = 0x0A;
	 regVal  = 0x0E;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // Set MAX86141 FIFO interrupt threshold to 122.
	 regAddr = 0x02;
	 regVal  = 0x86;
	 status = sh_set_reg( SH_SENSORIDX_MAX8614X, regAddr , regVal, SSMAX8614X_REG_SIZE);

	 // set poll period according to sample rate. ideal period is  (0.5 to 1.0) * (1000 / (sampleRate/ fifo threshold))
	 start_hub_event_poll(200);

	 hasActiveMeasurement = RT_TRUE;

	 while(hasActiveMeasurement){

	   	   if(sh_has_mfio_event()) {

	   		 sh_clear_mfio_event_flag();

	   		 uint8_t hubStatus = 0;
	   	     status = sh_get_sensorhub_status(&hubStatus);

	   		 if (hubStatus & SS_MASK_STATUS_DATA_RDY  && status == SS_SUCCESS) {

	   		    	 int num_samples;
	   		    	 status = sh_num_avail_samples(&num_samples);
	   	    		 if(status == SS_SUCCESS ) {
#ifdef PPG_ACCEL_RAW_MODE
    				 status = sh_read_fifo_data(num_samples,  PPG_REPORT_SIZE + ACCEL_REPORT_SIZE, &databuf[0], sizeof(databuf));
#else
    				 status = sh_read_fifo_data(num_samples,  PPG_REPORT_SIZE, &databuf[0], sizeof(databuf));
#endif
					 if(status == SS_SUCCESS ){

								 max8614x_mode1_data ppgDataSample;
								 accel_mode1_data    accelDataSamp;

								 uint8_t *ptr = &databuf[1]; //first byte is status so skip it.
								 //uint8_t *end = &databuf[num_samples*WHRM_FRAME_SIZE];
								 //while( ptr < end )

								 int sampleIdx = 0;
								 while( sampleIdx < num_samples ) {

									 ppgDataSample.led1  			    =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
									 ppgDataSample.led2  			    =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
									 ppgDataSample.led3  			    =  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
									 ppgDataSample.led4  				=  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
									 ppgDataSample.led5  				=  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
									 ppgDataSample.led6  				=  (*ptr++ << 16) + (*ptr++ << 8) + (*ptr++ << 0);
#ifdef PPG_ACCEL_RAW_MODE
									 accelDataSamp.x                    =  (*ptr++ << 8)  + (*ptr++ << 0);
									 accelDataSamp.y                    =  (*ptr++ << 8)  + (*ptr++ << 0);
									 accelDataSamp.z                    =  (*ptr++ << 8)  + (*ptr++ << 0);
#endif
									 LOG_D(" led1Cnt= %d , led2Cnt= %d , led3Cnt= %d, led4Cnt= %d, led5Cnt= %d, led6Cnt= %d \r\n" , ppgDataSample.led1,
											 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	      	    ppgDataSample.led2,
																																		ppgDataSample.led3,
																																		ppgDataSample.led4,
																																		ppgDataSample.led5,
																																		ppgDataSample.led6  );

									 sampleIdx += 1;

								 } //eof loop reading bytyes from hub report fifo

					 } // eof datas pull request form hub

	   	          } // eof fifo data count query

	   		 }// eof hub status query

	   	 } //eof mfio event query

	  }//eof main measurement loop
    return 0;           //by yangwensen@20200508
}




