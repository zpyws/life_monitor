/*
 * scdBasedPowSave.cpp
 *
 *  Created on: Sep 24, 2019
 *      Author: Yagmur.Gok
 */

#include "SHComm.h"

#include "demoDefinitions.h"
#include "algoConfigAPI.h"
#include "simpleDataCapture.h"

enum {
	SCDSM_EVENT_NOEVENT                         =  0,
	SCDSM_EVENT_STOP_MESUREMENT                 	,
	SCDSM_EVENT_STOP_SET_MODE_TO_REGUlAR_MEASUREMENT,
	SCDSM_EVENT_SET_MODE_TO_REGUlAR_MEASUREMENT     ,
	SCDSM_EVENT_SET_MODE_TO_ACCEL_WAKEUP            ,
	SCDSM_EVENT_CONFIG_ACCEL_FOR_MEASUREMENT
};




static bool hasActiveMeasurement = false;
static int sHubInputFifoSz = 5;
static uint8_t sh_data_report_mode = SSHUB_ALGO_RAW_DATA_REPORT_MODE;

sshub_meas_init_params_t exampAlgoMesConfig        = {};
sshub_meas_init_params_t exampAccelWakeupMesConfig = {};


typedef int (*funcPtr)(void *);

typedef struct{
	uint8_t eventType;
	funcPtr eventCallback;
	void *eventParams;
}event_table_entry_t;


static int stop_sensors(void *);
static int set_sh_accel_wakeup_mode( void*);
static int set_sh_accel_for_algomeas(void*);
static int init_measurement(void*);
static int stop_and_init_measurement( void*);

static Timer smTimer;


static funcPtr executeContext;

/*
 * SKIN CONTACT DETECTION BASED POWER SAVING
 *
 *
 * */



const event_table_entry_t EVENT_TABLE[] = {

		{SCDSM_EVENT_STOP_MESUREMENT, stop_sensors , NULL},
		{SCDSM_EVENT_STOP_SET_MODE_TO_REGUlAR_MEASUREMENT, stop_and_init_measurement , (void*) &exampAlgoMesConfig} ,
		{SCDSM_EVENT_SET_MODE_TO_REGUlAR_MEASUREMENT,  init_measurement , (void*) &exampAlgoMesConfig} ,
		{SCDSM_EVENT_SET_MODE_TO_ACCEL_WAKEUP, set_sh_accel_wakeup_mode , (void*) &exampAccelWakeupMesConfig} ,
		{SCDSM_EVENT_CONFIG_ACCEL_FOR_MEASUREMENT, set_sh_accel_for_algomeas , (void*) &exampAccelWakeupMesConfig} ,

};

static int set_sh_accel_wakeup_mode( void* config){

	 int status;

	 sshub_meas_init_params_t* iconfig = (sshub_meas_init_params_t*) config;

	 uint8_t val[3] = { 0x01 , 0x05 , 0x08};
     status = sh_set_sensor_cfg(SH_SENSORIDX_ACCEL, SS_CFGIDX_OPERATING_CONFIG, &val[0], 3, SS_DEFAULT_CMD_SLEEP_MS * 100 + 20);
     if( status != SS_SUCCESS )
    	 return -1;

	 status  =  sh_set_report_period( iconfig->reportPeriod_in40msSteps );
     if( status != SS_SUCCESS )
    	 return -1;

    // enable data type to both raw sensor and algorithm data
     status = sh_set_data_type( SS_DATATYPE_RAW, false );
     if( status != SS_SUCCESS )
    	 return -1;


     //set fifo threshold for mfio event frequency
     status = sh_set_fifo_thresh(1);
     if( status != SS_SUCCESS )
     	 return -1;

     status = sh_sensor_enable_(SH_SENSORIDX_ACCEL, 1 , SH_INPUT_DATA_DIRECT_SENSOR);
     if( status != SS_SUCCESS )
    	 return -1;

     start_hub_event_poll( (int) iconfig->poolPeriod_ms);

     hasActiveMeasurement = true;

     sh_data_report_mode = SSHUB_ACCEL_WAKEUP_REPORT_MODE;

     return SS_SUCCESS;
}


static int init_measurement( void* config ){

	 int status;

	 sshub_meas_init_params_t* iconfig = (sshub_meas_init_params_t*) config;

	 status  =  sh_set_report_period( (int) iconfig->reportPeriod_in40msSteps);
     if( status != SS_SUCCESS )
    	 return -1;

	 if(iconfig->accelBehavior == SH_INPUT_DATA_FROM_HOST) {

		 status = sh_get_input_fifo_size(&sHubInputFifoSz);
	     if( status != SS_SUCCESS )
	    	 return -1;
	 }

    // enable data type to both raw sensor and algorithm data
     status = sh_set_data_type( SS_DATATYPE_BOTH, false );
     if( status != SS_SUCCESS )
    	 return -1;


     //set fifo threshold for mfio event frequency
     const int sensHubReportFifoThresh      = 1;
     status = sh_set_fifo_thresh(sensHubReportFifoThresh);
     if( status != SS_SUCCESS )
     	 return -1;


     if(iconfig->accelBehavior == SH_INPUT_DATA_FROM_HOST) {

    	 status = sh_sensor_enable_(SH_SENSORIDX_ACCEL, 1 , SH_INPUT_DATA_FROM_HOST);
    	 if( status != SS_SUCCESS )
    		 return -1;
     }

     status = sh_set_cfg_wearablesuite_algomode( iconfig->algoSuiteOperatingMode );
     if( status != SS_SUCCESS )
     	 return -1;

     status = sh_enable_algo_(SS_ALGOIDX_WHRM_WSPO2_SUITE , (int) ALGO_REPORT_MODE_BASIC);
     if( status != SS_SUCCESS )
     	 return -1;

     //int poolPeriod_ms =  ((int)reportPeriod_in40msSteps) * 40 * 5;
     start_hub_event_poll( (int) iconfig->poolPeriod_ms);

     hasActiveMeasurement = true;

     sh_data_report_mode = SSHUB_ALGO_RAW_DATA_REPORT_MODE;

     return SS_SUCCESS;

}


static int set_sh_accel_for_algomeas(void* config){

	uint8_t val[3] = { 0x00 , 0xFF , 0xFF};
	int status = sh_set_sensor_cfg(SH_SENSORIDX_ACCEL, SS_CFGIDX_OPERATING_CONFIG, &val[0], 3, SS_DEFAULT_CMD_SLEEP_MS * 100 + 20);
	return status;

}

static int stop_and_init_measurement( void* config ){

	int status = stop_sensors(0);
	if(status == 0){
       status = init_measurement( config );
	}
    return status;

}

static int stop_sensors(void* config){
	sh_disable_sensor_list();
    return 0;
}



void acquire_algo_data( const uint8_t  accelBehavior , whrm_wspo2_suite_mode1_data* algoSamples , uint8_t* sampleCount){

	 extern int FeedAccDataIntoSensHub (void);

	 const int sensHubReportFifoThresh      = 1;
	 const int MAX_WHRMWSPO2_SAMPLE_COUNT   = 45;
	 const int WHRMWSPO2_FRAME_SIZE         = sizeof(accel_mode1_data)
				                              + sizeof(max8614x_mode1_data)
										      + sizeof(whrm_wspo2_suite_mode1_data);

     static uint8_t databuf[WHRMWSPO2_FRAME_SIZE * MAX_WHRMWSPO2_SAMPLE_COUNT + 1];

	 int status;
	 int _sampleCount = 0;

	 if( accelBehavior == SH_INPUT_DATA_FROM_HOST ) {

		 FeedAccDataIntoSensHub ();
	 }


	 if( sh_has_mfio_event()) {

		 sh_clear_mfio_event_flag();

		 uint8_t hubStatus = 0;
	     status = sh_get_sensorhub_status(&hubStatus);

		 if ( status == SS_SUCCESS && (hubStatus & SS_MASK_STATUS_DATA_RDY) == SS_MASK_STATUS_DATA_RDY ) {


		    	 int num_samples = 1;
		    	 status = sh_num_avail_samples(&num_samples);
	    		 if(status == SS_SUCCESS ) {

					 wait_ms(5);
					 status = sh_read_fifo_data(num_samples, PPG_REPORT_SIZE + ACCEL_REPORT_SIZE + ALGO_REPORT_SIZE, &databuf[0], sizeof(databuf));
					 if(status == SS_SUCCESS){

						 //SERIALOUT(" data pull >> %d \r\n" , num_samples);

						 max8614x_mode1_data             ppgDataSample;
						 accel_mode1_data                accelDataSamp;
						 whrm_wspo2_suite_mode1_data     algoDataSamp;

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
							 algoDataSamp.r                      =  (*ptr++ << 8)  + (*ptr++ << 0);
							 algoDataSamp.spo2_conf              =  (*ptr++);
							 algoDataSamp.spo2                   =  (*ptr++ << 8)  + (*ptr++ << 0);
							 algoDataSamp.percentComplete 		 =  (*ptr++);
							 algoDataSamp.lowSignalQualityFlag   =  (*ptr++);
							 algoDataSamp.motionFlag 			 =  (*ptr++);
							 algoDataSamp.lowPiFlag 			 =  (*ptr++);
							 algoDataSamp.unreliableRFlag 		 =  (*ptr++);
							 algoDataSamp.spo2State 			 =  (*ptr++);
							 algoDataSamp.scd_contact_state 	 =  (*ptr++);

							 uint32_t green_led_cnt   			 =  ppgDataSample.led1;
							 uint32_t ir_led_cnt   			     =  ppgDataSample.led2;
							 uint32_t red_led_cnt   			 =  ppgDataSample.led3;

							 uint32_t hr                  		 =  algoDataSamp.hr / 10;
							 uint32_t hr_conf                  	 =  algoDataSamp.hr_conf;
							 uint32_t spo2                 	     =  algoDataSamp.spo2 / 10;
							 uint32_t spo2_conf                  =  algoDataSamp.spo2_conf;

							 SERIALOUT(" greenCnt= %d , irCnt= %d , redCnt = %d ,"
									   " hr= %d , hr_conf= %d , spo2= %d , spo2_conf= %d \r\n"
									   , green_led_cnt , ir_led_cnt , red_led_cnt
									   , hr , hr_conf , spo2 , spo2_conf );

							 sampleIdx += 1;

							 *algoSamples++ = algoDataSamp;

						 } //eof loop reading bytyes from hub report fifo

						 _sampleCount =  num_samples;

					 } // eof datas pull request form hub

	          } // eof fifo data count query

		 }// eof hub status query

	 } //eof mfio event query

	 *sampleCount = _sampleCount;

}


void acquire_motion_detection_signal( const uint8_t  accelBehavior , uint8_t* isMotionDetected ) {


	 int status;
	 int num_samples = 0;


	 if( sh_has_mfio_event()) {

		 sh_clear_mfio_event_flag();

		 uint8_t hubStatus = 0;
	     status = sh_get_sensorhub_status(&hubStatus);

		 if ( status == SS_SUCCESS && (hubStatus & SS_MASK_STATUS_DATA_RDY) == SS_MASK_STATUS_DATA_RDY ) {

		    	 status = sh_num_avail_samples(&num_samples);
	    		 if(status == SS_SUCCESS && num_samples >= 1 ) {


	    		 } // eof fifo data count query

		 }// eof hub status query

	 } //eof mfio event query

	 *isMotionDetected = (num_samples > 0)? 1:0;

}


/*
void scdsm_reset_counter(){

	smTimer.stop();
	smTimer.reset();
	smTimer.start();
	beginTime = smTimer.read();
}


__attribute__((always_inline)) uint32_t TIME_PASSED(void)
{
	return (uint32_t) ( smTimer.read() + (~beginTime + 1) );
}
*/

#define SCDSM_RESET_COUNTER()  smTimer.stop();  \
						       smTimer.reset(); \
						       smTimer.start(); \
						       beginTime = smTimer.read(); \

#define SCDSM_TIME_PASSED()   ( smTimer.read() + (~beginTime + 1) )



#if 1
void  scdsm_powersave_run_sm( const uint8_t  isMotionDetected , /*input in mode 7*/ /* if host accel do not have htrshold for motion detection provide an outside function to handle */
		                      const uint8_t  scdState         , /* input in mode 4 -5*/
							  uint8_t    *output_event     	  ,
							  uint8_t    *n_events_set              )
{

	static int  beginTime       = 0;
	static int  currTime        = 0;
	static int  nTrials         = 0;
	uint8_t i_events_set 		= 0;

	typedef enum {
		ACTIVE_STATE      = 0,
		PROBING_LEDON_STATE  ,
		PROBING_LEDOFF_STATE ,
		SKINOFF_STATE
	}state_t;
	static state_t state     = ACTIVE_STATE;

	enum{
		SCDSTATE_UNDETECTED    = 0,
		SCDSTATE_NOSKINCONTACT = 1,
		SCDSTATE_SOMECONTACT   = 2,
		SCDSTATE_SKINCONTACT   = 3
	};

	//printf("scd= %d systate = %d \r\n" , instant_scdstate , state );

	switch(state){

		case ACTIVE_STATE: {

    		if( (scdState != SCDSTATE_SKINCONTACT) &&  SCDSM_TIME_PASSED()  > scdSmConfigStruct.probingLedonPeriod ) {
    			//report event : stop_sensors();
    			i_events_set++;
    			*output_event++ = SCDSM_EVENT_STOP_MESUREMENT ;

    			SCDSM_RESET_COUNTER();

				nTrials = 0;

				state     =  PROBING_LEDOFF_STATE;
				printf(" moving to PROBING LED OFF from ACTIVE \r\n");
			}

		}break;

		case PROBING_LEDON_STATE: {

			if( SCDSM_TIME_PASSED() > scdSmConfigStruct.probingLedonPeriod /*scdSmConfigStruct.probingWaitPeriod*/ ){

				//report event : stop_sensors();
    			i_events_set++;
    			*output_event++ = SCDSM_EVENT_STOP_MESUREMENT ;

				if( nTrials == 3) {

					nTrials = 0;


					if(scdSmConfigStruct.isHubAccelEnabled){

		    			i_events_set++;
		    			*output_event++ = SCDSM_EVENT_SET_MODE_TO_ACCEL_WAKEUP;
					}else{


					}

					state   =  SKINOFF_STATE;
					printf(" moving to SKIN OFF  \r\n");

				}else{

					state   =  PROBING_LEDOFF_STATE;
					printf(" moving to PROBING LED OFF from LEDON nTrials = %d \r\n" , nTrials);
				}

				SCDSM_RESET_COUNTER();
			}

			if ( scdState == SCDSTATE_SKINCONTACT ){
				// as scd is detected means measurement is aready running no need to stop just proceed.

				nTrials = 0;
				SCDSM_RESET_COUNTER();

				printf(" moving to ACTIVE from PROBING LEDON \r\n");
				state = ACTIVE_STATE;
			}


		}break;

		case PROBING_LEDOFF_STATE: {


			if( SCDSM_TIME_PASSED() >  (scdSmConfigStruct.probingWaitPeriod << (nTrials ))  /* (scdSmConfigStruct.probingWaitPeriod + 10) */ ) {

    			i_events_set++;
    			*output_event++ = SCDSM_EVENT_SET_MODE_TO_REGUlAR_MEASUREMENT ;

    			SCDSM_RESET_COUNTER();
				state = PROBING_LEDON_STATE;

				printf(" moving to PROBING LED ON from LEDOFF\r\n");
				nTrials++;

			}

		}break;

		case SKINOFF_STATE: {


			if( isMotionDetected || SCDSM_TIME_PASSED() > scdSmConfigStruct.skinoffWaitPeriod  ) {

    			i_events_set++;
    			*output_event++ = SCDSM_EVENT_STOP_MESUREMENT  ;

				if(scdSmConfigStruct.isHubAccelEnabled){

	    			i_events_set++;
	    			*output_event++ = SCDSM_EVENT_CONFIG_ACCEL_FOR_MEASUREMENT ;

				}else{
                    // configure host accel for wake up

				}

    			i_events_set++;
    			*output_event++ = SCDSM_EVENT_SET_MODE_TO_REGUlAR_MEASUREMENT;

    			SCDSM_RESET_COUNTER();

				nTrials = 0;
				state = PROBING_LEDON_STATE;

				printf(" moving to PROBING LED ON from SKINOFF\r\n");
			}
            /*  modify stop or add new stop to set initial state to ACTIVE after stop !*/

		}break;

	}

}

#endif



void scdsm_usage_demo(void){

	 whrm_wspo2_suite_mode1_data algoSamples[20];
	 uint8_t sampleCount;

	 sshub_meas_init_params_t initialMesConfig ={

			 .reportPeriod_in40msSteps = 1,
			 .algoSuiteOperatingMode   = 0,
			 .accelBehavior            = SH_INPUT_DATA_DIRECT_SENSOR,
             .poolPeriod_ms            = 0.2

	 };

	 uint8_t  isMotionDetected ;
	 uint8_t  scdState;
	 uint8_t  scdsm_output_events[5];
	 uint8_t  scdsm_num_events_set;

	 int status =  init_measurement( (void*) &initialMesConfig );

     while(1) {

        if( sh_data_report_mode = SSHUB_ALGO_RAW_DATA_REPORT_MODE) {
        	acquire_algo_data( initialMesConfig.accelBehavior , algoSamples , &sampleCount);
            printf(" hr = %d, spo2= %d , scd = %d " , algoSamples[0].hr , algoSamples[0].spo2 , algoSamples[0].scd_contact_state );

        }
        else if(sh_data_report_mode = SSHUB_ACCEL_WAKEUP_REPORT_MODE )
        	acquire_motion_detection_signal( initialMesConfig.accelBehavior, &isMotionDetected );

        scdsm_powersave_run_sm( isMotionDetected , algoSamples[0].scd_contact_state , scdsm_output_events , &scdsm_num_events_set );

        int num_events_processed = 0;
        while( num_events_processed != scdsm_num_events_set) {

        	EVENT_TABLE[scdsm_output_events[num_events_processed]].eventCallback(EVENT_TABLE[scdsm_output_events[num_events_processed]].eventParams);
        	num_events_processed++;

        }

     }


}
