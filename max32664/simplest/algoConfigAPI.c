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


#include <stdint.h>

#include "algoConfigAPI.h"
#include "SHComm.h"


#define MIN_MACRO(a,b) ((a)<(b)?(a):(b))




int SH_Max8614x_set_ppgreg(const uint8_t addr, const uint32_t val) {

	int status = sh_set_reg(SH_SENSORIDX_MAX8614X, addr, val, 1);
	return status;
}


int SH_Max8614x_get_ppgreg(const uint8_t addr , uint32_t *regVal){

    int status = sh_get_reg(SH_SENSORIDX_MAX8614X, (uint8_t) addr, regVal);
	return status;
}



int sh_set_cfg_wearablesuite_algomode( const uint8_t algoMode ){

	uint8_t Temp[1] = { algoMode };
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_ALGO_MODE, &Temp[0], 1);
	return status;
}

int sh_get_cfg_wearablesuite_algomode( uint8_t *algoMode ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_ALGO_MODE, &rxBuff[0], sizeof(rxBuff) );
	*algoMode =  rxBuff[1];
	return status;
}


int sh_set_cfg_wearablesuite_aecenable(const uint8_t isAecEnable ){

	uint8_t Temp[1] = { isAecEnable };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_AEC_ENABLE, &Temp[0], 1);
	return status;

}

int sh_get_cfg_wearablesuite_aecenable( uint8_t *isAecEnable ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_AEC_ENABLE, &rxBuff[0], sizeof(rxBuff) );
	*isAecEnable =  rxBuff[1];

	return status;
}

int sh_set_cfg_wearablesuite_scdenable( const uint8_t isScdcEnable ){

	uint8_t Temp[1] = { isScdcEnable };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_SCD_ENABLE, &Temp[0], 1);
	return status;

}

int sh_get_cfg_wearablesuite_scdenable( uint8_t *isScdEnable ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_SCD_ENABLE, &rxBuff[0], sizeof(rxBuff) );
	*isScdEnable =  rxBuff[1];

	return status;
}


int sh_set_cfg_wearablesuite_targetpdcurrent(const uint16_t targetPdCurr_x10){


	uint8_t Temp[2] = { (uint8_t)((targetPdCurr_x10 >> (1*8)) & 0xFF),  (uint8_t)((targetPdCurr_x10 >> (0*8)) & 0xFF)};
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT, &Temp[0], 2);

	return status;
}

int sh_get_cfg_wearablesuite_targetpdcurrent( uint16_t *targetPdCurr_x10){


	uint8_t rxBuff[2+1];  // first byte is status
    int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT, &rxBuff[0], sizeof(rxBuff));
    *targetPdCurr_x10 = (rxBuff[1] << 8) +  rxBuff[2] ;

    return status;

}

int sh_set_cfg_wearablesuite_minpdcurrent(const uint16_t minPdCurr_x10){


	uint8_t Temp[2] = { (uint8_t)((minPdCurr_x10 >> (1*8)) & 0xFF),  (uint8_t)((minPdCurr_x10 >> (0*8)) & 0xFF)};
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_PD_CURRENT, &Temp[0], 2);

	return status;
}

int sh_get_cfg_wearablesuite_minpdcurrent( uint16_t *minPdCurr_x10){


	uint8_t rxBuff[2+1];  // first byte is status
    int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_PD_CURRENT, &rxBuff[0], sizeof(rxBuff));
    *minPdCurr_x10 = (rxBuff[1] << 8) +  rxBuff[2] ;

    return status;

}


int sh_set_cfg_wearablesuite_initialpdcurrent(const uint16_t initPdCurr_x10){


	uint8_t Temp[2] = { (uint8_t)((initPdCurr_x10 >> (1*8)) & 0xFF),  (uint8_t)((initPdCurr_x10 >> (0*8)) & 0xFF)};
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_INIT_PD_CURRENT, &Temp[0], 2);

	return status;
}

int sh_get_cfg_wearablesuite_initialpdcurrent( uint16_t *initPdCurr_x10){


	uint8_t rxBuff[2+1];  // first byte is status
    int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_INIT_PD_CURRENT, &rxBuff[0], sizeof(rxBuff));
    *initPdCurr_x10 = (rxBuff[1] << 8) +  rxBuff[2] ;

    return status;

}


int sh_set_cfg_wearablesuite_autopdcurrentenable( const uint8_t isAutoPdCurrEnable ){

	uint8_t Temp[1] = { isAutoPdCurrEnable };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_AUTO_PD_CURRENT_ENABLE, &Temp[0], 1);
	return status;

}

int sh_get_cfg_wearablesuite_autopdcurrentenable( uint8_t *isAutoPdCurrEnable ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_AUTO_PD_CURRENT_ENABLE, &rxBuff[0], sizeof(rxBuff) );
	*isAutoPdCurrEnable =  rxBuff[1];

	return status;
}

int sh_set_cfg_wearablesuite_spo2cal( const uint32_t val[3] ){


	uint8_t CalCoef[12] = { (uint8_t)((val[0] >> (3*8)) & 0xFF),  (uint8_t)((val[0] >> (2*8)) & 0xFF), (uint8_t)((val[0] >> (1*8)) & 0xFF), (uint8_t)((val[0] >> (0*8)) & 0xFF), // A
							(uint8_t)((val[1] >> (3*8)) & 0xFF),  (uint8_t)((val[1] >> (2*8)) & 0xFF), (uint8_t)((val[1] >> (1*8)) & 0xFF), (uint8_t)((val[1] >> (0*8)) & 0xFF), // B
							(uint8_t)((val[2] >> (3*8)) & 0xFF),  (uint8_t)((val[2] >> (2*8)) & 0xFF), (uint8_t)((val[2] >> (1*8)) & 0xFF), (uint8_t)((val[2] >> (0*8)) & 0xFF)  // C
						   };

	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2_CAL, &CalCoef[0], 12);

	return status;
}

int sh_get_cfg_wearablesuite_spo2cal( int32_t val[3]){

	uint8_t rxBuff[12+1];  // first byte is status
    int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2_CAL, &rxBuff[0], sizeof(rxBuff));

    val[0] = (rxBuff[1] << 24) + (rxBuff[2] << 16) + (rxBuff[3] << 8) + (rxBuff[4] );
    val[1] = (rxBuff[5] << 24) + (rxBuff[6] << 16) + (rxBuff[7] << 8) + (rxBuff[8] );
    val[2] = (rxBuff[9] << 24) + (rxBuff[10] << 16) + (rxBuff[11] << 8) + (rxBuff[12] );

	return status;
}


/*  WARNING: GRANULARITY SHOULD BE 0.01 WILL TALK TO AFSHIN!!!!*/
int sh_set_cfg_wearablesuite_motionthreshold( const uint16_t motionThresh){

    uint8_t Temp[2] = { (uint8_t)((motionThresh >> (1*8)) & 0xFF),  (uint8_t)((motionThresh >> (0*8)) & 0xFF)};
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_MOTION_MAG_THRESHOLD, &Temp[0], 2);

	return status;
}

int sh_get_cfg_wearablesuite_motionthreshold( uint16_t *motionThresh){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_MOTION_MAG_THRESHOLD, &rxBuff[0], sizeof(rxBuff));
	*motionThresh = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}

int sh_set_cfg_wearablesuite_targetpdperiod( const uint16_t targPdPeriod){

    uint8_t Temp[2] = { (uint8_t)((targPdPeriod >> (1*8)) & 0xFF),  (uint8_t)((targPdPeriod >> (0*8)) & 0xFF)};
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT_PERIOD, &Temp[0], 2);

	return status;
}

int sh_get_cfg_wearablesuite_targetpdperiod( uint16_t *targPdPeriod){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT_PERIOD, &rxBuff[0], sizeof(rxBuff));
	*targPdPeriod = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}


int sh_set_cfg_wearablesuite_spo2motionperiod( const uint16_t spo2MotnPeriod){

    uint8_t Temp[2] = { (uint8_t)((spo2MotnPeriod >> (1*8)) & 0xFF),  (uint8_t)((spo2MotnPeriod >> (0*8)) & 0xFF)};
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_PERIOD, &Temp[0], 2);

	return status;
}

int sh_get_cfg_wearablesuite_spo2motionperiod( uint16_t *spo2MotnPeriod){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_PERIOD, &rxBuff[0], sizeof(rxBuff));
	*spo2MotnPeriod = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}

int sh_set_cfg_wearablesuite_spo2motionthreshold( const uint32_t spo2MotnThresh ){


	uint8_t Temp[4] = { (uint8_t)((spo2MotnThresh >> (3*8)) & 0xFF),  (uint8_t)((spo2MotnThresh >> (2*8)) & 0xFF),
			            (uint8_t)((spo2MotnThresh >> (1*8)) & 0xFF), (uint8_t)((spo2MotnThresh >> (0*8)) & 0xFF)   };

	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_THRESHOLD, &Temp[0], 4);

	return status;

}

int sh_get_cfg_wearablesuite_spo2motionthreshold( uint32_t *spo2MotnThresh){

	uint8_t rxBuff[4+1];  // first byte is status
	int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_THRESHOLD, &rxBuff[0], sizeof(rxBuff));
    *spo2MotnThresh = (rxBuff[1] << 24) + (rxBuff[2] << 16) + (rxBuff[3] << 8) + (rxBuff[4]);

    return status;

}


int sh_set_cfg_wearablesuite_spo2afecontrltimeout( const uint8_t spo2AfeCtrlTimeout ){

	uint8_t Temp[1] = { spo2AfeCtrlTimeout };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_AFE_TIMEOUT, &Temp[0], 1);
	return status;

}

int sh_get_cfg_wearablesuite_spo2afecontrltimeout( uint8_t *spo2AfeCtrlTimeout ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_AFE_TIMEOUT, &rxBuff[0], sizeof(rxBuff) );
	*spo2AfeCtrlTimeout =  rxBuff[1];

	return status;
}


int sh_set_cfg_wearablesuite_spo2timeout( const uint8_t spo2AlgoTimeout ){

	uint8_t Temp[1] = { spo2AlgoTimeout };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_TIMEOUT, &Temp[0], 1);

	return status;

}

int sh_get_cfg_wearablesuite_spo2timeout( uint8_t *spo2AlgoTimeout ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_TIMEOUT, &rxBuff[0], sizeof(rxBuff) );
	*spo2AlgoTimeout =  rxBuff[1];

	return status;
}

int sh_set_cfg_wearablesuite_initialhr( const uint8_t initialHr ){

	uint8_t Temp[1] = { initialHr };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_HR, &Temp[0], 1);

	return status;

}

int sh_get_cfg_wearablesuite_initialhr( uint8_t *initialHr ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_HR, &rxBuff[0], sizeof(rxBuff) );
	*initialHr =  rxBuff[1];

	return status;
}


int sh_set_cfg_wearablesuite_personheight( const uint16_t personHeight){

    uint8_t Temp[2] = { (uint8_t)((personHeight >> (1*8)) & 0xFF),  (uint8_t)((personHeight >> (0*8)) & 0xFF)};
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_HEIGHT, &Temp[0], 2);

	return status;
}

int sh_get_cfg_wearablesuite_personheight( uint16_t *personHeight){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_HEIGHT, &rxBuff[0], sizeof(rxBuff));
	*personHeight = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}


int sh_set_cfg_wearablesuite_personweight( const uint16_t personWeight){

    uint8_t Temp[2] = { (uint8_t)((personWeight >> (1*8)) & 0xFF),  (uint8_t)((personWeight >> (0*8)) & 0xFF)};
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_WEIGHT, &Temp[0], 2);

	return status;
}

int sh_get_cfg_wearablesuite_personweight( uint16_t *personWeight){

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_WEIGHT, &rxBuff[0], sizeof(rxBuff));
	*personWeight = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}

int sh_set_cfg_wearablesuite_personage( const uint8_t personAge ){

	uint8_t Temp[1] = { personAge };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,   SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_AGE, &Temp[0], 1);

	return status;

}

int sh_get_cfg_wearablesuite_personage( uint8_t *personAge ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_AGE, &rxBuff[0], sizeof(rxBuff) );
	*personAge =  rxBuff[1];

	return status;
}

int sh_set_cfg_wearablesuite_persongender( const uint8_t personGender ){

	uint8_t Temp[1] = { personGender };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,   SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_GENDER, &Temp[0], 1);

	return status;

}

int sh_get_cfg_wearablesuite_persongender( uint8_t *personGender ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_GENDER, &rxBuff[0], sizeof(rxBuff) );
	*personGender =  rxBuff[1];

	return status;
}


int sh_set_cfg_wearablesuite_mintintoption( const uint8_t minIntegrationTimeOpt ){

	uint8_t Temp[1] = { minIntegrationTimeOpt };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,   SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_INTEGRATION_TIME, &Temp[0], 1);

	return status;

}

int sh_get_cfg_wearablesuite_mintintoption( uint8_t *minIntegrationTimeOpt ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_INTEGRATION_TIME, &rxBuff[0], sizeof(rxBuff) );
	*minIntegrationTimeOpt =  rxBuff[1];

	return status;
}

int sh_set_cfg_wearablesuite_maxtintoption( const uint8_t maxIntegrationTimeOpt ){

	uint8_t Temp[1] = { maxIntegrationTimeOpt };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,   SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_INTEGRATION_TIME, &Temp[0], 1);

	return status;

}

int sh_get_cfg_wearablesuite_maxtintoption( uint8_t *maxIntegrationTimeOpt ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_INTEGRATION_TIME, &rxBuff[0], sizeof(rxBuff) );
	*maxIntegrationTimeOpt =  rxBuff[1];

	return status;
}


int sh_set_cfg_wearablesuite_minfsmpoption( const uint8_t minSampRateAveragingOpt ){

	uint8_t Temp[1] = { minSampRateAveragingOpt };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,   SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_SAMPLING_AVERAGE, &Temp[0], 1);

	return status;

}

int sh_get_cfg_wearablesuite_minfsmpoption( uint8_t *minSampRateAveragingOpt ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_SAMPLING_AVERAGE, &rxBuff[0], sizeof(rxBuff) );
	*minSampRateAveragingOpt =  rxBuff[1];

	return status;
}


int sh_set_cfg_wearablesuite_maxfsmpoption( const uint8_t maxSampRateAveragingOpt ){

	uint8_t Temp[1] = { maxSampRateAveragingOpt };
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,   SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_SAMPLING_AVERAGE, &Temp[0], 1);

	return status;

}

int sh_get_cfg_wearablesuite_maxfsmpoption( uint8_t *maxSampRateAveragingOpt ){

	uint8_t rxBuff[1+1]; // first byte is status
	int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE,  SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_SAMPLING_AVERAGE, &rxBuff[0], sizeof(rxBuff) );
	*maxSampRateAveragingOpt =  rxBuff[1];

	return status;
}

int sh_set_cfg_wearablesuite_whrmledpdconfig( const uint16_t whrmledpdconfig ){

	uint8_t Temp[2] = { (uint8_t)((whrmledpdconfig >> (1*8)) & 0xFF),  (uint8_t)((whrmledpdconfig >> (0*8)) & 0xFF)};
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_WHRMLEDPDCONFIGURATION, &Temp[0], 2);
	return status;

}


int sh_get_cfg_wearablesuite_whrmledpdconfig( uint16_t *whrmledpdconfig) {

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_WHRMLEDPDCONFIGURATION, &rxBuff[0], sizeof(rxBuff));
	*whrmledpdconfig = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}


int sh_set_cfg_wearablesuite_spo2ledpdconfig( const uint16_t spo2ledpdconfig ){

	uint8_t Temp[2] = { (uint8_t)((spo2ledpdconfig >> (1*8)) & 0xFF),  (uint8_t)((spo2ledpdconfig >> (0*8)) & 0xFF)};
    int status = sh_set_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2LEDPDCONFIGURATION, &Temp[0], 2);
	return status;

}


int sh_get_cfg_wearablesuite_spo2ledpdconfig( uint16_t *spo2ledpdconfig) {

	uint8_t rxBuff[2+1];  // first byte is status
	int status = sh_get_algo_cfg(SS_ALGOIDX_WHRM_WSPO2_SUITE, SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2LEDPDCONFIGURATION, &rxBuff[0], sizeof(rxBuff));
	*spo2ledpdconfig = (rxBuff[1] << 8) + rxBuff[2];

	return status;
}




/*
int SH_Max8614x_stop() {
	sh_disable_irq_mfioevent();



	int status;
	status = sh_sensor_disable(SH_SENSORIDX_MAX8614X);
	if(status != 0) {
		__DBGMESSAGE("Failed to disable 8614X\r\n",NULL);
		return -1;
    }


	status = sh_sensor_disable(SH_SENSORIDX_ACCEL);
	if(status != 0) {
		__DBGMESSAGE("Failed to disable Accelerometer\r\n",NULL);
		return -1;
    }


	status = sh_disable_algo(SH_ALGOIDX_WSPO2);
	if(status != 0) {
		__DBGMESSAGE("Failed to disable WSPO2\r\n",NULL);
		return -1;
    }


	status = sh_disable_algo(SH_ALGOIDX_WHRM);
	if(status != 0) {
		__DBGMESSAGE("Failed to disable WSPO2\r\n",NULL);
		return -1;
    }


	sh_clear_mfio_event_flag();
	sh_enable_irq_mfioevent();

	return status;
}

*/




/* **********************************************************************************************
 * 																							   	*
 *   					   COMMAND INTERFACE RELATED METHODS								 	*
 *																								*
 * **********************************************************************************************/




