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

/*
 * algoConfigAPI.h
 *
 *  Created on: Jan 30, 2019
 *      Author: Yagmur.Gok
 */

#ifndef ALGO_CONFIG_API_H_
#define ALGO_CONFIG_API_H_


enum _mxm_wearable_algo_suite_operating_mode {

    MXM_WEARABLE_ALGO_SUITE_CONTINUOUS_HRM_CONTINUOUS_SPO2_MODE= 0, /* Continuous HRM (and activity) and continuous SpO2 measurement mode */
    MXM_WEARABLE_ALGO_SUITE_CONTINUOUS_HRM_ONE_SHOT_SPO2_MODE  = 1, /* Continuous HRM (and activity) and one-shot SpO2 measurement mode   */
    MXM_WEARABLE_ALGO_SUITE_CONTINUOUS_HRM_MODE                = 2, /* Continuous HRM (and activity) mode                                 */
    MXM_WEARABLE_ALGO_SUITE_SAMPLED_HRM_MODE                   = 3, /* Sampled HRM (and activity) mode                 					  */
    MXM_WEARABLE_ALGO_SUITE_SAMPLED_HRM_ONE_SHOT_SPO2_MODE     = 4, /* Sampled HRM (and activity) and one-shot SpO2 measurement mode 	  */
    MXM_WEARABLE_ALGO_SUITE_ACTIVITY_TRACKING_ONLY_MODE        = 5, /* Activity tracking only mode                     					  */
    MXM_WEARABLE_ALGO_SUITE_SPO2_CALIBRATION_MODE              = 6, /* SpO2 calibration only mode                     					  */

};



int SH_Max8614x_set_ppgreg(const uint8_t addr, const uint32_t val);

int SH_Max8614x_get_ppgreg(const uint8_t addr , uint32_t *regVal);

int sh_set_cfg_wearablesuite_algomode( const uint8_t algoMode );

int sh_get_cfg_wearablesuite_algomode( uint8_t *algoMode );

int sh_set_cfg_wearablesuite_aecenable( const uint8_t isAecEnable );

int sh_get_cfg_wearablesuite_aecenable( uint8_t *isAecEnable );

int sh_set_cfg_wearablesuite_scdenable( const uint8_t isScdcEnable );

int sh_get_cfg_wearablesuite_scdenable( uint8_t *isScdEnable );

int sh_set_cfg_wearablesuite_targetpdcurrent(const uint16_t targetPdCurr_x10);

int sh_get_cfg_wearablesuite_targetpdcurrent( uint16_t *targetPdCurr_x10);

int sh_set_cfg_wearablesuite_minpdcurrent(const uint16_t minPdCurr_x10);

int sh_get_cfg_wearablesuite_minpdcurrent( uint16_t *minPdCurr_x10);

int sh_set_cfg_wearablesuite_initialpdcurrent(const uint16_t initPdCurr_x10);

int sh_get_cfg_wearablesuite_initialpdcurrent( uint16_t *initPdCurr_x10);

int sh_set_cfg_wearablesuite_autopdcurrentenable( const uint8_t isAutoPdCurrEnable );

int sh_get_cfg_wearablesuite_autopdcurrentenable( uint8_t *isAutoPdCurrEnable );

int sh_set_cfg_wearablesuite_spo2cal( const uint32_t val[3] );

int sh_get_cfg_wearablesuite_spo2cal( int32_t val[3]);

int sh_set_cfg_wearablesuite_motionthreshold( const uint16_t motionThresh);

int sh_get_cfg_wearablesuite_motionthreshold( uint16_t *motionThresh);

int sh_set_cfg_wearablesuite_targetpdperiod( const uint16_t targPdPeriod);

int sh_get_cfg_wearablesuite_targetpdperiod( uint16_t *targPdPeriod);

int sh_set_cfg_wearablesuite_spo2motionperiod( const uint16_t spo2MotnPeriod);

int sh_get_cfg_wearablesuite_spo2motionperiod( uint16_t *spo2MotnPeriod);

int sh_set_cfg_wearablesuite_spo2motionthreshold( const uint32_t spo2MotnThresh );

int sh_get_cfg_wearablesuite_spo2motionthreshold( uint32_t *spo2MotnThresh );

int sh_set_cfg_wearablesuite_spo2afecontrltimeout( const uint8_t spo2AfeCtrlTimeout );

int sh_get_cfg_wearablesuite_spo2afecontrltimeout( uint8_t *spo2AfeCtrlTimeout );

int sh_set_cfg_wearablesuite_spo2timeout( const uint8_t spo2AlgoTimeout );

int sh_get_cfg_wearablesuite_spo2timeout( uint8_t *spo2AlgoTimeout );

int sh_set_cfg_wearablesuite_initialhr( const uint8_t initialHr );

int sh_get_cfg_wearablesuite_initialhr( uint8_t *initialHr );

int sh_set_cfg_wearablesuite_personheight( const uint16_t personHeight);

int sh_get_cfg_wearablesuite_personheight( uint16_t *personHeight);

int sh_set_cfg_wearablesuite_personweight( const uint16_t personWeight);

int sh_get_cfg_wearablesuite_personweight( uint16_t *personWeight);

int sh_set_cfg_wearablesuite_personage( const uint8_t personAge );

int sh_get_cfg_wearablesuite_personage( uint8_t *personAge );

int sh_set_cfg_wearablesuite_persongender( const uint8_t personGender );

int sh_get_cfg_wearablesuite_persongender( uint8_t *personGender );

int sh_set_cfg_wearablesuite_mintintoption( const uint8_t minIntegrationTimeOpt );

int sh_get_cfg_wearablesuite_mintintoption( uint8_t *minIntegrationTimeOpt );

int sh_set_cfg_wearablesuite_maxtintoption( const uint8_t maxIntegrationTimeOpt );

int sh_get_cfg_wearablesuite_maxtintoption( uint8_t *maxIntegrationTimeOpt );

int sh_set_cfg_wearablesuite_minfsmpoption( const uint8_t minSampRateAveragingOpt );

int sh_get_cfg_wearablesuite_minfsmpoption( uint8_t *minSampRateAveragingOpt );

int sh_set_cfg_wearablesuite_maxfsmpoption( const uint8_t maxSampRateAveragingOpt );

int sh_get_cfg_wearablesuite_maxfsmpoption( uint8_t *maxSampRateAveragingOpt );

int sh_set_cfg_wearablesuite_whrmledpdconfig( const uint16_t whrmledpdconfig );

int sh_get_cfg_wearablesuite_whrmledpdconfig( uint16_t *whrmledpdconfig);

int sh_set_cfg_wearablesuite_spo2ledpdconfig( const uint16_t spo2ledpdconfig );

int sh_get_cfg_wearablesuite_spo2ledpdconfig( uint16_t *spo2ledpdconfig);

#endif /* ALGO_CONFIG_API_H_ */
