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

#include "authentication.h"
#include "SHComm.h"
#include "demoDefinitions.h"



/*
 * IMPORTANT NOTE: AUTHENTICATION PROCESS IS ONLY REUQIRED FOR USING MAXIM SINGLE LIBARY OF ALGORITHMS
 *                 HRV + STRESS + SLEEP + RESPIRATION RATE which accepts WHRM suite outputs of Sensorhub
 *                 USERS WHO JUST USE SENSORHUB SHALL SKIP THIS PROCESS!
 *
 * AUTHENTICATION STEPS:
 * 1. Ask Sensorhub for session authentication initials : call to sh_get_dhparams()
 * 2. Pass authentication initials to Maxim Single Library and get session local string
 *                        from Maxim Single Library
 * 3. Send session local string of Maxim Single Library to senbsorhub : sh_set_dhlocalpublic()
 * 4. Ask Sensorhub for session public string of sensorhub : call to sh_get_dhremotepublic()
 * 5. Ask Sensorhub for session final authentication string : call to sh_get_authentication()
 * 6. Pass session public string of sensorhub and session final authentication string from sensorhub
 *                        to Maxim Single Library
 *
 *
 * */


authentication_data_t sessionAuthData;

int authenticate_to_sensorhub(void) {

    int idx;
    /* Step 1*/
	int status = sh_get_dhparams( &sessionAuthData.authInitialsArray[0], AUTH_INITIALS_SEQUENCE_SIZE);
	if( status != 0)
		return -1;

	/* Step 2*/
	// PASS AUTH INITIALS TO MAXIM SINGLE LIBRARY OF ALOGORITHMS HRV + SLEEP +STRESS + RESPIRATION RATE
	for (idx = 0 ; idx < AUTH_INITIALS_SEQUENCE_SIZE ; idx++)
		SERIALOUT("%c", sessionAuthData.authInitialsArray[idx]);

	wait_ms(1000);

	/* Step 3*/
   // get form Maxim single library and pass session local string of Maxim Single Library to senbsorhub
   status = sh_set_dhlocalpublic( &sessionAuthData.authLocalPublicString[0] , AUTH_PUBLIC_STRING_SIZE);
	if( status != 0)
		return -1;

	wait_ms(1000);

	/* Step 4*/
    status =  sh_get_dhremotepublic( &sessionAuthData.authSensorhubPublicString[0], AUTH_PUBLIC_STRING_SIZE);
    if( status != 0)
		return -1;

	wait_ms(1000);

	/* Step 5*/
	status = sh_get_authentication( &sessionAuthData.authFinalAuthString[0] , AUTH_FINAL_SEQUENCE_SIZE );

	/* Step 6*/
	// Pass session public string of sensorhub and session final authentication string from sensorhub to Maxim Single Library
	for (idx = 0 ; idx < AUTH_INITIALS_SEQUENCE_SIZE ; idx++)
		SERIALOUT("%c", sessionAuthData.authSensorhubPublicString[idx]);
	for (idx = 0 ; idx < AUTH_FINAL_SEQUENCE_SIZE ; idx++)
		SERIALOUT("%c", sessionAuthData.authFinalAuthString[idx]);

}





