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

#include "cmdInterface.h"
#include "bootldrAPI.h"
#include "SHComm.h"
#include "demoDefinitions.h"


/* @brief    function to get the current firmware version of Sensor Hub".
 *
 * @param[in]   arg : NULL string, just to match the form of command table function pointer type
 *
 * */
static int get_hub_firmware_version(const char* arg){

    int idx;
	int status = -1;
    static const int MAXFWDESCSIZE = 5;
	uint8_t descArray[MAXFWDESCSIZE];
    uint8_t descSz;

    status = sh_get_ss_fw_version( &descArray[0] , &descSz);
    if(status == 0x00 && descSz > 0 &&  descSz < MAXFWDESCSIZE){
    	 SERIALOUT("\r\n Firmware Version of Sensor Hub is = ");
         for(idx = 0 ; idx != descSz ; idx++)
        	 SERIALOUT("%d.", descArray[idx]);
         SERIALOUT("\r\n");
    }


}


static int get_hub_operating_mode(const char* arg){

	uint8_t hubMode;
	int status = sh_get_sensorhub_operating_mode(&hubMode);
	if( status == 0x00)
		SERIALOUT("\r\n hub_operating_mode=%s\r\n", (hubMode == 0x00)? "APPMODE":"BOOTLOADERMODE" );
	else
		SERIALOUT("\r\n%s err=%d\r\n", "get_sensorhub_opmode", status);

    return status;
}


cmd_interface_tb getHubModeCMD       =  {"get_sensorhub_opmode", get_hub_operating_mode , "gets mode of host app or bootloader"};
cmd_interface_tb getHubFwVersionCMD  = {"get_hub_fwversion"   , get_hub_firmware_version , "gets mode of host app or bootloader"};

static bool starts_with(const char* str1, const char* str2)
{
	while (*str1 && *str2) {
		if (*str1 != *str2)
			return false;
		str1++;
		str2++;
	}

	if (*str2)
		return false;

	return true;
}


int parse_execute_command( const char *cmd_str)
{

	int found = 0;
    int tableIdx;


    if( starts_with(&cmd_str[0], getHubModeCMD.cmdStr)) {
    	int  status = getHubModeCMD.execute(cmd_str);
    	found = 1;
    }

    if( starts_with(&cmd_str[0], getHubFwVersionCMD.cmdStr)) {
    	int  status = getHubFwVersionCMD.execute(cmd_str);
    	found = 1;
    }

   	tableIdx = NUMCMDSBOOTLDRAPI;
	do{
			tableIdx -= 1;
			if (starts_with(&cmd_str[0], CMDTABLEBOOTLDR[tableIdx].cmdStr)){

				CMDTABLEBOOTLDR[tableIdx].execute(cmd_str);

				SERIALOUT(" \r\n"); // Here is needed due to a bug on mbed serial!
				found = 1;
			}

	}while(tableIdx && found == 0 );


    return found;
}

COMPILER_INLINED
void cmdIntf_build_command(char ch)
{
	static char cmd_str[1024];
    static int cmd_idx = 0;
    int status;

    if (ch == 0x00) {
		return;
	}

	if ((ch == '\n') || (ch == '\r')) {
		if (cmd_idx < 1024)
		cmd_str[cmd_idx++] = '\0';
		status = parse_execute_command(cmd_str);

		//Clear cmd_str
		while (cmd_idx > 0)
			cmd_str[--cmd_idx] = '\0';

	} else if ((ch == 0x08 || ch == 0x7F) && cmd_idx > 0) {
		//Backspace character
		if (cmd_idx > 0)
			cmd_str[--cmd_idx] = '\0';
	} else {

		if (cmd_idx < 1024)
			cmd_str[cmd_idx++] = ch;
	}

}


