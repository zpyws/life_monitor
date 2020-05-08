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
 * bootldrInterface.h
 *
 *  Created on: Feb 7, 2019
 *      Author: Yagmur.Gok
 */

#ifndef SOURCE_CMDUI_BOOTLDRINTERFACE_H_
#define SOURCE_CMDUI_BOOTLDRINTERFACE_H_

typedef int (*cmdExecFunc)( const char*); // typedef void (*cmdExecFunc)( const void*);

typedef struct {
	char const* cmdStr;
	cmdExecFunc execute;
	char const *help;
}cmd_interface_tb;

#define FLASH_ERR_GENERAL   -1
#define FLASH_ERR_CHECKSUM  -2
#define FLASH_ERR_AUTH      -3


int SH_BOOTLDR_enter_blmode(const char *arg);
int SH_BOOTLDR_exit_blmode(const char *arg);
int SH_BOOTLDR_get_pagesz(const char *arg);
int SH_BOOTLDR_set_pagecount(const char *arg);
int SH_BOOTLDR_set_iv(const char *arg);
int SH_BOOTLDR_set_authentication(const char *arg);
int SH_BOOTLDR_eraseflash(const char *arg);
int SH_BOOTLDR_flash(const char *arg);
int SH_BOOTLOADER_image_on_ram( const char *arg );
int SH_BOOTLDR_flash_appimage_from_ram(const char *arg);
int SH_BOOTLDR_set_host_bootcmds_delay_factor( const char *arg);
int SH_BOOTLDR_set_host_ebl_mode(const char *arg);
int SH_BOOTLDR_get_host_bootcmds_delay_factor( const char *arg);
int SH_BOOTLDR_get_host_ebl_mode(const char *arg);
int BOOTLDR_get_host_bootloader_state(const char *arg);

#define NUMCMDSBOOTLDRAPI (15)

const cmd_interface_tb CMDTABLEBOOTLDR[] = {

		{  "bootldr"     	  , SH_BOOTLDR_enter_blmode	      			  , "resets and puts sensor hub to bootloader mode "	   			 												},
		{  "exit"             , SH_BOOTLDR_exit_blmode        			  , "exits sensor hub from bootloader mode to app mode" 			  							  					},
		{  "page_size"        , SH_BOOTLDR_get_pagesz         			  , "returns sensor hub bootloader page size for app data pages"   							      					},
		{  "num_pages"        , SH_BOOTLDR_set_pagecount      			  , "sets sensor hub bootloader app image pages, uasge: num_pages PAGES" 				    						},
		{  "set_iv"           , SH_BOOTLDR_set_iv             			  , "sets sensor hub bootloader initial vector bytes, usage: set_iv XXXXXXXXXXX (11 hex chrs)"      				},
		{  "set_auth"         , SH_BOOTLDR_set_authentication 			  , "sets sensor hub bootloader authentication bytes, usage: set_iv XXXXXXXXXXXXXXXX (16 hex chrs)" 				},
		{  "erase"            , SH_BOOTLDR_eraseflash         			  , "erases sesn hub application flash memory" 							        				  					},
		{  "image_on_ram"     , SH_BOOTLOADER_image_on_ram    			  , "selects pagBypage download-flash / block download-flash options"           									},
		{  "flash"            , SH_BOOTLDR_flash              	    	  ,  "flash image to hub/dowload pages from PC based on image_on_ram selection" 									},
		{  "image_flash"      , SH_BOOTLDR_flash_appimage_from_ram        , "flashes app image in ram to sensor hub, call after flash cmd in image_on_ram mode"								},
		{  "set_cfg host cdf" , SH_BOOTLDR_set_host_bootcmds_delay_factor , "sets delay factor for bootoader cmd waits default 1, usage: set_cfg host cdf FACTOR"       					},
		{  "set_cfg host ebl" , SH_BOOTLDR_set_host_ebl_mode              , "sets GPIO/CMD reset for reset hub to bootoader mode. default GPIO, usage: set_cfg host ebl 1/0,  1 for GPIO"  	},
		{  "get_cfg host cdf" , SH_BOOTLDR_get_host_bootcmds_delay_factor , "sets delay factor for bootoader cmd waits default 1, usage: set_cfg host cdf FACTOR"       					},
		{  "get_cfg host ebl" , SH_BOOTLDR_get_host_ebl_mode              , "sets GPIO/CMD reset for reset hub to bootoader mode. default GPIO, usage: set_cfg host ebl 1/0,  1 for GPIO"  	},
		{  "get_host_boot_state_info" , BOOTLDR_get_host_bootloader_state , "gets boot state keeping struct of host"  	                                                                    },


};

#endif /* SOURCE_CMDUI_BOOTLDRINTERFACE_H_ */
