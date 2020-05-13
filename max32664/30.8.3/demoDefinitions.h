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
 * demoDefinitions.h
 *
 *  Created on: Feb 13, 2019
 *      Author: Yagmur.Gok
 */

#ifndef SOURCE_DEMODEFINITIONS_H_
#define SOURCE_DEMODEFINITIONS_H_


//#include "USBSerial.h"

//extern Serial daplink;
//extern USBSerial microUSB;
//#define SERIALOUT printf
#define SERIALOUT rt_kprintf
#define SERIALIN microUSB._getc
#define SERIAL_AVAILABLE microUSB.readable




#define SYSTEM_USES_MFIO_PIN

//RECOMMENDED TO UNCOMMENT FOR BOOTLOADING FACILITY
#define SYSTEM_USES_RST_PIN

// FOR BOOTLOADER FACILITY CHECK FOR ebl_mode setting defined within SHComm source file!!

#define COMPILER_INLINED  __attribute__((__always_inline__))

#endif /* SOURCE_DEMODEFINITIONS_H_ */
