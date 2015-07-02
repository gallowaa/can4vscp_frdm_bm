/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * main.h
 *
 *  Created on: Jun 5, 2015
 *      Author: Angus Galloway
 */

#ifndef SOURCES_MAIN_H_
#define SOURCES_MAIN_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/** vscp_firmware **/
#include <vscp_firmware.h>
#include <vscp_class.h>
#include <vscp_type.h>


#include "pin_mux.h"
#include "board.h"

#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"
#include "fsl_lptmr_driver.h"

//#include "flash_al.h"
//#include "SSD_FTFx.h"

/* This section replaces EEPROM Storage from can4vscp_paris/main.h with FLASH
 * There is a vscp wrapper that calls readEEPROM, readFLASH
 */



#define VSCP_FLASH_BOOTLOADER_FLAG		0x00 //reserved for bootloader flag

#define VSCP_FLASH_NICKNAME 			0x01	// Persistent nickname id storage
#define VSCP_FLASH_SEGMENT_CRC			0x02	// Persistent segment crc storage
#define VSCP_FLASH_CONTROL 				0x03	// Persistent control byte

#define VSCP_FLASH_REG_USERID 			0x04
#define VSCP_FLASH_REG_USERID1 			0x05
#define VSCP_FLASH_REG_USERID2 			0x06
#define VSCP_FLASH_REG_USERID3 			0x07
#define VSCP_FLASH_REG_USERID4 			0x08

// The following can be stored in flash or eeprom

#define VSCP_FLASH_REG_MANUFACTUR_ID0 	0x09
#define VSCP_FLASH_REG_MANUFACTUR_ID1 	0x0A
#define VSCP_FLASH_REG_MANUFACTUR_ID2 	0x0B
#define VSCP_FLASH_REG_MANUFACTUR_ID3 	0x0C

#define VSCP_FLASH_REG_MANUFACTUR_SUBID0	0x0D
#define VSCP_FLASH_REG_MANUFACTUR_SUBID1	0x0E
#define VSCP_FLASH_REG_MANUFACTUR_SUBID2	0x0F
#define VSCP_FLASH_REG_MANUFACTUR_SUBID3	0x10

// The following can be stored in program ROM or EEPROM

#define VSCP_FLASH_REG_GUID 			0x11 	// Start of GUID MSB of 16
												//		 0x11 - 0x20

#define VSCP_FLASH_REG_DEVICE_URL		0x21	// Start of Device URL storage
                                                // 		0x21 - 0x40

#define VSCP_FLASH_END                 	0x41	// marks end of VSCP EEPROM usage
                                                //   (next free position)

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  	For main.c application
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/** VSCP Status LED **/
#define STATUS_LED_EN 			LED1_EN
#define STATUS_LED_ON 			LED1_ON
#define STATUS_LED_OFF 			LED1_OFF
#define STATUS_LED_TOGGLE 		LED1_TOGGLE


/** FOR 1ms clock check on scope **/

#define CLOCK_PIN_EN  		(GPIO_DRV_OutputPinInit(&gpioPins[0]))  				  /*!< clk pin en */
#define CLOCK_PIN_HI	 	(GPIO_DRV_WritePinOutput(gpioPins[0].pinName, 1))         /*!< clk pin high level */
#define CLOCK_PIN_LO 		(GPIO_DRV_WritePinOutput(gpioPins[0].pinName, 0))         /*!< clk pin low level */
#define CLOCK_PIN_TOGGLE 	(GPIO_DRV_TogglePinOutput(gpioPins[0].pinName))        	  /*!< clk toggle */

/** FOR SPI0 **/

#define SPI0_CS_EN  		(GPIO_DRV_OutputPinInit(&gpioPins[1]))  				  /*!< Enable target SPI0 CS gpio pin */
#define SPI0_CS_DESELECT 	(GPIO_DRV_WritePinOutput(gpioPins[1].pinName, 1))         /*!< Turn off cs */
#define SPI0_CS_SELECT 		(GPIO_DRV_WritePinOutput(gpioPins[1].pinName, 0))         /*!< Turn on cs */


/** FOR FRDM-CAN-VSCP Shield CAN PHY Standby pin **/

#define CAN0_STB_EN  		(GPIO_DRV_OutputPinInit(&gpioPins[2]))  				  /*!< Enable STB pin, connected to PTB9*/
#define CAN0_STB_HI	 		(GPIO_DRV_WritePinOutput(gpioPins[2].pinName, 1))         /*!< CAN PHY is in low power mode */
#define CAN0_STB_LO 		(GPIO_DRV_WritePinOutput(gpioPins[2].pinName, 0))         /*!< CAN PHY is in normal mode */


///////////////////////////////////////////////////////////////////////////////
// 						Prototypes
///////////////////////////////////////////////////////////////////////////////

extern uint32_t RelocateFunction(uint32_t dest, uint32_t size, uint32_t src);

extern uint8_t sendTimer;  // Timer for CAN send


/* test_vscp_functions_al.c  */

/*!
 * @brief verify functionality of the custom defined vscp functions
 *        vscp_firmware.h @ line 457
 */
void test_vscp_externals();

void vscp_FLASHFlush();


#endif /* SOURCES_MAIN_H_ */
