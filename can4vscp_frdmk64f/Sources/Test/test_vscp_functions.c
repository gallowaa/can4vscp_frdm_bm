/*
 * test_vscp_functions.c
 *
 *  Created on: Jun 7, 2015
 *      Author: Angus Galloway
 */

#include "main.h"

/*!
 * @brief verify functionality of the custom defined vscp functions
 *        vscp_firmware.h @ line 457
 */
void test_vscp_externals(){

	uint8_t FlashValue8; 		   /*! gets an 8-bit vscp data field from permanent storage */
	uint8_t val;				   /*! assign a value to write to buffer -> permanent storage */

	printf("Testing vscp_functions\r\n");

	val = 0x11;
	vscp_writeNicknamePermanent(val);
	FlashValue8 = vscp_readNicknamePermanent();
	printf("Read %d dec, %x hex\r\n", FlashValue8, FlashValue8);

	val = 0x22;
	vscp_setSegmentCRC(val);
	FlashValue8 = vscp_getSegmentCRC();
	printf("Read %d dec, %x hex\r\n", FlashValue8, FlashValue8);

	val = 0x33;
	vscp_setControlByte(val);
	FlashValue8 = vscp_getControlByte();
	printf("Read %d dec, %x hex\r\n", FlashValue8, FlashValue8);
}


