/*
 * test_flexcan_functions.c
 *
 *  Created on: Jul 8, 2015
 *      Author: Angus
 */

#include "main.h"

void test_flexcan(void) {

	/*! this is just for testing the sendCANMessage function */
	int i;
	int8_t ret;
	uint8_t pdata8[8];
	uint32_t pdata32[8];

	uint32_t id = 0x123; //29-bit only for extended
	uint32_t pid = 0x3ffff;
	uint8_t dlc = 8;
	uint32_t pdlc;
	uint8_t pdlc8;

	for (i = 0; i < dlc; i++)
	{
		pdata8[i] = 10 + i;
	}



	//printf("Sending message \r\n");
	//ret =  FLEXCANSendMessage(id, dlc, pdata8, FLEXCAN_TX_STD_FRAME);
	//ret =  FLEXCANSendMessage(pid, dlc, pdata8, FLEXCAN_TX_XTD_FRAME); //extended deadbeef msg

	printf("Receiving message \r\n");


	/****************************************************
	 * FLEXCANReceiveMessage is now fully functional
	 */

	/*
			ret =  FLEXCANReceiveMessage(&pid, &pdlc, pdata32, &flags);

			printf("ID: 0x%lx, DLC=%lu \r\n",pid, pdlc);
			printf("nRX MB data: 0x\r\n");
			for (i = 0; i < pdlc; i++){
				printf("%02x ", pdata32[i]);

			}*/

	ret = getCANFrame(&pid, &pdlc8, pdata8);

	printf("ID: 0x%lx, DLC=%lu \r\n",pid, pdlc8);
	printf("RX MB data: 0x");

	for (i = 0; i < pdlc8; i++){
		printf("%02x ", pdata8[i]);

	}
	printf("\r\n");

}
