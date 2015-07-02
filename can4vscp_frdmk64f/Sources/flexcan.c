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
 */
#include "flexcan.h"


// ***************************************************************************
// 						Subroutines for flexcan
// ***************************************************************************

/* Global variables*/
uint32_t txIdentifier;
uint32_t rxIdentifier;
uint32_t txRemoteIdentifier;
uint32_t rxRemoteIdentifier;
uint32_t txMailboxNum;
uint32_t rxMailboxNum;
uint32_t rxRemoteMailboxNum;
uint32_t rxRemoteMailboxNum;
flexcan_state_t canState;

uint32_t instance = BOARD_CAN_INSTANCE;
uint32_t numErrors;

uint32_t result;
flexcan_user_config_t flexcanData;
uint32_t canPeClk;

/* The following tables are the CAN bit timing parameters that are calculated by using the method
 * outlined in AN1798, section 4.1.
 */
/*
 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
 * a protocol engine clock of 60MHz
 */
flexcan_time_segment_t bitRateTable60Mhz[] = {
		{ 6, 7, 7, 19, 3},  /* 125 kHz */
		{ 6, 7, 7,  9, 3},  /* 250 kHz */
		{ 6, 7, 7,  4, 3},  /* 500 kHz */
		{ 6, 5, 5,  3, 3},  /* 750 kHz */
		{ 6, 5, 5,  2, 3},  /* 1   MHz */
};

/*
 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
 * a protocol engine clock of 48MHz
 */
flexcan_time_segment_t bitRateTable48Mhz[] = {
		{ 6, 7, 7, 15, 3},  /* 125 kHz */
		{ 6, 7, 7,  7, 3},  /* 250 kHz */
		{ 6, 7, 7,  3, 3},  /* 500 kHz */
		{ 6, 3, 3,  3, 3},  /* 750 kHz */
		{ 6, 3, 3,  2, 3},  /* 1   MHz */
};


void flexcan_init(void) {

	numErrors = 0;

	flexcanData.max_num_mb = 16;
	flexcanData.num_id_filters = kFlexCanRxFifoIDFilters_8;
	flexcanData.is_rx_fifo_needed = false;
	flexcanData.flexcanMode = kFlexCanNormalMode;

	// Select mailbox number
	rxMailboxNum = 8;
	txMailboxNum = 9;
	rxRemoteMailboxNum = 10;
	rxRemoteMailboxNum = 11;

	// Select mailbox ID
	rxRemoteIdentifier = 0x0F0;
	txRemoteIdentifier = 0x00F;

	// Set rxIdentifier as same as txIdentifier to receive loopback data
	rxIdentifier = 0x124;
	txIdentifier = 0x123;

	/* The following tables are the CAN bit timing parameters that are calculated by using the method
	 * outlined in AN1798, section 4.1.
	 */
	/*
	 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
	 * a protocol engine clock of 60MHz
	 */
	flexcan_time_segment_t bitRateTable60Mhz[] = {
			{ 6, 7, 7, 19, 3},  /* 125 kHz */
			{ 6, 7, 7,  9, 3},  /* 250 kHz */
			{ 6, 7, 7,  4, 3},  /* 500 kHz */
			{ 6, 5, 5,  3, 3},  /* 750 kHz */
			{ 6, 5, 5,  2, 3},  /* 1   MHz */
	};

	/*
	 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
	 * a protocol engine clock of 48MHz
	 */
	flexcan_time_segment_t bitRateTable48Mhz[] = {
			{ 6, 7, 7, 15, 3},  /* 125 kHz */
			{ 6, 7, 7,  7, 3},  /* 250 kHz */
			{ 6, 7, 7,  3, 3},  /* 500 kHz */
			{ 6, 3, 3,  3, 3},  /* 750 kHz */
			{ 6, 3, 3,  2, 3},  /* 1   MHz */
	};


	result = FLEXCAN_DRV_Init(instance, &canState, &flexcanData);
	if (result)
	{
		numErrors++;
		printf("\r\nFLEXCAN initilization. result: 0x%lx", result);
	}

	if (FLEXCAN_HAL_GetClock((g_flexcanBase[instance])))
	{
		canPeClk = CLOCK_SYS_GetFlexcanFreq(0, kClockFlexcanSrcBusClk);
	}
	else
	{
		canPeClk = CLOCK_SYS_GetFlexcanFreq(0, kClockFlexcanSrcOsc0erClk);
	}

	switch (canPeClk)
	{
	case 60000000:
		result = FLEXCAN_DRV_SetBitrate(instance, &bitRateTable60Mhz[0]); // 125kbps for vscp
		break;
	case 48000000:
		result = FLEXCAN_DRV_SetBitrate(instance, &bitRateTable48Mhz[0]); // 125kbps
		break;
	default:
		printf("\r\nFLEXCAN bitrate table not available for PE clock: %lu", canPeClk);
		//return kStatus_FLEXCAN_Fail;
	}
	if (result)
	{
		numErrors++;
		printf("\r\nFLEXCAN set bitrate failed. result: 0x%lx", result);
	}

	//FLEXCAN_DRV_SetRxMaskType(instance, kFlexCanRxMaskIndividual);
	FLEXCAN_DRV_SetRxMaskType(instance, kFlexCanRxMaskGlobal);


	result = FLEXCAN_DRV_SetRxMbGlobalMask(instance, kFlexCanMsgIdStd, 0x123);
	if (result)
	{
		numErrors++;
		printf("\r\nFLEXCAN set rx MB global mask. result: 0x%lx", result);
	}

	/*
	// Standard ID
	result = FLEXCAN_DRV_SetRxIndividualMask(instance, kFlexCanMsgIdStd, rxMailboxNum, 0x7FF);
	if(result)
	{
		numErrors++;
		printf("\r\nFLEXCAN set rx individual mask with standard ID fail. result: 0x%lx", result);
	}
	 */

	// Extern ID
	result = FLEXCAN_DRV_SetRxIndividualMask(instance, kFlexCanMsgIdExt, rxMailboxNum, 0x123);
	if(result)
	{
		numErrors++;
		printf("\r\nFLEXCAN set rx individual mask with standard ID fail. result: 0x%lx", result);
	}
}

// FlexCAN receive configuration
void receive_mb_config(void)
{
	uint32_t result;
	flexcan_data_info_t rxInfo;

	rxInfo.msg_id_type = kFlexCanMsgIdStd;
	rxInfo.data_length = 8;

	printf("\r\nFlexCAN MB receive config");

	/* Configure RX MB fields*/
	result = FLEXCAN_DRV_ConfigRxMb(instance, rxMailboxNum, &rxInfo,rxIdentifier);
	if (result)
	{
		numErrors++;
		printf("\r\nFlexCAN RX MB configuration failed. result: 0x%lx", result);
	}
}

// Send configuration and transfer FlexCAN messages
void transfer_mb_loopback(void)
{
	uint32_t result, temp;
	flexcan_msgbuff_t rxMb;

	//send_data();

	result = FLEXCAN_DRV_RxMessageBuffer(instance, rxMailboxNum,&rxMb);

	while(FLEXCAN_DRV_GetReceiveStatus(instance) != kStatus_FLEXCAN_Success) {}
	if (result)
	{
		numErrors++;
		printf("\r\nFLEXCAN receive configuration failed. result: 0x%lx", result);
	}
	else
	{
		temp = ((rxMb.cs) >> 16) & 0xF;
		//printf("\r\nDLC=%d, mb_idx=%d", temp, rxMailboxNum);
		printf("\r\nDLC=%lu, mb_idx=%lu", temp, rxMailboxNum);
		printf("\r\nRX MB data: 0x");

		for (result = 0; result < temp; result++)
			printf("%02x ", rxMb.data[result]);

		printf("\r\nID: 0x%lx", rxMb.msgId);
	}

}

/*!
    @brief Send a VSCP frame
    @param unsigned long id -
    @return TRUE on success.
 */
int FLEXCANSendMessage(uint32_t id, uint8_t *pdata, uint8_t dataLengthBytes, FLEXCAN_TX_MSG_FLAGS msgFlags)
{
	//uint8_t data[8];
	uint32_t result, i;
	flexcan_data_info_t txInfo;

	txIdentifier = id;

	/* Extended CAN frame? */
	if( FLEXCAN_TX_XTD_FRAME == msgFlags ){
		txInfo.msg_id_type = kFlexCanMsgIdExt ;
	}
	/* Standard CAN frame? */
	else if (FLEXCAN_TX_STD_FRAME == msgFlags){
		txInfo.msg_id_type = kFlexCanMsgIdStd;
	}
	/* Default to standard */
	else {
		printf("\r\n Invalid FLEXCAN_TX_MSG_FLAG, defaulting to STD frame.");
		txInfo.msg_id_type = kFlexCanMsgIdStd;
	}

	//txInfo.data_length = 8;
	txInfo.data_length = dataLengthBytes;

	/*
	for (i = 0; i < txInfo.data_length; i++)
	{
		data[i] = 10 + i;
	}*/

	printf("\r\nFlexCAN send config");


	/* todo: txIdentifier and txMailboxNum are extra features, figure out they can cooperate with vscp */
	result = FLEXCAN_DRV_ConfigTxMb(instance, txMailboxNum, &txInfo, txIdentifier);

	if (result)
	{
		printf("\nTransmit MB config error. Error Code: 0x%lx", result);
	}
	else
	{
		result = FLEXCAN_DRV_SendBlocking(instance, txMailboxNum, &txInfo, txIdentifier, pdata, 1000); /* 1000ms timeout or can use OSA_WAIT_FOREVER */

		if (result)
		{
			numErrors++;
			printf("\r\nTransmit send configuration failed. result: 0x%lx", result);
		}

		else
		{
			printf("\r\nData transmitted: ");
			for (i = 0; i < txInfo.data_length; i++ )
			{
				printf("%02x ", pdata[i]);
			}
		}
	}
	return result;
}

/*!
    @brief Get a VSCP frame.  Use this function to check for
    	   full receive buffer and extract received data into local buffers.
    @param pid - pointer to id
    @param pdlc - pointer to data length in bytes
    @param pdata - pointer to data
    @param msgFlags - type of can frame
    @return TRUE on success.
 */
int FLEXCANReceiveMessage(uint32_t *pid, uint8_t *pdlc, uint8_t *pdata, ECAN_RX_MSG_FLAGS *msgFlags)
{

	//uint32_t result;
	//flexcan_data_info_t rxInfo;

	/*

	// Extended CAN frame?
	if( FLEXCAN_RX_XTD_FRAME == msgFlags ){
		rxInfo.msg_id_type = kFlexCanMsgIdExt ;
	}
	// Standard CAN frame?
	else if (FLEXCAN_RX_STD_FRAME == msgFlags){
		rxInfo.msg_id_type = kFlexCanMsgIdStd;
	}
	// Default to standard
	else {
		printf("\r\n Invalid FLEXCAN_RX_MSG_FLAG, defaulting to STD frame.");
		rxInfo.msg_id_type = kFlexCanMsgIdStd;
	}
	*/




}
