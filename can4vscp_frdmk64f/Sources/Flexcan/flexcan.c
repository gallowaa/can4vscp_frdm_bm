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
#define MASK
//#define BUSY_WAIT

// ***************************************************************************
// 						Subroutines for flexcan
// ***************************************************************************

/* Global variables*/
uint32_t txIdentifier;
uint32_t rxIdentifier;
uint32_t txRemoteIdentifier;
uint32_t rxRemoteIdentifier;
uint32_t rxMailboxNum = 0;
uint32_t txMailboxNum = 1;
uint32_t rxRemoteMailboxNum;
uint32_t rxRemoteMailboxNum;
flexcan_state_t canState;

//flexcan_msgbuff_t rxMb;

/*! These MUST be global or FLEXCAN_DRV_IRQHandler will hard fault */
flexcan_data_info_t rxInfo;
flexcan_data_info_t txInfo;

uint32_t instance = BOARD_CAN_INSTANCE;
uint32_t numErrors;

uint32_t result;
flexcan_user_config_t flexcanData;
uint32_t canPeClk;

void configure_CAN_MessageBuffs(void);

flexcan_status_t FLEXCAN_DRV_AbortSendingData(uint32_t instance);

/* The following tables are the CAN bit timing parameters that are calculated by using the method
 * outlined in AN1798, section 4.1.
 */
/*
 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
 * a protocol engine clock of 60MHz
 */
#ifdef IS_THIS_NEEDED
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
#endif

/*!
    @brief Initialize the flexcan module, baudrates, message buffers, etc.
    @param none
    @return TRUE on success.
 */
void init_flexcan(void) {

	numErrors = 0;

	flexcanData.max_num_mb = 16;
	flexcanData.num_id_filters = kFlexCanRxFifoIDFilters_8;
	flexcanData.is_rx_fifo_needed = false;
	flexcanData.flexcanMode = kFlexCanNormalMode;

	rxIdentifier = 0x123; // These get overwritten, but they will initially set ID0, & ID8
	txIdentifier = 0x122;

	/* The following tables are the CAN bit timing parameters that are calculated by using the method
	 * outlined in AN1798, section 4.1.
	 */
	/*
	 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
	 * a protocol engine clock of 60MHz
	 */
	flexcan_time_segment_t bitRateTable60Mhz[] = {

			{ 6, 7, 7, 19, 3},  /* 125 kHz */

#ifdef VSCP_ONLY_USES_125KPBS
			{ 6, 7, 7,  9, 3},  /* 250 kHz */
			{ 6, 7, 7,  4, 3},  /* 500 kHz */
			{ 6, 5, 5,  3, 3},  /* 750 kHz */
			{ 6, 5, 5,  2, 3},  /* 1   MHz */
#endif
	};

	/*
	 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
	 * a protocol engine clock of 48MHz
	 */
	flexcan_time_segment_t bitRateTable48Mhz[] = {

			{ 6, 7, 7, 15, 3},  /* 125 kHz */

#ifdef VSCP_ONLY_USES_125KPBS
			{ 6, 7, 7,  7, 3},  /* 250 kHz */
			{ 6, 7, 7,  3, 3},  /* 500 kHz */
			{ 6, 3, 3,  3, 3},  /* 750 kHz */
			{ 6, 3, 3,  2, 3},  /* 1   MHz */
#endif
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
	}
	if (result)
	{
		numErrors++;
		printf("\r\nFLEXCAN set bitrate failed. result: 0x%lx", result);
	}


#ifdef MASK

	FLEXCAN_DRV_SetRxMaskType(instance, kFlexCanRxMaskGlobal);
	//result = FLEXCAN_DRV_SetRxMbGlobalMask(instance, kFlexCanMsgIdStd, 0x123);
	result = FLEXCAN_DRV_SetRxMbGlobalMask(instance, kFlexCanMsgIdStd, 0x000); //kFlexCanMsgIdExt

	if (result)
	{
		numErrors++;
		printf("\r\nFLEXCAN set rx MB global mask. result: 0x%lx", result);
	}

#ifdef CAN0_INDIVIDUAL_MASK

	FLEXCAN_DRV_SetRxMaskType(instance, kFlexCanRxMaskIndividual);

	// Standard ID
	result = FLEXCAN_DRV_SetRxIndividualMask(instance, kFlexCanMsgIdStd, rxMailboxNum, 0x7FF);
	if(result)
	{
		numErrors++;
		printf("\r\nFLEXCAN set rx individual mask with standard ID fail. result: 0x%lx", result);
	}


	// Extern ID

	result = FLEXCAN_DRV_SetRxIndividualMask(instance, kFlexCanMsgIdExt, rxMailboxNum, 0x123);
	if(result)
	{
		numErrors++;
		printf("\r\nFLEXCAN set rx individual mask with standard ID fail. result: 0x%lx", result);
	}
#endif
#endif

	// Init all mailboxes for flexcan
	configure_CAN_MessageBuffs();

	/* Need this line so that state->rx_mb_idx agrees with rxMailboxNum */
	configure_CAN_rxMessageBuff(); // Configure the receive mailbox for flexcan


	/* Attempt to receive a message into rxMb */
	//result = FLEXCAN_DRV_RxMessageBuffer(instance, rxMailboxNum, &rxMb);
}

/*!
    @brief Initialize 16 message buffers, bottom half for RX, top half for TX
    @param none.
 */
void configure_CAN_MessageBuffs(void)
{
	uint8_t i;
	uint32_t result;

	//rxInfo.msg_id_type = kFlexCanMsgIdStd;
	rxInfo.msg_id_type = kFlexCanMsgIdExt;
	rxInfo.data_length = 8;

	txInfo.msg_id_type = kFlexCanMsgIdExt;
	txInfo.data_length = 8;

	for(i=0;i<8;i++){

		/* Configure RX MB fields */

		result = FLEXCAN_DRV_ConfigRxMb(instance, i, &rxInfo, rxIdentifier);
		if (result)
		{
			numErrors++;
			printf("FlexCAN RX MB configuration failed. result: 0x%lx \r\n", result);
		}
	}

	for(i=8;i<16;i++){

		/* Configure TX MB fields */
		result = FLEXCAN_DRV_ConfigTxMb(instance, i, &txInfo, txIdentifier);
		if (result)
		{
			printf("Transmit MB config error. Error Code: 0x%lx \r\n", result);
		}
	}
}

/*!
    @brief Initialize 1 RX message buffer, called by FLEXCANReceiveMessage()
    @param none.
 */
void configure_CAN_rxMessageBuff(void)
{
	uint32_t result;

	rxInfo.msg_id_type = kFlexCanMsgIdExt;
	rxInfo.data_length = 8;

	/* Configure RX MB fields */
	result = FLEXCAN_DRV_ConfigRxMb(instance, rxMailboxNum, &rxInfo, rxIdentifier);
	if (result)
	{
		numErrors++;
		printf("FlexCAN RX MB configuration failed. result: 0x%lx \r\n", result);
	}
}


/*!
    @brief Send a VSCP frame
    @param unsigned long id -
    @return TRUE on success.
 */

#define IS_CAN_SEND_BLOCKING
int FLEXCANSendMessage(uint32_t id, uint8_t dlc, uint8_t *pdata, FLEXCAN_TX_MSG_FLAGS msgFlags)
{
#ifdef DO_PRINT
	uint32_t result, i;
#endif

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
	txInfo.data_length = dlc;

	result = FLEXCAN_DRV_ConfigTxMb(instance, txMailboxNum, &txInfo, txIdentifier);

	if (result)
	{
		printf("Transmit MB config error. Error Code: 0x%lx \r\n", result);
	}
	else
	{
#ifdef IS_CAN_SEND_BLOCKING
		result = FLEXCAN_DRV_SendBlocking(instance, txMailboxNum, &txInfo, txIdentifier, pdata, 1000); /* 1000ms timeout or can use OSA_WAIT_FOREVER */
#else
		result = FLEXCAN_DRV_Send(instance, txMailboxNum, &txInfo, txIdentifier, pdata);
#endif
		if (result)
		{
			FLEXCAN_DRV_AbortSendingData(BOARD_CAN_INSTANCE);
			numErrors++;
#ifdef DO_PRINT_DEBUG
			PRINTF("Transmit send configuration failed. result: 0x%lx \r\n", result);
#endif
		}

		else
		{
#ifdef DO_PRINT
			printf("Data transmitted: \r\n");
			for (i = 0; i < txInfo.data_length; i++ )
			{
				printf("%02x ", pdata[i]);
			}
			printf("\r\n");
#endif
		}
	}
	return result;
}

/*!
    @brief Get a VSCP frame.  Use this function to check for
    	   full receive buffer and extract received data into local buffers.
    @param pid - Pointer to buffer that will be populated with receive ID.
    @param pdlc - Pointer to buffer that will be populated with count of bytes copied in data buffer.
    @param pdata - Pointer to buffer that will be populated with data if there is any
    @param msgFlags - type of can frame
    @return TRUE on success.
*/
#define BUSY_WAIT
flexcan_code_t FLEXCANReceiveMessage(uint32_t *pid, uint32_t *pdlc, uint32_t *pdata, FLEXCAN_RX_MSG_FLAGS *msgFlags)
{
	uint32_t result;
	flexcan_msgbuff_t rxMb;

	/* Attempt to receive a message into rxMb */
	result = FLEXCAN_DRV_RxMessageBuffer(instance, rxMailboxNum, &rxMb);

	/* The FlexCAN uses only an interrupt driven process, but we busy-wait here by,
	 * polling the CODE field of the message buffer code and status word */

#ifdef TEST_CAN
	if(VSCP_STATE_ACTIVE == vscp_node_state)
	{
		while(FLEXCAN_DRV_GetReceiveStatus(instance) != kStatus_FLEXCAN_Success) {}
	}
	else
	{
		if(FLEXCAN_DRV_GetReceiveStatus(instance) != kStatus_FLEXCAN_Success)
		{
				return FLEXCAN_FAIL;
		}
	}
#endif

	if(FLEXCAN_DRV_GetReceiveStatus(instance) != kStatus_FLEXCAN_Success)
	{
		return FLEXCAN_FAIL;
	}

	/*
	while(FLEXCAN_DRV_GetReceiveStatus(instance) != kStatus_FLEXCAN_Success) {

		//can implement a delay in here so that we return false after waiting too long
		delay++;
		if(delay > 1000) {
			delay = 0;
			return FLEXCAN_FAIL;
		}
	}*/

	if (result)
	{
		numErrors++;
#ifdef DO_PRINT
		printf("\r\nFLEXCAN receive configuration failed. result: 0x%lx", result);
#endif
	}

	else
	{	/* Successfully received something! Begin extracting information from message
	     * buffer which is defined on pg. 1400 of the K64 Sub-Family Ref. Manual */

		*pid = rxMb.msgId;				  // The message ID occupies bits 0-28 (EXT) or 18-28 (STD)

		*pdlc = ((rxMb.cs) >> 16) & 0xF;  // The data length field (DLC) occupies bits 16 - 19

		*msgFlags = ((rxMb.cs) >> 21) & 0x1; // The ID Extended Bit field (IDE) is bit 21

#ifdef DO_PRINT
		printf("ID: 0x%lx, DLC=%lu, mb_idx=%lu\n",*pid, *pdlc, rxMailboxNum);
		printf("RX MB data: 0x");
#endif
		for (result = 0; result < *pdlc; result++){
#ifdef DO_PRINT
		printf("%02x ", rxMb.data[result]);
#endif
		pdata[result] = rxMb.data[result];
		}
	}
	return FLEXCAN_SUCCESS;
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
		printf("\r\nDLC=%lu, mb_idx=%lu", temp, rxMailboxNum);
		printf("\r\nRX MB data: 0x");

		for (result = 0; result < temp; result++)
			printf("%02x ", rxMb.data[result]);

		printf("\r\nID: 0x%lx", rxMb.msgId);
	}

}

