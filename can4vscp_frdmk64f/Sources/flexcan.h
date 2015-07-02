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


#ifndef SOURCES_FLEXCAN_H_
#define SOURCES_FLEXCAN_H_

#include <stdio.h>
#include "main.h"
#include "fsl_flexcan_driver.h"

/* NOTE: The CAN Message flags are implemented differently due to the Flexcan api being higher level
 * than the or'ing of bits in the paris module. The KSDK uses structs to store configuration data such
 * as the ID type. Logic is implemented in flexcan.c to mimic the behaviour of the flags in
 * can4vscp_paris/ECAN.h */


/*! @brief FlexCAN Message Buffer ID type*/
typedef enum flexcan_tx_msg {
	FLEXCAN_TX_STD_FRAME,        /*!< Standard ID*/
	FLEXCAN_TX_XTD_FRAME         /*!< Extended ID*/
} FLEXCAN_TX_MSG_FLAGS;

/*! @brief FlexCAN Message Buffer ID type*/
typedef enum flexcan_rx_msg {
	FLEXCAN_RX_RTR_FRAME,        /*!< Standard ID*/
	FLEXCAN_RX_XTD_FRAME         /*!< Extended ID*/
} FLEXCAN_RX_MSG_FLAGS;

/*! @brief FlexCAN Message Buffer ID type*/
typedef enum flexcan_status {
	FLEXCAN_FAIL,
	FLEXCAN_SUCCESS
} flexcan_code_t;

void flexcan_init(void);
void receive_mb_config(void);
void transfer_mb_loopback(void);
int FLEXCANSendMessage(uint32_t id, uint8_t dlc, uint8_t *pdata, FLEXCAN_TX_MSG_FLAGS msgFlags);
flexcan_code_t FLEXCANReceiveMessage(uint32_t *pid, uint32_t *pdlc, uint32_t *pdata, FLEXCAN_RX_MSG_FLAGS *msgFlags);


#endif /* SOURCES_FLEXCAN_H_ */
