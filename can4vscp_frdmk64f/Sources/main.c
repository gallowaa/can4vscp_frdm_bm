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


#include "main.h"
#include "flexcan.h"

//#define USE_OLD_MAIN

// ***************************************************************************
// 								Definitions
// ***************************************************************************
#define FlashEverytime
#define regular_print //to use printf
#define LPTMR_INSTANCE     0U

// ***************************************************************************
// 								Prototypes
// ***************************************************************************

/*! This is only here because init_flash is called from main directly.*/
/*! The other flash_al.c functions are called from the external vscp functions in external_vscp_func.c */
void init_flash(void);
void read_spi_eeprom(void);
void flexcan_init(void);
/* these are only here for testing functions in main and will be removed later*/
void receive_mb_config(void);
void transfer_mb_loopback(void);
int FLEXCANSendMessage(uint32_t id, uint8_t dlc, uint8_t *pdata, FLEXCAN_TX_MSG_FLAGS msgFlags);
flexcan_code_t FLEXCANReceiveMessage(uint32_t *pid, uint32_t *pdlc, uint32_t *pdata, FLEXCAN_RX_MSG_FLAGS *msgFlags);
int8_t getCANFrame(uint32_t *pid, uint8_t *pdlc, uint8_t *pdata);

// ***************************************************************************
// 							Global Variables for flexcan
// ***************************************************************************

//#define can
//#ifdef can
//#endif


// ***************************************************************************
// 							Global Variables
// ***************************************************************************

//extern volatile uint16_t vscp_timer; //can change to uint32_t?
//extern volatile uint16_t vscp_configtimer; // configuration timer

//extern const uint32_t VSCP_FLASH_BASE;
const uint8_t vscp_deviceURL[] = "www.samplewebsite/frdm.xml";

volatile unsigned long measurement_clock; // Clock for measurements

uint8_t sendTimer;  // Timer for CAN send
uint8_t seconds;    // counter for seconds
uint8_t minutes;    // counter for minutes
uint8_t hours;      // Counter for hours


volatile uint32_t d=0;
volatile uint32_t lptmrCounter=0;

lptmr_state_t lptmrState; 	 /*! Use LPTMR in Time Counter mode */

// Configure LPTMR.
lptmr_user_config_t lptmrUserConfig =
{
		.timerMode            = kLptmrTimerModeTimeCounter, /*! Use LPTMR in Time Counter mode */
		.freeRunningEnable    = false, /*! When hit compare value, set counter back to zero */
		.prescalerEnable      = false, /*! bypass prescaler */
		.prescalerClockSource = kClockLptmrSrcMcgIrClk, /* use MCGIR clock, not 1kHz Low Power Clock */
		.isInterruptEnabled   = true
};




// ***************************************************************************
// 						Subroutines & ISRs not related to VSCP
// ***************************************************************************
/*!
 * @brief LPTMR interrupt callback, implements vscp 1ms timer
 */
void lptmr_isr_callback(void)
{
	/** BE careful! Any code running here must execute in less than 1ms. **/
	//for(d=0;d<7500;d++); /* this is about the max delay it can handle. */

	CLOCK_PIN_TOGGLE; 	/* Used to verify 1ms clock. Period/2 should = 1ms. */

	// critical vscp
	vscp_timer++;
	vscp_configtimer++;
	lptmrCounter++;
	// measurement_clock++; /* optional */

#ifdef TIMER_HAS_LONG_PERIOD
	if(VSCP_LED_BLINK1 == vscp_initledfunc)
		STATUS_LED_TOGGLE;
#else

	vscp_statuscnt++;

	if( (vscp_statuscnt > 100) && (vscp_initledfunc == VSCP_LED_BLINK1) ){
			STATUS_LED_TOGGLE; // blink the vscp status led
			vscp_statuscnt = 0;
	}
#endif

	else if (VSCP_LED_ON == vscp_initledfunc){
		STATUS_LED_ON;
	}
}

/*!
 * @brief takes place of init() function in vscp paris implementation
 *
 */

void hardware_init(void) {

	/* enable clock for PORTs */
	CLOCK_SYS_EnablePortClock(PORTA_IDX);
	CLOCK_SYS_EnablePortClock(PORTB_IDX);
	CLOCK_SYS_EnablePortClock(PORTD_IDX);

	/* Init board clock */
	BOARD_ClockInit();
	dbg_uart_init();

	configure_spi_pins(0); 	// Configure SPI pins for talking to EEPROM w/ MAC address
	configure_can_pins(0);  // Configure CAN pins
}


/*!
 * @brief all uC initialization done here
 *
 */
void init() {

	// Init basic hardware.
	hardware_init();

	STATUS_LED_EN; //LED1_EN;

	CLOCK_PIN_EN; //GPIO_DRV_OutputPinInit(&gpioPins[0]); /* scope this pin to test clk */
	SPI0_CS_EN; //GPIO_DRV_OutputPinInit(&gpioPins[1]); /* this is the CS for SPI0 */

	GPIO_DRV_InputPinInit(&switchPins[0]); /* init sw2 as input */

	//Initialize LPTMR
	LPTMR_DRV_Init(LPTMR_INSTANCE, &lptmrState, &lptmrUserConfig);

	// This is our vscp timer
	//LPTMR_DRV_SetTimerPeriodUs(LPTMR_INSTANCE,1000); /* Set the timer period for 1ms */
	LPTMR_DRV_SetTimerPeriodUs(LPTMR_INSTANCE,300000); /* Set the timer period for 300ms */

	// Specify the callback function when a LPTMR interrupt occurs
	LPTMR_DRV_InstallCallback(LPTMR_INSTANCE,lptmr_isr_callback);

	// Start counting
	LPTMR_DRV_Start(LPTMR_INSTANCE);
}

//#ifdef can
//#endif



// ***************************************************************************
// Main() - Main Routine
// ***************************************************************************
int main(void) {

	uint32_t result;               /*! Return code from each SSD function */
	uint32_t destAdrss;            /*! Address of the target location */
	uint32_t i, failAddr;          /*! comment */
	uint32_t FlashValue;		   /*! comment */
	uint32_t currentCounter = 0;   /*! comment */

	unsigned char c;
	unsigned char *dst;

	FLEXCAN_RX_MSG_FLAGS flags;

	/*! this is just for testing the sendCANMessage function */

	int8_t ret;
	uint8_t pdata8[8];
	uint32_t pdata32[8];

	uint32_t id = 0x123; //29-bit only for extended
	uint32_t pid = 0xdeadbeef;
	uint8_t dlc = 8;
	uint32_t pdlc;
	uint8_t pdlc8;

	for (i = 0; i < dlc; i++)
	{
		pdata8[i] = 10 + i;
	}

	// initialize the mcu, switched, 1ms clock, etc
	init();

	flexcan_init();

	// Config receive mailbox
	receive_mb_config();

	// Enable a gpio for taking STB low
	CAN0_STB_EN;
	CAN0_STB_LO;

	//can take this out once vscp fully implemented
	vscp_initledfunc = VSCP_LED_BLINK1; //0x02

	// Check VSCP persistent storage and
	// restore if needed
	if( !vscp_check_pstorage() ){

		//init_flash();
		/*
		// Spoiled or not initialized - reinitialize
		init_app_eeprom();
		init_app_ram();     // Needed because some ram positions
		// are initialized from EEPROM
		 */
	}

	// Initialize vscp
	//vscp_init(); /* defined in vscp_firmware, line 118 */

	while(1)
	{

		vscp_imsg.flags = 0;
		//vscp_getEvent(); //fetch one vscp event -> vscp_imsg struct

		//read_spi_eeprom();
		//send_data();

		/* do this every 1ms tick */

		if(currentCounter != lptmrCounter)
		{
			printf("Sending message \r\n");
			//ret =  FLEXCANSendMessage(id, dlc, pdata8, FLEXCAN_TX_STD_FRAME);
			ret =  FLEXCANSendMessage(pid, dlc, pdata8, FLEXCAN_TX_XTD_FRAME); //extended deadbeef msg


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
			printf("nRX MB data: 0x\r\n");
			for (i = 0; i < pdlc8; i++){
				printf("%02x ", pdata8[i]);

			}
			printf("\r\n");

			//test_vscp_externals();
			currentCounter = lptmrCounter;
		}


#ifdef DO_VSCP

		switch ( vscp_node_state ) {

		case VSCP_STATE_STARTUP: // Cold/warm reset

			// Get nickname from EEPROM
			if (VSCP_ADDRESS_FREE == vscp_nickname) {
				// new on segment need a nickname
				vscp_node_state = VSCP_STATE_INIT;
			} else {
				// been here before - go on
				vscp_node_state = VSCP_STATE_ACTIVE;
				vscp_goActiveState();
			}
			break;

		case VSCP_STATE_INIT: // Assigning nickname
			vscp_handleProbeState();
			break;

		case VSCP_STATE_PREACTIVE:  // Waiting for host initialisation
			vscp_goActiveState();
			break;

		case VSCP_STATE_ACTIVE:     // The normal state

			// Check for incoming event?
			if (vscp_imsg.flags & VSCP_VALID_MSG) {

				if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.vscp_class  ) {

					// Handle protocol event
					vscp_handleProtocolEvent();

				}

				//doDM();

			}
			break;

		case VSCP_STATE_ERROR: // Everything is *very* *very* bad.
			vscp_error();
			break;

		default: // Should not be here...
			vscp_node_state = VSCP_STATE_STARTUP;
			break;

		}

		// do a measurement if needed
		if ( measurement_clock > 1000 ) {

			measurement_clock = 0;

			sendTimer++;

			// Do VSCP one second jobs
			vscp_doOneSecondWork();

			// Temperature report timers are only updated if in active
			// state guid_reset
			if ( VSCP_STATE_ACTIVE == vscp_node_state ) {

				// Do VSCP one second jobs
				//doApplicationOneSecondWork();

			}

		}

		// Timekeeping
		if ( seconds > 59 ) {

			seconds = 0;
			minutes++;

			if ( minutes > 59 ) {
				minutes = 0;
				hours++;
			}

			if ( hours > 23 ) hours = 0;

		}
#endif



	}

	//vscp_handleProbeState(); //just a test

	/* Never leave main */
	return 0;
//#endif
}


/*!
 * @brief Interrupt service function for switch.
 *
 * This function registers init button press
 */
void BOARD_SW_IRQ_HANDLER(void)
{
	// Clear external interrupt flag.
	GPIO_DRV_ClearPinIntFlag(BOARD_SW_GPIO);

	vscp_initledfunc = VSCP_LED_ON;

	// If we haven't already been initialized
	if(VSCP_STATE_INIT != vscp_node_state) {

		//test_vscp_externals();
		//vscp_FLASHFlush();

		//vscp_init();
	}
}





