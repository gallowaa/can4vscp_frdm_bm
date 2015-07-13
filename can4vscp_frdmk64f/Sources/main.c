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

// ***************************************************************************
// 								Definitions
// ***************************************************************************
#define FlashEverytime
#define regular_print //to use printf
#define LPTMR_INSTANCE      0U
#define BOARD_PIT_INSTANCE  0U
#define DO_VSCP

// ***************************************************************************
// 								Prototypes
// ***************************************************************************

void hardware_init();

/*! This is only here because init_flash is called from main directly.*/
/*! The other flash_al.c functions are called from the external vscp functions in external_vscp_func.c */
void init_flash(void);
void init_flexcan(void);
void init_spi();

// ***************************************************************************
// 							Global Variables
// ***************************************************************************

//extern volatile uint16_t vscp_timer; //can change to uint32_t?
//extern volatile uint16_t vscp_configtimer; // configuration timer

const uint8_t vscp_deviceURL[] = "www.samplewebsite/frdm.xml";

volatile unsigned long measurement_clock; // Clock for measurements

uint8_t sendTimer;  // Timer for CAN send
uint8_t seconds;    // counter for seconds
uint8_t minutes;    // counter for minutes
uint8_t hours;      // Counter for hours

volatile uint32_t d=0;
volatile uint32_t pitCounter=0;
volatile bool pitIsrFlag[2] = {false};

lptmr_state_t lptmrState; 	 /*! Use LPTMR in Time Counter mode */

fxos_handler_t i2cDevice;

/*!
 * @brief takes place of init() function in vscp paris implementation
 *
 */

void hardware_init() {

	/* enable clock for PORTs */
	CLOCK_SYS_EnablePortClock(PORTA_IDX);
	CLOCK_SYS_EnablePortClock(PORTB_IDX);
	CLOCK_SYS_EnablePortClock(PORTD_IDX);
	CLOCK_SYS_EnablePortClock(PORTE_IDX);

	/* Init board clock */
	BOARD_ClockInit();
	dbg_uart_init();

	/* OSA_Init();
	 * Call OSA_Init to use built in time utilities.
	 * This is needed for delay functions, but OSA init sets up
	 * the lptmr with a different clock source than is desired for our 1ms clock.
	 * Going to skip this for first pass. Might have to enable a new timer (PIT, PDB?) to maintain
	 * the 1 ms clock while using the OSA.
	 */

	configure_spi_pins(0); 	// Configure SPI pins for talking to EEPROM w/ MAC address
	configure_i2c_pins(0);  // Configure IIC pins for accelerometer
	configure_can_pins(0);  // Configure CAN pins

	init_spi();				// For EEPROM
	init_flexcan();			// For vscp events

	// Initialize the eCompass.
	i2cDevice.i2cInstance = BOARD_I2C_COMM_INSTANCE;
	FXOS_Init(&i2cDevice, NULL);

	STATUS_LED_EN; //LED1_EN;
	CLOCK_PIN_EN;  //GPIO_DRV_OutputPinInit(&gpioPins[0]);  /* scope this pin to test clk */
	INIT_BTN_EN;   //GPIO_DRV_InputPinInit(&switchPins[0]); /* init sw2 as input */

	// Enable a gpio for taking STB low
	CAN0_STB_EN;
	CAN0_STB_LO;
}

/*!
 * @brief PIT initialization done here. PIT is used as vscp 1ms clock.
 *
 */
static void init_pit(pit_user_config_t pit_configuration) {

	// Init pit module and enable run in debug
	PIT_DRV_Init(BOARD_PIT_INSTANCE, false);

	// Initialize PIT timer instance for channel 0 and 1
	PIT_DRV_InitChannel(BOARD_PIT_INSTANCE, 0, &pit_configuration);
	//PIT_DRV_InitChannel(BOARD_PIT_INSTANCE, 1, &chn1Confg);

	// Start channel 0
	PRINTF("\n\rStarting channel No.0 ...");
	PIT_DRV_StartTimer(BOARD_PIT_INSTANCE, 0);
}


// ***************************************************************************
// Main() - Main Routine
// ***************************************************************************
int main(void) {

	uint32_t result;               /*! Return code from each SSD function */
	uint32_t destAdrss;            /*! Address of the target location     */
	uint32_t i, failAddr;          /*! comment */
	uint32_t FlashValue;		   /*! comment */
	uint32_t currentCounter = 0;   /*! do things according to 1ms lptmr */

	uint8_t c1,c2;
	unsigned char c;
	unsigned char *dst;

	// Structure of initialize PIT channel No.0
	pit_user_config_t chn0Confg = {
			.isInterruptEnabled = true,
			.periodUs = 1000u
	};

	/*
	uint8_t *xAngle;
	uint8_t *yAngle;*/
	accel_data_t accelData;
	uint8_t y;

	//int16_t xData, yData;
	//int16_t xAngle, yAngle;
	//uint32_t ftmModulo;

	vscp_node_state = VSCP_STATE_ACTIVE;

	// Init mcu and peripherals
	hardware_init();
	init_pit(chn0Confg);

	OSA_Init();

	//can take this out once vscp fully implemented
	vscp_initledfunc = VSCP_LED_BLINK1; //0x02

	// Check VSCP persistent storage and
	// restore if needed

#ifdef DO_VSCP
	if( !vscp_check_pstorage() ){

		init_flash();
		// Spoiled or not initialized - reinitialize
		init_app_eeprom();
		init_app_ram();     // Needed because some ram positions
							// are initialized from EEPROM
	}
	// Initialize vscp
	vscp_init(); /* defined in vscp_firmware, line 118 */

#endif

	printf("Hello!\r\n");

	while(1)
	{
#ifdef DO_VSCP
		vscp_imsg.flags = 0;
		vscp_getEvent(); //fetch one vscp event -> vscp_imsg struct
#endif


		/* do this every 1ms tick */
		if(currentCounter != pitCounter)
		{
			currentCounter = pitCounter;

			doWork();
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

void BOARD_SW_IRQ_HANDLER_NOTUSED(void){

	GPIO_DRV_ClearPinIntFlag(BOARD_SW_GPIO_3);
	printf("I don't do anything! \r\n");
}





