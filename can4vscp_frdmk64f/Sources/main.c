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
//#include "fsl_interrupt_manager.h"

// ***************************************************************************
// 								Definitions
// ***************************************************************************

//#define VSCP_REG_IN_FLASH  // Un-comment this to use FLASH instead of EEPROM for the VSCP registers
//#define DO_PRINT			 // Un-comment this to print out debug info


#define LPTMR_INSTANCE      (0U)
#define BOARD_PIT_INSTANCE  (0U)
#define ADC_0				(0U)

// ***************************************************************************
// 							Global Variables
// ***************************************************************************

//extern volatile uint16_t vscp_timer; //can change to uint32_t?
//extern volatile uint16_t vscp_configtimer; // configuration timer

/* VSCP core globals */
const uint8_t vscp_deviceURL[] = "website/frdm.xml";
volatile uint32_t measurement_clock; // Clock for measurements
uint8_t sendTimer;  // Timer for CAN send
uint8_t seconds;    // counter for seconds
uint8_t minutes;    // counter for minutes
uint8_t hours;      // Counter for hours

/* VSCP app globals */
uint8_t current_temp;
accel_data_t accelData;
//uint8_t current_xAngle;
//uint8_t current_yAngle;
uint8_t temp0_low_alarm;
uint8_t temp0_high_alarm;
uint8_t accel0_high_alarm;
uint8_t seconds_temp;        				// timer for temp event
uint8_t seconds_accel;        				// timer for accel event
volatile uint32_t timeout_clock;        	// Clock used for timeouts
volatile uint32_t d=0;


lptmr_state_t lptmrState; 	 /*! Use LPTMR in Time Counter mode */
fxos_handler_t i2cDevice;
extern SIM_Type * gSimBase[];

// Structure of initialize PIT channel No.0
pit_user_config_t channelConfig0 = {
		.isInterruptEnabled = true,
		.periodUs = 1000u			    /*! 1ms - vscp clk*/
};

pit_user_config_t channelConfig1 = {
		.isInterruptEnabled = false,	/*! channel is set as hw trigger for adc via
			 	 	 	 	 	 	 	 	    System Integration Module (SIM) 		*/
		.periodUs = 500000U,			/*! 500ms - adc measurement period 			*/
};


void hardware_init();			  /*! defined in main.c     */

void init_spi();				  /*! defined in spi.c      */
void init_flash(void);			  /*! defined in flash_al.c */
void init_flexcan(void);		  /*! defined in flexcan.c  */
void init_adc(uint32_t instance); /*! defined in adc16.c    */
void calibrateParams(void);		  /*! defined in adc16.c    */
void init_pit();

/* Test */
void test_adc(void);
void printAngle(fxos_handler_t i2cModule);
void test_spi_generic();


/*!
 * @brief takes place of init() function in vscp paris implementation
 *
 */

void hardware_init() {

	/* enable clock for PORTs */
	CLOCK_SYS_EnablePortClock(PORTA_IDX); /*! */
	CLOCK_SYS_EnablePortClock(PORTB_IDX); /*! The CAN pins are on port B */
	CLOCK_SYS_EnablePortClock(PORTC_IDX); /*! */
	CLOCK_SYS_EnablePortClock(PORTD_IDX); /*! The SPI pins are on port D */
	CLOCK_SYS_EnablePortClock(PORTE_IDX); /*! The I2C pins are on port E */

	/* Init board clock */
	BOARD_ClockInit();
	dbg_uart_init();

	configure_spi_pins(0); 	// Configure SPI pins for talking to EEPROM w/ MAC address
	configure_i2c_pins(0);  // Configure IIC pins for accelerometer
	configure_can_pins(0);  // Configure CAN pins

	OSA_Init();             //FXOS_Init seems to depend on this, so does OSA_TimeDelay(ms)

	init_spi();				// For EEPROM
	init_flexcan();			// For vscp events

	// Initialize the eCompass.
	i2cDevice.i2cInstance = BOARD_I2C_COMM_INSTANCE;
	FXOS_Init(&i2cDevice, NULL);

#ifdef VSCP_REG_IN_FLASH
	init_flash();
#endif

	STATUS_LED_EN; /* LED1_EN */
	CLOCK_PIN_EN;  /* scope this pin to test the 1 ms clock pulse width */
	INIT_BTN_EN;   /* init sw2 as input */

	// Enable a gpio for taking the STB pin on the CAN PHY low
	CAN0_STB_EN;
	CAN0_STB_LO;

	// Calibrate param Temperature sensor
	calibrateParams(); /* <- taken from adc_low_power_frdmk64f demo */

	init_adc(ADC_0);

	//init_pit(channelConfig0, channelConfig1);
	init_pit();
}

/*!
 * @brief PIT initialization done here. PIT is used as vscp 1ms clock.
 *
 */

void init_pit()
{
	// Init pit module and enable run in debug
	PIT_DRV_Init(BOARD_PIT_INSTANCE, true);

	PIT_DRV_InitChannel(BOARD_PIT_INSTANCE, 0, &channelConfig0);
	PIT_DRV_InitChannel(BOARD_PIT_INSTANCE, 1, &channelConfig1);

	// Start channel 0
#ifdef DO_PRINT
	PRINTF("Starting channel No.0: VSCP 1ms clock\r\n");
#endif
	PIT_DRV_StartTimer(BOARD_PIT_INSTANCE, 0);

	// Start channel 1
#ifdef DO_PRINT
	PRINTF("Starting channel No.1: ADC16 500ms clock\r\n");
#endif
	PIT_DRV_StartTimer(BOARD_PIT_INSTANCE, 1);

	// Configure SIM for ADC hw trigger source selection
	SIM_HAL_SetAdcAlternativeTriggerCmd(gSimBase[0], ADC_0, true);
	SIM_HAL_SetAdcPreTriggerMode(gSimBase[0], ADC_0, kSimAdcPretrgselA);
	SIM_HAL_SetAdcTriggerMode(gSimBase[0], ADC_0, kSimAdcTrgSelPit1);
}

//void wdog_isr(void);


extern uint32_t numErrors;

// ***************************************************************************
// Main() - Main Routine
// ***************************************************************************
int main(void) {

	// Init mcu and peripherals
	hardware_init();

	//can take this out once vscp fully implemented
	vscp_initledfunc = VSCP_LED_BLINK1; //0x02

	// Check VSCP persistent storage and
	// restore if needed

	if( !vscp_check_pstorage() ) {

#ifdef USE_EEPROM
		spi_eeprom_guid_init();

		// Spoiled or not initialized - reinitialize
		init_app_eeprom();
#endif
		init_app_ram();     // Needed because some ram positions
							// are initialized from EEPROM

	}

	// Initialize vscp
	vscp_init();


	while(1)
	{

		vscp_imsg.flags = 0;
		vscp_getEvent(); 		// fetch one vscp event -> vscp_imsg struct

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
				// doDM();
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

			PRINTF("Transmit send errors: %d\r", numErrors);

			measurement_clock = 0;

			// Do VSCP one second jobs
			vscp_doOneSecondWork();
			seconds++;
			// sendTimer++; // sendTimer should be incremented in the 1ms interrupt


			// Temperature report timers are only updated if in active
			// state guid_reset
			if ( VSCP_STATE_ACTIVE == vscp_node_state ) {

				// Do VSCP one second jobs

				/* temperature is done here, it will check seconds_temp variable
				 * so that it sends the event as per the REPORT_INTERVAL specified */
				doApplicationOneSecondWork();
				seconds_temp++; // Temperature report timers are only updated if in active state
				seconds_accel++;

			}
			doWork();
		}

		// Timekeeping - Can replace this with RTC which defines structs for keeping track of this automatically
		if ( seconds > 59 ) {

			seconds = 0;
			minutes++;

			if ( minutes > 59 ) {
				minutes = 0;
				hours++;
			}

			if ( hours > 23 ) hours = 0;

		}
	}

	vscp_handleProbeState(); //just a test

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





