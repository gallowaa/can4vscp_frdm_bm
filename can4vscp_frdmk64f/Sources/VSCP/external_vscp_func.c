/*
 * external_vscp_func.c
 *
 *  Created on: Jun 8, 2015
 *      Author: Angus
 */

#include "main.h"
#include "flash_al.h"
#include "spi.h"
#include "version.h"
#include "flexcan.h"
#include "temperature.h"

#define CAN_SUCCESS 0

extern fxos_handler_t i2cDevice;
uint8_t buffer[BUFFER_SIZE_BYTE]; /*! Not used */

/* VSCP core globals */
extern volatile unsigned long measurement_clock; // Clock for measurements
extern uint8_t sendTimer;  // Timer for CAN send
extern uint8_t seconds;    // counter for seconds
extern uint8_t minutes;    // counter for minutes
extern uint8_t hours;      // Counter for hours

/* VSCP app globals */
extern uint8_t current_temp;
extern uint8_t current_xAngle;
extern uint8_t current_yAngle;
extern uint8_t temp0_low_alarm;
extern uint8_t temp0_high_alarm;
extern uint8_t accel0_high_alarm;
extern uint8_t seconds_temp;        // timer for temp event
extern uint8_t current_temp;


/* needed to prevent error: assigning int to accel_data_t */
accel_data_t getAngle(fxos_handler_t i2cModule);


/* These functions are deliberately not prototyped by vscp_firmware.h,
 * however due to the flexibility of the KSDK the same interface as-is used
 * by the can4vscp_paris module was maintained */

int8_t sendCANFrame(uint32_t id, uint8_t dlc, uint8_t *pdata);
int8_t getCANFrame(uint32_t *pid, uint8_t *pdlc, uint8_t *pdata);



////////////////////////////////////////////////////////////////////////////////
//
//			VSCP Required Functions - see vscp_firmware.h @ line 457
//
////////////////////////////////////////////////////////////////////////////////

/*!
    Get a VSCP frame
    @param pvscpclass Pointer to variable that will get VSCP class.
    @param pvscptype Ponter to variable which will get VSCP type.
    @param pNodeId Pointer to variable which will get nodeid.
    @param pPriority Pointer to variable which will get priority (0-7).
    @param pSize Pointer to variable that will get data size.
    @param pData pinter to array that will get event data.
    @return TRUE on success.
 */
int8_t getVSCPFrame(uint16_t *pvscpclass,
        uint8_t *pvscptype,
        uint8_t *pNodeId,
        uint8_t *pPriority,
        uint8_t *pSize,
        uint8_t *pData)
{
	uint32_t id;

	if ( !getCANFrame(&id, pSize, pData) ) {
		return FALSE;
	}

	*pNodeId = id & 0x0ff;
	*pvscptype = (id >> 8) & 0xff;
	*pvscpclass = (id >> 16) & 0x1ff;
	*pPriority = (uint16_t) (0x07 & (id >> 26));

	return TRUE;

}

/*!
    Send a VSCP frame
    @param vscpclass VSCP class for event.
    @param vscptype VSCP type for event.
    @param nodeid Nodeid for originating node.
    @param priority Priority for event.
    @param size Size of data portion.
    @param pData Pointer to event data.
    @return TRUE on success.
 */
int8_t sendVSCPFrame( uint16_t vscpclass,
                        uint8_t vscptype,
                        uint8_t nodeid,
                        uint8_t priority,
                        uint8_t size,
                        uint8_t *pData )
{
	uint32_t id = ( (uint32_t)priority << 26 ) |
			( (uint32_t)vscpclass << 16 ) |
			( (uint32_t)vscptype << 8 ) |
			nodeid; // node address (our address)

	if ( !sendCANFrame( id, size, pData ) ) {
		return FALSE;
	}

	return TRUE;

}
int8_t sendCANFrame(uint32_t id, uint8_t dlc, uint8_t *pdata)
{
	uint8_t rv = FALSE;
	sendTimer = 0;

	while ( sendTimer < 1 ) {
		//if ( ECANSendMessage( id, pdata, dlc, ECAN_TX_XTD_FRAME ) ) {
		if( CAN_SUCCESS == FLEXCANSendMessage(id, dlc, pdata, FLEXCAN_TX_XTD_FRAME) ) {
			rv = TRUE;
			break;
		}
	}

	vscp_omsg.flags = 0;

	return rv;

}

int8_t getCANFrame(uint32_t *pid, uint8_t *pdlc, uint8_t *pdata)
{
    FLEXCAN_RX_MSG_FLAGS flags;
    int i;

    uint32_t pdlc_32;
    uint32_t pdata_32[8]; // For bit compatibility w/ FLEXCANReceive func


    // Don't read in new event if there already is an event
    // in the input buffer
    if (vscp_imsg.flags & VSCP_VALID_MSG) return FALSE;

    /*
     * pdlc & pdata must stay as 32-bit in FLEXCANReceive.
     * Would be nice to keep getCANFrame interface the same as Ake's implementation
     */

    if ( FLEXCANReceiveMessage( pid, &pdlc_32, pdata_32, &flags) ) {

        // RTR not interesting
        if (flags & FLEXCAN_RX_RTR_FRAME) return FALSE;

        // Must be extended frame
        if (!(flags & FLEXCAN_RX_XTD_FRAME)) return FALSE;

        /*printf("ID: 0x%lx, DLC=%lu \r\n",*pid, pdlc_32);
        printf("RX MB data: 0x");*/

        for (i = 0; i < pdlc_32; i++) {
        	//printf("%02x ", pdata_32[i]);
        	pdata[i] = pdata_32[i];
        }

        // convert all the 32-bit stuff to 8-bit, pdlc_32 is really only 4-bits so this is safe.
        *pdlc = (uint8_t) pdlc_32;

        return TRUE;

    }

    return FALSE;
}

/*!
    The following methods/callbacks must be defined
    in the application and should return firmware version
    information
 */

/*!
 *  Get Major version number for this hardware module
 */
uint8_t vscp_getMajorVersion( void )
{
	return FIRMWARE_MAJOR_VERSION;
}

/*!
 *  Get Minor version number for this hardware module
 */
uint8_t vscp_getMinorVersion( void )
{
	return FIRMWARE_MINOR_VERSION;
}

/*!
 *  Get SubMinor version number for this hardware module
 */
uint8_t vscp_getSubMinorVersion( void )
{
	return FIRMWARE_SUB_MINOR_VERSION;

}

/*!
    Get GUID from permanent storage
 */
uint8_t vscp_getGUID(uint8_t idx)
{
	return spi_eeprom_read(VSCP_EEPROM_REG_GUID + idx);
}

// Only if write to protected
// locations is enabled
//#ifdef ENABLE_WRITE_2PROTECTED_LOCATIONS
void vscp_setGUID(uint8_t idx, uint8_t data)
{
	if (idx > 15) return;
	spi_eeprom_write(VSCP_EEPROM_REG_GUID + idx, data);		// writeFLASH(VSCP_FLASH_REG_GUID + idx, data);
}
//#endif

/*!
    User ID 0 idx=0
    User ID 1 idx=1
    User ID 2 idx=2
    User ID 3 idx=3
 */
uint8_t vscp_getUserID(uint8_t idx)
{
	return readFLASH( VSCP_FLASH_REG_USERID + idx);

}
void vscp_setUserID(uint8_t idx, uint8_t data)
{
	buffer[(VSCP_FLASH_REG_USERID + idx) * 4] = data;

#ifdef FlashEverytime
	writeFLASH(VSCP_FLASH_BASE, buffer);
#endif
}

/*!
    Handle manufacturer id.

    Not that both main and sub id are fetched here
        Manufacturer device ID byte 0 - idx=0
        Manufacturer device ID byte 1 - idx=1
        Manufacturer device ID byte 2 - idx=2
        Manufacturer device ID byte 3 - idx=3
        Manufacturer device sub ID byte 0 - idx=4
        Manufacturer device sub ID byte 1 - idx=5
        Manufacturer device sub ID byte 2 - idx=6
        Manufacturer device sub ID byte 3 - idx=7
 */
uint8_t vscp_getManufacturerId(uint8_t idx)
{
	return readFLASH( VSCP_FLASH_REG_MANUFACTUR_ID0 + idx );

}
#ifdef ENABLE_WRITE_2PROTECTED_LOCATIONS

void vscp_setManufacturerId( uint8_t idx, uint8_t data ) {
    if ( idx > 7 ) return;
    writeFLASH(VSCP_FLASH_REG_MANUFACTUR_ID0 + idx, data);
}
#endif

/*!
    Get boot loader algorithm from permanent storage
 */
uint8_t vscp_getBootLoaderAlgorithm( void )
{
	return 0; /* bootloader not implemented */
}


/*!
    Get buffer size
 */
uint8_t vscp_getBufferSize(void)
{
	return CAN_BUFFER_SIZE; //Standard CAN frame
}


/*!
    tbd
 */
uint8_t vscp_getRegisterPagesUsed( void )
{
    return PAGES; // One page used
}

/*!
    Get URL from device from permanent storage
    index 0-15
 */
uint8_t vscp_getMDF_URL(uint8_t idx)
{
	return vscp_deviceURL[ idx ];
}

/*!
    Fetch nickname from permanent storage
    @return read nickname.
 */
uint8_t vscp_readNicknamePermanent(void)
{
	return readFLASH(VSCP_FLASH_NICKNAME ); //VSCP_EEPROM_NICKNAME
}


/*!
    Write nickname to permanent storage
    @param nickname to write
 */
void vscp_writeNicknamePermanent(uint8_t nickname)
{
	buffer[VSCP_FLASH_NICKNAME * 4] = nickname; //	writeEEPROM( VSCP_FLASH_NICKNAME, nickname );

#ifdef FlashEverytime
	writeFLASH(VSCP_FLASH_BASE, buffer);
#endif

}


/*!
    Fetch segment CRC from permanent storage
 */
uint8_t vscp_getSegmentCRC(void)
{
	return readFLASH( VSCP_FLASH_SEGMENT_CRC );	// return readEEPROM( VSCP_EEPROM_SEGMENT_CRC );

}


/*!
    Write segment CRC to permanent storage
 */
void vscp_setSegmentCRC(uint8_t crc)
{
	buffer[VSCP_FLASH_SEGMENT_CRC * 4] = crc; //writeEEPROM (VSCP_EEPROM_SEGMENT_CRC, crc);

#ifdef FlashEverytime
	writeFLASH(VSCP_FLASH_BASE, buffer);
#endif
}


/*!
    Fetch control byte from permanent storage
 */
uint8_t vscp_getControlByte(void)
{
 // readEEPROM(VSCP_EEPROM_CONTROL);
	return readFLASH(VSCP_FLASH_CONTROL );
}


/*!
    Write control byte permanent storage
 */
void vscp_setControlByte(uint8_t ctrl)
{
	buffer[VSCP_FLASH_CONTROL * 4] = ctrl;

#ifdef FlashEverytime
	writeFLASH(VSCP_FLASH_BASE, buffer);
#endif
}


/*!
    Get page select bytes
        idx=0 - byte 0 MSB
        idx=1 - byte 1 LSB
 */
uint8_t vscp_getPageSelect(uint8_t idx)
{
	return 0; /* todo: vscp_getPageSelect */
}


/*!
    Set page select registers
    @param idx 0 for LSB, 1 for MSB
    @param data Byte to set of page select registers
 */
void vscp_setPageSelect(uint8_t idx, uint8_t data)
{
	return; /* todo: function does not appear in can4vscp_paris module */
}

/*!
    The actual work is done here. Get angle
 */
void doWork(void)
{
	accel_data_t accelData;

	if ( VSCP_STATE_ACTIVE == vscp_node_state ) {

		accelData = getAngle(i2cDevice);
		PRINTF("X = %d, Y = %d \r\n", accelData.xAngle, accelData.yAngle);

		current_xAngle = accelData.xAngle;
		current_yAngle = accelData.yAngle;

		vscp_omsg.flags = VSCP_VALID_MSG + 3; // three data byte
		vscp_omsg.priority = VSCP_PRIORITY_LOW;
		vscp_omsg.vscp_class = VSCP_CLASS1_MEASUREMENT;
		vscp_omsg.vscp_type = VSCP_TYPE_MEASUREMENT64_ANGLE;
		vscp_omsg.data[ 0 ] = 0;
		vscp_omsg.data[ 1 ] = accelData.xAngle;
		vscp_omsg.data[ 2 ] = accelData.yAngle;

		// send the event
		vscp_sendEvent();

	}
}

void doOneSecondWork(void)
{
	uint8_t tmp;
	uint8_t i;

	// Check if events should be sent
	if ( VSCP_STATE_ACTIVE == vscp_node_state ) {

		// Time for temperature report
		tmp = spi_eeprom_read(DEFAULT_REPORT_INTERVAL_TEMP0 + i);
		if (tmp && (seconds_temp > tmp)) {

			// Send event
			if ( sendTempEvent( 0 ) ) {
				seconds_temp = 0;
			}

		}

	}
}


///////////////////////////////////////////////////////////////////////////////
// sendTempEvent
//

int8_t sendTempEvent(uint8_t i)
{
    vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
    vscp_omsg.flags = VSCP_VALID_MSG + 4;
    vscp_omsg.vscp_class = VSCP_CLASS1_MEASUREMENT;
    vscp_omsg.vscp_type = VSCP_TYPE_MEASUREMENT_TEMPERATURE;

    // Data format
    vscp_omsg.data[ 0 ] = 0x80 | // Normalized integer
            ((0x03 & spi_eeprom_read(i + REG_TEMP0_CONTROL)) << 3) | // Unit
            i; // Sensor
    // Exponent
    vscp_omsg.data[ 1 ] = 0x82;

    setEventData( current_temp,
            ( 0x03 & spi_eeprom_read(i + REG_TEMP0_CONTROL)) ); // Format data based on Unit

    // Send event
    if (!vscp_sendEvent()) {
        return FALSE;
    }

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// setEventData
//

void setEventData(int v, unsigned char unit)
{
    uint8_t newval; //double newval;
    //int ival;

    if (TEMP_UNIT_KELVIN == unit) {
        // Convert to Kelvin
        //newval = Celsius2Kelvin(v);
    } else if (TEMP_UNIT_FAHRENHEIT == unit) {
        // Convert to Fahrenheit
        newval = Celsius2Fahrenheit(v);
    } else {
        // Defaults to Celsius
        newval = v;
    }

    //ival = (int) newval;

    vscp_omsg.data[ 2 ] = newval;

    /*
    vscp_omsg.data[ 2 ] = ((ival & 0xff00) >> 8);
    vscp_omsg.data[ 3 ] = (ival & 0xff);
    */

}


/*!
    @brief vscp_readAppReg
 */
uint8_t vscp_readAppReg(uint8_t reg)
{
	uint8_t rv;
	rv = 0x00; //default read

	if( 0 == vscp_page_select){

		switch (reg) {

		// Zone
		case 0x00:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_FRDM_ZONE);
			break;

		// Subzone
		case 0x01:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_FRDM_SUBZONE);
			break;

		// Control register for temp sensor 0
		case 0x02:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_TEMP0_CONTROL);
			break;

		// Control register for accel sensor 0
		case 0x03:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_ACCEL0_CONTROL);
			break;

		// Current temp sensor value
		case 0x04:
			rv = current_temp; // Yes it is this simple for now (only using uint8_t). Will need 16 bits later for Kelvin.
			break;

		// Current accel x angle
		case 0x05:
			rv = current_xAngle;
			break;

		// Current accel y angle
		case 0x06:
			rv = current_yAngle;
			break;

			// Interval register for temp sensor 0
		case 0x07:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_TEMP0_REPORT_INTERVAL);
			break;

			// Interval register for accel sensor 0
		case 0x08:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_ACCEL0_REPORT_INTERVAL);
			break;

			// High alarm register for temp sensor 0
		case 0x09:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_TEMP0_HIGH_ALARM);
			break;

			// Low alarm register for temp sensor 0
		case 0x10:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_TEMP0_LOW_ALARM);
			break;

			// High alarm register for accel sensor 0
		case 0x11:
			rv = spi_eeprom_read(VSCP_EEPROM_END + REG_ACCEL0_HIGH_ALARM);
			break;
		}
	}

	return rv;
}

/*!
    @brief vscp_writeAppReg
 */
uint8_t vscp_writeAppReg( uint8_t reg, uint8_t val )
{
	uint8_t rv;
	rv = ~val; //error return

	if(0 == vscp_page_select){

		switch (reg) {

				// Zone
				case 0x00:
					spi_eeprom_write(VSCP_EEPROM_END + REG_FRDM_ZONE, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_FRDM_ZONE);
					break;

				// Subzone
				case 0x01:
					spi_eeprom_write(VSCP_EEPROM_END + REG_FRDM_SUBZONE, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_FRDM_SUBZONE);
					break;

				// Control register for temp sensor 0
				case 0x02:
					spi_eeprom_write(VSCP_EEPROM_END + REG_TEMP0_CONTROL, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_TEMP0_CONTROL);
					break;

				// Control register for accel sensor 0
				case 0x03:
					spi_eeprom_write(VSCP_EEPROM_END + REG_ACCEL0_CONTROL, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_ACCEL0_CONTROL);
					break;

				// Current temp sensor value - Cannot write this
				case 0x04:
					rv = current_temp;
					break;

				// Current accel x angle - Cannot write this
				case 0x05:
					rv = current_xAngle;
					break;

				// Current accel y angle - Cannot write this
				case 0x06:
					rv = current_yAngle;
					break;

					// Interval register for temp sensor 0
				case 0x07:
					spi_eeprom_write(VSCP_EEPROM_END + REG_TEMP0_REPORT_INTERVAL, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_TEMP0_REPORT_INTERVAL);
					break;

					// Interval register for accel sensor 0
				case 0x08:
					spi_eeprom_write(VSCP_EEPROM_END + REG_ACCEL0_REPORT_INTERVAL, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_ACCEL0_REPORT_INTERVAL);
					break;

					// High alarm register for temp sensor 0
				case 0x09:
					spi_eeprom_write(VSCP_EEPROM_END + REG_TEMP0_HIGH_ALARM, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_TEMP0_HIGH_ALARM);
					break;

					// Low alarm register for temp sensor 0
				case 0x10:
					spi_eeprom_write(VSCP_EEPROM_END + REG_TEMP0_LOW_ALARM, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_TEMP0_LOW_ALARM);
					break;

					// High alarm register for accel sensor 0
				case 0x11:
					spi_eeprom_write(VSCP_EEPROM_END + REG_ACCEL0_HIGH_ALARM, val);
					rv = spi_eeprom_read(VSCP_EEPROM_END + REG_ACCEL0_HIGH_ALARM);
					break;
				}
	}
	else if (1 == vscp_page_select){
		// Only used 1 page
	}

	return rv;
}


/*!
    Get DM matrix info
    The output message data structure should be filled with
    the following data by this routine.
        byte 0 - Number of DM rows. 0 if none.
        byte 1 - offset in register space.
        byte 2 - start page MSB
        byte 3 - start page LSB
        byte 4 - End page MSB
        byte 5 - End page LSB
        byte 6 - Level II size of DM row (Just for Level II nodes).
 */
void vscp_getMatrixInfo(char *pData)
{
	uint8_t i;

#ifdef FIRMWARE_DM
	vscp_omsg.data[ 0 ] = 7; // Matrix is seven rows
	vscp_omsg.data[ 1 ] = 72; //Matrix start offset

	// The rest set to zero no paging
	for(i=2; i<8; i++){
		vscp_omsg.data[ i ] = 0;
	}
#else
	// Alternatively, if module is not to implement a DM
	for(i=0; i<8; i++){
		pData[ i ] = 0;
	}
#endif

}


/*!
    Get embedded MDF info
    If available this routine sends an embedded MDF file
    in several events. See specification CLASS1.PROTOCOL
    Type=35/36

    Note that if sending the events back to back some devices
    will not be able to cope with the data stream.
    It is therefor advisable to have a short delay between
    each mdf data frame sent out.
 */
void vscp_getEmbeddedMdfInfo(void)
{
	// No embedded DM so we respond with info about that

	vscp_omsg.priority = VSCP_PRIORITY_NORMAL;
	vscp_omsg.flags = VSCP_VALID_MSG + 3;
	vscp_omsg.vscp_class = VSCP_CLASS1_PROTOCOL;
	vscp_omsg.vscp_type = VSCP_TYPE_PROTOCOL_RW_RESPONSE;

	vscp_omsg.data[ 0 ] = 0;
	vscp_omsg.data[ 1 ] = 0;
	vscp_omsg.data[ 2 ] = 0;

	// send the event
	vscp_sendEvent();

}


/*!
    Go boot loader mode
    This routine force the system into boot loader mode according
    to the selected protocol.
 */
void vscp_goBootloaderMode( uint8_t algorithm )
{
	return;
}

/*!
    Get Zone for device
    Just return zero if not used.
 */
uint8_t vscp_getZone(void)
{
	return 0;
}


/*!
    Get Subzone for device
    Just return zero if not used.
 */
uint8_t vscp_getSubzone(void)
{
	return 0;
}

/*!
    Get device family code
    return zero for not known.
*/
uint32_t vscp_getFamilyCode(void)
{
	return 0L;
}



/*!
    Get device family type
    return zero for not known.
*/
uint32_t vscp_getFamilyType(void)
{
	return 0;
}


/*!
    Restore defaults
    If 0x55/0xaa is written to register location
    162 within one second, defaults should be loaded
    by the device.
 */
void vscp_restoreDefaults(void)
{
	init_app_eeprom();
	init_app_ram();
	return;
}

#ifdef DROP_NICKNAME_EXTENDED_FEATURES

// Do a hard reset of the device
void vscp_hardreset(void)
{
	NVIC_SystemReset();
}

// Wait for milliseconds
void vscp_wait_ms(uint16_t ms)
{
	OSA_TimeDelay(ms); // Max delay of 65,536ms
}

// Wait for seconds
void vscp_wait_s(uint16_t sec)
{
	OSA_TimeDelay(sec*1000) //Up to 65.5s
}
#endif


	////////////////////////////////////////////////////////////////////////////////
	//
	//			Not part of vscp core functions
	//
	////////////////////////////////////////////////////////////////////////////////

void vscp_FLASHFlush()
{
	writeFLASH(VSCP_FLASH_BASE, buffer);
}


/* todo: finish this off once I decide what should be in ram */

void init_app_ram( void )
{
	uint8_t i;

    measurement_clock = 0;      // start a new measurement cycle

    seconds = 0;
    minutes = 0;
    hours = 0;

    current_temp = 0;
    current_xAngle = 0;
    current_yAngle = 0;

    temp0_low_alarm = 0;
    temp0_high_alarm = 0;
    accel0_high_alarm = 0;

    /// EXAMPLE BELOW FROM PARIS MODULE

   /* relay_pulse_flags = 0;      // No pulse outputs yet

    // Clear timers
    for ( i = 0; i < 8; i++ ) {
        relay_pulse_timer[ i ] = 0;
        relay_protection_timer[ i ] = 0;
    }

    for ( i=0; i<7; i++ ) {

        // Init pulsed relays
        if ( eeprom_read( VSCP_EEPROM_END + REG_RELAY0_CONTROL + i ) &
                                RELAY_CONTROLBIT_PULSE ) {

            relay_pulse_flags |= (1<<i); // Enable pulse output
            relay_pulse_timer[ i ] =
                eeprom_read( VSCP_EEPROM_END + REG_RELAY0_PULSE_TIME_MSB + i ) * 256 +
                eeprom_read( VSCP_EEPROM_END + REG_RELAY0_PULSE_TIME_LSB + i );
        }

        // Init protection timers
        if ( eeprom_read(VSCP_EEPROM_END + REG_RELAY5_CONTROL + i ) & RELAY_CONTROLBIT_PROTECTION) {
            relay_protection_timer[ i ] =
                eeprom_read(VSCP_EEPROM_END + REG_RELAY5_PROTECTION_TIME_MSB + i ) * 256 +
                eeprom_read(VSCP_EEPROM_END + REG_RELAY5_PROTECTION_TIME_LSB + i );
        }
    }*/

}


void init_app_eeprom(void)
{

	spi_eeprom_write(VSCP_EEPROM_END + REG_FRDM_ZONE, 0x05);
	spi_eeprom_write(VSCP_EEPROM_END + REG_FRDM_SUBZONE, 0x01);

	spi_eeprom_write(VSCP_EEPROM_END + REG_TEMP0_CONTROL, DEFAULT_CONTROL_REG);
	spi_eeprom_write(VSCP_EEPROM_END + REG_ACCEL0_CONTROL, DEFAULT_CONTROL_REG);

	spi_eeprom_write(VSCP_EEPROM_END + REG_TEMP0_REPORT_INTERVAL, DEFAULT_REPORT_INTERVAL_TEMP0);
	spi_eeprom_write(VSCP_EEPROM_END + REG_ACCEL0_REPORT_INTERVAL, DEFAULT_REPORT_INTERVAL_ACCEL0);

	spi_eeprom_write(VSCP_EEPROM_END + REG_TEMP0_HIGH_ALARM, DEFAULT_TEMP0_HIGH_ALARM);
	spi_eeprom_write(VSCP_EEPROM_END + REG_TEMP0_LOW_ALARM, DEFAULT_TEMP0_LOW_ALARM);
	spi_eeprom_write(VSCP_EEPROM_END + REG_ACCEL0_HIGH_ALARM, DEFAULT_ACCEL0_HIGH_ALARM);


#ifdef THIS_IS_AN_EXAMPLE
    unsigned char i, j;

    eeprom_write( VSCP_EEPROM_END + REG_RELAY_ZONE, 0 );
    eeprom_write( VSCP_EEPROM_END + REG_RELAY_SUBZONE, 0 );

    eeprom_write( VSCP_EEPROM_END + REG_RELAY0_CONTROL,
                    RELAY_CONTROLBIT_ONEVENT |
                    RELAY_CONTROLBIT_OFFEVENT |
                    RELAY_CONTROLBIT_ENABLED );

    ...

    eeprom_write( VSCP_EEPROM_END + REG_RELAY7_CONTROL,
                    RELAY_CONTROLBIT_ONEVENT |
                    RELAY_CONTROLBIT_OFFEVENT |
                    RELAY_CONTROLBIT_ENABLED );

    eeprom_write( VSCP_EEPROM_END + REG_RELAY0_PULSE_TIME_MSB, 0 );
    eeprom_write( VSCP_EEPROM_END + REG_RELAY0_PULSE_TIME_LSB, 0 );

    ...

    eeprom_write( VSCP_EEPROM_END + REG_RELAY7_PULSE_TIME_MSB, 0 );
    eeprom_write( VSCP_EEPROM_END + REG_RELAY7_PULSE_TIME_LSB, 0  );

    eeprom_write( VSCP_EEPROM_END + REG_RELAY0_PROTECTION_TIME_MSB, 0 );
    eeprom_write( VSCP_EEPROM_END + REG_RELAY0_PROTECTION_TIME_LSB, 0 );

    ...

    eeprom_write( VSCP_EEPROM_END + REG_RELAY7_PROTECTION_TIME_MSB, 0 );
    eeprom_write( VSCP_EEPROM_END + REG_RELAY7_PROTECTION_TIME_LSB, 0 );

    eeprom_write( VSCP_EEPROM_END + REG_RELAY0_ZONE, 0 );
    eeprom_write( VSCP_EEPROM_END + REG_RELAY0_SUBZONE, 0 );

    ...

    eeprom_write( VSCP_EEPROM_END + REG_RELAY7_ZONE, 0 );
    eeprom_write( VSCP_EEPROM_END + REG_RELAY7_SUBZONE, 0 );

    // * * * Decision Matrix * * *
    // All elements disabled.
    for ( i = 0; i < DESCION_MATRIX_ROWS; i++ ) {
        for ( j = 0; j < 8; j++ ) {
            eeprom_write( VSCP_EEPROM_END + REG_DESCION_MATRIX + i * 8 + j, 0 );
        }
    }
#endif

}
