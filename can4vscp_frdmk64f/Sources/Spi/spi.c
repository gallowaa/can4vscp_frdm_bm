/*
 * spi.c
 *
 *  Created on: Jun 18, 2015
 *      Author: Angus
 */

#include "spi.h"
#define USE_INIT

/* The 2K bit 25AA02E48 SPI EEPROM from microchip is used
 * By default the upper 1/4 of the array is write protected,
 * from 0xC0 - 0xFF. The EUI-48 id occupies 0xFA - 0xFF
 *
 * WRSR  = 0x001
 * WRITE = 0x010
 * READ  = 0x011
 * WRDI  = 0x100
 * RDSR  = 0x101
 * WREN  = 0x110
 *
 */

uint32_t calculatedBaudRate;
dspi_status_t dspiResult;
dspi_master_state_t masterState;
dspi_device_t masterDevice;

dspi_master_user_config_t masterUserConfig = {
				.isChipSelectContinuous     = true,
				.isSckContinuous            = false,
				.pcsPolarity                = kDspiPcs_ActiveLow,
				.whichCtar                  = kDspiCtar0,
			    .whichPcs                   = kDspiPcs0
};

/*!
    @brief init_spi - initialize dspi master driver for 1Mbps
    @param none
    @return none
 */
void init_spi() {

#ifdef USE_INIT
	// Setup the configuration and get user options.
	masterDevice.dataBusConfig.bitsPerFrame = 8;
	masterDevice.dataBusConfig.clkPhase     = kDspiClockPhase_FirstEdge;
	masterDevice.dataBusConfig.clkPolarity  = kDspiClockPolarity_ActiveHigh;
	masterDevice.dataBusConfig.direction    = kDspiMsbFirst;

	// Initialize master driver.
	dspiResult = DSPI_DRV_MasterInit(DSPI_INSTANCE, &masterState, &masterUserConfig);
	if (dspiResult != kStatus_DSPI_Success)	{
		printf("\r\n ERROR: Can not initialize master driver \n");
	}

	// configure baudrate.
	masterDevice.bitsPerSec = TRANSFER_BAUDRATE;
	dspiResult = DSPI_DRV_MasterConfigureBus(DSPI_INSTANCE, &masterDevice, &calculatedBaudRate);
	if (dspiResult != kStatus_DSPI_Success)	{
		printf("\r\nERROR: failure in config bus, error %d\n\r", dspiResult);
	}
	else {
#ifdef DO_PRINT
		printf("Transfer at baudrate %lu\r\n", calculatedBaudRate);
#endif
	}
#endif
}


/*!
    @brief spi_eeprom_write - Write a byte to the eeprom which is connected to SPI0
    @param data - byte to be written to the eeprom
    @param addr - address to write
    @return TRUE on success.
 */
void spi_eeprom_write(uint8_t addr, uint8_t data){

#ifdef DO_PRINT
	uint32_t i;
#endif

	dspi_status_t dspiResult;
	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {0};

	/**********************************************
	 * The WREN instruction must be sent separately
	 * to enable writes to the eeprom. This is done
	 * with its own API call to make the CS line
	 * comes back up which sets the WREN latch.
	 **********************************************/
	sendBuffer[0] = WREN;

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
												 NULL,sendBuffer,
												 receiveBuffer,
												 1,							/*! 1 byte for WREN */
												 MASTER_TRANSFER_TIMEOUT);

	/**********************************************
	 * Write the actual data with another API call.
	 **********************************************/

	sendBuffer[0] = WRITE; // Overwrite the WREN cmd with the WRITE command
	sendBuffer[1] = addr;  // address to write

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
												 NULL,
												 sendBuffer,
												 receiveBuffer,
												 TRANSFER_SIZE,
												 MASTER_TRANSFER_TIMEOUT);

	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\n ERROR: transfer error, err %d\n\r", dspiResult);
	}

#ifdef EEPROM_LOCK
	/**********************************************
		 * Send the WRDI instruction to prevent from
		 * accidentally writing to the eeprom
		 **********************************************/

		sendBuffer[0] = WRDI;

		dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
													NULL,sendBuffer,
													 receiveBuffer,
													 1,							/*! 1 byte for WREN */
													 MASTER_TRANSFER_TIMEOUT);
#endif

#ifdef DO_PRINT

	// Print out receive buffer.
	printf("Master receive:\r\n");
	for (i = 0; i < TRANSFER_SIZE; i++)
	{
		// Print 8 numbers in a line.
		if (0 == (i & 0x0F))
		{
			printf("\r\n");
		}
		printf(" %02X", receiveBuffer[i]);
	}
	printf("\r\n");
#endif
}


/*!
    @brief spi_eeprom_read - Read a byte from the eeprom
    @param addr - address to read from
    @return - byte read from memory
 */
uint8_t spi_eeprom_read(uint8_t addr) {

#ifdef DO_PRINT
	uint32_t i;
#endif

	dspi_status_t dspiResult;					/*! check status of transfer */
	uint8_t sendBuffer[MIN_TFR_SIZE] 	 = {0}; /*! [  op-code ] [  address ] [     dummy   ] */
	uint8_t receiveBuffer[ MIN_TFR_SIZE] = {0};	/*! [  dummy   ] [  dummy   ] [returned data] */


	sendBuffer[0] = READ; // The read op-code 0x03
	sendBuffer[1] = addr;

	/**********************************************
	 * Read the data with a blocking transfer API call.
	 **********************************************/

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
												 NULL,
												 sendBuffer,
												 receiveBuffer,
												 MIN_TFR_SIZE,
												 MASTER_TRANSFER_TIMEOUT);

	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\n ERROR: transfer error, err %d\n\r", dspiResult);
	}

	return receiveBuffer[DATA_BYTE];
}



/*!
    @brief spi_eeprom_guid_init - Initialize the eeprom with the GUID in the appropriate location
    @purpose The EUI-48 lives in a write protected area by default.
    		 Copy it to an earlier memory location and create a proper vscp GUID with the format
    		 for a GUID based on an ethernet MAC.
 */
void spi_eeprom_guid_init() {

	/* GUID layout in eeprom interfaced over SPI
	 *
	 * FF:FF:FF:FF:FF:FF:FF:FE:YY:YY:YY:YY:YY:YY:XX:XX
	 *
	 * The holder of the address can freely use the two least significant bytes of the GUID.
	 * MAC address in MSB - LSB order. Also called MAC-48 or EUI-48 by IEEE
	 * Source: http://www.vscp.org/docs/vscpspec/doku.php?id=globally_unique_identifiers
	 */

	uint32_t i,j;
	dspi_status_t dspiResult;

	uint8_t txBuffer[TRANSFER_SIZE] = {READ, EUI48_START, 2,3,4,5,6,7};	/*! contains op-code + addr for getting EUI-48 */

	uint8_t rxEUI48[TRANSFER_SIZE] = {0};						/*! buffer specifically for receiving EUI-48 from write protected region */
	uint8_t rxBuffer[TRANSFER_SIZE] = {0};						/*! generic receive buffer for WREN, WRDI writes */


	/* Note that we don't write the last 2 bytes (XX:XX) because they are for the nickname
	 * and are determined later. Array is still 16 bytes as 2 get taken up for opcode + address at beginning */
	uint8_t txGUID[16] = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
						// 	  ,	    , 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xYY, 0xYY, 0xYY, 0xYY, 0xYY, 0xYY, 0xXX, 0xXX

	uint8_t rxGUIDbuff[16] = {0};								/*! This may not be needed, but we provide a 16-byte buffer
																 *  to satisfy the api call when writing 16-byte GUID */


	/**********************************************
	 * First, get the EUI48 from the write protected region
	 **********************************************/

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
												NULL,
												txBuffer,
												rxEUI48,
												TRANSFER_SIZE,
												MASTER_TRANSFER_TIMEOUT);

	j=10; //position 10 is the start index of the EUI-48 in the GUID

	for (i = 2; i < TRANSFER_SIZE; i++)
	{
		txGUID[j] = rxEUI48[i];
		j++;
	}

	/**********************************************
	 * The WREN instruction must be sent separately
	 * to enable writes to the eeprom. This is done
	 * with its own API call to make the CS line
	 * come back up which sets the WREN latch.
	 **********************************************/
	txBuffer[0] = WREN;

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
												NULL,
												txBuffer,
												rxBuffer,
												1,							/*! 1 byte for WREN */
												MASTER_TRANSFER_TIMEOUT);

	/**********************************************
	 * Write the actual data with another API call.
	 **********************************************/

	txGUID[0] = WRITE; // WRITE command
	txGUID[1] = VSCP_EEPROM_REG_GUID;  // address to write

#ifdef DEBUG
	printf("GUID tx: ");
	for (i = 0; i < 16; i++) {
		printf(" %02X", txGUID[i]);
	}
	printf("\r\n");
#endif

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
												 NULL,
												 txGUID,
												 rxGUIDbuff,
												 16,
												 MASTER_TRANSFER_TIMEOUT);


	if (dspiResult != kStatus_DSPI_Success)	{
		printf("\r\n ERROR: transfer error, err %d\n\r", dspiResult);
	}

	/**********************************************
	 * Send the WRDI instruction to prevent from
	 * accidentally writing to the eeprom
	 **********************************************/

	txBuffer[0] = WRDI;

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
			NULL,
			txBuffer,
			rxBuffer,
			1,							/*! 1 byte for WREN */
			MASTER_TRANSFER_TIMEOUT);

}


/*!
    @brief spi_eeprom_print_eui48 - This function is only for testing!
 */
void spi_eeprom_print_eui48() {

#ifdef DO_PRINT
	uint32_t i;
#endif

	dspi_status_t dspiResult;
	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {0x03,0xFA,2,3,4,5,6,7};

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
			NULL,
			sendBuffer,
			receiveBuffer,
			TRANSFER_SIZE,
			MASTER_TRANSFER_TIMEOUT);

#ifdef DO_PRINT
	// Print out receive buffer.
	printf("Master receive:\r\n");
	for (i = 0; i < TRANSFER_SIZE; i++){
		printf(" %02X", receiveBuffer[i]);
	}
	printf("\r\n");
#endif
}


/*!
    @brief spi_eeprom_generic_cmd - This function is only for testing!
 */
uint8_t spi_eeprom_generic_cmd(uint8_t addr1, uint8_t addr2) {

	 /*! need these variables when using an init function */
	uint32_t i;
	dspi_status_t dspiResult;
	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {1,2,0x0A,4,5,6,7,8};

	sendBuffer[0] = addr1; //0x03; //read op-code

	if(addr2 == 0x1)
		sendBuffer[1] = 0xFA;
	else
		sendBuffer[1] = addr2;//0xfa; //address start

	// Print the transmit buffer.
	printf("Transmitting: \r\n");
	for (i = 0; i < TRANSFER_SIZE; i++)
	{
		printf(" %02X", sendBuffer[i]);
	}
	printf("\r\n");

	//SPI0_CS_SELECT;	// Make cs low

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
			NULL,
			sendBuffer,
			receiveBuffer,
			TRANSFER_SIZE,
			MASTER_TRANSFER_TIMEOUT);

	//SPI0_CS_DESELECT; // bring cs back high

	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\n ERROR: transfer error, err %d\n\r", dspiResult);
	}

	//dspiResult =  DSPI_DRV_MasterDeinit(DSPI_INSTANCE);

	// Print out receive buffer.
	printf("Master receive:\r\n");
	for (i = 0; i < TRANSFER_SIZE; i++)
	{
		// Print 8 numbers in a line.
		if (0 == (i & 0x0F))
		{
			printf("\r\n");
		}
		printf(" %02X", receiveBuffer[i]);
	}
	printf("\r\n");

	return 0;
}

void spi_eeprom_lock(void) {

	dspi_status_t dspiResult;

	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {0};

	/**********************************************
	 * Send the WRDI instruction to prevent from
	 * accidentally writing to the eeprom
	 **********************************************/

	sendBuffer[0] = WRDI;

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
												NULL,sendBuffer,
												 receiveBuffer,
												 1,							/*! 1 byte for WREN */
												 MASTER_TRANSFER_TIMEOUT);

	if (dspiResult != kStatus_DSPI_Success)
	{
		PRINTF("\r\n ERROR: transfer error, err %d\n\r", dspiResult);
	}
}
