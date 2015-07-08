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
			    .whichPcs                   = kDspiPcs0				// commented out as gpio is used for cs instead. This way did not allow the eeprom to be read
};

/*!
    @brief init_spi - initialize dspi master driver for 1Mbps
    @param none
    @return none
 */
void init_spi() {

	/*dspi_master_user_config_t masterUserConfig = {
			.isChipSelectContinuous     = false,
			.isSckContinuous            = false,
			.pcsPolarity                = kDspiPcs_ActiveLow,
			.whichCtar                  = kDspiCtar0,
			//     .whichPcs                   = kDspiPcs0				// commented out as gpio is used for cs instead. This way did not allow the eeprom to be read
	};*/

#ifdef USE_INIT
	// Setup the configuration and get user options.
	masterDevice.dataBusConfig.bitsPerFrame = 8;
	masterDevice.dataBusConfig.clkPhase     = kDspiClockPhase_FirstEdge;
	masterDevice.dataBusConfig.clkPolarity  = kDspiClockPolarity_ActiveHigh;
	masterDevice.dataBusConfig.direction    = kDspiMsbFirst;

	// Initialize master driver.
	dspiResult = DSPI_DRV_MasterInit(DSPI_INSTANCE, &masterState, &masterUserConfig);
	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\n ERROR: Can not initialize master driver \n");

	}

	// configure baudrate.
	masterDevice.bitsPerSec = TRANSFER_BAUDRATE;
	dspiResult = DSPI_DRV_MasterConfigureBus(DSPI_INSTANCE, &masterDevice, &calculatedBaudRate);
	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\nERROR: failure in config bus, error %d\n\r", dspiResult);
	}
	else
	{
		printf("\r\n Transfer at baudrate %lu\n", calculatedBaudRate);
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

	uint32_t i;
	dspi_status_t dspiResult;

	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {1};

	//SPI0_CS_DESELECT; // Make chip select high (the eeprom needs to see a falling edge)

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

	// Initialize the transmit buffer with some data bytes
	for (i = 2; i < TRANSFER_SIZE; i++)
	{
		sendBuffer[i] = i;
	}

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
}


/*!
    @brief spi_eeprom_read - Read a byte from the eeprom
    @param addr - address to read from
    @return - byte read from memory
 */
uint8_t spi_eeprom_read(uint8_t addr) {

	uint32_t i;
	dspi_status_t dspiResult;					/*! check status of transfer */
	uint8_t sendBuffer[MIN_TFR_SIZE] 	 = {0}; /*! [  op-code ] [  address ] [     dummy   ] */
	uint8_t receiveBuffer[ MIN_TFR_SIZE] = {0};	/*! [  dummy   ] [  dummy   ] [returned data] */


	sendBuffer[0] = READ; // The read op-code 0x03
	sendBuffer[1] = addr;

	// print the transmit buffer.
	printf("Transmitting: \r\n");
	for (i = 0; i <  MIN_TFR_SIZE; i++)
	{
		printf(" %02X", sendBuffer[i]);
	}
	printf("\r\n");

	//SPI0_CS_SELECT;	// Make cs low

	/**********************************************
	 * Read the data with a blocking transfer API call.
	 **********************************************/

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
												 NULL,
												 sendBuffer,
												 receiveBuffer,
												 MIN_TFR_SIZE,
												 MASTER_TRANSFER_TIMEOUT);

	//SPI0_CS_DESELECT; // bring cs back high

	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\n ERROR: transfer error, err %d\n\r", dspiResult);
	}

	//dspiResult =  DSPI_DRV_MasterDeinit(DSPI_INSTANCE);

	// Print out receive buffer.
	printf("Master receive:\r\n");
	for (i = 0; i < MIN_TFR_SIZE; i++)
	{
		// Print 8 numbers in a line.
		if (0 == (i & 0x0F))
		{
			printf("\r\n");
		}
		printf(" %02X", receiveBuffer[i]);
	}
	printf("\r\n");

	return receiveBuffer[DATA_BYTE];
}



/*!
    @brief spi_eeprom_read_generic - This function is only for testing!
 */
uint8_t spi_eeprom_read_generic(uint8_t addr1, uint8_t addr2) {

	 /*! need these variables when using an init function */
	uint32_t i;
	dspi_status_t dspiResult;
	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {1,2,0x0A,4,5,6,7,8};

#ifndef USE_INIT
	uint32_t calculatedBaudRate;
	dspi_status_t dspiResult;
	dspi_master_state_t masterState;
	dspi_device_t masterDevice;

	dspi_master_user_config_t masterUserConfig = {
			.isChipSelectContinuous     = false,
			.isSckContinuous            = false,
			.pcsPolarity                = kDspiPcs_ActiveLow,
			.whichCtar                  = kDspiCtar0,
			//     .whichPcs                   = kDspiPcs0				// commented out as gpio is used for cs instead. This way did not allow the eeprom to be read
	};

	// Setup the configuration and get user options.
	masterDevice.dataBusConfig.bitsPerFrame = 8;
	masterDevice.dataBusConfig.clkPhase     = kDspiClockPhase_FirstEdge;
	masterDevice.dataBusConfig.clkPolarity  = kDspiClockPolarity_ActiveHigh;
	masterDevice.dataBusConfig.direction    = kDspiMsbFirst;

	// Initialize master driver.
	dspiResult = DSPI_DRV_MasterInit(DSPI_INSTANCE, &masterState, &masterUserConfig);
	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\n ERROR: Can not initialize master driver \n");
	}

	// configure baudrate.
	masterDevice.bitsPerSec = TRANSFER_BAUDRATE;
	dspiResult = DSPI_DRV_MasterConfigureBus(DSPI_INSTANCE, &masterDevice, &calculatedBaudRate);
	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\nERROR: failure in config bus, error %d\n\r", dspiResult);
	}
	else
	{
		printf("\r\n Transfer at baudrate %lu\n", calculatedBaudRate);
	}
#endif

	sendBuffer[0] = addr1; //0x03; //read op-code

	if(addr2 == 0x1)
		sendBuffer[1] = 0xFA;
	else
		sendBuffer[1] = addr2;//0xfa; //address start

	// Initialize the transmit buffer.
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
}


