/*
 * spi.c
 *
 *  Created on: Jun 18, 2015
 *      Author: Angus
 */

#include "spi.h"

/* The 2K bit 25AA02E48 SPI EEPROM from microchip is used */

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

void init_spi() {

	/*dspi_master_user_config_t masterUserConfig = {
			.isChipSelectContinuous     = false,
			.isSckContinuous            = false,
			.pcsPolarity                = kDspiPcs_ActiveLow,
			.whichCtar                  = kDspiCtar0,
			//     .whichPcs                   = kDspiPcs0				// commented out as gpio is used for cs instead. This way did not allow the eeprom to be read
	};*/


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
}

uint8_t spi_eeprom_read(uint8_t addr1, uint8_t addr2) {

	uint32_t i;
	dspi_status_t dspiResult;

	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {1};

	/*
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
	}*/

	//SPI0_CS_DESELECT; // Make chip select high (the eeprom needs to see a falling edge)


	sendBuffer[0] = addr1;//0x03; //read op-code

	if(addr2 == 0x1)
		sendBuffer[1] = 0xFA;
	else
		sendBuffer[1] = addr2;//0xfa; //address start

	// Initialize the transmit buffer.
	/*
	for (i = 2; i < TRANSFER_SIZE; i++)
	{
		sendBuffer[i] = 0xff;
	}*/

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
	printf("\r\n Master receive:\n");
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
    Write a byte to the eeprom which is connected to SPI0
    @param writeByte - byte to be written to the eeprom
    @param address
    @return TRUE on success.
 */
void spi_eeprom_write(uint8_t writeByte, uint8_t addr){

	uint32_t i;
	dspi_status_t dspiResult;

	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {1};

	//SPI0_CS_DESELECT; // Make chip select high (the eeprom needs to see a falling edge)

	/* The WREN instruction must be sent */
	sendBuffer[0] = WREN;  // WREN  = 0b110 = 0x6
	sendBuffer[1] = WRITE; // WRITE = 0b010 = 0x2
	sendBuffer[2] = addr;  // address start

	// Initialize the transmit buffer.
	for (i = 2; i < TRANSFER_SIZE; i++)
	{
		sendBuffer[i] = 0xff;
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

	// Print out receive buffer.
	printf("\r\n Master receive:\n");
	for (i = 0; i < TRANSFER_SIZE; i++)
	{
		// Print 8 numbers in a line.
		if (0 == (i & 0x0F))
		{
			printf("\r\n    ");
		}
		printf(" %02X", receiveBuffer[i]);
	}
}
