/*
 * spi.c
 *
 *  Created on: Jun 18, 2015
 *      Author: Angus
 */

#include "spi.h"

void read_spi_eeprom() {

	uint32_t i;
	//uint8_t loopCount = 1;
	uint32_t calculatedBaudRate;
	dspi_status_t dspiResult;
	dspi_master_state_t masterState;
	dspi_device_t masterDevice;

	uint8_t receiveBuffer[TRANSFER_SIZE] = {0};
	uint8_t sendBuffer[TRANSFER_SIZE] = {1};

	SPI0_CS_DESELECT;

	dspi_master_user_config_t masterUserConfig = {
			.isChipSelectContinuous     = false,
			.isSckContinuous            = false,
			.pcsPolarity                = kDspiPcs_ActiveLow,
			.whichCtar                  = kDspiCtar0,
	 //     .whichPcs                   = kDspiPcs0				/*! commented out as gpio is used for cs instead. This way did not allow the eeprom to be read */
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

	sendBuffer[0] = 0x03; //read op-code
	sendBuffer[1] = 0xfa; //address start

	// Initialize the transmit buffer.
	for (i = 2; i < TRANSFER_SIZE; i++)
	{
		sendBuffer[i] = 0xff;
	}


	SPI0_CS_SELECT;

	dspiResult = DSPI_DRV_MasterTransferBlocking(DSPI_INSTANCE,
			NULL,
			sendBuffer,
			receiveBuffer,
			TRANSFER_SIZE,
			MASTER_TRANSFER_TIMEOUT);

	SPI0_CS_DESELECT;

	if (dspiResult != kStatus_DSPI_Success)
	{
		printf("\r\n ERROR: transfer error, err %d\n\r", dspiResult);

	}

	// Print out receive buffer.
	printf("\r\n Master receive:\n");
	for (i = 0; i < TRANSFER_SIZE; i++)
	{
		// Print 8 numbers in a line.
		if ((i & 0x0F) == 0)
		{
			printf("\r\n    ");
		}
		printf(" %02X", receiveBuffer[i]);
	}
}
