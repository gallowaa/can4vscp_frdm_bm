/*
 * spi.h
 *
 *  Created on: Jun 18, 2015
 *      Author: Angus
 */

#ifndef SOURCES_SPI_AL_H_
#define SOURCES_SPI_AL_H_

#include <stdio.h>
#include "main.h"
#include "fsl_dspi_master_driver.h"


#define WRSR  0x01 // 0x001
#define WRITE 0x02 // 0x010
#define READ  0x03 // 0x011
#define WRDI  0x04 // 0x100
#define RDSR  0x05 // 0x101
#define WREN  0x06 // 0x110

/*******************************************************************************
 * 							Definitions for SPI
 ******************************************************************************/
#define DSPI_INSTANCE               (0)       /*! User change define to choose DSPI instance */
#define DATA_BYTE					(2)
#define MIN_TFR_SIZE				(3)
#define TRANSFER_SIZE               (8)      /*! Transfer size */
#define TRANSFER_BAUDRATE           (1000000U) /*! Transfer baudrate - 1M */
#define MASTER_TRANSFER_TIMEOUT     (5000U)   /*! Transfer timeout of master - 5s */


/*******************************************************************************
 * 								Prototypes
 ******************************************************************************/

/*!
    @brief init_spi - initialize dspi master driver for 1Mbps
    @param none
    @return none
 */
void init_spi();


/*!
    @brief spi_eeprom_write - Write a byte to the eeprom which is connected to SPI0
    @param data - byte to be written to the eeprom
    @param addr - address to write
    @return TRUE on success.
 */
void spi_eeprom_write(uint8_t data, uint8_t addr);


/*!
    @brief spi_eeprom_read - Read a byte from the eeprom
    @param addr - address to read from
    @return - byte read from memory
 */
uint8_t spi_eeprom_read(uint8_t addr);


/*!
    @brief spi_eeprom_read_generic - This function is only for testing!
 */
uint8_t spi_eeprom_read_generic(uint8_t addr1, uint8_t addr2);

#endif /* SOURCES_SPI_AL_H_ */
