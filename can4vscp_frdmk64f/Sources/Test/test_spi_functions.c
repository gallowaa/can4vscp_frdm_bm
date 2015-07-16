/*
 * test_spi_functions.c
 *
 *  Created on: Jul 8, 2015
 *      Author: Angus
 */

#include "spi.h"

void spi_eeprom_guid_init();

void test_spi_generic() {

	int i;
	uint8_t addr,ret;

	spi_eeprom_guid_init();

	for(i=0;i<16;i++) {
		printf("Enter an address (0x10 to 0x1f) to read a byte from: \r\n");
		addr = GETCHAR();
		ret = spi_eeprom_read(addr);
		printf("Master receive: (0x%02x) = %02X \r\n", addr, ret);
	}

}
void test_spi(){

	uint8_t addr;
	uint8_t ret;

	printf("Enter an address (0x00 to 0xC0) to read a byte from: \r\n");
	addr = GETCHAR();
	ret = spi_eeprom_read(addr);
	printf("Received %02X\r\n", ret);
}

