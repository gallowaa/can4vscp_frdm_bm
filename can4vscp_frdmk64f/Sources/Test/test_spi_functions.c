/*
 * test_spi_functions.c
 *
 *  Created on: Jul 8, 2015
 *      Author: Angus
 */

#include "spi.h"

void test_spi_generic(){

	uint8_t c1, c2;

	printf("Press 'w' to write: \r\n");
	c1 = GETCHAR();

	if('w' == c1)
		spi_eeprom_write(0x0A,c1);


	printf("Enter an op-code, then address:\r\n");
	c1 = GETCHAR();
	printf("received %c = hex %x\r\n", c1, c1);
	c2 = GETCHAR();
	printf("received %c = hex %x\r\n", c2, c2);

	spi_eeprom_read_generic(c1, c2);

}
void test_spi(){

	uint8_t addr;
	uint8_t ret;

	printf("Enter an address (0x00 to 0xC0) to read a byte from: \r\n");
	addr = GETCHAR();
	ret = spi_eeprom_read(addr);
	printf("Received %02X\r\n", ret);
}

