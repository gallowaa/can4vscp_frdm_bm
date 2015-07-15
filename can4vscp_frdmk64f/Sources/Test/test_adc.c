/*
 * test_adc.c
 *
 *  Created on: Jul 14, 2015
 *      Author: Angus
 */
#include "main.h"

extern volatile bool conversionCompleted;  /*! Conversion is completed Flag */
int32_t GetCurrentTempValue(void);

void test_adc(void);

void test_adc(void) {

	int32_t currentTemperature = 0;

	while(!conversionCompleted){}

	// Get current Temperature Value
	currentTemperature = GetCurrentTempValue();
	currentTemperature = (uint8_t)(currentTemperature & 0xff);
	PRINTF("current temp = %d\r\n", currentTemperature);
	conversionCompleted = false;

}
