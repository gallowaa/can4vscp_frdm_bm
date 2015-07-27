/*
 * test_adc.c
 *
 *  Created on: Jul 14, 2015
 *      Author: Angus
 */
#include "main.h"

extern volatile bool conversionCompleted;  /*! Conversion is completed Flag */
extern uint8_t current_temp;
int32_t GetCurrentTempValue(void);

void test_adc(void);
void updateTemp(void);

void test_adc(void) {

	int32_t currentTemperature = 0;

	while(!conversionCompleted){}

	// Get current Temperature Value
	currentTemperature = GetCurrentTempValue();
	currentTemperature = (uint8_t)(currentTemperature & 0xff);
	PRINTF("current temp = %d\r\n", currentTemperature);
	conversionCompleted = false;

}

void updateTemp(void) {

	//int32_t currentTemperature = 0;

	while(!conversionCompleted){} //may not be necessary since adc is set to trigger faster than 1s

	// Get current Temperature Value
	current_temp = (uint8_t) ( GetCurrentTempValue() & 0xff );
#ifdef DO_PRINT
	PRINTF("current temp = %d\r\n", current_temp);
#endif
	conversionCompleted = false;

}
