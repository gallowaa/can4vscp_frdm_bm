/*
 * accel.c
 *
 *  Created on: Jul 10, 2015
 *      Author: Angus
 */

#include "main.h"

//void checkAngle(fxos_handler_t i2cModule, fxos_data_t sensorData) {


//accel_data_t getAngle() {
//uint8_t getAngle(){
accel_data_t getAngle(fxos_handler_t i2cModule){

	fxos_data_t sensorData;
	//fxos_handler_t i2cModule;
	int16_t xData, yData;
	int16_t xAngl16, yAngl16;
	uint8_t x, y;
	uint32_t ftmModulo;
	accel_data_t accel;

	// Initialize the eCompass.
	//i2cModule.i2cInstance = BOARD_I2C_COMM_INSTANCE;
	//FXOS_Init(&i2cModule, NULL);


	// Wait 5 ms in between samples (accelerometer updates at 200Hz).
	OSA_TimeDelay(5);

	// Get new accelerometer data.
	FXOS_ReadData(&i2cModule, &sensorData);

	// Turn off interrupts (FTM) while updating new duty cycle values.
	//INT_SYS_DisableIRQGlobal();

	// Get the X and Y data from the sensor data structure.
	xData = (int16_t)((sensorData.accelXMSB << 8) | sensorData.accelXLSB);
	yData = (int16_t)((sensorData.accelYMSB << 8) | sensorData.accelYLSB);

	// Convert raw data to angle (normalize to 0-90 degrees).  No negative angles.
	xAngl16 = abs((int16_t)(xData * 0.011));
	yAngl16 = abs((int16_t)(yData * 0.011));

	PRINTF("X = %d, Y = %d \r\n", xAngl16, yAngl16);
/*
	x = (uint8_t)((xAngl16) & 0xff);
	y = (uint8_t)((yAngl16) & 0xff);

	*/

	accel.xAngle = (uint8_t)((xAngl16) & 0xff);
	accel.yAngle = (uint8_t)((yAngl16) & 0xff);

	PRINTF("X = %d, Y = %d \r\n", accel.xAngle, accel.yAngle);
	return accel;

}
