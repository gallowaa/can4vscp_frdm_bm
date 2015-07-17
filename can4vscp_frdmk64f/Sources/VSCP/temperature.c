

#include "main.h"
#include "temperature.h"

uint8_t Celsius2Kelvin(int celsius) {
	return 0; // return (celsius + 275) /* todo: need MSB + LSB */
}

uint8_t Celsius2Fahrenheit(int celsius) {
	return (celsius * 9 / 5) + 32;
}

