/*******************************************************************************
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.                    *
 * All rights reserved.                                                        *
 *                                                                             *
 * Redistribution and use in source and binary forms, with or without modifi-  *
 * cation, are permitted provided that the following conditions are met:       *
 *                                                                             *
 * o Redistributions of source code must retain the above copyright notice,    *
 * this list of conditions and the following disclaimer.                       *
 *                                                                             *
 * o Redistributions in binary form must reproduce the above copyright notice, *
 * this list of conditions and the following disclaimer in the documentation   *
 * and/or other materials provided with the distribution.                      *
 *                                                                             *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its    *
 *   contributors may be used to endorse or promote products derived from this *
 *   software without specific prior written permission.                       *
 *                                                                             *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" *
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   *
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         *
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF        *
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    *
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN     *
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)     *
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE  *
 * POSSIBILITY OF SUCH DAMAGE.                                                 *
 *                                                                             *
 *******************************************************************************/

#ifndef SOURCES_ADC16_ADC16_H_
#define SOURCES_ADC16_ADC16_H_

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "main.h"
#include "fsl_adc16_driver.h"

#ifdef INCLUDED_IN_MAIN_H
// Standard C Included Files
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

// SDK Included Files
#include "fsl_debug_console.h"
//#include "adc_low_power.h"
#include "fsl_smc_hal.h"
#include "fsl_pmc_hal.h"
#include "fsl_adc16_driver.h"
#include "board.h"
#endif

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

#define ADC_0                   (0U)
#define CHANNEL_0               (0U)
#define LED_ON                  (0U)
#define LED_OFF                 (1U)
/*!
 * @brief These values are used to get the temperature. DO NOT MODIFY
 * The method used in this demo to calculate temperature of chip is mapped to
 * Temperature Sensor for the HCS08 Microcontroller Family document (Document Number: AN3031)
 */
#define ADCR_VDD                (65535U)    /*! Maximum value when use 16b resolution */
#define V_BG                    (1000U)     /*! BANDGAP voltage in mV (trim to 1.0V) */
#define V_TEMP25                (716U)      /*! Typical VTEMP25 in mV */
#define M                       (1620U)     /*! Typical slope: (mV x 1000)/oC */
#define STANDARD_TEMP           (25)

#define UPPER_VALUE_LIMIT       (1U)        /*! This value/10 is going to be added to current Temp to set the upper boundary*/
#define LOWER_VALUE_LIMIT       (1U)        /*! This Value/10 is going to be subtracted from current Temp to set the lower boundary*/
#define UPDATE_BOUNDARIES_TIME  (20U)       /*! This value indicates the number of cycles needed to update boundaries. To know the Time it will take, multiply this value times LPTMR_COMPARE_VALUE*/
#define kAdcChannelTemperature  (kAdc16Chn26)       /*! ADC channel of temperature sensor */
#define kAdcChannelBandgap      (kAdc16Chn27)       /*! ADC channel of BANDGAP */

/*!
* @brief Boundaries struct
*/
typedef struct lowPowerAdcBoundaries
{
    int32_t upperBoundary;
    int32_t lowerBoundary;
} lowPowerAdcBoundaries_t;



/*!
 * @brief Initialize the adc for PIT hw trigger
 */
void init_adc(uint32_t instance);

/*!
 * @brief Low Power Timer Interrupt handler. Clear LPT Compare flag.
 */
void LowPowerTimerIRQHandler(void);

/*!
 * @brief ADC Interrupt handler. Get current ADC value and set conversionCompleted flag.
 */
void ADC1IRQHandler(void);

/*!
 * @brief Initialize Low Power Timer. Use 1 kHz LPO with no prescaler and enable LPT interrupt.
 */
void InitLowPowerTmr(void);

/*!
 * @brief Calculate current temperature.
 *
 * @return uint32_t Returns current temperature.
 */
int32_t GetCurrentTempValue(void);

/*!
 * @brief Calculate current temperature.
 *
 * @param updateBoundariesCounter Indicate number of values into tempArray.
 *
 * @param tempArray Store temperature value.
 *
 * @return lowPowerAdcBoundaries_t Returns upper and lower temperature boundaries.
 */
lowPowerAdcBoundaries_t TempSensorCalibration(uint32_t updateBoundariesCounter,
                                                     int32_t *tempArray);

/*!
 * @brief User-defined function to install callback.
 */
void ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) );

/*!
 * @brief User-defined function to read conversion value in ADC ISR.
 */
uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup);



#endif /* SOURCES_ADC16_ADC16_H_ */
