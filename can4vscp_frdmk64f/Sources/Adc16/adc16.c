/*
 * adc16.c
 *
 *  Created on: Jul 13, 2015
 *      Author: Angus
 *
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "adc16.h"

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

///////////////////////////////////////////////////////////////////////////////
// Prototypes
///////////////////////////////////////////////////////////////////////////////

extern void init_trigger_source(uint32_t instance);

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////

volatile static uint32_t adcValue = 0;      /*! ADC value */
static uint32_t adcrTemp25 = 0;             /*! Calibrated ADCR_TEMP25 */
static uint32_t adcr100m = 0;
volatile bool conversionCompleted = false;  /*! Conversion is completed Flag */
SIM_Type * gSimBase[] = SIM_BASE_PTRS;
static smc_power_mode_config_t smcConfig;

///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////


/* ADC Interrupt Handler */
void ADC1IRQHandler(void)
{
    // Get current ADC value
    adcValue = ADC_TEST_GetConvValueRAWInt (ADC_0, CHANNEL_0);
    // Set conversionCompleted flag. This prevents an wrong conversion in main function
    conversionCompleted = true;
}


/*!
 * Parameters calibration: VDD and ADCR_TEMP25
 */
void calibrateParams(void)
{
    adc16_hw_average_config_t userHwAverageConfig;
#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif
    adc16_converter_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfig;
    pmc_bandgap_buffer_config_t pmcBandgapConfig = {
        .enable = true,
#if FSL_FEATURE_PMC_HAS_BGEN
        .enableInLowPower = false,
#endif

    };
    uint32_t bandgapValue = 0;  /*! ADC value of BANDGAP */
    uint32_t vdd = 0;           /*! VDD in mV */

    // Initialization ADC for
    // 16bit resolution, interrupt mode, hw trigger disabled.
    // normal convert speed, VREFH/L as reference,
    // disable continuous convert mode.
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.resolution = kAdc16ResolutionBitOf16;
    adcUserConfig.continuousConvEnable = false;
    adcUserConfig.clkSrc = kAdc16ClkSrcOfAsynClk;

    ADC16_DRV_Init(ADC_0, &adcUserConfig);

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration
    ADC16_DRV_GetAutoCalibrationParam(ADC_0, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC_0, &adcCalibraitionParam);
#endif

    // Enable BANDGAP reference voltage
    PMC_HAL_BandgapBufferConfig(PMC_BASE_PTR, &pmcBandgapConfig);


#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    // Use hardware average to increase stability of the measurement.
    userHwAverageConfig.hwAverageEnable = true;
    userHwAverageConfig.hwAverageCountMode = kAdc16HwAverageCountOf32;
    ADC16_DRV_ConfigHwAverage(ADC_0, &userHwAverageConfig);
#endif // FSL_FEATURE_ADC16_HAS_HW_AVERAGE

    adcChnConfig.chnIdx                  = kAdcChannelBandgap;
    adcChnConfig.diffConvEnable          = false;
    adcChnConfig.convCompletedIntEnable  = false;
    ADC16_DRV_ConfigConvChn(ADC_0, CHANNEL_0, &adcChnConfig);

    // Wait for the conversion to be done
    ADC16_DRV_WaitConvDone(ADC_0, CHANNEL_0);

    // Get current ADC BANDGAP value
    bandgapValue = ADC16_DRV_GetConvValueSigned(ADC_0, CHANNEL_0);

    // ADC stop conversion
    ADC16_DRV_PauseConv(ADC_0, CHANNEL_0);

    // Get VDD value measured in mV: VDD = (ADCR_VDD x V_BG) / ADCR_BG
    vdd = ADCR_VDD * V_BG / bandgapValue;
    // Calibrate ADCR_TEMP25: ADCR_TEMP25 = ADCR_VDD x V_TEMP25 / VDD
    adcrTemp25 = ADCR_VDD * V_TEMP25 / vdd;
    // ADCR_100M = ADCR_VDD x M x 100 / VDD
    adcr100m = (ADCR_VDD * M) / (vdd * 10);

    // Disable BANDGAP reference voltage
    pmcBandgapConfig.enable = false;
    PMC_HAL_BandgapBufferConfig(PMC_BASE_PTR, &pmcBandgapConfig);
}
/*!
 * @brief Initialize the ADCx for HW trigger.
 *
 * @param instance The ADC instance number
 */
void init_adc(uint32_t instance)
{
#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif
    adc16_converter_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfig;

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration
    ADC16_DRV_GetAutoCalibrationParam(instance, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(instance, &adcCalibraitionParam);
#endif

    // Initialization ADC for
    // 16bit resolution, interrupt mode, hw trigger enabled.
    // normal convert speed, VREFH/L as reference,
    // disable continuous convert mode.
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.resolution = kAdc16ResolutionBitOf16;
    adcUserConfig.hwTriggerEnable = true;
    adcUserConfig.continuousConvEnable = false;
    adcUserConfig.clkSrc = kAdc16ClkSrcOfAsynClk;

    ADC16_DRV_Init(instance, &adcUserConfig);

    // Install Callback function into ISR
    ADC_TEST_InstallCallback(instance, CHANNEL_0, ADC1IRQHandler);

    adcChnConfig.chnIdx                  = kAdcChannelTemperature;
    adcChnConfig.diffConvEnable          = false;
    adcChnConfig.convCompletedIntEnable  = true;
    // Configure channel0
    ADC16_DRV_ConfigConvChn(instance, CHANNEL_0, &adcChnConfig);

}

/* Calculate the current temperature */
int32_t GetCurrentTempValue(void)
{
    int32_t currentTemperature = 0;

    /* Temperature = 25 - (ADCR_T - ADCR_TEMP25) * 100 / ADCR_100M */
    currentTemperature = (int32_t)(STANDARD_TEMP - ((int32_t)adcValue - (int32_t)adcrTemp25) * 100 / (int32_t)adcr100m);

    return currentTemperature;
}

/* Calculate the average temperature and set boundaries */
#ifdef NO
lowPowerAdcBoundaries_t TempSensorCalibration(uint32_t updateBoundariesCounter,
                                                     int32_t *tempArray)
{
    uint32_t avgTemp = 0;
    lowPowerAdcBoundaries_t boundaries;

    for(int i = 0; i < updateBoundariesCounter; i++)
    {
        avgTemp += tempArray[i];
    }
    // Get average temperature
    avgTemp /= updateBoundariesCounter;

    // Set upper boundary
    boundaries.upperBoundary = avgTemp + UPPER_VALUE_LIMIT;

    // Set lower boundary
    boundaries.lowerBoundary = avgTemp - LOWER_VALUE_LIMIT;

    return boundaries;
}

/*!
 * @brief main function
 */

int main(void)
{
    int32_t currentTemperature = 0;
    uint32_t updateBoundariesCounter = 0;
    int32_t tempArray[UPDATE_BOUNDARIES_TIME * 2];
    lowPowerAdcBoundaries_t boundaries;

    // Init hardware
    hardware_init();

    // Initialize GPIO pins
    GPIO_DRV_Init(NULL, ledPins);

    // Configure SMC module to entry to vlps mode
    config_vlps();

    // Calibrate param Temperature sensor
    calibrateParams();

    // Initialize ADC
    if (init_adc(ADC_0))
    {
        PRINTF("Failed to do the ADC init\n");
        return -1;
    }

    // Show the currentTemperature value
    PRINTF("\r\n ADC LOW POWER DEMO \r\n\n\n");
    PRINTF("\r\nThe Low Power ADC project is designed to work with the Tower System or in a stand alone setting. \n\n");
    PRINTF("\r 1. Set your target board in a place where the temperature is constant.  \n");
#ifndef FREEDOM
    PRINTF("\r 2. Wait until the green LED light turns on. \n");
#else
    PRINTF("\r 2. Wait until the white LED light turns on. \n");
#endif
    PRINTF("\r 3. Increment or decrement the temperature to see the changes. \n");

    // setup the HW trigger source
    init_trigger_source(ADC_0);

#ifdef NOO
    // Warm up microcontroller and allow to set first boundaries
    while(updateBoundariesCounter < (UPDATE_BOUNDARIES_TIME * 2))
    {
        while(!conversionCompleted);
        currentTemperature = GetCurrentTempValue();
        tempArray[updateBoundariesCounter] = currentTemperature;
        updateBoundariesCounter++;
        conversionCompleted = false;
    }
#endif

    // Temp Sensor Calibration
    boundaries = TempSensorCalibration(updateBoundariesCounter, tempArray);
    updateBoundariesCounter = 0;

#if (!defined (FREEDOM) && !defined (DONGLE))
    // Green LED is turned on indicating calibration is done
    GPIO_DRV_WritePinOutput(BOARD_GPIO_LED_GREEN, LED_ON);
#endif

    // Wait for user input before beginning demo
    PRINTF("\n\n\rEnter any character to begin...\n\n\r");
    GETCHAR();

    for(;;)
    {
        // Prevents the use of wrong values
        while(!conversionCompleted)
        {}

        // Get current Temperature Value
        currentTemperature = GetCurrentTempValue();
        currentTemperature = (uint8_t)(currentTemperature & 0xff);
        PRINTF("current temp = %d\r", currentTemperature);

        // Store temperature values that are going to be use to calculate average temperature
        tempArray[updateBoundariesCounter] = currentTemperature;

        // Clear conversionCompleted flag
        conversionCompleted = false;

        // Entry to Low Power Mode
        // Once this mode exited, it will no longer be in PEE mode (assuming
        // the device entered this mode from PEE).  Therefore, the UART
        // baud rate will not be correct because the device's operating
        // frequency will be different from the startup of the demo.
        // SMC_HAL_SetMode(SMC, &smcConfig);
    }
}
#endif /* main() */




