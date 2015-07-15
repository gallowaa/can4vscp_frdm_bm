################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Test/test_adc.c \
../Sources/Test/test_flexcan_functions.c \
../Sources/Test/test_spi_functions.c \
../Sources/Test/test_vscp_functions.c 

OBJS += \
./Sources/Test/test_adc.o \
./Sources/Test/test_flexcan_functions.o \
./Sources/Test/test_spi_functions.o \
./Sources/Test/test_vscp_functions.o 

C_DEPS += \
./Sources/Test/test_adc.d \
./Sources/Test/test_flexcan_functions.d \
./Sources/Test/test_spi_functions.d \
./Sources/Test/test_vscp_functions.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/Test/%.o: ../Sources/Test/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -D"CPU_MK64FN1M0VMD12" -I"../Sources" -I"../Sources/Adc16" -I"../Sources/Interrupt" -I"../Sources/SPI" -I"C:\Users\Angus\Documents\GitHub\vscp_firmware\common" -I"../Sources/Board" -I"../Sources/Flexcan" -I"../Sources/Flash_Driver" -I"../Sources/Utilities" -I"../Sources/Accel" -I"../Project_Settings/Startup_Code" -I"../SDK/platform/CMSIS/Include" -I"../SDK/platform/devices" -I"../SDK/platform/devices/MK64F12/include" -I"C:\Freescale\KSDK_1.2.0/platform/utilities/inc" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/inc" -I"C:\Freescale\KSDK_1.2.0/platform/osa/inc" -I"C:\Freescale\KSDK_1.2.0/platform/hal/inc" -I"C:\Freescale\KSDK_1.2.0/platform/utilities/src" -I"C:\Freescale\KSDK_1.2.0/platform/system/inc" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/lptmr" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash/C90TFS/drvsrc/source" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash/C90TFS/drvsrc/include" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flexcan" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/i2c" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


