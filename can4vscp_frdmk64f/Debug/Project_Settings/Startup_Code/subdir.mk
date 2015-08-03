################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Project_Settings/Startup_Code/startup.c \
../Project_Settings/Startup_Code/system_MK64F12.c 

S_UPPER_SRCS += \
../Project_Settings/Startup_Code/startup_MK64F12.S 

OBJS += \
./Project_Settings/Startup_Code/startup.o \
./Project_Settings/Startup_Code/startup_MK64F12.o \
./Project_Settings/Startup_Code/system_MK64F12.o 

C_DEPS += \
./Project_Settings/Startup_Code/startup.d \
./Project_Settings/Startup_Code/system_MK64F12.d 

S_UPPER_DEPS += \
./Project_Settings/Startup_Code/startup_MK64F12.d 


# Each subdirectory must supply rules for building sources it contributes
Project_Settings/Startup_Code/%.o: ../Project_Settings/Startup_Code/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -D"CPU_MK64FN1M0VMD12" -I"../Sources" -I"../Sources/Adc16" -I"../Sources/VSCP" -I"../Sources/Interrupt" -I"../Sources/SPI" -I"C:\Users\Angus\Documents\GitHub\vscp_firmware\common" -I"../Sources/Board" -I"../Sources/Flexcan" -I"../Sources/Flash" -I"../Sources/Utilities" -I"../Sources/Accel" -I"../Project_Settings/Startup_Code" -I"../SDK/platform/CMSIS/Include" -I"../SDK/platform/devices" -I"../SDK/platform/devices/MK64F12/include" -I"C:\Freescale\KSDK_1.2.0/platform/utilities/inc" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/inc" -I"C:\Freescale\KSDK_1.2.0/platform/osa/inc" -I"C:\Freescale\KSDK_1.2.0/platform/hal/inc" -I"C:\Freescale\KSDK_1.2.0/platform/utilities/src" -I"C:\Freescale\KSDK_1.2.0/platform/system/inc" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/lptmr" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash/C90TFS/drvsrc/source" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash/C90TFS/drvsrc/include" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flexcan" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/i2c" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Project_Settings/Startup_Code/%.o: ../Project_Settings/Startup_Code/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


