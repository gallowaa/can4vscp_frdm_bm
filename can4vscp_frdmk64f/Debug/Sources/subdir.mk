################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/external_vscp_func.c \
../Sources/flash_al.c \
../Sources/flexcan.c \
../Sources/fsl_irq.c \
../Sources/main.c \
../Sources/spi.c \
../Sources/vscp_firmware.c 

OBJS += \
./Sources/external_vscp_func.o \
./Sources/flash_al.o \
./Sources/flexcan.o \
./Sources/fsl_irq.o \
./Sources/main.o \
./Sources/spi.o \
./Sources/vscp_firmware.o 

C_DEPS += \
./Sources/external_vscp_func.d \
./Sources/flash_al.d \
./Sources/flexcan.d \
./Sources/fsl_irq.d \
./Sources/main.d \
./Sources/spi.d \
./Sources/vscp_firmware.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -D"CPU_MK64FN1M0VMD12" -I"../Sources" -I"C:\Users\Angus\Documents\GitHub\vscp_firmware\common" -I"../Sources/Board" -I"../Sources/Utilities" -I"../Project_Settings/Startup_Code" -I"../SDK/platform/CMSIS/Include" -I"../SDK/platform/devices" -I"../SDK/platform/devices/MK64F12/include" -I"C:\Freescale\KSDK_1.2.0/platform/utilities/inc" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/inc" -I"C:\Freescale\KSDK_1.2.0/platform/osa/inc" -I"C:\Freescale\KSDK_1.2.0/platform/hal/inc" -I"C:\Freescale\KSDK_1.2.0/platform/utilities/src" -I"C:\Freescale\KSDK_1.2.0/platform/system/inc" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/lptmr" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash/C90TFS/drvsrc/source" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash/C90TFS/drvsrc/include" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flexcan" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


