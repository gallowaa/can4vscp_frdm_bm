################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Flash_Driver/CopyToRam.c \
../Sources/Flash_Driver/FlashCommandSequence.c \
../Sources/Flash_Driver/FlashEraseSector.c \
../Sources/Flash_Driver/FlashGetSecurityState.c \
../Sources/Flash_Driver/FlashInit.c \
../Sources/Flash_Driver/FlashProgram.c \
../Sources/Flash_Driver/FlashProgramCheck.c \
../Sources/Flash_Driver/FlashVerifySection.c 

OBJS += \
./Sources/Flash_Driver/CopyToRam.o \
./Sources/Flash_Driver/FlashCommandSequence.o \
./Sources/Flash_Driver/FlashEraseSector.o \
./Sources/Flash_Driver/FlashGetSecurityState.o \
./Sources/Flash_Driver/FlashInit.o \
./Sources/Flash_Driver/FlashProgram.o \
./Sources/Flash_Driver/FlashProgramCheck.o \
./Sources/Flash_Driver/FlashVerifySection.o 

C_DEPS += \
./Sources/Flash_Driver/CopyToRam.d \
./Sources/Flash_Driver/FlashCommandSequence.d \
./Sources/Flash_Driver/FlashEraseSector.d \
./Sources/Flash_Driver/FlashGetSecurityState.d \
./Sources/Flash_Driver/FlashInit.d \
./Sources/Flash_Driver/FlashProgram.d \
./Sources/Flash_Driver/FlashProgramCheck.d \
./Sources/Flash_Driver/FlashVerifySection.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/Flash_Driver/%.o: ../Sources/Flash_Driver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -D"CPU_MK64FN1M0VMD12" -I"../Sources" -I"C:\Users\Angus\Documents\GitHub\vscp_firmware\common" -I"../Sources/Board" -I"../Sources/Utilities" -I"../Project_Settings/Startup_Code" -I"../SDK/platform/CMSIS/Include" -I"../SDK/platform/devices" -I"../SDK/platform/devices/MK64F12/include" -I"C:\Freescale\KSDK_1.2.0/platform/utilities/inc" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/inc" -I"C:\Freescale\KSDK_1.2.0/platform/osa/inc" -I"C:\Freescale\KSDK_1.2.0/platform/hal/inc" -I"C:\Freescale\KSDK_1.2.0/platform/utilities/src" -I"C:\Freescale\KSDK_1.2.0/platform/system/inc" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/lptmr" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash/C90TFS/drvsrc/source" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flash/C90TFS/drvsrc/include" -I"C:\Freescale\KSDK_1.2.0/platform/drivers/src/flexcan" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


