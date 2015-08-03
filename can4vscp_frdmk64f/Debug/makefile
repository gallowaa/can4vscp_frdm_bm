################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Sources/VSCP/subdir.mk
-include Sources/Utilities/subdir.mk
-include Sources/Test/subdir.mk
-include Sources/Spi/subdir.mk
-include Sources/Interrupt/subdir.mk
-include Sources/Flexcan/subdir.mk
-include Sources/Flash/subdir.mk
-include Sources/Board/subdir.mk
-include Sources/Adc16/subdir.mk
-include Sources/Accel/subdir.mk
-include Sources/subdir.mk
-include Project_Settings/Startup_Code/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
can4vscp_frdmk64f.hex \

SECONDARY_SIZE += \
can4vscp_frdmk64f.siz \


# All Target
all: can4vscp_frdmk64f.elf secondary-outputs

# Tool invocations
can4vscp_frdmk64f.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -T "MK64FN1M0xxx12_flash.ld" -Xlinker --gc-sections -L"C:/Users/Angus/Documents/KDSWorkspace/can4vscp_frdmk64f/Project_Settings/Linker_Files" -L"C:\Freescale\KSDK_1.2.0\lib\ksdk_platform_lib\kds\K64F12\debug" -Wl,-Map,"can4vscp_frdmk64f.map" -specs=nosys.specs -specs=nano.specs -Xlinker --defsym=__ram_vector_table__=1 -o "can4vscp_frdmk64f.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

can4vscp_frdmk64f.hex: can4vscp_frdmk64f.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "can4vscp_frdmk64f.elf"  "can4vscp_frdmk64f.hex"
	@echo 'Finished building: $@'
	@echo ' '

can4vscp_frdmk64f.siz: can4vscp_frdmk64f.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley --totals "can4vscp_frdmk64f.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(C++_DEPS)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(CC_DEPS)$(SECONDARY_FLASH)$(CPP_DEPS)$(CXX_DEPS)$(C_UPPER_DEPS)$(S_UPPER_DEPS) can4vscp_frdmk64f.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
