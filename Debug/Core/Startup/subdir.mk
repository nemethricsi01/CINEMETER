################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f401rctx.s 

OBJS += \
./Core/Startup/startup_stm32f401rctx.o 

S_DEPS += \
./Core/Startup/startup_stm32f401rctx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers/VL53L1X" -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers" -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Core/Src" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f401rctx.d ./Core/Startup/startup_stm32f401rctx.o

.PHONY: clean-Core-2f-Startup
