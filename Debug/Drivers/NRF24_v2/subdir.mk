################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/NRF24_v2/NRF24L01.c 

OBJS += \
./Drivers/NRF24_v2/NRF24L01.o 

C_DEPS += \
./Drivers/NRF24_v2/NRF24L01.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/NRF24_v2/%.o Drivers/NRF24_v2/%.su: ../Drivers/NRF24_v2/%.c Drivers/NRF24_v2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers/NRF24_v2" -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers/OLED" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers" -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Core/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-NRF24_v2

clean-Drivers-2f-NRF24_v2:
	-$(RM) ./Drivers/NRF24_v2/NRF24L01.d ./Drivers/NRF24_v2/NRF24L01.o ./Drivers/NRF24_v2/NRF24L01.su

.PHONY: clean-Drivers-2f-NRF24_v2

