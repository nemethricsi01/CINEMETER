################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FLASH_SECTOR_F4.c \
../Core/Src/VL53L1X.c \
../Core/Src/VL53L1X_api.c \
../Core/Src/VL53L1X_calibration.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/vl53l1_platform.c 

OBJS += \
./Core/Src/FLASH_SECTOR_F4.o \
./Core/Src/VL53L1X.o \
./Core/Src/VL53L1X_api.o \
./Core/Src/VL53L1X_calibration.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/vl53l1_platform.o 

C_DEPS += \
./Core/Src/FLASH_SECTOR_F4.d \
./Core/Src/VL53L1X.d \
./Core/Src/VL53L1X_api.d \
./Core/Src/VL53L1X_calibration.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers/NRF24" -I../Core/Inc -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers/NRF24_v2" -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers/OLED" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers" -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Core/Src" -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/F4 SERIES" -I"/Users/constantiniandolino/STM32CubeIDE/workspace_1.10.1/CINEMETER/Drivers/VL53L1X" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FLASH_SECTOR_F4.d ./Core/Src/FLASH_SECTOR_F4.o ./Core/Src/FLASH_SECTOR_F4.su ./Core/Src/VL53L1X.d ./Core/Src/VL53L1X.o ./Core/Src/VL53L1X.su ./Core/Src/VL53L1X_api.d ./Core/Src/VL53L1X_api.o ./Core/Src/VL53L1X_api.su ./Core/Src/VL53L1X_calibration.d ./Core/Src/VL53L1X_calibration.o ./Core/Src/VL53L1X_calibration.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/vl53l1_platform.d ./Core/Src/vl53l1_platform.o ./Core/Src/vl53l1_platform.su

.PHONY: clean-Core-2f-Src

