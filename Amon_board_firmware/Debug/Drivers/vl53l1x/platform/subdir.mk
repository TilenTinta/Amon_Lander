################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/vl53l1x/platform/vl53l1_platform.c 

C_DEPS += \
./Drivers/vl53l1x/platform/vl53l1_platform.d 

OBJS += \
./Drivers/vl53l1x/platform/vl53l1_platform.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/vl53l1x/platform/%.o Drivers/vl53l1x/platform/%.su Drivers/vl53l1x/platform/%.cyclo: ../Drivers/vl53l1x/platform/%.c Drivers/vl53l1x/platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Drivers/vl53l1x/platform/inc -I../Drivers/vl53l1x/core/inc -I../Drivers/vl53l1x/platform/src -I../Drivers/vl53l1x/core/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-vl53l1x-2f-platform

clean-Drivers-2f-vl53l1x-2f-platform:
	-$(RM) ./Drivers/vl53l1x/platform/vl53l1_platform.cyclo ./Drivers/vl53l1x/platform/vl53l1_platform.d ./Drivers/vl53l1x/platform/vl53l1_platform.o ./Drivers/vl53l1x/platform/vl53l1_platform.su

.PHONY: clean-Drivers-2f-vl53l1x-2f-platform

