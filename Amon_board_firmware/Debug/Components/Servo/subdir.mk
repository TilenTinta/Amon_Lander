################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/Servo/Servo.c \
../Components/Servo/Servo_cfg.c 

C_DEPS += \
./Components/Servo/Servo.d \
./Components/Servo/Servo_cfg.d 

OBJS += \
./Components/Servo/Servo.o \
./Components/Servo/Servo_cfg.o 


# Each subdirectory must supply rules for building sources it contributes
Components/Servo/%.o Components/Servo/%.su: ../Components/Servo/%.c Components/Servo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I"C:/DATA/Projects/Amon_Lander/Amon_board_firmware/Components" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-Servo

clean-Components-2f-Servo:
	-$(RM) ./Components/Servo/Servo.d ./Components/Servo/Servo.o ./Components/Servo/Servo.su ./Components/Servo/Servo_cfg.d ./Components/Servo/Servo_cfg.o ./Components/Servo/Servo_cfg.su

.PHONY: clean-Components-2f-Servo

