################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f405rgtx.s 

S_DEPS += \
./Core/Startup/startup_stm32f405rgtx.d 

OBJS += \
./Core/Startup/startup_stm32f405rgtx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/DATA/Projects/Amon_Lander/Amon_board_firmware/Drivers/vl53l1x" -I"C:/DATA/Projects/Amon_Lander/Amon_board_firmware/Drivers" -I"C:/DATA/Projects/Amon_Lander/Amon_board_firmware/Drivers/vl53l1x/core" -I"C:/DATA/Projects/Amon_Lander/Amon_board_firmware/Drivers/vl53l1x/platform" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f405rgtx.d ./Core/Startup/startup_stm32f405rgtx.o

.PHONY: clean-Core-2f-Startup

