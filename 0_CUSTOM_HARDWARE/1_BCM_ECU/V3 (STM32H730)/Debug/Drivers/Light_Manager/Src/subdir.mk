################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Light_Manager/Src/light_manager.c 

OBJS += \
./Drivers/Light_Manager/Src/light_manager.o 

C_DEPS += \
./Drivers/Light_Manager/Src/light_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Light_Manager/Src/%.o Drivers/Light_Manager/Src/%.su Drivers/Light_Manager/Src/%.cyclo: ../Drivers/Light_Manager/Src/%.c Drivers/Light_Manager/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H730xx -c -I../Core/Inc -I../Drivers/MCP23017/Inc -I../Drivers/Light_Manager/Inc -I../Drivers/State_Manager/Inc -I../Drivers/TCAN1146/Inc -I../Drivers/ADC108S022/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Light_Manager-2f-Src

clean-Drivers-2f-Light_Manager-2f-Src:
	-$(RM) ./Drivers/Light_Manager/Src/light_manager.cyclo ./Drivers/Light_Manager/Src/light_manager.d ./Drivers/Light_Manager/Src/light_manager.o ./Drivers/Light_Manager/Src/light_manager.su

.PHONY: clean-Drivers-2f-Light_Manager-2f-Src

