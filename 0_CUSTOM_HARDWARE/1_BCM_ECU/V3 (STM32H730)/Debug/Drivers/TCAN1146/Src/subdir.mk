################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/TCAN1146/Src/TCAN1146.c 

OBJS += \
./Drivers/TCAN1146/Src/TCAN1146.o 

C_DEPS += \
./Drivers/TCAN1146/Src/TCAN1146.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/TCAN1146/Src/%.o Drivers/TCAN1146/Src/%.su Drivers/TCAN1146/Src/%.cyclo: ../Drivers/TCAN1146/Src/%.c Drivers/TCAN1146/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H730xx -c -I../Core/Inc -I../Drivers/MCP23017/Inc -I../Drivers/Light_Manager/Inc -I../Drivers/State_Manager/Inc -I../Drivers/TCAN1146/Inc -I../Drivers/ADC108S022/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-TCAN1146-2f-Src

clean-Drivers-2f-TCAN1146-2f-Src:
	-$(RM) ./Drivers/TCAN1146/Src/TCAN1146.cyclo ./Drivers/TCAN1146/Src/TCAN1146.d ./Drivers/TCAN1146/Src/TCAN1146.o ./Drivers/TCAN1146/Src/TCAN1146.su

.PHONY: clean-Drivers-2f-TCAN1146-2f-Src

