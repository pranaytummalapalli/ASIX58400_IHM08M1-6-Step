################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/6Step_Lib.c 

OBJS += \
./STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/6Step_Lib.o 

C_DEPS += \
./STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/6Step_Lib.d 


# Each subdirectory must supply rules for building sources it contributes
STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/%.o STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/%.su STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/%.cyclo: ../STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/%.c STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2" -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Inc" -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1" -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/Components" -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/Components/l6398" -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32CubeExpansion_SPN8_V1-2e-0-2e-2-2f-Middlewares-2f-ST-2f-MC_6Step_Lib-2f-Src

clean-STM32CubeExpansion_SPN8_V1-2e-0-2e-2-2f-Middlewares-2f-ST-2f-MC_6Step_Lib-2f-Src:
	-$(RM) ./STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/6Step_Lib.cyclo ./STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/6Step_Lib.d ./STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/6Step_Lib.o ./STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Src/6Step_Lib.su

.PHONY: clean-STM32CubeExpansion_SPN8_V1-2e-0-2e-2-2f-Middlewares-2f-ST-2f-MC_6Step_Lib-2f-Src

