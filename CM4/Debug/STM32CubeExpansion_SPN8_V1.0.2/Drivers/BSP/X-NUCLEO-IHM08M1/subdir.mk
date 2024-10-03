################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/X-NUCLEO-IHM08M1.c 

OBJS += \
./STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/X-NUCLEO-IHM08M1.o 

C_DEPS += \
./STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/X-NUCLEO-IHM08M1.d 


# Each subdirectory must supply rules for building sources it contributes
STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/%.o STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/%.su STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/%.cyclo: ../STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/%.c STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/Components/Common" -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/Components/l6398" -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1" -I"/home/pranayt/STM32CubeIDE/Asix_test/CM4/STM32CubeExpansion_SPN8_V1.0.2/Middlewares/ST/MC_6Step_Lib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32CubeExpansion_SPN8_V1-2e-0-2e-2-2f-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-IHM08M1

clean-STM32CubeExpansion_SPN8_V1-2e-0-2e-2-2f-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-IHM08M1:
	-$(RM) ./STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/X-NUCLEO-IHM08M1.cyclo ./STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/X-NUCLEO-IHM08M1.d ./STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/X-NUCLEO-IHM08M1.o ./STM32CubeExpansion_SPN8_V1.0.2/Drivers/BSP/X-NUCLEO-IHM08M1/X-NUCLEO-IHM08M1.su

.PHONY: clean-STM32CubeExpansion_SPN8_V1-2e-0-2e-2-2f-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-IHM08M1

