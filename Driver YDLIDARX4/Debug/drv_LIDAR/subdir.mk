################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drv_LIDAR/drv_LIDAR.c 

OBJS += \
./drv_LIDAR/drv_LIDAR.o 

C_DEPS += \
./drv_LIDAR/drv_LIDAR.d 


# Each subdirectory must supply rules for building sources it contributes
drv_LIDAR/%.o drv_LIDAR/%.su drv_LIDAR/%.cyclo: ../drv_LIDAR/%.c drv_LIDAR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F410Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tomch/Documents/ENSEA/2023-2024/Projet/ST/drv_LIDAR" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drv_LIDAR

clean-drv_LIDAR:
	-$(RM) ./drv_LIDAR/drv_LIDAR.cyclo ./drv_LIDAR/drv_LIDAR.d ./drv_LIDAR/drv_LIDAR.o ./drv_LIDAR/drv_LIDAR.su

.PHONY: clean-drv_LIDAR

