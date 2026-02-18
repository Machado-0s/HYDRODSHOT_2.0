################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/myCode/hydrv_time.cpp \
../Core/myCode/main.cpp 

C_SRCS += \
../Core/myCode/hydrolib_ring_queue.c 

C_DEPS += \
./Core/myCode/hydrolib_ring_queue.d 

OBJS += \
./Core/myCode/hydrolib_ring_queue.o \
./Core/myCode/hydrv_time.o \
./Core/myCode/main.o 

CPP_DEPS += \
./Core/myCode/hydrv_time.d \
./Core/myCode/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/myCode/%.o Core/myCode/%.su Core/myCode/%.cyclo: ../Core/myCode/%.c Core/myCode/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -u _scanf_float -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/myCode/%.o Core/myCode/%.su Core/myCode/%.cyclo: ../Core/myCode/%.cpp Core/myCode/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++20 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Core/myCode -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-myCode

clean-Core-2f-myCode:
	-$(RM) ./Core/myCode/hydrolib_ring_queue.cyclo ./Core/myCode/hydrolib_ring_queue.d ./Core/myCode/hydrolib_ring_queue.o ./Core/myCode/hydrolib_ring_queue.su ./Core/myCode/hydrv_time.cyclo ./Core/myCode/hydrv_time.d ./Core/myCode/hydrv_time.o ./Core/myCode/hydrv_time.su ./Core/myCode/main.cyclo ./Core/myCode/main.d ./Core/myCode/main.o ./Core/myCode/main.su

.PHONY: clean-Core-2f-myCode

