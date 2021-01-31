################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Project_library/encoder_driver.c \
../Project_library/step_driver.c \
../Project_library/uart_driver.c 

OBJS += \
./Project_library/encoder_driver.o \
./Project_library/step_driver.o \
./Project_library/uart_driver.o 

C_DEPS += \
./Project_library/encoder_driver.d \
./Project_library/step_driver.d \
./Project_library/uart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Project_library/encoder_driver.o: ../Project_library/encoder_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I/STM32_STEP_CONTROLLER_EMBEDDED_C_CODE/Project_library -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Project_library/encoder_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Project_library/step_driver.o: ../Project_library/step_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I/STM32_STEP_CONTROLLER_EMBEDDED_C_CODE/Project_library -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Project_library/step_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Project_library/uart_driver.o: ../Project_library/uart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I/STM32_STEP_CONTROLLER_EMBEDDED_C_CODE/Project_library -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Project_library/uart_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

