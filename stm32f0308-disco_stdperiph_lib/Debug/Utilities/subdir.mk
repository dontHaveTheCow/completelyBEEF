################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/stm32f0308_discovery.c 

OBJS += \
./Utilities/stm32f0308_discovery.o 

C_DEPS += \
./Utilities/stm32f0308_discovery.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/%.o: ../Utilities/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F0 -DSTM32F0308_DISCO -DSTM32F030R8Tx -DSTM32 -DUSE_STDPERIPH_DRIVER -DSTM32F030 -I"/home/niks/stm32workspace/stm32f0308-disco_stdperiph_lib/StdPeriph_Driver/inc" -I"/home/niks/stm32workspace/stm32f0308-disco_stdperiph_lib/CMSIS/device" -I"/home/niks/stm32workspace/stm32f0308-disco_stdperiph_lib/CMSIS/core" -I"/home/niks/stm32workspace/stm32f0308-disco_stdperiph_lib/Utilities" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


