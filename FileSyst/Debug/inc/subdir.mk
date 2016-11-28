################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../inc/IndicationGPIOs.c \
../inc/SPI1.c \
../inc/SysTickDelay.c \
../inc/sdCard.c 

OBJS += \
./inc/IndicationGPIOs.o \
./inc/SPI1.o \
./inc/SysTickDelay.o \
./inc/sdCard.o 

C_DEPS += \
./inc/IndicationGPIOs.d \
./inc/SPI1.d \
./inc/SysTickDelay.d \
./inc/sdCard.d 


# Each subdirectory must supply rules for building sources it contributes
inc/%.o: ../inc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F051R8Tx -DSTM32F0 -DSTM32 -DSTM32F0DISCOVERY -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F051 -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib" -I"/home/niks/stm32workspace/FileSyst/inc" -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib/StdPeriph_Driver/inc" -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib/CMSIS/device" -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib/CMSIS/core" -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib/Utilities" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


