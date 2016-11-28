################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../inc/A2035H.c \
../inc/ADC.c \
../inc/ADXL362.c \
../inc/Button.c \
../inc/IndicationGPIOs.c \
../inc/PWM.c \
../inc/SPI1.c \
../inc/SPI2.c \
../inc/SysTickDelay.c \
../inc/Timer.c \
../inc/USART1.c \
../inc/USART2.c \
../inc/XBee.c \
../inc/myStringFunctions.c \
../inc/sdCard.c \
../inc/serial.c 

OBJS += \
./inc/A2035H.o \
./inc/ADC.o \
./inc/ADXL362.o \
./inc/Button.o \
./inc/IndicationGPIOs.o \
./inc/PWM.o \
./inc/SPI1.o \
./inc/SPI2.o \
./inc/SysTickDelay.o \
./inc/Timer.o \
./inc/USART1.o \
./inc/USART2.o \
./inc/XBee.o \
./inc/myStringFunctions.o \
./inc/sdCard.o \
./inc/serial.o 

C_DEPS += \
./inc/A2035H.d \
./inc/ADC.d \
./inc/ADXL362.d \
./inc/Button.d \
./inc/IndicationGPIOs.d \
./inc/PWM.d \
./inc/SPI1.d \
./inc/SPI2.d \
./inc/SysTickDelay.d \
./inc/Timer.d \
./inc/USART1.d \
./inc/USART2.d \
./inc/XBee.d \
./inc/myStringFunctions.d \
./inc/sdCard.d \
./inc/serial.d 


# Each subdirectory must supply rules for building sources it contributes
inc/%.o: ../inc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F051R8Tx -DSTM32F0 -DSTM32 -DSTM32F0DISCOVERY -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F051 -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib" -I"/home/niks/stm32workspace/serialComplete/inc" -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib/CMSIS/core" -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib/CMSIS/device" -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib/Utilities" -I"/home/niks/stm32workspace/stm32f0discovery_stdperiph_lib/StdPeriph_Driver/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


