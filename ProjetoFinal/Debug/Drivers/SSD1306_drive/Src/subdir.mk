################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SSD1306_drive/Src/fonts.c \
../Drivers/SSD1306_drive/Src/ssd1306.c 

OBJS += \
./Drivers/SSD1306_drive/Src/fonts.o \
./Drivers/SSD1306_drive/Src/ssd1306.o 

C_DEPS += \
./Drivers/SSD1306_drive/Src/fonts.d \
./Drivers/SSD1306_drive/Src/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SSD1306_drive/Src/%.o Drivers/SSD1306_drive/Src/%.su Drivers/SSD1306_drive/Src/%.cyclo: ../Drivers/SSD1306_drive/Src/%.c Drivers/SSD1306_drive/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Ygor de Almeida/STM32CubeIDE/workspace_1.16.0/ProjetoFinal/Drivers/SSD1306_drive/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SSD1306_drive-2f-Src

clean-Drivers-2f-SSD1306_drive-2f-Src:
	-$(RM) ./Drivers/SSD1306_drive/Src/fonts.cyclo ./Drivers/SSD1306_drive/Src/fonts.d ./Drivers/SSD1306_drive/Src/fonts.o ./Drivers/SSD1306_drive/Src/fonts.su ./Drivers/SSD1306_drive/Src/ssd1306.cyclo ./Drivers/SSD1306_drive/Src/ssd1306.d ./Drivers/SSD1306_drive/Src/ssd1306.o ./Drivers/SSD1306_drive/Src/ssd1306.su

.PHONY: clean-Drivers-2f-SSD1306_drive-2f-Src

