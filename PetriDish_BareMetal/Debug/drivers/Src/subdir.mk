################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/ADXL343_Accel.c \
../drivers/Src/CAV25M01.c \
../drivers/Src/LM75B_TempSensor.c \
../drivers/Src/stm32f101xx.c \
../drivers/Src/stm32f101xx_afio_driver.c \
../drivers/Src/stm32f101xx_gpio_driver.c \
../drivers/Src/stm32f101xx_i2c_driver.c \
../drivers/Src/stm32f101xx_rcc_driver.c \
../drivers/Src/stm32f101xx_spi_driver.c \
../drivers/Src/stm32f101xx_tim_driver.c \
../drivers/Src/stm32f101xx_usart_driver.c 

OBJS += \
./drivers/Src/ADXL343_Accel.o \
./drivers/Src/CAV25M01.o \
./drivers/Src/LM75B_TempSensor.o \
./drivers/Src/stm32f101xx.o \
./drivers/Src/stm32f101xx_afio_driver.o \
./drivers/Src/stm32f101xx_gpio_driver.o \
./drivers/Src/stm32f101xx_i2c_driver.o \
./drivers/Src/stm32f101xx_rcc_driver.o \
./drivers/Src/stm32f101xx_spi_driver.o \
./drivers/Src/stm32f101xx_tim_driver.o \
./drivers/Src/stm32f101xx_usart_driver.o 

C_DEPS += \
./drivers/Src/ADXL343_Accel.d \
./drivers/Src/CAV25M01.d \
./drivers/Src/LM75B_TempSensor.d \
./drivers/Src/stm32f101xx.d \
./drivers/Src/stm32f101xx_afio_driver.d \
./drivers/Src/stm32f101xx_gpio_driver.d \
./drivers/Src/stm32f101xx_i2c_driver.d \
./drivers/Src/stm32f101xx_rcc_driver.d \
./drivers/Src/stm32f101xx_spi_driver.d \
./drivers/Src/stm32f101xx_tim_driver.d \
./drivers/Src/stm32f101xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -I"C:/Users/arron/Documents/STM32 MCU1-Course/PetriDish_BareMetal/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/ADXL343_Accel.cyclo ./drivers/Src/ADXL343_Accel.d ./drivers/Src/ADXL343_Accel.o ./drivers/Src/ADXL343_Accel.su ./drivers/Src/CAV25M01.cyclo ./drivers/Src/CAV25M01.d ./drivers/Src/CAV25M01.o ./drivers/Src/CAV25M01.su ./drivers/Src/LM75B_TempSensor.cyclo ./drivers/Src/LM75B_TempSensor.d ./drivers/Src/LM75B_TempSensor.o ./drivers/Src/LM75B_TempSensor.su ./drivers/Src/stm32f101xx.cyclo ./drivers/Src/stm32f101xx.d ./drivers/Src/stm32f101xx.o ./drivers/Src/stm32f101xx.su ./drivers/Src/stm32f101xx_afio_driver.cyclo ./drivers/Src/stm32f101xx_afio_driver.d ./drivers/Src/stm32f101xx_afio_driver.o ./drivers/Src/stm32f101xx_afio_driver.su ./drivers/Src/stm32f101xx_gpio_driver.cyclo ./drivers/Src/stm32f101xx_gpio_driver.d ./drivers/Src/stm32f101xx_gpio_driver.o ./drivers/Src/stm32f101xx_gpio_driver.su ./drivers/Src/stm32f101xx_i2c_driver.cyclo ./drivers/Src/stm32f101xx_i2c_driver.d ./drivers/Src/stm32f101xx_i2c_driver.o ./drivers/Src/stm32f101xx_i2c_driver.su ./drivers/Src/stm32f101xx_rcc_driver.cyclo ./drivers/Src/stm32f101xx_rcc_driver.d ./drivers/Src/stm32f101xx_rcc_driver.o ./drivers/Src/stm32f101xx_rcc_driver.su ./drivers/Src/stm32f101xx_spi_driver.cyclo ./drivers/Src/stm32f101xx_spi_driver.d ./drivers/Src/stm32f101xx_spi_driver.o ./drivers/Src/stm32f101xx_spi_driver.su ./drivers/Src/stm32f101xx_tim_driver.cyclo ./drivers/Src/stm32f101xx_tim_driver.d ./drivers/Src/stm32f101xx_tim_driver.o ./drivers/Src/stm32f101xx_tim_driver.su ./drivers/Src/stm32f101xx_usart_driver.cyclo ./drivers/Src/stm32f101xx_usart_driver.d ./drivers/Src/stm32f101xx_usart_driver.o ./drivers/Src/stm32f101xx_usart_driver.su

.PHONY: clean-drivers-2f-Src

